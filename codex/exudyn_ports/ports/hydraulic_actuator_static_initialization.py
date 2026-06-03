import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN Examples/HydraulicActuatorStaticInitialization.py:
# a one-arm HydraulicActuatorSimple scene where a distance constraint is used
# for static equilibrium, then the equivalent pressure preload initializes the
# dynamic hydraulic actuator.  PyChrono does not expose the EXUDYN hydraulic
# element, so this port keeps the same static preload idea in a force-controlled
# TSDA and renders the connector as a hydraulic barrel/rod.

LENGTH = 1.0
WIDTH = 0.1
THICKNESS = 0.01
MASS_ARM = 120.0
GRAVITY = 9.81
STEP = 1.0e-3
END_TIME = 1.0

GROUND_MOUNT = chrono.ChVector3d(0.0, -0.25 * LENGTH, 0.0)
ARM_MOUNT_LOCAL = chrono.ChVector3d(-0.25 * LENGTH, 0.0, 0.0)
HINGE_LOCAL = chrono.ChVector3d(-0.5 * LENGTH, 0.0, 0.0)
ACTUATOR_L0 = math.sqrt(2.0 * (0.25 * LENGTH) ** 2)
PISTON_AREA = 0.01
INITIAL_PRESSURE = 2.0e6
SERVO_STIFFNESS = 3.2e6
SERVO_DAMPING = 2.0e5
SYSTEM_PRESSURE = 200.0e5
HYDRAULIC_VISUAL_OFFSET = chrono.ChVector3d(0.0, 0.0, 0.065)


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def static_constraint_force():
    gravity_torque = MASS_ARM * GRAVITY * 0.5 * LENGTH
    actuator_moment_arm = 0.25 * LENGTH / math.sqrt(2.0)
    return gravity_torque / actuator_moment_arm


STATIC_FORCE = static_constraint_force()
STATIC_PRESSURE_0 = INITIAL_PRESSURE + STATIC_FORCE / PISTON_AREA
STATIC_PRESSURE_1 = INITIAL_PRESSURE


def actuator_target_length(time):
    source_command = max(0.5, min(1.5, 1.0 - math.cos(time * math.pi)))
    return (source_command - 0.5) * 0.1 + ACTUATOR_L0


class StaticInitializedHydraulicForce(chrono.ForceFunctor):
    def __init__(self):
        super().__init__()
        self.force = STATIC_FORCE
        self.length = ACTUATOR_L0
        self.velocity = 0.0
        self.target = ACTUATOR_L0
        self.static_force = STATIC_FORCE
        self.pressures = (STATIC_PRESSURE_0, STATIC_PRESSURE_1)
        self.valves = (0.0, 0.0)

    def evaluate(self, time, rest_length, length, vel, link):
        self.length = length
        self.velocity = vel
        self.target = actuator_target_length(time)
        length_error = self.target - length
        self.valves = (2.0 * length_error, -2.0 * length_error)
        self.force = STATIC_FORCE + SERVO_STIFFNESS * length_error - SERVO_DAMPING * vel
        p0 = INITIAL_PRESSURE + max(0.0, self.force) / PISTON_AREA
        p1 = INITIAL_PRESSURE + max(0.0, -self.force) / PISTON_AREA
        self.pressures = (min(SYSTEM_PRESSURE, p0), min(SYSTEM_PRESSURE, p1))
        return self.force


class MutableSegment:
    def __init__(self, system, name, tint, thickness=4):
        self.body = chrono.ChBody()
        self.body.SetName(name)
        self.body.SetFixed(True)
        self.body.EnableCollision(False)
        self.shape = chrono.ChVisualShapeSegment()
        self.shape.SetMutable(True)
        self.shape.SetColor(tint)
        self.shape.SetThickness(thickness)
        self.body.AddVisualShape(self.shape)
        system.AddBody(self.body)

    def update(self, point_a, point_b):
        self.shape.SetLineGeometry(chrono.ChLineSegment(point_a, point_b))
        self.body.UpdateVisualModel()


class HydraulicVisual:
    def __init__(self, system):
        self.cylinder = MutableSegment(system, "static-initialized hydraulic cylinder barrel", color(0.08, 0.22, 0.82), 12)
        self.rod = MutableSegment(system, "static-initialized hydraulic piston rod", color(0.78, 0.78, 0.80), 7)
        self.stroke = MutableSegment(system, "static-initialized hydraulic target stroke guide", color(0.92, 0.56, 0.08), 3)
        self.static_reference = MutableSegment(system, "inactive static distance-constraint reference", color(0.48, 0.48, 0.48), 2)
        self.ground_mount = make_sphere(system, "static hydraulic ground mount", 0.045, color(0.16, 0.16, 0.18), True)
        self.rod_mount = make_sphere(system, "static hydraulic rod mount", 0.040, color(0.16, 0.16, 0.18), True)

    def update(self, point_ground, point_rod, target_length):
        point_ground = point_ground + HYDRAULIC_VISUAL_OFFSET
        point_rod = point_rod + HYDRAULIC_VISUAL_OFFSET
        axis = point_rod - point_ground
        length = axis.Length()
        if length < 1e-12:
            split = point_ground
            target = point_ground
        else:
            direction = chrono.ChVector3d(axis.x, axis.y, axis.z)
            direction.Normalize()
            split = point_ground + direction * min(length, max(0.52 * length, ACTUATOR_L0 * 0.62))
            target = point_ground + direction * target_length
        self.cylinder.update(point_ground, split)
        self.rod.update(split, point_rod)
        self.stroke.update(point_rod, target)
        self.static_reference.update(
            GROUND_MOUNT + chrono.ChVector3d(0.0, 0.0, -0.035),
            chrono.ChVector3d(0.25 * LENGTH, 0.0, -0.035),
        )
        self.ground_mount.SetPos(point_ground)
        self.rod_mount.SetPos(point_rod)


def make_sphere(system, name, radius, tint, fixed=False):
    body = chrono.ChBodyEasySphere(radius, 1000, True, False)
    body.SetName(name)
    body.SetFixed(fixed)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def make_ground(system):
    ground = chrono.ChBody()
    ground.SetName("hydraulic static-initialization ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    floor = chrono.ChVisualShapeBox(1.45, 0.95, 0.018)
    floor.SetColor(color(0.70, 0.71, 0.69))
    floor.SetOpacity(0.32)
    ground.AddVisualShape(floor, chrono.ChFramed(chrono.ChVector3d(0.35, 0.0, -0.20)))

    hinge = chrono.ChVisualShapeCylinder(0.065, 0.18)
    hinge.SetColor(color(0.78, 0.78, 0.80))
    ground.AddVisualShape(hinge, chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT))

    mount = chrono.ChVisualShapeCylinder(0.040, 0.16)
    mount.SetColor(color(0.18, 0.18, 0.19))
    ground.AddVisualShape(mount, chrono.ChFramed(GROUND_MOUNT, chrono.QUNIT))

    system.AddBody(ground)
    return ground


def make_arm(system):
    arm = chrono.ChBodyEasyBox(LENGTH, WIDTH, THICKNESS, 1000, True, False)
    arm.SetName("hydraulic static-initialization arm")
    arm.EnableCollision(False)
    arm.SetMass(MASS_ARM)
    arm.SetInertiaXX(chrono.ChVector3d(0.10, 10.0, 10.1))
    arm.SetPos(chrono.ChVector3d(0.5 * LENGTH, 0.0, 0.0))
    arm.GetVisualShape(0).SetColor(color(0.10, 0.42, 0.86))

    for local, radius, tint in (
        (HINGE_LOCAL, 0.045, color(0.78, 0.78, 0.80)),
        (chrono.ChVector3d(0.0, 0.0, 0.0), 0.033, color(0.92, 0.56, 0.08)),
        (ARM_MOUNT_LOCAL, 0.036, color(0.04, 0.04, 0.045)),
    ):
        marker = chrono.ChVisualShapeSphere(radius)
        marker.SetColor(tint)
        arm.AddVisualShape(marker, chrono.ChFramed(local))

    hinge_pin = chrono.ChVisualShapeCylinder(0.055, 0.14)
    hinge_pin.SetColor(color(0.78, 0.78, 0.80))
    arm.AddVisualShape(hinge_pin, chrono.ChFramed(HINGE_LOCAL, chrono.QUNIT))
    system.AddBody(arm)
    return arm


def add_hinge(system, arm, ground):
    joint = chrono.ChLinkLockRevolute()
    joint.SetName("source RevoluteJoint2D analogue")
    joint.Initialize(arm, ground, chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT))
    system.AddLink(joint)
    return joint


def add_actuator(system, arm, ground):
    actuator = chrono.ChLinkTSDA()
    actuator.SetName("static-initialized HydraulicActuatorSimple force analogue")
    actuator.Initialize(arm, ground, True, ARM_MOUNT_LOCAL, GROUND_MOUNT)
    actuator.SetRestLength(ACTUATOR_L0)
    actuator._hydraulic_force = StaticInitializedHydraulicForce()
    actuator.RegisterForceFunctor(actuator._hydraulic_force)
    system.AddLink(actuator)
    return actuator


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -GRAVITY, 0.0))

    ground = make_ground(system)
    arm = make_arm(system)
    hinge = add_hinge(system, arm, ground)
    actuator = add_actuator(system, arm, ground)
    visual = HydraulicVisual(system)

    items = {"ground": ground, "arm": arm, "hinge": hinge, "actuator": actuator, "visual": visual}
    system._hydraulic_static_initialization_items = items
    update_visuals(system)
    return system, items


def actuator_points(items):
    return items["actuator"].GetPoint2Abs(), items["actuator"].GetPoint1Abs()


def update_visuals(system):
    items = system._hydraulic_static_initialization_items
    point_ground, point_rod = actuator_points(items)
    items["visual"].update(point_ground, point_rod, actuator_target_length(system.GetChTime()))


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, items


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: HydraulicActuatorStaticInitialization.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.85, -1.25, 0.95), chrono.ChVector3d(0.32, -0.02, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, items)
            next_log += 0.2


def print_state(system, items):
    arm = items["arm"]
    actuator = items["actuator"]
    functor = actuator._hydraulic_force
    angle = arm.GetRot().GetCardanAnglesXYZ().z
    solution_norm = math.sqrt(angle * angle + actuator.GetLength() * actuator.GetLength()) + 1e-6 * math.sqrt(
        functor.pressures[0] * functor.pressures[0] + functor.pressures[1] * functor.pressures[1]
    )
    print(
        f"t={system.GetChTime():6.3f}  "
        f"angle_z={angle:+.6f}  distance={actuator.GetLength():.6f}  "
        f"target={actuator_target_length(system.GetChTime()):.6f}  "
        f"force={actuator.GetForce():+.3f}  static_force={functor.static_force:+.3f}  "
        f"pressures=({functor.pressures[0]:.3f},{functor.pressures[1]:.3f})  "
        f"valves=({functor.valves[0]:+.4f},{functor.valves[1]:+.4f})  "
        f"solution_norm={solution_norm:.9f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: HydraulicActuatorStaticInitialization.py -> PyChrono static-preloaded hydraulic analogue")
    print(f"initial static constraint force={STATIC_FORCE:.6f}  preload pressures=({STATIC_PRESSURE_0:.6f},{STATIC_PRESSURE_1:.6f})")
    if args.no_vis:
        system, items = simulate(args.duration, args.step)
        print_state(system, items)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

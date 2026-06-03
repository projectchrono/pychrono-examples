import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN Examples/HydraulicActuator2Arms.py:
# a two-link planar mechanism driven by two HydraulicActuatorSimple connectors.
# PyChrono does not expose the EXUDYN hydraulic element, so each actuator is
# represented by a force-controlled TSDA with the source valve/length target
# law.  The connector visuals are hydraulic barrels and piston rods, not coils.

LENGTH = 1.0
WIDTH = 0.1
THICKNESS = 0.01
MASS_ARM = 120.0
GRAVITY = 9.81
STEP = 1.0e-3
END_TIME = 1.2

HINGE_LOCAL = chrono.ChVector3d(-0.5 * LENGTH, 0.0, 0.0)
ARM_END_LOCAL = chrono.ChVector3d(0.5 * LENGTH, 0.0, 0.0)
ACT1_GROUND_MOUNT = chrono.ChVector3d(0.0, -0.25 * LENGTH - 0.5 * WIDTH, 0.0)
ACT1_ARM_MOUNT = chrono.ChVector3d(-0.25 * LENGTH, -0.5 * WIDTH, 0.0)
ACT2_ARM1_MOUNT = chrono.ChVector3d(0.25 * LENGTH, -0.5 * WIDTH, 0.0)
ACT2_ARM2_MOUNT = chrono.ChVector3d(-0.25 * LENGTH, -0.5 * WIDTH, 0.0)

ACT1_L0 = math.sqrt(2.0 * (0.25 * LENGTH) ** 2)
ACT2_L0 = math.sqrt(2.0 * (0.25 * LENGTH - 0.5 * WIDTH) ** 2)
PISTON_AREA = 0.01
INITIAL_PRESSURE = 2.0e6
SERVO_STIFFNESS = 3.2e6
SERVO_DAMPING = 2.0e5
HYDRAULIC_VISUAL_OFFSET = chrono.ChVector3d(0.0, 0.0, 0.105)


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def target_command(time, frequency_scale):
    return max(0.5, min(1.5, 1.0 - math.cos(frequency_scale * math.pi * time)))


def actuator1_target_length(time):
    return (target_command(time, 1.0) - 0.5) * 0.15 + ACT1_L0


def actuator2_target_length(time):
    return (target_command(time, 2.0) - 0.5) * 0.20 + ACT2_L0


class HydraulicServoForce(chrono.ForceFunctor):
    def __init__(self, target_function):
        super().__init__()
        self.target_function = target_function
        self.force = 0.0
        self.length = 0.0
        self.velocity = 0.0
        self.target = target_function(0.0)
        self.pressures = (INITIAL_PRESSURE, INITIAL_PRESSURE)
        self.valves = (0.0, 0.0)

    def evaluate(self, time, rest_length, length, vel, link):
        self.length = length
        self.velocity = vel
        self.target = self.target_function(time)
        length_error = self.target - length
        self.valves = (2.0 * length_error, -2.0 * length_error)
        self.force = SERVO_STIFFNESS * length_error - SERVO_DAMPING * vel
        pressure_delta = max(-1.6e7, min(1.6e7, self.force / max(PISTON_AREA, 1e-9)))
        self.pressures = (INITIAL_PRESSURE + 0.5 * pressure_delta, INITIAL_PRESSURE - 0.5 * pressure_delta)
        return self.force


class MutableSegment:
    def __init__(self, system, name, tint, thickness):
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
    def __init__(self, system, name, barrel_radius, rod_radius, tint):
        self.barrel_radius = barrel_radius
        self.rod_radius = rod_radius
        self.cylinder = MutableSegment(system, f"{name} cylinder barrel", tint, max(4, int(barrel_radius * 220)))
        self.rod = MutableSegment(system, f"{name} piston rod", color(0.78, 0.78, 0.80), max(3, int(rod_radius * 220)))
        self.stroke = MutableSegment(system, f"{name} target stroke guide", color(0.92, 0.56, 0.08), 3)
        self.base_mount = make_sphere(system, f"{name} base mount", max(0.018, barrel_radius * 0.75), color(0.16, 0.16, 0.18), True)
        self.rod_mount = make_sphere(system, f"{name} rod mount", max(0.015, rod_radius * 0.95), color(0.16, 0.16, 0.18), True)

    def update(self, point_base, point_rod, target_length):
        point_base = point_base + HYDRAULIC_VISUAL_OFFSET
        point_rod = point_rod + HYDRAULIC_VISUAL_OFFSET
        axis = point_rod - point_base
        length = axis.Length()
        if length < 1e-12:
            split = point_base
            target = point_base
        else:
            direction = chrono.ChVector3d(axis.x, axis.y, axis.z)
            direction.Normalize()
            split = point_base + direction * min(length, max(0.56 * length, target_length * 0.50))
            target = point_base + direction * target_length
        self.cylinder.update(point_base, split)
        self.rod.update(split, point_rod)
        self.stroke.update(point_rod, target)
        self.base_mount.SetPos(point_base)
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
    ground.SetName("hydraulic two-arm ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    floor = chrono.ChVisualShapeBox(2.0, 1.55, 0.018)
    floor.SetColor(color(0.70, 0.71, 0.69))
    floor.SetOpacity(0.30)
    ground.AddVisualShape(floor, chrono.ChFramed(chrono.ChVector3d(0.72, -0.34, -0.22)))

    hinge = chrono.ChVisualShapeCylinder(0.065, 0.18)
    hinge.SetColor(color(0.78, 0.78, 0.80))
    ground.AddVisualShape(hinge, chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT))

    mount = chrono.ChVisualShapeCylinder(0.040, 0.16)
    mount.SetColor(color(0.18, 0.18, 0.19))
    ground.AddVisualShape(mount, chrono.ChFramed(ACT1_GROUND_MOUNT, chrono.QUNIT))

    system.AddBody(ground)
    return ground


def add_local_cylinder(body, local, radius, height, tint):
    shape = chrono.ChVisualShapeCylinder(radius, height)
    shape.SetColor(tint)
    body.AddVisualShape(shape, chrono.ChFramed(local, chrono.QUNIT))


def add_local_sphere(body, local, radius, tint):
    shape = chrono.ChVisualShapeSphere(radius)
    shape.SetColor(tint)
    body.AddVisualShape(shape, chrono.ChFramed(local))


def make_arm(system, name, position, rotation, include_outer_actuator_mount):
    arm = chrono.ChBodyEasyBox(LENGTH, 0.75 * WIDTH, 1.4 * WIDTH, 1000, True, False)
    arm.SetName(name)
    arm.EnableCollision(False)
    arm.SetMass(MASS_ARM)
    arm.SetInertiaXX(chrono.ChVector3d(0.12, 10.0, 10.1))
    arm.SetPos(position)
    arm.SetRot(rotation)
    arm.GetVisualShape(0).SetColor(color(0.10, 0.42, 0.86))

    blue = color(0.10, 0.42, 0.86)
    grey = color(0.78, 0.78, 0.80)
    black = color(0.04, 0.04, 0.045)
    orange = color(0.92, 0.56, 0.08)

    add_local_cylinder(arm, HINGE_LOCAL, 0.055, 0.15, blue)
    add_local_cylinder(arm, HINGE_LOCAL, 0.025, 0.16, grey)
    add_local_sphere(arm, chrono.ChVector3d(0.0, 0.0, 0.0), 0.030, orange)
    add_local_sphere(arm, HINGE_LOCAL, 0.032, grey)
    add_local_sphere(arm, ARM_END_LOCAL, 0.030, grey)
    add_local_sphere(arm, ACT1_ARM_MOUNT, 0.026, black)
    if include_outer_actuator_mount:
        add_local_sphere(arm, ACT2_ARM1_MOUNT, 0.026, black)

    system.AddBody(arm)
    return arm


def add_revolute(system, body_a, body_b, frame, name):
    joint = chrono.ChLinkLockRevolute()
    joint.SetName(name)
    joint.Initialize(body_a, body_b, frame)
    system.AddLink(joint)
    return joint


def add_actuator(system, name, body_rod, body_base, rod_local, base_local, rest_length, target_function):
    actuator = chrono.ChLinkTSDA()
    actuator.SetName(name)
    actuator.Initialize(body_rod, body_base, True, rod_local, base_local)
    actuator.SetRestLength(rest_length)
    actuator._hydraulic_force = HydraulicServoForce(target_function)
    actuator.RegisterForceFunctor(actuator._hydraulic_force)
    system.AddLink(actuator)
    return actuator


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -GRAVITY, 0))

    ground = make_ground(system)
    arm1 = make_arm(
        system,
        "hydraulic first arm",
        chrono.ChVector3d(0.5 * LENGTH, 0.0, 0.0),
        chrono.QUNIT,
        include_outer_actuator_mount=True,
    )
    arm2 = make_arm(
        system,
        "hydraulic second arm",
        chrono.ChVector3d(1.0 * LENGTH, -0.5 * LENGTH, 0.0),
        chrono.QuatFromAngleZ(-0.5 * math.pi),
        include_outer_actuator_mount=False,
    )

    ground_joint = add_revolute(
        system,
        arm1,
        ground,
        chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT),
        "source ground RevoluteJoint2D analogue",
    )
    elbow_joint = add_revolute(
        system,
        arm2,
        arm1,
        chrono.ChFramed(chrono.ChVector3d(1.0 * LENGTH, 0.0, 0.0), chrono.QUNIT),
        "source arm-to-arm RevoluteJoint2D analogue",
    )

    actuator1 = add_actuator(
        system,
        "HydraulicActuatorSimple arm1 force analogue",
        arm1,
        ground,
        ACT1_ARM_MOUNT,
        ACT1_GROUND_MOUNT,
        ACT1_L0,
        actuator1_target_length,
    )
    actuator2 = add_actuator(
        system,
        "HydraulicActuatorSimple arm2 force analogue",
        arm2,
        arm1,
        ACT2_ARM2_MOUNT,
        ACT2_ARM1_MOUNT,
        ACT2_L0,
        actuator2_target_length,
    )

    visual1 = HydraulicVisual(system, "arm1 hydraulic", 0.55 * WIDTH, 0.30 * WIDTH, color(0.92, 0.45, 0.10))
    visual2 = HydraulicVisual(system, "arm2 hydraulic", 0.45 * WIDTH, 0.20 * WIDTH, color(0.96, 0.55, 0.12))

    items = {
        "ground": ground,
        "arm1": arm1,
        "arm2": arm2,
        "ground_joint": ground_joint,
        "elbow_joint": elbow_joint,
        "actuator1": actuator1,
        "actuator2": actuator2,
        "visual1": visual1,
        "visual2": visual2,
    }
    system._hydraulic_actuator_2arms_items = items
    update_visuals(system)
    return system, items


def actuator_points(actuator):
    return actuator.GetPoint2Abs(), actuator.GetPoint1Abs()


def update_visuals(system):
    items = system._hydraulic_actuator_2arms_items
    base1, rod1 = actuator_points(items["actuator1"])
    base2, rod2 = actuator_points(items["actuator2"])
    time = system.GetChTime()
    items["visual1"].update(base1, rod1, actuator1_target_length(time))
    items["visual2"].update(base2, rod2, actuator2_target_length(time))


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
    vis.SetWindowSize(1120, 760)
    vis.SetWindowTitle("EXUDYN port: HydraulicActuator2Arms.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.35, -1.95, 1.20), chrono.ChVector3d(0.72, -0.35, 0.0))
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
            next_log += 0.25


def print_state(system, items):
    arm1 = items["arm1"]
    arm2 = items["arm2"]
    actuator1 = items["actuator1"]
    actuator2 = items["actuator2"]
    force1 = actuator1._hydraulic_force
    force2 = actuator2._hydraulic_force
    print(
        f"t={system.GetChTime():6.3f}  "
        f"angles=({arm1.GetRot().GetCardanAnglesXYZ().z:+.6f},{arm2.GetRot().GetCardanAnglesXYZ().z:+.6f})  "
        f"distances=({actuator1.GetLength():.6f},{actuator2.GetLength():.6f})  "
        f"targets=({actuator1_target_length(system.GetChTime()):.6f},{actuator2_target_length(system.GetChTime()):.6f})  "
        f"forces=({actuator1.GetForce():+.3f},{actuator2.GetForce():+.3f})  "
        f"pressures1=({force1.pressures[0]:.3f},{force1.pressures[1]:.3f})  "
        f"pressures2=({force2.pressures[0]:.3f},{force2.pressures[1]:.3f})  "
        f"valves=({force1.valves[0]:+.4f},{force1.valves[1]:+.4f};{force2.valves[0]:+.4f},{force2.valves[1]:+.4f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: HydraulicActuator2Arms.py -> PyChrono two-arm hydraulic analogue")
    if args.no_vis:
        system, items = simulate(args.duration, args.step)
        print_state(system, items)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces EXUDYN Examples/HydraulicsUserFunction.py:
# a one-arm mechanism actuated by an ObjectConnectorSpringDamper whose force
# is computed from two user-function hydraulic pressure states.  The connector
# is rendered as a real Chrono spring visual, matching EXUDYN's VSpringDamper.

LENGTH = 1.0
WIDTH = 0.1
THICKNESS = 0.1
MASS_ARM = 120.0
GRAVITY = 9.81
STEP = 1.0e-3
END_TIME = 0.4
PRESSURE_INTERNAL_STEP = 1.0e-5

GROUND_MOUNT = chrono.ChVector3d(0.0, -0.25 * LENGTH, 0.0)
ARM_MOUNT_LOCAL = chrono.ChVector3d(-0.25 * LENGTH, 0.0, 0.0)
HINGE_LOCAL = chrono.ChVector3d(-0.5 * LENGTH, 0.0, 0.0)
ACTUATOR_L0 = math.sqrt(2.0 * (0.25 * LENGTH) ** 2)

PISTON_AREA_0 = 0.01
PISTON_AREA_1 = 0.01
HOSE_VOLUME_0 = 1.0
HOSE_VOLUME_1 = 1.0
OIL_BULK_MODULUS = 1.0e11
NOMINAL_FLOW = 2.0e-5
SYSTEM_PRESSURE = 200.0e5
TANK_PRESSURE = 0.0
INITIAL_PRESSURE = 2.0e6
DAMPING_HA = 2.0e5


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def signed_sqrt(value):
    if value == 0.0:
        return 0.0
    return math.copysign(math.sqrt(abs(value)), value)


def actuator_target_length(time):
    source_command = max(0.5, min(1.5, 1.0 - math.cos(time * math.pi)))
    return (source_command - 0.5) * 0.1 + ACTUATOR_L0


class HydraulicUserFunctionForce(chrono.ForceFunctor):
    def __init__(self):
        super().__init__()
        self.pressures = [INITIAL_PRESSURE, INITIAL_PRESSURE]
        self.pressure_rates = [0.0, 0.0]
        self.valves = [0.0, 0.0]
        self.last_time = None
        self.length = ACTUATOR_L0
        self.velocity = 0.0
        self.target = ACTUATOR_L0
        self.force = 0.0

    def evaluate(self, time, rest_length, length, vel, link):
        self.length = length
        self.velocity = vel
        self.target = actuator_target_length(time)
        self.valves[0] = 2.0 * (self.target - length)
        self.valves[1] = -self.valves[0]

        if self.last_time is None:
            self.last_time = time
        elif time > self.last_time:
            remaining = time - self.last_time
            while remaining > 1e-12:
                dt = min(remaining, PRESSURE_INTERNAL_STEP)
                self._advance_pressure(dt, vel)
                remaining -= dt
            self.last_time = time

        self.force = -self.pressures[0] * PISTON_AREA_0 + self.pressures[1] * PISTON_AREA_1 + vel * DAMPING_HA
        # EXUDYN's user function reports positive tension; Chrono's TSDA
        # functor applies the opposite sign for the same endpoint order.
        return -self.force

    def _advance_pressure(self, dt, actuator_velocity):
        p0, p1 = self.pressures
        av0, av1 = self.valves

        if av0 >= 0.0:
            flow0 = av0 * NOMINAL_FLOW * signed_sqrt(SYSTEM_PRESSURE - p0)
        else:
            flow0 = av0 * NOMINAL_FLOW * signed_sqrt(p0 - TANK_PRESSURE)

        if av1 >= 0.0:
            flow1 = av1 * NOMINAL_FLOW * signed_sqrt(SYSTEM_PRESSURE - p1)
        else:
            flow1 = av1 * NOMINAL_FLOW * signed_sqrt(p1 - TANK_PRESSURE)

        dp0 = OIL_BULK_MODULUS / HOSE_VOLUME_0 * (-PISTON_AREA_0 * actuator_velocity + flow0)
        dp1 = OIL_BULK_MODULUS / HOSE_VOLUME_1 * (PISTON_AREA_1 * actuator_velocity + flow1)
        self.pressure_rates = [dp0, dp1]

        self.pressures[0] = max(TANK_PRESSURE, min(SYSTEM_PRESSURE, p0 + dt * dp0))
        self.pressures[1] = max(TANK_PRESSURE, min(SYSTEM_PRESSURE, p1 + dt * dp1))


class MutableSegment:
    def __init__(self, system, name, tint, thickness=3):
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


class PressureGaugeVisual:
    def __init__(self, system):
        self.force_bar = MutableSegment(system, "hydraulic differential force gauge", color(0.90, 0.50, 0.05), 5)
        self.p0_bar = MutableSegment(system, "hydraulic p0 pressure gauge", color(0.80, 0.14, 0.12), 4)
        self.p1_bar = MutableSegment(system, "hydraulic p1 pressure gauge", color(0.12, 0.42, 0.85), 4)

    def update(self, functor):
        base = chrono.ChVector3d(0.82, 0.10, 0.09)
        p0_height = 0.18 * functor.pressures[0] / SYSTEM_PRESSURE
        p1_height = 0.18 * functor.pressures[1] / SYSTEM_PRESSURE
        force_width = max(-0.16, min(0.16, functor.force / 4.0e4))
        self.p0_bar.update(base, base + chrono.ChVector3d(0.0, p0_height, 0.0))
        self.p1_bar.update(base + chrono.ChVector3d(0.06, 0.0, 0.0), base + chrono.ChVector3d(0.06, p1_height, 0.0))
        self.force_bar.update(base + chrono.ChVector3d(0.13, 0.0, 0.0), base + chrono.ChVector3d(0.13 + force_width, 0.0, 0.0))


def make_ground(system):
    ground = chrono.ChBody()
    ground.SetName("hydraulics user-function ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    floor = chrono.ChVisualShapeBox(1.45, 0.95, 0.018)
    floor.SetColor(color(0.70, 0.71, 0.69))
    floor.SetOpacity(0.32)
    ground.AddVisualShape(floor, chrono.ChFramed(chrono.ChVector3d(0.35, 0.0, -0.20)))

    hinge = chrono.ChVisualShapeCylinder(0.060, 0.18)
    hinge.SetColor(color(0.78, 0.78, 0.80))
    ground.AddVisualShape(hinge, chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT))

    mount = chrono.ChVisualShapeCylinder(0.038, 0.16)
    mount.SetColor(color(0.18, 0.18, 0.19))
    ground.AddVisualShape(mount, chrono.ChFramed(GROUND_MOUNT, chrono.QUNIT))

    system.AddBody(ground)
    return ground


def make_arm(system):
    arm = chrono.ChBodyEasyBox(LENGTH, WIDTH, THICKNESS, 1000, True, False)
    arm.SetName("hydraulics user-function rigid arm")
    arm.EnableCollision(False)
    arm.SetMass(MASS_ARM)
    arm.SetInertiaXX(chrono.ChVector3d(0.10, 10.0, 10.1))
    arm.SetPos(chrono.ChVector3d(0.5 * LENGTH, 0.0, 0.0))
    arm.GetVisualShape(0).SetColor(color(0.10, 0.42, 0.86))

    for local, radius, tint in (
        (HINGE_LOCAL, 0.044, color(0.78, 0.78, 0.80)),
        (chrono.ChVector3d(0.0, 0.0, 0.0), 0.032, color(0.92, 0.56, 0.08)),
        (ARM_MOUNT_LOCAL, 0.034, color(0.04, 0.04, 0.045)),
    ):
        marker = chrono.ChVisualShapeSphere(radius)
        marker.SetColor(tint)
        arm.AddVisualShape(marker, chrono.ChFramed(local))

    hinge_pin = chrono.ChVisualShapeCylinder(0.055, 0.15)
    hinge_pin.SetColor(color(0.78, 0.78, 0.80))
    arm.AddVisualShape(hinge_pin, chrono.ChFramed(HINGE_LOCAL, chrono.QUNIT))
    system.AddBody(arm)
    return arm


def make_mount_sphere(system, name, position, radius, tint):
    body = chrono.ChBodyEasySphere(radius, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(position)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def add_hinge(system, arm, ground):
    joint = chrono.ChLinkLockRevolute()
    joint.SetName("source RevoluteJoint2D analogue")
    joint.Initialize(arm, ground, chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT))
    system.AddLink(joint)
    return joint


def add_user_function_connector(system, arm, ground):
    connector = chrono.ChLinkTSDA()
    connector.SetName("ObjectConnectorSpringDamper hydraulic user function analogue")
    connector.Initialize(arm, ground, True, ARM_MOUNT_LOCAL, GROUND_MOUNT)
    connector.SetRestLength(ACTUATOR_L0)
    connector.SetSpringCoefficient(0.0)
    connector.SetDampingCoefficient(0.0)
    connector._hydraulics_user_force = HydraulicUserFunctionForce()
    connector.RegisterForceFunctor(connector._hydraulics_user_force)
    system.AddLink(connector)

    spring_shape = chrono.ChVisualShapeSpring(0.05, 96, 12)
    spring_shape.SetColor(color(0.85, 0.18, 0.12))
    connector.AddVisualShape(spring_shape)
    attach_spring_visual(system, connector, 0.05, 96, 12, color(0.85, 0.18, 0.12))
    return connector


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -GRAVITY, 0.0))

    ground = make_ground(system)
    arm = make_arm(system)
    hinge = add_hinge(system, arm, ground)
    connector = add_user_function_connector(system, arm, ground)
    ground_mount = make_mount_sphere(system, "hydraulics user-function ground mount", GROUND_MOUNT, 0.038, color(0.16, 0.16, 0.18))
    gauge = PressureGaugeVisual(system)

    items = {
        "ground": ground,
        "arm": arm,
        "hinge": hinge,
        "connector": connector,
        "ground_mount": ground_mount,
        "gauge": gauge,
    }
    system._hydraulics_user_function_items = items
    update_visuals(system)
    return system, items


def update_visuals(system):
    update_system_visuals(system)
    items = system._hydraulics_user_function_items
    items["gauge"].update(items["connector"]._hydraulics_user_force)


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
    vis.SetWindowTitle("EXUDYN port: HydraulicsUserFunction.py")
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
            next_log += 0.1


def print_state(system, items):
    arm = items["arm"]
    connector = items["connector"]
    functor = connector._hydraulics_user_force
    angle = arm.GetRot().GetCardanAnglesXYZ().z
    print(
        f"t={system.GetChTime():6.3f}  "
        f"angle_z={angle:+.6f}  distance={connector.GetLength():.6f}  "
        f"target={actuator_target_length(system.GetChTime()):.6f}  "
        f"force={connector.GetForce():+.3f}  "
        f"pressures=({functor.pressures[0]:.3f},{functor.pressures[1]:.3f})  "
        f"pressure_rates=({functor.pressure_rates[0]:+.3e},{functor.pressure_rates[1]:+.3e})  "
        f"valves=({functor.valves[0]:+.4f},{functor.valves[1]:+.4f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: HydraulicsUserFunction.py -> PyChrono pressure user-function spring")
    if args.no_vis:
        system, items = simulate(args.duration, args.step)
        print_state(system, items)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

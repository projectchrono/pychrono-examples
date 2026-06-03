import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN Examples/universalJoint.py:
# two angled shafts joined by a universal joint, driven at constant angular
# velocity and compared through local angular speeds.

LENGTH = 1.0
RADIUS = 0.3
OMEGA = 10.0
ANGLE = math.radians(45.0)
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    half_length = 0.5 * LENGTH
    sin_a = math.sin(ANGLE)
    cos_a = math.cos(ANGLE)
    bend_rotation = chrono.QuatFromAngleX(ANGLE)

    ground = chrono.ChBody()
    ground.SetName("universal joint ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    support_a = chrono.ChVisualShapeCylinder(0.035, 0.16)
    support_a.SetColor(color(0.42, 0.42, 0.42))
    ground.AddVisualShape(support_a, chrono.ChFramed(chrono.ChVector3d(0, 0, -half_length), chrono.QUNIT))
    support_b = chrono.ChVisualShapeCylinder(0.035, 0.16)
    support_b.SetColor(color(0.42, 0.42, 0.42))
    ground.AddVisualShape(
        support_b,
        chrono.ChFramed(chrono.ChVector3d(0, -half_length * sin_a, half_length * cos_a), bend_rotation),
    )
    system.AddBody(ground)

    shaft_1 = chrono.ChBody()
    shaft_1.SetName("input shaft")
    shaft_1.EnableCollision(False)
    shaft_1.SetMass(9.0)
    shaft_1.SetInertiaXX(chrono.ChVector3d(100, 100, 100))
    shaft_1.SetPos(chrono.ChVector3d(0, 0, -half_length))
    shaft_1.SetAngVelParent(chrono.ChVector3d(0, 0, OMEGA))
    body_1 = chrono.ChVisualShapeCylinder(RADIUS / 8.0, 0.78 * LENGTH)
    body_1.SetColor(color(0.12, 0.42, 0.85))
    shaft_1.AddVisualShape(body_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 0.28 * LENGTH), chrono.QUNIT))
    yoke_1 = chrono.ChVisualShapeBox(2.0 * RADIUS, 0.25 * RADIUS, 0.25 * RADIUS)
    yoke_1.SetColor(color(0.12, 0.42, 0.85))
    shaft_1.AddVisualShape(yoke_1, chrono.ChFramed(chrono.ChVector3d(0, 0, half_length - RADIUS)))
    system.AddBody(shaft_1)

    shaft_2 = chrono.ChBody()
    shaft_2.SetName("output shaft")
    shaft_2.EnableCollision(False)
    shaft_2.SetMass(9.0)
    shaft_2.SetInertiaXX(chrono.ChVector3d(100, 100, 100))
    shaft_2.SetPos(chrono.ChVector3d(0, -half_length * sin_a, half_length * cos_a))
    shaft_2.SetRot(bend_rotation)
    shaft_2.SetAngVelParent(chrono.ChVector3d(0, 0, OMEGA))
    body_2 = chrono.ChVisualShapeCylinder(RADIUS / 8.0, 0.70 * LENGTH)
    body_2.SetColor(color(0.35, 0.85, 0.15))
    shaft_2.AddVisualShape(body_2, chrono.ChFramed(chrono.ChVector3d(0, 0, 0.15 * LENGTH), chrono.QUNIT))
    yoke_2 = chrono.ChVisualShapeBox(0.25 * RADIUS, 2.0 * RADIUS, 0.25 * RADIUS)
    yoke_2.SetColor(color(0.35, 0.85, 0.15))
    shaft_2.AddVisualShape(yoke_2, chrono.ChFramed(chrono.ChVector3d(0, 0, -half_length + 0.45 * RADIUS)))
    system.AddBody(shaft_2)

    cross = chrono.ChBody()
    cross.SetName("universal cross visual")
    cross.SetFixed(True)
    cross.EnableCollision(False)
    cross.SetPos(chrono.ChVector3d(0, 0, 0))
    block = chrono.ChVisualShapeBox(0.16 * RADIUS, 0.16 * RADIUS, 0.16 * RADIUS)
    block.SetColor(color(0.55, 0.55, 0.55))
    cross.AddVisualShape(block)
    axle_x = chrono.ChVisualShapeCylinder(0.025, 0.65 * RADIUS)
    axle_x.SetColor(color(0.95, 0.50, 0.15))
    cross.AddVisualShape(axle_x, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleY(0.5 * math.pi)))
    axle_y = chrono.ChVisualShapeCylinder(0.025, 0.65 * RADIUS)
    axle_y.SetColor(color(0.15, 0.35, 0.95))
    cross.AddVisualShape(axle_y, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(0.5 * math.pi)))
    system.AddBody(cross)

    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(ground, shaft_1, chrono.ChFramed(chrono.ChVector3d(0, 0, -half_length), chrono.QUNIT))
    motor.SetMotorFunction(chrono.ChFunctionConst(-OMEGA))
    system.AddLink(motor)

    output_support = chrono.ChLinkLockCylindrical()
    output_support.Initialize(
        ground,
        shaft_2,
        chrono.ChFramed(chrono.ChVector3d(0, -half_length * sin_a, half_length * cos_a), bend_rotation),
    )
    system.AddLink(output_support)

    universal = chrono.ChLinkUniversal()
    universal.Initialize(shaft_1, shaft_2, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), bend_rotation))
    system.AddLink(universal)

    return system, shaft_1, shaft_2, motor, universal


def simulate(duration, step):
    system, shaft_1, shaft_2, motor, universal = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, shaft_1, shaft_2, motor, universal


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, shaft_1, shaft_2, motor, universal = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: universalJoint.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.4, 1.0, -1.0), chrono.ChVector3d(0, -0.2, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, shaft_1, shaft_2)
            next_log += 0.25


def print_state(system, shaft_1, shaft_2):
    omega_1 = shaft_1.GetAngVelLocal()
    omega_2 = shaft_2.GetAngVelLocal()
    beta = ANGLE
    gamma = OMEGA * system.GetChTime()
    analytical = OMEGA * math.cos(beta) / (1.0 - math.cos(gamma + 0.5 * math.pi) ** 2 * math.sin(beta) ** 2)
    print(
        f"t={system.GetChTime():6.3f}  "
        f"omega1_z={omega_1.z:+.5f}  omega2_z={omega_2.z:+.5f}  "
        f"analytical={analytical:+.5f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2.0 * math.pi / OMEGA)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: universalJoint.py -> PyChrono universal joint")
    if args.no_vis:
        system, shaft_1, shaft_2, motor, universal = simulate(args.duration, args.step)
        print_state(system, shaft_1, shaft_2)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/rollingDiscTangentialForces.py:
# a large rolling wheel at the end of an articulated arm. EXUDYN uses an ideal
# rolling-disc joint and reads tangential constraint forces; this PyChrono port
# uses high-friction rigid contact and reports an equivalent momentum/force
# indicator from the wheel state.

MASS = 30.0
RADIUS = 1.8
ARM_LENGTH = 3.0
WIDTH = 0.10
INERTIA_RADIUS = 1.2
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_material():
    material = chrono.ChContactMaterialNSC()
    material.SetFriction(1.0)
    material.SetRollingFriction(0.0)
    material.SetSpinningFriction(0.0)
    material.SetRestitution(0.0)
    return material


def add_wheel_spokes(wheel):
    spoke_a = chrono.ChVisualShapeBox(WIDTH * 1.25, 1.85 * RADIUS, 0.025)
    spoke_a.SetColor(color(0.95, 0.95, 0.95))
    wheel.AddVisualShape(spoke_a)
    spoke_b = chrono.ChVisualShapeBox(WIDTH * 1.25, 0.025, 1.85 * RADIUS)
    spoke_b.SetColor(color(0.95, 0.95, 0.95))
    wheel.AddVisualShape(spoke_b)


def angular_rates():
    normal_force = 400.0
    gravity = 9.81
    j_radial = MASS * INERTIA_RADIUS * INERTIA_RADIUS
    omega_y = math.sqrt(RADIUS * (normal_force - MASS * gravity) / j_radial)
    omega_z = omega_y
    omega_x = -ARM_LENGTH / RADIUS * omega_z
    return omega_x, omega_z


def build_system():
    system = chrono.ChSystemNSC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    material = make_material()
    omega_x, omega_z = angular_rates()
    forward_speed = omega_z * ARM_LENGTH

    ground = chrono.ChBodyEasyBox(3.2 * ARM_LENGTH, 3.2 * ARM_LENGTH, 0.10, 1000, True, True, material)
    ground.SetName("rolling-disc tangential force ground")
    ground.SetFixed(True)
    ground.SetPos(chrono.ChVector3d(0.5 * ARM_LENGTH, 0, -0.05))
    ground.GetVisualShape(0).SetColor(color(0.55, 0.56, 0.50))
    ground.GetVisualShape(0).SetOpacity(0.45)
    system.AddBody(ground)

    support = chrono.ChBody()
    support.SetName("fixed spherical support")
    support.SetFixed(True)
    support.EnableCollision(False)
    pivot = chrono.ChVisualShapeSphere(0.09)
    pivot.SetColor(color(0.05, 0.05, 0.05))
    support.AddVisualShape(pivot, chrono.ChFramed(chrono.ChVector3d(0, 0, RADIUS)))
    system.AddBody(support)

    arm = chrono.ChBodyEasyBox(ARM_LENGTH, 0.10, 0.10, 1000, True, False)
    arm.SetName("rolling-disc support arm")
    arm.SetMass(1.0)
    arm.SetInertiaXX(chrono.ChVector3d(1.0, 1.0, 1.0))
    arm.SetPos(chrono.ChVector3d(0.5 * ARM_LENGTH, 0, RADIUS))
    arm.SetPosDt(chrono.ChVector3d(0, 0.5 * forward_speed, 0))
    arm.SetAngVelParent(chrono.ChVector3d(0, 0, omega_z))
    arm.GetVisualShape(0).SetColor(color(0.86, 0.18, 0.12))
    system.AddBody(arm)

    wheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_X, RADIUS, WIDTH, 1200, True, True, material)
    wheel.SetName("large rolling disc")
    wheel.SetMass(MASS)
    wheel.SetInertiaXX(
        chrono.ChVector3d(
            MASS * INERTIA_RADIUS * INERTIA_RADIUS,
            MASS * RADIUS * RADIUS,
            MASS * RADIUS * RADIUS,
        )
    )
    wheel.SetPos(chrono.ChVector3d(ARM_LENGTH, 0, RADIUS))
    wheel.SetPosDt(chrono.ChVector3d(0, forward_speed, 0))
    wheel.SetAngVelParent(chrono.ChVector3d(omega_x, 0, omega_z))
    wheel.GetVisualShape(0).SetColor(color(0.12, 0.35, 0.88))
    add_wheel_spokes(wheel)
    system.AddBody(wheel)

    pivot_joint = chrono.ChLinkLockSpherical()
    pivot_joint.Initialize(arm, support, chrono.ChFramed(chrono.ChVector3d(0, 0, RADIUS)))
    system.AddLink(pivot_joint)

    axle_joint = chrono.ChLinkLockRevolute()
    axle_joint.Initialize(wheel, arm, chrono.ChFramed(chrono.ChVector3d(ARM_LENGTH, 0, RADIUS), chrono.Q_ROTATE_Z_TO_X))
    system.AddLink(axle_joint)

    return system, arm, wheel, (pivot_joint, axle_joint)


def simulate(duration, step):
    system, arm, wheel, joints = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, arm, wheel, joints


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, arm, wheel, joints = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: rollingDiscTangentialForces.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(6.0, -7.0, 4.0), chrono.ChVector3d(1.8, 0.0, 1.2))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, arm, wheel)
            next_log += 0.05


def print_state(system, arm, wheel):
    pos = wheel.GetPos()
    omega = wheel.GetAngVelParent()
    velocity = wheel.GetPosDt()
    force_indicator = 1e-3 * MASS * velocity.Length() * omega.Length()
    print(
        f"t={system.GetChTime():6.3f}  "
        f"wheel=({pos.x:+.4f}, {pos.y:+.4f}, {pos.z:+.4f})  "
        f"vel=({velocity.x:+.3f}, {velocity.y:+.3f}, {velocity.z:+.3f})  "
        f"omega=({omega.x:+.3f}, {omega.y:+.3f}, {omega.z:+.3f})  "
        f"force_indicator={force_indicator:.8f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0.1)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: rollingDiscTangentialForces.py -> PyChrono articulated rolling disc contact")
    if args.no_vis:
        system, arm, wheel, joints = simulate(args.duration, args.step)
        print_state(system, arm, wheel)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

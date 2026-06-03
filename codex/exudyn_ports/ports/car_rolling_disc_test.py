import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/carRollingDiscTest.py:
# a rigid car chassis with four rolling-disc wheels. EXUDYN uses
# ObjectConnectorRollingDiscPenalty; this PyChrono version uses high-friction
# rigid wheel contact against a plane, revolute axle joints to the chassis, and
# body-fixed drive torques on wheels 0 and 1. The front wheel steering angles
# follow the source geometry.

R_WHEEL = 0.4
W_WHEEL = 0.1
RHO_WHEEL = 500.0

L_CAR = 3.0
W_CAR = 2.0
H_CAR = R_WHEEL
M_CAR = 500.0
R_STEERING = 5.0
DRIVE_TORQUE = -200.0
DRIVE_END = 4.0
STEP = 2.0e-3
END_TIME = 0.5


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_material(friction=0.8, restitution=0.02):
    material = chrono.ChContactMaterialSMC()
    material.SetFriction(friction)
    material.SetRestitution(restitution)
    material.SetKn(1.0e5)
    material.SetGn(1.0e3)
    material.SetKt(1.0e5)
    material.SetGt(1.0e3)
    return material


def axis_quat(axis):
    axis_v = chrono.ChVector3d(axis.x, axis.y, axis.z)
    axis_v.Normalize()
    z_axis = chrono.ChVector3d(0, 0, 1)
    dot = max(-1.0, min(1.0, z_axis.Dot(axis_v)))
    cross = z_axis.Cross(axis_v)
    if cross.Length() < 1e-12:
        return chrono.QUNIT if dot > 0 else chrono.QuatFromAngleX(math.pi)
    cross.Normalize()
    return chrono.QuatFromAngleAxis(math.acos(dot), cross)


def add_wheel_visuals(wheel, tint):
    wheel.GetVisualShape(0).SetColor(tint)
    hub_marker = chrono.ChVisualShapeSphere(0.105)
    hub_marker.SetColor(tint)
    wheel.AddVisualShape(hub_marker)

    for angle, stripe_color in (
        (0.0, color(0.98, 0.98, 0.95)),
        (0.5 * math.pi, color(0.04, 0.04, 0.05)),
    ):
        spoke = chrono.ChVisualShapeBox(1.12 * W_WHEEL, 1.45 * R_WHEEL, 0.025)
        spoke.SetColor(stripe_color)
        wheel.AddVisualShape(spoke, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(angle)))

    hub = chrono.ChVisualShapeCylinder(0.09, 1.18 * W_WHEEL)
    hub.SetColor(color(0.04, 0.04, 0.05))
    wheel.AddVisualShape(hub, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.Q_ROTATE_Z_TO_X))


def add_axle_marker(chassis, local_pos, steer_angle, index):
    axis = chrono.ChVisualShapeCylinder(0.020, 0.36)
    axis.SetColor(color(0.03, 0.03, 0.035))
    chassis.AddVisualShape(
        axis,
        chrono.ChFramed(local_pos, chrono.QuatFromAngleZ(steer_angle) * chrono.Q_ROTATE_Z_TO_X),
    )

    marker = chrono.ChVisualShapeSphere(0.045)
    marker.SetColor(color(0.96, 0.72, 0.08) if index >= 2 else color(0.12, 0.12, 0.12))
    chassis.AddVisualShape(marker, chrono.ChFramed(local_pos))


def wheel_offset(index):
    dx = -0.5 * W_CAR
    dy = -0.5 * L_CAR
    if index > 1:
        dy *= -1.0
    if index == 1 or index == 3:
        dx *= -1.0
    return chrono.ChVector3d(dx, dy, 0.0)


def steering_angle(index):
    if index == 2:
        return math.atan(L_CAR / R_STEERING)
    if index == 3:
        return math.atan(L_CAR / (W_CAR + R_STEERING))
    return 0.0


def make_ground(system, material):
    ground = chrono.ChBodyEasyBox(30.0, 30.0, 0.04, 1000.0, True, True, material)
    ground.SetName("car rolling disc ground")
    ground.SetFixed(True)
    ground.SetPos(chrono.ChVector3d(0, 0, -0.02))
    ground.GetVisualShape(0).SetColor(color(0.74, 0.74, 0.72))
    ground.GetVisualShape(0).SetOpacity(0.45)
    system.AddBody(ground)
    return ground


def make_chassis(system):
    density = M_CAR / (W_CAR * L_CAR * H_CAR)
    chassis = chrono.ChBodyEasyBox(W_CAR - 1.1 * W_WHEEL, L_CAR, H_CAR, density, True, False)
    chassis.SetName("rolling-disc car chassis")
    chassis.SetMass(M_CAR)
    chassis.SetInertiaXX(
        chrono.ChVector3d(
            M_CAR / 12.0 * (L_CAR**2 + H_CAR**2),
            M_CAR / 12.0 * ((W_CAR - 1.1 * W_WHEEL) ** 2 + H_CAR**2),
            M_CAR / 12.0 * ((W_CAR - 1.1 * W_WHEEL) ** 2 + L_CAR**2),
        )
    )
    chassis.SetPos(chrono.ChVector3d(0, 0, R_WHEEL))
    chassis.GetVisualShape(0).SetColor(color(0.92, 0.30, 0.24))
    chassis.GetVisualShape(0).SetOpacity(0.30)

    for i in range(4):
        add_axle_marker(chassis, wheel_offset(i), steering_angle(i), i)

    system.AddBody(chassis)
    return chassis


def make_wheel(system, material, index):
    offset = wheel_offset(index)
    steer = steering_angle(index)
    wheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_X, R_WHEEL, W_WHEEL, RHO_WHEEL, True, True, material)
    wheel.SetName(f"rolling-disc car wheel {index}")
    wheel.SetPos(chrono.ChVector3d(offset.x, offset.y, R_WHEEL))
    wheel.SetRot(chrono.QuatFromAngleZ(steer))
    wheel.SetPosDt(chrono.ChVector3d(0, 0, 0))
    wheel.SetAngVelParent(chrono.ChVector3d(0, 0, 0))
    add_wheel_visuals(
        wheel,
        [color(0.12, 0.32, 0.88), color(0.10, 0.58, 0.25), color(0.95, 0.56, 0.08), color(0.56, 0.24, 0.82)][index],
    )
    system.AddBody(wheel)
    return wheel


def add_wheel_joint(system, wheel, chassis, index):
    offset = wheel_offset(index)
    steer = steering_angle(index)
    axis = chrono.ChVector3d(math.cos(steer), math.sin(steer), 0.0)
    joint = chrono.ChLinkLockRevolute()
    joint.SetName(f"rolling-disc car axle joint {index}")
    joint.Initialize(
        wheel,
        chassis,
        chrono.ChFramed(chrono.ChVector3d(offset.x, offset.y, R_WHEEL), axis_quat(axis)),
    )
    system.AddLink(joint)
    return joint


def add_drive_torques(system, wheels):
    load_container = chrono.ChLoadContainer()
    system.Add(load_container)
    torques = []
    for index in (0, 1):
        torque = chrono.ChLoadBodyTorque(wheels[index], chrono.ChVector3d(DRIVE_TORQUE, 0, 0), True)
        load_container.Add(torque)
        torques.append(torque)
    return load_container, torques


def update_drive_torques(system, torques):
    value = DRIVE_TORQUE if system.GetChTime() < DRIVE_END else 0.0
    for torque in torques:
        torque.SetTorque(chrono.ChVector3d(value, 0, 0), True)


def build_system():
    system = chrono.ChSystemSMC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    material = make_material(0.8, 0.02)
    ground = make_ground(system, material)
    chassis = make_chassis(system)
    wheels = [make_wheel(system, material, i) for i in range(4)]
    joints = [add_wheel_joint(system, wheel, chassis, i) for i, wheel in enumerate(wheels)]
    load_container, torques = add_drive_torques(system, wheels)

    return system, chassis, wheels, joints, ground, load_container, torques


def simulate(duration, step):
    system, chassis, wheels, joints, ground, load_container, torques = build_system()
    while system.GetChTime() < duration:
        update_drive_torques(system, torques)
        system.DoStepDynamics(step)
    return system, chassis, wheels, joints, ground, load_container, torques


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, chassis, wheels, joints, ground, load_container, torques = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: carRollingDiscTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(4.6, -5.8, 5.6), chrono.ChVector3d(0.0, 0.0, 0.35))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        update_drive_torques(system, torques)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, chassis, wheels)
            next_log += 0.25


def print_state(system, chassis, wheels):
    pos = chassis.GetPos()
    vel = chassis.GetPosDt()
    omega0 = wheels[0].GetAngVelLocal()
    omega1 = wheels[1].GetAngVelLocal()
    print(
        f"t={system.GetChTime():6.3f}  "
        f"car=({pos.x:+.4f}, {pos.y:+.4f}, {pos.z:+.4f})  "
        f"vel=({vel.x:+.4f}, {vel.y:+.4f}, {vel.z:+.4f})  "
        f"drive_omega=({omega0.x:+.3f}, {omega1.x:+.3f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: carRollingDiscTest.py -> PyChrono four-wheel rolling-disc car")
    if args.no_vis:
        system, chassis, wheels, joints, ground, load_container, torques = simulate(args.duration, args.step)
        print_state(system, chassis, wheels)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

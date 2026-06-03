import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/mecanumWheelRollingDiscTest.py:
# a four-wheel mecanum chassis where each rolling disc has a +/-45 degree
# friction direction and a wheel-speed controller. Chrono core does not expose
# EXUDYN's rolling-disc friction-angle connector, so this port uses the same
# chassis, wheel layout, revolute axles, controlled wheel torques, high-friction
# wheel contact, and an additional controller force on the chassis to reproduce
# the intended sideways mecanum translation. The visible wheels include the
# source's diagonal roller cylinders rather than plain discs.

R_WHEEL = 0.4
W_WHEEL = 0.2
RHO_WHEEL = 500.0

L_CAR = 3.0
W_CAR = 3.0
H_CAR = R_WHEEL
M_CAR = 500.0

P_CONTROL = 500.0
CHASSIS_DRIVE_GAIN = 1250.0
STEP = 2.0e-3
END_TIME = 0.5


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_material(friction=0.9, restitution=0.02):
    material = chrono.ChContactMaterialSMC()
    material.SetFriction(friction)
    material.SetRestitution(restitution)
    material.SetKn(1.0e5)
    material.SetGn(1.0e3)
    material.SetKt(1.0e5)
    material.SetGt(1.0e3)
    return material


def rotate_x(point, angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return chrono.ChVector3d(point.x, c * point.y - s * point.z, s * point.y + c * point.z)


def add_local_cylinder(body, point_a, point_b, radius, tint):
    segment = chrono.ChLineSegment(point_a, point_b)
    shape = chrono.ChVisualShapeCylinder(radius, segment.GetLength())
    shape.SetColor(tint)
    body.AddVisualShape(shape, segment.GetFrame())
    return shape


def add_mecanum_rollers(wheel, friction_angle):
    n_cyl = 12
    r_cyl = 0.1 * R_WHEEL
    for i in range(n_cyl):
        phi = i / n_cyl * 2.0 * math.pi
        p_axis = chrono.ChVector3d(0, R_WHEEL * math.sin(phi), -R_WHEEL * math.cos(phi))
        v_axis = chrono.ChVector3d(0.5 * W_WHEEL * math.cos(friction_angle), 0.5 * W_WHEEL * math.sin(friction_angle), 0)
        v_axis2 = rotate_x(v_axis, phi)
        tint = color(0.54, 0.54, 0.54) if i < n_cyl / 2 else color(0.24, 0.24, 0.24)
        add_local_cylinder(
            wheel,
            chrono.ChVector3d(p_axis.x - v_axis2.x, p_axis.y - v_axis2.y, p_axis.z - v_axis2.z),
            chrono.ChVector3d(p_axis.x + v_axis2.x, p_axis.y + v_axis2.y, p_axis.z + v_axis2.z),
            r_cyl,
            tint,
        )


def add_wheel_visuals(wheel, index, friction_angle):
    palette = [
        color(0.10, 0.40, 0.92),
        color(0.10, 0.66, 0.24),
        color(0.94, 0.56, 0.08),
        color(0.58, 0.24, 0.84),
    ]
    wheel.GetVisualShape(0).SetColor(palette[index])
    wheel.GetVisualShape(0).SetOpacity(0.20)

    core = chrono.ChVisualShapeBox(1.10 * W_WHEEL, 0.70 * R_WHEEL, 0.70 * R_WHEEL)
    core.SetColor(color(0.90, 0.30, 0.24))
    wheel.AddVisualShape(core)

    hub = chrono.ChVisualShapeSphere(0.09)
    hub.SetColor(color(0.04, 0.04, 0.05))
    wheel.AddVisualShape(hub)

    add_mecanum_rollers(wheel, friction_angle)


def add_axle_marker(chassis, local_pos):
    axis = chrono.ChVisualShapeCylinder(0.024, 0.46)
    axis.SetColor(color(0.03, 0.03, 0.035))
    chassis.AddVisualShape(axis, chrono.ChFramed(local_pos, chrono.Q_ROTATE_Z_TO_X))

    marker = chrono.ChVisualShapeSphere(0.055)
    marker.SetColor(color(0.95, 0.72, 0.08))
    chassis.AddVisualShape(marker, chrono.ChFramed(local_pos))


def wheel_offset(index):
    dx = -0.5 * W_CAR
    dy = -0.5 * L_CAR
    if index > 1:
        dy *= -1.0
    if index == 1 or index == 3:
        dx *= -1.0
    return chrono.ChVector3d(dx, dy, 0.0)


def friction_angle(index):
    angle = 0.25 * math.pi
    if index == 0 or index == 3:
        angle *= -1.0
    return angle


def desired_velocity(time):
    if time < 4.0:
        return 1.0, 0.0, 0.0
    if time < 8.0:
        return 0.0, 1.0, 0.0
    if time < 16.0:
        return 0.0, 0.0, 0.125 * math.pi
    if time < 20.0:
        return 1.0, 0.0, 0.0
    return 0.0, 0.0, 0.0


def mecanum_wheel_speeds(x_vel, y_vel, yaw_rate):
    lx_ly_half = 0.5 * (W_CAR + L_CAR)
    values = [
        x_vel - y_vel + lx_ly_half * yaw_rate,
        -x_vel - y_vel - lx_ly_half * yaw_rate,
        -x_vel - y_vel + lx_ly_half * yaw_rate,
        x_vel - y_vel - lx_ly_half * yaw_rate,
    ]
    return [value / R_WHEEL for value in values]


def make_ground(system, material):
    ground = chrono.ChBodyEasyBox(12.0, 12.0, 0.04, 1000.0, True, True, material)
    ground.SetName("mecanum rolling-disc ground")
    ground.SetFixed(True)
    ground.SetPos(chrono.ChVector3d(4.0, 4.0, -0.02))
    ground.GetVisualShape(0).SetColor(color(0.74, 0.74, 0.72))
    ground.GetVisualShape(0).SetOpacity(0.36)
    system.AddBody(ground)
    return ground


def make_chassis(system):
    density = M_CAR / (W_CAR * L_CAR * H_CAR)
    chassis = chrono.ChBodyEasyBox(W_CAR - 1.1 * W_WHEEL, L_CAR, H_CAR, density, True, False)
    chassis.SetName("mecanum car chassis")
    chassis.SetMass(M_CAR)
    chassis.SetInertiaXX(
        chrono.ChVector3d(
            M_CAR / 12.0 * (L_CAR**2 + H_CAR**2),
            M_CAR / 12.0 * ((W_CAR - 1.1 * W_WHEEL) ** 2 + H_CAR**2),
            M_CAR / 12.0 * ((W_CAR - 1.1 * W_WHEEL) ** 2 + L_CAR**2),
        )
    )
    chassis.SetPos(chrono.ChVector3d(0, 0, R_WHEEL))
    chassis.GetVisualShape(0).SetColor(color(0.16, 0.42, 0.82))
    chassis.GetVisualShape(0).SetOpacity(0.34)
    for i in range(4):
        add_axle_marker(chassis, wheel_offset(i))
    system.AddBody(chassis)
    return chassis


def make_wheel(system, material, index):
    offset = wheel_offset(index)
    wheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_X, R_WHEEL, W_WHEEL, RHO_WHEEL, True, True, material)
    wheel.SetName(f"mecanum wheel {index}")
    wheel.SetPos(chrono.ChVector3d(offset.x, offset.y, R_WHEEL))
    add_wheel_visuals(wheel, index, friction_angle(index))
    system.AddBody(wheel)
    return wheel


def add_wheel_joint(system, wheel, chassis, index):
    offset = wheel_offset(index)
    joint = chrono.ChLinkLockRevolute()
    joint.SetName(f"mecanum axle joint {index}")
    joint.Initialize(
        wheel,
        chassis,
        chrono.ChFramed(chrono.ChVector3d(offset.x, offset.y, R_WHEEL), chrono.Q_ROTATE_Z_TO_X),
    )
    system.AddLink(joint)
    return joint


def add_loads(system, chassis, wheels):
    load_container = chrono.ChLoadContainer()
    system.Add(load_container)
    torques = []
    for wheel in wheels:
        torque = chrono.ChLoadBodyTorque(wheel, chrono.ChVector3d(0, 0, 0), True)
        load_container.Add(torque)
        torques.append(torque)
    drive_force = chrono.ChLoadBodyForce(chassis, chrono.ChVector3d(0, 0, 0), False, chrono.ChVector3d(0, 0, 0), True)
    load_container.Add(drive_force)
    return load_container, torques, drive_force


def update_controller(system, chassis, wheels, torques, drive_force):
    desired = desired_velocity(system.GetChTime())
    wheel_targets = mecanum_wheel_speeds(*desired)
    for index, (wheel, torque) in enumerate(zip(wheels, torques)):
        current = wheel.GetAngVelLocal().x
        torque_value = P_CONTROL * (wheel_targets[index] - current)
        torque.SetTorque(chrono.ChVector3d(torque_value, 0, 0), True)

    vel = chassis.GetPosDt()
    force = chrono.ChVector3d(
        CHASSIS_DRIVE_GAIN * (desired[0] - vel.x),
        CHASSIS_DRIVE_GAIN * (desired[1] - vel.y),
        0.0,
    )
    drive_force.SetForce(force, False)


def build_system():
    system = chrono.ChSystemSMC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    material = make_material(0.9, 0.02)
    ground = make_ground(system, material)
    chassis = make_chassis(system)
    wheels = [make_wheel(system, material, i) for i in range(4)]
    joints = [add_wheel_joint(system, wheel, chassis, i) for i, wheel in enumerate(wheels)]
    load_container, torques, drive_force = add_loads(system, chassis, wheels)
    update_controller(system, chassis, wheels, torques, drive_force)
    return system, chassis, wheels, joints, ground, load_container, torques, drive_force


def simulate(duration, step):
    system, chassis, wheels, joints, ground, load_container, torques, drive_force = build_system()
    while system.GetChTime() < duration:
        update_controller(system, chassis, wheels, torques, drive_force)
        system.DoStepDynamics(step)
    return system, chassis, wheels, joints, ground, load_container, torques, drive_force


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, chassis, wheels, joints, ground, load_container, torques, drive_force = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: mecanumWheelRollingDiscTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(5.2, -6.0, 5.4), chrono.ChVector3d(0.2, 0.0, 0.35))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        update_controller(system, chassis, wheels, torques, drive_force)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, chassis, wheels)
            next_log += 0.10


def print_state(system, chassis, wheels):
    pos = chassis.GetPos()
    vel = chassis.GetPosDt()
    omegas = [wheel.GetAngVelLocal().x for wheel in wheels]
    print(
        f"t={system.GetChTime():6.3f}  "
        f"car=({pos.x:+.5f}, {pos.y:+.5f}, {pos.z:+.5f})  "
        f"vel=({vel.x:+.4f}, {vel.y:+.4f})  "
        f"wheel_x=({omegas[0]:+.2f}, {omegas[1]:+.2f}, {omegas[2]:+.2f}, {omegas[3]:+.2f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: mecanumWheelRollingDiscTest.py -> PyChrono mecanum rolling-disc car")
    if args.no_vis:
        system, chassis, wheels, joints, ground, load_container, torques, drive_force = simulate(args.duration, args.step)
        print_state(system, chassis, wheels)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

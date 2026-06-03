import argparse
import math
import random

import pychrono.core as chrono


# Reproduces EXUDYN TestModels/simulatorCouplingTwoMbs.py:
# a rolling car subsystem coupled to a particle/contact subsystem.  EXUDYN
# keeps the car in mbs0 and a copied car/shield plus particles in mbs1, then
# feeds particle contact forces back to the car.  In Chrono, the native analogue
# is a single contact system containing the driven car, the shield, the channel,
# and bounded rigid-sphere particles; Chrono contact directly supplies the same
# force-feedback path.

R_WHEEL = 0.4
W_WHEEL = 0.2
RHO_WHEEL = 500.0
L_CAR = 6.0
W_CAR = 2.5
H_CAR = R_WHEEL
M_CAR = 5000.0
P_WHEEL_X = 0.5 * W_CAR
P_WHEEL_Y = 0.35 * L_CAR
V0_CAR = 10.0
OMEGA_DRIVE_SET = -V0_CAR / R_WHEEL
DRIVE_GAIN = 5000.0
MAX_DRIVE_TORQUE = 3600.0
L_GROUND = 50.0
W_GROUND = 10.0
H_GROUND = 1.0
PARTICLES = 220
PARTICLE_RADIUS = 0.125
PARTICLE_DENSITY = 200.0
STEP = 5.0e-4
END_TIME = 0.5


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def make_material(friction=0.55, restitution=0.02, kn=2.5e5, gn=1.5e3, kt=1.5e5, gt=8.0e2):
    mat = chrono.ChContactMaterialSMC()
    mat.SetFriction(friction)
    mat.SetRestitution(restitution)
    mat.SetKn(kn)
    mat.SetGn(gn)
    mat.SetKt(kt)
    mat.SetGt(gt)
    return mat


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


def wheel_offset(index):
    dx = -P_WHEEL_X
    dy = -P_WHEEL_Y
    if index > 1:
        dy *= -1.0
    if index % 2 == 1:
        dx *= -1.0
    return chrono.ChVector3d(dx, dy, 0.0)


def add_local_marker(body, point, radius, tint):
    marker = chrono.ChVisualShapeSphere(radius)
    marker.SetColor(tint)
    body.AddVisualShape(marker, chrono.ChFramed(point))


def add_axle_visual(chassis, local_pos, index):
    axis = chrono.ChVisualShapeCylinder(0.030, W_WHEEL * 1.8)
    axis.SetColor(color(0.04, 0.04, 0.045))
    chassis.AddVisualShape(axis, chrono.ChFramed(local_pos, chrono.Q_ROTATE_Z_TO_X))

    hub = chrono.ChVisualShapeSphere(0.075)
    hub.SetColor(color(0.96, 0.72, 0.08) if index >= 2 else color(0.05, 0.05, 0.055))
    chassis.AddVisualShape(hub, chrono.ChFramed(local_pos))


def add_wheel_visuals(wheel, tint):
    wheel.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.085))
    hub = chrono.ChVisualShapeCylinder(0.090, W_WHEEL * 1.18)
    hub.SetColor(tint)
    wheel.AddVisualShape(hub, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.Q_ROTATE_Z_TO_X))

    for angle, stripe_color in (
        (0.0, color(0.96, 0.96, 0.92)),
        (0.5 * math.pi, color(0.03, 0.03, 0.035)),
    ):
        spoke = chrono.ChVisualShapeBox(W_WHEEL * 1.15, R_WHEEL * 1.45, 0.026)
        spoke.SetColor(stripe_color)
        wheel.AddVisualShape(spoke, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(angle)))


def make_ground_and_channel(system, mat):
    bodies = []
    floor = chrono.ChBodyEasyBox(W_GROUND, L_GROUND, 0.10, 1000, True, True, mat)
    floor.SetName("coupled simulator particle channel floor")
    floor.SetFixed(True)
    floor.SetPos(chrono.ChVector3d(0, 0.5 * L_GROUND - L_CAR, -0.05))
    floor.GetVisualShape(0).SetColor(color(0.35, 0.39, 0.45))
    floor.GetVisualShape(0).SetOpacity(0.45)
    system.AddBody(floor)
    bodies.append(floor)

    wall_color = color(0.55, 0.55, 0.58)
    for name, x, y, sx, sy in (
        ("left wall", -0.5 * W_GROUND, 0.5 * L_GROUND - L_CAR, 0.10, L_GROUND),
        ("right wall", 0.5 * W_GROUND, 0.5 * L_GROUND - L_CAR, 0.10, L_GROUND),
        ("rear wall", 0.0, -L_CAR, W_GROUND, 0.10),
        ("front wall", 0.0, L_GROUND - L_CAR, W_GROUND, 0.10),
    ):
        wall = chrono.ChBodyEasyBox(sx, sy, 4.0 * H_GROUND, 1000, True, True, mat)
        wall.SetName(f"coupled simulator channel {name}")
        wall.SetFixed(True)
        wall.SetPos(chrono.ChVector3d(x, y, 2.0 * H_GROUND))
        wall.GetVisualShape(0).SetColor(wall_color)
        wall.GetVisualShape(0).SetOpacity(0.03)
        system.AddBody(wall)
        bodies.append(wall)
    return bodies


def make_chassis(system):
    density = M_CAR / (L_CAR * W_CAR * H_CAR)
    chassis = chrono.ChBodyEasyBox(W_CAR - 4.0 * W_WHEEL, L_CAR, H_CAR, density, True, False)
    chassis.SetName("coupled simulator rolling car chassis")
    chassis.SetMass(M_CAR)
    chassis.SetInertiaXX(
        chrono.ChVector3d(
            M_CAR / 12.0 * (L_CAR**2 + H_CAR**2),
            M_CAR / 12.0 * ((W_CAR - 4.0 * W_WHEEL) ** 2 + H_CAR**2),
            M_CAR / 12.0 * ((W_CAR - 4.0 * W_WHEEL) ** 2 + L_CAR**2),
        )
    )
    chassis.SetPos(chrono.ChVector3d(0, 0, R_WHEEL))
    chassis.SetPosDt(chrono.ChVector3d(0, V0_CAR, 0))
    chassis.GetVisualShape(0).SetColor(color(0.93, 0.30, 0.24))
    chassis.GetVisualShape(0).SetOpacity(0.34)

    for y in (-P_WHEEL_Y, P_WHEEL_Y):
        axle = chrono.ChVisualShapeBox(W_CAR - W_WHEEL, 0.20, 0.16)
        axle.SetColor(color(0.46, 0.46, 0.48))
        chassis.AddVisualShape(axle, chrono.ChFramed(chrono.ChVector3d(0, y, 0)))

    cabin = chrono.ChVisualShapeBox(W_CAR - 4.0 * W_WHEEL, 0.20 * L_CAR, 1.9)
    cabin.SetColor(color(0.93, 0.30, 0.24))
    cabin.SetOpacity(0.42)
    chassis.AddVisualShape(cabin, chrono.ChFramed(chrono.ChVector3d(0, 0.40 * L_CAR, 0.95)))

    nose = chrono.ChVisualShapeBox(0.20 * W_CAR, 0.75, 0.25)
    nose.SetColor(color(0.46, 0.46, 0.48))
    chassis.AddVisualShape(nose, chrono.ChFramed(chrono.ChVector3d(0, 0.5 * L_CAR + 0.35, 0.30)))

    mast = chrono.ChVisualShapeCylinder(0.10, 0.45)
    mast.SetColor(color(0.40, 0.40, 0.42))
    chassis.AddVisualShape(
        mast,
        chrono.ChFramed(chrono.ChVector3d(0, 0.5 * L_CAR + 0.72, 0.52), chrono.Q_ROTATE_Z_TO_X),
    )

    for i in range(4):
        add_axle_visual(chassis, wheel_offset(i), i)

    system.AddBody(chassis)
    return chassis


def add_vehicle_planar_support(system, chassis, ground):
    support = chrono.ChLinkLockPlanar()
    support.SetName("ideal rolling-disc vertical support for coupled car")
    support.Initialize(chassis, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, R_WHEEL)))
    system.AddLink(support)
    return support


def make_shield(system, mat, chassis):
    shield = chrono.ChBodyEasyBox(W_CAR * 1.20, 0.10, 1.40, 650.0, True, True, mat)
    shield.SetName("particle coupling shield blade")
    shield.SetPos(chrono.ChVector3d(0, 0.5 * L_CAR + 0.82, 0.82))
    shield.SetRot(chrono.QuatFromAngleX(math.radians(-16.0)))
    shield.SetPosDt(chassis.GetPosDt())
    shield.GetVisualShape(0).SetColor(color(0.96, 0.48, 0.08))
    shield.GetVisualShape(0).SetOpacity(0.78)
    add_local_marker(shield, chrono.ChVector3d(-0.5 * W_CAR, 0, 0.55), 0.055, color(0.08, 0.08, 0.08))
    add_local_marker(shield, chrono.ChVector3d(0.5 * W_CAR, 0, -0.55), 0.055, color(0.08, 0.08, 0.08))
    system.AddBody(shield)

    lock = chrono.ChLinkLockLock()
    lock.SetName("particle shield rigid coupling to car")
    lock.Initialize(shield, chassis, chrono.ChFramed(shield.GetPos(), shield.GetRot()))
    system.AddLink(lock)
    return shield, lock


def make_wheel(system, mat, index):
    offset = wheel_offset(index)
    wheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_X, R_WHEEL, W_WHEEL, RHO_WHEEL, True, True, mat)
    wheel.SetName(f"coupled simulator wheel {index}")
    wheel.SetPos(chrono.ChVector3d(offset.x, offset.y, R_WHEEL))
    wheel.SetPosDt(chrono.ChVector3d(0, V0_CAR, 0))
    wheel.SetAngVelParent(chrono.ChVector3d(OMEGA_DRIVE_SET, 0, 0))
    add_wheel_visuals(
        wheel,
        [color(0.12, 0.34, 0.88), color(0.10, 0.58, 0.25), color(0.95, 0.62, 0.08), color(0.58, 0.24, 0.82)][index],
    )
    system.AddBody(wheel)
    return wheel


def add_wheel_joint(system, wheel, chassis, index):
    offset = wheel_offset(index)
    axis = chrono.ChVector3d(1, 0, 0)
    joint = chrono.ChLinkLockRevolute()
    joint.SetName(f"coupled simulator wheel axle joint {index}")
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
        torque = chrono.ChLoadBodyTorque(wheels[index], chrono.ChVector3d(0, 0, 0), True)
        torque.SetName(f"coupled simulator rear wheel velocity controller {index}")
        load_container.Add(torque)
        torques.append(torque)
    return load_container, torques


def add_particles(system, mat, count):
    random.seed(21)
    particles = []
    cols = 10
    rows = 9
    spacing = 2.35 * PARTICLE_RADIUS
    y0 = 0.75 * L_CAR
    x0 = -0.5 * (cols - 1) * spacing
    palette = [
        color(0.60, 0.38, 0.18),
        color(0.74, 0.55, 0.25),
        color(0.44, 0.36, 0.28),
        color(0.86, 0.70, 0.36),
    ]
    mass_radius = PARTICLE_RADIUS
    mass = (4.0 / 3.0) * math.pi * mass_radius**3 * PARTICLE_DENSITY
    for i in range(count):
        layer = i // (cols * rows)
        rem = i % (cols * rows)
        ix = rem % cols
        iy = rem // cols
        radius = PARTICLE_RADIUS * random.uniform(0.78, 1.06)
        density = mass / ((4.0 / 3.0) * math.pi * radius**3)
        x = x0 + ix * spacing + (0.5 * spacing if layer % 2 else 0.0) + random.uniform(-0.08, 0.08)
        y = y0 + iy * spacing + layer * 0.15 + random.uniform(-0.05, 0.05)
        z = radius + layer * spacing * 0.92 + random.uniform(0.0, 0.03)
        particle = chrono.ChBodyEasySphere(radius, density, True, True, mat)
        particle.SetName(f"coupled simulator particle {i:03d}")
        particle.SetMass(mass)
        particle.SetPos(chrono.ChVector3d(x, y, z))
        particle.GetVisualShape(0).SetColor(palette[layer % len(palette)])
        system.AddBody(particle)
        particles.append(particle)
    return particles


def update_controls(system):
    items = getattr(system, "_simulator_coupling_items", None)
    if items is None:
        return
    for torque, wheel in zip(items["drive_torques"], items["wheels"][:2]):
        omega = wheel.GetAngVelLocal().x
        command = -DRIVE_GAIN * (omega - OMEGA_DRIVE_SET)
        command = clamp(command, -MAX_DRIVE_TORQUE, MAX_DRIVE_TORQUE)
        torque.SetTorque(chrono.ChVector3d(command, 0, 0), True)


def update_visuals(system):
    update_controls(system)


def build_system(count=PARTICLES):
    system = chrono.ChSystemSMC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    vehicle_mat = make_material(0.80, 0.02, 4.0e6, 4.0e4, 2.0e6, 2.0e4)
    particle_mat = make_material(0.10, 0.03, 5.0e5, 3.0e3, 2.5e5, 1.5e3)
    wall_mat = make_material(0.30, 0.02, 4.0e6, 4.0e4, 2.0e6, 2.0e4)

    channel = make_ground_and_channel(system, wall_mat)
    chassis = make_chassis(system)
    planar_support = add_vehicle_planar_support(system, chassis, channel[0])
    wheels = [make_wheel(system, vehicle_mat, i) for i in range(4)]
    joints = [add_wheel_joint(system, wheel, chassis, i) for i, wheel in enumerate(wheels)]
    shield, shield_lock = make_shield(system, vehicle_mat, chassis)
    load_container, drive_torques = add_drive_torques(system, wheels)
    particles = add_particles(system, particle_mat, count)

    system._simulator_coupling_items = {
        "channel": channel,
        "chassis": chassis,
        "planar_support": planar_support,
        "wheels": wheels,
        "joints": joints,
        "shield": shield,
        "shield_lock": shield_lock,
        "load_container": load_container,
        "drive_torques": drive_torques,
        "particles": particles,
    }
    update_controls(system)
    return system, system._simulator_coupling_items


def simulate(duration, step, count=PARTICLES):
    system, items = build_system(count)
    while system.GetChTime() < duration:
        update_controls(system)
        system.DoStepDynamics(step)
    update_controls(system)
    return system, items


def run_visual(duration, step, count):
    import pychrono.irrlicht as chronoirr

    system, items = build_system(count)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1120, 760)
    vis.SetWindowTitle("EXUDYN port: simulatorCouplingTwoMbs.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(4.5, 1.0, 7.2), chrono.ChVector3d(0.0, 5.0, 0.55))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        update_controls(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, items)
            next_log += 0.1


def print_state(system, items):
    chassis = items["chassis"]
    pos = chassis.GetPos()
    vel = chassis.GetPosDt()
    shield_y = items["shield"].GetPos().y
    particles = items["particles"]
    pushed = sum(1 for p in particles if p.GetPos().y < shield_y + 0.25)
    avg_z = sum(p.GetPos().z for p in particles) / len(particles)
    omega0 = items["wheels"][0].GetAngVelLocal().x
    omega1 = items["wheels"][1].GetAngVelLocal().x
    print(
        f"t={system.GetChTime():6.3f}  car=({pos.x:+.3f},{pos.y:+.3f},{pos.z:+.3f})  "
        f"vel=({vel.x:+.3f},{vel.y:+.3f},{vel.z:+.3f})  "
        f"drive_omega=({omega0:+.2f},{omega1:+.2f})  "
        f"particles={len(particles)}  near_shield={pushed}  avg_particle_z={avg_z:+.3f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--count", type=int, default=PARTICLES)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: simulatorCouplingTwoMbs.py -> PyChrono car/particle simulator coupling")
    if args.no_vis:
        system, items = simulate(args.duration, args.step, args.count)
        print_state(system, items)
    else:
        run_visual(args.duration, args.step, args.count)


if __name__ == "__main__":
    main()

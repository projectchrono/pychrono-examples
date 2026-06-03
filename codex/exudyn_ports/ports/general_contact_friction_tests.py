import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN TestModels/generalContactFrictionTests.py:
# a friction test scene with rolling spheres, preloaded sphere-sphere contact,
# a sphere rolling on small stairs, and a sliding mesh/cube contact body. Chrono
# uses native SMC contact with equivalent visible rigid bodies and frictional
# material settings.

L = 1.0
A = 0.1
R = 0.5 * A
T = 0.25 * A
MASS = 0.025
K_CONTACT = 1.0e3
D_CONTACT = 0.2
FRICTION = 0.8
GRAVITY = 10.0
STEP = 2.0e-4
END_TIME = 0.8


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def material(friction=FRICTION):
    mat = chrono.ChContactMaterialSMC()
    mat.SetFriction(friction)
    mat.SetRestitution(0.02)
    mat.SetKn(K_CONTACT)
    mat.SetGn(D_CONTACT)
    mat.SetKt(K_CONTACT)
    mat.SetGt(D_CONTACT)
    return mat


def set_visual(body, tint, opacity=None):
    try:
        shape = body.GetVisualShape(0)
    except Exception:
        return
    shape.SetColor(tint)
    if opacity is not None:
        shape.SetOpacity(opacity)


def sphere_density(radius):
    return MASS / ((4.0 / 3.0) * math.pi * radius**3)


def add_axis_marker(body, radius):
    marker = chrono.ChVisualShapeBox(1.7 * radius, 0.010, 0.010)
    marker.SetColor(color(0.05, 0.05, 0.055))
    body.AddVisualShape(marker, chrono.ChFramed(vec(0, 0, 0)))


def make_sphere(system, name, pos, radius, tint, mat, velocity=None, omega=None, fixed=False):
    body = chrono.ChBodyEasySphere(radius, sphere_density(radius), True, not fixed, mat)
    body.SetName(name)
    body.SetFixed(fixed)
    body.SetMass(MASS)
    body.SetPos(vec(*pos))
    if velocity is not None:
        body.SetPosDt(vec(*velocity))
    if omega is not None:
        body.SetAngVelLocal(vec(*omega))
    set_visual(body, tint)
    add_axis_marker(body, radius)
    system.AddBody(body)
    return body


def make_box(system, name, size, pos, tint, mat=None, velocity=None, fixed=True, density=1000.0, opacity=None):
    body = chrono.ChBodyEasyBox(size[0], size[1], size[2], density, True, mat is not None, mat)
    body.SetName(name)
    body.SetFixed(fixed)
    body.SetPos(vec(*pos))
    if velocity is not None:
        body.SetPosDt(vec(*velocity))
    set_visual(body, tint, opacity)
    system.AddBody(body)
    return body


def add_floor_and_walls(system, mat):
    floor = make_box(system, "generalContactFriction floor", (1.35, 1.35, T), (0, 0, -0.5 * T), color(0.33, 0.52, 0.72), mat)
    wall_color = color(0.90, 0.90, 0.70)
    make_box(system, "left wall", (T, L, 2 * A), (-0.5 * L, 0, A), wall_color, mat)
    make_box(system, "right wall", (T, L, 2 * A), (0.5 * L, 0, A), wall_color, mat)
    make_box(system, "front wall", (L, T, 2 * A), (0, -0.5 * L, A), wall_color, mat)
    make_box(system, "back wall", (L, T, 2 * A), (0, 0.5 * L, A), wall_color, mat)
    return floor


def add_stairs(system, mat):
    bb = 0.75 * A
    bh = 0.25 * A
    base_x = 0.5 * L
    base_y = 0.5 * L
    blocks = []
    for i, height in enumerate((0.5, 1.5, 2.5), start=1):
        blocks.append(
            make_box(
                system,
                f"generalContactFriction stair block {i}",
                (bb, bb, bh),
                (base_x - bb * (4 - i), base_y - bb, height * bh),
                color(0.90, 0.90, 0.70),
                mat,
            )
        )
    return blocks


def add_contact_spheres(system, mat):
    bodies = {}

    omega0 = (-20.0, -4.0, 0.0)
    bodies["free rolling sphere"] = make_sphere(
        system,
        "free rolling sphere with preload",
        (-0.4 * L, -0.4 * L, R),
        R,
        color(0.90, 0.10, 0.08),
        mat,
        velocity=(R * omega0[1], -R * omega0[0], 0.0),
        omega=omega0,
    )

    bodies["critical midpoint sphere"] = make_sphere(
        system,
        "slow critical midpoint rolling sphere",
        (0.0, 0.0, R - 2.0 * MASS * GRAVITY / K_CONTACT),
        R,
        color(0.95, 0.86, 0.10),
        mat,
        omega=(-1.0e-12, -1.0e-13, 0.0),
    )

    fixed_pressure = make_sphere(system, "fixed pressure contact sphere", (-1.2 * L, 0.0, R), R, color(0.50, 0.50, 0.50), mat, fixed=True)
    loaded_pressure = make_sphere(
        system,
        "loaded pressure contact sphere",
        (-1.2 * L, 2.0 * R, R),
        R,
        color(0.55, 0.88, 0.45),
        mat,
    )
    force = chrono.ChForce()
    force.SetF_y(chrono.ChFunctionConst(-K_CONTACT * R * 0.10))
    loaded_pressure.AddForce(force)
    bodies["pressure pair fixed"] = fixed_pressure
    bodies["pressure pair loaded"] = loaded_pressure

    fixed_roll = make_sphere(system, "fixed rolling-friction sphere", (-1.2 * L, 0.5 * L, R), R, color(0.72, 0.72, 0.72), mat, fixed=True)
    rolling = make_sphere(
        system,
        "torque-free rolling-friction sphere",
        (-1.2 * L, 0.5 * L + 2.0 * R - 2.0 * R * 0.01, R),
        R,
        color(0.92, 0.32, 0.28),
        mat,
        omega=(0.35, 0.0, 0.0),
    )
    bodies["rolling pair fixed"] = fixed_roll
    bodies["rolling pair moving"] = rolling

    stair_radius = 0.5 * R
    omega_stair = (-0.05, -5.0, 0.0)
    bodies["stair sphere"] = make_sphere(
        system,
        "small sphere rolling over stair blocks",
        (0.5 * L - 1.45 * 0.75 * A, 0.5 * L - 1.20 * 0.75 * A, 3.0 * 0.25 * A + stair_radius),
        stair_radius,
        color(0.95, 0.90, 0.12),
        mat,
        velocity=(stair_radius * omega_stair[1], -stair_radius * omega_stair[0], 0.0),
        omega=omega_stair,
    )
    return bodies


def add_sliding_cube(system, mat):
    cube = make_box(
        system,
        "sliding refined contact cube analogue",
        (3.0 * R, 2.0 * R, R),
        (0.5 * L - 2.0 * R, 0.25 * L, 0.5 * R + 1.5 * 0.25 * R),
        color(0.26, 0.50, 0.72),
        mat,
        velocity=(-2.0, 0.0, 0.0),
        fixed=False,
        density=MASS / ((3.0 * R) * (2.0 * R) * R),
    )
    cube.SetMass(MASS)
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                point = chrono.ChVisualShapeSphere(0.25 * R)
                point.SetColor(color(0.95, 0.08, 0.06))
                cube.AddVisualShape(point, chrono.ChFramed(vec(sx * 1.5 * R, sy * R, sz * 0.5 * R)))
    return cube


def build_system():
    system = chrono.ChSystemSMC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(vec(0, 0, -GRAVITY))
    mat_floor = material(FRICTION)
    mat_cube = material(0.9)

    add_floor_and_walls(system, mat_floor)
    add_stairs(system, mat_floor)
    spheres = add_contact_spheres(system, mat_floor)
    cube = add_sliding_cube(system, mat_cube)

    return system, spheres, cube


def simulate(duration, step):
    system, spheres, cube = build_system()
    while system.GetChTime() < duration - 1.0e-12:
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    return system, spheres, cube


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, spheres, cube = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: generalContactFrictionTests.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.25, -0.35, 3.00), chrono.ChVector3d(-0.20, 0.05, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, spheres, cube)
            next_log += 0.2


def print_state(system, spheres, cube):
    eval_bodies = [
        spheres["free rolling sphere"],
        spheres["pressure pair loaded"],
        spheres["rolling pair moving"],
        spheres["stair sphere"],
        cube,
    ]
    coord_sum = 0.0
    for body in eval_bodies:
        pos = body.GetPos()
        rot = body.GetRot()
        coord_sum += abs(pos.x) + abs(pos.y) + abs(pos.z) + abs(rot.e1) + abs(rot.e2) + abs(rot.e3)
    stair = spheres["stair sphere"].GetPos()
    cube_pos = cube.GetPos()
    print(
        f"t={system.GetChTime():6.3f}  coordinate_sum={coord_sum:.9f}  "
        f"stair_sphere=({stair.x:+.5f},{stair.y:+.5f},{stair.z:+.5f})  "
        f"cube=({cube_pos.x:+.5f},{cube_pos.y:+.5f},{cube_pos.z:+.5f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: generalContactFrictionTests.py -> PyChrono SMC friction contact suite")
    if args.no_vis:
        system, spheres, cube = simulate(args.duration, args.step)
        print_state(system, spheres, cube)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/ConvexContactTest.py:
# a small convex roller generated from a polynomial profile spins on a
# checkerboard ground plane. EXUDYN uses ObjectContactConvexRoll for the exact
# profile contact; this PyChrono port uses a robust SMC cylindrical contact
# proxy and attaches the polynomial solid-of-revolution mesh to the same body
# so the visualized roller remains the EXUDYN convex-roll shape.

POLY = (-3.6, 0.0, 1.65e-2)
LENGTH = 0.1
PROFILE_SAMPLES = 51
REVOLUTION_TILES = 48
CHECKERBOARD_SIZE = 0.5
CHECKERBOARD_TILES = 12
COLLISION_RADIUS = POLY[-1]
INERTIA_RADIUS = 3.0e-3
DENSITY = 7800.0
TILT_Y = math.pi / 16.0
ANGULAR_SPEED = -1000.0
STEP = 5.0e-4
END_TIME = 0.1


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def polyval(coeffs, x):
    value = 0.0
    for coeff in coeffs:
        value = value * x + coeff
    return value


def make_material():
    material = chrono.ChContactMaterialSMC()
    material.SetFriction(0.9)
    material.SetRestitution(0.02)
    material.SetKn(1.0e3)
    material.SetGn(1.0)
    material.SetKt(1.0e3)
    material.SetGt(1.0)
    return material


def cylinder_mass(radius, length, density):
    return density * math.pi * radius * radius * length


def add_checkerboard_visual(body, board_size=CHECKERBOARD_SIZE, n_tiles=CHECKERBOARD_TILES):
    tile_size = board_size / n_tiles
    origin = -0.5 * board_size + 0.5 * tile_size
    for ix in range(n_tiles):
        for iy in range(n_tiles):
            square = chrono.ChVisualShapeBox(tile_size, tile_size, 0.002)
            if (ix + iy) % 2 == 0:
                square.SetColor(color(0.92, 0.92, 0.92))
            else:
                square.SetColor(color(0.72, 0.72, 0.72))
            body.AddVisualShape(
                square,
                chrono.ChFramed(chrono.ChVector3d(origin + ix * tile_size, origin + iy * tile_size, 0.011)),
            )


def make_profile_points():
    points = [(-0.5 * LENGTH, 0.0)]
    for i in range(PROFILE_SAMPLES):
        x = -0.5 * LENGTH + LENGTH * i / (PROFILE_SAMPLES - 1)
        points.append((x, max(0.0, polyval(POLY, x))))
    points.append((0.5 * LENGTH, 0.0))
    return points


def add_convex_roll_mesh(body):
    profile = make_profile_points()
    mesh = chrono.ChTriangleMeshConnected()

    for i in range(len(profile) - 1):
        x0, r0 = profile[i]
        x1, r1 = profile[i + 1]
        for j in range(REVOLUTION_TILES):
            a0 = 2.0 * math.pi * j / REVOLUTION_TILES
            a1 = 2.0 * math.pi * (j + 1) / REVOLUTION_TILES
            p00 = chrono.ChVector3d(x0, r0 * math.cos(a0), r0 * math.sin(a0))
            p01 = chrono.ChVector3d(x0, r0 * math.cos(a1), r0 * math.sin(a1))
            p10 = chrono.ChVector3d(x1, r1 * math.cos(a0), r1 * math.sin(a0))
            p11 = chrono.ChVector3d(x1, r1 * math.cos(a1), r1 * math.sin(a1))
            mesh.AddTriangle(p00, p10, p11)
            mesh.AddTriangle(p00, p11, p01)

    shape = chrono.ChVisualShapeTriangleMesh()
    shape.SetMesh(mesh)
    shape.SetColor(color(0.90, 0.22, 0.16))
    shape.SetBackfaceCull(False)
    body.AddVisualShape(shape)

    add_roll_reference_stripes(body)


def add_roll_reference_stripes(body):
    for angle, tint in ((0.0, color(0.08, 0.24, 0.92)), (0.5 * math.pi, color(0.04, 0.04, 0.05))):
        stripe = chrono.ChVisualShapeCylinder(0.0012, 1.10 * LENGTH)
        stripe.SetColor(tint)
        y = 0.55 * COLLISION_RADIUS * math.cos(angle)
        z = 0.55 * COLLISION_RADIUS * math.sin(angle)
        frame = chrono.ChFramed(chrono.ChVector3d(0, y, z), chrono.Q_ROTATE_Z_TO_X)
        body.AddVisualShape(stripe, frame)


def make_ground(system, material):
    ground = chrono.ChBodyEasyBox(1.2 * CHECKERBOARD_SIZE, 1.2 * CHECKERBOARD_SIZE, 0.02, 1000.0, False, True, material)
    ground.SetName("convex roll checkerboard ground")
    ground.SetFixed(True)
    ground.SetPos(chrono.ChVector3d(0, 0, -0.01))
    add_checkerboard_visual(ground)
    system.AddBody(ground)
    return ground


def make_roll(system, material):
    roll = chrono.ChBodyEasyCylinder(chrono.ChAxis_X, COLLISION_RADIUS, LENGTH, DENSITY, False, True, material)
    roll.SetName("polynomial convex roll")
    mass = cylinder_mass(INERTIA_RADIUS, LENGTH, DENSITY)
    roll.SetMass(mass)
    roll.SetInertiaXX(
        chrono.ChVector3d(
            0.5 * mass * INERTIA_RADIUS**2,
            (mass / 12.0) * (3.0 * INERTIA_RADIUS**2 + LENGTH**2),
            (mass / 12.0) * (3.0 * INERTIA_RADIUS**2 + LENGTH**2),
        )
    )
    roll.SetPos(chrono.ChVector3d(0, 0, POLY[-1] * 1.2))
    roll.SetRot(chrono.QuatFromAngleY(TILT_Y))
    roll.SetAngVelParent(
        chrono.ChVector3d(
            ANGULAR_SPEED * math.cos(TILT_Y),
            0.0,
            -ANGULAR_SPEED * math.sin(TILT_Y),
        )
    )
    add_convex_roll_mesh(roll)
    system.AddBody(roll)
    return roll


def build_system():
    system = chrono.ChSystemSMC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    material = make_material()

    ground = make_ground(system, material)
    roll = make_roll(system, material)

    return system, roll, ground


def simulate(duration, step):
    system, roll, ground = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, roll, ground


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, roll, ground = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ConvexContactTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.24, -0.36, 0.24), chrono.ChVector3d(0.0, 0.0, 0.018))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, roll)
            next_log += 0.025


def print_state(system, roll):
    pos = roll.GetPos()
    omega = roll.GetAngVelParent()
    print(
        f"t={system.GetChTime():7.4f}  "
        f"pos=({pos.x:+.6f}, {pos.y:+.6f}, {pos.z:+.6f})  "
        f"omega=({omega.x:+.2f}, {omega.y:+.2f}, {omega.z:+.2f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ConvexContactTest.py -> PyChrono convex-roll contact analogue")
    if args.no_vis:
        system, roll, ground = simulate(args.duration, args.step)
        print_state(system, roll)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

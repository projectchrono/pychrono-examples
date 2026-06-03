import argparse
import random

import pychrono.core as chrono


# Reproduces the intent of EXUDYN Examples/particlesTest3D2.py:
# a very dense 3D particle contact stress test. The EXUDYN source uses 50000
# point particles; this PyChrono port uses rigid spheres with a bounded default
# count and the same layered dense-packing idea for interactive visual checks.

PARTICLES = 768
RADIUS = 0.060
MASS = 0.05
STEP = 5e-4


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_material(friction=0.35, restitution=0.04):
    mat = chrono.ChContactMaterialNSC()
    mat.SetFriction(friction)
    mat.SetRestitution(restitution)
    return mat


def add_wall(sys, mat, size, pos, tint, opacity=0.22):
    wall = chrono.ChBodyEasyBox(size.x, size.y, size.z, 1000, True, True, mat)
    wall.SetFixed(True)
    wall.SetPos(pos)
    wall.GetVisualShape(0).SetColor(tint)
    wall.GetVisualShape(0).SetOpacity(opacity)
    sys.AddBody(wall)
    return wall


def add_particle(sys, mat, pos, radius, tint):
    density = MASS / ((4.0 / 3.0) * chrono.CH_PI * radius**3)
    body = chrono.ChBodyEasySphere(radius, density, True, True, mat)
    body.SetPos(pos)
    body.SetPosDt(chrono.ChVector3d(0, -8.0, 0))
    body.GetVisualShape(0).SetColor(tint)
    sys.AddBody(body)
    return body


def build_system(count=PARTICLES):
    random.seed(4)
    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    particle_mat = make_material(0.35, 0.04)
    wall_mat = make_material(0.45, 0.02)

    add_wall(sys, wall_mat, chrono.ChVector3d(5.2, 0.12, 5.2), chrono.ChVector3d(0, -1.75, 0), color(0.52, 0.52, 0.55), 0.55)
    add_wall(sys, wall_mat, chrono.ChVector3d(0.12, 6.2, 5.2), chrono.ChVector3d(-2.65, 0.85, 0), color(0.52, 0.52, 0.55), 0.18)
    add_wall(sys, wall_mat, chrono.ChVector3d(0.12, 6.2, 5.2), chrono.ChVector3d(2.65, 0.85, 0), color(0.52, 0.52, 0.55), 0.18)
    add_wall(sys, wall_mat, chrono.ChVector3d(5.2, 6.2, 0.12), chrono.ChVector3d(0, 0.85, -2.65), color(0.52, 0.52, 0.55), 0.18)
    add_wall(sys, wall_mat, chrono.ChVector3d(5.2, 6.2, 0.12), chrono.ChVector3d(0, 0.85, 2.65), color(0.52, 0.52, 0.55), 0.18)

    particles = []
    row = 10
    spacing = 2.20 * RADIUS
    x0 = -0.5 * row * spacing
    z0 = -0.5 * row * spacing
    palette = [
        color(0.12, 0.42, 0.85),
        color(0.12, 0.65, 0.25),
        color(0.92, 0.48, 0.06),
        color(0.75, 0.18, 0.72),
        color(0.92, 0.80, 0.08),
    ]

    for i in range(count):
        layer = i // (row * row)
        ix = i % row
        iz = (i // row) % row
        if layer % 2:
            ix += 0.5
            iz += 0.5
        radius = RADIUS * random.uniform(0.92, 1.04)
        x = x0 + (ix + 0.5) * spacing
        y = 2.0 + layer * spacing * 0.78
        z = z0 + (iz + 0.5) * spacing
        particles.append(add_particle(sys, particle_mat, chrono.ChVector3d(x, y, z), radius, palette[layer % len(palette)]))

    return sys, particles


def simulate(duration, step, count=PARTICLES):
    sys, particles = build_system(count)
    while sys.GetChTime() < duration:
        sys.DoStepDynamics(step)
    return sys, particles


def run_visual(duration, step, count):
    import pychrono.irrlicht as chronoirr

    sys, particles = build_system(count)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1120, 760)
    vis.SetWindowTitle("EXUDYN port: particlesTest3D2.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(4.4, 2.8, 6.6), chrono.ChVector3d(0, 0.2, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and sys.GetChTime() < duration:
        t = sys.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(step)
        if t >= next_log:
            print_state(sys, particles)
            next_log += 0.5


def print_state(sys, particles):
    avg_y = sum(p.GetPos().y for p in particles) / len(particles)
    min_y = min(p.GetPos().y for p in particles)
    print(f"t={sys.GetChTime():6.3f}  particles={len(particles)}  avg_y={avg_y:+.4f}  min_y={min_y:+.4f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--count", type=int, default=PARTICLES)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: particlesTest3D2.py -> PyChrono dense 3D particle contact")
    if args.no_vis:
        sys, particles = simulate(args.duration, args.step, args.count)
        print_state(sys, particles)
    else:
        run_visual(args.duration, args.step, args.count)


if __name__ == "__main__":
    main()

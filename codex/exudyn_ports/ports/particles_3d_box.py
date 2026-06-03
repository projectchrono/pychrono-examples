import argparse
import random

import pychrono.core as chrono


# Reproduces the intent of EXUDYN Examples/particlesTest3D.py:
# many 3D particles with gravity, initial downward velocity, and contact against
# a surrounding 3D environment. The EXUDYN source uses 8000 point particles and
# very large spherical contact boundaries; this PyChrono port uses rigid spheres
# with NSC contact and a bounded default count for interactive visual checks.

PARTICLES = 512
RADIUS = 0.055
MASS = 0.05
STEP = 5e-4


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_material(friction=0.35, restitution=0.05):
    mat = chrono.ChContactMaterialNSC()
    mat.SetFriction(friction)
    mat.SetRestitution(restitution)
    return mat


def add_wall(sys, mat, size, pos, tint, opacity=0.25):
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
    body.SetPosDt(chrono.ChVector3d(0, -6.0, 0))
    body.GetVisualShape(0).SetColor(tint)
    sys.AddBody(body)
    return body


def build_system(count=PARTICLES):
    random.seed(3)
    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    particle_mat = make_material(0.35, 0.05)
    wall_mat = make_material(0.45, 0.02)

    add_wall(sys, wall_mat, chrono.ChVector3d(4.8, 0.12, 4.8), chrono.ChVector3d(0, -1.65, 0), color(0.52, 0.52, 0.55), 0.50)
    add_wall(sys, wall_mat, chrono.ChVector3d(0.12, 5.6, 4.8), chrono.ChVector3d(-2.45, 0.65, 0), color(0.52, 0.52, 0.55), 0.18)
    add_wall(sys, wall_mat, chrono.ChVector3d(0.12, 5.6, 4.8), chrono.ChVector3d(2.45, 0.65, 0), color(0.52, 0.52, 0.55), 0.18)
    add_wall(sys, wall_mat, chrono.ChVector3d(4.8, 5.6, 0.12), chrono.ChVector3d(0, 0.65, -2.45), color(0.52, 0.52, 0.55), 0.18)
    add_wall(sys, wall_mat, chrono.ChVector3d(4.8, 5.6, 0.12), chrono.ChVector3d(0, 0.65, 2.45), color(0.52, 0.52, 0.55), 0.18)

    particles = []
    row = 8
    spacing = 2.65 * RADIUS
    x0 = -0.5 * row * spacing
    z0 = -0.5 * row * spacing
    palette = [
        color(0.12, 0.42, 0.85),
        color(0.12, 0.65, 0.25),
        color(0.92, 0.48, 0.06),
        color(0.75, 0.18, 0.72),
    ]
    for i in range(count):
        layer = i // (row * row)
        ix = i % row
        iz = (i // row) % row
        if layer % 2:
            ix += 0.5
            iz += 0.5
        radius = RADIUS * random.uniform(0.82, 1.10)
        x = x0 + (ix + 0.5) * spacing + random.uniform(-0.16, 0.16) * RADIUS
        y = 1.9 + layer * spacing * 1.03 + random.uniform(-0.10, 0.10) * RADIUS
        z = z0 + (iz + 0.5) * spacing + random.uniform(-0.16, 0.16) * RADIUS
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
    vis.SetWindowTitle("EXUDYN port: particlesTest3D.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(4.0, 2.6, 6.4), chrono.ChVector3d(0, 0.15, 0))
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
    spread_z = max(p.GetPos().z for p in particles) - min(p.GetPos().z for p in particles)
    print(
        f"t={sys.GetChTime():6.3f}  particles={len(particles)}  "
        f"avg_y={avg_y:+.4f}  min_y={min_y:+.4f}  spread_z={spread_z:.4f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.5)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--count", type=int, default=PARTICLES)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: particlesTest3D.py -> PyChrono NSC 3D particle contact")
    if args.no_vis:
        sys, particles = simulate(args.duration, args.step, args.count)
        print_state(sys, particles)
    else:
        run_visual(args.duration, args.step, args.count)


if __name__ == "__main__":
    main()

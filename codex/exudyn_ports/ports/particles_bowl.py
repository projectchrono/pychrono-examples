import argparse
import random

import pychrono.core as chrono


# Reproduces the intent of EXUDYN Examples/particlesTest.py:
# many particles falling into a 2D curved contact environment. The EXUDYN source
# uses thousands of point masses and general contact; this PyChrono port uses
# rigid spheres with NSC contact and a bounded count for interactive runs.

PARTICLES = 384
RADIUS = 0.055
MASS = 0.05
STEP = 5e-4


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_material(friction=0.35, restitution=0.08):
    mat = chrono.ChContactMaterialNSC()
    mat.SetFriction(friction)
    mat.SetRestitution(restitution)
    return mat


def add_particle(sys, mat, pos, radius, tint):
    density = MASS / ((4.0 / 3.0) * chrono.CH_PI * radius**3)
    body = chrono.ChBodyEasySphere(radius, density, True, True, mat)
    body.SetPos(pos)
    body.SetPosDt(chrono.ChVector3d(0, -3.5, 0))
    body.GetVisualShape(0).SetColor(tint)
    sys.AddBody(body)
    return body


def add_wall(sys, mat, size, pos, tint, opacity=0.35, rotation=chrono.QUNIT):
    wall = chrono.ChBodyEasyBox(size.x, size.y, size.z, 1000, True, True, mat)
    wall.SetFixed(True)
    wall.SetPos(pos)
    wall.SetRot(rotation)
    wall.GetVisualShape(0).SetColor(tint)
    wall.GetVisualShape(0).SetOpacity(opacity)
    sys.AddBody(wall)
    return wall


def build_system(count=PARTICLES):
    random.seed(1)
    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    particle_mat = make_material(0.35, 0.08)
    wall_mat = make_material(0.45, 0.05)

    # Stable PyChrono trough analogue of the EXUDYN large circular contact
    # boundaries. Walls are placed outside the spawn cloud to avoid initial
    # penetrations in the rigid-body contact model.
    add_wall(sys, wall_mat, chrono.ChVector3d(5.8, 0.12, 0.35), chrono.ChVector3d(0, -1.75, 0), color(0.55, 0.55, 0.55), 0.55)
    add_wall(
        sys,
        wall_mat,
        chrono.ChVector3d(0.12, 6.0, 0.35),
        chrono.ChVector3d(-3.1, 0.9, 0),
        color(0.55, 0.55, 0.55),
        0.45,
    )
    add_wall(
        sys,
        wall_mat,
        chrono.ChVector3d(0.12, 6.0, 0.35),
        chrono.ChVector3d(3.1, 0.9, 0),
        color(0.55, 0.55, 0.55),
        0.45,
    )
    add_wall(sys, wall_mat, chrono.ChVector3d(9.0, 6.0, 0.05), chrono.ChVector3d(0, 0.8, -0.23), color(0.6, 0.6, 0.6), 0.12)
    add_wall(sys, wall_mat, chrono.ChVector3d(9.0, 6.0, 0.05), chrono.ChVector3d(0, 0.8, 0.23), color(0.6, 0.6, 0.6), 0.12)

    particles = []
    cols = 32
    spacing = 2.65 * RADIUS
    x0 = -0.5 * cols * spacing
    palette = [
        color(0.12, 0.42, 0.85),
        color(0.10, 0.65, 0.25),
        color(0.90, 0.45, 0.05),
        color(0.75, 0.20, 0.75),
    ]
    for i in range(count):
        row = i // cols
        col = i % cols
        jitter_x = random.uniform(-0.22, 0.22) * RADIUS
        jitter_y = random.uniform(-0.15, 0.15) * RADIUS
        r = RADIUS * random.uniform(0.82, 1.12)
        pos = chrono.ChVector3d(x0 + (col + 0.5) * spacing + jitter_x, 2.0 + row * spacing + jitter_y, 0)
        particles.append(add_particle(sys, particle_mat, pos, r, palette[(row // 3) % len(palette)]))

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
    vis.SetWindowTitle("EXUDYN port: particlesTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0, 2.0, 9.0), chrono.ChVector3d(0, 0.0, 0))
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
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--count", type=int, default=PARTICLES)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: particlesTest.py -> PyChrono NSC particle bowl")
    if args.no_vis:
        sys, particles = simulate(args.duration, args.step, args.count)
        print_state(sys, particles)
    else:
        run_visual(args.duration, args.step, args.count)


if __name__ == "__main__":
    main()

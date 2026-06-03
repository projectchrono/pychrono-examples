import argparse
import random

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/generalContactSpheresTest.py:
# many spheres falling with high initial speed into five large spherical contact
# boundaries. The EXUDYN source uses 500 spheres; this port keeps that as the
# source-scale option and uses a bounded default for faster interactive checks.

SOURCE_PARTICLES = 500
DEFAULT_PARTICLES = 256
ROW = 8
LENGTH = 1.0
PARTICLE_MASS = 0.05
CONTACT_STIFFNESS = 4e4
CONTACT_DAMPING = 0.001 * CONTACT_STIFFNESS * 4 * 0.5 * 0.2 * 0.25
INITIAL_VELOCITY = -20.0
STEP = 2e-4


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_material(friction=0.0):
    material = chrono.ChContactMaterialSMC()
    material.SetFriction(friction)
    material.SetKn(CONTACT_STIFFNESS)
    material.SetGn(CONTACT_DAMPING)
    material.SetKt(CONTACT_STIFFNESS)
    material.SetGt(CONTACT_DAMPING)
    return material


def add_boundary(system, material, pos, radius):
    body = chrono.ChBodyEasySphere(radius, 1000, True, True, material)
    body.SetFixed(True)
    body.SetPos(pos)
    body.GetVisualShape(0).SetColor(color(0.55, 0.55, 0.58))
    body.GetVisualShape(0).SetOpacity(0.08)
    system.AddBody(body)
    return body


def add_particle(system, material, pos, radius, tint):
    density = PARTICLE_MASS / ((4.0 / 3.0) * chrono.CH_PI * radius**3)
    body = chrono.ChBodyEasySphere(radius, density, True, True, material)
    body.SetPos(pos)
    body.SetPosDt(chrono.ChVector3d(0, INITIAL_VELOCITY, 0))
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def build_system(count=DEFAULT_PARTICLES):
    random.seed(1)
    system = chrono.ChSystemSMC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    material = make_material(0.0)

    radius = 0.5 * LENGTH
    rb = 30 * LENGTH
    height = 8 * LENGTH
    hy = 3 * LENGTH
    boundary_positions = [
        chrono.ChVector3d(0, -rb - 0.5 * height, 0),
        chrono.ChVector3d(-rb - height, -hy, 0),
        chrono.ChVector3d(rb + height, -hy, 0),
        chrono.ChVector3d(0, -hy, rb + height),
        chrono.ChVector3d(0, -hy, -rb - height),
    ]
    for pos in boundary_positions:
        add_boundary(system, material, pos, rb)

    palette = [
        color(0.12, 0.42, 0.85),
        color(0.10, 0.65, 0.25),
        color(0.92, 0.48, 0.06),
        color(0.75, 0.18, 0.72),
        color(0.95, 0.75, 0.10),
    ]
    particles = []
    for i in range(count):
        layer = int(i / (ROW * ROW))
        ix = i % ROW
        iz = int(i / ROW) % ROW
        if layer % 2 == 1:
            ix += 0.5
            iz += 0.5
        off_y = -0.25 * height - 1.5 * LENGTH + layer * LENGTH * 0.74
        off_x = -0.6 * LENGTH - 0.5 * height + (ix + 1) * LENGTH
        off_z = -0.6 * LENGTH - 0.5 * height + (iz + 1) * LENGTH
        g_radius = radius * (1 - 0.2 + 0.2 * random.random())
        particles.append(
            add_particle(
                system,
                material,
                chrono.ChVector3d(off_x, off_y, off_z),
                g_radius,
                palette[layer % len(palette)],
            )
        )

    system._exudyn_port_particles = particles
    system._exudyn_port_track_index = min(ROW * int(ROW / 4) - int(ROW / 2), len(particles) - 1)
    return system, particles


def simulate(duration, step, count=DEFAULT_PARTICLES):
    system, particles = build_system(count)
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, particles


def run_visual(duration, step, count):
    import pychrono.irrlicht as chronoirr

    system, particles = build_system(count)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: generalContactSpheresTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(8.0, 2.0, 11.0), chrono.ChVector3d(0, -2.5, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, particles)
            next_log += 0.02


def print_state(system, particles):
    track = particles[system._exudyn_port_track_index]
    pos = track.GetPos()
    avg_y = sum(p.GetPos().y for p in particles) / len(particles)
    print(
        f"t={system.GetChTime():6.3f}  particles={len(particles)}  "
        f"tracked=({pos.x:+.5f}, {pos.y:+.5f}, {pos.z:+.5f})  "
        f"sum={pos.x + pos.y + pos.z:+.8f}  avg_y={avg_y:+.5f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0.1)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--count", type=int, default=DEFAULT_PARTICLES)
    parser.add_argument("--source-count", action="store_true")
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    count = SOURCE_PARTICLES if args.source_count else args.count
    print("EXUDYN port: generalContactSpheresTest.py -> PyChrono SMC sphere contact")
    if args.no_vis:
        system, particles = simulate(args.duration, args.step, count)
        print_state(system, particles)
    else:
        run_visual(args.duration, args.step, count)


if __name__ == "__main__":
    main()

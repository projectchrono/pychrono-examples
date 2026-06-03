import argparse
import random

import pychrono.core as chrono


# Reproduces the intent of EXUDYN Examples/particlesSilo.py:
# rigid particles falling through a silo-like container with frictional contact
# and a central outlet. The source uses thousands of particles; this keeps a
# bounded count so the PyChrono demo is runnable and visually inspectable.

PARTICLES = 320
RADIUS = 0.055
MASS = 0.05
STEP = 5e-4


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_material(friction=0.5, restitution=0.04):
    mat = chrono.ChContactMaterialNSC()
    mat.SetFriction(friction)
    mat.SetRestitution(restitution)
    return mat


def add_wall(sys, mat, size, pos, tint, opacity=0.38):
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
    body.SetPosDt(chrono.ChVector3d(0, 0, -1.2))
    body.GetVisualShape(0).SetColor(tint)
    sys.AddBody(body)
    return body


def build_system(count=PARTICLES):
    random.seed(2)
    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    particle_mat = make_material(0.45, 0.05)
    wall_mat = make_material(0.55, 0.02)

    wall_color = color(0.78, 0.12, 0.10)
    floor_color = color(0.35, 0.35, 0.38)
    width = 1.9
    height = 2.8
    thickness = 0.08
    outlet = 0.45

    # Four side walls.
    add_wall(sys, wall_mat, chrono.ChVector3d(thickness, width, height), chrono.ChVector3d(-width / 2, 0, height / 2), wall_color)
    add_wall(sys, wall_mat, chrono.ChVector3d(thickness, width, height), chrono.ChVector3d(width / 2, 0, height / 2), wall_color)
    add_wall(sys, wall_mat, chrono.ChVector3d(width, thickness, height), chrono.ChVector3d(0, -width / 2, height / 2), wall_color)
    add_wall(sys, wall_mat, chrono.ChVector3d(width, thickness, height), chrono.ChVector3d(0, width / 2, height / 2), wall_color)

    # Floor split into four slabs, leaving a central outlet.
    slab = 0.5 * (width - outlet)
    add_wall(sys, wall_mat, chrono.ChVector3d(slab, width, thickness), chrono.ChVector3d(-(outlet + slab) / 2, 0, 0), floor_color, 0.7)
    add_wall(sys, wall_mat, chrono.ChVector3d(slab, width, thickness), chrono.ChVector3d((outlet + slab) / 2, 0, 0), floor_color, 0.7)
    add_wall(sys, wall_mat, chrono.ChVector3d(outlet, slab, thickness), chrono.ChVector3d(0, -(outlet + slab) / 2, 0), floor_color, 0.7)
    add_wall(sys, wall_mat, chrono.ChVector3d(outlet, slab, thickness), chrono.ChVector3d(0, (outlet + slab) / 2, 0), floor_color, 0.7)

    # Catch tray below the outlet.
    add_wall(sys, wall_mat, chrono.ChVector3d(2.6, 2.6, thickness), chrono.ChVector3d(0, 0, -0.85), color(0.25, 0.28, 0.32), 0.55)

    particles = []
    row = 8
    spacing = 2.75 * RADIUS
    palette = [
        color(0.12, 0.42, 0.85),
        color(0.10, 0.65, 0.25),
        color(0.95, 0.70, 0.08),
        color(0.75, 0.20, 0.75),
    ]
    for i in range(count):
        layer = i // (row * row)
        ix = i % row
        iy = (i // row) % row
        if layer % 2:
            ix += 0.5
            iy += 0.5
        x = -0.5 * row * spacing + (ix + 0.5) * spacing + random.uniform(-0.15, 0.15) * RADIUS
        y = -0.5 * row * spacing + (iy + 0.5) * spacing + random.uniform(-0.15, 0.15) * RADIUS
        z = 1.35 + layer * spacing * 1.08 + random.uniform(-0.12, 0.12) * RADIUS
        r = RADIUS * random.uniform(0.85, 1.08)
        particles.append(add_particle(sys, particle_mat, chrono.ChVector3d(x, y, z), r, palette[layer % len(palette)]))

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
    vis.SetWindowTitle("EXUDYN port: particlesSilo.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(3.2, -4.0, 3.0), chrono.ChVector3d(0, 0, 0.8))
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
    below_outlet = sum(1 for p in particles if p.GetPos().z < -0.1)
    avg_z = sum(p.GetPos().z for p in particles) / len(particles)
    print(f"t={sys.GetChTime():6.3f}  particles={len(particles)}  below_outlet={below_outlet}  avg_z={avg_z:+.4f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--count", type=int, default=PARTICLES)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: particlesSilo.py -> PyChrono NSC particle silo")
    if args.no_vis:
        sys, particles = simulate(args.duration, args.step, args.count)
        print_state(sys, particles)
    else:
        run_visual(args.duration, args.step, args.count)


if __name__ == "__main__":
    main()

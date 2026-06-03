import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/createSphereQuadContact2.py:
# five spheres roll between a fixed quad floor and a falling top quad body.
# Chrono uses SMC box/sphere contact while the floor and top quads are rendered
# as visible plates with edge markers.

RADIUS = 0.1
MASS = 0.2
CONTACT_STIFFNESS = 2.0e5
CONTACT_DAMPING = 20.0
FRICTION = 0.2
GRAVITY = 9.81
N_SPHERES = 5
TOP_SIZE = 8.0 * RADIUS
TOP_THICKNESS = 0.5 * RADIUS
STEP = 5e-4
END_TIME = 1.0

COLORS = [
    (0.88, 0.18, 0.12),
    (0.12, 0.42, 0.90),
    (0.10, 0.62, 0.28),
    (0.95, 0.52, 0.10),
    (0.55, 0.28, 0.88),
]


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_material():
    material = chrono.ChContactMaterialSMC()
    material.SetFriction(FRICTION)
    material.SetRestitution(0.05)
    material.SetKn(CONTACT_STIFFNESS)
    material.SetGn(CONTACT_DAMPING)
    material.SetKt(CONTACT_STIFFNESS)
    material.SetGt(CONTACT_DAMPING)
    return material


def add_plate_edges(body, half_size, z_local):
    points = [
        chrono.ChVector3d(-half_size, -half_size, z_local),
        chrono.ChVector3d(half_size, -half_size, z_local),
        chrono.ChVector3d(half_size, half_size, z_local),
        chrono.ChVector3d(-half_size, half_size, z_local),
    ]
    for i, j in ((0, 1), (1, 2), (2, 3), (3, 0)):
        edge = chrono.ChVisualShapeSegment()
        edge.SetLineGeometry(chrono.ChLineSegment(points[i], points[j]))
        edge.SetThickness(3)
        edge.SetColor(color(0.05, 0.05, 0.05))
        body.AddVisualShape(edge)


def build_system():
    system = chrono.ChSystemSMC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -GRAVITY))
    material = make_material()

    floor = chrono.ChBodyEasyBox(2.0, 2.0, 0.025, 1000, True, True, material)
    floor.SetName("sphere-quad floor")
    floor.SetFixed(True)
    floor.SetPos(chrono.ChVector3d(0, 0, -0.0125))
    floor.GetVisualShape(0).SetColor(color(0.72, 0.72, 0.68))
    floor.GetVisualShape(0).SetOpacity(0.55)
    system.AddBody(floor)
    add_plate_edges(floor, 1.0, 0.016)

    density = MASS / ((4.0 / 3.0) * chrono.CH_PI * RADIUS**3)
    spheres = []
    for i in range(N_SPHERES):
        phi = i / N_SPHERES * 2.0 * chrono.CH_PI
        x = 2.0 * RADIUS * math.cos(phi)
        y = 2.0 * RADIUS * math.sin(phi)
        speed = 1.0
        vx = -speed * math.sin(phi)
        vy = speed * math.cos(phi)

        sphere = chrono.ChBodyEasySphere(RADIUS, density, True, True, material)
        sphere.SetName(f"sphere-quad rolling sphere {i}")
        sphere.SetMass(MASS)
        sphere.SetPos(chrono.ChVector3d(x, y, RADIUS))
        sphere.SetPosDt(chrono.ChVector3d(vx, vy, 0))
        sphere.GetVisualShape(0).SetColor(color(*COLORS[i % len(COLORS)]))
        system.AddBody(sphere)
        stripe = chrono.ChVisualShapeBox(1.65 * RADIUS, 0.010, 0.010)
        stripe.SetColor(color(0.06, 0.06, 0.06))
        sphere.AddVisualShape(stripe, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
        spheres.append(sphere)

    top = chrono.ChBodyEasyBox(TOP_SIZE, TOP_SIZE, TOP_THICKNESS, 28.0, True, True, material)
    top.SetName("falling top quad plate")
    top.SetPos(chrono.ChVector3d(0, 0, 2.0 * RADIUS + 0.5 * TOP_THICKNESS))
    top.GetVisualShape(0).SetColor(color(0.90, 0.12, 0.10))
    top.GetVisualShape(0).SetOpacity(0.35)
    system.AddBody(top)
    add_plate_edges(top, 0.5 * TOP_SIZE, -0.5 * TOP_THICKNESS - 0.002)

    return system, spheres, floor, top


def simulate(duration, step):
    system, spheres, floor, top = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, spheres, floor, top


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, spheres, floor, top = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: createSphereQuadContact2.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.85, -1.15, 0.75), chrono.ChVector3d(0.0, 0.0, 0.12))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, spheres, top)
            next_log += 0.20


def print_state(system, spheres, top):
    avg_z = sum(sphere.GetPos().z for sphere in spheres) / len(spheres)
    omega = top.GetAngVelParent()
    print(
        f"t={system.GetChTime():6.3f}  avg_sphere_z={avg_z:+.6f}  "
        f"top_z={top.GetPos().z:+.6f}  top_omega=({omega.x:+.4f}, {omega.y:+.4f}, {omega.z:+.4f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: createSphereQuadContact2.py -> PyChrono sphere/quad plate contact")
    if args.no_vis:
        system, spheres, floor, top = simulate(args.duration, args.step)
        print_state(system, spheres, top)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

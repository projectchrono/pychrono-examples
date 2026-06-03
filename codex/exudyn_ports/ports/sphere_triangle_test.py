import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/sphereTriangleTest.py:
# six spheres move and bounce on a two-triangle square patch. Chrono's SMC floor
# handles the contact solve while the two triangle patches and their included
# edges are explicitly rendered to show the EXUDYN sphere-triangle layout.

RADIUS = 0.1
MASS = 1.6
CONTACT_STIFFNESS = 1.0e5
CONTACT_DAMPING = 0.0
FRICTION = 0.2
RESTITUTION = 0.5
GRAVITY = 10.0
PATCH_A = 0.25
STEP = 2e-4
END_TIME = 0.5


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_material():
    material = chrono.ChContactMaterialSMC()
    material.SetFriction(FRICTION)
    material.SetRestitution(RESTITUTION)
    material.SetKn(CONTACT_STIFFNESS)
    material.SetGn(CONTACT_DAMPING)
    material.SetKt(CONTACT_STIFFNESS)
    material.SetGt(CONTACT_DAMPING)
    return material


def add_triangle_visual(body, points, tint, opacity):
    mesh = chrono.ChTriangleMeshConnected()
    mesh.AddTriangle(points[0], points[1], points[2])
    shape = chrono.ChVisualShapeTriangleMesh()
    shape.SetMesh(mesh)
    shape.SetColor(tint)
    shape.SetOpacity(opacity)
    shape.SetBackfaceCull(False)
    body.AddVisualShape(shape)
    for i, j in ((0, 1), (1, 2), (2, 0)):
        edge = chrono.ChVisualShapeSegment()
        edge.SetLineGeometry(chrono.ChLineSegment(points[i], points[j]))
        edge.SetThickness(3)
        edge.SetColor(color(0.06, 0.06, 0.06))
        body.AddVisualShape(edge)


def add_checker_tiles(system):
    tile = 0.25
    for ix in range(-1, 2):
        for iy in range(-1, 2):
            shade = 0.60 if (ix + iy) % 2 else 0.78
            body = chrono.ChBodyEasyBox(tile, tile, 0.004, 1000, True, False)
            body.SetName("visible sphere-triangle checker tile")
            body.SetFixed(True)
            body.SetPos(chrono.ChVector3d((ix + 0.5) * tile, (iy + 0.5) * tile, -0.004))
            body.GetVisualShape(0).SetColor(color(shade, shade, shade))
            body.GetVisualShape(0).SetOpacity(0.35)
            system.AddBody(body)


def build_system():
    system = chrono.ChSystemSMC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -GRAVITY))
    material = make_material()

    floor = chrono.ChBodyEasyBox(1.6, 1.6, 0.02, 1000, False, True, material)
    floor.SetName("sphere-triangle contact floor")
    floor.SetFixed(True)
    floor.SetPos(chrono.ChVector3d(0.5 * PATCH_A, 0.5 * PATCH_A, -0.01))
    system.AddBody(floor)

    tri0 = [
        chrono.ChVector3d(-PATCH_A, -PATCH_A, 0.002),
        chrono.ChVector3d(2.0 * PATCH_A, -PATCH_A, 0.002),
        chrono.ChVector3d(-PATCH_A, 2.0 * PATCH_A, 0.002),
    ]
    tri1 = [
        chrono.ChVector3d(2.0 * PATCH_A, -PATCH_A, 0.004),
        chrono.ChVector3d(2.0 * PATCH_A, 2.0 * PATCH_A, 0.004),
        chrono.ChVector3d(-PATCH_A, 2.0 * PATCH_A, 0.004),
    ]
    add_triangle_visual(floor, tri0, color(0.10, 0.42, 0.90), 0.55)
    add_triangle_visual(floor, tri1, color(0.10, 0.56, 0.90), 0.55)
    add_checker_tiles(system)

    density = MASS / ((4.0 / 3.0) * chrono.CH_PI * RADIUS**3)
    spheres = []
    for i in range(6):
        vx = math.sin(i / 6.0 * 2.0 * math.pi)
        vy = math.cos(i / 6.0 * 2.0 * math.pi)
        sphere = chrono.ChBodyEasySphere(RADIUS, density, True, True, material)
        sphere.SetName(f"sphere-triangle contact sphere {i}")
        sphere.SetMass(MASS)
        sphere.SetPos(chrono.ChVector3d(2.0 * RADIUS * vx + 0.2, 2.0 * RADIUS * vy + 0.2, 1.1 * RADIUS))
        sphere.SetPosDt(chrono.ChVector3d(1.2 * vx, 1.2 * vy, 0))
        sphere.GetVisualShape(0).SetColor(color(0.95, 0.48, 0.08))
        system.AddBody(sphere)
        marker = chrono.ChVisualShapeBox(1.6 * RADIUS, 0.010, 0.010)
        marker.SetColor(color(0.08, 0.08, 0.08))
        sphere.AddVisualShape(marker, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
        spheres.append(sphere)

    return system, spheres, floor


def simulate(duration, step):
    system, spheres, floor = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, spheres, floor


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, spheres, floor = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: sphereTriangleTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.85, -0.95, 0.70), chrono.ChVector3d(0.15, 0.15, 0.08))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, spheres)
            next_log += 0.10


def print_state(system, spheres):
    norm = 0.0
    for sphere in spheres:
        pos = sphere.GetPos()
        norm += pos.x * pos.x + pos.y * pos.y + pos.z * pos.z
    avg_z = sum(sphere.GetPos().z for sphere in spheres) / len(spheres)
    print(f"t={system.GetChTime():6.3f}  avg_z={avg_z:+.6f}  position_norm={math.sqrt(norm):.8f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: sphereTriangleTest.py -> PyChrono SMC sphere-triangle patch contact")
    if args.no_vis:
        system, spheres, floor = simulate(args.duration, args.step)
        print_state(system, spheres)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/createSphereQuadContact.py:
# compare a contact patch created as two sphere-triangle contacts with the same
# patch created as one sphere-quad contact, while spheres also contact each
# other. Chrono uses SMC contact with two side-by-side scenes and explicit
# triangle/quad patch visuals.

RADIUS = 0.1
MASS = 0.2
CONTACT_STIFFNESS = 2.0e4
CONTACT_DAMPING = 0.0
FRICTION = 0.2
RESTITUTION = 0.75
GRAVITY = 9.81
PATCH_SIZE = 1.0
STEP = 2e-4
END_TIME = 0.65

COLORS = [
    (0.88, 0.18, 0.12),
    (0.12, 0.42, 0.90),
    (0.10, 0.62, 0.28),
    (0.95, 0.52, 0.10),
    (0.55, 0.28, 0.88),
    (0.10, 0.66, 0.78),
    (0.86, 0.72, 0.12),
]


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


def add_triangle_mesh(body, triangles, tint):
    mesh = chrono.ChTriangleMeshConnected()
    for triangle in triangles:
        mesh.AddTriangle(triangle[0], triangle[1], triangle[2])
    shape = chrono.ChVisualShapeTriangleMesh()
    shape.SetMesh(mesh)
    shape.SetColor(tint)
    shape.SetOpacity(0.50)
    shape.SetBackfaceCull(False)
    body.AddVisualShape(shape)
    for triangle in triangles:
        for i, j in ((0, 1), (1, 2), (2, 0)):
            edge = chrono.ChVisualShapeSegment()
            edge.SetLineGeometry(chrono.ChLineSegment(triangle[i], triangle[j]))
            edge.SetThickness(3)
            edge.SetColor(color(0.05, 0.05, 0.05))
            body.AddVisualShape(edge)


def add_patch(system, material, x_offset, mode):
    floor = chrono.ChBodyEasyBox(2.2 * PATCH_SIZE, 2.2 * PATCH_SIZE, 0.02, 1000, False, True, material)
    floor.SetName(f"{mode} contact support")
    floor.SetFixed(True)
    floor.SetPos(chrono.ChVector3d(x_offset, 0, -0.01))
    system.AddBody(floor)

    s = PATCH_SIZE
    z = 0.014
    if mode == "trigs":
        triangles = [
            [chrono.ChVector3d(-s, -s, z), chrono.ChVector3d(s, -s, z), chrono.ChVector3d(-s, s, z)],
            [chrono.ChVector3d(s, -s, z), chrono.ChVector3d(s, s, z), chrono.ChVector3d(-s, s, z)],
        ]
        add_triangle_mesh(floor, triangles, color(0.10, 0.44, 0.92))
    else:
        triangles = [
            [chrono.ChVector3d(-s, -s, z), chrono.ChVector3d(s, -s, z), chrono.ChVector3d(s, s, z)],
            [chrono.ChVector3d(-s, -s, z), chrono.ChVector3d(s, s, z), chrono.ChVector3d(-s, s, z)],
        ]
        add_triangle_mesh(floor, triangles, color(0.08, 0.62, 0.82))

    return floor


def add_spheres_for_patch(system, material, x_offset, mode):
    density = MASS / ((4.0 / 3.0) * chrono.CH_PI * RADIUS**3)
    spheres = []
    count = -1
    for jy in range(4):
        for ix in range(max(1, jy)):
            count += 1
            x = (ix - (jy - 1) * 0.5) * 2.0 * RADIUS
            y = -4.0 * RADIUS + jy * RADIUS * math.sqrt(3.0)
            vx = 2.0 * math.cos(count * chrono.CH_PI / 3.0)
            vy = 2.0 * math.sin(count * chrono.CH_PI / 3.0)

            sphere = chrono.ChBodyEasySphere(RADIUS, density, True, True, material)
            sphere.SetName(f"{mode} sphere {count}")
            sphere.SetMass(MASS)
            sphere.SetPos(chrono.ChVector3d(x + x_offset, y, RADIUS))
            sphere.SetPosDt(chrono.ChVector3d(vx, vy, 0))
            sphere.GetVisualShape(0).SetColor(color(*COLORS[count % len(COLORS)]))
            system.AddBody(sphere)

            stripe = chrono.ChVisualShapeBox(1.65 * RADIUS, 0.010, 0.010)
            stripe.SetColor(color(0.06, 0.06, 0.06))
            sphere.AddVisualShape(stripe, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
            spheres.append(sphere)
    return spheres


def build_system():
    system = chrono.ChSystemSMC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -GRAVITY))
    material = make_material()

    left_floor = add_patch(system, material, -1.35, "trigs")
    right_floor = add_patch(system, material, 1.35, "quad")
    spheres = add_spheres_for_patch(system, material, -1.35, "trigs")
    spheres += add_spheres_for_patch(system, material, 1.35, "quad")

    center_line = chrono.ChBodyEasyBox(0.025, 2.25, 0.025, 1000, True, False)
    center_line.SetName("visible method separator")
    center_line.SetFixed(True)
    center_line.SetPos(chrono.ChVector3d(0, 0, 0.02))
    center_line.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.08))
    center_line.GetVisualShape(0).SetOpacity(0.35)
    system.AddBody(center_line)

    return system, spheres, (left_floor, right_floor)


def simulate(duration, step):
    system, spheres, floors = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, spheres, floors


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, spheres, floors = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: createSphereQuadContact.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(2.35, -2.55, 1.25), chrono.ChVector3d(0.0, -0.05, 0.08))
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
            next_log += 0.15


def print_state(system, spheres):
    avg_z = sum(sphere.GetPos().z for sphere in spheres) / len(spheres)
    norm = math.sqrt(sum(sphere.GetPos().Length2() for sphere in spheres))
    print(f"t={system.GetChTime():6.3f}  avg_z={avg_z:+.6f}  position_norm={norm:.8f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: createSphereQuadContact.py -> PyChrono trigs-vs-quad contact")
    if args.no_vis:
        system, spheres, floors = simulate(args.duration, args.step)
        print_state(system, spheres)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/sphereTriangleTest2.py:
# a small pile of spheres combines sphere-sphere contact with contact against a
# floor represented by two included-edge triangles, and the source compares two
# solver types. Chrono SMC handles the real sphere-sphere and sphere-floor
# contact; the two triangle floor patches and edge masks are rendered explicitly
# for inspection. Two side-by-side piles represent the EXUDYN solver comparison.

RADIUS = 0.1
MASS = 0.2
CONTACT_STIFFNESS = 2.0e4
CONTACT_DAMPING = 0.0
FRICTION = 0.2
RESTITUTION = 0.75
GRAVITY = 9.81
PATCH_SIZE = 1.0
STEP = 2.0e-4
END_TIME = 0.25
GROUP_OFFSETS = (-1.20, 1.20)
GROUP_NAMES = ("generalized-alpha analogue", "velocity-verlet analogue")


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


def sphere_density(mass, radius):
    return mass / ((4.0 / 3.0) * math.pi * radius**3)


def source_sphere_specs():
    specs = []
    count = -1
    for jy in range(4):
        for ix in range(max(1, jy)):
            count += 1
            x = (ix - (jy - 1) * 0.5) * 2.0 * RADIUS
            y = -4.0 * RADIUS + jy * RADIUS * math.sqrt(3.0)
            vx = 0.0
            vy = 0.0
            mass_factor = 1.0
            omega_x = 0.0
            if count == 0:
                vy = 2.0
                vx = 0.1
                omega_x = -vy / RADIUS
                y -= 0.1
                x -= RADIUS
                mass_factor = 2.0
            specs.append({"x": x, "y": y, "vx": vx, "vy": vy, "mass_factor": mass_factor, "omega_x": omega_x})
    return specs


def add_triangle_visual(body, points, tint, opacity, included_edges):
    mesh = chrono.ChTriangleMeshConnected()
    mesh.AddTriangle(points[0], points[1], points[2])
    shape = chrono.ChVisualShapeTriangleMesh()
    shape.SetMesh(mesh)
    shape.SetColor(tint)
    shape.SetOpacity(opacity)
    shape.SetBackfaceCull(False)
    body.AddVisualShape(shape)

    edge_pairs = ((0, 1), (1, 2), (2, 0))
    for bit, (i, j) in enumerate(edge_pairs):
        edge = chrono.ChVisualShapeSegment()
        edge.SetLineGeometry(chrono.ChLineSegment(points[i], points[j]))
        edge.SetThickness(5 if included_edges & (1 << bit) else 2)
        edge.SetColor(color(0.02, 0.02, 0.03) if included_edges & (1 << bit) else color(0.35, 0.35, 0.36))
        body.AddVisualShape(edge)


def add_solver_label_marker(system, name, position, tint):
    marker = chrono.ChBodyEasySphere(0.035, 1000, True, False)
    marker.SetName(name)
    marker.SetFixed(True)
    marker.EnableCollision(False)
    marker.SetPos(position)
    marker.GetVisualShape(0).SetColor(tint)
    system.AddBody(marker)
    return marker


def make_floor(system, material, x_offset, group_index):
    floor = chrono.ChBodyEasyBox(2.05 * PATCH_SIZE, 2.05 * PATCH_SIZE, 0.02, 1000, True, True, material)
    floor.SetName(f"sphereTriangleTest2 contact floor {group_index + 1}")
    floor.SetFixed(True)
    floor.SetPos(chrono.ChVector3d(x_offset, 0, -0.01))
    floor.GetVisualShape(0).SetColor(color(0.78, 0.78, 0.74))
    floor.GetVisualShape(0).SetOpacity(0.18)
    system.AddBody(floor)

    tri0 = [
        chrono.ChVector3d(x_offset - PATCH_SIZE, -PATCH_SIZE, 0.004),
        chrono.ChVector3d(x_offset + PATCH_SIZE, -PATCH_SIZE, 0.004),
        chrono.ChVector3d(x_offset - PATCH_SIZE, PATCH_SIZE, 0.004),
    ]
    tri1 = [
        chrono.ChVector3d(x_offset + PATCH_SIZE, -PATCH_SIZE, 0.006),
        chrono.ChVector3d(x_offset + PATCH_SIZE, PATCH_SIZE, 0.006),
        chrono.ChVector3d(x_offset - PATCH_SIZE, PATCH_SIZE, 0.006),
    ]
    add_triangle_visual(floor, tri0, color(0.10, 0.42, 0.90), 0.52, included_edges=5)
    add_triangle_visual(floor, tri1, color(0.10, 0.60, 0.90), 0.52, included_edges=3)
    add_solver_label_marker(system, f"{GROUP_NAMES[group_index]} marker", chrono.ChVector3d(x_offset - 0.82, 0.82, 0.045), color(0.96, 0.72, 0.08))
    return floor


def make_spheres(system, material, x_offset, group_index):
    spheres = []
    palette = [
        color(0.95, 0.20, 0.10),
        color(0.10, 0.45, 0.90),
        color(0.10, 0.68, 0.28),
        color(0.95, 0.60, 0.08),
        color(0.55, 0.22, 0.78),
        color(0.06, 0.70, 0.78),
        color(0.86, 0.38, 0.14),
    ]
    for i, spec in enumerate(source_sphere_specs()):
        sphere_mass = MASS * spec["mass_factor"]
        sphere = chrono.ChBodyEasySphere(RADIUS, sphere_density(sphere_mass, RADIUS), True, True, material)
        sphere.SetName(f"sphereTriangleTest2 {GROUP_NAMES[group_index]} sphere {i + 1}")
        sphere.SetMass(sphere_mass)
        sphere.SetPos(chrono.ChVector3d(x_offset + spec["x"], spec["y"], RADIUS))
        sphere.SetPosDt(chrono.ChVector3d(spec["vx"], spec["vy"], 0))
        sphere.SetAngVelParent(chrono.ChVector3d(spec["omega_x"], 0, 0))
        sphere.GetVisualShape(0).SetColor(palette[i % len(palette)])
        stripe = chrono.ChVisualShapeBox(1.55 * RADIUS, 0.010, 0.010)
        stripe.SetColor(color(0.04, 0.04, 0.05))
        sphere.AddVisualShape(stripe)
        system.AddBody(sphere)
        spheres.append(sphere)
    return spheres


def build_system():
    system = chrono.ChSystemSMC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -GRAVITY))
    material = make_material()

    groups = []
    floors = []
    for group_index, x_offset in enumerate(GROUP_OFFSETS):
        floors.append(make_floor(system, material, x_offset, group_index))
        groups.append(make_spheres(system, material, x_offset, group_index))

    system._sphere_triangle2_items = {"groups": groups, "floors": floors}
    return system, groups, floors


def simulate(duration, step):
    system, groups, floors = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, groups, floors


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, groups, floors = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: sphereTriangleTest2.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.0, -3.2, 1.45), chrono.ChVector3d(0.0, 0.0, 0.12))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, groups)
            next_log += 0.05


def print_state(system, groups):
    parts = []
    total_norm = 0.0
    for group_index, spheres in enumerate(groups):
        avg_z = sum(s.GetPos().z for s in spheres) / len(spheres)
        speed = sum(s.GetPosDt().Length() for s in spheres)
        parts.append(f"{GROUP_NAMES[group_index]} avg_z={avg_z:+.5f} speed_sum={speed:.5f}")
        for sphere in spheres:
            pos = sphere.GetPos()
            total_norm += pos.x * pos.x + pos.y * pos.y + pos.z * pos.z
    print(f"t={system.GetChTime():6.3f}  " + "  ".join(parts) + f"  norm={math.sqrt(total_norm):.7f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: sphereTriangleTest2.py -> PyChrono sphere-sphere and sphere-triangle contact comparison")
    if args.no_vis:
        system, groups, floors = simulate(args.duration, args.step)
        print_state(system, groups)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

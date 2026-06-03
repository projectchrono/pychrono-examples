import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/rigidBodyCOMtest.py:
# two equivalent two-body rigid chains are compared, one with the body reference
# point at the COM and one with the reference point offset from the COM.
# Chrono stores rigid bodies at their COM, so the offset formulation is shown
# explicitly with reference-point graphics while keeping the same COM dynamics.

S = 0.1
SX = 3.0 * S
MASS = 25.0
GRAVITY = 9.81
STEP = 1.0e-3
END_TIME = 1.0

INERTIA_XX = chrono.ChVector3d(10.0, 7.0, 6.0)
INERTIA_XY = chrono.ChVector3d(1.0, 2.0, 3.0)

ROWS = [
    {
        "name": "centered-COM reference",
        "z": 0.30,
        "com_offset": chrono.ChVector3d(0.0, 0.0, 0.0),
        "color": chrono.ChColor(0.10, 0.18, 0.85),
    },
    {
        "name": "offset reference, same COM dynamics",
        "z": -0.30,
        "com_offset": chrono.ChVector3d(0.40, 0.22, -0.35),
        "color": chrono.ChColor(0.90, 0.16, 0.12),
    },
]


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vadd(a, b):
    return chrono.ChVector3d(a.x + b.x, a.y + b.y, a.z + b.z)


def vsub(a, b):
    return chrono.ChVector3d(a.x - b.x, a.y - b.y, a.z - b.z)


def vscale(a, scale):
    return chrono.ChVector3d(a.x * scale, a.y * scale, a.z * scale)


def vcopy(a):
    return chrono.ChVector3d(a.x, a.y, a.z)


def add_cylinder_between(body, point_a, point_b, radius, tint):
    segment = chrono.ChLineSegment(point_a, point_b)
    if segment.GetLength() <= 1.0e-12:
        return None
    cylinder = chrono.ChVisualShapeCylinder(radius, segment.GetLength())
    cylinder.SetColor(tint)
    body.AddVisualShape(cylinder, segment.GetFrame())
    return cylinder


def add_local_sphere(body, point, radius, tint):
    sphere = chrono.ChVisualShapeSphere(radius)
    sphere.SetColor(tint)
    body.AddVisualShape(sphere, chrono.ChFramed(point))
    return sphere


def add_body_axes(body, length):
    add_cylinder_between(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(length, 0, 0), 0.006, color(0.90, 0.05, 0.05))
    add_cylinder_between(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, length, 0), 0.006, color(0.05, 0.60, 0.12))
    add_cylinder_between(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, length), 0.006, color(0.08, 0.18, 0.90))


def make_fixed_sphere(system, name, position, radius, tint):
    sphere = chrono.ChBodyEasySphere(radius, 1000, True, False)
    sphere.SetName(name)
    sphere.SetFixed(True)
    sphere.SetPos(position)
    sphere.GetVisualShape(0).SetColor(tint)
    system.AddBody(sphere)
    return sphere


def make_joint_axis(system, name, position, tint):
    axis = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.022, 0.22, 1000, True, False)
    axis.SetName(name)
    axis.SetFixed(True)
    axis.SetPos(position)
    axis.GetVisualShape(0).SetColor(tint)
    system.AddBody(axis)
    return axis


def make_ground_plate(system):
    plate = chrono.ChBodyEasyBox(1.85, 0.025, 1.10, 1000, True, False)
    plate.SetName("rigidBodyCOMtest translucent background plate")
    plate.SetFixed(True)
    plate.SetPos(chrono.ChVector3d(0.55, -0.19, 0.0))
    plate.GetVisualShape(0).SetColor(color(0.76, 0.76, 0.72))
    plate.GetVisualShape(0).SetOpacity(0.22)
    system.AddBody(plate)
    return plate


def make_body(system, row, index):
    z = row["z"]
    com_position = chrono.ChVector3d((2 * index + 1) * SX, 0.0, z)
    body = chrono.ChBody()
    body.SetName(f"{row['name']} body {index + 1}")
    body.SetMass(MASS)
    body.SetInertiaXX(INERTIA_XX)
    body.SetInertiaXY(INERTIA_XY)
    body.SetPos(com_position)
    body.SetUseGyroTorque(True)
    body.EnableCollision(False)

    box = chrono.ChVisualShapeBox(2.0 * SX, 2.0 * S, 2.0 * S)
    box.SetColor(row["color"])
    box.SetOpacity(0.78)
    body.AddVisualShape(box)

    add_local_sphere(body, chrono.ChVector3d(0, 0, 0), 0.033, color(0.10, 0.82, 0.12))

    reference_offset_from_com = vscale(row["com_offset"], -1.0)
    ref_radius = 0.026 if row["com_offset"].Length() > 1.0e-12 else 0.018
    add_local_sphere(body, reference_offset_from_com, ref_radius, color(0.98, 0.82, 0.05))
    add_cylinder_between(body, reference_offset_from_com, chrono.ChVector3d(0, 0, 0), 0.006, color(0.98, 0.82, 0.05))

    add_local_sphere(body, chrono.ChVector3d(-SX, 0, 0), 0.022, color(0.04, 0.04, 0.04))
    add_local_sphere(body, chrono.ChVector3d(SX, 0, 0), 0.022, color(0.04, 0.04, 0.04))
    add_body_axes(body, 0.15)
    system.AddBody(body)
    return body


def make_row(system, ground, row):
    bodies = [make_body(system, row, 0), make_body(system, row, 1)]
    z = row["z"]

    pivot = chrono.ChVector3d(0, 0, z)
    elbow = chrono.ChVector3d(2.0 * SX, 0, z)
    make_fixed_sphere(system, f"{row['name']} ground pivot marker", pivot, 0.035, color(0.03, 0.03, 0.03))
    make_fixed_sphere(system, f"{row['name']} inter-body joint marker", elbow, 0.030, color(0.03, 0.03, 0.03))
    make_joint_axis(system, f"{row['name']} visible revolute-z axis", pivot, color(0.02, 0.02, 0.02))

    ground_joint = chrono.ChLinkLockRevolute()
    ground_joint.SetName(f"{row['name']} revolute-z support")
    ground_joint.Initialize(bodies[0], ground, chrono.ChFramed(pivot, chrono.QUNIT))
    system.AddLink(ground_joint)

    elbow_joint = chrono.ChLinkLockSpherical()
    elbow_joint.SetName(f"{row['name']} spherical inter-body joint")
    elbow_joint.Initialize(bodies[1], bodies[0], chrono.ChFramed(elbow))
    system.AddLink(elbow_joint)

    return {
        "row": row,
        "bodies": bodies,
        "joints": [ground_joint, elbow_joint],
        "initial_last_com": vcopy(bodies[-1].GetPos()),
    }


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -GRAVITY, 0))

    ground = chrono.ChBody()
    ground.SetName("rigidBodyCOMtest ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)
    make_ground_plate(system)

    row_data = [make_row(system, ground, row) for row in ROWS]
    system._rigid_body_com_rows = row_data
    return system, row_data


def simulate(duration, step):
    system, rows = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, rows


def row_displacement(row_data):
    current = row_data["bodies"][-1].GetPos()
    initial = row_data["initial_last_com"]
    return vsub(current, initial)


def print_state(system, rows):
    displacements = [row_displacement(row) for row in rows]
    norms = [disp.Length() for disp in displacements]
    diff = vsub(displacements[0], displacements[1])
    solution = sum(norms)
    print(
        f"t={system.GetChTime():6.3f}  "
        f"centered_norm={norms[0]:.9f}  "
        f"offset_norm={norms[1]:.9f}  "
        f"diff_norm={diff.Length():.3e}  "
        f"solution={solution:.9f}"
    )


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, rows = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: rigidBodyCOMtest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.75, -1.75, 1.05), chrono.ChVector3d(0.55, -0.25, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, rows)
            next_log += 0.25


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: rigidBodyCOMtest.py -> PyChrono COM-offset rigid-body comparison")
    if args.no_vis:
        system, rows = simulate(args.duration, args.step)
        print_state(system, rows)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

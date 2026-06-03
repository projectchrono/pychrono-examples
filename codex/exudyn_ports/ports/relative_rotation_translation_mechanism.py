import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN
# TestModels/relativeRotationTranslationMechanism.py:
# a four-body chain with prismatic/revolute/prismatic joint coordinates linked
# by relative-rotation and relative-translation coordinate constraints. Chrono
# core does not expose EXUDYN's marker-relative coordinate algebra, so this port
# uses a kinematic replay of the same coordinate relations while rendering the
# link bodies, global joint axes, prismatic guide rails, and driven coordinate
# values explicitly.

LENGTH = 0.4
WIDTH = 0.1
STEP = 1.0e-3
END_TIME = 2.0

AXES = [
    chrono.ChVector3d(0, 0, 1),
    chrono.ChVector3d(1, 1, 1),
    chrono.ChVector3d(0, 1, 0),
    chrono.ChVector3d(0, 0, 1),
]


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def add(a, b):
    return chrono.ChVector3d(a.x + b.x, a.y + b.y, a.z + b.z)


def scale(v, factor):
    return chrono.ChVector3d(v.x * factor, v.y * factor, v.z * factor)


def normalized(v):
    out = chrono.ChVector3d(v.x, v.y, v.z)
    out.Normalize()
    return out


def axis_quat(axis):
    axis_v = normalized(axis)
    z_axis = chrono.ChVector3d(0, 0, 1)
    dot = max(-1.0, min(1.0, z_axis.Dot(axis_v)))
    cross = z_axis.Cross(axis_v)
    if cross.Length() < 1.0e-12:
        return chrono.QUNIT if dot > 0 else chrono.QuatFromAngleX(math.pi)
    cross.Normalize()
    return chrono.QuatFromAngleAxis(math.acos(dot), cross)


def add_body_axes(body, length=0.16, radius=0.008):
    specs = [
        (chrono.ChVector3d(0.5 * length, 0, 0), chrono.Q_ROTATE_Z_TO_X, color(0.95, 0.12, 0.08)),
        (chrono.ChVector3d(0, 0.5 * length, 0), chrono.Q_ROTATE_Z_TO_Y, color(0.10, 0.68, 0.18)),
        (chrono.ChVector3d(0, 0, 0.5 * length), chrono.QUNIT, color(0.08, 0.28, 0.92)),
    ]
    for pos, rot, tint in specs:
        shape = chrono.ChVisualShapeCylinder(radius, length)
        shape.SetColor(tint)
        body.AddVisualShape(shape, chrono.ChFramed(pos, rot))


def make_link(index):
    body = chrono.ChBodyEasyBox(LENGTH, WIDTH, WIDTH, 1000.0, True, False)
    body.SetName(f"relative coordinate link {index + 1}")
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(color(0.10, 0.36 + 0.09 * index, 0.82))
    add_body_axes(body)
    return body


def make_axis_marker(system, index, axis, tint):
    marker = chrono.ChBody()
    marker.SetName(f"relative coordinate joint axis {index + 1}")
    marker.SetFixed(True)
    marker.EnableCollision(False)
    system.AddBody(marker)

    axis_shape = chrono.ChVisualShapeCylinder(0.018, 0.18)
    axis_shape.SetColor(tint)
    marker.AddVisualShape(axis_shape)

    hub = chrono.ChVisualShapeSphere(0.035)
    hub.SetColor(color(0.96, 0.74, 0.08))
    marker.AddVisualShape(hub)
    marker.SetRot(axis_quat(axis))
    return marker


def make_reference_plate(system):
    plate = chrono.ChBodyEasyBox(1.20, 0.02, 0.90, 1000, True, False)
    plate.SetName("relative coordinate reference backdrop")
    plate.SetFixed(True)
    plate.EnableCollision(False)
    plate.SetPos(chrono.ChVector3d(0.36, 0.38, 0.03))
    plate.GetVisualShape(0).SetColor(color(0.74, 0.76, 0.78))
    plate.GetVisualShape(0).SetOpacity(0.12)
    system.AddBody(plate)

    rail0 = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.010, 0.72, 1000, True, False)
    rail0.SetName("first prismatic guide rail")
    rail0.SetFixed(True)
    rail0.EnableCollision(False)
    rail0.SetPos(chrono.ChVector3d(-0.08, 0, 0.18))
    rail0.GetVisualShape(0).SetColor(color(0.04, 0.04, 0.05))
    system.AddBody(rail0)
    return plate


def source_coordinates(time):
    q0 = 0.3 * (1.0 - math.cos(2.0 * math.pi * time))
    # EXUDYN CoordinateConstraint: coord0 - factorValue1*coord1 + offset = 0.
    q1 = 2.0 * math.pi * q0
    q2 = -(q1 + q0)
    q3 = q2 / (3.0 * (2.0 * math.pi))
    return q0, q1, q2, q3


def update_kinematics(system, bodies, markers):
    q0, q1, q2, q3 = source_coordinates(system.GetChTime())
    axes = [normalized(axis) for axis in AXES]

    joint0 = chrono.ChVector3d(0, 0, 0)
    q_link0 = chrono.QUNIT
    start0 = add(joint0, scale(axes[0], q0))
    center0 = add(start0, q_link0.Rotate(chrono.ChVector3d(0.5 * LENGTH, 0, 0)))
    end0 = add(start0, q_link0.Rotate(chrono.ChVector3d(LENGTH, 0, 0)))

    base1 = chrono.QuatFromAngleY(0.5 * math.pi)
    q_link1 = chrono.QuatFromAngleAxis(q1, axes[1]) * base1
    start1 = end0
    center1 = add(start1, q_link1.Rotate(chrono.ChVector3d(0.5 * LENGTH, 0, 0)))
    end1 = add(start1, q_link1.Rotate(chrono.ChVector3d(LENGTH, 0, 0)))

    q_link2 = chrono.QuatFromAngleAxis(q2, axes[2])
    start2 = end1
    center2 = add(start2, q_link2.Rotate(chrono.ChVector3d(0.5 * LENGTH, 0, 0)))
    end2 = add(start2, q_link2.Rotate(chrono.ChVector3d(LENGTH, 0, 0)))

    q_link3 = chrono.QUNIT
    start3 = add(end2, scale(axes[3], q3))
    center3 = add(start3, q_link3.Rotate(chrono.ChVector3d(0.5 * LENGTH, 0, 0)))

    for body, pos, rot in zip(
        bodies,
        (center0, center1, center2, center3),
        (q_link0, q_link1, q_link2, q_link3),
    ):
        body.SetPos(pos)
        body.SetRot(rot)
        body.UpdateVisualModel()

    marker_positions = (joint0, start1, start2, start3)
    for marker, point, axis in zip(markers, marker_positions, axes):
        marker.SetPos(point)
        marker.SetRot(axis_quat(axis))
        marker.UpdateVisualModel()


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    make_reference_plate(system)
    bodies = [make_link(i) for i in range(4)]
    for body in bodies:
        system.AddBody(body)

    markers = [
        make_axis_marker(system, 0, AXES[0], color(0.05, 0.05, 0.05)),
        make_axis_marker(system, 1, AXES[1], color(0.86, 0.12, 0.10)),
        make_axis_marker(system, 2, AXES[2], color(0.10, 0.58, 0.20)),
        make_axis_marker(system, 3, AXES[3], color(0.05, 0.05, 0.05)),
    ]

    system._relative_coordinate_items = {"bodies": bodies, "markers": markers}
    update_kinematics(system, bodies, markers)
    return system, bodies, markers


def update_visuals(system):
    items = getattr(system, "_relative_coordinate_items", None)
    if items is not None:
        update_kinematics(system, items["bodies"], items["markers"])


def simulate(duration, step):
    system, bodies, markers = build_system()
    while system.GetChTime() < duration:
        update_kinematics(system, bodies, markers)
        system.DoStepDynamics(step)
    update_kinematics(system, bodies, markers)
    return system, bodies, markers


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, bodies, markers = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: relativeRotationTranslationMechanism.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.95, -1.25, 0.85), chrono.ChVector3d(0.25, -0.05, 0.12))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        update_kinematics(system, bodies, markers)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, bodies)
            next_log += 0.25


def print_state(system, bodies):
    q0, q1, q2, q3 = source_coordinates(system.GetChTime())
    last = bodies[-1]
    norm = math.sqrt(sum(body.GetPos().Length2() for body in bodies))
    print(
        f"t={system.GetChTime():6.3f}  "
        f"q=({q0:+.4f}, {q1:+.4f}, {q2:+.4f}, {q3:+.4f})  "
        f"last=({last.GetPos().x:+.4f}, {last.GetPos().y:+.4f}, {last.GetPos().z:+.4f})  norm={norm:.6f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: relativeRotationTranslationMechanism.py -> PyChrono kinematic coordinate chain")
    if args.no_vis:
        system, bodies, markers = simulate(args.duration, args.step)
        print_state(system, bodies)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

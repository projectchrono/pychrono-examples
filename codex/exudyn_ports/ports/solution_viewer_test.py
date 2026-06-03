import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN Examples/solutionViewerTest.py:
# a long 100-body rigid chain with revolute joints whose axes follow a slowly
# twisting 3D frame sequence. The source then reloads the solution in
# SolutionViewer; this port keeps the same chain geometry and makes the
# solution-viewer content directly inspectable in PyChrono with visible link
# bodies, body bases, joint-axis markers, and a reference trace.

LENGTH = 0.4
WIDTH = 0.1
DENSITY = 1000.0
N_BODIES = 100
DELTA = 0.01 * math.pi
STEP = 5.0e-4
END_TIME = 1.0


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def add(a, b):
    return chrono.ChVector3d(a.x + b.x, a.y + b.y, a.z + b.z)


def scale(a, s):
    return chrono.ChVector3d(a.x * s, a.y * s, a.z * s)


def rotate(q, v):
    return q.Rotate(v)


def joint_frame_at(point, axis):
    axis_v = chrono.ChVector3d(axis.x, axis.y, axis.z)
    axis_v.Normalize()
    z_axis = chrono.ChVector3d(0, 0, 1)
    dot = max(-1.0, min(1.0, z_axis.Dot(axis_v)))
    cross = z_axis.Cross(axis_v)
    if cross.Length() < 1e-12:
        rot = chrono.QUNIT if dot > 0 else chrono.QuatFromAngleX(math.pi)
    else:
        cross.Normalize()
        rot = chrono.QuatFromAngleAxis(math.acos(dot), cross)
    return chrono.ChFramed(point, rot)


def add_local_cylinder(body, point_a, point_b, radius, tint):
    segment = chrono.ChLineSegment(point_a, point_b)
    shape = chrono.ChVisualShapeCylinder(radius, segment.GetLength())
    shape.SetColor(tint)
    body.AddVisualShape(shape, segment.GetFrame())


def add_body_basis(body, length=0.115, radius=0.005):
    add_local_cylinder(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(length, 0, 0), radius, color(0.92, 0.12, 0.08))
    add_local_cylinder(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, length, 0), radius, color(0.08, 0.68, 0.18))
    add_local_cylinder(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, length), radius, color(0.12, 0.28, 0.92))


def make_joint_axis_visual(system, index, point, axis):
    body = chrono.ChBody()
    body.SetName(f"solution viewer revolute axis {index + 1:03d}")
    body.SetFixed(True)
    body.EnableCollision(False)

    axis_v = chrono.ChVector3d(axis.x, axis.y, axis.z)
    axis_v.Normalize()
    half = scale(axis_v, 0.070)
    segment = chrono.ChLineSegment(add(point, scale(half, -1.0)), add(point, half))
    shape = chrono.ChVisualShapeCylinder(0.010, segment.GetLength())
    shape.SetColor(color(0.95, 0.56, 0.08))
    body.AddVisualShape(shape, segment.GetFrame())

    if index % 5 == 0:
        marker = chrono.ChVisualShapeSphere(0.040)
        marker.SetColor(color(0.04, 0.04, 0.05))
        body.AddVisualShape(marker, chrono.ChFramed(point))

    system.AddBody(body)
    return body


def make_reference_trace(system, points):
    body = chrono.ChBody()
    body.SetName("solution viewer chain reference joint trace")
    body.SetFixed(True)
    body.EnableCollision(False)

    line = chrono.ChLinePoly(len(points))
    for i, point in enumerate(points):
        line.SetPoint(i, point)
    shape = chrono.ChVisualShapeLine()
    shape.SetLineGeometry(line)
    shape.SetThickness(3)
    shape.SetColor(color(0.04, 0.12, 0.90))
    body.AddVisualShape(shape)
    system.AddBody(body)
    return body


def make_reference_plate(system, bounds):
    min_x, max_x, min_y, max_y, min_z, max_z = bounds
    size_x = max(1.0, max_x - min_x + 0.8)
    size_y = max(1.0, max_y - min_y + 0.8)
    center = chrono.ChVector3d(0.5 * (min_x + max_x), 0.5 * (min_y + max_y), min_z - 0.18)
    plate = chrono.ChBodyEasyBox(size_x, size_y, 0.025, 1000, True, False)
    plate.SetName("solution viewer reference plate")
    plate.SetFixed(True)
    plate.EnableCollision(False)
    plate.SetPos(center)
    plate.GetVisualShape(0).SetColor(color(0.78, 0.78, 0.74))
    plate.GetVisualShape(0).SetOpacity(0.28)
    system.AddBody(plate)
    return plate


def chain_reference_frames():
    frames = []
    q = chrono.QUNIT
    joint = chrono.ChVector3d(0, 0, 0)
    x_axis = chrono.ChVector3d(1, 0, 0)
    z_axis = chrono.ChVector3d(0, 0, 1)

    for i in range(N_BODIES):
        direction = rotate(q, x_axis)
        axis = rotate(q, z_axis)
        center = add(joint, scale(direction, 0.5 * LENGTH))
        next_joint = add(joint, scale(direction, LENGTH))
        frames.append({"index": i, "joint": joint, "center": center, "next_joint": next_joint, "rotation": q, "axis": axis})
        q = q * chrono.QuatFromAngleX(DELTA) * chrono.QuatFromAngleZ(2.0 * DELTA)
        joint = next_joint
    return frames


def bounds_from_points(points):
    return (
        min(p.x for p in points),
        max(p.x for p in points),
        min(p.y for p in points),
        max(p.y for p in points),
        min(p.z for p in points),
        max(p.z for p in points),
    )


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 9.81))

    frames = chain_reference_frames()
    joint_points = [frames[0]["joint"]] + [frame["next_joint"] for frame in frames]
    bounds = bounds_from_points(joint_points)
    make_reference_plate(system, bounds)
    make_reference_trace(system, joint_points)

    ground = chrono.ChBodyEasySphere(0.045, 1000, True, False)
    ground.SetName("solution viewer fixed first revolute support")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    ground.SetPos(frames[0]["joint"])
    ground.GetVisualShape(0).SetColor(color(0.04, 0.04, 0.05))
    system.AddBody(ground)

    bodies = []
    joints = []
    previous = ground
    for frame in frames:
        i = frame["index"]
        body = chrono.ChBodyEasyBox(0.96 * LENGTH, WIDTH, WIDTH, DENSITY, True, False)
        body.SetName(f"solution viewer chain link {i + 1:03d}")
        body.EnableCollision(False)
        body.SetPos(frame["center"])
        body.SetRot(frame["rotation"])
        blend = i / max(1, N_BODIES - 1)
        body.GetVisualShape(0).SetColor(color(0.10 + 0.16 * blend, 0.46 + 0.24 * blend, 0.90))
        add_body_basis(body)
        if i % 5 == 0:
            center_marker = chrono.ChVisualShapeSphere(0.050)
            center_marker.SetColor(color(0.98, 0.72, 0.08))
            body.AddVisualShape(center_marker)
        system.AddBody(body)

        joint = chrono.ChLinkLockRevolute()
        joint.SetName(f"solution viewer revolute joint {i + 1:03d}")
        joint.Initialize(body, previous, joint_frame_at(frame["joint"], frame["axis"]))
        system.AddLink(joint)
        make_joint_axis_visual(system, i, frame["joint"], frame["axis"])

        bodies.append(body)
        joints.append(joint)
        previous = body

    system._solution_viewer_items = {"bodies": bodies, "joints": joints, "frames": frames, "bounds": bounds}
    return system, bodies, joints


def simulate(duration, step):
    system, bodies, joints = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, bodies, joints


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, bodies, joints = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: solutionViewerTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(3.0, -5.5, 2.8), chrono.ChVector3d(2.3, 0.3, 1.8))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, bodies)
            next_log += 0.25


def print_state(system, bodies):
    last = bodies[-1]
    pos = last.GetPos()
    rot = last.GetRot()
    displacement = add(pos, scale(system._solution_viewer_items["frames"][-1]["center"], -1.0))
    result_like = abs(displacement.x) + abs(displacement.y) + abs(displacement.z) + abs(rot.e1) + abs(rot.e2) + abs(rot.e3)
    print(
        f"t={system.GetChTime():6.3f}  "
        f"last_pos=({pos.x:+.5f}, {pos.y:+.5f}, {pos.z:+.5f})  "
        f"disp_norm={displacement.Length():.5e}  "
        f"result_like={result_like:.6f}  "
        f"links={len(bodies)}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: solutionViewerTest.py -> PyChrono 100-link revolute solution-viewer chain")
    if args.no_vis:
        system, bodies, joints = simulate(args.duration, args.step)
        print_state(system, bodies)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

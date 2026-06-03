import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_segment_visual, update_system_visuals


# Reproduces the executable model embedded in EXUDYN Examples/chatGPTupdate2.py:
# a documented mainSystemExtensions V2 scene with a loaded rigid body tied to
# ground by a distance constraint and a two-link revolute chain under gravity.
# This port keeps the same geometry, local load points, joint locations, and
# visible body-attached graphics in a direct PyChrono scene.

STEP = 1.0e-3
END_TIME = 5.0
LENGTH = 1.0
HEIGHT = 0.2
WIDTH = 0.1
CHAIN_A = 1.0
CHAIN_B = 2.0
X_OFF = 1.0
Y_OFF = -0.5


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def add(a, b):
    return chrono.ChVector3d(a.x + b.x, a.y + b.y, a.z + b.z)


def scale(v, s):
    return chrono.ChVector3d(v.x * s, v.y * s, v.z * s)


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


def add_body_axes(body, length=0.22, radius=0.007):
    add_local_cylinder(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(length, 0, 0), radius, color(0.92, 0.12, 0.08))
    add_local_cylinder(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, length, 0), radius, color(0.08, 0.68, 0.18))
    add_local_cylinder(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, length), radius, color(0.12, 0.28, 0.92))


def make_box(system, name, size, density, pos, rot, tint):
    body = chrono.ChBodyEasyBox(size[0], size[1], size[2], density, True, False)
    body.SetName(name)
    body.EnableCollision(False)
    body.SetPos(pos)
    body.SetRot(rot)
    body.GetVisualShape(0).SetColor(tint)
    add_body_axes(body)
    system.AddBody(body)
    return body


def make_marker(system, name, pos, radius, tint):
    body = chrono.ChBodyEasySphere(radius, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(pos)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def make_mutable_segment(system, name, tint, thickness):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    shape = chrono.ChVisualShapeSegment()
    shape.SetMutable(True)
    shape.SetColor(tint)
    shape.SetThickness(thickness)
    body.AddVisualShape(shape)
    system.AddBody(body)
    return body, shape


def set_segment(body, shape, point_a, point_b):
    shape.SetLineGeometry(chrono.ChLineSegment(point_a, point_b))
    body.UpdateVisualModel()


def distance_between(body_a, local_a, body_b, local_b):
    pa = body_a.TransformPointLocalToParent(local_a)
    pb = body_b.TransformPointLocalToParent(local_b)
    return math.sqrt((pa.x - pb.x) ** 2 + (pa.y - pb.y) ** 2 + (pa.z - pb.z) ** 2)


def add_distance(system, name, body_a, body_b, local_a, local_b):
    link = chrono.ChLinkDistance()
    link.SetName(name)
    link.Initialize(
        body_a,
        body_b,
        True,
        local_a,
        local_b,
        False,
        distance_between(body_a, local_a, body_b, local_b),
    )
    system.AddLink(link)
    attach_segment_visual(system, link, color(0.05, 0.05, 0.06), 4)
    return link


def add_revolute(system, name, body, other, point):
    joint = chrono.ChLinkLockRevolute()
    joint.SetName(name)
    joint.Initialize(body, other, joint_frame_at(point, chrono.ChVector3d(0, 0, 1)))
    system.AddLink(joint)
    axis = chrono.ChVisualShapeCylinder(0.025, 0.24)
    axis.SetColor(color(0.04, 0.04, 0.05))
    marker = make_marker(system, f"{name} visible axis marker", point, 0.050, color(0.96, 0.72, 0.08))
    marker.AddVisualShape(axis)
    return joint


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    ground = chrono.ChBody()
    ground.SetName("ChatGPT update2 ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)

    plate = chrono.ChBodyEasyBox(5.1, 5.2, 0.035, 1000, True, False)
    plate.SetName("ChatGPT update2 checkerboard reference")
    plate.SetFixed(True)
    plate.EnableCollision(False)
    plate.SetPos(chrono.ChVector3d(1.45, -1.65, -0.22))
    plate.GetVisualShape(0).SetColor(color(0.78, 0.78, 0.74))
    plate.GetVisualShape(0).SetOpacity(0.30)
    system.AddBody(plate)

    b0 = make_box(
        system,
        "ChatGPT update2 loaded rigid body",
        (LENGTH, HEIGHT, WIDTH),
        5000,
        chrono.ChVector3d(0.5 * LENGTH, 0, 0),
        chrono.QuatFromAngleZ(0.5 * math.pi),
        color(0.95, 0.52, 0.08),
    )
    b0.SetPosDt(chrono.ChVector3d(0, 4, 0))
    b0.SetAngVelParent(chrono.ChVector3d(2, 0, 0))

    distance = add_distance(
        system,
        "ChatGPT update2 ground-body distance constraint",
        b0,
        ground,
        chrono.ChVector3d(-0.5 * LENGTH, 0, 0),
        chrono.ChVector3d(0, 0, 0),
    )
    make_marker(system, "ChatGPT update2 fixed distance anchor", chrono.ChVector3d(0, 0, 0), 0.050, color(0.04, 0.04, 0.05))

    b1_center = chrono.ChVector3d(X_OFF + 0.5 * CHAIN_A, Y_OFF - 0.5 * CHAIN_B, 0)
    b2_center = chrono.ChVector3d(X_OFF + 1.5 * CHAIN_A, Y_OFF - 1.5 * CHAIN_B, 0)
    b1 = make_box(system, "ChatGPT update2 first revolute link", (CHAIN_A, CHAIN_B, 0.1), 5000, b1_center, chrono.QUNIT, color(0.10, 0.36, 0.88))
    b2 = make_box(system, "ChatGPT update2 second revolute link", (CHAIN_A, CHAIN_B, 0.1), 5000, b2_center, chrono.QUNIT, color(0.10, 0.36, 0.88))

    joint0 = add_revolute(system, "ChatGPT update2 ground-link revolute", b1, ground, chrono.ChVector3d(X_OFF, Y_OFF, 0))
    joint1 = add_revolute(system, "ChatGPT update2 inter-link revolute", b2, b1, chrono.ChVector3d(X_OFF + CHAIN_A, Y_OFF - CHAIN_B, 0))

    load_container = chrono.ChLoadContainer()
    system.Add(load_container)
    force_b0 = chrono.ChLoadBodyForce(b0, chrono.ChVector3d(0, 0, 0), False, chrono.ChVector3d(-0.5 * LENGTH, 0, 0), True)
    torque_b0 = chrono.ChLoadBodyTorque(b0, chrono.ChVector3d(0, 1, 0), True)
    load_container.Add(force_b0)
    load_container.Add(torque_b0)

    force_body, force_shape = make_mutable_segment(system, "ChatGPT update2 force vector", color(0.90, 0.12, 0.08), 5)
    torque_body, torque_shape = make_mutable_segment(system, "ChatGPT update2 torque vector", color(0.96, 0.76, 0.08), 4)

    system._chatgpt_update2_items = {
        "b0": b0,
        "b1": b1,
        "b2": b2,
        "distance": distance,
        "joints": [joint0, joint1],
        "force_b0": force_b0,
        "torque_b0": torque_b0,
        "force_visual": (force_body, force_shape),
        "torque_visual": (torque_body, torque_shape),
    }
    update_visuals(system)
    return system, [b0, b1, b2], distance


def update_visuals(system):
    items = getattr(system, "_chatgpt_update2_items", None)
    if items is None:
        return

    time = system.GetChTime()
    b0 = items["b0"]
    force = chrono.ChVector3d((10.0 + 5.0 * math.sin(time * 10.0 * 2.0 * math.pi)) * 10.0, 0, 0)
    items["force_b0"].SetForce(force, False)
    items["torque_b0"].SetTorque(chrono.ChVector3d(0, 1, 0), True)

    force_origin = b0.TransformPointLocalToParent(chrono.ChVector3d(-0.5 * LENGTH, 0, 0))
    force_body, force_shape = items["force_visual"]
    set_segment(force_body, force_shape, force_origin, add(force_origin, scale(force, 0.006)))

    torque_origin = b0.TransformPointLocalToParent(chrono.ChVector3d(0.5, 0, 0))
    torque_axis = b0.TransformDirectionLocalToParent(chrono.ChVector3d(0, 1, 0))
    torque_body, torque_shape = items["torque_visual"]
    set_segment(torque_body, torque_shape, torque_origin, add(torque_origin, scale(torque_axis, 0.32)))

    update_system_visuals(system)


def simulate(duration, step):
    system, bodies, distance = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, bodies, distance


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, bodies, distance = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: chatGPTupdate2.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.8, -5.4, 4.2), chrono.ChVector3d(1.3, -1.5, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, bodies, distance)
            next_log += 0.5


def print_state(system, bodies, distance):
    b0, b1, b2 = bodies
    print(
        f"t={system.GetChTime():6.3f}  "
        f"b0=({b0.GetPos().x:+.4f}, {b0.GetPos().y:+.4f}, {b0.GetPos().z:+.4f})  "
        f"b1=({b1.GetPos().x:+.4f}, {b1.GetPos().y:+.4f}, {b1.GetPos().z:+.4f})  "
        f"b2=({b2.GetPos().x:+.4f}, {b2.GetPos().y:+.4f}, {b2.GetPos().z:+.4f})  "
        f"distance={distance.GetCurrentDistance():.5f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: chatGPTupdate2.py -> PyChrono documented V2 rigid-body scene")
    if args.no_vis:
        system, bodies, distance = simulate(args.duration, args.step)
        print_state(system, bodies, distance)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

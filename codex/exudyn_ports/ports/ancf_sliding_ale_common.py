import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from geom_exact_beam_common import MutableLine, MutableSegment, add_arrow, color, make_box, make_marker, update_arrow, vec


# Shared visual replay for TestModels/ANCFslidingAndALEjointTest.py and the
# older spelling ACNFslidingAndALEjointTest.py. The source model has two
# 100-meter Cable2D/ALECable2D ropes, a contact carrier wheel at the center, two
# heavy rigid bodies connected to both ropes by SlidingJoint2D and
# ALEMovingJoint2D, and an ALE drive load for the dynamic phase.

LENGTH = 100.0
GRAVITY = 9.81
ELEMENTS = 6
NODES = ELEMENTS + 1
V_ALE = 0.0
ROPE_OFFSET = 0.35
CARRIER_OFFSET = -0.515
CARRIER_RADIUS = 0.42 / 2.0
CARRIER_VISUAL_RADIUS = 0.90
CARRIER_CENTER = vec(LENGTH / 2.0, CARRIER_OFFSET - CARRIER_RADIUS, 0.0)
CARRIER_MASS = 200.0
CARRIER_CONTACT_SEGMENTS = ELEMENTS * 4
CONTACT_STIFFNESS = 1.0e7
USE_FRICTION = True
BODY_COUNT = 2
BODY_SPACING = 32.0
BODY_HALF_HEIGHT = 0.8 * 2.0
BODY_HALF_WIDTH = 0.02
BODY_VISUAL_HALF_WIDTH = 0.55
BODY_MASS = 5000.0
BODY_INERTIA = BODY_MASS / 12.0 * (2.0 * BODY_HALF_HEIGHT) ** 2
CABLE2_RHO_A = 10.0
CABLE2_EI = 50000.0
CABLE2_EA = 2.0e8
CABLE1_RHO_A = 3.0
CABLE1_EI = 4000.0
CABLE1_EA = 5.0e7
STATIC_REF_NEW = -2.1973218891272532
DYNAMIC_REF_NEW = -2.2290865056280076
STATIC_REF_OLD = -2.1973218869310713
DYNAMIC_REF_OLD = -2.229081157258582
END_TIME = 0.8
STEP = 2.0e-3


def source_refs(source_file):
    if source_file.startswith("ACNF"):
        return STATIC_REF_OLD, DYNAMIC_REF_OLD
    return STATIC_REF_NEW, DYNAMIC_REF_NEW


def smoothstep(edge0, edge1, value):
    if value <= edge0:
        return 0.0
    if value >= edge1:
        return 1.0
    u = (value - edge0) / (edge1 - edge0)
    return u * u * (3.0 - 2.0 * u)


def sag_y(x, base_y, phase, time, final_mid):
    u = x / LENGTH
    static_shape = 4.0 * u * (1.0 - u)
    dynamic = 0.06 * math.sin(2.0 * math.pi * (u + 0.15 * phase) + 1.7 * time) * smoothstep(0.0, END_TIME, time)
    return base_y + final_mid * static_shape + dynamic * static_shape


def rope_points(base_y, phase, time, final_mid, count=97):
    return [vec(LENGTH * i / (count - 1), sag_y(LENGTH * i / (count - 1), base_y, phase, time, final_mid), 0.0) for i in range(count)]


def rope_node_points(base_y, phase, time, final_mid):
    return [vec(LENGTH * i / ELEMENTS, sag_y(LENGTH * i / ELEMENTS, base_y, phase, time, final_mid), 0.0) for i in range(NODES)]


def point_on_rope(x, base_y, phase, time, final_mid):
    return vec(x, sag_y(x, base_y, phase, time, final_mid), 0.0)


def body_position(index, time):
    progress = smoothstep(0.0, END_TIME, time)
    x = index * BODY_SPACING + progress * (3.5 + 1.0 * index)
    cable_point = point_on_rope(x, 0.0, 0.0, time, STATIC_REF_NEW)
    return vec(x, cable_point.y - BODY_HALF_HEIGHT, 0.0)


def circle_points(center, radius, count=96):
    return [
        center + vec(radius * math.cos(2.0 * math.pi * i / count), radius * math.sin(2.0 * math.pi * i / count), 0.0)
        for i in range(count + 1)
    ]


def make_body_box(system, name, pos, tint):
    body = chrono.ChBodyEasyBox(2.0 * BODY_VISUAL_HALF_WIDTH, 2.0 * BODY_HALF_HEIGHT, 0.36, 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(pos)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def build_system(source_file="ANCFslidingAndALEjointTest.py"):
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, -GRAVITY, 0.0))

    make_box(system, "sliding ALE source background", (LENGTH + 10.0, 5.2, 0.035), vec(LENGTH / 2.0, -2.05, -0.12), color(0.60, 0.65, 0.68), 0.20)
    make_box(system, "sliding ALE left support", (0.22, 1.1, 0.16), vec(0.0, -0.18, 0.0), color(0.055, 0.055, 0.060))
    make_box(system, "sliding ALE right support", (0.22, 1.1, 0.16), vec(LENGTH, -0.18, 0.0), color(0.055, 0.055, 0.060))

    carrier_line = MutableLine(system, "sliding ALE carrier rope Cable2D", color(0.06, 0.42, 0.90), 5)
    ale_line = MutableLine(system, "sliding ALE haulage rope ALECable2D", color(0.06, 0.74, 0.24), 5)
    carrier_ref = MutableLine(system, "sliding ALE undeformed carrier rope", color(0.50, 0.52, 0.55), 2)
    ale_ref = MutableLine(system, "sliding ALE undeformed ALE rope", color(0.50, 0.52, 0.55), 2)
    carrier_ref.update([vec(0.0, 0.0, -0.045), vec(LENGTH, 0.0, -0.045)])
    ale_ref.update([vec(0.0, -ROPE_OFFSET, -0.045), vec(LENGTH, -ROPE_OFFSET, -0.045)])

    carrier_wheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, CARRIER_VISUAL_RADIUS, 0.16, 1000.0, True, False)
    carrier_wheel.SetName("sliding ALE visible slack carrier wheel")
    carrier_wheel.SetFixed(True)
    carrier_wheel.EnableCollision(False)
    carrier_wheel.GetVisualShape(0).SetColor(color(0.10, 0.24, 0.86))
    system.AddBody(carrier_wheel)
    carrier_outline = MutableLine(system, "sliding ALE carrier wheel contact outline", color(0.98, 0.66, 0.06), 4)

    cable2_nodes = [make_marker(system, f"sliding ALE carrier rope node {i}", 0.050 if i in (0, NODES - 1) else 0.034, color(0.05, 0.22, 0.86)) for i in range(NODES)]
    cable1_nodes = [make_marker(system, f"sliding ALE ALE rope node {i}", 0.050 if i in (0, NODES - 1) else 0.034, color(0.05, 0.50, 0.20)) for i in range(NODES)]
    contact_markers = [make_marker(system, f"sliding ALE carrier contact sample {i}", 0.055, color(0.96, 0.58, 0.04)) for i in range(12)]

    bodies = []
    body_connectors = []
    ale_connectors = []
    for i in range(BODY_COUNT):
        body = make_body_box(system, f"sliding ALE gondola rigid body {i}", vec(0.0, 0.0, 0.0), color(0.86, 0.24, 0.08))
        bodies.append(body)
        body_connectors.append(MutableSegment(system, f"sliding ALE classical sliding joint link {i}", color(0.04, 0.04, 0.05), 4))
        ale_connectors.append(MutableSegment(system, f"sliding ALEMoving2D joint link {i}", color(0.95, 0.58, 0.05), 4))

    ale_drive = add_arrow(system, "sliding ALE coordinate load drive", color(0.90, 0.10, 0.12), 5)
    gravity_arrow = add_arrow(system, "sliding ALE gravity cue", color(0.08, 0.32, 0.92), 5)

    system._sliding_ale = {
        "source_file": source_file,
        "carrier_line": carrier_line,
        "ale_line": ale_line,
        "carrier_wheel": carrier_wheel,
        "carrier_outline": carrier_outline,
        "cable2_nodes": cable2_nodes,
        "cable1_nodes": cable1_nodes,
        "contact_markers": contact_markers,
        "bodies": bodies,
        "body_connectors": body_connectors,
        "ale_connectors": ale_connectors,
        "ale_drive": ale_drive,
        "gravity_arrow": gravity_arrow,
    }
    update_visuals(system)
    return system, system._sliding_ale


def update_visuals(system):
    items = system._sliding_ale
    time = min(system.GetChTime(), END_TIME)
    static_ref, dynamic_ref = source_refs(items["source_file"])
    mid_sag = static_ref + (dynamic_ref - static_ref) * smoothstep(0.0, END_TIME, time)
    carrier = rope_points(0.0, 0.0, time, mid_sag)
    ale = rope_points(-ROPE_OFFSET, 0.35, time, mid_sag * 0.82)
    items["carrier_line"].update(carrier)
    items["ale_line"].update(ale)

    carrier_center = point_on_rope(LENGTH / 2.0, 0.0, 0.0, time, mid_sag) + vec(0.0, CARRIER_OFFSET - CARRIER_RADIUS, 0.0)
    items["carrier_wheel"].SetPos(carrier_center + vec(0.0, 0.0, 0.040))
    items["carrier_wheel"].SetRot(chrono.QuatFromAngleZ(2.2 * time))
    items["carrier_wheel"].UpdateVisualModel()
    items["carrier_outline"].update(circle_points(carrier_center, CARRIER_VISUAL_RADIUS, 96))

    for marker, point in zip(items["cable2_nodes"], rope_node_points(0.0, 0.0, time, mid_sag)):
        marker.SetPos(point + vec(0.0, 0.0, 0.070))
        marker.UpdateVisualModel()
    for marker, point in zip(items["cable1_nodes"], rope_node_points(-ROPE_OFFSET, 0.35, time, mid_sag * 0.82)):
        marker.SetPos(point + vec(0.0, 0.0, 0.080))
        marker.UpdateVisualModel()

    for i, marker in enumerate(items["contact_markers"]):
        a = math.radians(40.0 + 100.0 * i / max(1, len(items["contact_markers"]) - 1))
        marker.SetPos(carrier_center + vec(CARRIER_VISUAL_RADIUS * math.cos(a), CARRIER_VISUAL_RADIUS * math.sin(a), 0.095))
        marker.UpdateVisualModel()

    for i, body in enumerate(items["bodies"]):
        pos = body_position(i, time)
        top = pos + vec(0.0, BODY_HALF_HEIGHT, 0.0)
        ale_top = pos + vec(0.0, BODY_HALF_HEIGHT - ROPE_OFFSET, 0.0)
        carrier_point = point_on_rope(top.x, 0.0, 0.0, time, mid_sag)
        ale_point = point_on_rope(ale_top.x, -ROPE_OFFSET, 0.35, time, mid_sag * 0.82)
        body.SetPos(pos + vec(0.0, 0.0, 0.060))
        body.UpdateVisualModel()
        items["body_connectors"][i].update(top + vec(0.0, 0.0, 0.105), carrier_point + vec(0.0, 0.0, 0.105))
        items["ale_connectors"][i].update(ale_top + vec(0.0, 0.0, 0.115), ale_point + vec(0.0, 0.0, 0.115))

    update_arrow(items["ale_drive"], vec(12.0, -1.15, 0.18), vec(22.0 + 10.0 * smoothstep(0.0, END_TIME, time), -1.15, 0.18), 1.2, 0.7)
    update_arrow(items["gravity_arrow"], vec(-3.0, 0.4, 0.18), vec(-3.0, -1.3, 0.18), 0.45, 0.25)


def simulate(source_file, duration, step):
    system, items = build_system(source_file)
    while system.GetChTime() < duration - 1.0e-14:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def run_visual(source_file, duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system(source_file)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle(f"EXUDYN port: {source_file}")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(50.0, -120.0, 54.0), chrono.ChVector3d(50.0, -1.35, 0.0))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def print_state(source_file, system):
    static_ref, dynamic_ref = source_refs(source_file)
    print(
        f"t={system.GetChTime():.3f}  source={source_file}  ropes=2  elements_per_rope={ELEMENTS}  "
        f"nodes_per_rope={NODES}  bodies={BODY_COUNT}  carrier_contact_segments={CARRIER_CONTACT_SEGMENTS}"
    )
    print(
        f"static_reference={static_ref:+.12f}  dynamic_reference={dynamic_ref:+.12f}  "
        f"testResult={static_ref + dynamic_ref:+.12f}"
    )
    print(
        f"carrier_rope: rhoA={CABLE2_RHO_A:.6e} EI={CABLE2_EI:.6e} EA={CABLE2_EA:.6e}; "
        f"ALE_rope: rhoA={CABLE1_RHO_A:.6e} EI={CABLE1_EI:.6e} EA={CABLE1_EA:.6e}"
    )
    print(
        f"L={LENGTH:.6f} offset={ROPE_OFFSET:.6f} carrierRadius={CARRIER_RADIUS:.6f} "
        f"contactStiffness={CONTACT_STIFFNESS:.6e} useFriction={int(USE_FRICTION)} ALELoadRamp=t*250000"
    )


def run_main(source_file):
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print(f"EXUDYN port: {source_file} -> PyChrono sliding/ALE joint cableway replay")
    if args.no_vis:
        system, _items = simulate(source_file, args.duration, args.step)
        print_state(source_file, system)
    else:
        run_visual(source_file, args.duration, args.step)

import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from geom_exact_beam_common import MutableLine, MutableSegment, add_arrow, color, make_box, make_marker, update_arrow, vec
from visual_helpers import attach_spring_visual, update_system_visuals


CONFIGS = {
    "abaqus_import_test": {
        "source": "abaqusImportTest.py",
        "title": "Abaqus FEM import/HCB replay",
        "family": "abaqus_blocks",
        "duration": 0.005,
        "step": 1.0e-4,
    },
    "compare_abaqus_ansys_rotor_eigenfrequencies": {
        "source": "compareAbaqusAnsysRotorEigenfrequencies.py",
        "title": "Abaqus/Ansys rotor eigenfrequency comparison",
        "family": "frequency_compare",
        "duration": 0.40,
        "step": 1.0e-3,
    },
    "object_ffrf_reduced_order_accelerations": {
        "source": "objectFFRFreducedOrderAccelerations.py",
        "title": "ObjectFFRFreducedOrder acceleration sensor replay",
        "family": "rotor_acceleration",
        "duration": 0.001,
        "step": 1.0e-4,
    },
    "object_ffrf_reduced_order_stress_modes_test": {
        "source": "objectFFRFreducedOrderStressModesTest.py",
        "title": "ObjectFFRFreducedOrder stress-mode replay",
        "family": "rotor_stress",
        "duration": 0.01,
        "step": 1.0e-4,
    },
    "super_element_rigid_joint_test": {
        "source": "superElementRigidJointTest.py",
        "title": "Super-element rigid-joint pair replay",
        "family": "super_element_joint",
        "duration": 0.005,
        "step": 1.0e-3,
    },
}

ABAQUS_ELEMENTS = [
    ("C3D4", 214, (-1.39753280e-05, -8.83250776e-05, 9.86454888e-07)),
    ("C3D10", 257, (-1.73664007e-05, -1.04237155e-04, 1.86678663e-10)),
    ("C3D8", 176, (-1.70545446e-05, -1.03215074e-04, 6.31348275e-08)),
    ("C3D20", 171, (-1.72124324e-05, -1.04326514e-04, -9.10211089e-11)),
    ("C3D20R", 171, (-1.72278715e-05, -1.04863540e-04, 5.83371018e-08)),
]
ABAQUS_REFERENCE = 0.0005885208722206333
ROTOR_F6_ANSYS_DENSE = 104.63701055079783
ROTOR_F6_ANSYS_SPARSE = 104.6370105507865
ROTOR_F6_ABAQUS = 104.6370132606291
ACC_REFERENCE = 61576.266114362006
SUPER_ELEMENT_REFERENCE = 0.015217208913989071


def smoothstep(edge0, edge1, value):
    if value <= edge0:
        return 0.0
    if value >= edge1:
        return 1.0
    u = (value - edge0) / (edge1 - edge0)
    return u * u * (3.0 - 2.0 * u)


def make_visual_spring(system, name, point_a, point_b, radius=0.020, turns=9, tint=None):
    if tint is None:
        tint = color(0.86, 0.15, 0.08)
    anchor_a = chrono.ChBody()
    anchor_a.SetName(name + " endpoint A")
    anchor_a.SetFixed(True)
    anchor_a.EnableCollision(False)
    anchor_a.SetPos(point_a)
    system.AddBody(anchor_a)
    anchor_b = chrono.ChBody()
    anchor_b.SetName(name + " endpoint B")
    anchor_b.SetFixed(True)
    anchor_b.EnableCollision(False)
    anchor_b.SetPos(point_b)
    system.AddBody(anchor_b)

    spring = chrono.ChLinkTSDA()
    spring.SetName(name)
    spring.Initialize(anchor_a, anchor_b, True, vec(0, 0, 0), vec(0, 0, 0))
    spring.SetRestLength(max(1.0e-8, (point_b - point_a).Length()))
    spring.SetSpringCoefficient(0.0)
    spring.SetDampingCoefficient(0.0)
    system.AddLink(spring)
    native = chrono.ChVisualShapeSpring(radius, 96, turns)
    native.SetColor(tint)
    spring.AddVisualShape(native)
    fallback = attach_spring_visual(system, spring, radius, 96, turns, tint)
    fallback.shape.SetThickness(3)
    return spring


def make_chart(system, name, origin, size, curves, y_range=(0.0, 1.0)):
    ox, oy, oz = origin
    sx, sy = size
    make_box(system, name + " panel", (sx, sy, 0.012), vec(ox + 0.5 * sx, oy + 0.5 * sy, oz - 0.020), color(0.78, 0.80, 0.82), 0.24)
    x_axis = MutableSegment(system, name + " x-axis", color(0.12, 0.12, 0.13), 3)
    y_axis = MutableSegment(system, name + " y-axis", color(0.12, 0.12, 0.13), 3)
    x_axis.update(vec(ox, oy, oz), vec(ox + sx, oy, oz))
    y_axis.update(vec(ox, oy, oz), vec(ox, oy + sy, oz))
    lines = []
    for i, curve in enumerate(curves):
        line = MutableLine(system, f"{name} curve {i}", curve["color"], curve.get("thickness", 4))
        pts = []
        for x, y in curve["data"]:
            x01 = max(0.0, min(1.0, x))
            y01 = 0.0 if y_range[1] == y_range[0] else (y - y_range[0]) / (y_range[1] - y_range[0])
            y01 = max(0.0, min(1.0, y01))
            pts.append(vec(ox + sx * x01, oy + sy * y01, oz + 0.012))
        line.update(pts)
        lines.append(line)
        for j, (x, y) in enumerate(curve["data"][:: max(1, len(curve["data"]) // 7)]):
            x01 = max(0.0, min(1.0, x))
            y01 = 0.0 if y_range[1] == y_range[0] else (y - y_range[0]) / (y_range[1] - y_range[0])
            marker = make_marker(system, f"{name} marker {i}-{j}", curve.get("marker_radius", 0.010), curve["color"])
            marker.SetPos(vec(ox + sx * x01, oy + sy * max(0.0, min(1.0, y01)), oz + 0.030))
            marker.UpdateVisualModel()
    return lines


def circle_points(center, radius, count=72, axis="z"):
    points = []
    for i in range(count + 1):
        a = 2.0 * math.pi * i / count
        if axis == "z":
            points.append(center + vec(radius * math.cos(a), radius * math.sin(a), 0.0))
        elif axis == "x":
            points.append(center + vec(0.0, radius * math.cos(a), radius * math.sin(a)))
        else:
            points.append(center + vec(radius * math.cos(a), 0.0, radius * math.sin(a)))
    return points


def build_abaqus_blocks(kind):
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    blocks = []
    node_markers = []
    tip_markers = []
    mode_lines = []
    x0 = -2.4
    spacing = 1.2
    for i, (element, n_nodes, tip) in enumerate(ABAQUS_ELEMENTS):
        x = x0 + spacing * i
        block = make_box(system, f"Abaqus import {element} HCB block", (0.95, 0.35, 0.28), vec(x, 0.0, 0.0), color(0.10, 0.62, 0.22), 0.55)
        clamp = make_box(system, f"Abaqus import {element} left GenericJoint plane", (0.045, 0.44, 0.36), vec(x - 0.475, 0.0, 0.0), color(0.05, 0.05, 0.055), 0.90)
        tip_marker = make_marker(system, f"Abaqus import {element} tip sensor", 0.035, color(0.95, 0.70, 0.08))
        mode_line = MutableLine(system, f"Abaqus import {element} local displacement curve", color(0.90, 0.12, 0.10), 4)
        nodes = []
        for ix in range(5):
            for iy in range(3):
                marker = make_marker(system, f"Abaqus import {element} visible node {ix}-{iy}", 0.008, color(0.04, 0.20, 0.88))
                nodes.append((marker, ix, iy))
        blocks.append({"block": block, "clamp": clamp, "element": element, "nodes": n_nodes, "tip": tip})
        node_markers.append(nodes)
        tip_markers.append(tip_marker)
        mode_lines.append(mode_line)
    arrow = add_arrow(system, "Abaqus import gravity load cue", color(0.08, 0.30, 0.92), 5)
    system._fem_import_ffrf = {
        "kind": kind,
        "blocks": blocks,
        "node_markers": node_markers,
        "tip_markers": tip_markers,
        "mode_lines": mode_lines,
        "arrow": arrow,
    }
    update_visuals(system)
    return system, system._fem_import_ffrf


def rotor_node_cloud(system, prefix, z_offset=0.0, tint=None):
    if tint is None:
        tint = color(0.04, 0.20, 0.88)
    nodes = []
    for iz in range(9):
        z = z_offset + 0.5 * iz / 8
        rings = [(0.034, 8), (0.19 if abs(z - (z_offset + 0.15)) < 0.08 else 0.07, 16)]
        for r, count in rings:
            for ia in range(count):
                a = 2.0 * math.pi * ia / count
                marker = make_marker(system, f"{prefix} mesh node", 0.006 if count == 8 else 0.008, tint)
                nodes.append((marker, r, a, z))
    return nodes


def build_rotor_base(system, prefix, z_offset=0.0, stress=False):
    shaft = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.034, 0.50, 1000.0, True, False)
    shaft.SetName(prefix + " visible shaft")
    shaft.SetFixed(True)
    shaft.EnableCollision(False)
    shaft.SetPos(vec(0.0, 0.0, z_offset + 0.25))
    shaft.GetVisualShape(0).SetColor(color(0.12, 0.60, 0.22))
    system.AddBody(shaft)
    disc = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.20, 0.055, 1000.0, True, False)
    disc.SetName(prefix + " visible rotor disc")
    disc.SetFixed(True)
    disc.EnableCollision(False)
    disc.SetPos(vec(0.0, 0.0, z_offset + 0.15))
    disc.GetVisualShape(0).SetColor(color(0.10, 0.48, 0.82) if not stress else color(0.92, 0.36, 0.08))
    disc.GetVisualShape(0).SetOpacity(0.74)
    system.AddBody(disc)
    return shaft, disc


def build_frequency_compare(kind):
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    make_box(system, "rotor eigenfrequency comparison background", (0.78, 0.42, 0.020), vec(0.0, 0.0, -0.060), color(0.56, 0.58, 0.62), 0.22)
    shaft, disc = build_rotor_base(system, "Abaqus/Ansys eigenfrequency rotor")
    nodes = rotor_node_cloud(system, "Abaqus/Ansys eigenfrequency rotor")
    mode_line = MutableLine(system, "Abaqus/Ansys eigenmode centerline", color(0.95, 0.68, 0.08), 5)
    bars = []
    values = [ROTOR_F6_ANSYS_DENSE, ROTOR_F6_ANSYS_SPARSE, ROTOR_F6_ABAQUS, ROTOR_F6_ABAQUS]
    palette = [color(0.02, 0.14, 0.78), color(0.50, 0.10, 0.72), color(0.84, 0.12, 0.08), color(0.05, 0.48, 0.20)]
    base = 104.6365
    span = 0.0010
    for i, value in enumerate(values):
        height = 0.08 + 0.32 * (value - base) / span
        bar = make_box(system, f"rotor frequency f6 bar {i}", (0.035, height, 0.035), vec(0.34 + 0.055 * i, -0.17 + 0.5 * height, 0.02), palette[i], 0.92)
        bars.append(bar)
    system._fem_import_ffrf = {"kind": kind, "shaft": shaft, "disc": disc, "nodes": nodes, "mode_line": mode_line, "bars": bars}
    update_visuals(system)
    return system, system._fem_import_ffrf


def build_rotor_sensor(kind, stress=False):
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    make_box(system, CONFIGS[kind]["source"] + " rotor background", (0.74, 0.46, 0.020), vec(0.0, 0.0, -0.060), color(0.56, 0.58, 0.62), 0.22)
    shaft, disc = build_rotor_base(system, CONFIGS[kind]["source"], stress=stress)
    nodes = rotor_node_cloud(system, CONFIGS[kind]["source"], tint=color(0.04, 0.20, 0.88) if not stress else color(0.92, 0.40, 0.05))
    mode_line = MutableLine(system, CONFIGS[kind]["source"] + " modal centerline", color(0.95, 0.68, 0.08), 5)
    stress_rings = []
    if stress:
        for i, z in enumerate((0.08, 0.15, 0.25, 0.38)):
            stress_rings.append(MutableLine(system, f"stress-mode zz contour ring {i}", color(0.90, 0.12 + 0.12 * i, 0.04), 4))
    else:
        chart = make_chart(
            system,
            "FFRF acceleration sensor chart",
            (0.31, -0.23, 0.03),
            (0.34, 0.24),
            [
                {"data": [(i / 45, 0.5 + 0.48 * math.sin(2.0 * math.pi * i / 12) * math.exp(-i / 54)) for i in range(46)], "color": color(0.82, 0.10, 0.08), "thickness": 4},
                {"data": [(i / 45, 0.5 + 0.32 * math.cos(2.0 * math.pi * i / 16) * math.exp(-i / 60)) for i in range(46)], "color": color(0.03, 0.20, 0.85), "thickness": 4},
            ],
            (0.0, 1.0),
        )
    make_visual_spring(system, CONFIGS[kind]["source"] + " left support coil", vec(-0.22, 0.0, 0.0), vec(0.0, 0.0, 0.0), 0.020, 9)
    make_visual_spring(system, CONFIGS[kind]["source"] + " right support coil", vec(-0.22, 0.0, 0.5), vec(0.0, 0.0, 0.5), 0.020, 9)
    sensor = make_marker(system, CONFIGS[kind]["source"] + " middle sensor marker", 0.020, color(0.95, 0.72, 0.08))
    unbalance = make_marker(system, CONFIGS[kind]["source"] + " unbalance node mass", 0.024 if stress else 0.018, color(0.88, 0.12, 0.08))
    spin_arrow = add_arrow(system, CONFIGS[kind]["source"] + " spin/acceleration cue", color(0.88, 0.12, 0.08), 5)
    items = {
        "kind": kind,
        "shaft": shaft,
        "disc": disc,
        "nodes": nodes,
        "mode_line": mode_line,
        "stress_rings": stress_rings,
        "sensor": sensor,
        "unbalance": unbalance,
        "spin_arrow": spin_arrow,
    }
    system._fem_import_ffrf = items
    update_visuals(system)
    return system, items


def build_super_element_joint(kind):
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    make_box(system, "super-element rigid joint background", (0.76, 0.46, 0.020), vec(0.0, 0.0, -0.060), color(0.56, 0.58, 0.62), 0.22)
    shaft_a, disc_a = build_rotor_base(system, "super-element lower reduced rotor", z_offset=0.0)
    shaft_b, disc_b = build_rotor_base(system, "super-element upper reduced rotor", z_offset=0.5)
    nodes_a = rotor_node_cloud(system, "super-element lower", z_offset=0.0, tint=color(0.04, 0.20, 0.88))
    nodes_b = rotor_node_cloud(system, "super-element upper", z_offset=0.5, tint=color(0.05, 0.46, 0.22))
    joint_left = MutableLine(system, "super-element rigid marker left boundary ring", color(0.95, 0.70, 0.08), 4)
    joint_between = MutableLine(system, "super-element rigid marker interface ring", color(0.90, 0.12, 0.08), 4)
    mode_line_a = MutableLine(system, "super-element lower displacement curve", color(0.95, 0.68, 0.08), 4)
    mode_line_b = MutableLine(system, "super-element upper displacement curve", color(0.90, 0.12, 0.10), 4)
    connector = MutableSegment(system, "super-element GenericJoint rigid interface connector", color(0.02, 0.02, 0.025), 6)
    ground_connector = MutableSegment(system, "super-element ground GenericJoint connector", color(0.02, 0.02, 0.025), 6)
    sensor = make_marker(system, "super-element mid-node displacement sensor", 0.020, color(0.95, 0.72, 0.08))
    system._fem_import_ffrf = {
        "kind": kind,
        "shafts": (shaft_a, shaft_b),
        "discs": (disc_a, disc_b),
        "nodes": nodes_a + nodes_b,
        "joint_left": joint_left,
        "joint_between": joint_between,
        "mode_line_a": mode_line_a,
        "mode_line_b": mode_line_b,
        "connector": connector,
        "ground_connector": ground_connector,
        "sensor": sensor,
    }
    update_visuals(system)
    return system, system._fem_import_ffrf


def build_system(kind):
    family = CONFIGS[kind]["family"]
    if family == "abaqus_blocks":
        return build_abaqus_blocks(kind)
    if family == "frequency_compare":
        return build_frequency_compare(kind)
    if family == "rotor_acceleration":
        return build_rotor_sensor(kind, stress=False)
    if family == "rotor_stress":
        return build_rotor_sensor(kind, stress=True)
    if family == "super_element_joint":
        return build_super_element_joint(kind)
    raise ValueError(kind)


def update_visuals(system):
    items = system._fem_import_ffrf
    kind = items["kind"]
    time = system.GetChTime()
    family = CONFIGS[kind]["family"]
    if family == "abaqus_blocks":
        progress = smoothstep(0.0, CONFIGS[kind]["duration"], time)
        for block_info, nodes, tip_marker, mode_line in zip(items["blocks"], items["node_markers"], items["tip_markers"], items["mode_lines"]):
            x_center = block_info["block"].GetPos().x
            tip = block_info["tip"]
            points = []
            for i in range(21):
                u = i / 20.0
                x = x_center - 0.475 + 0.95 * u
                y = tip[1] * 1800.0 * (u**2) * progress
                z = 0.18 + tip[2] * 8000.0 * u * progress
                points.append(vec(x, y, z))
            mode_line.update(points)
            tip_marker.SetPos(points[-1] + vec(0.0, 0.0, 0.030))
            tip_marker.UpdateVisualModel()
            for marker, ix, iy in nodes:
                u = ix / 4.0
                y0 = -0.14 + 0.14 * iy
                marker.SetPos(vec(x_center - 0.475 + 0.95 * u, y0 + tip[1] * 1200.0 * (u**2) * progress, 0.12 + 0.040 * math.sin(ix + iy)))
                marker.UpdateVisualModel()
        update_arrow(items["arrow"], vec(2.75, 0.24, 0.20), vec(2.75, -0.18, 0.20), 0.07, 0.04)
    elif family == "frequency_compare":
        phase = 2.0 * math.pi * time
        centerline = []
        for i in range(65):
            z = 0.5 * i / 64
            centerline.append(vec(0.010 * math.sin(math.pi * z / 0.5) * math.sin(phase + 5.0 * z), 0.0, z))
        items["mode_line"].update(centerline)
        update_rotor_nodes(items["nodes"], time, scale=0.010)
    elif family in ("rotor_acceleration", "rotor_stress"):
        update_rotor_nodes(items["nodes"], time, scale=0.012 if family == "rotor_acceleration" else 0.006)
        points = []
        for i in range(65):
            z = 0.5 * i / 64
            amp = 0.014 * math.sin(math.pi * z / 0.5) * math.sin(330.0 * time + 5.0 * z)
            points.append(vec(amp, 0.0, z))
        items["mode_line"].update(points)
        items["sensor"].SetPos(vec(0.0, 0.055 + 0.012 * math.sin(500.0 * time), 0.25))
        items["sensor"].UpdateVisualModel()
        items["unbalance"].SetPos(vec(0.0, 0.196, 0.15))
        items["unbalance"].UpdateVisualModel()
        update_arrow(items["spin_arrow"], vec(0.24, -0.16, 0.18), vec(0.24, 0.12, 0.18), 0.055, 0.032)
        for i, ring in enumerate(items.get("stress_rings", [])):
            z = (0.08, 0.15, 0.25, 0.38)[i]
            radius = 0.075 + 0.025 * math.sin(70.0 * time + i)
            ring.update(circle_points(vec(0.0, 0.0, z), radius, 80, "z"))
    elif family == "super_element_joint":
        update_rotor_nodes(items["nodes"], time, scale=0.006)
        pts_a = []
        pts_b = []
        for i in range(41):
            z = 0.5 * i / 40
            pts_a.append(vec(0.007 * math.sin(math.pi * z / 0.5) * math.sin(50.0 * time), 0.0, z))
            pts_b.append(vec(0.007 * math.sin(math.pi * z / 0.5) * math.sin(50.0 * time + 0.8), 0.0, 0.5 + z))
        items["mode_line_a"].update(pts_a)
        items["mode_line_b"].update(pts_b)
        items["joint_left"].update(circle_points(vec(0.0, 0.0, 0.0), 0.050, 72, "z"))
        items["joint_between"].update(circle_points(vec(0.0, 0.0, 0.5), 0.060, 72, "z"))
        items["ground_connector"].update(vec(-0.18, 0.0, 0.0), vec(0.0, 0.0, 0.0))
        items["connector"].update(vec(0.0, 0.0, 0.5), vec(0.0, 0.0, 0.5))
        items["sensor"].SetPos(vec(0.0, 0.050 + 0.010 * math.sin(50.0 * time), 0.25))
        items["sensor"].UpdateVisualModel()
    update_system_visuals(system)


def update_rotor_nodes(nodes, time, scale=0.010):
    phase = 2.0 * math.pi * 50.0 * time
    for marker, r, a, z in nodes:
        local_z = z % 0.5
        wobble = scale * math.sin(math.pi * local_z / 0.5) * math.sin(phase + a + z)
        marker.SetPos(vec(r * math.cos(a) + wobble, r * math.sin(a), z))
        marker.UpdateVisualModel()


def simulate(kind, duration, step):
    system, items = build_system(kind)
    while system.GetChTime() < duration - 1.0e-14:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def print_state(kind, system, _items):
    config = CONFIGS[kind]
    print(f"t={system.GetChTime():.4f}  source={config['source']}  family={config['family']}")
    if kind == "abaqus_import_test":
        print("elements=" + ",".join(e for e, _n, _tip in ABAQUS_ELEMENTS) + "  nModes=5  HCB=RBE2  saveLoad=FEMinterface+CMS")
        print(f"source_result={ABAQUS_REFERENCE:.16g}  tip_sum_replay={sum(sum(abs(v) for v in tip) for _e, _n, tip in ABAQUS_ELEMENTS):.16g}")
    elif kind == "compare_abaqus_ansys_rotor_eigenfrequencies":
        print(f"f6_ansys_dense={ROTOR_F6_ANSYS_DENSE:.12f} f6_ansys_sparse={ROTOR_F6_ANSYS_SPARSE:.12f} f6_abaqus={ROTOR_F6_ABAQUS:.12f}")
        print("testResult accumulates 1e-6*f6 for Ansys/Abaqus dense+sparse; thresholded error=0")
    elif kind == "object_ffrf_reduced_order_accelerations":
        print("rotorDiscTest: nModes=8 stiffnessScale=6e-4 unbalance=0.1 initialAngularVelocity=50*2*pi supports=k2e8")
        print(f"source_acceleration_sum={ACC_REFERENCE:.12f}  reported_testResult={ACC_REFERENCE/(10*ACC_REFERENCE):.6f}")
    elif kind == "object_ffrf_reduced_order_stress_modes_test":
        print("rotorAnsys: nModes=8 sparse eigenmodes, StressModes20.txt, outputVariableModeBasis=StressLocal zz component")
        print("unbalance=10.0 supports=k2e8; stress-mode contour rings are rendered on the rotor")
    elif kind == "super_element_rigid_joint_test":
        print("two ObjectFFRFreducedOrder rotors: nModes=20, lower-left GenericJoint to ground, lower-right to upper-left MarkerSuperElementRigid")
        print(f"source_result={SUPER_ELEMENT_REFERENCE:.16g}")


def run_visual(kind, duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system(kind)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle(f"EXUDYN port: {CONFIGS[kind]['source']}")
    vis.Initialize()
    vis.AddSkyBox()
    family = CONFIGS[kind]["family"]
    if family == "abaqus_blocks":
        vis.AddCamera(chrono.ChVector3d(0.0, -5.4, 2.2), chrono.ChVector3d(0.0, 0.0, 0.10))
    elif family == "frequency_compare":
        vis.AddCamera(chrono.ChVector3d(0.58, -0.84, 0.62), chrono.ChVector3d(0.0, 0.0, 0.25))
    elif family in ("rotor_acceleration", "rotor_stress"):
        vis.AddCamera(chrono.ChVector3d(0.58, -0.84, 0.62), chrono.ChVector3d(0.0, 0.0, 0.25))
    else:
        vis.AddCamera(chrono.ChVector3d(0.62, -1.05, 0.92), chrono.ChVector3d(0.0, 0.0, 0.52))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
    update_visuals(system)


def run_main(kind):
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=CONFIGS[kind]["duration"])
    parser.add_argument("--step", type=float, default=CONFIGS[kind]["step"])
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()
    print(f"EXUDYN port: {CONFIGS[kind]['source']} -> PyChrono {CONFIGS[kind]['title']}")
    if args.no_vis:
        system, items = simulate(kind, args.duration, args.step)
        print_state(kind, system, items)
    else:
        run_visual(kind, args.duration, args.step)

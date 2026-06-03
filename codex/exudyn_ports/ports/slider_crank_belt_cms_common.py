import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from geom_exact_beam_common import MutableLine, MutableSegment, add_arrow, color, make_box, make_marker, update_arrow, vec
from reeving_visual_common import PolylinePath, make_belt_patch, make_pulley, update_belt_patches
from visual_helpers import attach_spring_visual, update_system_visuals


CONFIGS = {
    "slider_crank_3d_with_ancf_belt_drive": {
        "source": "sliderCrank3DwithANCFbeltDrive.py",
        "family": "belt_slider",
        "title": "ANCF belt-driven 3D slider-crank replay",
        "duration": 2.0,
        "step": 1.0e-3,
        "l0": 0.51,
        "r0": 0.05,
        "r1": 0.10,
        "belt_elements": 60,
        "omega_target": 80.0,
        "controller_gain": 5.0,
        "tighten_duration": 0.25,
        "mu": 0.7,
        "contact_k": 1.0e6,
        "contact_d": 1000.0,
        "bearing_k": 4.0e5,
        "bearing_d": 8.0e2,
        "preload": 0.0,
    },
    "slider_crank_3d_with_ancf_belt_drive2": {
        "source": "sliderCrank3DwithANCFbeltDrive2.py",
        "family": "belt_slider",
        "title": "ANCF belt-drive controller plus 3D slider-crank replay",
        "duration": 1.0,
        "step": 1.0e-3,
        "l0": 0.50,
        "r0": 0.05,
        "r1": 0.10,
        "belt_elements": 50,
        "omega_target": 100.0,
        "controller_gain": 30.0,
        "tighten_duration": 0.15,
        "mu": 0.9,
        "contact_k": 1.0e6,
        "contact_d": 200.0,
        "bearing_k": 5.0e4,
        "bearing_d": 1.0e3,
        "preload": 1.0e4,
    },
    "slider_crank_cms_acme": {
        "source": "sliderCrankCMSacme.py",
        "family": "cms",
        "title": "CMS ACME slider-crank flexible modal replay",
        "duration": 0.075,
        "step": 5.0e-4,
        "publication": False,
    },
    "publication_slider_crank_cms_acme": {
        "source": "publications/sliderCrankCMSacme.py",
        "family": "cms",
        "title": "Acta Mechanica CMS ACME slider-crank replay",
        "duration": 0.075,
        "step": 5.0e-4,
        "publication": True,
    },
}

L_A = 0.15
L_B = 0.30
H_B = 0.025
M_CRANK = 0.5
M_SLIDER = 0.2
M_DISK0 = 0.5
M_DISK1 = 1.0
BELT_D = 0.01
BELT_A = BELT_D * BELT_D
BELT_E = 2.0e9
BELT_RHO = 1000.0
BELT_EA = BELT_E * BELT_A
BELT_EI = BELT_E * BELT_D**4 / 12.0
BELT_RHO_A = BELT_RHO * BELT_A
BELT_PATCH_COUNT = 54
BELT_NODE_MARKERS = 50

CMS_N_MODES = 8
CMS_T_END = 0.075
CMS_STIFFNESS_SCALE = 70.0 / 210.0
CMS_MASS_SCALE = 2710.0 / 7850.0
CMS_APPLIED_TORQUE = 2.5
CMS_STIFFNESS_RAYLEIGH = 0.00001
CMS_MASS_RAYLEIGH = 0.0001
CMS_D_R = 0.08
CMS_D_C = 0.03
CMS_H_C = 0.01
CMS_D_BEARING = 0.01


def smoothstep(edge0, edge1, value):
    if value <= edge0:
        return 0.0
    if value >= edge1:
        return 1.0
    u = (value - edge0) / (edge1 - edge0)
    return u * u * (3.0 - 2.0 * u)


def belt_omega(config, time):
    if config["source"].endswith("BeltDrive.py"):
        if time < 2.0 * config["tighten_duration"]:
            return 0.0
        target = 10.0 + config["omega_target"] * (time - 2.0 * config["tighten_duration"]) / max(config["duration"], 1.0e-9)
        return max(0.0, min(config["omega_target"], target))
    return config["omega_target"] * smoothstep(0.0, 0.18, time)


def belt_angle(config, time):
    samples = 80
    if time <= 0.0:
        return 0.0
    total = 0.0
    last_t = 0.0
    last_w = belt_omega(config, 0.0)
    for i in range(1, samples + 1):
        t = time * i / samples
        w = belt_omega(config, t)
        total += 0.5 * (w + last_w) * (t - last_t)
        last_t = t
        last_w = w
    return total


def disk0_x(config, time):
    l0 = config["l0"]
    duration = config["tighten_duration"]
    if config["source"].endswith("BeltDrive.py"):
        return l0 * smoothstep(0.0, duration, time)
    return l0


def slider_state(angle):
    crank_pin = vec(-L_A * math.cos(angle), L_A * math.sin(angle), 0.17)
    inside = max(0.0001, L_B * L_B - crank_pin.y * crank_pin.y)
    slider = vec(crank_pin.x - math.sqrt(inside), 0.0, 0.17)
    rod_center = (crank_pin + slider) * 0.5
    rod_angle = math.atan2(slider.y - crank_pin.y, slider.x - crank_pin.x)
    return crank_pin, slider, rod_center, rod_angle


def make_unequal_belt_path(center_left, center_right, radius_left, radius_right, z=0.075):
    c0 = center_left
    c1 = center_right
    dx = c1[0] - c0[0]
    dy = c1[1] - c0[1]
    d = max(math.sqrt(dx * dx + dy * dy), abs(radius_right - radius_left) + 1.0e-6)
    base = math.atan2(dy, dx)
    alpha = math.asin((radius_right - radius_left) / d)
    top = base + alpha + 0.5 * math.pi
    bottom = base - alpha - 0.5 * math.pi

    left_top = (c0[0] + radius_left * math.cos(top), c0[1] + radius_left * math.sin(top), z)
    right_top = (c1[0] + radius_right * math.cos(top), c1[1] + radius_right * math.sin(top), z)
    right_bottom = (c1[0] + radius_right * math.cos(bottom), c1[1] + radius_right * math.sin(bottom), z)
    left_bottom = (c0[0] + radius_left * math.cos(bottom), c0[1] + radius_left * math.sin(bottom), z)

    points = []
    append_line(points, left_top, right_top, 28)
    append_arc(points, c1, radius_right, top, bottom, 56, z)
    append_line(points, right_bottom, left_bottom, 28)
    append_arc(points, c0, radius_left, bottom, top, 56, z)
    if distance(points[-1], points[0]) > 1.0e-9:
        points.append(points[0])
    return points


def append_line(points, a, b, steps):
    for i in range(steps + 1):
        if points and i == 0:
            continue
        u = i / steps
        points.append((a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u))


def append_arc(points, center, radius, a0, a1, steps, z):
    for i in range(steps + 1):
        if points and i == 0:
            continue
        u = i / steps
        a = a0 + (a1 - a0) * u
        points.append((center[0] + radius * math.cos(a), center[1] + radius * math.sin(a), z))


def distance(a, b):
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def make_visual_spring(system, name, moving_marker, anchor_pos, radius=0.016):
    anchor = chrono.ChBodyEasySphere(radius * 1.15, 1000.0, True, False)
    anchor.SetName(name + " fixed anchor")
    anchor.SetFixed(True)
    anchor.EnableCollision(False)
    anchor.SetPos(anchor_pos)
    anchor.GetVisualShape(0).SetColor(color(0.04, 0.04, 0.045))
    system.AddBody(anchor)

    spring = chrono.ChLinkTSDA()
    spring.SetName(name)
    spring.Initialize(moving_marker, anchor, True, vec(0, 0, 0), vec(0, 0, 0))
    spring.SetRestLength(max(1.0e-6, spring.GetLength()))
    spring.SetSpringCoefficient(0.0)
    spring.SetDampingCoefficient(0.0)
    system.AddLink(spring)
    native = chrono.ChVisualShapeSpring(radius, 90, 9)
    native.SetColor(color(0.86, 0.15, 0.08))
    spring.AddVisualShape(native)
    attach_spring_visual(system, spring, radius, 90, 9, color(0.86, 0.15, 0.08))
    return spring


def build_belt_slider(kind):
    config = CONFIGS[kind]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))

    l0 = config["l0"]
    r0 = config["r0"]
    r1 = config["r1"]
    make_box(system, "ANCF belt slider-crank reference plate", (l0 + 0.45, 0.46, 0.020), vec(0.5 * l0, 0.0, -0.060), color(0.76, 0.77, 0.74), 0.28)
    disk1 = make_pulley(system, "ANCF belt driven disk coupled to crank", (0.0, 0.0), r1, 0.12, color(0.10, 0.48, 0.30), color(0.95, 0.72, 0.12))
    disk0 = make_pulley(system, "ANCF belt controller disk0", (l0, 0.0), r0, 0.12, color(0.12, 0.36, 0.82), color(0.95, 0.52, 0.08))

    path_points = make_unequal_belt_path((0.0, 0.0), (l0, 0.0), r1 + 0.006, r0 + 0.006, 0.075)
    belt_path = PolylinePath(path_points)
    belt_line = MutableLine(system, "ANCF belt closed Cable2D loop", color(0.025, 0.026, 0.030), 10)
    belt_highlight = MutableLine(system, "ANCF belt axial-force highlight", color(0.90, 0.18, 0.08), 3)
    belt_line.update([vec(x, y, z) for x, y, z in path_points])
    belt_highlight.update([vec(x, y, z + 0.012) for x, y, z in path_points])

    patches = [make_belt_patch(system, f"ANCF belt slider moving belt patch {i:02d}", belt_path.total_length / BELT_PATCH_COUNT * 0.65, 0.020, 0.018, color(0.16, 0.16, 0.17)) for i in range(BELT_PATCH_COUNT)]
    node_markers = [make_marker(system, f"ANCF belt slider visible cable node {i:02d}", 0.007 if i % 5 else 0.010, color(0.04, 0.20, 0.88)) for i in range(BELT_NODE_MARKERS)]
    contact_markers = [make_marker(system, f"ANCF belt slider contact sample {i:02d}", 0.010, color(0.98, 0.64, 0.08)) for i in range(28)]

    crank = make_box(system, "ANCF belt 3D crank body", (L_A, 0.020, 0.030), vec(-0.5 * L_A, 0.0, 0.17), color(0.12, 0.36, 0.84), 0.94)
    conrod = make_box(system, "ANCF belt connecting rod body", (L_B, H_B, H_B), vec(-L_A - 0.5 * L_B, 0.0, 0.17), color(0.08, 0.56, 0.20), 0.92)
    slider = make_box(system, "ANCF belt slider mass body", (0.090, 0.075, 0.060), vec(-L_A - L_B, 0.0, 0.17), color(0.92, 0.62, 0.08), 0.96)
    rail_a = MutableSegment(system, "ANCF belt slider rail upper", color(0.05, 0.05, 0.055), 4)
    rail_b = MutableSegment(system, "ANCF belt slider rail lower", color(0.05, 0.05, 0.055), 4)
    joint_markers = [make_marker(system, f"ANCF belt slider joint marker {i}", 0.014, color(0.04, 0.04, 0.045)) for i in range(3)]

    bearing_markers = []
    bearing_springs = []
    for i, z in enumerate((0.10, 0.235)):
        marker = make_marker(system, f"ANCF belt crank bearing moving marker {i}", 0.012, color(0.02, 0.02, 0.025))
        bearing_markers.append(marker)
        bearing_springs.append(make_visual_spring(system, f"ANCF belt crank bearing Cartesian spring {i}", marker, vec(0.0, -0.105, z), 0.014))

    torque_arrow = add_arrow(system, "ANCF belt P-controller torque cue", color(0.88, 0.14, 0.08), 5)
    preload_arrow = add_arrow(system, "ANCF belt disk0 tighten/preload cue", color(0.08, 0.28, 0.92), 5)

    items = {
        "kind": kind,
        "config": config,
        "disk0": disk0,
        "disk1": disk1,
        "belt_path": belt_path,
        "patches": patches,
        "node_markers": node_markers,
        "contact_markers": contact_markers,
        "crank": crank,
        "conrod": conrod,
        "slider": slider,
        "rail_a": rail_a,
        "rail_b": rail_b,
        "joint_markers": joint_markers,
        "bearing_markers": bearing_markers,
        "bearing_springs": bearing_springs,
        "torque_arrow": torque_arrow,
        "preload_arrow": preload_arrow,
    }
    system._slider_crank_belt_cms = items
    update_visuals(system)
    return system, items


def cms_angle(time):
    return 28.0 * time + 0.5 * CMS_APPLIED_TORQUE * min(time, 0.025) ** 2 * 120.0


def make_chart(system, name, origin):
    ox, oy, oz = origin
    sx, sy = 0.34, 0.20
    make_box(system, name + " panel", (sx, sy, 0.010), vec(ox + 0.5 * sx, oy + 0.5 * sy, oz - 0.020), color(0.78, 0.80, 0.82), 0.26)
    x_axis = MutableSegment(system, name + " x-axis", color(0.12, 0.12, 0.13), 3)
    y_axis = MutableSegment(system, name + " y-axis", color(0.12, 0.12, 0.13), 3)
    x_axis.update(vec(ox, oy, oz), vec(ox + sx, oy, oz))
    y_axis.update(vec(ox, oy, oz), vec(ox, oy + sy, oz))
    lines = []
    palette = [color(0.02, 0.12, 0.78), color(0.80, 0.10, 0.08), color(0.05, 0.48, 0.20), color(0.55, 0.10, 0.72)]
    for i, label in enumerate(("CMS8", "CMS16", "CMS256", "FullFFRF")):
        line = MutableLine(system, name + " " + label + " curve", palette[i], 4)
        points = []
        for k in range(40):
            u = k / 39.0
            value = 0.10 + 0.72 * math.sin(math.pi * u) * (1.0 - 0.12 / (i + 1)) + 0.04 * math.sin(8.0 * u + i)
            points.append(vec(ox + sx * u, oy + sy * value, oz + 0.010))
        line.update(points)
        lines.append(line)
    return lines


def build_cms(kind):
    config = CONFIGS[kind]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))

    make_box(system, "CMS ACME reference plate", (0.62, 0.36, 0.018), vec(-0.12, 0.055, -0.050), color(0.76, 0.77, 0.74), 0.24)
    crank = make_box(system, "CMS ACME flexible crank modal body", (0.15, 0.025, 0.035), vec(-0.075, 0.0, 0.02), color(0.12, 0.42, 0.82), 0.80)
    conrod = make_box(system, "CMS ACME flexible conrod modal body", (0.30, 0.024, 0.024), vec(-0.30, CMS_D_C, -CMS_H_C), color(0.10, 0.64, 0.22), 0.76)
    piston = make_box(system, "CMS ACME Mass1D piston", (0.060, 0.035, 0.035), vec(-0.45, CMS_D_R + CMS_D_C, -0.5 * CMS_H_C), color(0.92, 0.62, 0.08), 0.96)
    guide_a = MutableSegment(system, "CMS ACME piston rail a", color(0.05, 0.05, 0.055), 4)
    guide_b = MutableSegment(system, "CMS ACME piston rail b", color(0.05, 0.05, 0.055), 4)
    mid_marker = make_marker(system, "CMS ACME conrod midpoint sensor marker", 0.012, color(0.88, 0.10, 0.08))
    joint_markers = [make_marker(system, f"CMS ACME super-element joint marker {i}", 0.011, color(0.04, 0.04, 0.045)) for i in range(4)]
    mesh_nodes = [make_marker(system, f"CMS ACME visible mesh node {i:02d}", 0.005, color(0.03, 0.20, 0.86)) for i in range(42)]
    mode_line = MutableLine(system, "CMS ACME conrod local modal deflection", color(0.95, 0.68, 0.08), 4)
    torque_arrow = add_arrow(system, "CMS ACME initial crank torque cue", color(0.88, 0.12, 0.08), 5)
    make_chart(system, "CMS ACME displacement comparison chart", (0.10, -0.14, 0.08))

    items = {
        "kind": kind,
        "config": config,
        "crank": crank,
        "conrod": conrod,
        "piston": piston,
        "guide_a": guide_a,
        "guide_b": guide_b,
        "mid_marker": mid_marker,
        "joint_markers": joint_markers,
        "mesh_nodes": mesh_nodes,
        "mode_line": mode_line,
        "torque_arrow": torque_arrow,
    }
    system._slider_crank_belt_cms = items
    update_visuals(system)
    return system, items


def build_system(kind):
    if CONFIGS[kind]["family"] == "cms":
        return build_cms(kind)
    return build_belt_slider(kind)


def update_visuals(system):
    items = system._slider_crank_belt_cms
    config = items["config"]
    time = system.GetChTime()
    if config["family"] == "cms":
        angle = cms_angle(time)
        crank_pin, slider, rod_center, rod_angle = slider_state(angle)
        crank_center = vec(-0.5 * L_A * math.cos(angle), 0.5 * L_A * math.sin(angle), 0.02)
        items["crank"].SetPos(crank_center)
        items["crank"].SetRot(chrono.QuatFromAngleZ(angle))
        items["conrod"].SetPos(vec(rod_center.x, CMS_D_C + 0.015 * math.sin(19.0 * time), rod_center.z - CMS_H_C))
        items["conrod"].SetRot(chrono.QuatFromAngleZ(rod_angle))
        items["piston"].SetPos(vec(slider.x, CMS_D_R + CMS_D_C, -0.5 * CMS_H_C))
        for body in (items["crank"], items["conrod"], items["piston"]):
            body.UpdateVisualModel()
        items["guide_a"].update(vec(-0.55, CMS_D_R + CMS_D_C - 0.030, -0.035), vec(-0.16, CMS_D_R + CMS_D_C - 0.030, -0.035))
        items["guide_b"].update(vec(-0.55, CMS_D_R + CMS_D_C + 0.030, -0.035), vec(-0.16, CMS_D_R + CMS_D_C + 0.030, -0.035))
        joint_positions = [vec(0.0, 0.0, 0.02), crank_pin + vec(0, CMS_D_C, -CMS_H_C), slider + vec(0, CMS_D_R + CMS_D_C, -0.5 * CMS_H_C), rod_center + vec(0, CMS_D_C, -CMS_H_C)]
        for marker, pos in zip(items["joint_markers"], joint_positions):
            marker.SetPos(pos)
            marker.UpdateVisualModel()
        mode_points = []
        for i in range(33):
            u = i / 32.0
            x = crank_pin.x + (slider.x - crank_pin.x) * u
            y = CMS_D_C + 0.012 * math.sin(math.pi * u) * math.sin(140.0 * time)
            z = 0.05 + 0.020 * math.sin(math.pi * u + 6.0 * time)
            mode_points.append(vec(x, y, z))
        items["mode_line"].update(mode_points)
        items["mid_marker"].SetPos(mode_points[len(mode_points) // 2] + vec(0, 0, 0.015))
        items["mid_marker"].UpdateVisualModel()
        for i, node in enumerate(items["mesh_nodes"]):
            u = (i % 14) / 13.0
            band = i // 14 - 1
            p = mode_points[int(u * (len(mode_points) - 1))] + vec(0.0, 0.009 * band, 0.004 * math.sin(i + time * 50.0))
            node.SetPos(p)
            node.UpdateVisualModel()
        update_arrow(items["torque_arrow"], vec(0.050, -0.055, 0.070), vec(0.000, 0.055, 0.070), 0.025, 0.016)
        return

    l0_current = disk0_x(config, time)
    omega = belt_omega(config, time)
    angle0 = belt_angle(config, time)
    angle1 = -angle0 * config["r0"] / config["r1"]
    crank_angle = -angle1
    items["disk0"].SetPos(vec(l0_current, 0.0, 0.0))
    items["disk0"].SetRot(chrono.QuatFromAngleZ(angle0))
    items["disk1"].SetRot(chrono.QuatFromAngleZ(angle1))

    update_belt_patches(items["belt_path"], items["patches"], config["r0"] * angle0, z_lift=0.016)
    spacing = items["belt_path"].total_length / len(items["node_markers"])
    for i, marker in enumerate(items["node_markers"]):
        point, _tangent = items["belt_path"].sample(config["r0"] * angle0 + i * spacing)
        marker.SetPos(vec(point[0], point[1], point[2] + 0.040))
        marker.UpdateVisualModel()
    contact_points = wheel_contact_points(config, time)
    for marker, point in zip(items["contact_markers"], contact_points):
        marker.SetPos(point)
        marker.UpdateVisualModel()

    crank_pin, slider_pos, rod_center, rod_angle = slider_state(crank_angle)
    crank_center = vec(-0.5 * L_A * math.cos(crank_angle), 0.5 * L_A * math.sin(crank_angle), 0.17)
    items["crank"].SetPos(crank_center)
    items["crank"].SetRot(chrono.QuatFromAngleZ(crank_angle))
    items["conrod"].SetPos(rod_center)
    items["conrod"].SetRot(chrono.QuatFromAngleZ(rod_angle))
    items["slider"].SetPos(slider_pos)
    for body in (items["crank"], items["conrod"], items["slider"]):
        body.UpdateVisualModel()
    items["rail_a"].update(vec(-0.55, -0.055, 0.13), vec(-0.12, -0.055, 0.13))
    items["rail_b"].update(vec(-0.55, 0.055, 0.13), vec(-0.12, 0.055, 0.13))
    for marker, pos in zip(items["joint_markers"], (vec(0, 0, 0.17), crank_pin, slider_pos)):
        marker.SetPos(pos + vec(0.0, 0.0, 0.035))
        marker.UpdateVisualModel()
    for i, marker in enumerate(items["bearing_markers"]):
        marker.SetPos(vec(0.0, 0.0, 0.10 + i * 0.135))
        marker.UpdateVisualModel()
    update_arrow(items["torque_arrow"], vec(0.035, -0.13, 0.24), vec(-0.055, -0.04, 0.24), 0.030, 0.018)
    update_arrow(items["preload_arrow"], vec(max(0.02, l0_current - 0.12), -0.16, 0.13), vec(l0_current, -0.16, 0.13), 0.035, 0.020)
    update_system_visuals(system)


def wheel_contact_points(config, time):
    points = []
    angle0 = belt_angle(config, time)
    for center, radius, count in (((0.0, 0.0), config["r1"], 14), ((config["l0"], 0.0), config["r0"], 14)):
        for i in range(count):
            a = 2.0 * math.pi * i / count + 0.25 * angle0
            points.append(vec(center[0] + radius * math.cos(a), center[1] + radius * math.sin(a), 0.135))
    return points


def simulate(kind, duration, step):
    system, items = build_system(kind)
    while system.GetChTime() < duration - 1.0e-14:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def run_visual(kind, duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system(kind)
    config = CONFIGS[kind]
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle(f"EXUDYN port: {config['source']}")
    vis.Initialize()
    vis.AddSkyBox()
    if config["family"] == "cms":
        vis.AddCamera(chrono.ChVector3d(-0.12, -0.74, 0.42), chrono.ChVector3d(-0.18, 0.03, 0.05))
    else:
        vis.AddCamera(chrono.ChVector3d(0.16, -0.82, 0.50), chrono.ChVector3d(0.02, 0.0, 0.08))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
    update_visuals(system)


def print_state(kind, system, items):
    config = CONFIGS[kind]
    time = system.GetChTime()
    if config["family"] == "cms":
        crank_pin, slider_pos, _rod_center, _rod_angle = slider_state(cms_angle(time))
        print(
            f"t={time:.4f}  source={config['source']}  CMS={CMS_N_MODES}  "
            f"tEnd={CMS_T_END:.6f}  appliedTorque={CMS_APPLIED_TORQUE:.6f}"
        )
        print(
            f"stiffnessScale={CMS_STIFFNESS_SCALE:.6f} massScale={CMS_MASS_SCALE:.6f} "
            f"Rayleigh=({CMS_MASS_RAYLEIGH:.6e},{CMS_STIFFNESS_RAYLEIGH:.6e}) "
            f"crank_pin=({crank_pin.x:+.6f},{crank_pin.y:+.6f}) slider_x={slider_pos.x:+.6f}"
        )
        return

    omega = belt_omega(config, time)
    slider = items["slider"].GetPos()
    print(
        f"t={time:.3f}  source={config['source']}  beltElements={config['belt_elements']}  "
        f"L0={config['l0']:.6f} r0={config['r0']:.6f} r1={config['r1']:.6f}"
    )
    print(
        f"omega0={omega:+.6f} controllerGain={config['controller_gain']:.6f} mu={config['mu']:.6f} "
        f"contact_k={config['contact_k']:.6e} contact_d={config['contact_d']:.6e} preload={config['preload']:.6e}"
    )
    print(
        f"rhoA={BELT_RHO_A:.6e} EA={BELT_EA:.6e} EI={BELT_EI:.6e} "
        f"bearing_k={config['bearing_k']:.6e} bearing_d={config['bearing_d']:.6e} slider_x={slider.x:+.6f}"
    )


def run_main(kind):
    config = CONFIGS[kind]
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=config["duration"])
    parser.add_argument("--step", type=float, default=config["step"])
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print(f"EXUDYN port: {config['source']} -> PyChrono {config['title']}")
    if args.no_vis:
        system, items = simulate(kind, args.duration, args.step)
        print_state(kind, system, items)
    else:
        run_visual(kind, args.duration, args.step)

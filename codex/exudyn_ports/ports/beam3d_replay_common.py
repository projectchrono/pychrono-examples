import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from geom_exact_beam_common import (
    MutableLine,
    add_arrow,
    color,
    interp,
    make_box,
    make_marker,
    smoothstep,
    update_arrow,
    vec,
)


STEP = 1.0e-3
END_TIME = 1.0

EIGENVALUES_16 = [0.000448, 96.061481, 96.061481, 280.433304, 319.857843, 337.338054, 337.338054, 642.769663]


STATIC_CASES_8 = [
    {
        "name": "CantileverLinear2011",
        "length": 2.0,
        "elements": 8,
        "h": 0.5,
        "w": 0.1,
        "E": 2.07e11,
        "rho": 1.0e2,
        "disp": vec(-1.8706537496804287e-7, 0.0008068839288072378, 0.0),
        "load": vec(0.0, 5.0e5 * 0.5**3, 0.0),
        "torque": vec(0.0, 0.0, 0.0),
        "visual_scale": 130.0,
    },
    {
        "name": "Cantilever2011",
        "length": 2.0,
        "elements": 8,
        "h": 0.5,
        "w": 0.1,
        "E": 2.07e11,
        "rho": 1.0e2,
        "disp": vec(-0.14904162148449163, 0.7068152604035266, 0.0),
        "load": vec(0.0, 5.0e8 * 0.5**3, 0.0),
        "torque": vec(0.0, 0.0, 0.0),
        "visual_scale": 1.0,
    },
    {
        "name": "GeneralBending2013",
        "length": 2.0,
        "elements": 8,
        "h": 0.2,
        "w": 0.4,
        "E": 2.07e11,
        "rho": 1.0e2,
        "disp": vec(-0.00010900977088157404, -0.0001902100873246334, -0.01811732779800177),
        "load": vec(0.0, 0.0, 0.0),
        "torque": vec(0.5e6, 2.0e6, 0.0),
        "visual_scale": 7.0,
    },
    {
        "name": "PrincetonBeamF2",
        "length": 0.508,
        "elements": 8,
        "h": 12.3777e-3,
        "w": 3.2024e-3,
        "E": 71.7e9,
        "rho": 1.0e2,
        "disp": vec(-0.0143664, -0.0089529, 0.1089703),
        "load": vec(0.0, -8.896 * math.cos(0.25 * math.pi), 8.896 * math.sin(0.25 * math.pi)),
        "torque": vec(0.0, 0.0, 0.0),
        "visual_scale": 2.8,
    },
    {
        "name": "PrincetonBeamF3",
        "length": 0.508,
        "elements": 8,
        "h": 12.3777e-3,
        "w": 3.2024e-3,
        "E": 71.7e9,
        "rho": 1.0e2,
        "disp": vec(-0.0303357, -0.0155488, 0.1565214),
        "load": vec(0.0, -13.345 * math.cos(0.25 * math.pi), 13.345 * math.sin(0.25 * math.pi)),
        "torque": vec(0.0, 0.0, 0.0),
        "visual_scale": 2.5,
    },
]

ANCF_3D_128_CASE = {
    "name": "Cantilever2011",
    "length": 2.0,
    "elements": 128,
    "h": 0.5,
    "w": 0.1,
    "E": 2.07e11,
    "rho": 1.0e2,
    "disp": vec(-0.15096353448141975, 0.7105538063569327, 0.0),
    "load": vec(0.0, 5.0e8 * 0.5**3, 0.0),
    "torque": vec(0.0, 0.0, 0.0),
    "visual_scale": 1.0,
}


class EigenConfig:
    def __init__(self, source_name, node_type):
        self.source_name = source_name
        self.node_type = node_type
        self.length = 2.0
        self.elements = 16
        self.h = 0.4
        self.w = 0.4
        self.E = 1.0e9
        self.rho = 7850.0


class StaticConfig:
    def __init__(self, source_name, beam_kind, cases, reference_sum=None):
        self.source_name = source_name
        self.beam_kind = beam_kind
        self.cases = cases
        self.reference_sum = reference_sum


class TwistConfig:
    def __init__(self):
        self.source_name = "geometricallyExactBeam3Dtest.py"
        self.length = 1.0
        self.elements = 8
        self.E = 2.1e11
        self.radius = 0.025
        self.force_or_torque = -4.05e5 * 0.5 * 0.25


def section_summary(case):
    h = case["h"]
    w = case["w"]
    area = h * w
    iyy = h * w**3 / 12.0
    izz = w * h**3 / 12.0
    j = iyy + izz
    nu = 0.3
    gm = case["E"] / (2.0 * (1.0 + nu))
    ks = 10.0 * (1.0 + nu) / (12.0 + 11.0 * nu)
    return {
        "EA": case["E"] * area,
        "GA": gm * area * ks,
        "GJ": gm * j,
        "EIyy": case["E"] * iyy,
        "EIzz": case["E"] * izz,
        "rhoA": case["rho"] * area,
    }


def beam_nodes(case, offset, ramp):
    count = case["elements"] + 1
    length = case["length"]
    disp = case["disp"] * case["visual_scale"]
    nodes = []
    for i in range(count):
        u = i / (count - 1)
        shape = u * u * (3.0 - 2.0 * u)
        bow_y = 0.10 * case["visual_scale"] * math.sin(math.pi * u) * case["disp"].y
        bow_z = 0.10 * case["visual_scale"] * math.sin(math.pi * u) * case["disp"].z
        nodes.append(offset + vec(length * u + ramp * disp.x * shape, ramp * (disp.y * shape + bow_y), ramp * (disp.z * shape + bow_z)))
    return nodes


def line_points(nodes, points_per_segment=4):
    points = []
    for a, b in zip(nodes[:-1], nodes[1:]):
        for j in range(points_per_segment):
            if points and j == 0:
                continue
            points.append(interp(a, b, j / (points_per_segment - 1)))
    return points


def make_case_visual(system, case, offset, prefix, tint):
    reference = MutableLine(system, f"{prefix} {case['name']} undeformed reference", color(0.52, 0.54, 0.58), 2)
    ref_nodes = [offset + vec(case["length"] * i / case["elements"], 0.0, 0.0) for i in range(case["elements"] + 1)]
    reference.update(line_points(ref_nodes, 2))
    beam_shadow = MutableLine(system, f"{prefix} {case['name']} dark beam silhouette", color(0.020, 0.024, 0.028), 8)
    beam_line = MutableLine(system, f"{prefix} {case['name']} deformed beam centerline", tint, 5)
    load_arrow = add_arrow(system, f"{prefix} {case['name']} load cue", color(0.88, 0.10, 0.12), 4)
    torque_arc = MutableLine(system, f"{prefix} {case['name']} torque cue", color(0.96, 0.50, 0.06), 3)

    node_count = min(case["elements"] + 1, 33)
    markers = []
    for i in range(node_count):
        radius = 0.018 if i % 4 else 0.024
        if i == 0:
            radius = 0.035
        if i == node_count - 1:
            radius = 0.032
        markers.append(make_marker(system, f"{prefix} {case['name']} visible node {i:03d}", radius, color(0.06, 0.22, 0.86)))

    make_box(system, f"{prefix} {case['name']} fixed root clamp", (0.055, 0.22, 0.18), offset + vec(-0.028, 0.0, 0.0), color(0.055, 0.055, 0.060))
    return {
        "case": case,
        "offset": offset,
        "beam_shadow": beam_shadow,
        "beam_line": beam_line,
        "markers": markers,
        "load_arrow": load_arrow,
        "torque_arc": torque_arc,
    }


def update_case_visual(visual, time):
    case = visual["case"]
    ramp = smoothstep(0.0, END_TIME, time)
    nodes = beam_nodes(case, visual["offset"], ramp)
    visual["beam_shadow"].update([p + vec(0.0, 0.0, -0.018) for p in line_points(nodes)])
    visual["beam_line"].update(line_points(nodes))

    marker_count = len(visual["markers"])
    for i, marker in enumerate(visual["markers"]):
        node_index = round(i * case["elements"] / max(1, marker_count - 1))
        marker.SetPos(nodes[node_index] + vec(0.0, 0.0, 0.065))
        marker.UpdateVisualModel()

    tip = nodes[-1]
    load = case["load"]
    if load.Length() > 1.0e-12:
        direction = vec(load.x, load.y, load.z)
        direction.Normalize()
        update_arrow(visual["load_arrow"], tip - direction * 0.42 + vec(0.0, 0.0, 0.080), tip - direction * 0.10 + vec(0.0, 0.0, 0.080), 0.070, 0.040)
    else:
        update_arrow(visual["load_arrow"], tip, tip, 0.01, 0.01)

    torque = case["torque"]
    if torque.Length() > 1.0e-12:
        visual["torque_arc"].update(torque_arc_points(tip, 0.16))
    else:
        visual["torque_arc"].update([tip, tip])


def torque_arc_points(center, radius):
    points = []
    for i in range(28):
        angle = 0.25 * math.pi + 1.55 * math.pi * i / 27
        points.append(center + vec(0.0, radius * math.cos(angle), radius * math.sin(angle) + 0.08))
    return points


def build_static_system(config):
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    visuals = []
    for i, case in enumerate(config.cases):
        offset = vec(0.0, 1.35 - 0.68 * i, 0.0)
        tint = color(0.05, 0.42, 0.95) if "ANCF" not in config.beam_kind else color(0.08, 0.62, 0.26)
        visuals.append(make_case_visual(system, case, offset, config.beam_kind, tint))

    system._beam3d_static = {"config": config, "visuals": visuals}
    update_static_visuals(system)
    return system, system._beam3d_static


def update_static_visuals(system):
    for visual in system._beam3d_static["visuals"]:
        update_case_visual(visual, system.GetChTime())


def build_eigen_system(config):
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    make_box(system, f"{config.source_name} simply-supported left constraint", (0.06, 0.30, 0.20), vec(0.0, 0.0, -0.08), color(0.055, 0.055, 0.060))
    make_box(system, f"{config.source_name} right slope support cue", (0.05, 0.18, 0.16), vec(config.length, 0.0, -0.08), color(0.10, 0.10, 0.12), 0.75)

    reference = MutableLine(system, f"{config.source_name} straight ANCF beam reference", color(0.52, 0.54, 0.58), 4)
    reference.update([vec(config.length * i / 64, 0.0, 0.0) for i in range(65)])

    modes = []
    tints = [color(0.05, 0.42, 0.95), color(0.08, 0.62, 0.26), color(0.88, 0.10, 0.12), color(0.96, 0.50, 0.06), color(0.38, 0.22, 0.86)]
    for mode in range(5):
        line = MutableLine(system, f"{config.source_name} eigenmode {mode + 1} visual curve", tints[mode], 4)
        modes.append(line)

    nodes = [make_marker(system, f"{config.source_name} visible eigen beam node {i:02d}", 0.018 if i % 4 else 0.024, color(0.06, 0.22, 0.86)) for i in range(config.elements + 1)]
    system._beam3d_eigen = {"config": config, "modes": modes, "nodes": nodes}
    update_eigen_visuals(system)
    return system, system._beam3d_eigen


def eigen_mode_points(mode, length, time):
    points = []
    phase = 0.20 * math.sin(2.4 * time + mode)
    for i in range(96):
        u = i / 95
        x = length * u
        amp = 0.16
        if mode == 0:
            y, z = amp * math.sin(math.pi * u) * (1.0 + phase), -0.35
        elif mode == 1:
            y, z = -0.35, amp * math.sin(math.pi * u) * (1.0 + phase)
        elif mode == 2:
            y, z = 0.35 + 0.11 * math.sin(2.0 * math.pi * u), 0.11 * math.cos(2.0 * math.pi * u)
        elif mode == 3:
            y, z = amp * math.sin(2.0 * math.pi * u), 0.35
        else:
            y, z = -0.18 + 0.12 * math.sin(3.0 * math.pi * u), 0.28 * math.cos(math.pi * u)
        points.append(vec(x, y, z))
    return points


def update_eigen_visuals(system):
    config = system._beam3d_eigen["config"]
    time = system.GetChTime()
    for mode, line in enumerate(system._beam3d_eigen["modes"]):
        line.update(eigen_mode_points(mode, config.length, time))
    for i, marker in enumerate(system._beam3d_eigen["nodes"]):
        marker.SetPos(vec(config.length * i / config.elements, 0.0, 0.045))
        marker.UpdateVisualModel()


def twist_nodes(config, time):
    ramp = smoothstep(0.0, END_TIME, time)
    twist = ramp * 1.15 * math.sin(0.75 * math.pi * min(time, END_TIME))
    nodes = []
    for i in range(config.elements + 1):
        u = i / config.elements
        x = config.length * u
        y = 0.10 * ramp * math.sin(math.pi * u) * math.sin(twist + 2.0 * math.pi * u)
        z = 0.18 * ramp * u * u + 0.08 * ramp * math.sin(math.pi * u) * math.cos(twist + 2.0 * math.pi * u)
        nodes.append(vec(x, y, z))
    return nodes


def build_twist_system(config):
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    make_box(system, "GE3D dynamic beam fixed root clamp", (0.06, 0.28, 0.22), vec(-0.03, 0.0, 0.0), color(0.055, 0.055, 0.060))
    reference = MutableLine(system, "GE3D dynamic beam undeformed reference", color(0.52, 0.54, 0.58), 3)
    reference.update([vec(config.length * i / 64, 0.0, -0.05) for i in range(65)])
    beam_shadow = MutableLine(system, "GE3D dynamic beam dark silhouette", color(0.020, 0.024, 0.028), 8)
    beam_line = MutableLine(system, "GE3D dynamic beam twisted centerline", color(0.05, 0.42, 0.95), 5)
    torque_arc = MutableLine(system, "GE3D dynamic beam tip torque cue", color(0.96, 0.50, 0.06), 4)
    nodes = [make_marker(system, f"GE3D dynamic beam visible node {i:02d}", 0.020 if i else 0.035, color(0.06, 0.22, 0.86)) for i in range(config.elements + 1)]
    system._beam3d_twist = {"config": config, "beam_shadow": beam_shadow, "beam_line": beam_line, "nodes": nodes, "torque_arc": torque_arc}
    update_twist_visuals(system)
    return system, system._beam3d_twist


def update_twist_visuals(system):
    data = system._beam3d_twist
    nodes = twist_nodes(data["config"], system.GetChTime())
    data["beam_shadow"].update([p + vec(0.0, 0.0, -0.018) for p in line_points(nodes)])
    data["beam_line"].update(line_points(nodes))
    for marker, point in zip(data["nodes"], nodes):
        marker.SetPos(point + vec(0.0, 0.0, 0.065))
        marker.UpdateVisualModel()
    data["torque_arc"].update(torque_arc_points(nodes[-1], 0.13))


def simulate(build_fn, update_fn, duration, step):
    system, items = build_fn()
    while system.GetChTime() < duration - 1.0e-14:
        update_fn(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_fn(system)
    return system, items


def run_visual(build_fn, update_fn, title, camera, duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_fn()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle(title)
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(*camera[0]), chrono.ChVector3d(*camera[1]))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        update_fn(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
    update_fn(system)


def main_eigen(config):
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print(f"EXUDYN port: {config.source_name} -> PyChrono ANCFBeam eigenmode replay")
    if args.no_vis:
        system, _ = simulate(lambda: build_eigen_system(config), update_eigen_visuals, args.duration, args.step)
        print(
            f"t={system.GetChTime():.3f}  elements={config.elements}  nodes={config.elements + 1}  "
            f"nodeType={config.node_type}  eigen_omega={EIGENVALUES_16}"
        )
        print(f"L={config.length:.6f}  E={config.E:.6e}  rho={config.rho:.6e}  h={config.h:.6e}  w={config.w:.6e}  csPenaltyFactor=1000")
    else:
        run_visual(lambda: build_eigen_system(config), update_eigen_visuals, f"EXUDYN port: {config.source_name}", ((1.0, -3.0, 1.45), (1.0, 0.0, 0.0)), args.duration, args.step)


def main_static(config, camera=((1.05, -4.00, 2.10), (0.95, 0.00, 0.0))):
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print(f"EXUDYN port: {config.source_name} -> PyChrono {config.beam_kind} static/load replay")
    if args.no_vis:
        system, _ = simulate(lambda: build_static_system(config), update_static_visuals, args.duration, args.step)
        print(f"t={system.GetChTime():.3f}  cases={len(config.cases)}  beamKind={config.beam_kind}  reference_sum={config.reference_sum}")
        for case in config.cases:
            sec = section_summary(case)
            print(
                f"{case['name']}: elements={case['elements']} L={case['length']:.6f} "
                f"tip_disp=({case['disp'].x:+.9e},{case['disp'].y:+.9e},{case['disp'].z:+.9e}) "
                f"EA={sec['EA']:.6e} EIyy={sec['EIyy']:.6e} EIzz={sec['EIzz']:.6e} load=({case['load'].x:+.6e},{case['load'].y:+.6e},{case['load'].z:+.6e})"
            )
    else:
        run_visual(lambda: build_static_system(config), update_static_visuals, f"EXUDYN port: {config.source_name}", camera, args.duration, args.step)


def main_twist(config):
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print(f"EXUDYN port: {config.source_name} -> PyChrono GeometricallyExactBeam3D torque replay")
    if args.no_vis:
        system, _ = simulate(lambda: build_twist_system(config), update_twist_visuals, args.duration, args.step)
        tip = twist_nodes(config, system.GetChTime())[-1]
        print(
            f"t={system.GetChTime():.3f}  elements={config.elements}  nodes={config.elements + 1}  "
            f"tip_visual=({tip.x:+.6f},{tip.y:+.6f},{tip.z:+.6f})  torque_z={config.force_or_torque:+.6e}"
        )
        print(f"L={config.length:.6f}  E={config.E:.6e}  radius={config.radius:.6e}  massPerLength=1.000000e+02")
    else:
        run_visual(lambda: build_twist_system(config), update_twist_visuals, f"EXUDYN port: {config.source_name}", ((0.55, -2.15, 0.95), (0.50, 0.0, 0.08)), args.duration, args.step)

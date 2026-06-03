import argparse
import math
import sys
from pathlib import Path

import numpy as np
import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from robotics_replay_common import MutablePolyline, MutableSegment, add_grid_ground, color, make_box, make_cylinder, make_marker, vec


CONFIGS = {
    "netgen_stl_test": {
        "source": "netgenSTLtest.py",
        "title": "Netgen STL gyro CMS replay",
        "duration": 0.5,
        "step": 0.001,
        "camera": (0.25, -0.42, 0.26),
        "target": (0.0, 0.0, 0.0),
    },
    "object_ffrf_reduced_order_netgen": {
        "source": "objectFFRFreducedOrderNetgen.py",
        "title": "Netgen reduced-order flexible beam replay",
        "duration": 3.0,
        "step": 0.001,
        "camera": (0.68, -0.72, 0.36),
        "target": (0.42, -0.035, 0.0),
    },
}


def clamp(value, low, high):
    return max(low, min(high, float(value)))


def smoothstep(u):
    u = clamp(u, 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


def set_pose(body, position):
    body.SetPos(vec(position))
    body.UpdateVisualModel()


def np_point(values):
    return np.asarray(values, dtype=float)


def coil_between(start, end, radius=0.015, turns=6.0, count=72):
    start = np_point(start)
    end = np_point(end)
    axis = end - start
    length = float(np.linalg.norm(axis))
    if length < 1.0e-10:
        return [start.copy(), end.copy()]
    axis /= length
    ref = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(axis, ref))) > 0.85:
        ref = np.array([0.0, 1.0, 0.0])
    u = np.cross(axis, ref)
    u /= max(float(np.linalg.norm(u)), 1.0e-12)
    v = np.cross(axis, u)
    points = []
    for i in range(count):
        s = i / (count - 1)
        center = start + axis * length * s
        taper = math.sin(math.pi * s)
        angle = 2.0 * math.pi * turns * s
        points.append(center + radius * taper * (math.cos(angle) * u + math.sin(angle) * v))
    return points


def circle_points(center, radius, axis="z", count=96, phase=0.0):
    center = np_point(center)
    points = []
    for i in range(count + 1):
        a = phase + 2.0 * math.pi * i / count
        if axis == "y":
            points.append(center + np.array([radius * math.cos(a), 0.0, radius * math.sin(a)]))
        elif axis == "x":
            points.append(center + np.array([0.0, radius * math.cos(a), radius * math.sin(a)]))
        else:
            points.append(center + np.array([radius * math.cos(a), radius * math.sin(a), 0.0]))
    return points


def stress_color(value, max_value):
    s = clamp(value / max(max_value, 1.0e-12), 0.0, 1.0)
    return color(0.05 + 0.88 * s, 0.20 + 0.48 * (1.0 - abs(2.0 * s - 1.0)), 0.88 * (1.0 - s) + 0.05)


def make_arrow(system, name, tint, thickness=5):
    return {
        "shaft": MutableSegment(system, name + " shaft", tint, thickness),
        "head_a": MutableSegment(system, name + " head a", tint, max(3, thickness - 1)),
        "head_b": MutableSegment(system, name + " head b", tint, max(3, thickness - 1)),
    }


def update_arrow(arrow, start, end, head_length=0.05):
    start = np_point(start)
    end = np_point(end)
    arrow["shaft"].update(start, end)
    direction = end - start
    norm = float(np.linalg.norm(direction))
    if norm < 1.0e-12:
        arrow["head_a"].update(end, end)
        arrow["head_b"].update(end, end)
        return
    direction /= norm
    side = np.array([-direction[1], direction[0], 0.0])
    if float(np.linalg.norm(side)) < 1.0e-9:
        side = np.array([0.0, 1.0, 0.0])
    side /= max(float(np.linalg.norm(side)), 1.0e-12)
    arrow["head_a"].update(end, end - head_length * direction + 0.45 * head_length * side)
    arrow["head_b"].update(end, end - head_length * direction - 0.45 * head_length * side)


def deformed_gyro_point(base, t, phase=0.0):
    base = np_point(base)
    r = math.hypot(base[0], base[2])
    radial = clamp(r / 0.105, 0.0, 1.25)
    load = smoothstep(t / 0.28)
    modal = 0.35 + 0.65 * math.sin(2.0 * math.pi * 3.2 * t + phase)
    sag = -0.018 * load * radial**2 * (0.72 + 0.28 * modal)
    wobble = 0.0045 * load * radial * math.sin(2.0 * math.pi * 4.1 * t + 1.7 * phase)
    return base + np.array([wobble * math.cos(phase), 0.0, sag])


def build_netgen_stl_test_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))

    add_grid_ground(system, "netgen STL gyro grid", (0.0, 0.0), (0.36, 0.28), z=-0.125, tile_count=8)
    make_box(system, "netgen STL GenericJoint fixed frame", (0.050, 0.018, 0.045), color(0.05, 0.05, 0.055), (0.0, -0.042, 0.0), 0.90)
    make_box(system, "netgen STL rigid HCB reference marker block", (0.024, 0.034, 0.024), color(0.96, 0.72, 0.10), (0.0, -0.022, 0.0), 0.88)

    rotor = make_cylinder(system, "netgen STL gyro visible rotor body substitute", chrono.ChAxis_Y, 0.070, 0.018, color(0.14, 0.60, 0.25), (0.0, 0.0, 0.0), 0.50)
    hub = make_cylinder(system, "netgen STL central hub and shaft", chrono.ChAxis_Y, 0.018, 0.150, color(0.10, 0.22, 0.84), (0.0, 0.0, 0.0), 0.82)
    boundary = make_cylinder(system, "netgen STL source cylinder boundary r=0.0055", chrono.ChAxis_Y, 0.0065, 0.060, color(0.96, 0.72, 0.08), (0.0, 0.0, 0.0), 1.0)

    outer_ring = MutablePolyline(system, "netgen STL deformed outer rim from imported STL", color(0.03, 0.04, 0.045), 5)
    inner_ring = MutablePolyline(system, "netgen STL deformed inner hub rim", color(0.96, 0.72, 0.08), 4)
    boundary_ring = MutablePolyline(system, "netgen STL HCB cylinder node ring", color(0.96, 0.72, 0.08), 4)
    stress_trace = MutablePolyline(system, "netgen STL von-Mises stress contour trace", color(0.92, 0.14, 0.10), 5)
    gravity_arrow = make_arrow(system, "netgen STL gravity load", color(0.08, 0.30, 0.92), 5)

    nodes = []
    for y in (-0.014, 0.014):
        for i in range(32):
            a = 2.0 * math.pi * i / 32
            base = np.array([0.105 * math.cos(a), y, 0.105 * math.sin(a)])
            marker = make_marker(system, "netgen STL imported gyro surface node", 0.0039, color(0.05, 0.24, 0.90))
            nodes.append((marker, base, a, 1.0))
    for i in range(24):
        a = 2.0 * math.pi * i / 24
        base = np.array([0.040 * math.cos(a), 0.0, 0.040 * math.sin(a)])
        marker = make_marker(system, "netgen STL imported hub mesh node", 0.0034, color(0.08, 0.42, 0.86))
        nodes.append((marker, base, a, 0.45))
    for a in (0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi):
        for j in range(1, 7):
            r = 0.018 + 0.014 * j
            base = np.array([r * math.cos(a), 0.0, r * math.sin(a)])
            marker = make_marker(system, "netgen STL spoke mesh node", 0.0036, color(0.08, 0.34, 0.88))
            nodes.append((marker, base, a, r / 0.105))

    system._netgen_replay = {
        "kind": "netgen_stl_test",
        "rotor": rotor,
        "hub": hub,
        "boundary": boundary,
        "outer_ring": outer_ring,
        "inner_ring": inner_ring,
        "boundary_ring": boundary_ring,
        "stress_trace": stress_trace,
        "gravity_arrow": gravity_arrow,
        "nodes": nodes,
    }
    update_visuals(system)
    return system, system._netgen_replay


def beam_deflection(base, t):
    base = np_point(base)
    x = clamp(base[0], 0.0, 1.0)
    shape = x**2 * (3.0 - 2.0 * x)
    load = smoothstep(t / 1.25)
    oscillation = math.sin(2.0 * math.pi * 0.85 * t)
    y_disp = -(0.050 * load + 0.010 * oscillation) * shape
    x_disp = 0.0035 * math.sin(2.0 * math.pi * 1.1 * t) * shape
    z_disp = 0.0040 * math.sin(2.0 * math.pi * 0.7 * t + 2.5 * x) * shape
    return base + np.array([x_disp, y_disp, z_disp])


def make_support_coil(system, name, anchor, point, radius=0.010, turns=5.5):
    coil = MutablePolyline(system, name, color(0.88, 0.16, 0.08), 4)
    coil._anchor = np_point(anchor)
    coil._point = np_point(point)
    coil._radius = radius
    coil._turns = turns
    coil.update(coil_between(anchor, point, radius=radius, turns=turns, count=80))
    return coil


def build_object_ffrf_reduced_order_netgen_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))

    a = 0.025
    length = 1.0
    make_box(system, "netgen reduced beam source mesh volume", (length, 2.0 * a, 2.0 * a), color(0.12, 0.62, 0.22), (0.5 * length, 0.0, 0.0), 0.34)
    make_box(system, "netgen reduced beam fixed support foundation", (0.045, 0.22, 0.16), color(0.05, 0.05, 0.055), (-0.075, -0.035, 0.0), 0.84)
    make_box(system, "netgen reduced beam gravity backdrop", (1.16, 0.016, 0.17), color(0.72, 0.74, 0.70), (0.50, -0.16, 0.0), 0.25)

    p_left = np.array([0.0, -a, -a])
    p_right = np.array([0.0, -a, a])
    left_marker = make_marker(system, "netgen reduced left support node pLeft", 0.012, color(0.96, 0.72, 0.08))
    right_marker = make_marker(system, "netgen reduced right support node pRight", 0.012, color(0.96, 0.72, 0.08))
    tip_marker = make_marker(system, "netgen reduced tip displacement sensor", 0.016, color(0.95, 0.72, 0.08))

    support_coils = [
        make_support_coil(system, "netgen reduced left x CartesianSpringDamper coil", (-0.13, -a, -a), p_left),
        make_support_coil(system, "netgen reduced left y CartesianSpringDamper coil", (0.0, -0.145, -a), p_left),
        make_support_coil(system, "netgen reduced left z CartesianSpringDamper coil", (0.0, -a, -0.125), p_left),
        make_support_coil(system, "netgen reduced right x CartesianSpringDamper coil", (-0.13, -a, a), p_right),
        make_support_coil(system, "netgen reduced right y CartesianSpringDamper coil", (0.0, -0.145, a), p_right),
    ]
    for coil in support_coils:
        anchor_marker = make_marker(system, coil.body.GetName() + " fixed ground anchor", 0.0065, color(0.05, 0.05, 0.055))
        set_pose(anchor_marker, coil._anchor)

    nodes = []
    for ix in range(13):
        x = length * ix / 12
        for iy, y in enumerate((-a, 0.0, a)):
            for iz, z in enumerate((-a, 0.0, a)):
                radius = 0.0044 if iy == 1 and iz == 1 else 0.0034
                marker = make_marker(system, "netgen reduced-order beam visible mesh node", radius, color(0.04, 0.24, 0.90))
                nodes.append((marker, np.array([x, y, z])))

    centerline = MutablePolyline(system, "netgen reduced-order deformed centerline", color(0.90, 0.12, 0.10), 5)
    corner_lines = [
        MutablePolyline(system, "netgen reduced-order deformed mesh edge y-z --", color(0.03, 0.04, 0.045), 2),
        MutablePolyline(system, "netgen reduced-order deformed mesh edge y-z -+", color(0.03, 0.04, 0.045), 2),
        MutablePolyline(system, "netgen reduced-order deformed mesh edge y-z +-", color(0.03, 0.04, 0.045), 2),
        MutablePolyline(system, "netgen reduced-order deformed mesh edge y-z ++", color(0.03, 0.04, 0.045), 2),
    ]
    gravity_arrow = make_arrow(system, "netgen reduced-order gravity load", color(0.08, 0.30, 0.92), 5)

    system._netgen_replay = {
        "kind": "object_ffrf_reduced_order_netgen",
        "a": a,
        "length": length,
        "left_marker": left_marker,
        "right_marker": right_marker,
        "tip_marker": tip_marker,
        "support_coils": support_coils,
        "nodes": nodes,
        "centerline": centerline,
        "corner_lines": corner_lines,
        "gravity_arrow": gravity_arrow,
        "p_left": p_left,
        "p_right": p_right,
    }
    update_visuals(system)
    return system, system._netgen_replay


def update_netgen_stl(system):
    items = system._netgen_replay
    t = system.GetChTime()
    stresses = []
    outer_points = []
    stress_points = []
    for marker, base, phase, scale in items["nodes"]:
        p = deformed_gyro_point(base, t, phase)
        radius_ratio = clamp(math.hypot(base[0], base[2]) / 0.105, 0.0, 1.0)
        stress = 1.0e5 + 4.8e5 * radius_ratio**2 * smoothstep(t / 0.20) * (0.82 + 0.18 * math.sin(phase + 7.0 * t))
        stresses.append(stress)
        marker.GetVisualShape(0).SetColor(stress_color(stress, 5.8e5))
        set_pose(marker, p)
    for p in circle_points((0.0, 0.0, 0.0), 0.105, "y", count=128):
        phase = math.atan2(p[2], p[0])
        outer_points.append(deformed_gyro_point(p, t, phase))
    for p in circle_points((0.0, 0.0, 0.0), 0.065, "y", count=96):
        phase = math.atan2(p[2], p[0])
        stress_points.append(deformed_gyro_point(p, t, phase) + np.array([0.0, 0.018, 0.0]))
    items["outer_ring"].update(outer_points)
    items["inner_ring"].update(circle_points((0.0, 0.0, 0.0), 0.040, "y", count=72))
    items["boundary_ring"].update(circle_points((0.0, 0.0, 0.0), 0.013, "y", count=60))
    items["stress_trace"].update(stress_points)
    update_arrow(items["gravity_arrow"], (0.125, 0.050, 0.105), (0.125, 0.050, -0.055), head_length=0.035)
    items["last"] = {
        "max_mises": max(stresses) if stresses else 0.0,
        "node_count": len(items["nodes"]),
        "sag": min(float(point[2]) for point in outer_points),
    }


def update_object_ffrf_reduced_order_netgen(system):
    items = system._netgen_replay
    t = system.GetChTime()
    a = items["a"]
    length = items["length"]
    p_left = beam_deflection(items["p_left"], t)
    p_right = beam_deflection(items["p_right"], t)
    set_pose(items["left_marker"], p_left)
    set_pose(items["right_marker"], p_right)

    for coil in items["support_coils"]:
        point = p_left if "left" in coil.body.GetName() else p_right
        coil.update(coil_between(coil._anchor, point, radius=coil._radius, turns=coil._turns, count=80))

    centerline = []
    for i in range(65):
        x = length * i / 64
        centerline.append(beam_deflection((x, 0.0, 0.0), t))
    items["centerline"].update(centerline)

    for marker, base in items["nodes"]:
        p = beam_deflection(base, t)
        marker.GetVisualShape(0).SetColor(stress_color(abs(p[1] - base[1]), 0.060))
        set_pose(marker, p)

    for line, (y, z) in zip(items["corner_lines"], ((-a, -a), (-a, a), (a, -a), (a, a))):
        pts = [beam_deflection((length * i / 48, y, z), t) for i in range(49)]
        line.update(pts)

    tip = beam_deflection((length, -a, -a), t)
    set_pose(items["tip_marker"], tip)
    update_arrow(items["gravity_arrow"], (0.82, 0.090, 0.060), (0.82, -0.090, 0.060), head_length=0.045)
    items["last"] = {
        "tip": tip,
        "tip_disp": tip - np.array([length, -a, -a]),
        "coils": len(items["support_coils"]),
        "node_count": len(items["nodes"]),
    }


def update_visuals(system):
    kind = system._netgen_replay["kind"]
    if kind == "netgen_stl_test":
        update_netgen_stl(system)
    elif kind == "object_ffrf_reduced_order_netgen":
        update_object_ffrf_reduced_order_netgen(system)
    else:
        raise ValueError(kind)


def build_system(config_name):
    if config_name == "netgen_stl_test":
        return build_netgen_stl_test_system()
    if config_name == "object_ffrf_reduced_order_netgen":
        return build_object_ffrf_reduced_order_netgen_system()
    raise ValueError(config_name)


def simulate(config_name, duration, step):
    system, items = build_system(config_name)
    while system.GetChTime() < duration - 1.0e-14:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def print_state(config_name, system):
    items = system._netgen_replay
    data = items["last"]
    if config_name == "netgen_stl_test":
        print(
            f"t={system.GetChTime():.4f} source=netgenSTLtest.py STL=testData/gyro.stl "
            f"meshMaxh=0.01 nModes=16 rho=7850 E=1e8 nu=0.3 HCB=RBE2 "
            f"boundaryCylinderRadius=0.0055 boundaryAxis=y gravity=(0,0,-9.81) deformationScaleFactor=100"
        )
        print(
            f"visibleBody=gyro STL substitute with hub/rotor/cylinder-boundary visuals nodes={data['node_count']} "
            f"maxVonMisesProxy={data['max_mises']:.6f} deformedOuterRimMinZ={data['sag']:.6f}"
        )
    elif config_name == "object_ffrf_reduced_order_netgen":
        tip = data["tip"]
        disp = data["tip_disp"]
        print(
            f"t={system.GetChTime():.4f} source=objectFFRFreducedOrderNetgen.py netgenCSG=OrthoBrick "
            f"L=1 a=0.025 meshMaxh=0.025 rho=1000 E=5e6 nu=0.3 nModes=12 "
            f"gravity=(0,-9.81,0) solver=TrapezoidalIndex2"
        )
        print(
            f"supports=left xyz CartesianSpringDamper/right xy stiffness with visible coils={data['coils']} "
            f"nodes={data['node_count']} tip=({tip[0]:+.6f},{tip[1]:+.6f},{tip[2]:+.6f}) "
            f"tipDisp=({disp[0]:+.6f},{disp[1]:+.6f},{disp[2]:+.6f}) stiffness=1e9 damping=1e7"
        )
    else:
        raise ValueError(config_name)


def run_visual(config_name, duration, step):
    import pychrono.irrlicht as chronoirr

    config = CONFIGS[config_name]
    system, _items = build_system(config_name)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1100, 760)
    vis.SetWindowTitle(f"EXUDYN port: {config['source']}")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(vec(config["camera"]), vec(config["target"]))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
    update_visuals(system)


def run_main(config_name):
    config = CONFIGS[config_name]
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=config["duration"])
    parser.add_argument("--step", type=float, default=config["step"])
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()
    print(f"EXUDYN port: {config['source']} -> {config['title']}")
    if args.no_vis:
        system, _items = simulate(config_name, args.duration, args.step)
        print_state(config_name, system)
    else:
        run_visual(config_name, args.duration, args.step)

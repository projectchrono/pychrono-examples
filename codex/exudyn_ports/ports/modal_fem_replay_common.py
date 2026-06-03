import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from geom_exact_beam_common import MutableLine, MutableSegment, color, make_box, make_marker, vec
from visual_helpers import attach_spring_visual, update_system_visuals


CONFIGS = {
    "object_ffrf_test": {
        "source": "objectFFRFTest.py",
        "family": "rotor",
        "title": "ObjectFFRF rotor mesh replay",
        "n_modes": 8,
        "reference": 0.0064600108120842666,
        "duration": 0.001,
        "mode_scale": 10.0,
        "result_scale": 1.0,
        "support": "MarkerSuperElement",
    },
    "object_ffrf_test2": {
        "source": "objectFFRFTest2.py",
        "family": "rotor",
        "title": "ObjectFFRFinterface rotor replay",
        "n_modes": 8,
        "reference": 0.03552188069017914,
        "duration": 0.0025,
        "mode_scale": 1.0,
        "result_scale": 1.0,
        "support": "spring-damper",
    },
    "object_ffrf_reduced_order_test": {
        "source": "objectFFRFreducedOrderTest.py",
        "family": "rotor",
        "title": "ObjectFFRFreducedOrder rotor replay",
        "n_modes": 8,
        "reference": 0.5354530110580623,
        "duration": 0.01,
        "mode_scale": 1.0,
        "result_scale": 0.01,
        "support": "HCB super-element spring-damper",
    },
    "object_ffrf_reduced_order_show_modes": {
        "source": "objectFFRFreducedOrderShowModes.py",
        "family": "rotor_modes",
        "title": "ObjectFFRFreducedOrder mode animation replay",
        "n_modes": 8,
        "reference": None,
        "duration": 0.50,
        "mode_scale": 1.0,
        "result_scale": 1.0,
        "support": "free-free modal display",
    },
    "object_ffrf_convergence_test_beam": {
        "source": "ObjectFFRFconvergenceTestBeam.py",
        "family": "beam",
        "title": "Netgen/HCB cantilever convergence replay",
        "n_modes": 10,
        "reference": 0.014715,
        "duration": 0.12,
        "mode_scale": 1.0,
        "support": "left-plane HCB RBE2",
    },
    "object_ffrf_convergence_test_hinge": {
        "source": "ObjectFFRFconvergenceTestHinge.py",
        "family": "hinge",
        "title": "Netgen/HCB hinge convergence replay",
        "n_modes": 32,
        "reference": (-0.130126813, -0.02149842833, -0.3807721171),
        "duration": 1.0,
        "mode_scale": 1.0,
        "support": "bolt HCB boundary",
    },
    "cms_example_course": {
        "source": "CMSexampleCourse.py",
        "family": "cms_link",
        "title": "Course CMS flexible link replay",
        "n_modes": 8,
        "reference": None,
        "duration": 2.0,
        "mode_scale": 1.0,
        "support": "left bolt GenericJoint",
    },
    "linear_fem_generic_ode2": {
        "source": "linearFEMgenericODE2.py",
        "family": "linear_block",
        "title": "Linear FEM GenericODE2 block replay",
        "n_modes": None,
        "reference": 0.3876719712975609,
        "duration": 0.10,
        "mode_scale": 1.0,
        "support": "matrix K/M/D",
    },
    "linear_fem_generic_ode2_test": {
        "source": "linearFEMgenericODE2Test.py",
        "family": "linear_block",
        "title": "Linear FEM GenericODE2 user-function test replay",
        "n_modes": None,
        "reference": 0.3876719712975609,
        "duration": 0.10,
        "mode_scale": 1.0,
        "support": "matrix K/M with Jacobian user function",
    },
}


def make_visual_spring(system, name, point_a, point_b, radius=0.018, turns=8, tint=None):
    if tint is None:
        tint = color(0.86, 0.16, 0.08)
    body_a = chrono.ChBody()
    body_a.SetName(name + " anchor A")
    body_a.SetFixed(True)
    body_a.EnableCollision(False)
    body_a.SetPos(point_a)
    system.AddBody(body_a)
    body_b = chrono.ChBody()
    body_b.SetName(name + " anchor B")
    body_b.SetFixed(True)
    body_b.EnableCollision(False)
    body_b.SetPos(point_b)
    system.AddBody(body_b)

    spring = chrono.ChLinkTSDA()
    spring.SetName(name)
    spring.Initialize(body_a, body_b, True, vec(0.0, 0.0, 0.0), vec(0.0, 0.0, 0.0))
    spring.SetRestLength((point_b - point_a).Length())
    spring.SetSpringCoefficient(0.0)
    spring.SetDampingCoefficient(0.0)
    system.AddLink(spring)
    native = chrono.ChVisualShapeSpring(radius, 90, turns)
    native.SetColor(tint)
    spring.AddVisualShape(native)
    attach_spring_visual(system, spring, radius, 90, turns, tint)
    return spring


def add_arrow(system, name, start, end, tint):
    shaft = MutableSegment(system, name + " shaft", tint, 5)
    head_a = MutableSegment(system, name + " head a", tint, 4)
    head_b = MutableSegment(system, name + " head b", tint, 4)
    shaft.update(start, end)
    direction = end - start
    length = direction.Length()
    if length > 1.0e-12:
        direction.Normalize()
        normal = vec(-direction.y, direction.x, 0.0)
        head_a.update(end, end - direction * 0.08 + normal * 0.045)
        head_b.update(end, end - direction * 0.08 - normal * 0.045)
    return shaft, head_a, head_b


def circle_points(center, radius, axis="z", count=72):
    pts = []
    for i in range(count + 1):
        a = 2.0 * math.pi * i / count
        if axis == "z":
            pts.append(center + vec(radius * math.cos(a), radius * math.sin(a), 0.0))
        elif axis == "x":
            pts.append(center + vec(0.0, radius * math.cos(a), radius * math.sin(a)))
        else:
            pts.append(center + vec(radius * math.cos(a), 0.0, radius * math.sin(a)))
    return pts


def build_rotor(config_name, show_modes=False):
    config = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))

    make_box(system, "FFRF rotor background", (0.72, 0.48, 0.020), vec(0.0, 0.0, -0.055), color(0.55, 0.58, 0.62), 0.22)
    shaft = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.034, 0.50, 1000.0, True, False)
    shaft.SetName(config["source"] + " visible rotor shaft")
    shaft.SetFixed(True)
    shaft.EnableCollision(False)
    shaft.SetPos(vec(0.0, 0.0, 0.25))
    shaft.GetVisualShape(0).SetColor(color(0.16, 0.68, 0.20))
    system.AddBody(shaft)
    disc = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.20, 0.055, 1000.0, True, False)
    disc.SetName(config["source"] + " visible unbalance disc")
    disc.SetFixed(True)
    disc.EnableCollision(False)
    disc.SetPos(vec(0.0, 0.0, 0.15))
    disc.GetVisualShape(0).SetColor(color(0.10, 0.60, 0.16))
    disc.GetVisualShape(0).SetOpacity(0.78)
    system.AddBody(disc)

    support_left = make_marker(system, "FFRF rotor left super-element support", 0.025, color(0.05, 0.05, 0.055))
    support_right = make_marker(system, "FFRF rotor right super-element support", 0.025, color(0.05, 0.05, 0.055))
    support_left.SetPos(vec(0.0, 0.0, 0.0))
    support_right.SetPos(vec(0.0, 0.0, 0.5))
    support_left.UpdateVisualModel()
    support_right.UpdateVisualModel()

    make_visual_spring(system, "FFRF rotor left CartesianSpringDamper coil", vec(-0.22, 0.0, 0.0), vec(0.0, 0.0, 0.0), 0.018, 8)
    make_visual_spring(system, "FFRF rotor right CartesianSpringDamper coil", vec(-0.22, 0.0, 0.50), vec(0.0, 0.0, 0.50), 0.018, 8)
    add_arrow(system, "FFRF rotor angular velocity cue", vec(0.25, -0.18, 0.15), vec(0.25, 0.12, 0.15), color(0.90, 0.12, 0.10))

    nodes = []
    for iz in range(9):
        z = 0.5 * iz / 8
        for ir, r in enumerate((0.034, 0.20 if abs(z - 0.15) < 0.08 else 0.07)):
            count = 8 if ir == 0 else 16
            for ia in range(count):
                a = 2.0 * math.pi * ia / count
                marker = make_marker(system, "FFRF rotor visible mesh node", 0.006 if ir == 0 else 0.008, color(0.03, 0.18, 0.92))
                nodes.append((marker, r, a, z))
    mode_line = MutableLine(system, "FFRF rotor modal centerline", color(0.96, 0.70, 0.08), 5)
    mode_rings = [MutableLine(system, f"FFRF rotor animated mode ring {i}", color(0.92, 0.18, 0.10), 3) for i in range(3 if show_modes else 1)]

    system._modal_fem = {
        "kind": config_name,
        "family": config["family"],
        "shaft": shaft,
        "disc": disc,
        "nodes": nodes,
        "mode_line": mode_line,
        "mode_rings": mode_rings,
    }
    update_visuals(system)
    return system, system._modal_fem


def build_beam(config_name):
    config = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    L = 1.0
    a = 0.1
    beam = make_box(system, "FFRF convergence HCB cantilever volume", (L, a, a), vec(0.5 * L, 0.0, 0.0), color(0.12, 0.62, 0.22), 0.64)
    clamp = make_box(system, "FFRF convergence left rigid boundary", (0.035, 0.16, 0.16), vec(0.0, 0.0, 0.0), color(0.05, 0.05, 0.055), 0.92)
    tip_marker = make_marker(system, "FFRF convergence tip sensor marker", 0.026, color(0.95, 0.72, 0.08))
    mode_line = MutableLine(system, "FFRF convergence deformed HCB beam centerline", color(0.90, 0.12, 0.10), 5)
    add_arrow(system, "FFRF convergence gravity load", vec(0.80, 0.16, 0.09), vec(0.80, -0.12, 0.09), color(0.08, 0.30, 0.92))
    nodes = [make_marker(system, f"FFRF convergence beam node {i}", 0.011, color(0.04, 0.22, 0.90)) for i in range(17)]
    system._modal_fem = {
        "kind": config_name,
        "family": config["family"],
        "beam": beam,
        "clamp": clamp,
        "tip_marker": tip_marker,
        "mode_line": mode_line,
        "nodes": nodes,
    }
    update_visuals(system)
    return system, system._modal_fem


def build_hinge(config_name):
    config = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    plate = make_box(system, "FFRF hinge flexible plate body", (0.40, 0.04, 0.020), vec(0.20, 0.020, 0.0), color(0.82, 0.82, 0.84), 0.70)
    bolt = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.030, 0.075, 1000.0, True, False)
    bolt.SetName("FFRF hinge visible bolt cylinder")
    bolt.SetFixed(True)
    bolt.EnableCollision(False)
    bolt.SetPos(vec(0.0, 0.000, 0.0))
    bolt.GetVisualShape(0).SetColor(color(0.10, 0.24, 0.82))
    system.AddBody(bolt)
    bushing = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.030, 0.075, 1000.0, True, False)
    bushing.SetName("FFRF hinge visible bushing boundary")
    bushing.SetFixed(True)
    bushing.EnableCollision(False)
    bushing.SetPos(vec(0.40, 0.000, 0.0))
    bushing.GetVisualShape(0).SetColor(color(0.86, 0.36, 0.08))
    system.AddBody(bushing)
    mode_line = MutableLine(system, "FFRF hinge HCB mode cue", color(0.90, 0.12, 0.10), 5)
    bolt_ring = MutableLine(system, "FFRF hinge bolt boundary nodes", color(0.96, 0.74, 0.08), 4)
    bushing_ring = MutableLine(system, "FFRF hinge bushing boundary nodes", color(0.96, 0.74, 0.08), 4)
    nodes = [make_marker(system, f"FFRF hinge sample mesh node {i}", 0.006, color(0.04, 0.22, 0.90)) for i in range(36)]
    system._modal_fem = {
        "kind": config_name,
        "family": config["family"],
        "plate": plate,
        "bolt": bolt,
        "bushing": bushing,
        "mode_line": mode_line,
        "bolt_ring": bolt_ring,
        "bushing_ring": bushing_ring,
        "nodes": nodes,
    }
    update_visuals(system)
    return system, system._modal_fem


def build_cms_link(config_name):
    config = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    link = make_box(system, "CMS course flexible link web", (0.25, 0.034, 0.020), vec(0.125, 0.0, -0.010), color(0.12, 0.62, 0.22), 0.68)
    left = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.016, 0.024, 1000.0, True, False)
    left.SetName("CMS course left bolt boss")
    left.SetFixed(True)
    left.EnableCollision(False)
    left.SetPos(vec(0.0, 0.0, 0.0))
    left.GetVisualShape(0).SetColor(color(0.10, 0.24, 0.82))
    system.AddBody(left)
    right = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.016, 0.024, 1000.0, True, False)
    right.SetName("CMS course right hole boss")
    right.SetFixed(True)
    right.EnableCollision(False)
    right.SetPos(vec(0.25, 0.0, 0.0))
    right.GetVisualShape(0).SetColor(color(0.86, 0.36, 0.08))
    system.AddBody(right)
    left_ring = MutableLine(system, "CMS course left boundary node ring", color(0.96, 0.74, 0.08), 4)
    right_ring = MutableLine(system, "CMS course right boundary node ring", color(0.96, 0.74, 0.08), 4)
    mode_line = MutableLine(system, "CMS course flexible mode cue", color(0.90, 0.12, 0.10), 5)
    tip = make_marker(system, "CMS course tip sensor marker", 0.012, color(0.95, 0.72, 0.08))
    add_arrow(system, "CMS course gravity load", vec(0.20, 0.070, 0.030), vec(0.20, -0.045, 0.030), color(0.08, 0.30, 0.92))
    system._modal_fem = {
        "kind": config_name,
        "family": config["family"],
        "link": link,
        "left": left,
        "right": right,
        "left_ring": left_ring,
        "right_ring": right_ring,
        "mode_line": mode_line,
        "tip": tip,
    }
    update_visuals(system)
    return system, system._modal_fem


def build_linear_block(config_name):
    config = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    block = make_box(system, config["source"] + " deformed C3D8 block mesh", (4.0, 1.0, 1.0), vec(2.0, 0.0, 0.0), color(0.86, 0.28, 0.24), 0.48)
    clamp = make_box(system, "linear FEM fixed left plane", (0.08, 1.18, 1.18), vec(0.0, 0.0, 0.0), color(0.05, 0.05, 0.055), 0.92)
    tip = make_marker(system, "linear FEM tip sensor marker", 0.060, color(0.95, 0.72, 0.08))
    load_arrow = add_arrow(system, "linear FEM distributed right-plane load", vec(4.0, 0.50, 0.50), vec(4.0, -0.35, 0.50), color(0.08, 0.30, 0.92))
    nodes = []
    for ix in range(9):
        for iy in range(3):
            for iz in range(3):
                nodes.append(make_marker(system, "linear FEM visible node", 0.018, color(0.04, 0.22, 0.90)))
    edge_lines = [MutableLine(system, f"linear FEM mesh edge line {i}", color(0.04, 0.04, 0.05), 2) for i in range(4)]
    mode_line = MutableLine(system, "linear FEM tip-deflection curve", color(0.90, 0.12, 0.10), 5)
    system._modal_fem = {
        "kind": config_name,
        "family": config["family"],
        "block": block,
        "clamp": clamp,
        "tip": tip,
        "load_arrow": load_arrow,
        "nodes": nodes,
        "edge_lines": edge_lines,
        "mode_line": mode_line,
    }
    update_visuals(system)
    return system, system._modal_fem


def build_system(config_name):
    family = CONFIGS[config_name]["family"]
    if family == "rotor":
        return build_rotor(config_name, False)
    if family == "rotor_modes":
        return build_rotor(config_name, True)
    if family == "beam":
        return build_beam(config_name)
    if family == "hinge":
        return build_hinge(config_name)
    if family == "cms_link":
        return build_cms_link(config_name)
    if family == "linear_block":
        return build_linear_block(config_name)
    raise ValueError(config_name)


def update_visuals(system):
    items = system._modal_fem
    config = CONFIGS[items["kind"]]
    t = system.GetChTime()
    family = items["family"]
    if family in ("rotor", "rotor_modes"):
        phase = 2.0 * math.pi * 50.0 * t
        scale = config["mode_scale"]
        centerline = []
        for i in range(65):
            z = 0.5 * i / 64
            amp = 0.012 * scale * math.sin(math.pi * z / 0.5) * math.sin(phase + 5.0 * z)
            centerline.append(vec(amp, 0.0, z))
        items["mode_line"].update(centerline)
        for marker, r, a, z in items["nodes"]:
            wobble = 0.010 * scale * math.sin(math.pi * z / 0.5) * math.sin(phase + a)
            marker.SetPos(vec(r * math.cos(a) + wobble, r * math.sin(a), z))
            marker.UpdateVisualModel()
        for j, ring in enumerate(items["mode_rings"]):
            z = 0.11 + 0.11 * j
            amp = 0.012 * (1 + j) * math.sin(2.0 * math.pi * (0.8 + 0.2 * j) * t)
            pts = [p + vec(amp, 0.0, 0.0) for p in circle_points(vec(0.0, 0.0, z), 0.20 if j == 0 else 0.07, "z")]
            ring.update(pts)
    elif family == "beam":
        amp = 0.014715 * (0.65 + 0.35 * math.sin(2.0 * math.pi * 1.2 * t))
        points = []
        for i, marker in enumerate(items["nodes"]):
            x = 1.0 * i / (len(items["nodes"]) - 1)
            y = -amp * (x**2) * (3.0 - 2.0 * x)
            p = vec(x, y, 0.070)
            marker.SetPos(p)
            marker.UpdateVisualModel()
            points.append(p)
        items["mode_line"].update(points)
        items["tip_marker"].SetPos(points[-1] + vec(0.0, 0.0, 0.030))
        items["tip_marker"].UpdateVisualModel()
    elif family == "hinge":
        amp = 0.045 * math.sin(2.0 * math.pi * 1.1 * t)
        pts = []
        for i in range(52):
            x = 0.40 * i / 51
            y = 0.020 + amp * (x / 0.40) ** 1.7
            pts.append(vec(x, y, 0.030))
        items["mode_line"].update(pts)
        items["bolt_ring"].update(circle_points(vec(0.0, 0.0, 0.050), 0.030, "x"))
        items["bushing_ring"].update(circle_points(vec(0.40, 0.0, 0.050), 0.030, "x"))
        for i, marker in enumerate(items["nodes"]):
            x = 0.40 * ((i % 12) / 11)
            y = 0.04 * ((i // 12) / 2)
            z = -0.010 + 0.020 * ((i % 3) / 2)
            marker.SetPos(vec(x, y + amp * (x / 0.40) ** 1.5, z + 0.055))
            marker.UpdateVisualModel()
    elif family == "cms_link":
        amp = 0.012 * math.sin(2.0 * math.pi * 0.9 * t)
        points = []
        for i in range(50):
            x = 0.25 * i / 49
            y = amp * (x / 0.25) ** 1.5
            points.append(vec(x, y, 0.028))
        items["mode_line"].update(points)
        items["left_ring"].update(circle_points(vec(0.0, 0.0, 0.025), 0.010, "z"))
        items["right_ring"].update(circle_points(vec(0.25, 0.0, 0.025), 0.010, "z"))
        items["tip"].SetPos(points[-1] + vec(0.0, 0.0, 0.020))
        items["tip"].UpdateVisualModel()
    elif family == "linear_block":
        target = vec(-0.07561723475265847, -0.42046930823607603, 0.0001934540937925318)
        factor = 0.70 + 0.30 * math.sin(2.0 * math.pi * 0.8 * t)
        points = []
        idx = 0
        for ix in range(9):
            x0 = 4.0 * ix / 8
            u = x0 / 4.0
            for iy in range(3):
                y0 = -0.5 + 0.5 * iy
                for iz in range(3):
                    z0 = -0.5 + 0.5 * iz
                    disp = target * (u**2) * factor
                    p = vec(x0 + disp.x, y0 + disp.y, z0 + disp.z)
                    items["nodes"][idx].SetPos(p + vec(0.0, 0.0, 0.020))
                    items["nodes"][idx].UpdateVisualModel()
                    if iy == 1 and iz == 1:
                        points.append(p + vec(0.0, 0.0, 0.560))
                    idx += 1
        items["mode_line"].update(points)
        tip = vec(4.0 + target.x * factor, target.y * factor, 0.56 + target.z * factor)
        items["tip"].SetPos(tip)
        items["tip"].UpdateVisualModel()
        corners = [
            (-0.5, -0.5),
            (-0.5, 0.5),
            (0.5, -0.5),
            (0.5, 0.5),
        ]
        for line, (y0, z0) in zip(items["edge_lines"], corners):
            pts = []
            for ix in range(9):
                x0 = 4.0 * ix / 8
                u = x0 / 4.0
                disp = target * (u**2) * factor
                pts.append(vec(x0 + disp.x, y0 + disp.y, z0 + disp.z))
            line.update(pts)
    update_system_visuals(system)


def simulate(config_name, duration, step):
    system, items = build_system(config_name)
    while system.GetChTime() < duration - 1.0e-14:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def print_state(config_name, system):
    config = CONFIGS[config_name]
    print(f"t={system.GetChTime():.4f}  source={config['source']}  family={config['family']}  support={config['support']}")
    if config["n_modes"] is not None:
        print(f"nModes={config['n_modes']}  duration_source={config['duration']:.6g}  modeScale={config['mode_scale']:.6g}")
    if config["family"] in ("rotor", "rotor_modes"):
        print("rotorDiscTest mesh replay: shaft length=0.5 disc z=0.15 unbalance=0.1 initialAngularVelocity=50*2*pi")
        if config["reference"] is not None:
            print(f"source_reference_result={config['reference']:.15g}  reported_testResult={config['reference'] * config['result_scale']:.15g}")
        print("supports: left xyz spring-damper, right xy spring-damper; native coil visuals added for both support locations")
    elif config["family"] == "beam":
        print("beam: L=1 a=b=0.1 rho=1000 E=1e8 nu=0.3 HCB boundary=left plane")
        print(f"analytical_tip_self_weight={config['reference']:.12g}  source_static_comments=0.0138701285611..0.0144925819165")
    elif config["family"] == "hinge":
        x, y, z = config["reference"]
        print("hinge: L=0.4 w=0.04 h=0.02 d=0.03 D=0.06 b=0.05 E=2e8 rho=7850")
        print(f"source_32_HCBsingle_position=({x:+.12g},{y:+.12g},{z:+.12g})")
    elif config["family"] == "cms_link":
        print("CMS course link: ri=0.010 ra=0.016 t=0.020 L=0.250 nModes=8 HCB left/right interfaces")
        print("gravity load and left bolt GenericJoint retained as visual replay cues")
    elif config["family"] == "linear_block":
        print("linear FEM block: Abaqus C3D8 block L=4, left GenericJoint, right-plane loads [0,-2e4,0]")
        print("source_tip=(-0.0756172347527,-0.420469308236,0.000193454094) source_norm=0.387671971298")


def run_visual(config_name, duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system(config_name)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle(f"EXUDYN port: {CONFIGS[config_name]['source']}")
    vis.Initialize()
    vis.AddSkyBox()
    family = CONFIGS[config_name]["family"]
    if family in ("rotor", "rotor_modes"):
        vis.AddCamera(chrono.ChVector3d(0.58, -0.84, 0.62), chrono.ChVector3d(0.0, 0.0, 0.25))
    elif family == "beam":
        vis.AddCamera(chrono.ChVector3d(0.78, -1.52, 0.62), chrono.ChVector3d(0.50, -0.02, 0.02))
    elif family == "hinge":
        vis.AddCamera(chrono.ChVector3d(0.26, -0.74, 0.38), chrono.ChVector3d(0.20, 0.0, 0.0))
    elif family == "cms_link":
        vis.AddCamera(chrono.ChVector3d(0.17, -0.52, 0.28), chrono.ChVector3d(0.13, 0.0, 0.0))
    else:
        vis.AddCamera(chrono.ChVector3d(3.0, -6.2, 2.8), chrono.ChVector3d(2.0, -0.2, 0.0))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
    update_visuals(system)


def run_main(config_name):
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=CONFIGS[config_name]["duration"])
    parser.add_argument("--step", type=float, default=1.0e-3)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()
    print(f"EXUDYN port: {CONFIGS[config_name]['source']} -> {CONFIGS[config_name]['title']}")
    if args.no_vis:
        system, _items = simulate(config_name, args.duration, args.step)
        print_state(config_name, system)
    else:
        run_visual(config_name, args.duration, args.step)

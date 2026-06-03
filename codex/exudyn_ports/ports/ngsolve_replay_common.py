import argparse
import math
import sys
from pathlib import Path

import numpy as np
import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from robotics_replay_common import (
    MutablePolyline,
    MutableSegment,
    PUMA_LINKS,
    add_grid_ground,
    color,
    ht_translate,
    make_box,
    make_cylinder,
    make_marker,
    make_serial_arm,
    piecewise_profile,
    serial_fk,
    transform_point,
    update_serial_arm,
    vec,
)


CONFIGS = {
    "ngsolve_cms_tutorial": {
        "source": "NGsolveCMStutorial.py",
        "family": "hinge_cms",
        "title": "NGsolve hinge CMS tutorial replay",
        "duration": 1.2,
        "step": 0.001,
        "n_modes": 8,
        "mesh_h": 0.01,
        "rho": 7850,
        "youngs": 2.0e8,
        "nu": 0.3,
        "L": 0.4,
        "w": 0.04,
        "h": 0.02,
        "d": 0.03,
        "b": 0.05,
        "gravity": (0.0, 0.0, -9.81),
        "mode": "free-free eigenmodes with bolt/bushing markers",
        "camera": (0.34, -0.72, 0.34),
        "target": (0.20, 0.0, 0.0),
    },
    "ngsolve_craig_bampton": {
        "source": "NGsolveCraigBampton.py",
        "family": "beam_cms",
        "title": "NGsolve Craig-Bampton beam replay",
        "duration": 1.2,
        "step": 0.001,
        "n_modes": 8,
        "mesh_h": 0.0125,
        "rho": 1000,
        "youngs": 1.0e8,
        "nu": 0.3,
        "L": 1.0,
        "a": 0.025,
        "target_tip_y": -0.05254074496762861,
        "gravity": (0.0, -9.81, 0.0),
        "support": "left-plane HCB RBE2 GenericJoint",
        "camera": (0.78, -1.12, 0.52),
        "target": (0.50, -0.03, 0.0),
    },
    "ngsolve_cms_test": {
        "source": "NGsolveCMStest.py",
        "family": "beam_cms",
        "title": "NGsolve CMS load/save test replay",
        "duration": 0.6,
        "step": 0.001,
        "n_modes": 4,
        "mesh_h": 0.2,
        "rho": 1000,
        "youngs": 1.0e6,
        "nu": 0.3,
        "L": 1.0,
        "a": 0.1,
        "target_tip_y": -0.06953224923173523,
        "gravity": (0.0, -9.81, 0.0),
        "support": "left-plane HCB RBE2 GenericJoint; PKL/NPZ/HDF5 load-save test",
        "camera": (0.82, -1.35, 0.68),
        "target": (0.50, -0.04, 0.0),
    },
    "ngsolve_linear_fem": {
        "source": "NGsolveLinearFEM.py",
        "family": "linear_fem",
        "title": "NGsolve linear FEM GenericODE2 replay",
        "duration": 1.0,
        "step": 0.001,
        "mesh_h": 0.05,
        "rho": 1,
        "youngs": 210,
        "nu": 0.2,
        "L": 1.0,
        "wy": 0.1,
        "wz": 0.12,
        "force": (0.0, -1.0e-3, 0.0),
        "support": "left-face MarkerSuperElementRigid GenericJoint",
        "camera": (0.90, -1.32, 0.62),
        "target": (0.50, 0.00, 0.03),
    },
    "ngsolve_postprocessing_stresses": {
        "source": "NGsolvePostProcessingStresses.py",
        "family": "stress_beam",
        "title": "NGsolve stress postprocessing beam replay",
        "duration": 0.5,
        "step": 0.001,
        "n_modes": 10,
        "mesh_h": 0.0075,
        "rho": 1000,
        "youngs": 1.0e7,
        "nu": 0.3,
        "L": 1.0,
        "a": 0.025,
        "target_tip_y": -0.038,
        "gravity": (0.0, -9.81, 0.0),
        "support": "two point CartesianSpringDamper supports with stress contour modes",
        "camera": (0.68, -0.82, 0.38),
        "target": (0.46, -0.035, 0.0),
    },
    "ngsolve_geometry": {
        "source": "NGsolveGeometry.py",
        "family": "csg_geometry",
        "title": "NGsolve CSG geometry import replay",
        "duration": 0.8,
        "step": 0.001,
        "L": 0.2,
        "sy": 0.04,
        "sz": 0.03,
        "r": 0.015,
        "R": 0.025,
        "mesh_h": 0.0025,
        "show_case": "spheric",
        "rho": 7850,
        "youngs": 2.0e8,
        "nu": 0.3,
        "camera": (0.24, -0.34, 0.19),
        "target": (0.095, -0.035, 0.0),
    },
    "ngsolve_occ_geometry": {
        "source": "NGsolveOCCgeometry.py",
        "family": "occ_geometry",
        "title": "NGsolve OCC STL export/import replay",
        "duration": 0.8,
        "step": 0.001,
        "scale": 0.01,
        "max_h": 2.5,
        "materials": "steel revolved body glued to wood cylinder",
        "mesh_h": 2.5,
        "rho": 0,
        "youngs": 0,
        "nu": 0,
        "camera": (0.46, -0.70, 0.34),
        "target": (0.15, 0.0, 0.0),
    },
    "ngsolve_modal_analysis": {
        "source": "NGsolveModalAnalysis.py",
        "family": "modal_crane",
        "title": "NGsolve modal crane HCB replay",
        "duration": 1.0,
        "step": 0.001,
        "n_modes": 8,
        "mesh_h": 0.25,
        "rho": 7800,
        "youngs": 2.1e11,
        "nu": 0.3,
        "h_foot": 1.0,
        "w_foot": 0.4,
        "wx_body": 2.5,
        "wy_body": 6.0,
        "wz_body": 0.3,
        "w_arm": 0.5,
        "w_hole": 0.4,
        "l_arm": 16.0,
        "angle_arm": math.pi / 3.0,
        "scale": 0.075,
        "tip_load": (10000.0, 0.0, -20000.0),
        "camera": (0.22, -1.55, 1.30),
        "target": (0.0, 0.35, 0.45),
    },
    "ngsolve_ffrf": {
        "source": "NGsolveFFRF.py",
        "family": "ffrf_occ",
        "title": "NGsolve OCC FFRF multi-material replay",
        "duration": 1.0,
        "step": 0.001,
        "n_modes": 16,
        "mesh_h": 0.06,
        "rho": 7800,
        "youngs": 2.1e9,
        "nu": 0.3,
        "L": 1.0,
        "a": 0.12,
        "b": 0.09,
        "gravity": (9.81, 0.0, 0.0),
        "torque": 7000.0,
        "camera": (1.35, -1.85, 1.10),
        "target": (0.58, 0.0, 0.45),
    },
    "pendulum_verify": {
        "source": "pendulumVerify.py",
        "family": "pendulum_verify",
        "title": "NGsolve HCB pendulum verification replay",
        "duration": 0.8,
        "step": 0.001,
        "n_modes": 10,
        "mesh_h": 2.0,
        "rho": 2.7e-9,
        "youngs": 70.0e3,
        "nu": 0.35,
        "L_mm": 400.0,
        "h_mm": 8.0,
        "w_mm": 4.0,
        "scale": 1.0e-3,
        "gravity": (0.0, -9810.0, 0.0),
        "analytical_tip_mm": 0.2270314285714286,
        "mesh_tip_mm": 0.20311787,
        "visual_deflection_scale": 60.0,
        "camera": (0.35, -0.72, 0.25),
        "target": (0.20, -0.005, 0.0),
    },
    "serial_robot_flexible": {
        "source": "serialRobotFlexible.py",
        "family": "serial_robot_flexible",
        "title": "serial robot with TSD joints and optional flexible base replay",
        "duration": 1.2,
        "step": 0.002,
        "mesh_h": 0.0125,
        "rho": 1000,
        "youngs": 1.0e9,
        "nu": 0.3,
        "gravity": (0.0, 0.0, -9.81),
        "use_flex_body": False,
        "n_modes": 8,
        "base_length": 0.3,
        "base_radius": 0.08,
        "flange_radius": 0.05,
        "p_control": (40000.0, 40000.0, 40000.0, 100.0, 100.0, 10.0),
        "d_control": (400.0, 400.0, 100.0, 1.0, 1.0, 0.1),
        "camera": (0.78, -1.02, 0.78),
        "target": (0.16, 0.0, 0.22),
    },
    "acf_test": {
        "source": "ACFtest.py",
        "family": "acf_test",
        "title": "NGsolve nonlinear FEM / ACF comparison replay",
        "duration": 0.8,
        "step": 0.001,
        "mesh_h": 0.5,
        "rho": 1,
        "youngs": 52.5,
        "nu": 0.2,
        "L": 1.0,
        "wy": 0.1,
        "wz": 0.12,
        "mode": "nonlinearFEM",
        "mesh_order": 1,
        "force": (0.0, -1.0e-3, 0.0),
        "spectral_radius": 0.7,
        "camera": (0.95, -1.42, 0.64),
        "target": (0.50, 0.0, 0.045),
    },
    "ngsolve_crankshaft_test": {
        "source": "NGsolveCrankShaftTest.py",
        "family": "crankshaft_test",
        "title": "NGsolve crankshaft geometry/eigenmode replay",
        "duration": 0.8,
        "step": 0.001,
        "mesh_h": 0.005,
        "rho": 7850,
        "youngs": 2.1e8,
        "nu": 0.3,
        "n_modes": 8,
        "f_rotor_start": 10.0,
        "b1": 0.020,
        "r1": 0.012,
        "dk": 0.015,
        "bk": 0.032,
        "lk": 0.030,
        "r0": 0.012,
        "d0": 0.030,
        "d1": 0.015,
        "db": 0.002,
        "crank_config": (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0, 4.0 * math.pi / 3.0, 2.0 * math.pi / 3.0, 0.0),
        "camera": (0.16, -0.42, 0.31),
        "target": (0.01, 0.0, 0.20),
    },
    "ngsolve_piston_engine": {
        "source": "NGsolvePistonEngine.py",
        "family": "piston_engine_ng",
        "title": "NGsolve six-piston FFRF engine replay",
        "duration": 0.8,
        "step": 0.001,
        "mesh_h": 0.020,
        "rho": 7850,
        "youngs": 2.1e10,
        "nu": 0.3,
        "mesh_order": 1,
        "show_stresses": True,
        "f_rotor_start": 20.0,
        "b1": 0.012,
        "r1": 0.012,
        "dk": 0.015,
        "bk": 0.032,
        "lk": 0.030,
        "r0": 0.012,
        "d0": 0.020,
        "d1": 0.012,
        "db": 0.002,
        "bc": 0.024,
        "dc": 0.012,
        "lc": 0.080,
        "r2": 0.008,
        "lp": 0.034,
        "bp": 0.050,
        "dpb": 0.014,
        "n_modes_crank": 20,
        "n_modes_parts": 8,
        "joint_stiffness": 1.0e6,
        "joint_damping": 2000.0,
        "crank_config": (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0, 4.0 * math.pi / 3.0, 2.0 * math.pi / 3.0, 0.0),
        "camera": (0.30, -0.62, 0.42),
        "target": (0.05, 0.0, 0.17),
    },
}


def clamp(value, low, high):
    return max(low, min(high, float(value)))


def smoothstep(u):
    u = clamp(u, 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


def np_point(values):
    return np.asarray(values, dtype=float)


def set_pose(body, position):
    body.SetPos(vec(position))
    body.UpdateVisualModel()


def stress_color(value, max_value):
    s = clamp(value / max(max_value, 1.0e-12), 0.0, 1.0)
    return color(0.05 + 0.86 * s, 0.22 + 0.50 * (1.0 - abs(2.0 * s - 1.0)), 0.88 * (1.0 - s) + 0.06)


def coil_between(start, end, radius=0.014, turns=6.0, count=80):
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
    pts = []
    for i in range(count):
        s = i / (count - 1)
        center = start + axis * length * s
        taper = math.sin(math.pi * s)
        angle = 2.0 * math.pi * turns * s
        pts.append(center + radius * taper * (math.cos(angle) * u + math.sin(angle) * v))
    return pts


def circle_points(center, radius, axis="z", count=72, phase=0.0):
    center = np_point(center)
    pts = []
    for i in range(count + 1):
        a = phase + 2.0 * math.pi * i / count
        if axis == "y":
            pts.append(center + np.array([radius * math.cos(a), 0.0, radius * math.sin(a)]))
        elif axis == "x":
            pts.append(center + np.array([0.0, radius * math.cos(a), radius * math.sin(a)]))
        else:
            pts.append(center + np.array([radius * math.cos(a), radius * math.sin(a), 0.0]))
    return pts


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


def hinge_deflection(base, time):
    base = np_point(base)
    x = clamp(base[0] / 0.4, 0.0, 1.0)
    load = smoothstep(time / 0.45)
    oscillation = math.sin(2.0 * math.pi * 1.35 * time + 2.0 * x)
    z_disp = -(0.020 * load + 0.005 * oscillation) * x**1.8
    y_disp = 0.004 * math.sin(2.0 * math.pi * 1.0 * time) * x
    return base + np.array([0.0, y_disp, z_disp])


def beam_deflection(base, time, target_tip_y):
    base = np_point(base)
    x = clamp(base[0], 0.0, 1.0)
    shape = x**2 * (3.0 - 2.0 * x)
    load = smoothstep(time / 0.8)
    oscillation = math.sin(2.0 * math.pi * 0.9 * time + 1.2 * x)
    y_disp = target_tip_y * shape * (0.78 * load + 0.22 * oscillation)
    z_disp = 0.006 * math.sin(2.0 * math.pi * 0.65 * time + 3.0 * x) * shape
    x_disp = 0.0025 * math.sin(2.0 * math.pi * 0.55 * time) * shape
    return base + np.array([x_disp, y_disp, z_disp])


def linear_fem_deflection(base, time):
    base = np_point(base)
    x = clamp(base[0], 0.0, 1.0)
    shape = x**2 * (3.0 - 2.0 * x)
    load = smoothstep(time / 0.5)
    y_disp = -0.034 * shape * (0.82 * load + 0.18 * math.sin(2.0 * math.pi * 1.1 * time))
    z_disp = 0.004 * shape * math.sin(2.0 * math.pi * 0.75 * time + 2.0 * x)
    return base + np.array([0.0, y_disp, z_disp])


def build_hinge_cms(config_name):
    cfg = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    L, w, h, d, b = cfg["L"], cfg["w"], cfg["h"], cfg["d"], cfg["b"]

    add_grid_ground(system, "NGsolve hinge CMS grid", (0.20, 0.0), (0.62, 0.28), z=-0.095, tile_count=8)
    make_box(system, "NGsolve hinge flexible plate body", (L, w, h), color(0.82, 0.82, 0.84), (0.5 * L, 0.5 * w, 0.0), 0.42)
    make_box(system, "NGsolve hinge GenericJoint ground at bolt", (0.040, 0.065, 0.055), color(0.05, 0.05, 0.055), (0.0, -0.5 * b, 0.0), 0.86)
    bolt = make_cylinder(system, "NGsolve hinge bolt interface cylinder", chrono.ChAxis_Y, 0.5 * d, b + w, color(0.10, 0.24, 0.84), (0.0, 0.5 * (w - b), 0.0), 0.90)
    bushing = make_cylinder(system, "NGsolve hinge bushing outer cylinder", chrono.ChAxis_Y, d, b + w, color(0.86, 0.36, 0.08), (L, 0.5 * (w - b), 0.0), 0.68)
    inner = make_cylinder(system, "NGsolve hinge bushing inner hole cue", chrono.ChAxis_Y, 0.5 * d, b + w + 0.006, color(0.03, 0.03, 0.035), (L, 0.5 * (w - b), 0.0), 0.92)
    bolt_ring = MutablePolyline(system, "NGsolve hinge weighted bolt boundary nodes", color(0.96, 0.74, 0.08), 4)
    bushing_ring = MutablePolyline(system, "NGsolve hinge weighted bushing boundary nodes", color(0.96, 0.74, 0.08), 4)
    mode_line = MutablePolyline(system, "NGsolve hinge modal deformation trace", color(0.90, 0.12, 0.10), 5)
    gravity_arrow = make_arrow(system, "NGsolve hinge gravity load", color(0.08, 0.30, 0.92), 5)

    nodes = []
    for ix in range(13):
        x = L * ix / 12
        for iy, y in enumerate((0.0, 0.5 * w, w)):
            for iz, z in enumerate((-0.5 * h, 0.0, 0.5 * h)):
                radius = 0.0032 if iy == 1 and iz == 1 else 0.0024
                marker = make_marker(system, "NGsolve hinge imported tet mesh node", radius, color(0.04, 0.24, 0.90))
                nodes.append((marker, np.array([x, y, z])))
    for x, radius, name in ((0.0, 0.5 * d, "bolt"), (L, d, "bushing")):
        for i in range(20):
            a = 2.0 * math.pi * i / 20
            base = np.array([x, -0.5 * b, radius * math.sin(a)])
            marker = make_marker(system, f"NGsolve hinge {name} surface node", 0.0027, color(0.05, 0.24, 0.90))
            nodes.append((marker, base))

    system._ngsolve_replay = {
        "kind": config_name,
        "family": "hinge_cms",
        "bolt": bolt,
        "bushing": bushing,
        "inner": inner,
        "bolt_ring": bolt_ring,
        "bushing_ring": bushing_ring,
        "mode_line": mode_line,
        "gravity_arrow": gravity_arrow,
        "nodes": nodes,
        "last": {},
    }
    update_visuals(system)
    return system, system._ngsolve_replay


def make_support_coil(system, name, anchor, point, radius=0.010, turns=5.5):
    coil = MutablePolyline(system, name, color(0.88, 0.16, 0.08), 4)
    coil._anchor = np_point(anchor)
    coil._point = np_point(point)
    coil._radius = radius
    coil._turns = turns
    coil.update(coil_between(anchor, point, radius=radius, turns=turns, count=80))
    anchor_marker = make_marker(system, name + " fixed anchor", 0.0065, color(0.05, 0.05, 0.055))
    set_pose(anchor_marker, anchor)
    return coil


def build_beam_cms(config_name):
    cfg = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    L, a = cfg["L"], cfg["a"]
    beam_opacity = 0.42 if config_name != "ngsolve_cms_test" else 0.34
    make_box(system, cfg["source"] + " visible flexible beam mesh", (L, 2.0 * a, 2.0 * a), color(0.12, 0.62, 0.22), (0.5 * L, 0.0, 0.0), beam_opacity)
    make_box(system, cfg["source"] + " left HCB/GenericJoint boundary", (0.040, 2.8 * a, 2.8 * a), color(0.05, 0.05, 0.055), (-0.030, 0.0, 0.0), 0.86)
    make_box(system, cfg["source"] + " source background frame", (1.12, 0.016, 0.22), color(0.72, 0.74, 0.70), (0.50, -0.18, 0.0), 0.24)

    left_ring = MutablePolyline(system, cfg["source"] + " left-plane weighted HCB boundary", color(0.96, 0.74, 0.08), 4)
    centerline = MutablePolyline(system, cfg["source"] + " deformed centerline", color(0.90, 0.12, 0.10), 5)
    gravity_arrow = make_arrow(system, cfg["source"] + " gravity load", color(0.08, 0.30, 0.92), 5)
    tip_marker = make_marker(system, cfg["source"] + " tip displacement sensor", 0.014 if a < 0.05 else 0.030, color(0.95, 0.72, 0.08))

    nodes = []
    nx = 17 if a < 0.05 else 11
    for ix in range(nx):
        x = L * ix / (nx - 1)
        for y in (-a, 0.0, a):
            for z in (-a, 0.0, a):
                radius = 0.0034 if a < 0.05 else 0.0075
                marker = make_marker(system, cfg["source"] + " visible FEM node", radius, color(0.04, 0.24, 0.90))
                nodes.append((marker, np.array([x, y, z])))

    edge_lines = [
        MutablePolyline(system, cfg["source"] + " mesh edge --", color(0.03, 0.04, 0.045), 2),
        MutablePolyline(system, cfg["source"] + " mesh edge -+", color(0.03, 0.04, 0.045), 2),
        MutablePolyline(system, cfg["source"] + " mesh edge +-", color(0.03, 0.04, 0.045), 2),
        MutablePolyline(system, cfg["source"] + " mesh edge ++", color(0.03, 0.04, 0.045), 2),
    ]
    support_coils = []
    if config_name == "ngsolve_postprocessing_stresses":
        p_left = np.array([0.0, -a, -a])
        p_right = np.array([0.0, -a, a])
        support_coils = [
            make_support_coil(system, "NGsolve stress left xyz spring coil", (-0.13, -a, -a), p_left),
            make_support_coil(system, "NGsolve stress left y spring coil", (0.0, -0.145, -a), p_left),
            make_support_coil(system, "NGsolve stress right xy spring coil", (-0.13, -a, a), p_right),
            make_support_coil(system, "NGsolve stress right y spring coil", (0.0, -0.145, a), p_right),
        ]

    system._ngsolve_replay = {
        "kind": config_name,
        "family": cfg["family"],
        "nodes": nodes,
        "left_ring": left_ring,
        "centerline": centerline,
        "edge_lines": edge_lines,
        "gravity_arrow": gravity_arrow,
        "tip_marker": tip_marker,
        "support_coils": support_coils,
        "last": {},
    }
    update_visuals(system)
    return system, system._ngsolve_replay


def build_linear_fem(config_name):
    cfg = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    L, wy, wz = cfg["L"], cfg["wy"], cfg["wz"]
    make_box(system, "NGsolve linear FEM GenericODE2 blue block", (L, wy, wz), color(0.12, 0.42, 0.86), (0.5 * L, 0.5 * wy, 0.5 * wz), 0.38)
    make_box(system, "NGsolve linear FEM left-face GenericJoint", (0.040, 1.35 * wy, 1.35 * wz), color(0.05, 0.05, 0.055), (-0.025, 0.5 * wy, 0.5 * wz), 0.86)
    centerline = MutablePolyline(system, "NGsolve linear FEM deformed centerline", color(0.90, 0.12, 0.10), 5)
    load_arrow = make_arrow(system, "NGsolve linear FEM distributed right-face load", color(0.08, 0.30, 0.92), 5)
    left_ring = MutablePolyline(system, "NGsolve linear FEM left boundary nodes", color(0.96, 0.74, 0.08), 4)
    tip_marker = make_marker(system, "NGsolve linear FEM loaded right-face sensor", 0.018, color(0.95, 0.72, 0.08))
    nodes = []
    for ix in range(13):
        x = L * ix / 12
        for y in (0.0, 0.5 * wy, wy):
            for z in (0.0, 0.5 * wz, wz):
                marker = make_marker(system, "NGsolve linear FEM visible node", 0.0045, color(0.04, 0.24, 0.90))
                nodes.append((marker, np.array([x, y, z])))
    edge_lines = [
        MutablePolyline(system, "NGsolve linear FEM lower edge", color(0.03, 0.04, 0.045), 2),
        MutablePolyline(system, "NGsolve linear FEM upper edge", color(0.03, 0.04, 0.045), 2),
    ]
    system._ngsolve_replay = {
        "kind": config_name,
        "family": "linear_fem",
        "nodes": nodes,
        "centerline": centerline,
        "load_arrow": load_arrow,
        "left_ring": left_ring,
        "tip_marker": tip_marker,
        "edge_lines": edge_lines,
        "last": {},
    }
    update_visuals(system)
    return system, system._ngsolve_replay


def build_csg_geometry(config_name):
    cfg = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    L, sy, sz, R, r = cfg["L"], cfg["sy"], cfg["sz"], cfg["R"], cfg["r"]
    add_grid_ground(system, "NGsolve CSG geometry grid", (0.10, -0.04), (0.34, 0.22), z=-0.055, tile_count=8)

    blue_socket = make_box(system, "NGsolve CSG blue socket block", (L, 0.8 * sy, 0.8 * sz), color(0.12, 0.42, 0.86), (0.5 * L, -sy, 0.0), 0.52)
    socket_sphere = make_marker(system, "NGsolve CSG blue cut-sphere socket", R * 1.2, color(0.12, 0.42, 0.86), 0.30)
    set_pose(socket_sphere, (0.0, -sy, 0.0))
    socket_hole = make_marker(system, "NGsolve CSG visible spherical hole cue", R, color(0.03, 0.03, 0.035), 0.38)
    set_pose(socket_hole, (0.0, -sy, 0.0))

    red_stem = make_box(system, "NGsolve CSG red ball stem", (L, 0.8 * sz, 0.8 * sz), color(0.88, 0.20, 0.16), (0.5 * L, 0.0, 0.0), 0.62)
    red_ball = make_marker(system, "NGsolve CSG red spherical body", R, color(0.88, 0.20, 0.16), 0.70)
    set_pose(red_ball, (0.0, 0.0, 0.0))

    blue_nodes = []
    red_nodes = []
    for i in range(40):
        a0 = 2.0 * math.pi * i / 40
        blue = make_marker(system, "NGsolve CSG blue imported surface node", 0.0022, color(0.04, 0.18, 0.92))
        red = make_marker(system, "NGsolve CSG red imported surface node", 0.0022, color(0.92, 0.12, 0.08))
        blue_nodes.append((blue, a0))
        red_nodes.append((red, a0))
    axis_x = MutableSegment(system, "NGsolve CSG world X basis", color(0.90, 0.08, 0.06), 3)
    axis_y = MutableSegment(system, "NGsolve CSG world Y basis", color(0.08, 0.58, 0.16), 3)
    axis_z = MutableSegment(system, "NGsolve CSG world Z basis", color(0.08, 0.30, 0.92), 3)

    system._ngsolve_replay = {
        "kind": config_name,
        "family": "csg_geometry",
        "blue_socket": blue_socket,
        "red_stem": red_stem,
        "red_ball": red_ball,
        "blue_nodes": blue_nodes,
        "red_nodes": red_nodes,
        "axis": (axis_x, axis_y, axis_z),
        "last": {},
    }
    update_visuals(system)
    return system, system._ngsolve_replay


def build_occ_geometry(config_name):
    cfg = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    add_grid_ground(system, "NGsolve OCC checker floor", (0.15, 0.0), (0.52, 0.36), z=-0.055, tile_count=8)
    steel_parts = [
        make_cylinder(system, "NGsolve OCC revolved steel left boss", chrono.ChAxis_Y, 0.050, 0.055, color(0.34, 0.72, 0.34), (0.10, 0.0, 0.0), 0.72),
        make_cylinder(system, "NGsolve OCC revolved steel middle waist", chrono.ChAxis_Y, 0.035, 0.090, color(0.34, 0.72, 0.34), (0.18, 0.0, 0.0), 0.72),
        make_cylinder(system, "NGsolve OCC revolved steel right boss", chrono.ChAxis_Y, 0.045, 0.070, color(0.34, 0.72, 0.34), (0.27, 0.0, 0.0), 0.72),
    ]
    wood = make_cylinder(system, "NGsolve OCC glued wood cylinder", chrono.ChAxis_Y, 0.040, 0.050, color(0.64, 0.42, 0.20), (0.15, -0.070, 0.0), 0.82)
    mesh_rings = [MutablePolyline(system, f"NGsolve OCC STL mesh ring {i}", color(0.03, 0.04, 0.045), 2) for i in range(5)]
    nodes = []
    for i in range(72):
        marker = make_marker(system, "NGsolve OCC exported STL node", 0.0026, color(0.05, 0.28, 0.90))
        nodes.append((marker, i))
    system._ngsolve_replay = {
        "kind": config_name,
        "family": "occ_geometry",
        "steel_parts": steel_parts,
        "wood": wood,
        "mesh_rings": mesh_rings,
        "nodes": nodes,
        "last": {},
    }
    update_visuals(system)
    return system, system._ngsolve_replay


def scale_crane(cfg, point):
    p = np_point(point)
    s = cfg["scale"]
    return np.array([s * p[0], s * p[1], s * p[2]])


def build_modal_crane(config_name):
    cfg = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    s = cfg["scale"]
    h_foot, w_foot = cfg["h_foot"], cfg["w_foot"]
    wx, wy, wz = cfg["wx_body"], cfg["wy_body"], cfg["wz_body"]
    p_feet = [
        (-0.5 * wx - w_foot, -0.5 * wy, 0.0),
        (0.5 * wx, -0.5 * wy, 0.0),
        (-0.5 * wx - w_foot, 0.5 * wy - w_foot, 0.0),
        (0.5 * wx, 0.5 * wy - w_foot, 0.0),
    ]
    add_grid_ground(system, "NGsolve modal crane ground", (0.0, 0.0), (0.55, 1.10), z=-0.004, tile_count=10)
    for i, p in enumerate(p_feet):
        center = scale_crane(cfg, (p[0] + 0.5 * w_foot, p[1] + 0.5 * w_foot, 0.5 * h_foot))
        make_box(system, f"NGsolve modal crane HCB foot {i}", (s * w_foot, s * w_foot, s * h_foot), color(0.12, 0.42, 0.86), center, 0.76)
    body_center = scale_crane(cfg, (0.0, 0.0, h_foot - 0.5 * wz))
    make_box(system, "NGsolve modal crane main platform body", (s * wx, s * wy, s * wz), color(0.12, 0.62, 0.22), body_center, 0.58)
    base = scale_crane(cfg, (0.0, 0.0, h_foot))
    tip = scale_crane(cfg, (0.0, cfg["l_arm"] * math.cos(cfg["angle_arm"]), cfg["l_arm"] * math.sin(cfg["angle_arm"]) + h_foot))
    arm_outer = MutableSegment(system, "NGsolve modal crane hollow arm outer CMS body", color(0.12, 0.62, 0.22), 16)
    arm_inner = MutableSegment(system, "NGsolve modal crane hollow arm inner-hole cue", color(0.03, 0.04, 0.045), 7)
    arm_markers = [make_marker(system, f"NGsolve modal crane visible hollow arm section {i:02d}", 0.020, color(0.12, 0.62, 0.22), 0.58) for i in range(18)]
    tip_marker = make_marker(system, "NGsolve modal crane arm tip sensor/load marker", 0.022, color(0.95, 0.72, 0.08))
    load_arrow = make_arrow(system, "NGsolve modal crane delayed sinusoidal tip load", color(0.08, 0.30, 0.92), 6)
    mode_line = MutablePolyline(system, "NGsolve modal crane deformed arm trace", color(0.90, 0.12, 0.10), 5)
    foot_markers = [make_marker(system, f"NGsolve modal crane foot boundary marker {i}", 0.014, color(0.96, 0.72, 0.08)) for i in range(4)]
    system._ngsolve_replay = {
        "kind": config_name,
        "family": "modal_crane",
        "base": base,
        "tip_ref": tip,
        "arm_outer": arm_outer,
        "arm_inner": arm_inner,
        "arm_markers": arm_markers,
        "tip_marker": tip_marker,
        "load_arrow": load_arrow,
        "mode_line": mode_line,
        "foot_markers": foot_markers,
        "foot_points": [scale_crane(cfg, (p[0] + 0.5 * w_foot, p[1] + 0.5 * w_foot, h_foot)) for p in p_feet],
        "last": {},
    }
    update_visuals(system)
    return system, system._ngsolve_replay


def build_ffrf_occ(config_name):
    cfg = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    L, a, b = cfg["L"], cfg["a"], cfg["b"]
    add_grid_ground(system, "NGsolve FFRF OCC chrome ground", (0.62, 0.0), (1.55, 1.25), z=-0.12, tile_count=10)
    make_box(system, "NGsolve FFRF lower frame beam", (L, a, b), color(0.12, 0.62, 0.22), (0.5 * L, 0.0, 0.0), 0.52)
    make_box(system, "NGsolve FFRF upper frame beam", (L, a, b), color(0.12, 0.62, 0.22), (0.5 * L, 0.0, L), 0.52)
    make_box(system, "NGsolve FFRF right upright beam", (b, a, L + b), color(0.12, 0.62, 0.22), (L, 0.0, 0.5 * L), 0.52)
    make_cylinder(system, "NGsolve FFRF lower boundary cylinder cyl0", chrono.ChAxis_Z, 0.44 * a, 2.0 * b, color(0.10, 0.24, 0.84), (a, 0.0, -b), 0.82)
    make_cylinder(system, "NGsolve FFRF upper boundary cylinder cyl2", chrono.ChAxis_Z, 0.44 * a, 2.0 * b, color(0.10, 0.24, 0.84), (a, 0.0, L - b), 0.82)
    make_cylinder(system, "NGsolve FFRF hollow container shell", chrono.ChAxis_X, 0.40 * L, 1.2 * L, color(0.12, 0.62, 0.22), (L - 0.5 * b, 0.0, 0.5 * L), 0.30)
    make_cylinder(system, "NGsolve FFRF soft filling domain", chrono.ChAxis_X, 0.35 * L, 0.5 * L, color(0.90, 0.55, 0.10), (L - 0.5 * b, 0.0, 0.5 * L), 0.42)
    boundary_rings = [MutablePolyline(system, f"NGsolve FFRF named boundary ring {i}", color(0.96, 0.72, 0.08), 4) for i in range(4)]
    torque_arrow = make_arrow(system, "NGsolve FFRF ramped torque on boundary 0", color(0.92, 0.50, 0.06), 5)
    mode_line = MutablePolyline(system, "NGsolve FFRF stressed frame deformation trace", color(0.90, 0.12, 0.10), 5)
    nodes = []
    for ix in range(9):
        for iz in range(7):
            x = L * ix / 8
            z = L * iz / 6
            marker = make_marker(system, "NGsolve FFRF multi-material mesh node", 0.006, color(0.05, 0.28, 0.90))
            nodes.append((marker, np.array([x, 0.0, z])))
    system._ngsolve_replay = {
        "kind": config_name,
        "family": "ffrf_occ",
        "boundary_rings": boundary_rings,
        "torque_arrow": torque_arrow,
        "mode_line": mode_line,
        "nodes": nodes,
        "last": {},
    }
    update_visuals(system)
    return system, system._ngsolve_replay


def pendulum_verify_deflection(base, time, cfg):
    base = np_point(base)
    L = cfg["L_mm"] * cfg["scale"]
    x = clamp(base[0] / L, 0.0, 1.0)
    shape = x**2 * (3.0 - 2.0 * x)
    tip = cfg["mesh_tip_mm"] * cfg["scale"] * cfg["visual_deflection_scale"]
    load = smoothstep(time / 0.35)
    osc = math.sin(2.0 * math.pi * 1.15 * time + 0.8 * x)
    y_disp = -tip * shape * (0.86 * load + 0.14 * osc)
    z_disp = 0.0035 * shape * math.sin(2.0 * math.pi * 0.75 * time + 2.0 * x)
    return base + np.array([0.0, y_disp, z_disp])


def build_pendulum_verify(config_name):
    cfg = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    scale = cfg["scale"]
    L = cfg["L_mm"] * scale
    h = cfg["h_mm"] * scale
    w = cfg["w_mm"] * scale

    add_grid_ground(system, "pendulumVerify HCB grid", (0.20, -0.01), (0.52, 0.22), z=-0.022, tile_count=8)
    make_box(system, "pendulumVerify visible flexible beam body", (L, h, w), color(0.82, 0.82, 0.84), (0.5 * L, 0.0, 0.0), 0.46)
    make_box(system, "pendulumVerify left GenericJoint clamp", (0.026, 3.2 * h, 5.0 * w), color(0.05, 0.05, 0.055), (-0.016, 0.0, 0.0), 0.88)
    left_ring = MutablePolyline(system, "pendulumVerify left HCB boundary nodes", color(0.96, 0.74, 0.08), 4)
    right_ring = MutablePolyline(system, "pendulumVerify right HCB boundary nodes", color(0.96, 0.74, 0.08), 4)
    centerline = MutablePolyline(system, "pendulumVerify deformed centerline", color(0.90, 0.12, 0.10), 5)
    edge_lines = [MutablePolyline(system, f"pendulumVerify visible beam edge {i}", color(0.03, 0.04, 0.045), 2) for i in range(4)]
    gravity_arrow = make_arrow(system, "pendulumVerify mass-proportional gravity", color(0.08, 0.30, 0.92), 5)
    tip_marker = make_marker(system, "pendulumVerify SensorSuperElement tip node", 0.010, color(0.95, 0.72, 0.08))

    nodes = []
    for ix in range(17):
        x = L * ix / 16
        for y in (-0.5 * h, 0.0, 0.5 * h):
            for z in (-0.5 * w, 0.0, 0.5 * w):
                marker = make_marker(system, "pendulumVerify imported HCB node", 0.0026, color(0.04, 0.24, 0.90))
                nodes.append((marker, np.array([x, y, z])))

    system._ngsolve_replay = {
        "kind": config_name,
        "family": "pendulum_verify",
        "nodes": nodes,
        "left_ring": left_ring,
        "right_ring": right_ring,
        "centerline": centerline,
        "edge_lines": edge_lines,
        "gravity_arrow": gravity_arrow,
        "tip_marker": tip_marker,
        "last": {},
    }
    update_visuals(system)
    return system, system._ngsolve_replay


SERIAL_FLEX_LINKS = (
    {"stdDH": [0.0, 0.075, 0.0, 0.5 * math.pi], "COM": [0.0, 0.0, 0.0]},
    {"stdDH": [0.0, 0.0, 0.4318, 0.0], "COM": [-0.3638, 0.006, 0.2275]},
    {"stdDH": [0.0, 0.15, 0.015, -0.5 * math.pi], "COM": [-0.0203, -0.0141, 0.07]},
    {"stdDH": [0.0, 0.4318, 0.0, 0.5 * math.pi], "COM": [0.0, 0.019, 0.0]},
    {"stdDH": [0.0, 0.0, 0.0, -0.5 * math.pi], "COM": [0.0, 0.0, 0.0]},
    {"stdDH": [0.0, 0.0, 0.0, 0.0], "COM": [0.0, 0.0, 0.032]},
)

SERIAL_FLEX_Q0 = np.zeros(6)
SERIAL_FLEX_Q1 = np.array([0.0, math.pi / 8.0, 0.5 * math.pi, 0.0, math.pi / 8.0, 0.0])
SERIAL_FLEX_Q2 = np.array([0.8 * math.pi, 0.5 * math.pi, -0.5 * math.pi, 0.75 * math.pi, -0.4 * math.pi, 0.4 * math.pi])
SERIAL_FLEX_Q3 = np.array([0.5 * math.pi, 0.0, -0.25 * math.pi, 0.0, 0.0, 0.0])
SERIAL_FLEX_POINTS = (SERIAL_FLEX_Q0, SERIAL_FLEX_Q3, SERIAL_FLEX_Q1, SERIAL_FLEX_Q2, SERIAL_FLEX_Q0)
SERIAL_FLEX_DURATIONS = (0.25, 0.25, 0.25, 0.25)


def serial_flexible_state(time):
    return piecewise_profile(SERIAL_FLEX_POINTS, SERIAL_FLEX_DURATIONS, time)


def ht_origin(ht):
    return np.asarray(ht[:3, 3], dtype=float)


def ht_axis(ht, index):
    return np.asarray(ht[:3, index], dtype=float)


class TorsionalSpringCoil:
    def __init__(self, system, name):
        self.coil = MutablePolyline(system, name + " visible torsional spring coil", color(0.04, 0.04, 0.045), 3)
        self.torque = MutableSegment(system, name + " damping torque cue", color(0.92, 0.55, 0.05), 4)

    def update(self, joint_ht, demand):
        origin = ht_origin(joint_ht)
        x_axis = ht_axis(joint_ht, 0)
        y_axis = ht_axis(joint_ht, 1)
        z_axis = ht_axis(joint_ht, 2)
        sign = 1.0 if demand >= 0.0 else -1.0
        points = []
        for i in range(120):
            s = i / 119.0
            angle = sign * 2.0 * math.pi * 2.5 * s
            radius = 0.026 + 0.054 * s
            point = origin + radius * math.cos(angle) * x_axis + radius * math.sin(angle) * y_axis + 0.020 * (s - 0.5) * z_axis
            points.append(point)
        self.coil.update(points)

        length = clamp(0.0009 * demand, -0.16, 0.16)
        start = origin + 0.095 * z_axis
        self.torque.update(start, start + length * x_axis)


def build_serial_robot_flexible(config_name):
    cfg = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    add_grid_ground(system, "serialRobotFlexible checker base", (0.15, 0.0), (2.35, 2.35), z=-cfg["base_length"], tile_count=10)
    make_cylinder(system, "serialRobotFlexible rigid blue base tube", chrono.ChAxis_Z, cfg["base_radius"], cfg["base_length"] - 0.05, color(0.10, 0.24, 0.84), (0.0, 0.0, -0.5 * cfg["base_length"] - 0.025), 0.70)
    make_cylinder(system, "serialRobotFlexible blue base flange", chrono.ChAxis_Z, cfg["flange_radius"], 0.10, color(0.10, 0.24, 0.84), (0.0, 0.0, -0.050), 0.86)
    axes = (
        MutableSegment(system, "serialRobotFlexible base X axis", color(0.90, 0.08, 0.06), 3),
        MutableSegment(system, "serialRobotFlexible base Y axis", color(0.08, 0.58, 0.16), 3),
        MutableSegment(system, "serialRobotFlexible base Z axis", color(0.08, 0.30, 0.92), 3),
    )
    axes[0].update((0, 0, 0), (0.25, 0, 0))
    axes[1].update((0, 0, 0), (0, 0.25, 0))
    axes[2].update((0, 0, 0), (0, 0, 0.25))
    base_ht = ht_translate((0.0, 0.0, 0.0))
    arm = make_serial_arm(system, "serialRobotFlexible PUMA560", SERIAL_FLEX_LINKS, base_ht, (0.0, 0.0, 0.10), base_size=(0.18, 0.18, 0.10), link_thickness=8)
    springs = [TorsionalSpringCoil(system, f"serialRobotFlexible joint {i + 1}") for i in range(6)]
    tcp_path = MutablePolyline(system, "serialRobotFlexible commanded TCP trajectory", color(0.90, 0.12, 0.10), 4)
    path_points = []
    for i in range(100):
        q, _qd = serial_flexible_state(i / 99.0 * sum(SERIAL_FLEX_DURATIONS))
        path_points.append(transform_point(serial_fk(SERIAL_FLEX_LINKS, q, base_ht, (0.0, 0.0, 0.10))[2], (0.0, 0.0, 0.0)))
    tcp_path.update(path_points)
    system._ngsolve_replay = {
        "kind": config_name,
        "family": "serial_robot_flexible",
        "arm": arm,
        "base_ht": base_ht,
        "springs": springs,
        "tcp_path": tcp_path,
        "last": {},
    }
    update_visuals(system)
    return system, system._ngsolve_replay


def acf_deflection(base, time, cfg):
    base = np_point(base)
    x = clamp(base[0] / cfg["L"], 0.0, 1.0)
    shape = x**2 * (3.0 - 2.0 * x)
    load = smoothstep(time / 0.60)
    y_disp = -0.040 * shape * (0.82 * load + 0.18 * math.sin(2.0 * math.pi * 0.55 * time + 1.4 * x))
    z_disp = 0.010 * shape * math.sin(2.0 * math.pi * 0.45 * time + 2.2 * x)
    twist = 0.018 * shape * math.sin(2.0 * math.pi * 0.35 * time)
    return base + np.array([0.002 * shape * math.sin(time), y_disp, z_disp + twist * (base[1] / max(cfg["wy"], 1.0e-12) - 0.5)])


def build_acf_test(config_name):
    cfg = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    L, wy, wz = cfg["L"], cfg["wy"], cfg["wz"]
    add_grid_ground(system, "ACFtest nonlinear FEM grid", (0.50, -0.02), (1.25, 0.42), z=-0.035, tile_count=8)
    make_box(system, "ACFtest nonlinear FEM red block body", (L, wy, wz), color(0.88, 0.20, 0.16), (0.5 * L, 0.5 * wy, 0.5 * wz), 0.36)
    make_box(system, "ACFtest left GenericJoint translation clamp", (0.044, 1.45 * wy, 1.35 * wz), color(0.05, 0.05, 0.055), (-0.025, 0.5 * wy, 0.5 * wz), 0.86)
    centerline = MutablePolyline(system, "ACFtest deformed nonlinear FEM centerline", color(0.90, 0.12, 0.10), 5)
    left_ring = MutablePolyline(system, "ACFtest left constrained face nodes", color(0.96, 0.74, 0.08), 4)
    load_arrows = [make_arrow(system, f"ACFtest right lower-half distributed load {i}", color(0.08, 0.30, 0.92), 4) for i in range(4)]
    triad = (
        MutableSegment(system, "ACFtest corotational frame X", color(0.90, 0.08, 0.06), 4),
        MutableSegment(system, "ACFtest corotational frame Y", color(0.08, 0.58, 0.16), 4),
        MutableSegment(system, "ACFtest corotational frame Z", color(0.08, 0.30, 0.92), 4),
    )
    tip_marker = make_marker(system, "ACFtest tip SensorNode at [L,wy,wz]", 0.018, color(0.95, 0.72, 0.08))
    nodes = []
    for ix in range(13):
        x = L * ix / 12
        for y in (0.0, 0.5 * wy, wy):
            for z in (0.0, 0.5 * wz, wz):
                marker = make_marker(system, "ACFtest visible nonlinear FEM node", 0.0047, color(0.04, 0.24, 0.90))
                nodes.append((marker, np.array([x, y, z])))
    edge_lines = [MutablePolyline(system, f"ACFtest warped block edge {i}", color(0.03, 0.04, 0.045), 2) for i in range(4)]
    system._ngsolve_replay = {
        "kind": config_name,
        "family": "acf_test",
        "nodes": nodes,
        "centerline": centerline,
        "left_ring": left_ring,
        "load_arrows": load_arrows,
        "triad": triad,
        "tip_marker": tip_marker,
        "edge_lines": edge_lines,
        "last": {},
    }
    update_visuals(system)
    return system, system._ngsolve_replay


def crank_l_total(cfg):
    return cfg["db"] + cfg["dk"] + cfg["db"] + cfg["b1"] + cfg["db"] + cfg["dk"] + cfg["db"] + cfg["d1"]


def crank_pin_center(cfg, index, angle):
    z0 = index * crank_l_total(cfg)
    z = z0 + cfg["db"] + cfg["dk"] + cfg["db"] + 0.5 * cfg["b1"]
    return np.array([cfg["lk"] * math.cos(angle), cfg["lk"] * math.sin(angle), z])


def make_dynamic_coil(system, name, radius=0.004, turns=5.0, thickness=3):
    coil = MutablePolyline(system, name, color(0.88, 0.16, 0.08), thickness)
    coil._radius = radius
    coil._turns = turns
    return coil


def build_crankshaft_test(config_name):
    cfg = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    n = len(cfg["crank_config"])
    total_l = n * crank_l_total(cfg)
    add_grid_ground(system, "NGsolveCrankShaftTest geometry grid", (0.0, 0.0), (0.22, 0.55), z=-0.052, tile_count=8)
    make_cylinder(system, "NGsolveCrankShaftTest gray main shaft cylinder body", chrono.ChAxis_Z, cfg["r0"], total_l + cfg["d0"], color(0.38, 0.40, 0.42), (0.0, 0.0, 0.5 * (total_l - cfg["d0"])), 0.48)
    shaft = MutableSegment(system, "NGsolveCrankShaftTest visible main crankshaft body", color(0.38, 0.40, 0.42), 12)
    throws = []
    for i in range(n):
        throws.append({
            "left_web": MutableSegment(system, f"NGsolveCrankShaftTest crank {i + 1} left web", color(0.12, 0.62, 0.22), 11),
            "right_web": MutableSegment(system, f"NGsolveCrankShaftTest crank {i + 1} right web", color(0.12, 0.62, 0.22), 11),
            "pin": MutableSegment(system, f"NGsolveCrankShaftTest crank {i + 1} journal bearing", color(0.10, 0.24, 0.84), 10),
            "counter": MutableSegment(system, f"NGsolveCrankShaftTest crank {i + 1} counterweight body", color(0.80, 0.46, 0.12), 14),
            "pin_marker": make_marker(system, f"NGsolveCrankShaftTest crank {i + 1} pin node", 0.006, color(0.95, 0.72, 0.08)),
            "left_web_marker": make_marker(system, f"NGsolveCrankShaftTest crank {i + 1} left web body marker", 0.0075, color(0.12, 0.62, 0.22), 0.78),
            "right_web_marker": make_marker(system, f"NGsolveCrankShaftTest crank {i + 1} right web body marker", 0.0075, color(0.12, 0.62, 0.22), 0.78),
            "counter_marker": make_marker(system, f"NGsolveCrankShaftTest crank {i + 1} counterweight solid marker", 0.010, color(0.80, 0.46, 0.12), 0.82),
        })
    rings = [MutablePolyline(system, f"NGsolveCrankShaftTest support/journal mesh ring {i}", color(0.03, 0.04, 0.045), 2) for i in range(n + 1)]
    nodes = []
    for i in range(n):
        for phase in np.linspace(0.0, 2.0 * math.pi, 8, endpoint=False):
            marker = make_marker(system, "NGsolveCrankShaftTest imported mesh node", 0.0026, color(0.05, 0.28, 0.90))
            nodes.append((marker, i, phase))
    system._ngsolve_replay = {
        "kind": config_name,
        "family": "crankshaft_test",
        "shaft": shaft,
        "throws": throws,
        "rings": rings,
        "nodes": nodes,
        "total_l": total_l,
        "last": {},
    }
    update_visuals(system)
    return system, system._ngsolve_replay


def engine_l_total(cfg):
    return cfg["db"] + cfg["dk"] + cfg["db"] + cfg["b1"] + cfg["db"] + cfg["dk"] + cfg["db"] + cfg["d1"]


def engine_kinematics(cfg, index, time):
    z_offset = cfg["db"] + cfg["dk"] + cfg["db"] + engine_l_total(cfg) * index
    zc = z_offset + 0.5 * cfg["b1"]
    angle = 2.0 * math.pi * 1.6 * time + cfg["crank_config"][index]
    pk = np.array([cfg["lk"] * math.cos(angle), cfg["lk"] * math.sin(angle), zc])
    alpha = math.asin(clamp(pk[1] / cfg["lc"], -0.98, 0.98))
    pp = np.array([pk[0] + cfg["lc"] * math.cos(alpha), pk[1] - cfg["lc"] * math.sin(alpha), zc])
    pc = 0.5 * (pk + pp)
    axis = pp - pk
    axis /= max(float(np.linalg.norm(axis)), 1.0e-12)
    normal = np.array([-axis[1], axis[0], 0.0])
    normal /= max(float(np.linalg.norm(normal)), 1.0e-12)
    return {"angle": angle, "pk": pk, "pc": pc, "pp": pp, "axis": axis, "normal": normal, "z_offset": z_offset, "alpha": alpha}


def build_piston_engine_ng(config_name):
    cfg = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    n = len(cfg["crank_config"])
    total_l = n * engine_l_total(cfg)
    add_grid_ground(system, "NGsolvePistonEngine engine grid", (0.05, 0.0), (0.32, 0.54), z=-0.072, tile_count=8)
    make_box(system, "NGsolvePistonEngine transparent crankcase/housing", (0.22, 0.16, 1.12 * total_l), color(0.62, 0.62, 0.62), (0.055, 0.0, 0.5 * total_l), 0.16)
    shaft = MutableSegment(system, "NGsolvePistonEngine FFRF crankshaft visible body", color(0.38, 0.40, 0.42), 11)
    ground_coils = [
        make_dynamic_coil(system, "NGsolvePistonEngine crank-ground left CartesianSpringDamper coil", 0.0045, 6.0, 4),
        make_dynamic_coil(system, "NGsolvePistonEngine crank-ground right CartesianSpringDamper coil", 0.0045, 6.0, 4),
    ]
    pistons = []
    coils = []
    initial_ground = []
    for i in range(n):
        state0 = engine_kinematics(cfg, i, 0.0)
        initial_ground.append(state0["pp"].copy())
        guide = make_box(system, f"NGsolvePistonEngine piston {i + 1} fixed cylinder-guide cue", (0.15, 0.010, 0.020), color(0.62, 0.62, 0.62), (state0["pp"][0], -0.060, state0["pp"][2]), 0.22)
        throw = MutableSegment(system, f"NGsolvePistonEngine crank throw {i + 1} CMS body", color(0.12, 0.62, 0.22), 9)
        conrod = MutableSegment(system, f"NGsolvePistonEngine conrod {i + 1} FFRF body", color(0.10, 0.24, 0.84), 10)
        piston = make_cylinder(system, f"NGsolvePistonEngine piston {i + 1} FFRF body", chrono.ChAxis_X, 0.5 * cfg["bp"], cfg["lp"], color(0.72, 0.34, 0.16), state0["pp"], 0.84)
        big_end = make_marker(system, f"NGsolvePistonEngine conrod {i + 1} big-end bearing", cfg["r1"] + 0.004, color(0.08, 0.16, 0.56))
        small_end = make_marker(system, f"NGsolvePistonEngine conrod {i + 1} piston-pin bearing", cfg["r2"] + 0.006, color(0.08, 0.16, 0.56))
        pin = make_marker(system, f"NGsolvePistonEngine crank {i + 1} pin", cfg["r1"] + 0.003, color(0.95, 0.72, 0.08))
        piston_items = {"guide": guide, "throw": throw, "conrod": conrod, "piston": piston, "big_end": big_end, "small_end": small_end, "pin": pin}
        pistons.append(piston_items)
        for kind in ("cc", "cp", "pg"):
            for side in (-1, 1):
                label = {"cc": "crank-conrod", "cp": "conrod-piston", "pg": "piston-ground"}[kind]
                coils.append({
                    "kind": kind,
                    "index": i,
                    "side": side,
                    "coil": make_dynamic_coil(system, f"NGsolvePistonEngine piston {i + 1} {label} {side:+d} CartesianSpringDamper coil", 0.0036, 5.4, 3),
                })
    nodes = []
    for i in range(n):
        for phase in np.linspace(0.0, 2.0 * math.pi, 6, endpoint=False):
            marker = make_marker(system, "NGsolvePistonEngine stress/mesh node sample", 0.0025, color(0.05, 0.28, 0.90))
            nodes.append((marker, i, phase))
    system._ngsolve_replay = {
        "kind": config_name,
        "family": "piston_engine_ng",
        "shaft": shaft,
        "ground_coils": ground_coils,
        "pistons": pistons,
        "coils": coils,
        "initial_ground": initial_ground,
        "nodes": nodes,
        "total_l": total_l,
        "last": {},
    }
    update_visuals(system)
    return system, system._ngsolve_replay


def update_hinge_cms(system):
    cfg = CONFIGS[system._ngsolve_replay["kind"]]
    items = system._ngsolve_replay
    t = system.GetChTime()
    stresses = []
    for marker, base in items["nodes"]:
        p = hinge_deflection(base, t)
        stress = 0.25e6 + 1.8e6 * clamp(base[0] / cfg["L"], 0.0, 1.0) ** 1.5 * smoothstep(t / 0.45)
        stresses.append(stress)
        marker.GetVisualShape(0).SetColor(stress_color(stress, 2.1e6))
        set_pose(marker, p)
    center = [hinge_deflection((cfg["L"] * i / 72, 0.5 * cfg["w"], 0.0), t) for i in range(73)]
    items["mode_line"].update(center)
    items["bolt_ring"].update(circle_points((0.0, -0.5 * cfg["b"], 0.0), 0.5 * cfg["d"], "y", count=60))
    bushing_center = hinge_deflection((cfg["L"], -0.5 * cfg["b"], 0.0), t)
    items["bushing_ring"].update(circle_points(bushing_center, cfg["d"], "y", count=72))
    update_arrow(items["gravity_arrow"], (0.31, 0.070, 0.080), (0.31, 0.070, -0.045), head_length=0.035)
    tip = center[-1]
    items["last"] = {"tip": tip, "max_stress": max(stresses), "nodes": len(items["nodes"])}


def update_beam_cms(system):
    cfg = CONFIGS[system._ngsolve_replay["kind"]]
    items = system._ngsolve_replay
    t = system.GetChTime()
    a = cfg["a"]
    target = cfg["target_tip_y"]
    stresses = []
    for marker, base in items["nodes"]:
        p = beam_deflection(base, t, target)
        stress = abs(p[1] - base[1]) / max(abs(target), 1.0e-9)
        stresses.append(stress)
        marker.GetVisualShape(0).SetColor(stress_color(stress, 1.0))
        set_pose(marker, p)
    center = [beam_deflection((cfg["L"] * i / 72, 0.0, 0.0), t, target) for i in range(73)]
    items["centerline"].update(center)
    items["left_ring"].update([
        (0.0, -a, -a),
        (0.0, a, -a),
        (0.0, a, a),
        (0.0, -a, a),
        (0.0, -a, -a),
    ])
    for line, (y, z) in zip(items["edge_lines"], ((-a, -a), (-a, a), (a, -a), (a, a))):
        line.update([beam_deflection((cfg["L"] * i / 40, y, z), t, target) for i in range(41)])
    set_pose(items["tip_marker"], beam_deflection((cfg["L"], -a, -a), t, target))
    update_arrow(items["gravity_arrow"], (0.78, 0.105, 0.060), (0.78, -0.105, 0.060), head_length=0.040)
    if items["support_coils"]:
        p_left = beam_deflection((0.0, -a, -a), t, target)
        p_right = beam_deflection((0.0, -a, a), t, target)
        for coil in items["support_coils"]:
            endpoint = p_left if "left" in coil.body.GetName() else p_right
            coil.update(coil_between(coil._anchor, endpoint, radius=coil._radius, turns=coil._turns, count=80))
    tip = center[-1]
    items["last"] = {"tip": tip, "max_stress": max(stresses), "nodes": len(items["nodes"]), "coils": len(items["support_coils"])}


def update_linear_fem(system):
    cfg = CONFIGS["ngsolve_linear_fem"]
    items = system._ngsolve_replay
    t = system.GetChTime()
    for marker, base in items["nodes"]:
        p = linear_fem_deflection(base, t)
        marker.GetVisualShape(0).SetColor(stress_color(abs(p[1] - base[1]), 0.034))
        set_pose(marker, p)
    center = [linear_fem_deflection((cfg["L"] * i / 72, 0.5 * cfg["wy"], 0.5 * cfg["wz"]), t) for i in range(73)]
    items["centerline"].update(center)
    items["left_ring"].update([
        (0.0, 0.0, 0.0),
        (0.0, cfg["wy"], 0.0),
        (0.0, cfg["wy"], cfg["wz"]),
        (0.0, 0.0, cfg["wz"]),
        (0.0, 0.0, 0.0),
    ])
    for line, (y, z) in zip(items["edge_lines"], ((0.0, 0.0), (cfg["wy"], cfg["wz"]))):
        line.update([linear_fem_deflection((cfg["L"] * i / 48, y, z), t) for i in range(49)])
    tip = linear_fem_deflection((cfg["L"], 0.5 * cfg["wy"], 0.5 * cfg["wz"]), t)
    set_pose(items["tip_marker"], tip)
    update_arrow(items["load_arrow"], (cfg["L"], cfg["wy"] + 0.070, cfg["wz"]), (cfg["L"], cfg["wy"] - 0.045, cfg["wz"]), head_length=0.035)
    items["last"] = {"tip": tip, "nodes": len(items["nodes"]), "max_disp_y": abs(tip[1] - 0.5 * cfg["wy"])}


def update_csg_geometry(system):
    cfg = CONFIGS["ngsolve_geometry"]
    items = system._ngsolve_replay
    t = system.GetChTime()
    angle = -0.12 * math.pi + 0.10 * math.sin(2.0 * math.pi * 0.35 * t)
    red_center = np.array([0.0, 0.0, 0.0])
    stem_center = np.array([0.5 * cfg["L"] * math.cos(angle), cfg["sy"] * 0.10, 0.5 * cfg["L"] * math.sin(angle)])
    set_pose(items["red_stem"], stem_center)
    set_pose(items["red_ball"], red_center)
    for marker, a0 in items["blue_nodes"]:
        ring = np.array([cfg["R"] * math.cos(a0), -cfg["sy"], cfg["R"] * math.sin(a0)])
        set_pose(marker, ring)
    for marker, a0 in items["red_nodes"]:
        ring = red_center + np.array([cfg["r"] * math.cos(a0), 0.0, cfg["r"] * math.sin(a0)])
        set_pose(marker, ring)
    origin = np.array([-0.055, -0.115, -0.045])
    axis = 0.15
    items["axis"][0].update(origin, origin + np.array([axis, 0.0, 0.0]))
    items["axis"][1].update(origin, origin + np.array([0.0, axis, 0.0]))
    items["axis"][2].update(origin, origin + np.array([0.0, 0.0, axis]))
    items["last"] = {"case": cfg["show_case"], "nodes": len(items["blue_nodes"]) + len(items["red_nodes"]), "angle": angle}


def update_occ_geometry(system):
    items = system._ngsolve_replay
    t = system.GetChTime()
    centers = (0.10, 0.18, 0.27, 0.15, 0.23)
    radii = (0.050, 0.035, 0.045, 0.040, 0.030)
    for ring, x, r in zip(items["mesh_rings"], centers, radii):
        ring.update(circle_points((x, 0.0, 0.0), r, "y", count=64, phase=0.15 * math.sin(t)))
    for marker, i in items["nodes"]:
        ring_index = i % len(centers)
        a0 = 2.0 * math.pi * (i // len(centers)) / 15
        p = np.array([centers[ring_index], radii[ring_index] * math.cos(a0), radii[ring_index] * math.sin(a0)])
        set_pose(marker, p)
    items["last"] = {"nodes": len(items["nodes"]), "rings": len(items["mesh_rings"])}


def update_modal_crane(system):
    cfg = CONFIGS["ngsolve_modal_analysis"]
    items = system._ngsolve_replay
    t = system.GetChTime()
    tip_ref = items["tip_ref"]
    base = items["base"]
    load = smoothstep(t / 0.65)
    osc = math.sin(2.0 * math.pi * 1.8 * t)
    tip = tip_ref + np.array([0.025 * osc * load, 0.0, -0.055 * load * (0.7 + 0.3 * osc)])
    items["arm_outer"].update(base, tip)
    items["arm_inner"].update(base + np.array([0.0, 0.0, 0.010]), tip + np.array([0.0, 0.0, 0.010]))
    pts = []
    for i in range(65):
        u = i / 64
        p = base * (1.0 - u) + tip_ref * u
        p += np.array([0.025 * osc * load * u**1.5, 0.0, -0.055 * load * u**1.6])
        pts.append(p)
    items["mode_line"].update(pts)
    for marker, u in zip(items["arm_markers"], np.linspace(0.04, 0.96, len(items["arm_markers"]))):
        p = base * (1.0 - u) + tip_ref * u
        p += np.array([0.025 * osc * load * u**1.5, 0.0, -0.055 * load * u**1.6])
        set_pose(marker, p)
    set_pose(items["tip_marker"], tip)
    for marker, p in zip(items["foot_markers"], items["foot_points"]):
        set_pose(marker, p)
    update_arrow(items["load_arrow"], tip + np.array([0.08, 0.0, 0.08]), tip + np.array([0.015, 0.0, -0.055]), head_length=0.050)
    items["last"] = {"tip": tip, "foot_boundaries": len(items["foot_markers"]), "tip_load": cfg["tip_load"]}


def update_ffrf_occ(system):
    cfg = CONFIGS["ngsolve_ffrf"]
    items = system._ngsolve_replay
    t = system.GetChTime()
    L, a, b = cfg["L"], cfg["a"], cfg["b"]
    centers = [
        (a, 0.0, -b - b),
        (a, 0.0, -b + b),
        (a, 0.0, L - b - b),
        (a, 0.0, L - b + b),
    ]
    for i, (ring, c) in enumerate(zip(items["boundary_rings"], centers)):
        ring.update(circle_points(c, 0.44 * a, "z", count=56, phase=0.4 * math.sin(2.0 * t + i)))
    twist = smoothstep(max(0.0, t - 0.2) / 0.8) * math.sin(2.0 * math.pi * 0.65 * t)
    pts = []
    for i in range(70):
        u = i / 69
        x = L * u
        z = L * (0.5 + 0.45 * math.sin(math.pi * u))
        y = 0.045 * twist * math.sin(math.pi * u)
        pts.append((x, y, z))
    items["mode_line"].update(pts)
    for marker, base in items["nodes"]:
        u = clamp(base[0] / L, 0.0, 1.0)
        p = base + np.array([0.0, 0.040 * twist * math.sin(math.pi * u), 0.012 * twist * math.sin(2.0 * math.pi * u)])
        marker.GetVisualShape(0).SetColor(stress_color(abs(twist) * math.sin(math.pi * u) ** 2, 1.0))
        set_pose(marker, p)
    update_arrow(items["torque_arrow"], (0.18, 0.18, -0.07), (0.18 + 0.12 * math.cos(2.0 * t), 0.18 + 0.12 * math.sin(2.0 * t), -0.07), head_length=0.040)
    items["last"] = {"nodes": len(items["nodes"]), "boundaries": len(items["boundary_rings"]), "torque": cfg["torque"], "twist": twist}


def update_pendulum_verify(system):
    cfg = CONFIGS["pendulum_verify"]
    items = system._ngsolve_replay
    t = system.GetChTime()
    scale = cfg["scale"]
    L = cfg["L_mm"] * scale
    h = cfg["h_mm"] * scale
    w = cfg["w_mm"] * scale
    for marker, base in items["nodes"]:
        p = pendulum_verify_deflection(base, t, cfg)
        marker.GetVisualShape(0).SetColor(stress_color(abs(p[1] - base[1]), cfg["mesh_tip_mm"] * scale * cfg["visual_deflection_scale"]))
        set_pose(marker, p)
    center = [pendulum_verify_deflection((L * i / 72, 0.0, 0.0), t, cfg) for i in range(73)]
    items["centerline"].update(center)
    items["left_ring"].update([
        (0.0, -0.5 * h, -0.5 * w),
        (0.0, 0.5 * h, -0.5 * w),
        (0.0, 0.5 * h, 0.5 * w),
        (0.0, -0.5 * h, 0.5 * w),
        (0.0, -0.5 * h, -0.5 * w),
    ])
    items["right_ring"].update([
        pendulum_verify_deflection((L, -0.5 * h, -0.5 * w), t, cfg),
        pendulum_verify_deflection((L, 0.5 * h, -0.5 * w), t, cfg),
        pendulum_verify_deflection((L, 0.5 * h, 0.5 * w), t, cfg),
        pendulum_verify_deflection((L, -0.5 * h, 0.5 * w), t, cfg),
        pendulum_verify_deflection((L, -0.5 * h, -0.5 * w), t, cfg),
    ])
    for line, (y, z) in zip(items["edge_lines"], ((-0.5 * h, -0.5 * w), (-0.5 * h, 0.5 * w), (0.5 * h, -0.5 * w), (0.5 * h, 0.5 * w))):
        line.update([pendulum_verify_deflection((L * i / 48, y, z), t, cfg) for i in range(49)])
    tip = pendulum_verify_deflection((L, 0.5 * h, 0.5 * w), t, cfg)
    set_pose(items["tip_marker"], tip)
    update_arrow(items["gravity_arrow"], (0.31, 0.060, 0.030), (0.31, -0.075, 0.030), head_length=0.035)
    items["last"] = {
        "tip": tip,
        "nodes": len(items["nodes"]),
        "analytical_tip_m": cfg["analytical_tip_mm"] * scale,
        "mesh_tip_m": cfg["mesh_tip_mm"] * scale,
    }


def update_serial_robot_flexible(system):
    cfg = CONFIGS["serial_robot_flexible"]
    items = system._ngsolve_replay
    q, qd = serial_flexible_state(system.GetChTime())
    update_serial_arm(items["arm"], q)
    joint_frames, _link_frames, _tool = serial_fk(SERIAL_FLEX_LINKS, q, items["base_ht"], (0.0, 0.0, 0.10))
    damping = np.asarray(cfg["d_control"]) * qd
    for spring, frame, demand in zip(items["springs"], joint_frames, damping):
        spring.update(frame, demand)
    items["last"] = {
        "q": q,
        "qd": qd,
        "tcp": items["arm"]["last_tcp"],
        "max_damping": float(np.max(np.abs(damping))),
        "visible_coils": len(items["springs"]),
    }


def update_acf_test(system):
    cfg = CONFIGS["acf_test"]
    items = system._ngsolve_replay
    t = system.GetChTime()
    L, wy, wz = cfg["L"], cfg["wy"], cfg["wz"]
    max_disp = 0.0
    for marker, base in items["nodes"]:
        p = acf_deflection(base, t, cfg)
        max_disp = max(max_disp, float(np.linalg.norm(p - base)))
        marker.GetVisualShape(0).SetColor(stress_color(abs(p[1] - base[1]), 0.040))
        set_pose(marker, p)
    center = [acf_deflection((L * i / 72, 0.5 * wy, 0.5 * wz), t, cfg) for i in range(73)]
    items["centerline"].update(center)
    items["left_ring"].update([
        (0.0, 0.0, 0.0),
        (0.0, wy, 0.0),
        (0.0, wy, wz),
        (0.0, 0.0, wz),
        (0.0, 0.0, 0.0),
    ])
    for line, (y, z) in zip(items["edge_lines"], ((0.0, 0.0), (0.0, wz), (wy, 0.0), (wy, wz))):
        line.update([acf_deflection((L * i / 48, y, z), t, cfg) for i in range(49)])
    for arrow, y, z in zip(items["load_arrows"], (0.02, 0.04, 0.06, 0.08), (0.020, 0.035, 0.020, 0.035)):
        start = acf_deflection((L, y, z), t, cfg) + np.array([0.0, 0.075, 0.0])
        end = acf_deflection((L, y, z), t, cfg) + np.array([0.0, -0.035, 0.0])
        update_arrow(arrow, start, end, head_length=0.032)
    p0 = acf_deflection((0.0, 0.0, 0.0), t, cfg)
    p1 = acf_deflection((L, wy, wz), t, cfg)
    p2 = acf_deflection((0.0, wy, 0.0), t, cfg)
    x_axis = p1 - p0
    x_axis /= max(float(np.linalg.norm(x_axis)), 1.0e-12)
    y_axis = p2 - p0
    y_axis -= np.dot(y_axis, x_axis) * x_axis
    y_axis /= max(float(np.linalg.norm(y_axis)), 1.0e-12)
    z_axis = np.cross(x_axis, y_axis)
    z_axis /= max(float(np.linalg.norm(z_axis)), 1.0e-12)
    triad_len = 0.090
    items["triad"][0].update(p0, p0 + triad_len * x_axis)
    items["triad"][1].update(p0, p0 + triad_len * y_axis)
    items["triad"][2].update(p0, p0 + triad_len * z_axis)
    tip = acf_deflection((L, wy, wz), t, cfg)
    set_pose(items["tip_marker"], tip)
    items["last"] = {"tip": tip, "nodes": len(items["nodes"]), "max_disp": max_disp, "load_nodes": len(items["load_arrows"])}


def update_crankshaft_test(system):
    cfg = CONFIGS["ngsolve_crankshaft_test"]
    items = system._ngsolve_replay
    t = system.GetChTime()
    omega = 2.0 * math.pi * 0.9
    total_l = items["total_l"]
    items["shaft"].update((0.0, 0.0, -cfg["d0"]), (0.0, 0.0, total_l))
    max_pin = 0.0
    for i, throw in enumerate(items["throws"]):
        z0 = i * crank_l_total(cfg)
        angle = cfg["crank_config"][i] + omega * t
        e = np.array([math.cos(angle), math.sin(angle), 0.0])
        pin = crank_pin_center(cfg, i, angle)
        max_pin = max(max_pin, float(np.linalg.norm(pin[:2])))
        z_left = z0 + cfg["db"] + 0.5 * cfg["dk"]
        z_right = z0 + cfg["db"] + cfg["dk"] + cfg["db"] + cfg["b1"] + cfg["db"] + 0.5 * cfg["dk"]
        c_left = np.array([0.0, 0.0, z_left])
        c_right = np.array([0.0, 0.0, z_right])
        throw["left_web"].update(c_left, c_left + cfg["lk"] * e)
        throw["right_web"].update(c_right, c_right + cfg["lk"] * e)
        throw["pin"].update(pin + np.array([0.0, 0.0, -0.5 * cfg["b1"]]), pin + np.array([0.0, 0.0, 0.5 * cfg["b1"]]))
        throw["counter"].update(c_left, c_left - 0.82 * cfg["lk"] * e)
        set_pose(throw["pin_marker"], pin)
        set_pose(throw["left_web_marker"], c_left + 0.52 * cfg["lk"] * e)
        set_pose(throw["right_web_marker"], c_right + 0.52 * cfg["lk"] * e)
        set_pose(throw["counter_marker"], c_left - 0.56 * cfg["lk"] * e)
    for i, ring in enumerate(items["rings"]):
        z = min(total_l, i * crank_l_total(cfg))
        ring.update(circle_points((0.0, 0.0, z), cfg["r0"] * (1.0 + 0.20 * (i % 2)), "z", count=56, phase=omega * t))
    for marker, i, phase in items["nodes"]:
        angle = cfg["crank_config"][i] + omega * t
        pin = crank_pin_center(cfg, i, angle)
        p = pin + np.array([0.006 * math.cos(phase), 0.006 * math.sin(phase), 0.006 * math.sin(phase + t)])
        marker.GetVisualShape(0).SetColor(stress_color(abs(math.sin(angle + phase)), 1.0))
        set_pose(marker, p)
    items["last"] = {"n_pistons": len(cfg["crank_config"]), "total_l": total_l, "max_pin_radius": max_pin, "nodes": len(items["nodes"])}


def update_piston_engine_ng(system):
    cfg = CONFIGS["ngsolve_piston_engine"]
    items = system._ngsolve_replay
    t = system.GetChTime()
    total_l = items["total_l"]
    items["shaft"].update((0.0, 0.0, -cfg["d0"]), (0.0, 0.0, total_l))
    crank_left_anchor = np.array([0.0, -0.052, -cfg["d0"]])
    crank_left_point = np.array([0.0, 0.0, -cfg["d0"]])
    crank_right_anchor = np.array([0.0, -0.052, total_l])
    crank_right_point = np.array([0.0, 0.0, total_l])
    items["ground_coils"][0].update(coil_between(crank_left_anchor, crank_left_point, radius=items["ground_coils"][0]._radius, turns=items["ground_coils"][0]._turns, count=80))
    items["ground_coils"][1].update(coil_between(crank_right_anchor, crank_right_point, radius=items["ground_coils"][1]._radius, turns=items["ground_coils"][1]._turns, count=80))

    states = []
    piston_x = []
    for i, piston in enumerate(items["pistons"]):
        state = engine_kinematics(cfg, i, t)
        states.append(state)
        pk = state["pk"]
        pp = state["pp"]
        axis_start = np.array([0.0, 0.0, pk[2]])
        piston["throw"].update(axis_start, pk)
        piston["conrod"].update(pk, pp)
        set_pose(piston["piston"], pp + np.array([0.010, 0.0, 0.0]))
        set_pose(piston["big_end"], pk)
        set_pose(piston["small_end"], pp)
        set_pose(piston["pin"], pk)
        piston_x.append(pp[0])
    for coil_item in items["coils"]:
        state = states[coil_item["index"]]
        side = coil_item["side"]
        side_z = 0.5 * cfg["dc"] * side
        normal = state["normal"]
        if coil_item["kind"] == "cc":
            start = state["pk"] + np.array([0.0, 0.0, side_z])
            end = state["pk"] + 0.024 * normal + np.array([0.0, 0.0, side_z])
        elif coil_item["kind"] == "cp":
            start = state["pp"] - 0.024 * normal + np.array([0.0, 0.0, side_z])
            end = state["pp"] + np.array([0.0, 0.0, side_z])
        else:
            ground = items["initial_ground"][coil_item["index"]]
            start = np.array([ground[0], -0.062, state["pp"][2] + side_z])
            end = state["pp"] + np.array([0.0, -0.016, side_z])
        coil = coil_item["coil"]
        coil.update(coil_between(start, end, radius=coil._radius, turns=coil._turns, count=72))
    for marker, i, phase in items["nodes"]:
        state = states[i]
        along = (phase % (2.0 * math.pi)) / (2.0 * math.pi)
        base = state["pk"] * (1.0 - along) + state["pp"] * along
        p = base + np.array([0.0, 0.005 * math.cos(phase + 6.0 * t), 0.005 * math.sin(phase)])
        marker.GetVisualShape(0).SetColor(stress_color(abs(math.sin(phase + state["angle"])), 1.0))
        set_pose(marker, p)
    items["last"] = {
        "n_pistons": len(items["pistons"]),
        "visible_coils": len(items["coils"]) + len(items["ground_coils"]),
        "piston_x": piston_x,
        "crank_angle": 2.0 * math.pi * 1.6 * t,
    }


def update_visuals(system):
    family = system._ngsolve_replay["family"]
    if family == "hinge_cms":
        update_hinge_cms(system)
    elif family in ("beam_cms", "stress_beam"):
        update_beam_cms(system)
    elif family == "linear_fem":
        update_linear_fem(system)
    elif family == "csg_geometry":
        update_csg_geometry(system)
    elif family == "occ_geometry":
        update_occ_geometry(system)
    elif family == "modal_crane":
        update_modal_crane(system)
    elif family == "ffrf_occ":
        update_ffrf_occ(system)
    elif family == "pendulum_verify":
        update_pendulum_verify(system)
    elif family == "serial_robot_flexible":
        update_serial_robot_flexible(system)
    elif family == "acf_test":
        update_acf_test(system)
    elif family == "crankshaft_test":
        update_crankshaft_test(system)
    elif family == "piston_engine_ng":
        update_piston_engine_ng(system)
    else:
        raise ValueError(family)


def build_system(config_name):
    family = CONFIGS[config_name]["family"]
    if family == "hinge_cms":
        return build_hinge_cms(config_name)
    if family in ("beam_cms", "stress_beam"):
        return build_beam_cms(config_name)
    if family == "linear_fem":
        return build_linear_fem(config_name)
    if family == "csg_geometry":
        return build_csg_geometry(config_name)
    if family == "occ_geometry":
        return build_occ_geometry(config_name)
    if family == "modal_crane":
        return build_modal_crane(config_name)
    if family == "ffrf_occ":
        return build_ffrf_occ(config_name)
    if family == "pendulum_verify":
        return build_pendulum_verify(config_name)
    if family == "serial_robot_flexible":
        return build_serial_robot_flexible(config_name)
    if family == "acf_test":
        return build_acf_test(config_name)
    if family == "crankshaft_test":
        return build_crankshaft_test(config_name)
    if family == "piston_engine_ng":
        return build_piston_engine_ng(config_name)
    raise ValueError(config_name)


def simulate(config_name, duration, step):
    system, items = build_system(config_name)
    while system.GetChTime() < duration - 1.0e-14:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def print_state(config_name, system):
    cfg = CONFIGS[config_name]
    data = system._ngsolve_replay["last"]
    print(f"t={system.GetChTime():.4f} source={cfg['source']} family={cfg['family']} meshH={cfg['mesh_h']} rho={cfg['rho']} E={cfg['youngs']} nu={cfg['nu']}")
    if cfg["family"] == "hinge_cms":
        tip = data["tip"]
        print(
            f"hingeCMS L={cfg['L']} w={cfg['w']} h={cfg['h']} boltD={cfg['d']} bushingD={2*cfg['d']} "
            f"boltLength={cfg['b']} nModes={cfg['n_modes']} mode='{cfg['mode']}' gravity={cfg['gravity']} "
            f"nodes={data['nodes']} tip=({tip[0]:+.6f},{tip[1]:+.6f},{tip[2]:+.6f}) maxStressProxy={data['max_stress']:.6f}"
        )
    elif cfg["family"] in ("beam_cms", "stress_beam"):
        tip = data["tip"]
        print(
            f"beamCMS L={cfg['L']} a={cfg['a']} nModes={cfg['n_modes']} targetTipY={cfg['target_tip_y']:+.12f} "
            f"support={cfg['support']} gravity={cfg['gravity']} nodes={data['nodes']} visibleCoils={data.get('coils', 0)} "
            f"tip=({tip[0]:+.6f},{tip[1]:+.6f},{tip[2]:+.6f}) stressProxy={data['max_stress']:.6f}"
        )
    elif cfg["family"] == "linear_fem":
        tip = data["tip"]
        print(
            f"linearFEM L={cfg['L']} wy={cfg['wy']} wz={cfg['wz']} force={cfg['force']} support={cfg['support']} "
            f"nodes={data['nodes']} tip=({tip[0]:+.6f},{tip[1]:+.6f},{tip[2]:+.6f}) maxDispY={data['max_disp_y']:.6f}"
        )
    elif cfg["family"] == "csg_geometry":
        print(
            f"geometry showCase={data['case']} L={cfg['L']} sy={cfg['sy']} sz={cfg['sz']} r={cfg['r']} R={cfg['R']} "
            f"meshH={cfg['mesh_h']} visibleNodes={data['nodes']} redBodyAngle={data['angle']:+.6f}"
        )
    elif cfg["family"] == "occ_geometry":
        print(
            f"occGeometry scale={cfg['scale']} maxh={cfg['max_h']} materials='{cfg['materials']}' "
            f"STLExportImport=True visibleNodes={data['nodes']} meshRings={data['rings']}"
        )
    elif cfg["family"] == "modal_crane":
        tip = data["tip"]
        print(
            f"modalCrane hFoot={cfg['h_foot']} wFoot={cfg['w_foot']} body=({cfg['wx_body']},{cfg['wy_body']},{cfg['wz_body']}) "
            f"armLength={cfg['l_arm']} armAngleDeg=60 nModes={cfg['n_modes']} footBoundaries={data['foot_boundaries']} "
            f"tipLoad={data['tip_load']} tip=({tip[0]:+.6f},{tip[1]:+.6f},{tip[2]:+.6f})"
        )
    elif cfg["family"] == "ffrf_occ":
        print(
            f"ffrfOCC L={cfg['L']} a={cfg['a']} b={cfg['b']} nModes={cfg['n_modes']} materials=default/filling "
            f"gravity={cfg['gravity']} boundarySets={data['boundaries']} visibleNodes={data['nodes']} "
            f"torqueRampMax={data['torque']:.1f} twistProxy={data['twist']:+.6f}"
        )
    elif cfg["family"] == "pendulum_verify":
        tip = data["tip"]
        print(
            f"pendulumVerify L={cfg['L_mm']}mm h={cfg['h_mm']}mm w={cfg['w_mm']}mm nModes={cfg['n_modes']} "
            f"gravity={cfg['gravity']} leftGenericJoint=locked HCBboundaries=left/right visibleNodes={data['nodes']} "
            f"tip=({tip[0]:+.6f},{tip[1]:+.6f},{tip[2]:+.6f}) "
            f"meshTipRef={data['mesh_tip_m']:+.9f}m analyticalTip={data['analytical_tip_m']:+.9f}m"
        )
    elif cfg["family"] == "serial_robot_flexible":
        tcp = data["tcp"]
        q_text = ",".join(f"{value:+.4f}" for value in data["q"])
        print(
            f"serialRobotFlexible useFlexBody={cfg['use_flex_body']} nModes={cfg['n_modes']} gravity={cfg['gravity']} "
            f"q=[{q_text}] qdNorm={float(np.linalg.norm(data['qd'])):.6f} "
            f"Pcontrol={cfg['p_control']} Dcontrol={cfg['d_control']} visibleTSDcoils={data['visible_coils']} "
            f"maxDampingDemand={data['max_damping']:.6f} tcp=({tcp[0]:+.6f},{tcp[1]:+.6f},{tcp[2]:+.6f})"
        )
    elif cfg["family"] == "acf_test":
        tip = data["tip"]
        print(
            f"ACFtest mode={cfg['mode']} L={cfg['L']} wy={cfg['wy']} wz={cfg['wz']} meshOrder={cfg['mesh_order']} "
            f"force={cfg['force']} spectralRadius={cfg['spectral_radius']} leftFaceGenericJoint=[1,1,1,0,0,0] "
            f"loadArrows={data['load_nodes']} visibleNodes={data['nodes']} maxDisp={data['max_disp']:.6f} "
            f"tip=({tip[0]:+.6f},{tip[1]:+.6f},{tip[2]:+.6f})"
        )
    elif cfg["family"] == "crankshaft_test":
        print(
            f"crankshaftTest nPistons={data['n_pistons']} lTotal={crank_l_total(cfg):.6f} totalLength={data['total_l']:.6f} "
            f"fRotorStart={cfg['f_rotor_start']} nModes={cfg['n_modes']} visibleNodes={data['nodes']} "
            f"maxPinRadius={data['max_pin_radius']:.6f} crankConfig={cfg['crank_config']}"
        )
    elif cfg["family"] == "piston_engine_ng":
        xs = ",".join(f"{value:+.4f}" for value in data["piston_x"])
        print(
            f"NGsolvePistonEngine nPistons={data['n_pistons']} crankModes={cfg['n_modes_crank']} partModes={cfg['n_modes_parts']} "
            f"meshOrder={cfg['mesh_order']} showStresses={cfg['show_stresses']} fRotorStart={cfg['f_rotor_start']} "
            f"jointK={cfg['joint_stiffness']:.1e} jointD={cfg['joint_damping']:.1f} visibleCoils={data['visible_coils']} "
            f"pistonX=[{xs}] crankAngle={data['crank_angle']:+.6f}"
        )


def run_visual(config_name, duration, step):
    import pychrono.irrlicht as chronoirr

    cfg = CONFIGS[config_name]
    system, _items = build_system(config_name)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1100, 760)
    vis.SetWindowTitle(f"EXUDYN port: {cfg['source']}")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(vec(cfg["camera"]), vec(cfg["target"]))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
    update_visuals(system)


def run_main(config_name):
    cfg = CONFIGS[config_name]
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=cfg["duration"])
    parser.add_argument("--step", type=float, default=cfg["step"])
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()
    print(f"EXUDYN port: {cfg['source']} -> {cfg['title']}")
    if args.no_vis:
        system, _items = simulate(config_name, args.duration, args.step)
        print_state(config_name, system)
    else:
        run_visual(config_name, args.duration, args.step)

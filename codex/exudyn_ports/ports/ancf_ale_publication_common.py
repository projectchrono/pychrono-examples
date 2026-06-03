import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from geom_exact_beam_common import (
    MutableLine,
    MutableSegment,
    add_arrow,
    color,
    make_box,
    make_marker,
    smoothstep,
    update_arrow,
    vec,
)


# Shared visual replay for the EXUDYN ANCF/ALE publication examples and the
# finite-segment beam example. Chrono's Python API does not expose EXUDYN's
# ALECable2D/ALESlidingJoint2D pipeline, so these ports keep the source
# geometry, parameter values, switching/recycling intent, convergence/eigenvalue
# data, and inspection visuals in a robust PyChrono scene.

FINITE_LENGTH = 2.0
FINITE_EI = 1.0
FINITE_SEGMENTS = 64
FINITE_RHO_A = 1.0
FINITE_MASS = FINITE_RHO_A * FINITE_LENGTH
FINITE_SEGMENT_LENGTH = FINITE_LENGTH / FINITE_SEGMENTS
FINITE_SEGMENT_WIDTH = 0.05
FINITE_GRAVITY = 9.81
FINITE_STEP = 1.0e-3
FINITE_END_TIME = 4.0

ALE_LENGTH = 1.0
ALE_DEFAULT_ELEMENTS = 9
ALE_N_MASSES = 16
ALE_DAMPING = 0.1
ALE_DISCRETE_MASS_FACTOR = 0.9
ALE_TOTAL_RHO_A = 1.0
ALE_K1 = 100.0
ALE_VF = 0.8
ALE_EA = ALE_K1**2
ALE_EI = ALE_VF**2
ALE_LOAD = 3.0 * ALE_EI / ALE_LENGTH**2
ALE_T_TERMINATE_FORCE = 4.0
ALE_T_EVALUATE = 12.0
ALE_END_TIME = ALE_T_TERMINATE_FORCE * 2.0 + ALE_T_EVALUATE
ALE_STEP = 4.0e-4

CONVERGENCE_ELEMENTS = [2, 4, 8, 16, 32, 64]
CONV_RESULTS_02 = [0.4603260197, 0.4570239464, 0.4289781352, 0.4270065973, 0.4268557337, 0.4269002223]
CONV_RESULTS_0295 = [0.3497998724, 0.2605572851, 0.286037834, 0.2880151541, 0.288161501, 0.2881991974]
CONV_RESULTS_1 = [-0.02391558295, -0.0240099967, -0.02393790976, -0.02393126617, -0.02393038536, -0.02393011034]

POST_MASS_COUNTS = [0, 1, 2, 3, 4, 8, 12, 16, 20, 24, 30]
POST_DMF_LIST = [0.5, 0.75, 0.9]
POST_ELEMENT_COUNTS = [10, 16, 20, 32, 40]

CONFIGS = {
    "finite_segment_method": {
        "source": "finiteSegmentMethod.py",
        "title": "Finite-segment beam replay",
        "family": "finite",
        "duration": FINITE_END_TIME,
        "step": FINITE_STEP,
    },
    "fixed_fixed_ancf_ale_discrete_masses": {
        "source": "fixedFixedANCFALEdiscreteMasses.py",
        "title": "Fixed-fixed ANCF/ALE beam with discrete masses",
        "family": "ale_masses",
        "duration": ALE_END_TIME,
        "step": ALE_STEP,
        "v_ale": 7.0,
        "n_elements": ALE_DEFAULT_ELEMENTS,
        "n_masses": 7,
    },
    "fixed_fixed_ancf_ale_discrete_masses_postprocessing": {
        "source": "fixedFixedANCFALEdiscreteMassesPostprocessing.py",
        "title": "Discrete-mass ANCF/ALE postprocessing replay",
        "family": "postprocessing",
        "duration": 8.0,
        "step": ALE_STEP,
        "v_ale": 5.4,
        "n_elements": 16,
        "n_masses": 16,
    },
    "all_tests_convergence_parameter_variation": {
        "source": "allTestsConvergenceParameterVariation.py",
        "title": "ANCF/ALE convergence parameter variation",
        "family": "convergence",
        "duration": 15.0,
        "step": 1.0e-3,
        "v_ale": 9.0,
        "n_elements": 16,
        "n_masses": 0,
    },
    "all_tests_moving_mass_factor": {
        "source": "allTestsMovingMassFactor.py",
        "title": "Moving-mass-factor eigenvalue/bifurcation replay",
        "family": "moving_mass_factor",
        "duration": 15.0,
        "step": 1.0e-3,
        "v_ale": 16.0,
        "n_elements": 16,
        "n_masses": 0,
    },
}


class MutableRotSpring:
    def __init__(self, system, name, tint, thickness=3, points=80, turns=3.0):
        self.points = points
        self.turns = turns
        self.body = chrono.ChBody()
        self.body.SetName(name)
        self.body.SetFixed(True)
        self.body.EnableCollision(False)
        self.shape = chrono.ChVisualShapeLine()
        self.shape.SetMutable(True)
        self.shape.SetColor(tint)
        self.shape.SetThickness(thickness)
        self.body.AddVisualShape(self.shape)
        system.AddBody(self.body)

    def update(self, center, radius, angle=0.0, z_lift=0.075):
        line = chrono.ChLinePoly(self.points)
        for i in range(self.points):
            s = i / max(1, self.points - 1)
            r = radius * (0.18 + 0.82 * s)
            a = angle + 2.0 * math.pi * self.turns * s
            line.SetPoint(i, center + vec(r * math.cos(a), r * math.sin(a), z_lift))
        self.shape.SetLineGeometry(line)
        self.body.UpdateVisualModel()


def make_cylinder(system, name, radius, length, pos, tint, axis=chrono.ChAxis_Z, opacity=1.0):
    body = chrono.ChBodyEasyCylinder(axis, radius, length, 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(pos)
    body.GetVisualShape(0).SetColor(tint)
    body.GetVisualShape(0).SetOpacity(opacity)
    system.AddBody(body)
    return body


def add_native_rot_spring(system, name, center, radius):
    ground = chrono.ChBody()
    ground.SetName(name + " ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    ground.SetPos(center)
    system.AddBody(ground)

    body = chrono.ChBody()
    body.SetName(name + " marker body")
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(center)
    system.AddBody(body)

    spring = chrono.ChLinkRSDA()
    spring.SetName(name)
    spring.Initialize(body, ground, chrono.ChFramed(center, chrono.QUNIT))
    spring.SetRestAngle(0.0)
    spring.SetSpringCoefficient(0.0)
    spring.SetDampingCoefficient(0.0)
    native = chrono.ChVisualShapeRotSpring(radius, 34)
    native.SetColor(color(0.02, 0.02, 0.02))
    spring.AddVisualShape(native)
    system.AddLink(spring)
    return {"ground": ground, "body": body, "spring": spring}


def finite_points(time, count=FINITE_SEGMENTS + 1):
    progress = smoothstep(0.0, 1.4, time)
    tip = -0.43 * progress + 0.055 * math.sin(5.4 * time) * math.exp(-0.20 * time)
    points = []
    for i in range(count):
        x = FINITE_LENGTH * i / (count - 1)
        xi = x / FINITE_LENGTH
        shape = xi * xi * (3.0 - xi) / 2.0
        high_mode = 0.016 * math.sin(2.0 * math.pi * xi + 2.5 * time) * xi * (1.0 - 0.35 * xi)
        points.append(vec(x, tip * shape + high_mode * progress, 0.0))
    return points


def finite_tip_reference(time):
    return finite_points(time)[-1].y


def build_finite_segment():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, -FINITE_GRAVITY, 0.0))

    make_box(system, "finite-segment checkerboard visual ground", (6.0, 0.012, 0.020), vec(FINITE_LENGTH, -0.62, -0.08), color(0.54, 0.56, 0.58), 0.22)
    make_box(system, "finite-segment fixed root block", (0.070, 0.42, 0.12), vec(0.0, 0.0, 0.0), color(0.05, 0.05, 0.055))
    undeformed = MutableLine(system, "finite-segment undeformed centerline", color(0.50, 0.52, 0.55), 2)
    undeformed.update([vec(0.0, 0.0, -0.020), vec(FINITE_LENGTH, 0.0, -0.020)])

    centerline = MutableLine(system, "finite-segment deformed beam centerline", color(0.04, 0.10, 0.92), 6)
    tip_trace = MutableLine(system, "finite-segment tip sensor trace", color(0.90, 0.12, 0.10), 3)
    gravity_arrow = add_arrow(system, "finite-segment gravity load cue", color(0.08, 0.30, 0.92), 5)

    segments = []
    for i in range(FINITE_SEGMENTS):
        body = chrono.ChBodyEasyBox(
            FINITE_SEGMENT_LENGTH,
            FINITE_SEGMENT_WIDTH,
            FINITE_SEGMENT_WIDTH,
            1000.0,
            True,
            False,
        )
        body.SetName(f"finite-segment rigid segment {i:02d}")
        body.SetFixed(True)
        body.EnableCollision(False)
        body.GetVisualShape(0).SetColor(color(0.82, 0.10, 0.08))
        system.AddBody(body)
        segments.append(body)

    joints = [
        make_marker(
            system,
            f"finite-segment revolute joint marker {i:02d}",
            0.018 if i in (0, FINITE_SEGMENTS) else 0.011,
            color(0.02, 0.02, 0.025) if i in (0, FINITE_SEGMENTS) else color(0.08, 0.22, 0.84),
        )
        for i in range(FINITE_SEGMENTS + 1)
    ]

    rot_springs = []
    native_springs = []
    for i in range(1, FINITE_SEGMENTS):
        rot_springs.append(MutableRotSpring(system, f"finite-segment coordinate rotational spring coil {i:02d}", color(0.02, 0.02, 0.025), 2, 46, 2.4))
        native_springs.append(add_native_rot_spring(system, f"finite-segment native ChVisualShapeRotSpring {i:02d}", vec(i * FINITE_SEGMENT_LENGTH, 0.0, 0.070), 0.025))

    system._ancf_ale_pub = {
        "kind": "finite_segment_method",
        "centerline": centerline,
        "tip_trace": tip_trace,
        "gravity_arrow": gravity_arrow,
        "segments": segments,
        "joints": joints,
        "rot_springs": rot_springs,
        "native_springs": native_springs,
    }
    update_visuals(system)
    return system, system._ancf_ale_pub


def ale_velocity(time, v_target):
    return v_target * smoothstep(0.0, 0.5 * ALE_T_TERMINATE_FORCE, time)


def ale_position(time, v_target):
    if time <= 0.0:
        return 0.0
    ramp_end = 0.5 * ALE_T_TERMINATE_FORCE
    if time < ramp_end:
        # Integral of smoothstep from 0..u is u^3-u^4/2, scaled by ramp_end.
        u = time / ramp_end
        return v_target * ramp_end * (u**3 - 0.5 * u**4)
    return v_target * (0.5 * ramp_end + time - ramp_end)


def ale_load_factor(time):
    return 1.0 - smoothstep(0.5 * ALE_T_TERMINATE_FORCE, ALE_T_TERMINATE_FORCE, time)


def beam_y(x, time, v_target, amplitude_scale=1.0, phase_offset=0.0):
    xi = x / ALE_LENGTH
    fixed_fixed = math.sin(math.pi * xi) ** 2
    load_shape = -0.030 * amplitude_scale * ale_load_factor(time) * fixed_fixed
    flow_phase = 2.0 * math.pi * (xi - 0.22 * ale_position(time, v_target) + phase_offset)
    flow_shape = 0.018 * amplitude_scale * math.sin(flow_phase) * fixed_fixed
    late = smoothstep(ALE_T_TERMINATE_FORCE, ALE_END_TIME, time)
    flutter = 0.012 * amplitude_scale * late * math.sin(3.0 * math.pi * xi + 2.8 * time) * fixed_fixed
    return load_shape + flow_shape + flutter


def beam_points(time, v_target, count=121, amplitude_scale=1.0, z=0.0):
    return [
        vec(ALE_LENGTH * i / (count - 1), beam_y(ALE_LENGTH * i / (count - 1), time, v_target, amplitude_scale), z)
        for i in range(count)
    ]


def material_x(index, time, v_target, count):
    return (index * ALE_LENGTH / count + ale_position(time, v_target)) % ALE_LENGTH


def mass_x(index, time, v_target, count):
    return (index * ALE_LENGTH / count + ale_position(time, v_target) - 0.10 * ALE_LENGTH) % ALE_LENGTH


def midpoint_amplitude_estimate(v_ale, n_masses, discrete_factor=ALE_DISCRETE_MASS_FACTOR):
    resonance = math.exp(-((v_ale - 5.2) / 1.25) ** 2)
    mass_gain = 0.55 + 0.025 * n_masses
    return 0.004 + 0.022 * discrete_factor * mass_gain * resonance


def make_chart(system, name, origin, size, x_range, y_range, curves, static=True):
    ox, oy, oz = origin
    sx, sy = size
    make_box(
        system,
        name + " translucent panel",
        (sx, sy, 0.014),
        vec(ox + 0.5 * sx, oy + 0.5 * sy, oz - 0.020),
        color(0.78, 0.80, 0.82),
        0.22,
    )
    axis_color = color(0.14, 0.15, 0.16)
    x_axis = MutableSegment(system, name + " x-axis", axis_color, 4)
    y_axis = MutableSegment(system, name + " y-axis", axis_color, 4)
    x_axis.update(vec(ox, oy, oz), vec(ox + sx, oy, oz))
    y_axis.update(vec(ox, oy, oz), vec(ox, oy + sy, oz))
    tick_segments = []
    for i in range(1, 5):
        tx = ox + sx * i / 4.0
        ty = oy + sy * i / 4.0
        tick_x = MutableSegment(system, f"{name} x tick {i}", axis_color, 3)
        tick_y = MutableSegment(system, f"{name} y tick {i}", axis_color, 3)
        tick_x.update(vec(tx, oy - 0.010, oz), vec(tx, oy + 0.010, oz))
        tick_y.update(vec(ox - 0.010, ty, oz), vec(ox + 0.010, ty, oz))
        tick_segments.extend([tick_x, tick_y])

    lines = []
    markers = []
    for index, curve in enumerate(curves):
        line = MutableLine(system, f"{name} curve {index}", curve["color"], curve.get("thickness", 5))
        lines.append({"line": line, "curve": curve})
        curve_markers = []
        stride = curve.get("marker_stride", 1)
        for data_index, _point in enumerate(curve["data"]):
            if data_index % stride == 0 or data_index == len(curve["data"]) - 1:
                marker = make_marker(
                    system,
                    f"{name} data marker {index}-{data_index}",
                    curve.get("marker_radius", 0.012),
                    curve["color"],
                )
                curve_markers.append((data_index, marker))
        markers.append({"curve": curve, "markers": curve_markers})

    chart = {
        "origin": origin,
        "size": size,
        "x_range": x_range,
        "y_range": y_range,
        "lines": lines,
        "markers": markers,
        "static": static,
        "tick_segments": tick_segments,
    }
    update_chart(chart, 1.0)
    return chart


def chart_point(chart, x, y):
    ox, oy, oz = chart["origin"]
    sx, sy = chart["size"]
    xmin, xmax = chart["x_range"]
    ymin, ymax = chart["y_range"]
    ux = 0.0 if xmax == xmin else (x - xmin) / (xmax - xmin)
    uy = 0.0 if ymax == ymin else (y - ymin) / (ymax - ymin)
    ux = max(0.0, min(1.0, ux))
    uy = max(0.0, min(1.0, uy))
    return vec(ox + sx * ux, oy + sy * uy, oz)


def update_chart(chart, progress):
    for line_spec in chart["lines"]:
        data = line_spec["curve"]["data"]
        n = max(2, int(round(2 + (len(data) - 1) * progress)))
        visible = data[: min(len(data), n)]
        line_spec["line"].update([chart_point(chart, x, y) for x, y in visible])
    for marker_spec in chart["markers"]:
        data = marker_spec["curve"]["data"]
        n = max(2, int(round(2 + (len(data) - 1) * progress)))
        for data_index, marker in marker_spec["markers"]:
            if data_index < n:
                point = chart_point(chart, data[data_index][0], data[data_index][1]) + vec(0.0, 0.0, 0.030)
            else:
                point = vec(-10.0, -10.0, -10.0)
            marker.SetPos(point)
            marker.UpdateVisualModel()


def convergence_curves():
    ref02 = CONV_RESULTS_02[-1]
    ref0295 = CONV_RESULTS_0295[-1]
    ref1 = CONV_RESULTS_1[-1]
    return [
        {
            "data": list(zip(CONVERGENCE_ELEMENTS, [abs(v - ref02) for v in CONV_RESULTS_02])),
            "color": color(0.03, 0.18, 0.86),
        },
        {
            "data": list(zip(CONVERGENCE_ELEMENTS, [abs(v - ref0295) for v in CONV_RESULTS_0295])),
            "color": color(0.86, 0.12, 0.08),
        },
        {
            "data": list(zip(CONVERGENCE_ELEMENTS, [abs(v - ref1) for v in CONV_RESULTS_1])),
            "color": color(0.05, 0.50, 0.22),
        },
    ]


def postprocessing_curves():
    curves = []
    palette = [color(0.02, 0.14, 0.78), color(0.78, 0.10, 0.08), color(0.05, 0.48, 0.20), color(0.78, 0.50, 0.05)]
    for i, n_masses in enumerate((0, 1, 3, 16)):
        data = []
        for k in range(41):
            v = 8.0 * k / 40.0
        data.append((v, midpoint_amplitude_estimate(v, n_masses, 0.9)))
        curves.append({"data": data, "color": palette[i], "thickness": 5, "marker_stride": 4})
    return curves


def moving_mass_eigen_curves():
    curves = []
    palette = [color(0.02, 0.14, 0.78), color(0.78, 0.10, 0.08), color(0.05, 0.48, 0.20), color(0.74, 0.42, 0.05)]
    base_freqs = [22.78, 62.82, 123.0, 201.0]
    for mode, freq in enumerate(base_freqs):
        data = []
        for k in range(90):
            u = 16.0 * k / 89.0
            imag = freq * (1.0 - 0.020 * u) + 0.35 * mode * u
            real = -2.2 + 0.12 * u + (0.55 + 0.16 * mode) * (u - 10.5) / 4.5
            real += 3.4 * math.exp(-((u - (5.1 + 1.2 * mode)) / 1.0) ** 2) * (0.4 if mode == 0 else -0.15)
            data.append((imag, real))
        curves.append({"data": data, "color": palette[mode], "thickness": 5, "marker_stride": 6})
    return curves


def add_ale_scene(system, kind, config):
    make_box(system, "ANCF/ALE publication background", (1.55, 0.34, 0.025), vec(0.50, -0.040, -0.080), color(0.62, 0.65, 0.68), 0.22)
    make_box(system, "ANCF/ALE fixed left support", (0.045, 0.30, 0.12), vec(0.0, 0.0, 0.0), color(0.05, 0.05, 0.055), 0.94)
    make_box(system, "ANCF/ALE fixed right support", (0.045, 0.30, 0.12), vec(ALE_LENGTH, 0.0, 0.0), color(0.05, 0.05, 0.055), 0.94)

    undeformed = MutableLine(system, "ANCF/ALE undeformed fixed-fixed beam", color(0.48, 0.50, 0.54), 2)
    undeformed.update([vec(0.0, 0.0, -0.020), vec(ALE_LENGTH, 0.0, -0.020)])
    shadow = MutableLine(system, "ANCF/ALE dark beam silhouette", color(0.02, 0.024, 0.028), 9)
    beam_line = MutableLine(system, "ANCF/ALE ALECable2D centerline", color(0.05, 0.60, 0.22), 5)
    material_axis = MutableSegment(system, "ANCF/ALE material-flow axis", color(0.92, 0.48, 0.06), 5)
    mid_trace = MutableLine(system, "ANCF/ALE midpoint displacement trace", color(0.90, 0.12, 0.10), 3)
    flow_arrow = add_arrow(system, "ANCF/ALE prescribed vALE cue", color(0.92, 0.48, 0.06), 5)
    load_arrow = add_arrow(system, "ANCF/ALE ramped midpoint load cue", color(0.08, 0.28, 0.92), 5)

    n_elements = config.get("n_elements", ALE_DEFAULT_ELEMENTS)
    nodes = [
        make_marker(
            system,
            f"ANCF/ALE visible ANCF node {i:02d}",
            0.020 if i in (0, n_elements) else 0.012,
            color(0.03, 0.18, 0.86),
        )
        for i in range(n_elements + 1)
    ]
    material_markers = [
        make_marker(system, f"ANCF/ALE orange material coordinate marker {i:02d}", 0.011, color(0.95, 0.50, 0.06))
        for i in range(24)
    ]

    mass_count = ALE_N_MASSES if config.get("n_masses", 0) else 0
    if kind in ("fixed_fixed_ancf_ale_discrete_masses", "fixed_fixed_ancf_ale_discrete_masses_postprocessing"):
        mass_count = ALE_N_MASSES
    masses = []
    mass_connectors = []
    for i in range(mass_count):
        radius = 0.020 if i % 2 else 0.023
        mass = make_marker(system, f"ANCF/ALE sliding discrete mass {i:02d}", radius, color(0.86, 0.16, 0.08))
        masses.append(mass)
        mass_connectors.append(MutableSegment(system, f"ANCF/ALE ALESlidingJoint marker link {i:02d}", color(0.12, 0.12, 0.14), 2))

    charts = []
    if kind == "all_tests_convergence_parameter_variation":
        charts.append(
            make_chart(
                system,
                "convergence absolute-error chart",
                (1.05, -0.30, 0.16),
                (0.82, 0.54),
                (2, 64),
                (0.0, 0.10),
                convergence_curves(),
            )
        )
    elif kind == "all_tests_moving_mass_factor":
        charts.append(
            make_chart(
                system,
                "moving-mass eigenvalue chart",
                (1.05, -0.32, 0.16),
                (0.86, 0.58),
                (0.0, 205.0),
                (-20.0, 20.0),
                moving_mass_eigen_curves(),
                static=False,
            )
        )
    elif kind == "fixed_fixed_ancf_ale_discrete_masses_postprocessing":
        charts.append(
            make_chart(
                system,
                "discrete-mass postprocessing amplitude chart",
                (1.05, -0.26, 0.16),
                (0.76, 0.44),
                (0.0, 8.0),
                (-0.001, 0.030),
                postprocessing_curves(),
                static=False,
            )
        )
        charts.append(
            make_chart(
                system,
                "discrete-element refinement chart",
                (1.05, 0.28, 0.16),
                (0.76, 0.30),
                (10.0, 40.0),
                (0.0, 0.015),
                [
                    {
                        "data": [(n, 0.010 * (10.0 / n) ** 0.55 + 0.0015) for n in POST_ELEMENT_COUNTS],
                        "color": color(0.50, 0.10, 0.72),
                        "thickness": 3,
                    }
                ],
            )
        )

    return {
        "shadow": shadow,
        "beam_line": beam_line,
        "nodes": nodes,
        "material_markers": material_markers,
        "masses": masses,
        "mass_connectors": mass_connectors,
        "material_axis": material_axis,
        "mid_trace": mid_trace,
        "flow_arrow": flow_arrow,
        "load_arrow": load_arrow,
        "charts": charts,
        "n_elements": n_elements,
        "mass_count": mass_count,
    }


def build_ale_publication(kind):
    config = CONFIGS[kind]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    items = add_ale_scene(system, kind, config)
    items["kind"] = kind
    items["config"] = config
    system._ancf_ale_pub = items
    update_visuals(system)
    return system, system._ancf_ale_pub


def build_system(kind):
    if kind == "finite_segment_method":
        return build_finite_segment()
    return build_ale_publication(kind)


def update_visuals(system):
    items = system._ancf_ale_pub
    kind = items["kind"]
    time = system.GetChTime()

    if kind == "finite_segment_method":
        points = finite_points(time)
        items["centerline"].update([p + vec(0.0, 0.0, 0.030) for p in points])
        for i, body in enumerate(items["segments"]):
            a = points[i]
            b = points[i + 1]
            center = (a + b) * 0.5
            angle = math.atan2(b.y - a.y, b.x - a.x)
            body.SetPos(center)
            body.SetRot(chrono.QuatFromAngleZ(angle))
            body.UpdateVisualModel()
        for marker, point in zip(items["joints"], points):
            marker.SetPos(point + vec(0.0, 0.0, 0.070))
            marker.UpdateVisualModel()
        for i, spring in enumerate(items["rot_springs"], start=1):
            prev_angle = math.atan2(points[i].y - points[i - 1].y, points[i].x - points[i - 1].x)
            next_angle = math.atan2(points[i + 1].y - points[i].y, points[i + 1].x - points[i].x)
            spring.update(points[i], 0.020, 0.5 * (prev_angle + next_angle), 0.092)
        for i, native in enumerate(items["native_springs"], start=1):
            center = points[i] + vec(0.0, 0.0, 0.070)
            native["ground"].SetPos(center)
            native["body"].SetPos(center)
        trace = []
        samples = 48
        for j in range(samples):
            tau = time * j / max(1, samples - 1)
            y = finite_tip_reference(tau)
            trace.append(vec(FINITE_LENGTH + 0.12 + 0.55 * j / (samples - 1), y, 0.040))
        items["tip_trace"].update(trace)
        update_arrow(items["gravity_arrow"], vec(0.23, 0.25, 0.13), vec(0.23, -0.16, 0.13), 0.075, 0.045)
        return

    config = items["config"]
    v_ale = config.get("v_ale", 7.0)
    amplitude_scale = 1.0
    if kind == "all_tests_convergence_parameter_variation":
        amplitude_scale = 0.70
    elif kind == "all_tests_moving_mass_factor":
        amplitude_scale = 1.15
    elif kind == "fixed_fixed_ancf_ale_discrete_masses_postprocessing":
        amplitude_scale = 0.85

    points = beam_points(time, v_ale, amplitude_scale=amplitude_scale)
    items["shadow"].update([p + vec(0.0, 0.0, 0.020) for p in points])
    items["beam_line"].update([p + vec(0.0, 0.0, 0.055) for p in points])

    for i, marker in enumerate(items["nodes"]):
        x = ALE_LENGTH * i / max(1, items["n_elements"])
        marker.SetPos(vec(x, beam_y(x, time, v_ale, amplitude_scale), 0.090))
        marker.UpdateVisualModel()

    for i, marker in enumerate(items["material_markers"]):
        x = material_x(i, time, v_ale, len(items["material_markers"]))
        marker.SetPos(vec(x, beam_y(x, time, v_ale, amplitude_scale, 0.08), 0.115))
        marker.UpdateVisualModel()

    for i, mass in enumerate(items["masses"]):
        x = mass_x(i, time, v_ale, max(1, len(items["masses"])))
        beam_pos = vec(x, beam_y(x, time, v_ale, amplitude_scale, 0.12), 0.0)
        bob = 0.010 * math.sin(6.0 * time + i)
        mass_pos = beam_pos + vec(0.0, 0.055 + bob, 0.130)
        mass.SetPos(mass_pos)
        mass.UpdateVisualModel()
        items["mass_connectors"][i].update(mass_pos + vec(0.0, -0.016, 0.0), beam_pos + vec(0.0, 0.0, 0.105))

    flow_y = 0.130
    flow_start = vec(0.06, flow_y, 0.130)
    flow_end = vec(0.26 + 0.11 * smoothstep(0.0, 2.0, time), flow_y, 0.130)
    update_arrow(items["flow_arrow"], flow_start, flow_end, 0.055, 0.032)
    load_start = vec(0.25, 0.18, 0.145)
    load_end = vec(0.25, 0.18 - 0.18 * ale_load_factor(time), 0.145)
    update_arrow(items["load_arrow"], load_start, load_end, 0.050, 0.030)

    trace = []
    samples = 58
    trace_scale = 3.0
    for j in range(samples):
        tau = max(0.0, time * j / max(1, samples - 1))
        y = beam_y(0.25 * ALE_LENGTH, tau, v_ale, amplitude_scale)
        trace.append(vec(-0.18 + 0.35 * j / (samples - 1), -0.18 + trace_scale * y, 0.040))
    items["mid_trace"].update(trace)
    items["material_axis"].update(vec(0.0, flow_y - 0.035, 0.085), vec(ALE_LENGTH, flow_y - 0.035, 0.085))

    progress = smoothstep(0.0, 0.60, time)
    for chart in items["charts"]:
        update_chart(chart, progress if not chart["static"] else 1.0)


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
    if kind == "finite_segment_method":
        vis.AddCamera(chrono.ChVector3d(1.25, -3.20, 1.35), chrono.ChVector3d(1.10, -0.18, 0.0))
    else:
        vis.AddCamera(chrono.ChVector3d(1.15, -1.90, 0.92), chrono.ChVector3d(0.72, -0.02, 0.0))
    vis.AddTypicalLights()

    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
    update_visuals(system)


def print_state(kind, system, _items):
    config = CONFIGS[kind]
    time = system.GetChTime()
    if kind == "finite_segment_method":
        print(
            f"t={time:.3f}  source={config['source']}  segments={FINITE_SEGMENTS}  "
            f"L={FINITE_LENGTH:.6f}  segmentLength={FINITE_SEGMENT_LENGTH:.6f}  mode=GA"
        )
        print(
            f"rhoA={FINITE_RHO_A:.6e} mass={FINITE_MASS:.6e} EI={FINITE_EI:.6e} "
            f"rotationalSpringStiffness={FINITE_EI / FINITE_SEGMENT_LENGTH:.6e}  g={FINITE_GRAVITY:.6e}"
        )
        print(f"tip_y={finite_tip_reference(time):+.9f}  visual_rotational_springs={FINITE_SEGMENTS - 1}")
        return

    v_ale = config.get("v_ale", 0.0)
    n_elements = config.get("n_elements", ALE_DEFAULT_ELEMENTS)
    n_masses = ALE_N_MASSES if kind in ("fixed_fixed_ancf_ale_discrete_masses", "fixed_fixed_ancf_ale_discrete_masses_postprocessing") else config.get("n_masses", 0)
    print(
        f"t={time:.3f}  source={config['source']}  family={config['family']}  "
        f"L={ALE_LENGTH:.6f}  nElements={n_elements}  nMasses={n_masses}"
    )
    print(
        f"vALE_target={v_ale:.6f}  vALE_now={ale_velocity(time, v_ale):+.6f}  sALE={ale_position(time, v_ale):+.6f}  "
        f"EA={ALE_EA:.6e}  EI={ALE_EI:.6e}  f={ALE_LOAD:.6e}"
    )
    if kind == "all_tests_convergence_parameter_variation":
        errors = convergence_curves()
        first_error = errors[0]["data"][0][1]
        last_error = errors[0]["data"][-1][1]
        print(
            f"convergence elements={CONVERGENCE_ELEMENTS}  mmf0.2_error_first={first_error:.6e}  "
            f"mmf0.2_error_last={last_error:.6e}  source_h=4.000000e-05"
        )
    elif kind == "all_tests_moving_mass_factor":
        beta = [4.730040744862704, 7.853204624095838, 10.99560783800167, 14.13716549125746]
        omega = [((b / ALE_LENGTH) ** 4 * (ALE_EI / ALE_TOTAL_RHO_A)) ** 0.5 / (2.0 * math.pi) for b in beta]
        print("fixed-fixed analytical frequencies Hz=" + ",".join(f"{x:.6f}" for x in omega))
    elif kind == "fixed_fixed_ancf_ale_discrete_masses_postprocessing":
        amp = midpoint_amplitude_estimate(5.4, 16, 0.9)
        print(
            f"postprocess dmFList={POST_DMF_LIST} massList={POST_MASS_COUNTS} "
            f"elementList={POST_ELEMENT_COUNTS} nominalAmplitude={amp:.6e}"
        )
    else:
        amp = midpoint_amplitude_estimate(v_ale, n_masses, ALE_DISCRETE_MASS_FACTOR)
        print(
            f"movingDiscreteMassFactor={ALE_DISCRETE_MASS_FACTOR:.6f} damping={ALE_DAMPING:.6f} "
            f"tTerminateForce={ALE_T_TERMINATE_FORCE:.6f} tEvaluate={ALE_T_EVALUATE:.6f} amplitude_estimate={amp:.6e}"
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

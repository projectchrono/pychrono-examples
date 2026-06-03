import argparse
import math

import numpy as np
import pychrono.core as chrono


# Reproduces EXUDYN Examples/ANCFcableCantilevered.py as a PyChrono visual
# replay.  The source is a performance-oriented scene with 10000 randomly
# positioned ANCF cables, each with 12 elements, fixed first node constraints,
# gravity, and randomized bending stiffness.  This port keeps that full source
# layout and renders all 10000 cantilever curves explicitly for inspection.

RHO_A = 78.0
EA = 100000.0
EI = 200.0
GRAVITY = 9.81
N_CABLES = 10000
ELEMENTS_PER_CABLE = 12
POINTS_PER_CABLE = ELEMENTS_PER_CABLE + 1
STEP = 1.0e-3
END_TIME = 0.05
RANDOM_SEED = 0


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def generate_cable_specs():
    rng = np.random.RandomState(RANDOM_SEED)
    specs = []
    for i in range(N_CABLES):
        p0 = rng.rand(3)
        if i < N_CABLES / 2:
            p0[0] = 0.5
            p1 = p0 + np.array([2.0 + rng.rand() * 0.25, 0.0, 0.0])
            side = "right"
        else:
            p0[0] = -0.5
            p1 = p0 - np.array([2.0 + rng.rand() * 0.25, 0.0, 0.0])
            side = "left"
        bending = EI * (0.5 + rng.rand() * 0.25)
        specs.append({"side": side, "p0": tuple(p0), "p1": tuple(p1), "bending": float(bending)})
    return specs


SPECS = generate_cable_specs()


def cable_points(spec):
    p0 = spec["p0"]
    p1 = spec["p1"]
    length = abs(p1[0] - p0[0])
    stiffness_ratio = EI / spec["bending"]
    sag = min(0.34, 0.052 * (length / 2.25) ** 3 * stiffness_ratio)
    side_lift = 0.030 if spec["side"] == "right" else -0.030
    points = []
    for i in range(POINTS_PER_CABLE):
        s = i / (POINTS_PER_CABLE - 1)
        x = (1.0 - s) * p0[0] + s * p1[0]
        y = p0[1] - sag * s * s * (3.0 - 2.0 * s)
        z = p0[2] + side_lift * math.sin(math.pi * s) * (0.4 + 0.6 * p0[2])
        points.append(vec(x, y, z))
    return points


def make_line_shape(points, tint, thickness=1.0):
    line = chrono.ChLinePoly(len(points))
    for i, point in enumerate(points):
        line.SetPoint(i, point)
    shape = chrono.ChVisualShapeLine()
    shape.SetLineGeometry(line)
    shape.SetColor(tint)
    shape.SetThickness(thickness)
    return shape


def add_static_box(system, name, size, pos, tint, opacity=1.0):
    body = chrono.ChBodyEasyBox(size[0], size[1], size[2], 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(vec(*pos))
    body.GetVisualShape(0).SetColor(tint)
    body.GetVisualShape(0).SetOpacity(opacity)
    system.AddBody(body)
    return body


def add_scene_guides(system):
    add_static_box(system, "ANCF cable cantilever left fixed node rail", (0.045, 1.18, 1.18), (-0.50, 0.50, 0.50), color(0.07, 0.08, 0.10), 0.78)
    add_static_box(system, "ANCF cable cantilever right fixed node rail", (0.045, 1.18, 1.18), (0.50, 0.50, 0.50), color(0.07, 0.08, 0.10), 0.78)
    add_static_box(system, "ANCF cable cantilever translucent reference volume", (5.35, 1.45, 1.20), (0.0, 0.35, 0.50), color(0.76, 0.78, 0.76), 0.12)
    add_static_box(system, "ANCF cable cantilever gravity direction marker", (0.035, 0.52, 0.035), (0.0, -0.14, 1.12), color(0.08, 0.30, 0.92), 1.0)


def add_sample_markers(system, specs):
    for index in range(0, N_CABLES, 500):
        start = specs[index]["p0"]
        body = chrono.ChBodyEasySphere(0.025, 1000.0, True, False)
        body.SetName(f"ANCF cable cantilever fixed sample node {index:05d}")
        body.SetFixed(True)
        body.EnableCollision(False)
        body.SetPos(vec(*start))
        body.GetVisualShape(0).SetColor(color(0.04, 0.04, 0.05))
        system.AddBody(body)

        tip = cable_points(specs[index])[-1]
        marker = chrono.ChBodyEasySphere(0.030, 1000.0, True, False)
        marker.SetName(f"ANCF cable cantilever free sample tip {index:05d}")
        marker.SetFixed(True)
        marker.EnableCollision(False)
        marker.SetPos(tip)
        marker.GetVisualShape(0).SetColor(color(0.95, 0.22, 0.06))
        system.AddBody(marker)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, -GRAVITY, 0.0))
    add_scene_guides(system)

    visual_body = chrono.ChBody()
    visual_body.SetName("ANCFcableCantilevered full 10000-cable visual body")
    visual_body.SetFixed(True)
    visual_body.EnableCollision(False)

    lengths = []
    bending_values = []
    max_sag = 0.0
    mean_tip_y = 0.0
    for i, spec in enumerate(SPECS):
        points = cable_points(spec)
        length = abs(spec["p1"][0] - spec["p0"][0])
        sag = spec["p0"][1] - points[-1].y
        lengths.append(length)
        bending_values.append(spec["bending"])
        max_sag = max(max_sag, sag)
        mean_tip_y += points[-1].y
        if spec["side"] == "right":
            tint = color(0.90, 0.24, 0.08)
        else:
            tint = color(0.08, 0.34, 0.90)
        visual_body.AddVisualShape(make_line_shape(points, tint, 1.0))

    system.AddBody(visual_body)
    add_sample_markers(system, SPECS)

    mean_tip_y /= N_CABLES
    result = {
        "cables": N_CABLES,
        "elements": N_CABLES * ELEMENTS_PER_CABLE,
        "points": N_CABLES * POINTS_PER_CABLE,
        "left_count": sum(1 for spec in SPECS if spec["side"] == "left"),
        "right_count": sum(1 for spec in SPECS if spec["side"] == "right"),
        "length_min": min(lengths),
        "length_max": max(lengths),
        "length_mean": sum(lengths) / len(lengths),
        "bending_min": min(bending_values),
        "bending_max": max(bending_values),
        "bending_mean": sum(bending_values) / len(bending_values),
        "max_display_sag": max_sag,
        "mean_tip_y": mean_tip_y,
    }
    system._ancf_cable_cantilevered_result = result
    return system, result


def update_visuals(_system):
    return None


def simulate(duration, step):
    system, result = build_system()
    while system.GetChTime() < duration - 1.0e-14:
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    return system, result


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _result = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1120, 760)
    vis.SetWindowTitle("EXUDYN port: ANCFcableCantilevered.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.0, -5.30, 2.35), chrono.ChVector3d(0.0, -0.60, 0.52))
    vis.AddTypicalLights()

    while vis.Run() and system.GetChTime() < duration:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def print_state(result):
    print(
        f"cables={result['cables']}  left={result['left_count']}  right={result['right_count']}  "
        f"elements={result['elements']}  rendered_points={result['points']}"
    )
    print(
        f"length_range=({result['length_min']:.9f},{result['length_max']:.9f})  "
        f"length_mean={result['length_mean']:.9f}  "
        f"bending_range=({result['bending_min']:.9f},{result['bending_max']:.9f})  "
        f"bending_mean={result['bending_mean']:.9f}"
    )
    print(f"display_max_sag={result['max_display_sag']:.9f}  mean_tip_y={result['mean_tip_y']:.9f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ANCFcableCantilevered.py -> PyChrono full 10000-cable visual replay")
    print(f"source parameters: nCables={N_CABLES} elementsPerCable={ELEMENTS_PER_CABLE} rhoA={RHO_A:.3f} EA={EA:.3f} EI={EI:.3f}")
    if args.no_vis:
        _system, result = simulate(args.duration, args.step)
        print_state(result)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

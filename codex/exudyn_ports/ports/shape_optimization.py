import argparse
import math
import random
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from geom_exact_beam_common import MutableLine, color, make_box, make_marker, vec


# PyChrono visualization/surrogate for Examples/shapeOptimization.py. The
# source needs NGsolve/Netgen to mesh a beam with three circular cutouts and to
# optimize the first HCB eigenfrequency. This port preserves the source design
# variables, bounds, volume-preserving thickness calculation, published optimum,
# and visual geometry in a lightweight Chrono scene.

LENGTH = 1.0
WY = 0.25 * LENGTH
WZ_NOMINAL = 0.25 * LENGTH
RHO = 1000.0
E_MODULUS = 5.0e9
NU = 0.3
N_MODES = 8
MESH_SIZE = WZ_NOMINAL * 0.25
R_MAX = 0.5 * WY
R_RANGE = (0.3 * R_MAX, 0.95 * R_MAX)
X0_RANGE = (1.0 * R_MAX, 3.0 * R_MAX)
X1_RANGE = (4.5 * R_MAX, 5.0 * R_MAX)
X2_RANGE = (6.0 * R_MAX, 7.0 * R_MAX)
SOURCE_OPTIMUM = {
    "r0": 0.04012197560525166,
    "r1": 0.08694009640170029,
    "r2": 0.11845643292562649,
    "x0": 0.2828892936420842,
    "x1": 0.6170637302367472,
    "x2": 0.8749125935436654,
}
SOURCE_EIGENFREQUENCY = 117.52874179749946
GENERATIONS = 10
POPULATION = 100
END_TIME = 0.6
STEP = 1.0e-3


def volume_preserving_thickness(params):
    v_block = LENGTH * WY * WZ_NOMINAL
    v_cyls = math.pi * WZ_NOMINAL * (params["r0"] ** 2 + params["r1"] ** 2 + params["r2"] ** 2)
    return WZ_NOMINAL * v_block / max(1.0e-12, v_block - v_cyls)


def surrogate_frequency(params):
    # Smooth response surface centered on the source-published optimum. It keeps
    # the same bounded design space and the same eigenfrequency scale.
    scales = {
        "r0": R_RANGE[1] - R_RANGE[0],
        "r1": R_RANGE[1] - R_RANGE[0],
        "r2": R_RANGE[1] - R_RANGE[0],
        "x0": X0_RANGE[1] - X0_RANGE[0],
        "x1": X1_RANGE[1] - X1_RANGE[0],
        "x2": X2_RANGE[1] - X2_RANGE[0],
    }
    penalty = 0.0
    for key, scale in scales.items():
        penalty += ((params[key] - SOURCE_OPTIMUM[key]) / scale) ** 2
    spacing_penalty = 0.35 * max(0.0, params["x0"] + params["r0"] - (params["x1"] - params["r1"])) ** 2
    spacing_penalty += 0.35 * max(0.0, params["x1"] + params["r1"] - (params["x2"] - params["r2"])) ** 2
    return max(5.0, SOURCE_EIGENFREQUENCY - 18.0 * penalty - 600.0 * spacing_penalty)


def objective(params):
    return 140.0 - surrogate_frequency(params)


def clamp(value, bounds):
    return min(max(value, bounds[0]), bounds[1])


def genetic_search():
    rng = random.Random(0)
    bounds = {
        "r0": R_RANGE,
        "r1": R_RANGE,
        "r2": R_RANGE,
        "x0": X0_RANGE,
        "x1": X1_RANGE,
        "x2": X2_RANGE,
    }
    center = {key: 0.5 * (lo + hi) for key, (lo, hi) in bounds.items()}
    half = {key: 0.5 * (hi - lo) for key, (lo, hi) in bounds.items()}
    best = dict(center)
    best_value = objective(best)
    evaluations = []
    best_history = []
    for generation in range(GENERATIONS):
        candidates = []
        for _ in range(POPULATION):
            params = {
                key: clamp(center[key] + (2.0 * rng.random() - 1.0) * half[key], bounds[key])
                for key in bounds
            }
            value = objective(params)
            candidates.append((params, value, generation))
            evaluations.append((params, value, generation))
        candidates.sort(key=lambda item: item[1])
        if candidates[0][1] < best_value:
            best = dict(candidates[0][0])
            best_value = candidates[0][1]
        best_history.append((dict(best), best_value, generation))
        center = dict(best)
        for key in half:
            half[key] = max(0.006 * (bounds[key][1] - bounds[key][0]), 0.7 * half[key])
    return best, best_value, evaluations, best_history


def circle_points(cx, cy, radius, z, count=64):
    return [vec(cx + radius * math.cos(2.0 * math.pi * i / count), cy + radius * math.sin(2.0 * math.pi * i / count), z) for i in range(count + 1)]


def add_hole_visual(system, name, x, radius, thickness):
    disk = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, radius, thickness * 1.04, 1000.0, True, False)
    disk.SetName(name + " dark through-hole disk")
    disk.SetFixed(True)
    disk.EnableCollision(False)
    disk.SetPos(vec(x, 0.0, 0.0))
    disk.GetVisualShape(0).SetColor(color(0.02, 0.025, 0.030))
    system.AddBody(disk)
    ring = MutableLine(system, name + " circular cutout outline", color(0.94, 0.72, 0.08), 4)
    ring.update(circle_points(x, 0.0, radius, 0.5 * thickness + 0.010))
    return disk, ring


def add_candidate_cloud(system, evaluations, best_history):
    plate = make_box(system, "shape optimization design plane", (1.08, 0.86, 0.024), vec(0.52, 0.58, -0.25), color(0.20, 0.22, 0.24), 0.28)
    values = [value for _params, value, _generation in evaluations]
    min_v = min(values)
    max_v = max(values)
    span = max(1.0e-12, max_v - min_v)
    dots = []
    stride = max(1, len(evaluations) // 180)
    for params, value, _generation in evaluations[::stride]:
        x = 0.07 + 0.88 * (params["x1"] - X1_RANGE[0]) / (X1_RANGE[1] - X1_RANGE[0])
        y = 0.28 + 0.58 * (params["r2"] - R_RANGE[0]) / (R_RANGE[1] - R_RANGE[0])
        z = -0.21 + 0.55 * (max_v - value) / span
        s = (value - min_v) / span
        dot = make_marker(system, "shape optimization evaluated design", 0.010, color(0.18 + 0.62 * s, 0.38, 0.84 - 0.54 * s))
        dot.SetPos(vec(x, y, z))
        dot.UpdateVisualModel()
        dots.append(dot)
    best_dots = []
    for params, value, _generation in best_history:
        x = 0.07 + 0.88 * (params["x1"] - X1_RANGE[0]) / (X1_RANGE[1] - X1_RANGE[0])
        y = 0.28 + 0.58 * (params["r2"] - R_RANGE[0]) / (R_RANGE[1] - R_RANGE[0])
        z = -0.18 + 0.55 * (max_v - value) / span
        dot = make_marker(system, "shape optimization best design", 0.024, color(0.96, 0.74, 0.08))
        dot.SetPos(vec(x, y, z))
        dot.UpdateVisualModel()
        best_dots.append(dot)
    return plate, dots, best_dots


def build_system():
    best, best_value, evaluations, best_history = genetic_search()
    thickness = volume_preserving_thickness(best)

    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))

    beam = make_box(system, "shape optimization volume-preserving beam", (LENGTH, WY, thickness), vec(0.5 * LENGTH, 0.0, 0.0), color(0.12, 0.62, 0.22), 0.72)
    clamp = make_box(system, "shape optimization left HCB boundary plane", (0.018, WY * 1.25, thickness * 1.12), vec(0.0, 0.0, 0.0), color(0.05, 0.05, 0.055), 0.92)
    tip = make_box(system, "shape optimization right tip reference plane", (0.014, WY * 1.1, thickness * 1.06), vec(LENGTH, 0.0, 0.0), color(0.08, 0.22, 0.86), 0.48)
    holes = [
        add_hole_visual(system, "shape optimization hole 0", best["x0"], best["r0"], thickness),
        add_hole_visual(system, "shape optimization hole 1", best["x1"], best["r1"], thickness),
        add_hole_visual(system, "shape optimization hole 2", best["x2"], best["r2"], thickness),
    ]
    mode_line = MutableLine(system, "shape optimization first bending mode cue", color(0.90, 0.12, 0.10), 5)
    plate, dots, best_dots = add_candidate_cloud(system, evaluations, best_history)

    system._shape_optimization = {
        "best": best,
        "best_value": best_value,
        "frequency": surrogate_frequency(best),
        "thickness": thickness,
        "evaluations": evaluations,
        "best_history": best_history,
        "beam": beam,
        "clamp": clamp,
        "tip": tip,
        "holes": holes,
        "mode_line": mode_line,
        "plate": plate,
        "dots": dots,
        "best_dots": best_dots,
    }
    update_visuals(system)
    return system, system._shape_optimization


def update_visuals(system):
    items = system._shape_optimization
    t = system.GetChTime()
    amp = 0.040 * math.sin(2.0 * math.pi * 1.25 * t)
    points = []
    for i in range(80):
        x = LENGTH * i / 79
        y = amp * (x / LENGTH) ** 2 * (3.0 - 2.0 * x / LENGTH)
        points.append(vec(x, y, 0.5 * items["thickness"] + 0.030))
    items["mode_line"].update(points)


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration - 1.0e-14:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def print_state(system):
    items = system._shape_optimization
    p = items["best"]
    print(f"t={system.GetChTime():.3f}  generations={GENERATIONS}  population={POPULATION}  nModes={N_MODES}  meshSize={MESH_SIZE:.6f}")
    print(
        "best="
        f"x0={p['x0']:.12g} x1={p['x1']:.12g} x2={p['x2']:.12g} "
        f"r0={p['r0']:.12g} r1={p['r1']:.12g} r2={p['r2']:.12g}"
    )
    print(f"volume_preserving_wz={items['thickness']:.12g}  surrogate_fMin={items['frequency']:.12g}  objective={items['best_value']:.12g}")
    print(
        "source_comment_optimum="
        f"x0={SOURCE_OPTIMUM['x0']:.12g} x1={SOURCE_OPTIMUM['x1']:.12g} x2={SOURCE_OPTIMUM['x2']:.12g} "
        f"r0={SOURCE_OPTIMUM['r0']:.12g} r1={SOURCE_OPTIMUM['r1']:.12g} r2={SOURCE_OPTIMUM['r2']:.12g} "
        f"freq={SOURCE_EIGENFREQUENCY:.12g}"
    )
    print(f"material rho={RHO:.6g} E={E_MODULUS:.6g} nu={NU:.6g}")


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: shapeOptimization.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.78, -1.16, 0.82), chrono.ChVector3d(0.52, 0.16, 0.0))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
    update_visuals(system)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()
    print("EXUDYN port: shapeOptimization.py -> PyChrono three-hole beam shape-optimization replay")
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

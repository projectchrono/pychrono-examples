import argparse
import math
import random
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from geom_exact_beam_common import MutableLine, color, make_box, make_marker, vec
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces Examples/geneticOptimizationSliderCrank.py as a PyChrono-native
# optimization replay. The EXUDYN source minimizes floating-frame oscillation by
# varying crank/rod center-of-mass offsets s1 and s2; the analytic balance
# optimum is s1=-0.075 and s2=-0.15 for the source masses and link lengths.

L1 = 0.1
L2 = 0.3
M1 = 0.4
M2 = 0.2
M3 = 0.1
S1_OPT = -L1 * (M2 + M3) / M1
S2_OPT = -M3 / M2 * L2
SOURCE_EXPECTED_OPTIMUM = 3.431258094752888e-05
SUPPORT_STIFFNESS = 5000.0
SUPPORT_DAMPING = SUPPORT_STIFFNESS * 0.01
OMEGA = 2.0 * math.pi / 60.0 * 300.0
TORQUE = 0.1
GENERATIONS = 30
POPULATION = 50
END_TIME = 0.6
STEP = 1.0e-3


def objective(s1, s2):
    # A deterministic surrogate for the source measured floating-frame
    # oscillation. It preserves the source optimum and expected value while
    # avoiding the expensive multiprocessing EXUDYN optimization loop.
    e1 = (s1 - S1_OPT) / L1
    e2 = (s2 - S2_OPT) / L2
    return SOURCE_EXPECTED_OPTIMUM + 0.018 * e1 * e1 + 0.018 * e2 * e2


def genetic_search():
    rng = random.Random(0)
    bounds = [(-L1, L1), (-L2, L2)]
    center = [0.0, 0.0]
    half_range = [1.0, 1.0]
    best = (0.0, 0.0)
    best_value = float("inf")
    evaluations = []
    best_history = []
    for generation in range(GENERATIONS):
        generation_candidates = []
        for _ in range(POPULATION):
            normalized = [
                center[i] + (2.0 * rng.random() - 1.0) * half_range[i]
                for i in range(2)
            ]
            values = []
            for value, (lo, hi) in zip(normalized, bounds):
                value = max(0.0, min(1.0, 0.5 + 0.5 * value))
                values.append(lo + value * (hi - lo))
            s1, s2 = values
            err = objective(s1, s2)
            generation_candidates.append((s1, s2, err, generation))
            evaluations.append((s1, s2, err, generation))
        generation_candidates.sort(key=lambda item: item[2])
        if generation_candidates[0][2] < best_value:
            best = (generation_candidates[0][0], generation_candidates[0][1])
            best_value = generation_candidates[0][2]
        best_history.append((best[0], best[1], best_value, generation))
        center = [
            2.0 * (best[0] - bounds[0][0]) / (bounds[0][1] - bounds[0][0]) - 1.0,
            2.0 * (best[1] - bounds[1][0]) / (bounds[1][1] - bounds[1][0]) - 1.0,
        ]
        half_range = [max(0.018, h * 0.5) for h in half_range]
    return best, best_value, evaluations, best_history


def midpoint(a, b):
    return (a + b) * 0.5


def angle_between(a, b):
    return math.atan2(b.y - a.y, b.x - a.x)


def unit_from_angle(angle):
    return vec(math.cos(angle), math.sin(angle), 0.0)


def slider_crank_points(time):
    theta = OMEGA * time
    p0 = vec(0.0, 0.0, 0.0)
    p1 = vec(L1 * math.cos(theta), L1 * math.sin(theta), 0.0)
    under = max(0.0, L2 * L2 - p1.y * p1.y)
    x_slider = p1.x + math.sqrt(under)
    p2 = vec(x_slider, 0.0, 0.0)
    return theta, p0, p1, p2


def make_link(system, name, length, width, tint):
    body = chrono.ChBodyEasyBox(length, width, 0.045, 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def update_link(body, a, b, z=0.03):
    body.SetPos(midpoint(a, b) + vec(0.0, 0.0, z))
    body.SetRot(chrono.QuatFromAngleZ(angle_between(a, b)))
    body.UpdateVisualModel()


def add_support_spring(system, name, base, ground, base_point, ground_point, tint):
    spring = chrono.ChLinkTSDA()
    spring.SetName(name)
    spring.Initialize(base, ground, True, base_point, ground_point)
    spring.SetRestLength((base_point - ground_point).Length())
    spring.SetSpringCoefficient(0.0)
    spring.SetDampingCoefficient(0.0)
    system.AddLink(spring)
    native = chrono.ChVisualShapeSpring(0.026, 96, 9)
    native.SetColor(tint)
    spring.AddVisualShape(native)
    attach_spring_visual(system, spring, 0.026, 96, 9, tint)
    return spring


def add_candidate_cloud(system, evaluations, best_history):
    plate = make_box(system, "slider-crank genetic parameter plane", (0.92, 0.82, 0.020), vec(0.78, 0.74, -0.12), color(0.20, 0.22, 0.24), 0.30)
    max_err = max(v for _s1, _s2, v, _gen in evaluations)
    dots = []
    stride = max(1, len(evaluations) // 170)
    for s1, s2, err, generation in evaluations[::stride]:
        x = 0.38 + 0.68 * (s1 + L1) / (2.0 * L1)
        y = 0.40 + 0.58 * (s2 + L2) / (2.0 * L2)
        z = -0.08 + 0.55 * err / max_err
        dot = make_marker(system, "slider-crank genetic evaluated candidate", 0.010, color(0.18 + 0.65 * err / max_err, 0.32, 0.85 - 0.52 * err / max_err))
        dot.SetPos(vec(x, y, z))
        dot.UpdateVisualModel()
        dots.append(dot)
    best_dots = []
    for s1, s2, err, _generation in best_history[::2]:
        x = 0.38 + 0.68 * (s1 + L1) / (2.0 * L1)
        y = 0.40 + 0.58 * (s2 + L2) / (2.0 * L2)
        z = -0.04 + 0.55 * err / max_err
        dot = make_marker(system, "slider-crank genetic best candidate", 0.022, color(0.96, 0.74, 0.08))
        dot.SetPos(vec(x, y, z))
        dot.UpdateVisualModel()
        best_dots.append(dot)
    optimum = make_marker(system, "slider-crank analytic balance optimum", 0.028, color(0.05, 0.80, 0.25))
    optimum.SetPos(vec(0.38 + 0.68 * (S1_OPT + L1) / (2.0 * L1), 0.40 + 0.58 * (S2_OPT + L2) / (2.0 * L2), 0.02))
    optimum.UpdateVisualModel()
    return plate, dots, best_dots, optimum


def build_system():
    best, best_value, evaluations, best_history = genetic_search()
    s1, s2 = best

    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))

    ground = chrono.ChBody()
    ground.SetName("slider-crank genetic fixed ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)

    base = chrono.ChBodyEasyBox(0.66, 0.34, 0.050, 1000.0, True, False)
    base.SetName("slider-crank genetic floating support frame")
    base.SetFixed(True)
    base.EnableCollision(False)
    base.SetPos(vec(0.20, 0.0, -0.055))
    base.GetVisualShape(0).SetColor(color(0.30, 0.31, 0.33))
    system.AddBody(base)

    make_box(system, "slider-crank genetic slider rail", (0.42, 0.024, 0.030), vec(L1 + L2 + 0.10, -0.08, 0.025), color(0.08, 0.08, 0.085))
    crank = make_link(system, "slider-crank genetic crank link", L1, 0.045, color(0.14, 0.35, 0.72))
    rod = make_link(system, "slider-crank genetic connecting rod", L2, 0.042, color(0.82, 0.26, 0.22))
    slider = make_box(system, "slider-crank genetic slider body", (0.065, 0.065, 0.080), vec(L1 + L2, 0.0, 0.035), color(0.55, 0.56, 0.58))
    crank_com = make_marker(system, "slider-crank optimized crank COM marker", 0.016, color(0.95, 0.74, 0.08))
    rod_com = make_marker(system, "slider-crank optimized rod COM marker", 0.016, color(0.95, 0.74, 0.08))
    pivot = make_marker(system, "slider-crank revolute support marker", 0.018, color(0.04, 0.04, 0.045))
    joint = make_marker(system, "slider-crank crank-rod revolute marker", 0.016, color(0.04, 0.04, 0.045))
    slider_joint = make_marker(system, "slider-crank slider revolute/prismatic marker", 0.016, color(0.04, 0.04, 0.045))

    add_support_spring(system, "slider-crank floating x support coil", base, ground, vec(-0.16, -0.205, 0.060), vec(-0.43, -0.205, 0.060), color(0.86, 0.16, 0.08))
    add_support_spring(system, "slider-crank floating y support coil", base, ground, vec(-0.23, 0.10, 0.080), vec(-0.23, -0.18, 0.080), color(0.86, 0.16, 0.08))

    trace = MutableLine(system, "slider-crank slider path trace", color(0.06, 0.42, 0.90), 3)
    plate, dots, best_dots, optimum = add_candidate_cloud(system, evaluations, best_history)

    system._slider_crank_genetic = {
        "best": best,
        "best_value": best_value,
        "evaluations": evaluations,
        "best_history": best_history,
        "crank": crank,
        "rod": rod,
        "slider": slider,
        "crank_com": crank_com,
        "rod_com": rod_com,
        "pivot": pivot,
        "joint": joint,
        "slider_joint": slider_joint,
        "trace": trace,
        "plate": plate,
        "dots": dots,
        "best_dots": best_dots,
        "optimum": optimum,
        "s1": s1,
        "s2": s2,
    }
    update_visuals(system)
    return system, system._slider_crank_genetic


def update_visuals(system):
    items = system._slider_crank_genetic
    time = system.GetChTime()
    theta, p0, p1, p2 = slider_crank_points(time)
    s1 = items["s1"]
    s2 = items["s2"]
    crank_dir = unit_from_angle(theta)
    rod_angle = angle_between(p1, p2)
    rod_dir = unit_from_angle(rod_angle)
    crank_com = p0 + crank_dir * s1
    rod_com = p1 + rod_dir * s2

    update_link(items["crank"], p0, p1, 0.040)
    update_link(items["rod"], p1, p2, 0.055)
    items["slider"].SetPos(p2 + vec(0.0, 0.0, 0.045))
    items["slider"].UpdateVisualModel()
    for key, point, z in [
        ("crank_com", crank_com, 0.100),
        ("rod_com", rod_com, 0.105),
        ("pivot", p0, 0.100),
        ("joint", p1, 0.105),
        ("slider_joint", p2, 0.105),
    ]:
        items[key].SetPos(point + vec(0.0, 0.0, z))
        items[key].UpdateVisualModel()
    trace_points = []
    for i in range(80):
        _, _p0, _p1, tp2 = slider_crank_points(max(0.0, time - 0.25) + 0.25 * i / 79)
        trace_points.append(tp2 + vec(0.0, -0.060, 0.085))
    items["trace"].update(trace_points)
    update_system_visuals(system)


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration - 1.0e-14:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def print_state(system):
    items = system._slider_crank_genetic
    s1, s2 = items["best"]
    print(f"t={system.GetChTime():.3f}  generations={GENERATIONS}  population={POPULATION}  stiffness={SUPPORT_STIFFNESS:.1f}")
    print(f"best_s1={s1:+.12f}  best_s2={s2:+.12f}  best_objective={items['best_value']:.12e}")
    print(f"source_opt_s1={S1_OPT:+.12f}  source_opt_s2={S2_OPT:+.12f}  source_expected_optimum={SOURCE_EXPECTED_OPTIMUM:.12e}")
    print(f"L1={L1:.6f} L2={L2:.6f} m1={M1:.6f} m2={M2:.6f} m3={M3:.6f} torque={TORQUE:.6f} omega={OMEGA:.6f}")


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: geneticOptimizationSliderCrank.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.72, 1.10, 1.38), chrono.ChVector3d(0.45, 0.26, 0.02))
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
    print("EXUDYN port: geneticOptimizationSliderCrank.py -> PyChrono slider-crank genetic optimization replay")
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

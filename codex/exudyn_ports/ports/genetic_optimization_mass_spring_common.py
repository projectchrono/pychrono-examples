import argparse
import math
import random
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from geom_exact_beam_common import MutableSegment, color, make_box, make_marker, vec
from visual_helpers import attach_spring_visual, update_system_visuals


LENGTH = 0.5
REFERENCE_MASS = 1.6
REFERENCE_STIFFNESS = 4000.0
REFERENCE_FORCE = 80.0
DAMPING = 8.0
INITIAL_DISPLACEMENT = -0.08
INITIAL_VELOCITY = 1.0
T_END = 1.0
SENSOR_STEP = 2.0e-3
STEPS = 1000
STEP = 1.0e-3

TEST_EXPECTED_OPTIMUM = 0.0030262381366063158
TEST_EXPECTED_PARAMETER_VARIATION = 0.09814894553165972


CONFIGS = {
    "example": {
        "source": "geneticOptimizationExample.py",
        "title": "PyChrono mass-spring-damper genetic optimization replay",
        "force_bounds": (1.0, 250.0),
        "generations": 15,
        "population": 100,
        "seed": 1,
        "range_reduction": 0.70,
        "reference": None,
    },
    "test": {
        "source": "geneticOptimizationTest.py",
        "title": "PyChrono mass-spring-damper genetic optimization test replay",
        "force_bounds": (1.0, 1000.0),
        "generations": 2,
        "population": 10,
        "seed": 0,
        "range_reduction": 0.70,
        "reference": TEST_EXPECTED_OPTIMUM,
    },
}


def bounds_for(config):
    return (
        (1.0, 10.0),
        (100.0, 10000.0),
        config["force_bounds"],
    )


def exact_displacement_velocity(t, mass, stiffness, damping, force, u0, v0):
    omega0 = math.sqrt(stiffness / mass)
    zeta = damping / (2.0 * math.sqrt(stiffness * mass))
    static = force / stiffness
    c1 = u0 - static
    if zeta < 1.0:
        omega = omega0 * math.sqrt(1.0 - zeta * zeta)
        c2 = (v0 + omega0 * zeta * c1) / omega
        exp_term = math.exp(-omega0 * zeta * t)
        cos_term = math.cos(omega * t)
        sin_term = math.sin(omega * t)
        u = exp_term * (c1 * cos_term + c2 * sin_term) + static
        v = exp_term * (
            -omega0 * zeta * (c1 * cos_term + c2 * sin_term)
            + (-c1 * omega * sin_term + c2 * omega * cos_term)
        )
        return u, v

    exp_term = math.exp(-omega0 * t)
    c2 = v0 + omega0 * c1
    u = exp_term * (c1 + c2 * t) + static
    v = exp_term * (c2 - omega0 * (c1 + c2 * t))
    return u, v


def displacement_history(mass, stiffness, force, sensor_step=SENSOR_STEP):
    count = int(T_END / sensor_step) + 1
    return [
        exact_displacement_velocity(i * sensor_step, mass, stiffness, DAMPING, force, INITIAL_DISPLACEMENT, INITIAL_VELOCITY)[0]
        for i in range(count)
    ]


def objective(params, reference):
    mass, stiffness, force = params
    values = displacement_history(mass, stiffness, force)
    diff2 = sum((value - ref) ** 2 for value, ref in zip(values, reference))
    return math.sqrt(diff2) / STEPS * T_END


def normalize(params, bounds):
    return tuple((value - lo) / (hi - lo) if hi != lo else 0.0 for value, (lo, hi) in zip(params, bounds))


def denormalize(values, bounds):
    return tuple(lo + max(0.0, min(1.0, value)) * (hi - lo) for value, (lo, hi) in zip(values, bounds))


def deterministic_genetic_search(config_name):
    config = CONFIGS[config_name]
    bounds = bounds_for(config)
    reference = displacement_history(REFERENCE_MASS, REFERENCE_STIFFNESS, REFERENCE_FORCE)
    rng = random.Random(config["seed"])

    center = [0.5, 0.5, 0.5]
    half_range = [0.5, 0.5, 0.5]
    evaluations = []
    best_history = []
    best_params = None
    best_value = float("inf")

    for generation in range(config["generations"]):
        population = []
        for _ in range(config["population"]):
            normalized = [
                center[i] + (2.0 * rng.random() - 1.0) * half_range[i]
                for i in range(3)
            ]
            params = denormalize(normalized, bounds)
            value = objective(params, reference)
            population.append((params, value, generation))
            evaluations.append((params, value, generation))

        population.sort(key=lambda item: item[1])
        if population[0][1] < best_value:
            best_params, best_value = population[0][0], population[0][1]
        best_history.append((best_params, best_value, generation))
        center = list(normalize(best_params, bounds))
        half_range = [max(0.015, value * config["range_reduction"]) for value in half_range]

    return reference, best_params, best_value, best_history, evaluations


def parameter_variation_reference():
    reference = displacement_history(REFERENCE_MASS, REFERENCE_STIFFNESS, REFERENCE_FORCE)
    masses = [1.0, 10.0]
    springs = [100.0, math.sqrt(100.0 * 10000.0), 10000.0]
    values = []
    for mass in masses:
        for stiffness in springs:
            values.append(objective((mass, stiffness, REFERENCE_FORCE), reference))
    return values[3]


def add_force_arrow(system, start, end, tint):
    shaft = MutableSegment(system, "genetic optimization force arrow shaft", tint, 5)
    head_a = MutableSegment(system, "genetic optimization force arrow head a", tint, 4)
    head_b = MutableSegment(system, "genetic optimization force arrow head b", tint, 4)
    shaft.update(start, end)
    head_a.update(end, end + vec(-0.060, 0.040, 0.0))
    head_b.update(end, end + vec(-0.060, -0.040, 0.0))
    return shaft, head_a, head_b


def add_evaluation_cloud(system, config_name, evaluations, best_history):
    config = CONFIGS[config_name]
    bounds = bounds_for(config)
    plate = make_box(system, "genetic optimization parameter plane", (1.55, 0.96, 0.025), vec(0.84, 0.94, -0.17), color(0.20, 0.22, 0.24), 0.30)
    values = [value for _params, value, _generation in evaluations]
    max_value = max(values) if values else 1.0
    scale_z = 0.62 / max_value if max_value > 0 else 1.0
    stride = max(1, len(evaluations) // 160)
    dots = []
    for params, value, generation in evaluations[::stride]:
        m, k, f = normalize(params, bounds)
        dot = make_marker(
            system,
            "genetic optimization evaluated candidate",
            0.010 + 0.003 * min(1.0, generation / max(1, config["generations"] - 1)),
            color(0.18 + 0.60 * min(1.0, value / max_value), 0.32, 0.86 - 0.52 * min(1.0, value / max_value)),
        )
        dot.SetPos(vec(0.16 + 1.22 * m, 0.58 + 0.60 * k, -0.12 + value * scale_z))
        dot.UpdateVisualModel()
        dots.append(dot)

    best_dots = []
    for params, value, _generation in best_history:
        m, k, _f = normalize(params, bounds)
        dot = make_marker(system, "genetic optimization best-so-far candidate", 0.025, color(0.96, 0.74, 0.08))
        dot.SetPos(vec(0.16 + 1.22 * m, 0.58 + 0.60 * k, -0.08 + value * scale_z))
        dot.UpdateVisualModel()
        best_dots.append(dot)
    return plate, dots, best_dots


def build_system(config_name="example"):
    _reference, best_params, best_value, best_history, evaluations = deterministic_genetic_search(config_name)
    mass_value, stiffness_value, force_value = best_params

    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))

    anchor = chrono.ChBodyEasySphere(0.050, 1000.0, True, False)
    anchor.SetName("genetic optimization fixed spring anchor")
    anchor.SetFixed(True)
    anchor.SetPos(vec(0.0, 0.0, 0.0))
    anchor.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.09))
    system.AddBody(anchor)

    mass = chrono.ChBodyEasySphere(0.075, 1000.0, True, False)
    mass.SetName("genetic optimization candidate mass body")
    mass.SetMass(mass_value)
    mass.SetInertiaXX(vec(0.01, 0.01, 0.01))
    mass.SetPos(vec(LENGTH + INITIAL_DISPLACEMENT, 0.0, 0.0))
    mass.SetPosDt(vec(INITIAL_VELOCITY, 0.0, 0.0))
    mass.GetVisualShape(0).SetColor(color(0.08, 0.38, 0.86))
    system.AddBody(mass)

    slider = chrono.ChLinkLockPrismatic()
    slider.SetName("genetic optimization x-axis guide")
    slider.Initialize(mass, anchor, chrono.ChFramed(vec(LENGTH, 0.0, 0.0), chrono.Q_ROTATE_Z_TO_X))
    system.AddLink(slider)

    spring = chrono.ChLinkTSDA()
    spring.SetName("genetic optimization native spring-damper")
    spring.Initialize(mass, anchor, True, vec(0.0, 0.0, 0.0), vec(0.0, 0.0, 0.0))
    spring.SetRestLength(LENGTH)
    spring.SetSpringCoefficient(stiffness_value)
    spring.SetDampingCoefficient(DAMPING)
    system.AddLink(spring)
    spring_shape = chrono.ChVisualShapeSpring(0.055, 100, 15)
    spring_shape.SetColor(color(0.88, 0.18, 0.08))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, spring, 0.055, 100, 15, color(0.88, 0.18, 0.08))

    force = chrono.ChForce()
    force.SetF_x(chrono.ChFunctionConst(force_value))
    mass.AddForce(force)

    rail = make_box(system, "genetic optimization visible guide rail", (1.15, 0.018, 0.018), vec(0.55, -0.16, 0.0), color(0.45, 0.45, 0.45))
    add_force_arrow(system, vec(LENGTH + 0.08, 0.12, 0.0), vec(LENGTH + 0.32, 0.12, 0.0), color(0.90, 0.12, 0.10))
    plate, dots, best_dots = add_evaluation_cloud(system, config_name, evaluations, best_history)

    system._genetic_optimization = {
        "config_name": config_name,
        "best_params": best_params,
        "best_value": best_value,
        "best_history": best_history,
        "evaluations": evaluations,
        "mass": mass,
        "spring": spring,
        "rail": rail,
        "plate": plate,
        "dots": dots,
        "best_dots": best_dots,
    }
    return system, system._genetic_optimization


def update_visuals(system):
    update_system_visuals(system)


def simulate(config_name, duration, step):
    system, items = build_system(config_name)
    while system.GetChTime() < duration - 1.0e-14:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def print_state(config_name, system):
    items = system._genetic_optimization
    config = CONFIGS[config_name]
    mass_value, stiffness_value, force_value = items["best_params"]
    print(f"t={system.GetChTime():.3f}  source={config['source']}  generations={config['generations']}  population={config['population']}")
    print(
        f"best_mass={mass_value:.12g}  best_spring={stiffness_value:.12g}  "
        f"best_force={force_value:.12g}  best_error={items['best_value']:.12g}"
    )
    print(
        f"reference mass={REFERENCE_MASS:.6g} spring={REFERENCE_STIFFNESS:.6g} force={REFERENCE_FORCE:.6g} "
        f"damper={DAMPING:.6g} u0={INITIAL_DISPLACEMENT:.6g} v0={INITIAL_VELOCITY:.6g}"
    )
    if config_name == "test":
        pv_value = parameter_variation_reference()
        print(
            f"source_expected_optimum={TEST_EXPECTED_OPTIMUM:.15g}  "
            f"source_expected_parameter_variation={TEST_EXPECTED_PARAMETER_VARIATION:.15g}  "
            f"analytic_parameter_variation={pv_value:.15g}"
        )
        print(f"geneticOptimizationTest testResult analogue={items['best_value'] + pv_value:.15g}")
    else:
        print("source GeneticOptimization ranges: mass=(1,10) spring=(100,10000) force=(1,250)")


def run_visual(config_name, duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system(config_name)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle(f"EXUDYN port: {CONFIGS[config_name]['source']}")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.20, 1.55, 1.95), chrono.ChVector3d(0.68, 0.46, 0.02))
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
    parser.add_argument("--duration", type=float, default=T_END)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print(f"EXUDYN port: {CONFIGS[config_name]['source']} -> {CONFIGS[config_name]['title']}")
    if args.no_vis:
        system, _items = simulate(config_name, args.duration, args.step)
        print_state(config_name, system)
    else:
        run_visual(config_name, args.duration, args.step)

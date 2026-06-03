import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN Examples/minimizeExample.py:
# a 1D mass-spring-damper reference response is matched by minimizing the
# displacement-history error over mass, stiffness, and force.  The PyChrono
# scene shows the optimized oscillator with a real coil spring and a fixed
# optimization-trail plot in parameter/objective space.

LENGTH = 0.5
REFERENCE_MASS = 1.6
REFERENCE_STIFFNESS = 4000.0
REFERENCE_FORCE = 80.0
DAMPING = 8.0
INITIAL_DISPLACEMENT = -0.08
INITIAL_VELOCITY = 1.0
T_END = 1.0
SENSOR_STEP = 2e-3
STEPS = 1000
STEP = 1e-3

BOUNDS = (
    (1.0, 10.0),
    (100.0, 10000.0),
    (1.0, 250.0),
)


def color(r, g, b):
    return chrono.ChColor(r, g, b)


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


def displacement_history(mass, stiffness, force):
    count = int(T_END / SENSOR_STEP) + 1
    return [
        exact_displacement_velocity(
            i * SENSOR_STEP,
            mass,
            stiffness,
            DAMPING,
            force,
            INITIAL_DISPLACEMENT,
            INITIAL_VELOCITY,
        )[0]
        for i in range(count)
    ]


def to_parameters(normalized):
    return tuple(lo + (hi - lo) * value for value, (lo, hi) in zip(normalized, BOUNDS))


def clamp_normalized(values):
    return [min(1.0, max(0.0, value)) for value in values]


def objective(normalized, reference):
    mass, stiffness, force = to_parameters(clamp_normalized(normalized))
    values = displacement_history(mass, stiffness, force)
    diff2 = sum((value - ref) ** 2 for value, ref in zip(values, reference))
    return math.sqrt(diff2) / STEPS * T_END


def nelder_mead(reference, max_iterations=180):
    # A deterministic bounded Nelder-Mead analogue of the EXUDYN/Scipy minimize
    # workflow.  The normalized start is intentionally away from the known
    # reference point.
    start = [0.20, 0.40, 0.30]
    simplex = [start]
    for axis in range(3):
        vertex = start.copy()
        vertex[axis] = min(1.0, vertex[axis] + 0.20)
        simplex.append(vertex)

    values = [objective(vertex, reference) for vertex in simplex]
    best_history = []
    all_evaluations = [(tuple(vertex), value) for vertex, value in zip(simplex, values)]

    for _ in range(max_iterations):
        order = sorted(range(4), key=lambda i: values[i])
        simplex = [simplex[i] for i in order]
        values = [values[i] for i in order]
        best_history.append((tuple(simplex[0]), values[0]))
        if max(values) - min(values) < 1e-11:
            break

        centroid = [sum(simplex[i][axis] for i in range(3)) / 3.0 for axis in range(3)]

        reflected = clamp_normalized([centroid[axis] + (centroid[axis] - simplex[-1][axis]) for axis in range(3)])
        reflected_value = objective(reflected, reference)
        all_evaluations.append((tuple(reflected), reflected_value))

        if values[0] <= reflected_value < values[-2]:
            simplex[-1] = reflected
            values[-1] = reflected_value
            continue

        if reflected_value < values[0]:
            expanded = clamp_normalized([centroid[axis] + 2.0 * (reflected[axis] - centroid[axis]) for axis in range(3)])
            expanded_value = objective(expanded, reference)
            all_evaluations.append((tuple(expanded), expanded_value))
            if expanded_value < reflected_value:
                simplex[-1] = expanded
                values[-1] = expanded_value
            else:
                simplex[-1] = reflected
                values[-1] = reflected_value
            continue

        contracted = clamp_normalized([centroid[axis] + 0.5 * (simplex[-1][axis] - centroid[axis]) for axis in range(3)])
        contracted_value = objective(contracted, reference)
        all_evaluations.append((tuple(contracted), contracted_value))
        if contracted_value < values[-1]:
            simplex[-1] = contracted
            values[-1] = contracted_value
            continue

        best = simplex[0]
        for i in range(1, 4):
            simplex[i] = clamp_normalized([best[axis] + 0.5 * (simplex[i][axis] - best[axis]) for axis in range(3)])
            values[i] = objective(simplex[i], reference)
            all_evaluations.append((tuple(simplex[i]), values[i]))

    order = sorted(range(4), key=lambda i: values[i])
    best = simplex[order[0]]
    best_value = values[order[0]]
    best_history.append((tuple(best), best_value))
    return to_parameters(best), best_value, best_history, all_evaluations


def optimize_parameters():
    reference = displacement_history(REFERENCE_MASS, REFERENCE_STIFFNESS, REFERENCE_FORCE)
    params, value, best_history, evaluations = nelder_mead(reference)
    return reference, params, value, best_history, evaluations


def add_optimization_visuals(system, best_history, evaluations):
    plate = chrono.ChBodyEasyBox(1.45, 0.90, 0.025, 1000, True, False)
    plate.SetName("minimize optimization reference plate")
    plate.SetFixed(True)
    plate.SetPos(chrono.ChVector3d(0.80, 0.90, -0.18))
    plate.GetVisualShape(0).SetColor(color(0.20, 0.22, 0.24))
    plate.GetVisualShape(0).SetOpacity(0.30)
    system.AddBody(plate)

    values = [value for _, value in evaluations]
    max_value = max(values) if values else 1.0
    scale_z = 0.65 / max_value if max_value > 0 else 1.0
    dots = []
    stride = max(1, len(evaluations) // 120)
    for normalized, value in evaluations[::stride]:
        x = 0.20 + 1.15 * normalized[0]
        y = 0.55 + 0.58 * normalized[1]
        z = -0.15 + value * scale_z
        dot = chrono.ChBodyEasySphere(0.012, 1000, True, False)
        dot.SetName("minimize objective sample")
        dot.SetFixed(True)
        dot.SetPos(chrono.ChVector3d(x, y, z))
        dot.GetVisualShape(0).SetColor(color(0.76, 0.30, 0.86))
        system.AddBody(dot)
        dots.append(dot)

    trail = []
    for normalized, value in best_history[:: max(1, len(best_history) // 36)]:
        x = 0.20 + 1.15 * normalized[0]
        y = 0.55 + 0.58 * normalized[1]
        z = -0.10 + value * scale_z
        dot = chrono.ChBodyEasySphere(0.020, 1000, True, False)
        dot.SetName("minimize best-iteration sample")
        dot.SetFixed(True)
        dot.SetPos(chrono.ChVector3d(x, y, z))
        dot.GetVisualShape(0).SetColor(color(0.95, 0.74, 0.08))
        system.AddBody(dot)
        trail.append(dot)

    return plate, dots, trail


def build_system():
    reference, params, value, best_history, evaluations = optimize_parameters()
    mass_opt, stiffness_opt, force_opt = params

    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    anchor = chrono.ChBodyEasySphere(0.045, 1000, True, False)
    anchor.SetName("minimize spring anchor and fixed guide")
    anchor.SetFixed(True)
    anchor.SetPos(chrono.ChVector3d(0, 0, 0))
    anchor.GetVisualShape(0).SetColor(color(0.10, 0.10, 0.10))
    system.AddBody(anchor)

    mass = chrono.ChBodyEasySphere(0.070, 1000, True, False)
    mass.SetName("minimize optimized oscillator mass")
    mass.SetMass(mass_opt)
    mass.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    mass.SetPos(chrono.ChVector3d(LENGTH + INITIAL_DISPLACEMENT, 0, 0))
    mass.SetPosDt(chrono.ChVector3d(INITIAL_VELOCITY, 0, 0))
    mass.GetVisualShape(0).SetColor(color(0.08, 0.38, 0.86))
    system.AddBody(mass)

    slider = chrono.ChLinkLockPrismatic()
    slider.SetName("minimize x guide")
    slider.Initialize(mass, anchor, chrono.ChFramed(chrono.ChVector3d(LENGTH, 0, 0), chrono.Q_ROTATE_Z_TO_X))
    system.AddLink(slider)

    spring = chrono.ChLinkTSDA()
    spring.SetName("minimize optimized spring-damper")
    spring.Initialize(mass, anchor, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
    spring.SetRestLength(LENGTH)
    spring.SetSpringCoefficient(stiffness_opt)
    spring.SetDampingCoefficient(DAMPING)
    system.AddLink(spring)
    spring_shape = chrono.ChVisualShapeSpring(0.055, 100, 15)
    spring_shape.SetColor(color(0.88, 0.18, 0.08))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, spring, 0.055, 100, 15, color(0.88, 0.18, 0.08))

    rail = chrono.ChBodyEasyBox(1.15, 0.018, 0.018, 1000, True, False)
    rail.SetName("minimize visible guide rail")
    rail.SetFixed(True)
    rail.SetPos(chrono.ChVector3d(0.55, -0.16, 0))
    rail.GetVisualShape(0).SetColor(color(0.45, 0.45, 0.45))
    system.AddBody(rail)

    force = chrono.ChForce()
    force.SetF_x(chrono.ChFunctionConst(force_opt))
    mass.AddForce(force)

    plate, dots, trail = add_optimization_visuals(system, best_history, evaluations)
    system._minimize_items = {
        "reference": reference,
        "params": params,
        "value": value,
        "best_history": best_history,
        "evaluations": evaluations,
        "plate": plate,
        "dots": dots,
        "trail": trail,
    }
    return system, mass, spring, params, value, best_history, evaluations


def update_visuals(system):
    update_system_visuals(system)


def simulate(duration, step):
    system, mass, spring, params, value, best_history, evaluations = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    return system, mass, spring, params, value, best_history, evaluations


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, mass, spring, params, value, best_history, evaluations = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: minimizeExample.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.15, 1.55, 1.95), chrono.ChVector3d(0.65, 0.45, 0.02))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, mass, spring, params, value, best_history, evaluations)
            next_log += 0.25


def print_state(system, mass, spring, params, value, best_history, evaluations):
    displacement = mass.GetPos().x - LENGTH
    reference, _ = exact_displacement_velocity(
        system.GetChTime(),
        REFERENCE_MASS,
        REFERENCE_STIFFNESS,
        DAMPING,
        REFERENCE_FORCE,
        INITIAL_DISPLACEMENT,
        INITIAL_VELOCITY,
    )
    print(
        f"t={system.GetChTime():6.3f}  "
        f"u={displacement:+.8f}  exact_ref={reference:+.8f}  "
        f"opt_mass={params[0]:+.6f}  opt_k={params[1]:+.6f}  opt_force={params[2]:+.6f}  "
        f"minimum={value:+.3e}  iterations={len(best_history)}  evaluations={len(evaluations)}  "
        f"spring_force={spring.GetForce():+.3f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=T_END)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: minimizeExample.py -> PyChrono mass-spring-damper optimization")
    if args.no_vis:
        system, mass, spring, params, value, best_history, evaluations = simulate(args.duration, args.step)
        print_state(system, mass, spring, params, value, best_history, evaluations)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

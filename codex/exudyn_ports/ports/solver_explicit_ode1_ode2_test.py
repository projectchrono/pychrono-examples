import argparse
import math
import sys
from pathlib import Path

import numpy as np
import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN TestModels/solverExplicitODE1ODE2test.py:
# compare explicit time integrators on equivalent ODE1 and ODE2 oscillator
# formulations. Chrono does not expose EXUDYN's DOPRI5/RK67/RK44/RK33 solver
# switch for GenericODE objects, so this port integrates the same first-order
# system explicitly in Python and renders the ODE1 and ODE2 coordinate-0
# oscillators with visible coil springs.

MASS = 2.0
K0 = 200.0
K1 = 100.0
FORCE1 = 1.0
T_END = 2.0
REFERENCE = 0.40808206181339224
STEP = 1.0e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def rhs(_time, state):
    q0, q1, v0, v1 = state
    return np.array([v0, v1, -K0 / MASS * q0, (FORCE1 - K1 * q1) / MASS], dtype=float)


def exact_q0(time):
    return math.cos(math.sqrt(K0 / MASS) * time)


def step_euler(time, state, h):
    return state + h * rhs(time, state)


def step_midpoint(time, state, h):
    return state + h * rhs(time + 0.5 * h, state + 0.5 * h * rhs(time, state))


def step_rk3(time, state, h):
    k1 = rhs(time, state)
    k2 = rhs(time + 0.5 * h, state + 0.5 * h * k1)
    k3 = rhs(time + h, state - h * k1 + 2.0 * h * k2)
    return state + h * (k1 + 4.0 * k2 + k3) / 6.0


def step_rk4(time, state, h):
    k1 = rhs(time, state)
    k2 = rhs(time + 0.5 * h, state + 0.5 * h * k1)
    k3 = rhs(time + 0.5 * h, state + 0.5 * h * k2)
    k4 = rhs(time + h, state + h * k3)
    return state + h * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0


def integrate(method, h, duration=T_END):
    state = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    time = 0.0
    while time < duration - 1.0e-15:
        dt = min(h, duration - time)
        state = method(time, state, dt)
        time += dt
    return state


def convergence_results():
    cases = []
    for name, method, steps in (
        ("RK4", step_rk4, [1.0e-1, 1.0e-2, 1.0e-3, 1.0e-4]),
        ("RK3", step_rk3, [1.0e-1, 1.0e-2, 1.0e-3, 1.0e-4]),
        ("ExplicitMidpoint", step_midpoint, [1.0e-2, 1.0e-3, 1.0e-4]),
        ("ExplicitEuler", step_euler, [1.0e-4]),
    ):
        for h in steps:
            state = integrate(method, h)
            # ODE1 and ODE2 formulations produce the same q0 here; include both
            # terms as the source test sums both sensor errors.
            err = 2.0 * abs(state[0] - REFERENCE)
            cases.append((name, h, state[0], err))
    total_error = sum(item[3] for item in cases)
    return cases, total_error


def add_oscillator(system, ground, name, y_offset, tint):
    anchor = chrono.ChBodyEasySphere(0.045, 1000, True, False)
    anchor.SetName(name + " fixed anchor")
    anchor.SetFixed(True)
    anchor.SetPos(chrono.ChVector3d(0.0, y_offset, 0.0))
    anchor.GetVisualShape(0).SetColor(color(0.06, 0.06, 0.065))
    system.AddBody(anchor)

    mass = chrono.ChBodyEasySphere(0.085, 1000, True, False)
    mass.SetName(name + " visible coordinate mass")
    mass.SetFixed(True)
    mass.SetPos(chrono.ChVector3d(1.0, y_offset, 0.0))
    mass.GetVisualShape(0).SetColor(tint)
    system.AddBody(mass)

    rail = chrono.ChBodyEasyBox(2.25, 0.018, 0.018, 1000, True, False)
    rail.SetName(name + " coordinate rail")
    rail.SetFixed(True)
    rail.SetPos(chrono.ChVector3d(1.02, y_offset - 0.16, 0.0))
    rail.GetVisualShape(0).SetColor(color(0.46, 0.46, 0.46))
    system.AddBody(rail)

    spring = chrono.ChLinkTSDA()
    spring.SetName(name + " visible coil spring")
    spring.Initialize(mass, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, y_offset, 0))
    spring.SetRestLength(1.0)
    spring.SetSpringCoefficient(K0)
    spring.SetDampingCoefficient(0)
    system.AddLink(spring)
    shape = chrono.ChVisualShapeSpring(0.055, 100, 12)
    shape.SetColor(tint)
    spring.AddVisualShape(shape)
    attach_spring_visual(system, spring, 0.055, 100, 12, tint)
    return mass, spring


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
    ground = chrono.ChBody()
    ground.SetName("explicit solver fixed ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)

    plate = chrono.ChBodyEasyBox(2.55, 0.90, 0.025, 1000, True, False)
    plate.SetName("explicit solver oscillator background")
    plate.SetFixed(True)
    plate.SetPos(chrono.ChVector3d(1.02, 0.0, -0.08))
    plate.GetVisualShape(0).SetColor(color(0.78, 0.78, 0.74))
    plate.GetVisualShape(0).SetOpacity(0.28)
    system.AddBody(plate)

    mass_ode1, spring_ode1 = add_oscillator(system, ground, "ODE1 formulation", 0.22, color(0.10, 0.38, 0.88))
    mass_ode2, spring_ode2 = add_oscillator(system, ground, "ODE2 formulation", -0.22, color(0.90, 0.18, 0.10))
    system._solver_explicit_items = {"masses": (mass_ode1, mass_ode2), "springs": (spring_ode1, spring_ode2)}
    update_visuals(system)
    return system, (mass_ode1, mass_ode2), (spring_ode1, spring_ode2)


def update_visuals(system):
    items = getattr(system, "_solver_explicit_items", None)
    if items is not None:
        x = exact_q0(system.GetChTime())
        for mass, y_offset in zip(items["masses"], (0.22, -0.22)):
            mass.SetPos(chrono.ChVector3d(x, y_offset, 0.0))
            mass.UpdateVisualModel()
    update_system_visuals(system)


def simulate(duration, step):
    system, masses, springs = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, masses, springs


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, masses, springs = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: solverExplicitODE1ODE2test.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.05, -1.35, 1.55), chrono.ChVector3d(0.85, 0.0, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print(f"t={system.GetChTime():6.3f}  q0_exact={exact_q0(system.GetChTime()):+.9f}")
            next_log += 0.25


def print_state():
    cases, total_error = convergence_results()
    for name, h, q0, err in cases:
        print(f"{name:16s} h={h:.1e}  q0={q0:+.12f}  two_form_error={err:.3e}")
    print(f"solverExplicitODE1ODE2 err={total_error:.12f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=T_END)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: solverExplicitODE1ODE2test.py -> PyChrono explicit integrator comparison")
    if args.no_vis:
        print_state()
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

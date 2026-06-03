import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces EXUDYN TestModels/MiniExamples/ObjectGenericODE1.py.
# The source model is q' = A q + b with A=[[0,1],[-100,0]], b=[0,1],
# initial q=[1,0].  This is the first-order form of an undamped oscillator
# q0'' + 100 q0 = 1.  PyChrono replays the ODE1 coordinates and renders the
# equivalent mass-spring oscillator with a visible coil spring.

OMEGA = 10.0
STIFFNESS = 100.0
LOAD = 1.0
INITIAL_Q0 = 1.0
INITIAL_Q1 = 0.0
EQUILIBRIUM = LOAD / STIFFNESS
END_TIME = 1.0
STEP = 0.001


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def rhs(_time, state):
    q0, q1 = state
    return (q1, -STIFFNESS * q0 + LOAD)


def rk4_step(time, state, step):
    k1 = rhs(time, state)
    k2 = rhs(time + 0.5 * step, (state[0] + 0.5 * step * k1[0], state[1] + 0.5 * step * k1[1]))
    k3 = rhs(time + 0.5 * step, (state[0] + 0.5 * step * k2[0], state[1] + 0.5 * step * k2[1]))
    k4 = rhs(time + step, (state[0] + step * k3[0], state[1] + step * k3[1]))
    return (
        state[0] + step * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0,
        state[1] + step * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0,
    )


def exact_solution(time):
    amplitude = INITIAL_Q0 - EQUILIBRIUM
    return (
        EQUILIBRIUM + amplitude * math.cos(OMEGA * time),
        -amplitude * OMEGA * math.sin(OMEGA * time),
    )


def solve(duration=END_TIME, step=STEP):
    time = 0.0
    state = (INITIAL_Q0, INITIAL_Q1)
    while time < duration - 1e-12:
        dt = min(step, duration - time)
        state = rk4_step(time, state, dt)
        time += dt
    return state


def mass_position(q0):
    return chrono.ChVector3d(q0, 0.0, 0.0)


class MutableSegment:
    def __init__(self, system, name, tint, thickness=4):
        self.body = chrono.ChBody()
        self.body.SetName(name)
        self.body.SetFixed(True)
        self.body.EnableCollision(False)
        self.shape = chrono.ChVisualShapeSegment()
        self.shape.SetMutable(True)
        self.shape.SetColor(tint)
        self.shape.SetThickness(thickness)
        self.body.AddVisualShape(self.shape)
        system.AddBody(self.body)

    def update(self, point_a, point_b):
        self.shape.SetLineGeometry(chrono.ChLineSegment(point_a, point_b))
        self.body.UpdateVisualModel()


class ForceArrow:
    def __init__(self, system):
        self.main = MutableSegment(system, "ObjectGenericODE1 constant rhs force", color(0.90, 0.52, 0.06), 5)
        self.tip_a = MutableSegment(system, "ObjectGenericODE1 rhs force tip a", color(0.90, 0.52, 0.06), 4)
        self.tip_b = MutableSegment(system, "ObjectGenericODE1 rhs force tip b", color(0.90, 0.52, 0.06), 4)

    def update(self, mass_pos):
        start = mass_pos + chrono.ChVector3d(0.0, 0.16, 0.0)
        end = start + chrono.ChVector3d(0.24, 0.0, 0.0)
        self.main.update(start, end)
        self.tip_a.update(end, end + chrono.ChVector3d(-0.055, 0.030, 0.0))
        self.tip_b.update(end, end + chrono.ChVector3d(-0.055, -0.030, 0.0))


def make_trace_body(system):
    body = chrono.ChBody()
    body.SetName("ObjectGenericODE1 exact coordinate trace")
    body.SetFixed(True)
    body.EnableCollision(False)
    line = chrono.ChLinePoly(241)
    for i in range(241):
        t = END_TIME * i / 240.0
        q0, q1 = exact_solution(t)
        line.SetPoint(i, chrono.ChVector3d(q0, -0.58, 0.035 * q1))
    shape = chrono.ChVisualShapeLine()
    shape.SetLineGeometry(line)
    shape.SetColor(color(0.18, 0.48, 0.86))
    shape.SetThickness(2)
    body.AddVisualShape(shape)
    system.AddBody(body)
    return body


def make_ground(system):
    ground = chrono.ChBody()
    ground.SetName("ObjectGenericODE1 ground and guides")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    rail = chrono.ChVisualShapeBox(2.7, 0.018, 0.018)
    rail.SetColor(color(0.42, 0.42, 0.42))
    ground.AddVisualShape(rail, chrono.ChFramed(chrono.ChVector3d(0.0, -0.10, 0.0)))

    equilibrium = chrono.ChVisualShapeBox(0.018, 0.20, 0.018)
    equilibrium.SetColor(color(0.05, 0.05, 0.05))
    ground.AddVisualShape(equilibrium, chrono.ChFramed(chrono.ChVector3d(EQUILIBRIUM, -0.10, 0.0)))

    phase_axis = chrono.ChVisualShapeBox(2.7, 0.010, 0.010)
    phase_axis.SetColor(color(0.70, 0.70, 0.70))
    ground.AddVisualShape(phase_axis, chrono.ChFramed(chrono.ChVector3d(0.0, -0.58, 0.0)))

    system.AddBody(ground)
    return ground


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))
    ground = make_ground(system)
    trace = make_trace_body(system)

    anchor = chrono.ChBodyEasySphere(0.045, 1000, True, False)
    anchor.SetName("ObjectGenericODE1 oscillator anchor")
    anchor.SetFixed(True)
    anchor.EnableCollision(False)
    anchor.SetPos(chrono.ChVector3d(-1.15, 0.0, 0.0))
    anchor.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.08))
    system.AddBody(anchor)

    mass = chrono.ChBodyEasySphere(0.075, 1000, True, False)
    mass.SetName("ObjectGenericODE1 coordinate mass")
    mass.SetFixed(True)
    mass.EnableCollision(False)
    mass.SetPos(mass_position(INITIAL_Q0))
    mass.GetVisualShape(0).SetColor(color(0.12, 0.38, 0.88))
    system.AddBody(mass)

    phase_dot = chrono.ChBodyEasySphere(0.035, 1000, True, False)
    phase_dot.SetName("ObjectGenericODE1 phase-space marker")
    phase_dot.SetFixed(True)
    phase_dot.EnableCollision(False)
    phase_dot.GetVisualShape(0).SetColor(color(0.85, 0.18, 0.12))
    system.AddBody(phase_dot)

    spring = chrono.ChLinkTSDA()
    spring.SetName("ObjectGenericODE1 equivalent oscillator spring")
    spring.Initialize(mass, anchor, True, chrono.ChVector3d(0.0, 0.0, 0.0), chrono.ChVector3d(0.0, 0.0, 0.0))
    spring.SetRestLength(1.16)
    spring.SetSpringCoefficient(0.0)
    spring.SetDampingCoefficient(0.0)
    system.AddLink(spring)
    spring_shape = chrono.ChVisualShapeSpring(0.045, 90, 12)
    spring_shape.SetColor(color(0.86, 0.16, 0.10))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, spring, 0.045, 90, 12, color(0.86, 0.16, 0.10))

    items = {
        "ground": ground,
        "trace": trace,
        "anchor": anchor,
        "mass": mass,
        "phase_dot": phase_dot,
        "spring": spring,
        "force_arrow": ForceArrow(system),
        "time": 0.0,
        "state": (INITIAL_Q0, INITIAL_Q1),
    }
    system._object_generic_ode1_items = items
    update_visuals(system)
    return system, items


def set_state(items, time, state):
    q0, q1 = state
    items["time"] = time
    items["state"] = state
    items["mass"].SetPos(mass_position(q0))
    items["mass"].SetPosDt(chrono.ChVector3d(q1, 0.0, 0.0))
    items["mass"].UpdateVisualModel()
    items["phase_dot"].SetPos(chrono.ChVector3d(q0, -0.58, 0.035 * q1))
    items["phase_dot"].UpdateVisualModel()
    items["force_arrow"].update(items["mass"].GetPos())


def update_visuals(system):
    items = system._object_generic_ode1_items
    target_time = system.GetChTime()
    time = items["time"]
    state = items["state"]
    while time < target_time - 1e-12:
        dt = min(STEP, target_time - time)
        state = rk4_step(time, state, dt)
        time += dt
    set_state(items, target_time, state)
    update_system_visuals(system)


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration - 1e-12:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ObjectGenericODE1.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.15, -3.15, 1.25), chrono.ChVector3d(0.0, -0.16, 0.0))
    vis.AddTypicalLights()

    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def print_result(duration, step):
    state = solve(duration, step)
    exact = exact_solution(duration)
    error = math.sqrt((state[0] - exact[0]) ** 2 + (state[1] - exact[1]) ** 2)
    print(
        f"generic_ode1: t={duration:7.3f}  "
        f"q=({state[0]:+.12f},{state[1]:+.12f})  "
        f"exact=({exact[0]:+.12f},{exact[1]:+.12f})  "
        f"state_error={error:.3e}  testResult={state[0]:+.12f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ObjectGenericODE1.py -> PyChrono GenericODE1 oscillator replay")
    if args.no_vis:
        print_result(args.duration, args.step)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

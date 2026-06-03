import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces EXUDYN TestModels/loadUserFunctionTest.py:
# two mass-point spring systems using a Python rotating load user function and
# a symbolic cosine-scaled load user function.  Chrono replays the same RK4
# point-mass equations while rendering both spring connectors as actual coils.

MASS = 1.0
REST_LENGTH = 1.0
INITIAL_X = 1.05
STIFFNESS = 100.0
DAMPING = 1.0
LOAD = 10.0
LOAD_OMEGA = 4.0 * math.pi
END_TIME = 10.0
SOURCE_STEP = 0.005
REPLAY_STEP = 0.001
REFERENCE_RESULT = 1.8051173706570725


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def add_vec(a, b):
    return tuple(a[i] + b[i] for i in range(6))


def scale_vec(a, s):
    return tuple(x * s for x in a)


def load_vector(case_name, time):
    if case_name == "PythonUserFunction":
        return (LOAD * math.sin(LOAD_OMEGA * time), LOAD * math.cos(LOAD_OMEGA * time), 0.0)
    return (LOAD * math.cos(LOAD_OMEGA * time), 0.0, 0.0)


def rhs(time, state, case_name):
    x, y, z, vx, vy, vz = state
    length = math.sqrt(x * x + y * y + z * z)
    if length < 1e-12:
        unit = (1.0, 0.0, 0.0)
    else:
        unit = (x / length, y / length, z / length)

    radial_velocity = vx * unit[0] + vy * unit[1] + vz * unit[2]
    spring_force = -(STIFFNESS * (length - REST_LENGTH) + DAMPING * radial_velocity)
    load = load_vector(case_name, time)
    acceleration = tuple((spring_force * unit[i] + load[i]) / MASS for i in range(3))
    return (vx, vy, vz, acceleration[0], acceleration[1], acceleration[2])


def rk4_step(time, state, step, case_name):
    k1 = rhs(time, state, case_name)
    k2 = rhs(time + 0.5 * step, add_vec(state, scale_vec(k1, 0.5 * step)), case_name)
    k3 = rhs(time + 0.5 * step, add_vec(state, scale_vec(k2, 0.5 * step)), case_name)
    k4 = rhs(time + step, add_vec(state, scale_vec(k3, step)), case_name)
    return tuple(state[i] + step * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0 for i in range(6))


def solve_case(case_name, duration=END_TIME, step=SOURCE_STEP):
    time = 0.0
    state = (INITIAL_X, 0.0, 0.0, 0.0, 0.0, 0.0)
    while time < duration - 1e-12:
        dt = min(step, duration - time)
        state = rk4_step(time, state, dt, case_name)
        time += dt
    return state


def norm3(state):
    return math.sqrt(state[0] * state[0] + state[1] * state[1] + state[2] * state[2])


def to_chrono_position(state, lane_y):
    return chrono.ChVector3d(state[0], lane_y, state[1])


def to_chrono_vector(vector):
    return chrono.ChVector3d(vector[0], 0.0, vector[1])


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


class LoadArrow:
    def __init__(self, system, name, tint):
        self.main = MutableSegment(system, name, tint, 5)
        self.tip_a = MutableSegment(system, name + " tip a", tint, 4)
        self.tip_b = MutableSegment(system, name + " tip b", tint, 4)

    def update(self, mass_pos, load):
        direction = to_chrono_vector(load)
        length = direction.Length()
        start = mass_pos + chrono.ChVector3d(0.0, 0.0, 0.12)
        if length < 1e-12:
            end = start
            tangent = chrono.ChVector3d(1.0, 0.0, 0.0)
        else:
            direction *= 0.025
            end = start + direction
            tangent = direction
            tangent.Normalize()
        normal = chrono.ChVector3d(-tangent.z, 0.0, tangent.x)
        self.main.update(start, end)
        self.tip_a.update(end, end - tangent * 0.055 + normal * 0.030)
        self.tip_b.update(end, end - tangent * 0.055 - normal * 0.030)


class CaseVisual:
    def __init__(self, system, case_name, lane_y, tint):
        self.case_name = case_name
        self.lane_y = lane_y
        self.state = (INITIAL_X, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.time = 0.0

        self.anchor = chrono.ChBodyEasySphere(0.035, 1000, True, False)
        self.anchor.SetName(case_name + " spring anchor")
        self.anchor.SetFixed(True)
        self.anchor.EnableCollision(False)
        self.anchor.SetPos(chrono.ChVector3d(0.0, lane_y, 0.0))
        self.anchor.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.08))
        system.AddBody(self.anchor)

        self.mass = chrono.ChBodyEasySphere(0.060, 1000, True, False)
        self.mass.SetName(case_name + " loaded mass point")
        self.mass.SetFixed(True)
        self.mass.EnableCollision(False)
        self.mass.SetMass(MASS)
        self.mass.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
        self.mass.SetPos(to_chrono_position(self.state, lane_y))
        self.mass.GetVisualShape(0).SetColor(tint)
        system.AddBody(self.mass)

        self.spring = chrono.ChLinkTSDA()
        self.spring.SetName(case_name + " spring-damper")
        self.spring.Initialize(self.mass, self.anchor, True, chrono.ChVector3d(0.0, 0.0, 0.0), chrono.ChVector3d(0.0, 0.0, 0.0))
        self.spring.SetRestLength(REST_LENGTH)
        self.spring.SetSpringCoefficient(0.0)
        self.spring.SetDampingCoefficient(0.0)
        system.AddLink(self.spring)
        spring_shape = chrono.ChVisualShapeSpring(0.035, 90, 12)
        spring_shape.SetColor(color(0.86, 0.16, 0.10))
        self.spring.AddVisualShape(spring_shape)
        attach_spring_visual(system, self.spring, 0.035, 90, 12, color(0.86, 0.16, 0.10))

        self.load_arrow = LoadArrow(system, case_name + " load-user-function arrow", color(0.92, 0.55, 0.05))
        self.reference = self._make_reference_circle(system)
        self.update()

    def _make_reference_circle(self, system):
        body = chrono.ChBody()
        body.SetName(self.case_name + " reference spring length")
        body.SetFixed(True)
        body.EnableCollision(False)
        line = chrono.ChLinePoly(97)
        for i in range(97):
            angle = 2.0 * math.pi * i / 96.0
            line.SetPoint(i, chrono.ChVector3d(REST_LENGTH * math.cos(angle), self.lane_y, REST_LENGTH * math.sin(angle)))
        shape = chrono.ChVisualShapeLine()
        shape.SetLineGeometry(line)
        shape.SetColor(color(0.55, 0.55, 0.55))
        shape.SetThickness(1)
        body.AddVisualShape(shape)
        system.AddBody(body)
        return body

    def advance_to(self, target_time):
        while self.time < target_time - 1e-12:
            dt = min(REPLAY_STEP, target_time - self.time)
            self.state = rk4_step(self.time, self.state, dt, self.case_name)
            self.time += dt
        self.update()

    def set_state(self, time, state):
        self.time = time
        self.state = state
        self.update()

    def update(self):
        self.mass.SetPos(to_chrono_position(self.state, self.lane_y))
        self.mass.SetPosDt(to_chrono_vector((self.state[3], self.state[4], self.state[5])))
        self.mass.UpdateVisualModel()
        self.load_arrow.update(self.mass.GetPos(), load_vector(self.case_name, self.time))


def make_background(system):
    ground = chrono.ChBody()
    ground.SetName("load user-function test ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    base = chrono.ChVisualShapeBox(2.4, 0.04, 0.025)
    base.SetColor(color(0.45, 0.45, 0.45))
    ground.AddVisualShape(base, chrono.ChFramed(chrono.ChVector3d(0.0, -0.35, -1.08)))
    ground.AddVisualShape(base, chrono.ChFramed(chrono.ChVector3d(0.0, 0.35, -1.08)))

    x_axis = chrono.ChVisualShapeBox(2.3, 0.012, 0.012)
    x_axis.SetColor(color(0.18, 0.18, 0.18))
    ground.AddVisualShape(x_axis, chrono.ChFramed(chrono.ChVector3d(0.15, -0.35, 0.0)))
    ground.AddVisualShape(x_axis, chrono.ChFramed(chrono.ChVector3d(0.15, 0.35, 0.0)))

    system.AddBody(ground)
    return ground


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))
    make_background(system)
    cases = [
        CaseVisual(system, "PythonUserFunction", -0.35, color(0.12, 0.38, 0.88)),
        CaseVisual(system, "SymbolicUserFunction", 0.35, color(0.12, 0.58, 0.28)),
    ]
    system._load_user_function_cases = cases
    update_visuals(system)
    return system, cases


def update_visuals(system):
    target_time = system.GetChTime()
    for case in system._load_user_function_cases:
        case.advance_to(target_time)
    update_system_visuals(system)


def simulate(duration, step):
    system, cases = build_system()
    while system.GetChTime() < duration - 1e-12:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, cases


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _ = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: loadUserFunctionTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.05, -3.0, 1.35), chrono.ChVector3d(0.25, 0.0, -0.12))
    vis.AddTypicalLights()

    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def print_results(duration, step):
    total = 0.0
    for case_name in ("PythonUserFunction", "SymbolicUserFunction"):
        state = solve_case(case_name, duration, step)
        value = norm3(state)
        total += value
        print(
            f"{case_name}: t={duration:7.3f}  "
            f"pos=({state[0]:+.9f},{state[1]:+.9f},{state[2]:+.9f})  "
            f"vel=({state[3]:+.9f},{state[4]:+.9f},{state[5]:+.9f})  "
            f"norm={value:.12f}"
        )
    print(f"result={total:.15f}  reference_error={total - REFERENCE_RESULT:+.3e}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=SOURCE_STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: loadUserFunctionTest.py -> PyChrono load user-function spring test")
    if args.no_vis:
        print_results(args.duration, args.step)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

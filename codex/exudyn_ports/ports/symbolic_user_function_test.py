import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces EXUDYN TestModels/symbolicUserFunctionTest.py:
# one mass point with a symbolic nonlinear spring-force user function and a
# sinusoidal coordinate-load user function.  PyChrono replays the same scalar
# equation while rendering the spring connector as a native coil plus fallback.

REFERENCE_POSITION = 1.05
MASS = 1.0
REST_LENGTH = 0.1
STIFFNESS = 100.0
DAMPING = 1.0
LOAD = 10.0
LOAD_OMEGA = 10.0 * (2.0 * math.pi)
END_TIME = 50.0
STEP = 0.005
VISUAL_STEP = 0.001
REFERENCE_NORM = 0.10039884426884882


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def sign(value):
    if value > 0.0:
        return 1.0
    if value < 0.0:
        return -1.0
    return 0.0


def load_user_function(time):
    return LOAD * math.sin(LOAD_OMEGA * time)


def spring_force_user_function(delta_length, delta_length_dt):
    return 10.0 * DAMPING * delta_length_dt + STIFFNESS * sign(delta_length) * abs(delta_length) ** 1.2


def acceleration(time, coordinate, velocity):
    position = REFERENCE_POSITION + coordinate
    direction = sign(position) or 1.0
    delta_length = abs(position) - REST_LENGTH
    delta_length_dt = velocity * direction
    spring_force = spring_force_user_function(delta_length, delta_length_dt)
    return load_user_function(time) - spring_force * direction


def rk4_step(time, coordinate, velocity, step):
    def rhs(t, q, v):
        return v, acceleration(t, q, v)

    k1q, k1v = rhs(time, coordinate, velocity)
    k2q, k2v = rhs(time + 0.5 * step, coordinate + 0.5 * step * k1q, velocity + 0.5 * step * k1v)
    k3q, k3v = rhs(time + 0.5 * step, coordinate + 0.5 * step * k2q, velocity + 0.5 * step * k2v)
    k4q, k4v = rhs(time + step, coordinate + step * k3q, velocity + step * k3v)
    coordinate += step * (k1q + 2.0 * k2q + 2.0 * k3q + k4q) / 6.0
    velocity += step * (k1v + 2.0 * k2v + 2.0 * k3v + k4v) / 6.0
    return coordinate, velocity


def solve(duration=END_TIME, step=STEP):
    time = 0.0
    coordinate = 0.0
    velocity = 0.0
    while time < duration - 1.0e-12:
        dt = min(step, duration - time)
        coordinate, velocity = rk4_step(time, coordinate, velocity, dt)
        time += dt
    return coordinate, velocity


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
    def __init__(self, system):
        self.main = MutableSegment(system, "symbolic UF coordinate-load arrow", color(0.92, 0.55, 0.05), 5)
        self.tip_a = MutableSegment(system, "symbolic UF coordinate-load arrow tip a", color(0.92, 0.55, 0.05), 4)
        self.tip_b = MutableSegment(system, "symbolic UF coordinate-load arrow tip b", color(0.92, 0.55, 0.05), 4)

    def update(self, mass_pos, load_value):
        span = max(-0.28, min(0.28, 0.028 * load_value))
        start = mass_pos + chrono.ChVector3d(0.0, 0.16, 0.0)
        end = start + chrono.ChVector3d(span, 0.0, 0.0)
        direction = 1.0 if span >= 0.0 else -1.0
        self.main.update(start, end)
        self.tip_a.update(end, end + chrono.ChVector3d(-0.055 * direction, 0.030, 0.0))
        self.tip_b.update(end, end + chrono.ChVector3d(-0.055 * direction, -0.030, 0.0))


class SpringForceBar:
    def __init__(self, system):
        self.segment = MutableSegment(system, "symbolic UF nonlinear spring-force bar", color(0.72, 0.08, 0.08), 5)

    def update(self, mass_pos, force_value):
        span = max(-0.30, min(0.30, 0.016 * force_value))
        start = mass_pos + chrono.ChVector3d(0.0, -0.16, 0.0)
        self.segment.update(start, start - chrono.ChVector3d(span, 0.0, 0.0))


def make_ground(system):
    ground = chrono.ChBody()
    ground.SetName("symbolic user-function test ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    wall = chrono.ChVisualShapeBox(0.060, 0.42, 0.18)
    wall.SetColor(color(0.55, 0.56, 0.55))
    ground.AddVisualShape(wall, chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0)))

    rail = chrono.ChVisualShapeBox(1.45, 0.018, 0.018)
    rail.SetColor(color(0.42, 0.42, 0.42))
    ground.AddVisualShape(rail, chrono.ChFramed(chrono.ChVector3d(0.44, -0.18, 0.0)))

    rest_positive = chrono.ChVisualShapeBox(0.014, 0.18, 0.018)
    rest_positive.SetColor(color(0.08, 0.08, 0.08))
    ground.AddVisualShape(rest_positive, chrono.ChFramed(chrono.ChVector3d(REST_LENGTH, -0.18, 0.0)))
    ground.AddVisualShape(rest_positive, chrono.ChFramed(chrono.ChVector3d(-REST_LENGTH, -0.18, 0.0)))

    system.AddBody(ground)
    return ground


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))
    ground = make_ground(system)

    mass = chrono.ChBodyEasySphere(0.065, 1000, True, False)
    mass.SetName("symbolic UF mass point")
    mass.SetFixed(True)
    mass.EnableCollision(False)
    mass.SetMass(MASS)
    mass.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    mass.SetPos(chrono.ChVector3d(REFERENCE_POSITION, 0.0, 0.0))
    mass.GetVisualShape(0).SetColor(color(0.12, 0.38, 0.88))
    system.AddBody(mass)

    anchor = chrono.ChBodyEasySphere(0.035, 1000, True, False)
    anchor.SetName("symbolic UF spring anchor")
    anchor.SetFixed(True)
    anchor.EnableCollision(False)
    anchor.SetPos(chrono.ChVector3d(0.0, 0.0, 0.0))
    anchor.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.08))
    system.AddBody(anchor)

    spring = chrono.ChLinkTSDA()
    spring.SetName("symbolic nonlinear spring-force user-function analogue")
    spring.Initialize(mass, anchor, True, chrono.ChVector3d(0.0, 0.0, 0.0), chrono.ChVector3d(0.0, 0.0, 0.0))
    spring.SetRestLength(REST_LENGTH)
    spring.SetSpringCoefficient(0.0)
    spring.SetDampingCoefficient(0.0)
    system.AddLink(spring)
    spring_shape = chrono.ChVisualShapeSpring(0.035, 100, 13)
    spring_shape.SetColor(color(0.86, 0.16, 0.10))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, spring, 0.035, 100, 13, color(0.86, 0.16, 0.10))

    items = {
        "ground": ground,
        "anchor": anchor,
        "mass": mass,
        "spring": spring,
        "load_arrow": LoadArrow(system),
        "spring_force_bar": SpringForceBar(system),
        "time": 0.0,
        "coordinate": 0.0,
        "velocity": 0.0,
        "load": 0.0,
        "spring_force": 0.0,
    }
    system._symbolic_user_function_items = items
    update_visuals(system)
    return system, items


def set_state(items, time, coordinate, velocity):
    position = REFERENCE_POSITION + coordinate
    direction = sign(position) or 1.0
    delta_length = abs(position) - REST_LENGTH
    delta_length_dt = velocity * direction
    spring_force = spring_force_user_function(delta_length, delta_length_dt)
    load_value = load_user_function(time)

    items["time"] = time
    items["coordinate"] = coordinate
    items["velocity"] = velocity
    items["load"] = load_value
    items["spring_force"] = spring_force
    items["mass"].SetPos(chrono.ChVector3d(position, 0.0, 0.0))
    items["mass"].SetPosDt(chrono.ChVector3d(velocity, 0.0, 0.0))
    items["mass"].UpdateVisualModel()
    items["load_arrow"].update(items["mass"].GetPos(), load_value)
    items["spring_force_bar"].update(items["mass"].GetPos(), spring_force)


def update_visuals(system):
    items = system._symbolic_user_function_items
    target_time = system.GetChTime()
    time = items["time"]
    coordinate = items["coordinate"]
    velocity = items["velocity"]
    while time < target_time - 1.0e-12:
        dt = min(VISUAL_STEP, target_time - time)
        coordinate, velocity = rk4_step(time, coordinate, velocity, dt)
        time += dt
    set_state(items, target_time, coordinate, velocity)
    update_system_visuals(system)


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration - 1.0e-12:
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
    vis.SetWindowTitle("EXUDYN port: symbolicUserFunctionTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.38, -1.45, 0.82), chrono.ChVector3d(0.35, 0.0, 0.0))
    vis.AddTypicalLights()

    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def print_result(duration, step):
    coordinate, velocity = solve(duration, step)
    position = REFERENCE_POSITION + coordinate
    delta_length = abs(position) - REST_LENGTH
    spring_force = spring_force_user_function(delta_length, velocity * (sign(position) or 1.0))
    norm = abs(position)
    print(
        f"symbolic_user_function: t={duration:7.3f}  "
        f"position={position:+.12f}  coordinate={coordinate:+.12f}  "
        f"velocity={velocity:+.12f}  spring_force={spring_force:+.9f}  "
        f"norm={norm:.15f}  reference_error={norm - REFERENCE_NORM:+.3e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: symbolicUserFunctionTest.py -> PyChrono symbolic spring/load user functions")
    if args.no_vis:
        print_result(args.duration, args.step)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

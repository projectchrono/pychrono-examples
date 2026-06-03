import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces EXUDYN TestModels/springDamperUserFunctionTest.py:
# a 1D Duffing oscillator with a user-defined coordinate spring-damper force
# and a swept-frequency user load.  The PyChrono model constrains a mass along
# X, applies the nonlinear force with a TSDA functor, and uses native coil
# spring visualization plus the screenshot fallback.

LENGTH = 0.5
MASS = 1.6
STIFFNESS = 4000.0
DAMPING = 4.0
LOAD0 = 80.0
OMEGA0 = math.sqrt(STIFFNESS / MASS)
FREQ0 = 0.0
FREQ1 = OMEGA0 / (2.0 * math.pi)
END_TIME = 50.0
SOURCE_STEPS = 5000
SOURCE_STEP = END_TIME / SOURCE_STEPS
STEP = 1.0e-3
REFERENCE_FINAL_X = 0.5062872273010898


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def sweep(time, duration=END_TIME, freq0=FREQ0, freq1=FREQ1):
    slope = (freq1 - freq0) / duration
    return math.sin(2.0 * math.pi * (freq0 + 0.5 * slope * time) * time)


def user_load(time):
    return LOAD0 * sweep(time)


def spring_user_force(displacement, velocity):
    return 0.1 * STIFFNESS * displacement + STIFFNESS * displacement**3 + DAMPING * velocity


def acceleration(time, displacement, velocity):
    return (user_load(time) - spring_user_force(displacement, velocity)) / MASS


def rk4_step(time, displacement, velocity, step):
    def rhs(t, q, v):
        return v, acceleration(t, q, v)

    k1q, k1v = rhs(time, displacement, velocity)
    k2q, k2v = rhs(time + 0.5 * step, displacement + 0.5 * step * k1q, velocity + 0.5 * step * k1v)
    k3q, k3v = rhs(time + 0.5 * step, displacement + 0.5 * step * k2q, velocity + 0.5 * step * k2v)
    k4q, k4v = rhs(time + step, displacement + step * k3q, velocity + step * k3v)
    displacement += step * (k1q + 2.0 * k2q + 2.0 * k3q + k4q) / 6.0
    velocity += step * (k1v + 2.0 * k2v + 2.0 * k3v + k4v) / 6.0
    return displacement, velocity


class DuffingSpringState:
    def __init__(self):
        self.force = 0.0
        self.total_force = 0.0
        self.load = 0.0
        self.displacement = 0.0
        self.velocity = 0.0

    def update(self, time, x, velocity):
        displacement = x - LENGTH
        self.displacement = displacement
        self.velocity = velocity
        self.force = spring_user_force(displacement, velocity)
        self.load = user_load(time)
        self.total_force = self.load - self.force
        return self.total_force


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


class LoadVisual:
    def __init__(self, system):
        self.arrow = MutableSegment(system, "swept user-load arrow", color(0.90, 0.52, 0.06), 5)
        self.tip_a = MutableSegment(system, "swept user-load arrow tip a", color(0.90, 0.52, 0.06), 4)
        self.tip_b = MutableSegment(system, "swept user-load arrow tip b", color(0.90, 0.52, 0.06), 4)

    def update(self, mass_pos, load_value):
        scale = max(-0.35, min(0.35, load_value / LOAD0 * 0.28))
        start = mass_pos + chrono.ChVector3d(0.0, 0.18, 0.0)
        end = start + chrono.ChVector3d(scale, 0.0, 0.0)
        sign = 1.0 if scale >= 0 else -1.0
        self.arrow.update(start, end)
        self.tip_a.update(end, end + chrono.ChVector3d(-0.055 * sign, 0.030, 0.0))
        self.tip_b.update(end, end + chrono.ChVector3d(-0.055 * sign, -0.030, 0.0))


def make_ground(system):
    ground = chrono.ChBody()
    ground.SetName("Duffing oscillator ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    wall = chrono.ChVisualShapeBox(0.08, 0.62, 0.22)
    wall.SetColor(color(0.55, 0.56, 0.55))
    ground.AddVisualShape(wall, chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0)))

    rail = chrono.ChVisualShapeBox(1.28, 0.020, 0.020)
    rail.SetColor(color(0.42, 0.42, 0.42))
    ground.AddVisualShape(rail, chrono.ChFramed(chrono.ChVector3d(0.64, -0.17, 0.0)))

    reference = chrono.ChVisualShapeBox(0.015, 0.17, 0.020)
    reference.SetColor(color(0.12, 0.12, 0.12))
    ground.AddVisualShape(reference, chrono.ChFramed(chrono.ChVector3d(LENGTH, -0.17, 0.0)))

    system.AddBody(ground)
    return ground


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))

    ground = make_ground(system)

    anchor = chrono.ChBodyEasySphere(0.040, 1000, True, False)
    anchor.SetName("spring user-function anchor")
    anchor.SetFixed(True)
    anchor.SetPos(chrono.ChVector3d(0.0, 0.0, 0.0))
    anchor.EnableCollision(False)
    anchor.GetVisualShape(0).SetColor(color(0.10, 0.10, 0.10))
    system.AddBody(anchor)

    mass = chrono.ChBodyEasySphere(0.070, 1000, True, False)
    mass.SetName("Duffing oscillator mass")
    mass.EnableCollision(False)
    mass.SetFixed(True)
    mass.SetMass(MASS)
    mass.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    mass.SetPos(chrono.ChVector3d(LENGTH, 0.0, 0.0))
    mass.SetPosDt(chrono.ChVector3d(0.0, 0.0, 0.0))
    mass.GetVisualShape(0).SetColor(color(0.14, 0.38, 0.88))
    system.AddBody(mass)

    spring = chrono.ChLinkTSDA()
    spring.SetName("Duffing CoordinateSpringDamper user-function analogue")
    spring.Initialize(mass, ground, True, chrono.ChVector3d(0.0, 0.0, 0.0), chrono.ChVector3d(0.0, 0.0, 0.0))
    spring.SetRestLength(LENGTH)
    spring.SetSpringCoefficient(0.0)
    spring.SetDampingCoefficient(0.0)
    spring._duffing_state = DuffingSpringState()
    system.AddLink(spring)

    spring_shape = chrono.ChVisualShapeSpring(0.055, 120, 14)
    spring_shape.SetColor(color(0.85, 0.18, 0.12))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, spring, 0.055, 120, 14, color(0.85, 0.18, 0.12))

    load_visual = LoadVisual(system)
    items = {"ground": ground, "mass": mass, "spring": spring, "load_visual": load_visual, "time": 0.0, "q": 0.0, "v": 0.0}
    system._spring_damper_user_function_items = items
    update_visuals(system)
    return system, items


def set_mass_state(items, time, displacement, velocity):
    mass = items["mass"]
    x = LENGTH + displacement
    mass.SetPos(chrono.ChVector3d(x, 0.0, 0.0))
    mass.SetPosDt(chrono.ChVector3d(velocity, 0.0, 0.0))
    mass.UpdateVisualModel()
    items["spring"]._duffing_state.update(time, x, velocity)
    items["time"] = time
    items["q"] = displacement
    items["v"] = velocity


def sync_state_to_system_time(system):
    items = system._spring_damper_user_function_items
    target_time = system.GetChTime()
    time = items["time"]
    displacement = items["q"]
    velocity = items["v"]
    while time < target_time - 1e-12:
        dt = min(STEP, target_time - time)
        displacement, velocity = rk4_step(time, displacement, velocity, dt)
        time += dt
    set_mass_state(items, target_time, displacement, velocity)


def update_visuals(system):
    sync_state_to_system_time(system)
    items = system._spring_damper_user_function_items
    load_value = items["spring"]._duffing_state.load
    items["load_visual"].update(items["mass"].GetPos(), load_value)
    update_system_visuals(system)


def simulate(duration, step):
    system, items = build_system()
    time = 0.0
    q = 0.0
    v = 0.0
    while time < duration - 1e-12:
        set_mass_state(items, time, q, v)
        update_visuals(system)
        dt = min(step, duration - time)
        q, v = rk4_step(time, q, v, dt)
        system.DoStepDynamics(dt)
        time += dt
    set_mass_state(items, duration, q, v)
    update_visuals(system)
    return system, items


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: springDamperUserFunctionTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.55, -1.15, 0.72), chrono.ChVector3d(0.50, 0.0, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    time = 0.0
    q = 0.0
    v = 0.0
    while vis.Run() and system.GetChTime() < duration:
        set_mass_state(items, time, q, v)
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        dt = min(step, duration - time)
        q, v = rk4_step(time, q, v, dt)
        system.DoStepDynamics(dt)
        time += dt
        if time >= next_log:
            print_state(system, items)
            next_log += max(0.25, duration / 5.0)


def print_state(system, items):
    mass = items["mass"]
    spring = items["spring"]
    spring_state = spring._duffing_state
    x = mass.GetPos().x
    print(
        f"t={system.GetChTime():7.3f}  "
        f"x={x:+.9f}  vx={mass.GetPosDt().x:+.9f}  "
        f"load={spring_state.load:+.6f}  "
        f"spring_force={spring_state.force:+.6f}  "
        f"total_force={spring_state.total_force:+.6f}  "
        f"reference_error={x - REFERENCE_FINAL_X:+.6e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: springDamperUserFunctionTest.py -> PyChrono Duffing spring/load user functions")
    if args.no_vis:
        system, items = simulate(args.duration, args.step)
        print_state(system, items)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces EXUDYN Examples/springMassFriction.py:
# a 1D mass-spring oscillator with a user load and a spring force user function
# containing EXUDYN's regularized Stribeck friction.  The scalar source
# equations are integrated directly, while PyChrono provides the rendered mass,
# wall, rail, load arrow, and native coil spring visualization.

LENGTH = 1.0
STIFFNESS = 1600.0
MASS = 1.0
LOAD_AMPLITUDE = 200.0
FRICTION_NORMAL_FORCE = 20.0
STEP = 1.0e-4
END_TIME = 1.0


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def stribeck_function(velocity, mu_dynamic, mu_static_offset, mu_viscous=0.0, exp_vel=1.0e-3, reg_vel=1.0e-3):
    if abs(velocity) <= reg_vel and reg_vel != 0.0:
        return (mu_dynamic + mu_static_offset) * velocity / reg_vel
    sign = 1.0 if velocity > 0.0 else -1.0
    relative_velocity = abs(velocity) - reg_vel
    return sign * (mu_dynamic + mu_static_offset * math.exp(-relative_velocity / exp_vel) + mu_viscous * relative_velocity)


def user_load(time):
    if time >= 0.5:
        return 0.2 * LOAD_AMPLITUDE
    return LOAD_AMPLITUDE


def spring_user_force(displacement, velocity):
    friction = FRICTION_NORMAL_FORCE * stribeck_function(velocity, 1.0, 0.1)
    return STIFFNESS * displacement + friction


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


class LoadArrowVisual:
    def __init__(self, system):
        self.arrow = MutableSegment(system, "spring friction user-load arrow", color(0.95, 0.52, 0.06), 5)
        self.tip_a = MutableSegment(system, "spring friction user-load arrow tip a", color(0.95, 0.52, 0.06), 4)
        self.tip_b = MutableSegment(system, "spring friction user-load arrow tip b", color(0.95, 0.52, 0.06), 4)

    def update(self, mass_position, load_value):
        scale = load_value / LOAD_AMPLITUDE * 0.38
        start = mass_position + chrono.ChVector3d(0.0, 0.23, 0.0)
        end = start + chrono.ChVector3d(scale, 0.0, 0.0)
        self.arrow.update(start, end)
        self.tip_a.update(end, end + chrono.ChVector3d(-0.055, 0.030, 0.0))
        self.tip_b.update(end, end + chrono.ChVector3d(-0.055, -0.030, 0.0))


class FrictionVisual:
    def __init__(self, system):
        self.bar = MutableSegment(system, "regularized Stribeck friction indicator", color(0.70, 0.10, 0.10), 5)

    def update(self, mass_position, friction_force):
        scale = max(-0.28, min(0.28, friction_force / FRICTION_NORMAL_FORCE * 0.16))
        start = mass_position + chrono.ChVector3d(0.0, -0.23, 0.0)
        end = start - chrono.ChVector3d(scale, 0.0, 0.0)
        self.bar.update(start, end)


def make_ground(system):
    ground = chrono.ChBody()
    ground.SetName("spring-mass-friction ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    wall = chrono.ChVisualShapeBox(0.20, 0.40, 0.40)
    wall.SetColor(color(0.55, 0.56, 0.55))
    ground.AddVisualShape(wall, chrono.ChFramed(chrono.ChVector3d(-0.10, 0.0, 0.0)))

    rail = chrono.ChVisualShapeBox(1.65, 0.025, 0.025)
    rail.SetColor(color(0.42, 0.42, 0.42))
    ground.AddVisualShape(rail, chrono.ChFramed(chrono.ChVector3d(0.82, -0.17, 0.0)))

    reference = chrono.ChVisualShapeBox(0.018, 0.17, 0.020)
    reference.SetColor(color(0.10, 0.10, 0.10))
    ground.AddVisualShape(reference, chrono.ChFramed(chrono.ChVector3d(LENGTH, -0.17, 0.0)))

    system.AddBody(ground)
    return ground


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))

    ground = make_ground(system)

    anchor = chrono.ChBodyEasySphere(0.040, 1000, True, False)
    anchor.SetName("spring-mass-friction anchor")
    anchor.SetFixed(True)
    anchor.SetPos(chrono.ChVector3d(0.0, 0.0, 0.0))
    anchor.EnableCollision(False)
    anchor.GetVisualShape(0).SetColor(color(0.10, 0.10, 0.10))
    system.AddBody(anchor)

    mass = chrono.ChBodyEasySphere(0.10 * LENGTH, 1000, True, False)
    mass.SetName("spring-mass-friction mass")
    mass.SetFixed(True)
    mass.EnableCollision(False)
    mass.SetPos(chrono.ChVector3d(LENGTH, 0.0, 0.0))
    mass.GetVisualShape(0).SetColor(color(0.95, 0.48, 0.08))
    system.AddBody(mass)

    spring = chrono.ChLinkTSDA()
    spring.SetName("SpringDamper user-function friction analogue")
    spring.Initialize(mass, ground, True, chrono.ChVector3d(0.0, 0.0, 0.0), chrono.ChVector3d(0.0, 0.0, 0.0))
    spring.SetRestLength(LENGTH)
    spring.SetSpringCoefficient(0.0)
    spring.SetDampingCoefficient(0.0)
    system.AddLink(spring)

    spring_shape = chrono.ChVisualShapeSpring(0.10, 120, 14)
    spring_shape.SetColor(color(0.12, 0.42, 0.85))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, spring, 0.10, 120, 14, color(0.12, 0.42, 0.85))

    items = {
        "ground": ground,
        "mass": mass,
        "spring": spring,
        "load_arrow": LoadArrowVisual(system),
        "friction": FrictionVisual(system),
        "time": 0.0,
        "q": 0.0,
        "v": 0.0,
        "load": user_load(0.0),
        "spring_force": 0.0,
        "friction_force": 0.0,
    }
    system._spring_mass_friction_items = items
    update_visuals(system)
    return system, items


def set_mass_state(items, time, displacement, velocity):
    mass = items["mass"]
    x = LENGTH + displacement
    mass.SetPos(chrono.ChVector3d(x, 0.0, 0.0))
    mass.SetPosDt(chrono.ChVector3d(velocity, 0.0, 0.0))
    mass.UpdateVisualModel()
    friction = FRICTION_NORMAL_FORCE * stribeck_function(velocity, 1.0, 0.1)
    items["time"] = time
    items["q"] = displacement
    items["v"] = velocity
    items["load"] = user_load(time)
    items["spring_force"] = STIFFNESS * displacement + friction
    items["friction_force"] = friction


def sync_state_to_system_time(system):
    items = system._spring_mass_friction_items
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
    items = system._spring_mass_friction_items
    mass_pos = items["mass"].GetPos()
    items["load_arrow"].update(mass_pos, items["load"])
    items["friction"].update(mass_pos, items["friction_force"])
    update_system_visuals(system)


def simulate(duration, step):
    system, items = build_system()
    time = 0.0
    displacement = 0.0
    velocity = 0.0
    while time < duration - 1e-12:
        set_mass_state(items, time, displacement, velocity)
        update_visuals(system)
        dt = min(step, duration - time)
        displacement, velocity = rk4_step(time, displacement, velocity, dt)
        system.DoStepDynamics(dt)
        time += dt
    set_mass_state(items, duration, displacement, velocity)
    update_visuals(system)
    return system, items


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: springMassFriction.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.85, -1.55, 0.95), chrono.ChVector3d(0.82, 0.0, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if system.GetChTime() >= next_log:
            print_state(system, items)
            next_log += 0.2


def print_state(system, items):
    print(
        f"t={system.GetChTime():6.3f}  "
        f"x={items['mass'].GetPos().x:+.9f}  vx={items['v']:+.9f}  "
        f"load={items['load']:+.6f}  spring_force={items['spring_force']:+.6f}  "
        f"friction={items['friction_force']:+.6f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: springMassFriction.py -> PyChrono mass-spring Stribeck friction replay")
    if args.no_vis:
        system, items = simulate(args.duration, args.step)
        print_state(system, items)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

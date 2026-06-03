import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces EXUDYN Examples/cartesianSpringDamperUserFunction.py:
# a mass point driven along X by a CartesianSpringDamper user function and a
# constant force.  The source motion is one-dimensional, so this port integrates
# the signed X equation with the source explicit midpoint step while PyChrono
# renders the ground, mass, guide rail, load arrow, and a native coil spring.

LENGTH = 0.5
MASS = 1.6
STIFFNESS = 4000.0
LOAD = 80.0
INITIAL_DISPLACEMENT = -0.08
INITIAL_VELOCITY = 1.0
END_TIME = 2.0
SOURCE_STEP = 1.0e-5
STEP = 1.0e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def sign(value):
    if value > 0.0:
        return 1.0
    if value < 0.0:
        return -1.0
    return 0.0


def spring_user_force(displacement, velocity):
    return 0.5 * displacement * displacement * STIFFNESS + sign(velocity) * 10.0


def acceleration(displacement, velocity):
    return (LOAD - spring_user_force(displacement, velocity)) / MASS


def midpoint_step(displacement, velocity, step):
    accel = acceleration(displacement, velocity)
    mid_displacement = displacement + 0.5 * step * velocity
    mid_velocity = velocity + 0.5 * step * accel
    displacement += step * mid_velocity
    velocity += step * acceleration(mid_displacement, mid_velocity)
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
        self.arrow = MutableSegment(system, "Cartesian CSD constant load arrow", color(0.94, 0.50, 0.06), 5)
        self.tip_a = MutableSegment(system, "Cartesian CSD load arrow tip a", color(0.94, 0.50, 0.06), 4)
        self.tip_b = MutableSegment(system, "Cartesian CSD load arrow tip b", color(0.94, 0.50, 0.06), 4)

    def update(self, mass_position):
        start = mass_position + chrono.ChVector3d(0.0, 0.18, 0.0)
        end = start + chrono.ChVector3d(0.28, 0.0, 0.0)
        self.arrow.update(start, end)
        self.tip_a.update(end, end + chrono.ChVector3d(-0.055, 0.030, 0.0))
        self.tip_b.update(end, end + chrono.ChVector3d(-0.055, -0.030, 0.0))


class NonlinearForceVisual:
    def __init__(self, system):
        self.bar = MutableSegment(system, "Cartesian CSD nonlinear force indicator", color(0.72, 0.10, 0.10), 5)

    def update(self, mass_position, force_value):
        width = max(-0.34, min(0.34, force_value / 100.0 * 0.26))
        start = mass_position + chrono.ChVector3d(0.0, -0.20, 0.0)
        end = start - chrono.ChVector3d(width, 0.0, 0.0)
        self.bar.update(start, end)


def make_ground(system):
    ground = chrono.ChBody()
    ground.SetName("Cartesian spring user-function ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    wall = chrono.ChVisualShapeBox(0.08, 0.56, 0.22)
    wall.SetColor(color(0.55, 0.56, 0.55))
    ground.AddVisualShape(wall, chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0)))

    rail = chrono.ChVisualShapeBox(1.12, 0.020, 0.020)
    rail.SetColor(color(0.42, 0.42, 0.42))
    ground.AddVisualShape(rail, chrono.ChFramed(chrono.ChVector3d(0.56, -0.15, 0.0)))

    offset_marker = chrono.ChVisualShapeBox(0.015, 0.16, 0.020)
    offset_marker.SetColor(color(0.10, 0.10, 0.10))
    ground.AddVisualShape(offset_marker, chrono.ChFramed(chrono.ChVector3d(LENGTH, -0.15, 0.0)))

    system.AddBody(ground)
    return ground


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))

    ground = make_ground(system)

    anchor = chrono.ChBodyEasySphere(0.035, 1000, True, False)
    anchor.SetName("Cartesian CSD anchor")
    anchor.SetFixed(True)
    anchor.SetPos(chrono.ChVector3d(0.0, 0.0, 0.0))
    anchor.EnableCollision(False)
    anchor.GetVisualShape(0).SetColor(color(0.10, 0.10, 0.10))
    system.AddBody(anchor)

    mass = chrono.ChBodyEasySphere(0.070, 1000, True, False)
    mass.SetName("Cartesian CSD mass point")
    mass.SetFixed(True)
    mass.EnableCollision(False)
    mass.SetPos(chrono.ChVector3d(LENGTH + INITIAL_DISPLACEMENT, 0.0, 0.0))
    mass.GetVisualShape(0).SetColor(color(0.14, 0.40, 0.88))
    system.AddBody(mass)

    spring = chrono.ChLinkTSDA()
    spring.SetName("CartesianSpringDamper user-function visual analogue")
    spring.Initialize(mass, ground, True, chrono.ChVector3d(0.0, 0.0, 0.0), chrono.ChVector3d(0.0, 0.0, 0.0))
    spring.SetRestLength(LENGTH)
    spring.SetSpringCoefficient(0.0)
    spring.SetDampingCoefficient(0.0)
    system.AddLink(spring)

    spring_shape = chrono.ChVisualShapeSpring(0.060, 120, 14)
    spring_shape.SetColor(color(0.85, 0.18, 0.12))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, spring, 0.060, 120, 14, color(0.85, 0.18, 0.12))

    items = {
        "ground": ground,
        "mass": mass,
        "spring": spring,
        "load_arrow": LoadArrowVisual(system),
        "nonlinear_force": NonlinearForceVisual(system),
        "time": 0.0,
        "q": INITIAL_DISPLACEMENT,
        "v": INITIAL_VELOCITY,
        "spring_force": spring_user_force(INITIAL_DISPLACEMENT, INITIAL_VELOCITY),
    }
    system._cartesian_csd_user_function_items = items
    update_visuals(system)
    return system, items


def set_mass_state(items, time, displacement, velocity):
    x = LENGTH + displacement
    items["mass"].SetPos(chrono.ChVector3d(x, 0.0, 0.0))
    items["mass"].SetPosDt(chrono.ChVector3d(velocity, 0.0, 0.0))
    items["mass"].UpdateVisualModel()
    items["time"] = time
    items["q"] = displacement
    items["v"] = velocity
    items["spring_force"] = spring_user_force(displacement, velocity)


def sync_state_to_system_time(system):
    items = system._cartesian_csd_user_function_items
    target_time = system.GetChTime()
    time = items["time"]
    displacement = items["q"]
    velocity = items["v"]
    while time < target_time - 1e-12:
        dt = min(SOURCE_STEP, target_time - time)
        displacement, velocity = midpoint_step(displacement, velocity, dt)
        time += dt
    set_mass_state(items, target_time, displacement, velocity)


def update_visuals(system):
    sync_state_to_system_time(system)
    items = system._cartesian_csd_user_function_items
    mass_position = items["mass"].GetPos()
    items["load_arrow"].update(mass_position)
    items["nonlinear_force"].update(mass_position, items["spring_force"])
    update_system_visuals(system)


def simulate(duration, step):
    system, items = build_system()
    time = 0.0
    displacement = INITIAL_DISPLACEMENT
    velocity = INITIAL_VELOCITY
    while time < duration - 1e-12:
        set_mass_state(items, time, displacement, velocity)
        update_visuals(system)
        dt = min(step, duration - time)
        local = 0.0
        while local < dt - 1e-12:
            sub = min(SOURCE_STEP, dt - local)
            displacement, velocity = midpoint_step(displacement, velocity, sub)
            local += sub
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
    vis.SetWindowTitle("EXUDYN port: cartesianSpringDamperUserFunction.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.58, -1.16, 0.72), chrono.ChVector3d(0.55, 0.0, 0.0))
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
            next_log += 0.4


def print_state(system, items):
    total_force = LOAD - items["spring_force"]
    print(
        f"t={system.GetChTime():6.3f}  "
        f"pos=({items['mass'].GetPos().x:+.9f},+0.000000000,+0.000000000)  "
        f"vx={items['v']:+.9f}  spring_force={items['spring_force']:+.6f}  "
        f"load={LOAD:+.6f}  total_force={total_force:+.6f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: cartesianSpringDamperUserFunction.py -> PyChrono Cartesian user spring replay")
    if args.no_vis:
        system, items = simulate(args.duration, args.step)
        print_state(system, items)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

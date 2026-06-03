import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN Examples/slidercrankWithMassSpring.py:
# a torque-driven planar slider-crank with a second prismatic mass connected
# to the slider by a coordinate spring.

L1 = 0.3
L2 = 0.6
L3 = 0.2
M1 = 0.36
M2 = 0.15
M3 = 0.1
M4 = 0.7
TORQUE_Z = 1.0
SPRING_K = 1000.0
SPRING_D = 0.0
WIDTH = 0.05
STEP = 5e-4


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def update_visuals(system):
    update_system_visuals(system)


def add_pin(system, point, radius=0.018):
    pin = chrono.ChBodyEasySphere(radius, 1000, True, False)
    pin.SetFixed(True)
    pin.SetPos(point)
    pin.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.08))
    system.AddBody(pin)
    return pin


def make_link(name, length, mass, p0, p1, tint):
    midpoint = chrono.ChVector3d(0.5 * (p0.x + p1.x), 0.5 * (p0.y + p1.y), 0)
    theta = math.atan2(p1.y - p0.y, p1.x - p0.x)
    body = chrono.ChBodyEasyBox(length, WIDTH, WIDTH, 1000, True, False)
    body.SetName(name)
    body.SetMass(mass)
    body.SetInertiaXX(chrono.ChVector3d(mass * length * length / 12.0, mass * length * length / 12.0, mass * length * length / 12.0))
    body.SetPos(midpoint)
    body.SetRot(chrono.QuatFromAngleZ(theta))
    body.GetVisualShape(0).SetColor(tint)
    return body


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    crank_angle = 0.5 * math.pi
    point_a = chrono.ChVector3d(0, 0, 0)
    point_b = chrono.ChVector3d(L1 * math.cos(crank_angle), L1 * math.sin(crank_angle), 0)
    point_c = chrono.ChVector3d(math.sqrt(max(L2 * L2 - point_b.y * point_b.y, 0)), 0, 0)
    point_d = chrono.ChVector3d(point_c.x + L3, 0, 0)

    ground = chrono.ChBodyEasyBox(1.2, 0.035, 0.035, 1000, True, False)
    ground.SetName("slider-crank rail ground")
    ground.SetFixed(True)
    ground.SetPos(chrono.ChVector3d(0.42, -0.08, -0.04))
    ground.GetVisualShape(0).SetColor(color(0.48, 0.48, 0.48))
    system.AddBody(ground)

    crank = make_link("crank", L1, M1, point_a, point_b, color(0.12, 0.42, 0.85))
    rod = make_link("connecting rod", L2, M2, point_b, point_c, color(0.86, 0.18, 0.12))
    system.AddBody(crank)
    system.AddBody(rod)

    slider = chrono.ChBodyEasyBox(0.07, 0.07, 0.07, 1000, True, False)
    slider.SetName("primary slider mass")
    slider.SetMass(M3)
    slider.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
    slider.SetPos(point_c)
    slider.GetVisualShape(0).SetColor(color(0.95, 0.72, 0.08))
    system.AddBody(slider)

    spring_mass = chrono.ChBodyEasySphere(0.04, 1000, True, False)
    spring_mass.SetName("secondary spring mass")
    spring_mass.SetMass(M4)
    spring_mass.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
    spring_mass.SetPos(point_d)
    spring_mass.GetVisualShape(0).SetColor(color(0.30, 0.72, 0.30))
    system.AddBody(spring_mass)

    load_container = chrono.ChLoadContainer()
    system.Add(load_container)
    load_container.Add(chrono.ChLoadBodyTorque(crank, chrono.ChVector3d(0, 0, TORQUE_Z), False))

    ground_joint = chrono.ChLinkLockRevolute()
    ground_joint.Initialize(crank, ground, chrono.ChFramed(point_a))
    system.AddLink(ground_joint)

    crank_rod_joint = chrono.ChLinkLockRevolute()
    crank_rod_joint.Initialize(rod, crank, chrono.ChFramed(point_b))
    system.AddLink(crank_rod_joint)

    rod_slider_joint = chrono.ChLinkLockRevolute()
    rod_slider_joint.Initialize(slider, rod, chrono.ChFramed(point_c))
    system.AddLink(rod_slider_joint)

    slider_axis = chrono.ChFramed(point_c, chrono.Q_ROTATE_Z_TO_X)
    slider_joint = chrono.ChLinkLockPrismatic()
    slider_joint.Initialize(slider, ground, slider_axis)
    system.AddLink(slider_joint)

    spring_mass_axis = chrono.ChFramed(point_d, chrono.Q_ROTATE_Z_TO_X)
    spring_mass_joint = chrono.ChLinkLockPrismatic()
    spring_mass_joint.Initialize(spring_mass, ground, spring_mass_axis)
    system.AddLink(spring_mass_joint)

    spring = chrono.ChLinkTSDA()
    spring.Initialize(slider, spring_mass, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
    spring.SetRestLength(L3)
    spring.SetSpringCoefficient(SPRING_K)
    spring.SetDampingCoefficient(SPRING_D)
    system.AddLink(spring)
    spring_shape = chrono.ChVisualShapeSpring(0.025, 90, 12)
    spring_shape.SetColor(color(0.85, 0.18, 0.12))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, spring, 0.025, 90, 12, color(0.85, 0.18, 0.12))

    rail = chrono.ChBodyEasyBox(0.78, 0.012, 0.012, 1000, True, False)
    rail.SetFixed(True)
    rail.SetPos(chrono.ChVector3d(point_c.x + 0.18, -0.055, 0))
    rail.GetVisualShape(0).SetColor(color(0.25, 0.25, 0.25))
    system.AddBody(rail)

    for point in (point_a, point_b, point_c, point_d):
        add_pin(system, point)

    return system, {
        "crank": crank,
        "rod": rod,
        "slider": slider,
        "spring_mass": spring_mass,
        "spring": spring,
    }


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, items


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: slidercrankWithMassSpring.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.45, 0.65, 1.55), chrono.ChVector3d(0.35, 0.05, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        update_visuals(system)
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, items)
            next_log += 0.1


def print_state(system, items):
    slider = items["slider"]
    spring_mass = items["spring_mass"]
    spring = items["spring"]
    print(
        f"t={system.GetChTime():6.3f}  "
        f"slider_x={slider.GetPos().x:+.6f}  "
        f"mass_x={spring_mass.GetPos().x:+.6f}  "
        f"spring_L={spring.GetLength():+.6f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: slidercrankWithMassSpring.py -> PyChrono slider-crank with coil spring")
    if args.no_vis:
        system, items = simulate(args.duration, args.step)
        print_state(system, items)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

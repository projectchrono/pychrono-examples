import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN Examples/basicTutorial2024.py:
# a 12 kg mass point at [2,0,0] connected to ground by a spring-damper and
# loaded by gravity in negative y.

INITIAL_LENGTH = 2.0
MASS = 12.0
STIFFNESS = 500.0
DAMPING = 10.0
GRAVITY = 9.81
STEP = 2e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def update_visuals(system):
    update_system_visuals(system)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -GRAVITY, 0))

    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)

    background = chrono.ChBodyEasyBox(5.2, 0.025, 0.025, 1000, True, False)
    background.SetFixed(True)
    background.SetPos(chrono.ChVector3d(1.4, -0.28, 0))
    background.GetVisualShape(0).SetColor(color(0.78, 0.78, 0.78))
    system.AddBody(background)

    anchor = chrono.ChBodyEasySphere(0.06, 1000, True, False)
    anchor.SetFixed(True)
    anchor.SetPos(chrono.ChVector3d(0, 0, 0))
    anchor.GetVisualShape(0).SetColor(color(0.10, 0.10, 0.10))
    system.AddBody(anchor)

    mass = chrono.ChBodyEasySphere(0.20, 1000, True, False)
    mass.SetMass(MASS)
    mass.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
    mass.SetPos(chrono.ChVector3d(INITIAL_LENGTH, 0, 0))
    mass.GetVisualShape(0).SetColor(color(0.90, 0.12, 0.08))
    system.AddBody(mass)

    spring = chrono.ChLinkTSDA()
    spring.Initialize(mass, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
    spring.SetRestLength(INITIAL_LENGTH)
    spring.SetSpringCoefficient(STIFFNESS)
    spring.SetDampingCoefficient(DAMPING)
    system.AddLink(spring)
    spring_shape = chrono.ChVisualShapeSpring(0.10, 100, 14)
    spring_shape.SetColor(color(0.85, 0.18, 0.12))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, spring, 0.10, 100, 14, color(0.85, 0.18, 0.12))

    return system, mass, spring


def simulate(duration, step):
    system, mass, spring = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, mass, spring


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, mass, spring = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: basicTutorial2024.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.2, 0.9, 4.5), chrono.ChVector3d(1.0, -0.8, 0))
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
            print_state(system, mass, spring)
            next_log += 1.0


def print_state(system, mass, spring):
    pos = mass.GetPos()
    length = math.sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z)
    print(
        f"t={system.GetChTime():6.3f}  "
        f"pos=({pos.x:+.5f}, {pos.y:+.5f}, {pos.z:+.5f})  "
        f"length={length:.6f}  spring_force={spring.GetForce():+.3f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: basicTutorial2024.py -> PyChrono mass with TSDA and gravity")
    if args.no_vis:
        system, mass, spring = simulate(args.duration, args.step)
        print_state(system, mass, spring)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

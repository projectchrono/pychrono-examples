import argparse
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN
# TestModels/MiniExamples/ObjectConnectorSpringDamper.py:
# a point mass starts at length 1.05 m on a 1 m spring-damper.

REST_LENGTH = 1.0
INITIAL_LENGTH = 1.05
STIFFNESS = 100.0
DAMPING = 1.0
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def update_visuals(sys):
    update_system_visuals(sys)


def build_system():
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    ground = chrono.ChBody()
    ground.SetFixed(True)
    sys.AddBody(ground)

    mass = chrono.ChBodyEasySphere(0.055, 1000, True, False)
    mass.SetMass(1.0)
    mass.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    mass.SetPos(chrono.ChVector3d(INITIAL_LENGTH, 0, 0))
    mass.GetVisualShape(0).SetColor(color(0.12, 0.38, 0.88))
    sys.AddBody(mass)

    spring = chrono.ChLinkTSDA()
    spring.Initialize(mass, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
    spring.SetRestLength(REST_LENGTH)
    spring.SetSpringCoefficient(STIFFNESS)
    spring.SetDampingCoefficient(DAMPING)
    sys.AddLink(spring)
    spring_shape = chrono.ChVisualShapeSpring(0.045, 80, 12)
    spring_shape.SetColor(color(0.85, 0.18, 0.12))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(sys, spring, 0.045, 80, 12, color(0.85, 0.18, 0.12))

    anchor = chrono.ChBodyEasySphere(0.035, 1000, True, False)
    anchor.SetFixed(True)
    anchor.GetVisualShape(0).SetColor(color(0.1, 0.1, 0.1))
    sys.AddBody(anchor)

    return sys, mass, spring


def simulate(duration, step):
    sys, mass, spring = build_system()
    while sys.GetChTime() < duration:
        sys.DoStepDynamics(step)
    return sys, mass, spring


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    sys, mass, spring = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ObjectConnectorSpringDamper.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.55, 0.45, 1.8), chrono.ChVector3d(0.55, 0, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and sys.GetChTime() < duration:
        t = sys.GetChTime()
        vis.BeginScene()
        update_visuals(sys)
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(step)
        if t >= next_log:
            print_state(sys, mass, spring)
            next_log += 0.25


def print_state(sys, mass, spring):
    print(f"t={sys.GetChTime():6.3f}  x={mass.GetPos().x:+.8f}  length={spring.GetLength():.8f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ObjectConnectorSpringDamper.py -> PyChrono TSDA spring")
    if args.no_vis:
        sys, mass, spring = simulate(args.duration, args.step)
        print_state(sys, mass, spring)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

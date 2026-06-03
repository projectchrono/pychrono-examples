import argparse
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_segment_visual, update_system_visuals


# Reproduces the intent of EXUDYN
# TestModels/MiniExamples/ObjectConnectorDistance.py:
# a 50 kg point mass constrained to stay 1 m from ground under gravity.

LENGTH = 1.0
MASS = 50.0
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def update_visuals(sys):
    update_system_visuals(sys)


def build_system():
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    ground = chrono.ChBody()
    ground.SetFixed(True)
    sys.AddBody(ground)

    mass = chrono.ChBodyEasySphere(0.07, 1000, True, False)
    mass.SetMass(MASS)
    mass.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, 0.02))
    mass.SetPos(chrono.ChVector3d(LENGTH, 0, 0))
    mass.GetVisualShape(0).SetColor(color(0.95, 0.20, 0.12))
    sys.AddBody(mass)

    distance = chrono.ChLinkDistance()
    distance.Initialize(
        mass,
        ground,
        True,
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        False,
        LENGTH,
    )
    sys.AddLink(distance)
    attach_segment_visual(sys, distance, color(0.10, 0.10, 0.10), 3)

    anchor = chrono.ChBodyEasySphere(0.045, 1000, True, False)
    anchor.SetFixed(True)
    anchor.SetPos(chrono.ChVector3d(0, 0, 0))
    anchor.GetVisualShape(0).SetColor(color(0.12, 0.12, 0.12))
    sys.AddBody(anchor)

    return sys, mass, distance


def simulate(duration, step):
    sys, mass, distance = build_system()
    while sys.GetChTime() < duration:
        sys.DoStepDynamics(step)
    return sys, mass, distance


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    sys, mass, distance = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ObjectConnectorDistance.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.45, 0.35, 2.45), chrono.ChVector3d(0.45, -0.25, 0))
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
            print_state(sys, mass, distance)
            next_log += 0.5


def print_state(sys, mass, distance):
    pos = mass.GetPos()
    print(
        f"t={sys.GetChTime():6.3f}  "
        f"pos=({pos.x:+.5f}, {pos.y:+.5f})  "
        f"distance={distance.GetCurrentDistance():.8f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ObjectConnectorDistance.py -> PyChrono distance constraint")
    if args.no_vis:
        sys, mass, distance = simulate(args.duration, args.step)
        print_state(sys, mass, distance)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

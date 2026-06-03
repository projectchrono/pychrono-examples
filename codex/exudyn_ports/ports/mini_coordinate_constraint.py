import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN
# TestModels/MiniExamples/ObjectConnectorCoordinate.py:
# a coordinate constraint drives the x-coordinate of a point mass with a
# time-dependent offset. Chrono represents this directly as a linear position
# motor along x.

OFFSET = 0.1
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def offset_at(t):
    return 0.5 * (1.0 - math.cos(0.5 * math.pi * t)) * OFFSET


def make_offset_function():
    func = chrono.ChFunctionInterp()
    for i in range(101):
        t = i / 100.0
        func.AddPoint(t, offset_at(t))
    func.SetExtrapolate(True)
    return func


def build_system():
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    ground = chrono.ChBody()
    ground.SetFixed(True)
    sys.AddBody(ground)

    mass = chrono.ChBodyEasySphere(0.055, 1000, True, False)
    mass.SetMass(5.0)
    mass.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    mass.SetPos(chrono.ChVector3d(0, 0, 0))
    mass.GetVisualShape(0).SetColor(color(0.12, 0.38, 0.88))
    sys.AddBody(mass)

    motor = chrono.ChLinkMotorLinearPosition()
    motor.Initialize(mass, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.Q_ROTATE_Z_TO_X))
    motor.SetMotionFunction(make_offset_function())
    sys.AddLink(motor)

    rail = chrono.ChBodyEasyBox(0.18, 0.012, 0.012, 1000, True, False)
    rail.SetFixed(True)
    rail.SetPos(chrono.ChVector3d(0.05, -0.11, 0))
    rail.GetVisualShape(0).SetColor(color(0.45, 0.45, 0.45))
    sys.AddBody(rail)

    return sys, mass, motor


def simulate(duration, step):
    sys, mass, motor = build_system()
    while sys.GetChTime() < duration:
        sys.DoStepDynamics(step)
    return sys, mass, motor


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    sys, mass, motor = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ObjectConnectorCoordinate.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.08, 0.25, 0.55), chrono.ChVector3d(0.04, 0, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and sys.GetChTime() < duration:
        t = sys.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(step)
        if t >= next_log:
            print_state(sys, mass)
            next_log += 0.25


def print_state(sys, mass):
    coordinate = mass.GetPos().x
    print(
        f"t={sys.GetChTime():6.3f}  coordinate={coordinate:+.8f}  "
        f"offset={offset_at(min(sys.GetChTime(), 1.0)):+.8f}  "
        f"error={coordinate - offset_at(min(sys.GetChTime(), 1.0)):+.3e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ObjectConnectorCoordinate.py -> PyChrono linear position motor")
    if args.no_vis:
        sys, mass, motor = simulate(args.duration, args.step)
        print_state(sys, mass)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

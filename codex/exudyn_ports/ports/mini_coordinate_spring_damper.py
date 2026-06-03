import argparse
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN
# TestModels/MiniExamples/ObjectConnectorCoordinateSpringDamper.py:
# a 1D coordinate spring-damper with a nonlinear force user function and a
# unit coordinate load.

MASS = 5.0
STIFFNESS = 5000.0
DAMPING = 80.0
LOAD = 1.0
STEP = 1e-3


class NonlinearCoordinateSpring(chrono.ForceFunctor):
    def __init__(self):
        super().__init__()

    def evaluate(self, time, rest_length, length, vel, link):
        u = length - rest_length
        return -(0.1 * STIFFNESS * u + STIFFNESS * u**3 + DAMPING * vel)


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
    mass.SetMass(MASS)
    mass.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    mass.SetPos(chrono.ChVector3d(0, 0, 0))
    mass.GetVisualShape(0).SetColor(color(0.12, 0.38, 0.88))
    sys.AddBody(mass)

    slider = chrono.ChLinkLockPrismatic()
    slider.Initialize(mass, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.Q_ROTATE_Z_TO_X))
    sys.AddLink(slider)

    spring = chrono.ChLinkTSDA()
    spring.Initialize(mass, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
    spring.SetRestLength(0)
    spring._exudyn_port_force_functor = NonlinearCoordinateSpring()
    spring.RegisterForceFunctor(spring._exudyn_port_force_functor)
    sys.AddLink(spring)
    spring_shape = chrono.ChVisualShapeSpring(0.035, 80, 10)
    spring_shape.SetColor(color(0.85, 0.18, 0.12))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(sys, spring, 0.035, 80, 10, color(0.85, 0.18, 0.12))

    force = chrono.ChForce()
    force.SetF_x(chrono.ChFunctionConst(LOAD))
    mass.AddForce(force)

    rail = chrono.ChBodyEasyBox(0.22, 0.012, 0.012, 1000, True, False)
    rail.SetFixed(True)
    rail.SetPos(chrono.ChVector3d(0.07, -0.13, 0))
    rail.GetVisualShape(0).SetColor(color(0.45, 0.45, 0.45))
    sys.AddBody(rail)

    visual_spring = chrono.ChLinkTSDA()
    visual_spring.Initialize(
        mass,
        ground,
        True,
        chrono.ChVector3d(0, 0.13, 0),
        chrono.ChVector3d(0, 0.38, 0),
    )
    visual_spring.SetSpringCoefficient(0)
    visual_spring.SetDampingCoefficient(0)
    sys.AddLink(visual_spring)
    visual_shape = chrono.ChVisualShapeSpring(0.045, 90, 10)
    visual_shape.SetColor(color(0.85, 0.18, 0.12))
    visual_spring.AddVisualShape(visual_shape)
    attach_spring_visual(sys, visual_spring, 0.045, 90, 10, color(0.85, 0.18, 0.12))

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
    vis.SetWindowTitle("EXUDYN port: ObjectConnectorCoordinateSpringDamper.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.10, 0.28, 0.62), chrono.ChVector3d(0.05, 0, 0))
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
    x = mass.GetPos().x
    print(f"t={sys.GetChTime():6.3f}  x={x:+.8f}  spring_force={spring.GetForce():+.6f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print(
        "EXUDYN port: ObjectConnectorCoordinateSpringDamper.py -> "
        "PyChrono nonlinear coordinate spring"
    )
    if args.no_vis:
        sys, mass, spring = simulate(args.duration, args.step)
        print_state(sys, mass, spring)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

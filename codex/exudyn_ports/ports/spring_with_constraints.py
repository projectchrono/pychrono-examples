import argparse
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN Examples/SpringWithConstraints.py:
# a point connected to ground by a spring, with the transverse coordinates and
# the ground-side point constrained, then loaded in x.

LENGTH = 1.0
MASS = 1.0
STIFFNESS = 1000.0
DAMPING = 80.0
FORCE = 10.0
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
    mass.SetMass(MASS)
    mass.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    mass.SetPos(chrono.ChVector3d(LENGTH, 0, 0))
    mass.GetVisualShape(0).SetColor(color(0.9, 0.55, 0.05))
    sys.AddBody(mass)

    slider = chrono.ChLinkLockPrismatic()
    slider.Initialize(mass, ground, chrono.ChFramed(chrono.ChVector3d(LENGTH, 0, 0), chrono.Q_ROTATE_Z_TO_X))
    sys.AddLink(slider)

    spring = chrono.ChLinkTSDA()
    spring.Initialize(mass, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
    spring.SetRestLength(LENGTH)
    spring.SetSpringCoefficient(STIFFNESS)
    spring.SetDampingCoefficient(DAMPING)
    sys.AddLink(spring)
    spring_shape = chrono.ChVisualShapeSpring(0.05, 80, 15)
    spring_shape.SetColor(color(0.1, 0.4, 0.85))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(sys, spring, 0.05, 80, 15, color(0.1, 0.4, 0.85))

    anchor = chrono.ChBodyEasySphere(0.04, 1000, True, False)
    anchor.SetFixed(True)
    anchor.SetPos(chrono.ChVector3d(0, 0, 0))
    anchor.GetVisualShape(0).SetColor(color(0.15, 0.15, 0.15))
    sys.AddBody(anchor)

    rail = chrono.ChBodyEasyBox(1.3, 0.015, 0.015, 1000, True, False)
    rail.SetFixed(True)
    rail.SetPos(chrono.ChVector3d(0.65, -0.14, 0))
    rail.GetVisualShape(0).SetColor(color(0.45, 0.45, 0.45))
    sys.AddBody(rail)

    force = chrono.ChForce()
    force.SetF_x(chrono.ChFunctionConst(FORCE))
    mass.AddForce(force)

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
    vis.SetWindowTitle("EXUDYN port: SpringWithConstraints.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.8, 0.55, 2.2), chrono.ChVector3d(0.55, 0, 0))
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
    displacement = mass.GetPos().x - LENGTH
    static_reference = FORCE / STIFFNESS
    print(
        f"t={sys.GetChTime():6.3f}  u={displacement:+.6f}  "
        f"static_ref={static_reference:+.6f}  "
        f"spring_force={spring.GetForce():+.3f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: SpringWithConstraints.py -> PyChrono constrained spring")
    if args.no_vis:
        sys, mass, spring = simulate(args.duration, args.step)
        print_state(sys, mass, spring)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

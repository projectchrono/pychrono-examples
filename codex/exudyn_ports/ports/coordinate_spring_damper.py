import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN Examples/coordinateSpringDamper.py:
# a 1D coordinate spring-damper oscillator, with transverse coordinates locked
# by the prismatic joint and an analytical reference solution.

LENGTH = 0.5
MASS = 1.6
STIFFNESS = 4000.0
DAMPING_RATIO = 0.05
DAMPING = DAMPING_RATIO * 2.0 * math.sqrt(MASS * STIFFNESS)
INITIAL_DISPLACEMENT = -0.08
INITIAL_VELOCITY = 1.0
FORCE = 80.0
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def update_visuals(sys):
    update_system_visuals(sys)


def exact_displacement(t):
    omega0 = math.sqrt(STIFFNESS / MASS)
    damping_ratio = DAMPING / (2.0 * math.sqrt(STIFFNESS * MASS))
    omega = omega0 * math.sqrt(1.0 - damping_ratio**2)
    static_displacement = FORCE / STIFFNESS
    c1 = INITIAL_DISPLACEMENT - static_displacement
    c2 = (INITIAL_VELOCITY + omega0 * damping_ratio * c1) / omega
    return (
        math.exp(-omega0 * damping_ratio * t)
        * (c1 * math.cos(omega * t) + c2 * math.sin(omega * t))
        + static_displacement
    )


def build_system():
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    ground = chrono.ChBody()
    ground.SetFixed(True)
    sys.AddBody(ground)

    body = chrono.ChBodyEasyBox(0.11, 0.11, 0.11, 1000, True, False)
    body.SetMass(MASS)
    body.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    body.SetPos(chrono.ChVector3d(LENGTH + INITIAL_DISPLACEMENT, 0, 0))
    body.SetPosDt(chrono.ChVector3d(INITIAL_VELOCITY, 0, 0))
    body.GetVisualShape(0).SetColor(color(0.9, 0.55, 0.05))
    sys.AddBody(body)

    slider = chrono.ChLinkLockPrismatic()
    slider.Initialize(body, ground, chrono.ChFramed(chrono.ChVector3d(LENGTH, 0, 0), chrono.Q_ROTATE_Z_TO_X))
    sys.AddLink(slider)

    spring = chrono.ChLinkTSDA()
    spring.Initialize(body, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
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

    rail = chrono.ChBodyEasyBox(1.2, 0.015, 0.015, 1000, True, False)
    rail.SetFixed(True)
    rail.SetPos(chrono.ChVector3d(0.55, -0.14, 0))
    rail.GetVisualShape(0).SetColor(color(0.45, 0.45, 0.45))
    sys.AddBody(rail)

    force = chrono.ChForce()
    force.SetF_x(chrono.ChFunctionConst(FORCE))
    body.AddForce(force)

    return sys, body, spring


def simulate(duration, step):
    sys, body, spring = build_system()
    while sys.GetChTime() < duration:
        sys.DoStepDynamics(step)
    return sys, body, spring


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    sys, body, spring = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: coordinateSpringDamper.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.5, 0.6, 1.8), chrono.ChVector3d(0.5, 0, 0))
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
            print_state(sys, body, spring)
            next_log += 0.25


def print_state(sys, body, spring):
    displacement = body.GetPos().x - LENGTH
    reference = exact_displacement(sys.GetChTime())
    print(
        f"t={sys.GetChTime():6.3f}  u={displacement:+.8f}  "
        f"exact={reference:+.8f}  error={displacement - reference:+.3e}  "
        f"spring_force={spring.GetForce():+.3f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: coordinateSpringDamper.py -> PyChrono 1D spring-damper oscillator")
    if args.no_vis:
        sys, body, spring = simulate(args.duration, args.step)
        print_state(sys, body, spring)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

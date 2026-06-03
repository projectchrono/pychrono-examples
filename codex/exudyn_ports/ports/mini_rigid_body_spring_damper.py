import argparse
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN
# TestModels/MiniExamples/ObjectConnectorRigidBodySpringDamper.py:
# a rigid body with translational bushing stiffness in x/y/z and damping in y,
# launched with a large initial y-velocity.

STIFFNESS = 500.0
DAMPING_Y = STIFFNESS * 0.01
STEP = 1e-4


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def diagonal_matrix(values):
    matrix = chrono.ChMatrix66d()
    matrix.SetZero()
    for i, value in enumerate(values):
        matrix.SetItem(i, i, value)
    return matrix


def update_visuals(sys):
    update_system_visuals(sys)


def build_system():
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    ground = chrono.ChBody()
    ground.SetFixed(True)
    sys.AddBody(ground)

    body = chrono.ChBodyEasyBox(0.12, 0.12, 0.12, 1000, True, False)
    body.SetMass(1.0)
    body.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
    body.SetPosDt(chrono.ChVector3d(0, 1000, 0))
    body.GetVisualShape(0).SetColor(color(0.12, 0.38, 0.88))
    sys.AddBody(body)

    k = diagonal_matrix([STIFFNESS, STIFFNESS, STIFFNESS, 0, 0, 0])
    r = diagonal_matrix([0, DAMPING_Y, 0, 0, 0, 0])
    bushing = chrono.ChLinkBushing()
    bushing.Initialize(ground, body, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)), k, r)
    sys.AddLink(bushing)

    visual_spring = chrono.ChLinkTSDA()
    visual_spring.Initialize(
        body,
        ground,
        True,
        chrono.ChVector3d(0.12, 0, 0),
        chrono.ChVector3d(0.12, 0, 0),
    )
    visual_spring.SetSpringCoefficient(0)
    visual_spring.SetDampingCoefficient(0)
    sys.AddLink(visual_spring)
    spring_shape = chrono.ChVisualShapeSpring(0.16, 140, 18)
    spring_shape.SetColor(color(0.85, 0.18, 0.12))
    visual_spring.AddVisualShape(spring_shape)
    attach_spring_visual(sys, visual_spring, 0.16, 140, 18, color(0.85, 0.18, 0.12))

    anchor = chrono.ChBodyEasySphere(0.035, 1000, True, False)
    anchor.SetFixed(True)
    anchor.GetVisualShape(0).SetColor(color(0.1, 0.1, 0.1))
    sys.AddBody(anchor)

    return sys, body, bushing, visual_spring


def simulate(duration, step):
    sys, body, bushing, visual_spring = build_system()
    while sys.GetChTime() < duration:
        sys.DoStepDynamics(step)
    return sys, body, bushing, visual_spring


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    sys, body, bushing, visual_spring = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ObjectConnectorRigidBodySpringDamper.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.4, 1.3, 2.6), chrono.ChVector3d(0.1, 0.6, 0))
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
            print_state(sys, body)
            next_log += 0.02


def print_state(sys, body):
    print(f"t={sys.GetChTime():6.3f}  y={body.GetPos().y:+.6f}  vy={body.GetPosDt().y:+.6f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0.05)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ObjectConnectorRigidBodySpringDamper.py -> PyChrono bushing")
    if args.no_vis:
        sys, body, bushing, _ = simulate(args.duration, args.step)
        print_state(sys, body)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

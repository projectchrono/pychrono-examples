import argparse
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN
# TestModels/MiniExamples/ObjectConnectorLinearSpringDamper.py:
# a rigid body slides along x in a prismatic joint, resisted by a linear spring
# and loaded by a unit force.

STIFFNESS = 2000.0
DAMPING = STIFFNESS * 0.01
LOAD = 1.0
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

    body = chrono.ChBodyEasyBox(0.12, 0.08, 0.08, 1000, True, False)
    body.SetMass(1.0)
    body.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
    body.GetVisualShape(0).SetColor(color(0.12, 0.38, 0.88))
    sys.AddBody(body)

    prismatic = chrono.ChLinkLockPrismatic()
    prismatic.Initialize(body, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.Q_ROTATE_Z_TO_X))
    sys.AddLink(prismatic)

    spring = chrono.ChLinkTSDA()
    spring.Initialize(body, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
    spring.SetRestLength(0)
    spring.SetSpringCoefficient(STIFFNESS)
    spring.SetDampingCoefficient(DAMPING)
    sys.AddLink(spring)
    spring_shape = chrono.ChVisualShapeSpring(0.035, 80, 10)
    spring_shape.SetColor(color(0.85, 0.18, 0.12))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(sys, spring, 0.035, 80, 10, color(0.85, 0.18, 0.12))

    visual_spring = chrono.ChLinkTSDA()
    visual_spring.Initialize(
        body,
        ground,
        True,
        chrono.ChVector3d(0, 0.14, 0),
        chrono.ChVector3d(0, 0.40, 0),
    )
    visual_spring.SetSpringCoefficient(0)
    visual_spring.SetDampingCoefficient(0)
    sys.AddLink(visual_spring)
    visual_shape = chrono.ChVisualShapeSpring(0.045, 90, 10)
    visual_shape.SetColor(color(0.85, 0.18, 0.12))
    visual_spring.AddVisualShape(visual_shape)
    attach_spring_visual(sys, visual_spring, 0.045, 90, 10, color(0.85, 0.18, 0.12))

    force = chrono.ChForce()
    force.SetF_x(chrono.ChFunctionConst(LOAD))
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
    vis.SetWindowTitle("EXUDYN port: ObjectConnectorLinearSpringDamper.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.08, 0.25, 0.58), chrono.ChVector3d(0.04, 0, 0))
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
    print(f"t={sys.GetChTime():6.3f}  x={body.GetPos().x:+.8f}  spring_force={spring.GetForce():+.6f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ObjectConnectorLinearSpringDamper.py -> PyChrono prismatic TSDA")
    if args.no_vis:
        sys, body, spring = simulate(args.duration, args.step)
        print_state(sys, body, spring)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

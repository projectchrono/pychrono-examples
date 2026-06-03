import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN TestModels/connectorRigidBodySpringDamperTest.py:
# a rigid body launched with translational and angular velocity, connected to
# ground by a compliant rigid-body spring-damper. Chrono's bushing provides the
# native six-DOF compliance analogue.

MASS = 1.0
INERTIA = 1.0
STIFFNESS = 5000.0
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


def update_visuals(system):
    update_system_visuals(system)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)

    anchor = chrono.ChBodyEasySphere(0.035, 1000, True, False)
    anchor.SetFixed(True)
    anchor.GetVisualShape(0).SetColor(color(0.10, 0.10, 0.10))
    system.AddBody(anchor)

    body = chrono.ChBodyEasyBox(0.09, 0.09, 0.20, 1000, True, False)
    body.SetMass(MASS)
    body.SetInertiaXX(chrono.ChVector3d(INERTIA, INERTIA, INERTIA))
    body.SetPos(chrono.ChVector3d(0, 0, 0))
    body.SetPosDt(chrono.ChVector3d(0, 10, 0))
    body.SetAngVelLocal(chrono.ChVector3d(2 * math.pi * 4, 0, 0))
    body.GetVisualShape(0).SetColor(color(0.95, 0.35, 0.30))
    system.AddBody(body)

    k = diagonal_matrix([STIFFNESS, STIFFNESS, STIFFNESS, STIFFNESS, STIFFNESS, STIFFNESS])
    r = diagonal_matrix([0, DAMPING_Y, 0, STIFFNESS * 0.001, 0, 0])
    bushing = chrono.ChLinkBushing()
    bushing.Initialize(ground, body, chrono.ChFramed(chrono.ChVector3d(0, 1, 0)), k, r)
    system.AddLink(bushing)

    visual_spring = chrono.ChLinkTSDA()
    visual_spring.Initialize(
        body,
        ground,
        True,
        chrono.ChVector3d(0.12, 0, 0),
        chrono.ChVector3d(0.12, 1, 0),
    )
    visual_spring.SetSpringCoefficient(0)
    visual_spring.SetDampingCoefficient(0)
    system.AddLink(visual_spring)
    spring_shape = chrono.ChVisualShapeSpring(0.12, 120, 14)
    spring_shape.SetColor(color(0.85, 0.18, 0.12))
    visual_spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, visual_spring, 0.12, 120, 14, color(0.85, 0.18, 0.12))

    return system, body, bushing, visual_spring


def simulate(duration, step):
    system, body, bushing, visual_spring = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, body, bushing, visual_spring


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, body, bushing, visual_spring = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: connectorRigidBodySpringDamperTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.45, 1.4, 3.0), chrono.ChVector3d(0.0, 0.6, 0.0))
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
            print_state(system, body)
            next_log += 0.02


def print_state(system, body):
    marker = body.TransformPointLocalToParent(chrono.ChVector3d(0.1, 0.1, 0.1))
    print(
        f"t={system.GetChTime():6.3f}  "
        f"pos=({body.GetPos().x:+.6f}, {body.GetPos().y:+.6f}, {body.GetPos().z:+.6f})  "
        f"marker_sum={marker.x + marker.y:+.8f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0.1)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: connectorRigidBodySpringDamperTest.py -> PyChrono rigid bushing")
    if args.no_vis:
        system, body, bushing, visual_spring = simulate(args.duration, args.step)
        print_state(system, body)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

import argparse
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN Examples/rigid3Dexample.py:
# a chain of 3D rigid bodies connected by Cartesian spring-dampers without
# revolute/prismatic joints. Chrono bushings provide the physical Cartesian
# compliance; zero-force TSDA coils make every spring-damper visible.

COUNT = 20
S = 0.1
SX = 3.0 * S
BODY_LENGTH = 1.8 * SX
BODY_HEIGHT = 2.0 * S
CONNECTOR_Z = 0.1
MASS = 2.0
STIFFNESS = 2.0e5
DAMPING = 2.0e3
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


def make_body(index):
    body = chrono.ChBodyEasyBox(BODY_LENGTH, BODY_HEIGHT, BODY_HEIGHT, 1000, True, False)
    body.SetName(f"spring-chain body {index:02d}")
    body.SetMass(MASS)
    body.SetInertiaXX(chrono.ChVector3d(6.0, 1.0, 6.0))
    body.SetPos(chrono.ChVector3d(-SX + index * 2.0 * SX, 0, 0))
    body.GetVisualShape(0).SetColor(color(0.12, 0.46, 0.86))

    left_marker = chrono.ChVisualShapeSphere(0.045)
    left_marker.SetColor(color(0.18, 0.18, 0.18))
    body.AddVisualShape(left_marker, chrono.ChFramed(chrono.ChVector3d(-SX, 0, CONNECTOR_Z)))
    return body


def add_visual_spring(system, body, previous_body, current_anchor, previous_anchor, radius):
    spring = chrono.ChLinkTSDA()
    spring.Initialize(body, previous_body, True, current_anchor, previous_anchor)
    spring.SetSpringCoefficient(0)
    spring.SetDampingCoefficient(0)
    system.AddLink(spring)
    spring_shape = chrono.ChVisualShapeSpring(radius, 80, 10)
    spring_shape.SetColor(color(0.85, 0.18, 0.12))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, spring, radius, 80, 10, color(0.85, 0.18, 0.12))
    return spring


def build_system(count=COUNT):
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    ground = chrono.ChBodyEasyBox(2.0 * SX, 0.045, 0.045, 1000, True, False)
    ground.SetName("spring-chain fixed support")
    ground.SetFixed(True)
    ground.SetPos(chrono.ChVector3d(-2.0 * SX, -0.18, CONNECTOR_Z))
    ground.GetVisualShape(0).SetColor(color(0.40, 0.40, 0.40))
    system.AddBody(ground)

    k = diagonal_matrix([STIFFNESS, STIFFNESS, STIFFNESS, 0, 0, 0])
    r = diagonal_matrix([DAMPING, DAMPING, DAMPING, 0, 0, 0])

    bodies = []
    bushings = []
    visual_springs = []

    previous_body = ground
    ground_visual_anchor = chrono.ChVector3d(0, 0.36, 0)

    for i in range(count):
        body = make_body(i)
        system.AddBody(body)
        bodies.append(body)

        connector = chrono.ChVector3d(-2.0 * SX + i * 2.0 * SX, 0, CONNECTOR_Z)
        bushing = chrono.ChLinkBushing()
        bushing.Initialize(previous_body, body, chrono.ChFramed(connector), k, r)
        system.AddLink(bushing)
        bushings.append(bushing)

        if i == 0:
            visual_springs.append(
                add_visual_spring(
                    system,
                    body,
                    previous_body,
                    chrono.ChVector3d(0, 0.18, CONNECTOR_Z),
                    ground_visual_anchor,
                    0.035,
                )
            )
        else:
            visual_springs.append(
                add_visual_spring(
                    system,
                    body,
                    previous_body,
                    chrono.ChVector3d(0, 0.18, CONNECTOR_Z),
                    chrono.ChVector3d(0, 0.18, CONNECTOR_Z),
                    0.035,
                )
            )

        previous_body = body

    return system, bodies, bushings, visual_springs


def simulate(duration, step, count):
    system, bodies, bushings, visual_springs = build_system(count)
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, bodies, bushings, visual_springs


def run_visual(duration, step, count):
    import pychrono.irrlicht as chronoirr

    system, bodies, bushings, visual_springs = build_system(count)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: rigid3Dexample.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(2.8, 1.8, 2.2), chrono.ChVector3d(2.6, -0.3, 0.0))
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
            print_state(system, bodies)
            next_log += 0.2


def print_state(system, bodies):
    tip = bodies[-1].TransformPointLocalToParent(chrono.ChVector3d(SX, 0, CONNECTOR_Z))
    print(
        f"t={system.GetChTime():6.3f}  "
        f"bodies={len(bodies)}  "
        f"tip=({tip.x:+.5f}, {tip.y:+.5f}, {tip.z:+.5f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0.6)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--count", type=int, default=COUNT)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: rigid3Dexample.py -> PyChrono 3D rigid spring-damper chain")
    if args.no_vis:
        system, bodies, bushings, visual_springs = simulate(args.duration, args.step, args.count)
        print_state(system, bodies)
    else:
        run_visual(args.duration, args.step, args.count)


if __name__ == "__main__":
    main()

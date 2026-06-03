import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN Examples/SpringDamperMasspointSystem.py:
# a 6x4 point-mass grid with x, y, and both diagonal spring-damper connectors;
# the left and right columns are clamped and the free nodes sag under gravity.

NODES_X = 6
NODES_Y = 4
REST = 1.0
MASS = 0.5
STIFFNESS = 1000.0
DAMPING = 5.0
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def update_visuals(sys):
    update_system_visuals(sys)


def node_index(i, j):
    return i * NODES_X + j


def make_node(i, j):
    fixed = j == 0 or j == NODES_X - 1
    radius = 0.045 if fixed else 0.055
    body = chrono.ChBodyEasySphere(radius, 1000, True, False)
    body.SetName(f"node {i},{j}")
    body.SetMass(MASS)
    body.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    body.SetFixed(fixed)
    body.SetPos(chrono.ChVector3d(j * REST, i * REST, 0))
    body.GetVisualShape(0).SetColor(color(0.12, 0.12, 0.12) if fixed else color(0.1, 0.35, 0.9))
    return body


def add_spring(sys, bodies, a, b, rest_length):
    spring = chrono.ChLinkTSDA()
    spring.Initialize(bodies[a], bodies[b], True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
    spring.SetRestLength(rest_length)
    spring.SetSpringCoefficient(STIFFNESS)
    spring.SetDampingCoefficient(DAMPING)
    sys.AddLink(spring)
    spring_shape = chrono.ChVisualShapeSpring(0.035, 70, 10)
    spring_shape.SetColor(color(0.85, 0.18, 0.12))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(sys, spring, 0.035, 70, 10, color(0.85, 0.18, 0.12))
    return spring


def build_system():
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    bodies = []
    for i in range(NODES_Y):
        for j in range(NODES_X):
            body = make_node(i, j)
            sys.AddBody(body)
            bodies.append(body)

    springs = []
    for i in range(NODES_Y):
        for j in range(NODES_X - 1):
            springs.append(add_spring(sys, bodies, node_index(i, j), node_index(i, j + 1), REST))

    for i in range(NODES_Y - 1):
        for j in range(NODES_X):
            springs.append(add_spring(sys, bodies, node_index(i, j), node_index(i + 1, j), REST))

    diagonal = math.sqrt(2.0) * REST
    for i in range(NODES_Y - 1):
        for j in range(NODES_X - 1):
            springs.append(add_spring(sys, bodies, node_index(i, j), node_index(i + 1, j + 1), diagonal))
            springs.append(add_spring(sys, bodies, node_index(i, j + 1), node_index(i + 1, j), diagonal))

    return sys, bodies, springs


def simulate(duration, step):
    sys, bodies, springs = build_system()
    while sys.GetChTime() < duration:
        sys.DoStepDynamics(step)
    return sys, bodies, springs


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    sys, bodies, springs = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1120, 760)
    vis.SetWindowTitle("EXUDYN port: SpringDamperMasspointSystem.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(2.5, 2.0, 8.0), chrono.ChVector3d(2.5, 1.5, 0))
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
            print_state(sys, bodies, springs)
            next_log += 0.5


def print_state(sys, bodies, springs):
    center = bodies[node_index(NODES_Y // 2, NODES_X // 2)]
    print(
        f"t={sys.GetChTime():6.3f}  "
        f"center=({center.GetPos().x:+.4f}, {center.GetPos().y:+.4f}, {center.GetPos().z:+.4f})  "
        f"nodes={len(bodies)}  springs={len(springs)}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: SpringDamperMasspointSystem.py -> PyChrono mass-spring grid")
    if args.no_vis:
        sys, bodies, springs = simulate(args.duration, args.step)
        print_state(sys, bodies, springs)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

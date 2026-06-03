import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN Examples/bungeeJump.py:
# a jumper attached to a long flexible bungee. EXUDYN uses ANCF cable elements;
# this PyChrono port uses a mass-spring cable chain with visible spring coils
# between every cable node and a visible rigid-body jumper.

NODE_COUNT = 34
ROPE_LENGTH = 44.0
JUMPER_HEIGHT = 1.8
JUMPER_MASS = 90.0
ROPE_MASS = 18.0
SPRING_K = 1800.0
SPRING_D = 85.0
STEP = 2e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def update_visuals(system):
    update_system_visuals(system)


def add_bridge(system):
    bridge = chrono.ChBodyEasyBox(28.0, 3.0, 16.0, 1000, True, False)
    bridge.SetName("bridge deck")
    bridge.SetFixed(True)
    bridge.SetPos(chrono.ChVector3d(-2.0, 1.4, 0))
    bridge.GetVisualShape(0).SetColor(color(0.45, 0.45, 0.45))
    bridge.GetVisualShape(0).SetOpacity(0.65)
    system.AddBody(bridge)

    tower = chrono.ChBodyEasyBox(3.0, 80.0, 3.0, 1000, True, False)
    tower.SetName("bridge tower")
    tower.SetFixed(True)
    tower.SetPos(chrono.ChVector3d(-14.0, -42.0, 0))
    tower.GetVisualShape(0).SetColor(color(0.58, 0.58, 0.58))
    tower.GetVisualShape(0).SetOpacity(0.35)
    system.AddBody(tower)

    anchor = chrono.ChBodyEasySphere(0.35, 1000, True, False)
    anchor.SetName("bungee anchor")
    anchor.SetFixed(True)
    anchor.SetPos(chrono.ChVector3d(-2.0, 0.0, 0))
    anchor.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.08))
    system.AddBody(anchor)
    return anchor


def add_jumper_visuals(jumper):
    torso = chrono.ChVisualShapeBox(0.45, 0.95, 0.35)
    torso.SetColor(color(0.10, 0.28, 0.85))
    jumper.AddVisualShape(torso, chrono.ChFramed(chrono.ChVector3d(0, 0.15, 0)))
    legs = chrono.ChVisualShapeBox(0.30, 0.78, 0.28)
    legs.SetColor(color(0.14, 0.14, 0.14))
    jumper.AddVisualShape(legs, chrono.ChFramed(chrono.ChVector3d(0, -0.62, 0)))
    head = chrono.ChVisualShapeSphere(0.16)
    head.SetColor(color(0.95, 0.58, 0.20))
    jumper.AddVisualShape(head, chrono.ChFramed(chrono.ChVector3d(0, 0.82, 0)))


def make_jumper():
    jumper = chrono.ChBody()
    jumper.SetName("bungee jumper")
    jumper.SetMass(JUMPER_MASS)
    jumper.SetInertiaXX(chrono.ChVector3d(18, 4, 18))
    jumper.SetPos(chrono.ChVector3d(0.0, 0.5 * JUMPER_HEIGHT, 0))
    jumper.SetPosDt(chrono.ChVector3d(0.25, 0, 0))
    jumper.SetAngVelParent(chrono.ChVector3d(0, 0, -0.2 * math.pi))
    jumper.EnableCollision(False)
    add_jumper_visuals(jumper)
    return jumper


def make_node(index, position, fixed=False):
    node = chrono.ChBodyEasySphere(0.07 if fixed else 0.055, 1000, True, False)
    node.SetName(f"bungee cable node {index:02d}")
    node.SetMass(max(ROPE_MASS / NODE_COUNT, 0.1))
    node.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    node.SetPos(position)
    node.SetFixed(fixed)
    node.GetVisualShape(0).SetColor(color(0.85, 0.18, 0.12) if not fixed else color(0.08, 0.08, 0.08))
    return node


def distance(a, b):
    delta = a.GetPos() - b.GetPos()
    return delta.Length()


def add_rope_spring(system, body_a, body_b, rest_length):
    spring = chrono.ChLinkTSDA()
    spring.Initialize(body_a, body_b, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
    spring.SetRestLength(rest_length)
    spring.SetSpringCoefficient(SPRING_K)
    spring.SetDampingCoefficient(SPRING_D)
    system.AddLink(spring)
    spring_shape = chrono.ChVisualShapeSpring(0.14, 70, 8)
    spring_shape.SetColor(color(0.85, 0.18, 0.12))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, spring, 0.14, 70, 8, color(0.85, 0.18, 0.12))
    return spring


def build_system(node_count=NODE_COUNT):
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    anchor = add_bridge(system)
    jumper = make_jumper()
    system.AddBody(jumper)

    nodes = [anchor]
    start = chrono.ChVector3d(-2.0, 0.0, 0)
    end = chrono.ChVector3d(0.0, -0.5 * JUMPER_HEIGHT, 0)
    for i in range(1, node_count):
        s = i / node_count
        # A mild S curve gives the initially reeved bungee a visible shape.
        x = (1.0 - s) * start.x + s * end.x + 0.7 * math.sin(math.pi * s)
        y = (1.0 - s) * start.y + s * end.y - ROPE_LENGTH * 0.10 * math.sin(math.pi * s)
        node = make_node(i, chrono.ChVector3d(x, y, 0))
        system.AddBody(node)
        nodes.append(node)

    springs = []
    for a, b in zip(nodes[:-1], nodes[1:]):
        springs.append(add_rope_spring(system, a, b, 0.92 * distance(a, b)))
    springs.append(add_rope_spring(system, nodes[-1], jumper, 0.92 * distance(nodes[-1], jumper)))

    return system, jumper, nodes, springs


def simulate(duration, step, node_count):
    system, jumper, nodes, springs = build_system(node_count)
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, jumper, nodes, springs


def run_visual(duration, step, node_count):
    import pychrono.irrlicht as chronoirr

    system, jumper, nodes, springs = build_system(node_count)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: bungeeJump.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(9, 4, 28), chrono.ChVector3d(-1.0, -8.0, 0.0))
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
            print_state(system, jumper, springs)
            next_log += 0.5


def print_state(system, jumper, springs):
    pos = jumper.GetPos()
    vel = jumper.GetPosDt()
    max_stretch = max((spring.GetLength() - spring.GetRestLength()) for spring in springs)
    print(
        f"t={system.GetChTime():6.3f}  "
        f"jumper=({pos.x:+.3f}, {pos.y:+.3f}, {pos.z:+.3f})  "
        f"vy={vel.y:+.3f}  max_rope_stretch={max_stretch:+.3f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--nodes", type=int, default=NODE_COUNT)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: bungeeJump.py -> PyChrono mass-spring bungee jumper")
    if args.no_vis:
        system, jumper, nodes, springs = simulate(args.duration, args.step, args.nodes)
        print_state(system, jumper, springs)
    else:
        run_visual(args.duration, args.step, args.nodes)


if __name__ == "__main__":
    main()

import argparse
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_segment_visual, attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN Examples/3SpringsDistance.py:
# one mass point connected to three ground anchors, with one exact distance
# constraint and two spring-damper connectors, driven by a horizontal force.

LENGTH = 0.1
MASS = 2.5
STIFFNESS = 4000.0
DAMPING = 20.0
FORCE = 10.0
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def update_visuals(sys):
    update_system_visuals(sys)


def make_anchor(pos, tint):
    body = chrono.ChBodyEasySphere(0.012, 1000, True, False)
    body.SetFixed(True)
    body.SetPos(pos)
    body.GetVisualShape(0).SetColor(tint)
    return body


def build_system():
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    anchors = [
        make_anchor(chrono.ChVector3d(0, 0, 0), color(0.15, 0.15, 0.15)),
        make_anchor(chrono.ChVector3d(LENGTH, LENGTH, 0), color(0.15, 0.15, 0.15)),
        make_anchor(chrono.ChVector3d(0, LENGTH, LENGTH), color(0.15, 0.15, 0.15)),
    ]
    for anchor in anchors:
        sys.AddBody(anchor)

    mass = chrono.ChBodyEasySphere(0.018, 1000, True, False)
    mass.SetMass(MASS)
    mass.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
    mass.SetPos(chrono.ChVector3d(0, LENGTH, 0))
    mass.GetVisualShape(0).SetColor(color(0.1, 0.35, 0.9))
    sys.AddBody(mass)

    distance = chrono.ChLinkDistance()
    distance.Initialize(
        mass,
        anchors[0],
        True,
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        False,
        LENGTH,
    )
    sys.AddLink(distance)
    distance_shape = chrono.ChVisualShapeSegment()
    distance_shape.SetColor(color(0.95, 0.1, 0.1))
    distance_shape.SetThickness(3)
    distance.AddVisualShape(distance_shape)
    attach_segment_visual(sys, distance, color(0.95, 0.1, 0.1), 3)

    springs = []
    for anchor in anchors[1:]:
        spring = chrono.ChLinkTSDA()
        spring.Initialize(mass, anchor, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
        spring.SetRestLength(LENGTH)
        spring.SetSpringCoefficient(STIFFNESS)
        spring.SetDampingCoefficient(DAMPING)
        sys.AddLink(spring)
        spring_shape = chrono.ChVisualShapeSpring(0.012, 70, 10)
        spring_shape.SetColor(color(0.85, 0.18, 0.12))
        spring.AddVisualShape(spring_shape)
        attach_spring_visual(sys, spring, 0.012, 70, 10, color(0.85, 0.18, 0.12))
        springs.append(spring)

    force = chrono.ChForce()
    force.SetF_x(chrono.ChFunctionConst(FORCE))
    mass.AddForce(force)

    return sys, mass, distance, springs


def simulate(duration, step):
    sys, mass, distance, springs = build_system()
    while sys.GetChTime() < duration:
        sys.DoStepDynamics(step)
    return sys, mass, distance, springs


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    sys, mass, distance, springs = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: 3SpringsDistance.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.18, 0.25, 0.45), chrono.ChVector3d(0.04, 0.08, 0.02))
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
            print_state(sys, mass, distance, springs)
            next_log += 0.25


def print_state(sys, mass, distance, springs):
    print(
        f"t={sys.GetChTime():6.3f}  "
        f"pos=({mass.GetPos().x:+.5f}, {mass.GetPos().y:+.5f}, {mass.GetPos().z:+.5f})  "
        f"distance={distance.GetCurrentDistance():.6f}  "
        f"spring_forces=({springs[0].GetForce():+.2f}, {springs[1].GetForce():+.2f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: 3SpringsDistance.py -> PyChrono distance constraint plus springs")
    if args.no_vis:
        sys, mass, distance, springs = simulate(args.duration, args.step)
        print_state(sys, mass, distance, springs)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

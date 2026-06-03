import argparse
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_segment_visual, update_system_visuals


# Reproduces the intent of EXUDYN TestModels/coordinateVectorConstraint.py:
# a double pendulum made from point masses and user-defined coordinate-vector
# distance constraints. Chrono's native distance links provide the equivalent
# geometric constraints, with visible segment helpers for both constrained spans.

LENGTH = 0.8
MASS = 2.5
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_mass(name, position, tint):
    body = chrono.ChBodyEasySphere(0.055, 1000, True, False)
    body.SetName(name)
    body.SetMass(MASS)
    body.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    body.SetPos(position)
    body.GetVisualShape(0).SetColor(tint)
    return body


def add_distance(system, body_a, body_b, rest_length, name, tint):
    link = chrono.ChLinkDistance()
    link.SetName(name)
    link.Initialize(
        body_a,
        body_b,
        True,
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        False,
        rest_length,
    )
    system.AddLink(link)
    attach_segment_visual(system, link, tint, 4)
    return link


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    ground = chrono.ChBody()
    ground.SetName("coordinate-vector constraint ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)

    plate = chrono.ChBodyEasyBox(2.25, 2.25, 0.025, 1000, True, False)
    plate.SetName("visible coordinate-vector pendulum reference plate")
    plate.SetFixed(True)
    plate.SetPos(chrono.ChVector3d(0.65, -0.55, -0.12))
    plate.GetVisualShape(0).SetColor(color(0.80, 0.80, 0.76))
    plate.GetVisualShape(0).SetOpacity(0.28)
    system.AddBody(plate)

    anchor = chrono.ChBodyEasySphere(0.045, 1000, True, False)
    anchor.SetName("coordinate-vector pendulum fixed anchor")
    anchor.SetFixed(True)
    anchor.SetPos(chrono.ChVector3d(0, 0, 0))
    anchor.GetVisualShape(0).SetColor(color(0.06, 0.06, 0.06))
    system.AddBody(anchor)

    mass0 = make_mass("coordinate-vector pendulum mass 1", chrono.ChVector3d(LENGTH, 0, 0), color(0.10, 0.35, 0.90))
    mass1 = make_mass("coordinate-vector pendulum mass 2", chrono.ChVector3d(2.0 * LENGTH, 0, 0), color(0.90, 0.16, 0.12))
    system.AddBody(mass0)
    system.AddBody(mass1)

    link0 = add_distance(system, mass0, anchor, LENGTH, "coordinate-vector distance anchor to mass 1", color(0.05, 0.05, 0.05))
    link1 = add_distance(system, mass1, mass0, LENGTH, "coordinate-vector distance mass 1 to mass 2", color(0.05, 0.05, 0.05))
    update_visuals(system)
    return system, mass0, mass1, (link0, link1)


def update_visuals(system):
    update_system_visuals(system)


def simulate(duration, step):
    system, mass0, mass1, links = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, mass0, mass1, links


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, mass0, mass1, links = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: coordinateVectorConstraint.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.9, 0.6, 3.1), chrono.ChVector3d(0.65, -0.55, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, mass0, mass1)
            next_log += 0.25


def print_state(system, mass0, mass1):
    p0 = mass0.GetPos()
    p1 = mass1.GetPos()
    print(
        f"t={system.GetChTime():6.3f}  "
        f"p0=({p0.x:+.5f}, {p0.y:+.5f}, {p0.z:+.5f})  "
        f"p1=({p1.x:+.5f}, {p1.y:+.5f}, {p1.z:+.5f})  "
        f"sum_p0={p0.x + p0.y + p0.z:+.6f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: coordinateVectorConstraint.py -> PyChrono constrained double pendulum")
    if args.no_vis:
        system, mass0, mass1, links = simulate(args.duration, args.step)
        print_state(system, mass0, mass1)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_segment_visual, update_system_visuals


# Reproduces the intent of EXUDYN Examples/pendulumIftommBenchmark.py:
# IFToMM planar simple pendulum benchmark with a 1 kg point mass, 1 m distance
# constraint, and gravity in negative y.

LENGTH = 1.0
MASS = 1.0
GRAVITY = 9.81
STEP = 0.8e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def update_visuals(system):
    update_system_visuals(system)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -GRAVITY, 0))

    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)

    background = chrono.ChBodyEasyBox(2.4, 0.02, 0.02, 1000, True, False)
    background.SetFixed(True)
    background.SetPos(chrono.ChVector3d(0, -1.18, -0.02))
    background.GetVisualShape(0).SetColor(color(0.92, 0.92, 0.92))
    system.AddBody(background)

    anchor = chrono.ChBodyEasySphere(0.035, 1000, True, False)
    anchor.SetFixed(True)
    anchor.SetPos(chrono.ChVector3d(0, 0, 0))
    anchor.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.08))
    system.AddBody(anchor)

    mass = chrono.ChBodyEasySphere(0.05, 1000, True, False)
    mass.SetMass(MASS)
    mass.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    mass.SetPos(chrono.ChVector3d(-LENGTH, 0, 0))
    mass.GetVisualShape(0).SetColor(color(1.0, 0.20, 0.20))
    system.AddBody(mass)

    distance = chrono.ChLinkDistance()
    distance.Initialize(
        mass,
        ground,
        True,
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        False,
        LENGTH,
    )
    system.AddLink(distance)
    attach_segment_visual(system, distance, color(0.1, 0.1, 0.1), 3)

    return system, mass, distance


def simulate(duration, step):
    system, mass, distance = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, mass, distance


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, mass, distance = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: pendulumIftommBenchmark.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.0, 0.25, 2.65), chrono.ChVector3d(0, -0.45, 0))
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
            print_state(system, mass, distance)
            next_log += 1.0


def total_energy(mass):
    pos = mass.GetPos()
    vel = mass.GetPosDt()
    return 0.5 * MASS * (vel.x * vel.x + vel.y * vel.y + vel.z * vel.z) + MASS * GRAVITY * pos.y


def print_state(system, mass, distance):
    pos = mass.GetPos()
    vel = mass.GetPosDt()
    energy = total_energy(mass)
    print(
        f"t={system.GetChTime():7.4f}  "
        f"pos=({pos.x:+.6f}, {pos.y:+.6f})  "
        f"vel=({vel.x:+.6f}, {vel.y:+.6f})  "
        f"distance_error={distance.GetCurrentDistance() - LENGTH:+.3e}  "
        f"energy={energy:+.8f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: pendulumIftommBenchmark.py -> PyChrono distance-constrained benchmark")
    if args.no_vis:
        system, mass, distance = simulate(args.duration, args.step)
        print_state(system, mass, distance)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

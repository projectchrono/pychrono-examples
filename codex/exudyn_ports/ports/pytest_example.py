import argparse
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_segment_visual, update_system_visuals


# Reproduces EXUDYN main/pythonDev/pytest.py.
# The source is a standalone mathematical-pendulum example despite the file
# name. It creates 10 equal point-mass pendulums, each with its own shifted
# ground anchor, a DistanceConstraint of length 0.8 m, gravity, and initial
# transverse velocity from omegaInit=2 rad/s.

LENGTH = 0.8
MASS = 2.5
GRAVITY = 9.81
OMEGA_INIT = 2.0
COUNT = 10
STEP = 1.0e-3


def color(r, g, b):
    return chrono.ChColor(float(r), float(g), float(b))


def vec(x, y=None, z=None):
    if y is None:
        return chrono.ChVector3d(float(x[0]), float(x[1]), float(x[2]))
    return chrono.ChVector3d(float(x), float(y), float(z))


def update_visuals(system):
    update_system_visuals(system)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, -GRAVITY, 0.0))

    floor = chrono.ChBodyEasyBox(COUNT * LENGTH + 0.35, 0.035, 0.20, 1000.0, True, False)
    floor.SetName("pytest.py white background strip")
    floor.SetFixed(True)
    floor.SetPos(vec(0.5 * COUNT * LENGTH, -1.03 * LENGTH, -0.075))
    floor.GetVisualShape(0).SetColor(color(0.84, 0.86, 0.88))
    floor.GetVisualShape(0).SetOpacity(0.30)
    system.AddBody(floor)

    pendulums = []
    for i in range(COUNT):
        anchor_pos = vec(i * LENGTH, 0.0, 0.0)
        mass_pos = vec((i + 1) * LENGTH, 0.0, 0.0)

        ground = chrono.ChBody()
        ground.SetName(f"pytest.py pendulum {i + 1} shifted ground")
        ground.SetFixed(True)
        ground.SetPos(anchor_pos)
        ground.EnableCollision(False)
        system.AddBody(ground)

        anchor = chrono.ChBodyEasySphere(0.045, 1000.0, True, False)
        anchor.SetName(f"pytest.py pendulum {i + 1} anchor visualization")
        anchor.SetFixed(True)
        anchor.SetPos(anchor_pos)
        anchor.GetVisualShape(0).SetColor(color(0.05, 0.05, 0.055))
        system.AddBody(anchor)

        mass = chrono.ChBodyEasySphere(0.055, 1000.0, True, False)
        mass.SetName(f"pytest.py pendulum {i + 1} red mass point body")
        mass.SetMass(MASS)
        mass.SetInertiaXX(vec(0.01, 0.01, 0.01))
        mass.SetPos(mass_pos)
        mass.SetLinVel(vec(0.0, -LENGTH * OMEGA_INIT, 0.0))
        mass.GetVisualShape(0).SetColor(color(0.95, 0.20, 0.16))
        system.AddBody(mass)

        distance = chrono.ChLinkDistance()
        distance.SetName(f"pytest.py pendulum {i + 1} DistanceConstraint")
        distance.Initialize(mass, ground, True, vec(0, 0, 0), vec(0, 0, 0), False, LENGTH)
        system.AddLink(distance)
        attach_segment_visual(system, distance, color(0.06, 0.06, 0.065), 3)
        pendulums.append((mass, distance))

    system._pytest_items = {"pendulums": pendulums}
    update_visuals(system)
    return system, system._pytest_items


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration - 1.0e-14:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1100, 760)
    vis.SetWindowTitle("EXUDYN port: pytest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(vec(4.2, -2.35, 5.8), vec(4.0, -0.42, 0.0))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
    update_visuals(system)


def print_state(system):
    pendulums = system._pytest_items["pendulums"]
    y_values = [mass.GetPos().y for mass, _link in pendulums]
    distances = [link.GetCurrentDistance() for _mass, link in pendulums]
    print(
        f"t={system.GetChTime():.4f} source=pytest.py nPendulums={len(pendulums)} "
        f"L={LENGTH} mass={MASS} omegaInit={OMEGA_INIT} "
        f"yRange=({min(y_values):+.6f},{max(y_values):+.6f}) "
        f"distanceRange=({min(distances):.6f},{max(distances):.6f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: pytest.py -> PyChrono 10x distance-constrained mathematical pendulum")
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

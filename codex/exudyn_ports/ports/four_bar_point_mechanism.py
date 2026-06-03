import argparse
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_segment_visual, update_system_visuals


# Reproduces the intent of EXUDYN TestModels/fourBarMechanismTest.py:
# four point nodes with three exact distance constraints, two fixed ground
# points, initial angular-velocity-equivalent velocities, and gravity.

MASS = 3.0
GRAVITY = 9.81
OMEGA0 = 2.0
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def update_visuals(system):
    update_system_visuals(system)


def add_point(system, position, mass, tint, fixed=False, velocity=None):
    body = chrono.ChBodyEasySphere(0.07 if not fixed else 0.055, 1000, True, False)
    body.SetFixed(fixed)
    body.SetMass(mass if not fixed else 1.0)
    body.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    body.SetPos(position)
    if velocity is not None:
        body.SetPosDt(velocity)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def add_distance(system, body_a, body_b, length, tint):
    link = chrono.ChLinkDistance()
    link.Initialize(
        body_a,
        body_b,
        True,
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        False,
        length,
    )
    system.AddLink(link)
    attach_segment_visual(system, link, tint, 3)
    return link


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -GRAVITY, 0))

    ground0 = add_point(system, chrono.ChVector3d(0, 0, 0), 1.0, color(0.08, 0.08, 0.08), True)
    point1 = add_point(
        system,
        chrono.ChVector3d(0, 2, 0),
        MASS,
        color(0.92, 0.14, 0.10),
        False,
        chrono.ChVector3d(2 * OMEGA0, 0, 0),
    )
    point2 = add_point(
        system,
        chrono.ChVector3d(4, 5, 0),
        MASS,
        color(0.12, 0.42, 0.85),
        False,
        chrono.ChVector3d(5 * OMEGA0, 0, 0),
    )
    ground3 = add_point(system, chrono.ChVector3d(4, 0, 0), 1.0, color(0.08, 0.08, 0.08), True)

    links = [
        add_distance(system, ground0, point1, 2.0, color(0.10, 0.10, 0.10)),
        add_distance(system, point1, point2, 5.0, color(0.10, 0.10, 0.10)),
        add_distance(system, point2, ground3, 5.0, color(0.10, 0.10, 0.10)),
    ]

    return system, (ground0, point1, point2, ground3), links


def simulate(duration, step):
    system, points, links = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, points, links


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, points, links = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: fourBarMechanismTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(2.0, 2.0, 11.0), chrono.ChVector3d(2.0, 1.4, 0))
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
            print_state(system, points, links)
            next_log += 0.5


def print_state(system, points, links):
    point1 = points[1]
    distance_error = max(abs(link.GetCurrentDistance() - length) for link, length in zip(links, (2.0, 5.0, 5.0)))
    print(
        f"t={system.GetChTime():6.3f}  "
        f"point1_y={point1.GetPos().y:+.6f}  "
        f"displacement_y={point1.GetPos().y - 2.0:+.6f}  "
        f"max_distance_error={distance_error:+.3e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: fourBarMechanismTest.py -> PyChrono point-distance four-bar")
    if args.no_vis:
        system, points, links = simulate(args.duration, args.step)
        print_state(system, points, links)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

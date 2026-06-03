import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from reeving_visual_common import (
    PolylinePath,
    color,
    make_polyline_body,
    make_pulley,
    make_reeving_path,
    make_trace_body,
    make_visual_plate,
    update_tracers,
)


# Reproduces the intent of EXUDYN Examples/reevingSystem.py:
# an ANCF rope route around multiple revolute pulleys, with the first pulley
# velocity-controlled and the remaining pulleys following the rope motion. This
# port keeps the source reeving geometry and drive law, but uses explicit
# kinematic cable/pulley visuals for reliable PyChrono inspection.

STEP = 1e-3
PULLEY_WIDTH = 0.12
ROPE_Z = 0.085
ROPE_MARKER_RADIUS = 0.032
TRACE_COUNT = 44

CIRCLE_SPECS = [
    ((0.0, 0.0), 0.3, "L"),
    ((1.0, 0.0), 0.3, "R"),
    ((1.0, 1.0), 0.3, "R"),
    ((2.0, 1.0), 0.4, "R"),
    ((3.0, 1.0), 0.4, "R"),
    ((4.0, 1.0), 0.4, "L"),
    ((5.0, 1.0), 0.4, "R"),
    ((5.0, -1.0), 0.3, "L"),
    ((0.0, -1.0), 0.4, "L"),
]


def drive_omega(time):
    return max(10.0 * time, 5.0)


def drive_angle(time):
    if time <= 0.5:
        return 5.0 * time
    return 5.0 * time * time + 1.25


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    make_visual_plate(
        system,
        "reeving system visible reference plate",
        (2.5, 0.0, -0.09),
        (5.9, 2.9, 0.025),
        color(0.78, 0.78, 0.74),
        0.28,
    )

    pulleys = []
    for i, (center, radius, side) in enumerate(CIRCLE_SPECS):
        pulley = make_pulley(
            system,
            f"reeving pulley {i + 1:02d} side {side}",
            center,
            radius,
            PULLEY_WIDTH,
            color(0.12, 0.36, 0.82),
            color(0.95, 0.52, 0.08),
        )
        pulleys.append(pulley)

    points = make_reeving_path(CIRCLE_SPECS, z=ROPE_Z, points_per_arc=20)
    make_polyline_body(system, "reeving cable path visual", points, color(0.05, 0.06, 0.07), 8)
    make_polyline_body(system, "reeving cable highlight visual", points, color(0.98, 0.72, 0.12), 3)
    path = PolylinePath(points)

    tracers = []
    for i in range(TRACE_COUNT):
        tracers.append(make_trace_body(system, f"moving rope marker {i + 1:02d}", ROPE_MARKER_RADIUS, color(0.92, 0.18, 0.10)))

    system._reeving_visual_items = {"pulleys": pulleys, "path": path, "tracers": tracers}
    update_visuals(system)
    return system, pulleys, tracers


def update_visuals(system):
    items = getattr(system, "_reeving_visual_items", None)
    if items is None:
        return

    time = system.GetChTime()
    drive = drive_angle(time)
    rope_offset = CIRCLE_SPECS[0][1] * drive
    for pulley, (_, radius, side) in zip(items["pulleys"], CIRCLE_SPECS):
        sign = 1.0 if side == "L" else -1.0
        pulley.SetRot(chrono.QuatFromAngleZ(sign * rope_offset / radius))
        pulley.SetAngVelParent(chrono.ChVector3d(0, 0, sign * drive_omega(time) * CIRCLE_SPECS[0][1] / radius))

    update_tracers(items["path"], items["tracers"], rope_offset, z_lift=0.045)


def simulate(duration, step):
    system, pulleys, tracers = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, pulleys, tracers


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, pulleys, tracers = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: reevingSystem.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(2.6, -7.0, 5.2), chrono.ChVector3d(2.5, 0.0, 0.0))
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
            print_state(system, tracers)
            next_log += 0.5


def print_state(system, tracers):
    sample = tracers[0].GetPos()
    path = system._reeving_visual_items["path"]
    omega = drive_omega(system.GetChTime())
    print(
        f"t={system.GetChTime():6.3f}  "
        f"drive_omega={omega:+.4f}  "
        f"rope_speed={omega * CIRCLE_SPECS[0][1]:+.4f}  "
        f"sample_marker=({sample.x:+.4f}, {sample.y:+.4f}, {sample.z:+.4f})  "
        f"path_length={path.total_length:.4f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: reevingSystem.py -> PyChrono visual reeving system")
    if args.no_vis:
        system, pulleys, tracers = simulate(args.duration, args.step)
        print_state(system, tracers)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

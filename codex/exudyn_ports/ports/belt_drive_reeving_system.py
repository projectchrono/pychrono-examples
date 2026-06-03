import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from reeving_visual_common import (
    PolylinePath,
    color,
    make_belt_patch,
    make_polyline_body,
    make_pulley,
    make_two_pulley_belt_path,
    make_visual_plate,
    update_belt_patches,
)


# Reproduces the intent of EXUDYN Examples/beltDriveReevingSystem.py:
# a two-pulley belt drive with the source pulley radius, spacing, improved-belt
# run time, and velocity-control ramp. The ANCF belt/contact solver is replaced
# here by an explicit moving belt loop and rotating pulley visuals.

STEP = 1e-3
T_END = 2.45
T_ACC_START = 0.05
T_ACC_END = 0.60
OMEGA_FINAL = 12.0
RADIUS_PULLEY = 0.09995
POSITION_PULLEY_2_X = 0.1 * math.pi
PULLEY_WIDTH = 0.12
BELT_Z = 0.070
BELT_PATCH_COUNT = 32

CENTER_LEFT = (0.0, 0.0)
CENTER_RIGHT = (POSITION_PULLEY_2_X, 0.0)


def drive_omega(time):
    if time < T_ACC_START:
        return 0.0
    if time < T_ACC_END:
        return OMEGA_FINAL * (time - T_ACC_START) / (T_ACC_END - T_ACC_START)
    return OMEGA_FINAL


def drive_angle(time):
    if time <= T_ACC_START:
        return 0.0
    acceleration = OMEGA_FINAL / (T_ACC_END - T_ACC_START)
    if time < T_ACC_END:
        dt = time - T_ACC_START
        return 0.5 * acceleration * dt * dt
    ramp_angle = 0.5 * acceleration * (T_ACC_END - T_ACC_START) ** 2
    return ramp_angle + OMEGA_FINAL * (time - T_ACC_END)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    make_visual_plate(
        system,
        "belt drive visible reference plate",
        (0.5 * POSITION_PULLEY_2_X, 0.0, -0.09),
        (0.56, 0.36, 0.020),
        color(0.78, 0.78, 0.74),
        0.30,
    )

    left = make_pulley(
        system,
        "belt drive velocity-controlled pulley",
        CENTER_LEFT,
        RADIUS_PULLEY,
        PULLEY_WIDTH,
        color(0.12, 0.36, 0.82),
        color(0.95, 0.52, 0.08),
    )
    right = make_pulley(
        system,
        "belt drive damped driven pulley",
        CENTER_RIGHT,
        RADIUS_PULLEY,
        PULLEY_WIDTH,
        color(0.10, 0.48, 0.30),
        color(0.95, 0.72, 0.12),
    )

    points = make_two_pulley_belt_path(CENTER_LEFT, CENTER_RIGHT, RADIUS_PULLEY, z=BELT_Z)
    make_polyline_body(system, "belt drive continuous belt visual", points, color(0.04, 0.04, 0.045), 11)
    make_polyline_body(system, "belt drive belt centerline highlight", points, color(0.88, 0.18, 0.10), 3)
    path = PolylinePath(points)

    patches = []
    patch_length = path.total_length / BELT_PATCH_COUNT * 0.70
    for i in range(BELT_PATCH_COUNT):
        patch = make_belt_patch(
            system,
            f"moving belt patch {i + 1:02d}",
            patch_length,
            0.026,
            0.018,
            color(0.16, 0.16, 0.17),
        )
        patches.append(patch)

    system._belt_drive_items = {"left": left, "right": right, "path": path, "patches": patches}
    update_visuals(system)
    return system, left, right, patches


def update_visuals(system):
    items = getattr(system, "_belt_drive_items", None)
    if items is None:
        return

    time = system.GetChTime()
    angle = drive_angle(time)
    omega = drive_omega(time)
    belt_offset = RADIUS_PULLEY * angle

    items["left"].SetRot(chrono.QuatFromAngleZ(angle))
    items["left"].SetAngVelParent(chrono.ChVector3d(0, 0, omega))
    items["right"].SetRot(chrono.QuatFromAngleZ(angle))
    items["right"].SetAngVelParent(chrono.ChVector3d(0, 0, omega))
    update_belt_patches(items["path"], items["patches"], belt_offset, z_lift=0.016)


def simulate(duration, step):
    system, left, right, patches = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, left, right, patches


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, left, right, patches = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: beltDriveReevingSystem.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.16, -0.70, 0.42), chrono.ChVector3d(0.16, 0.0, 0.0))
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
            print_state(system, patches)
            next_log += 0.25


def print_state(system, patches):
    sample = patches[0].GetPos()
    omega = drive_omega(system.GetChTime())
    print(
        f"t={system.GetChTime():6.3f}  "
        f"drive_omega={omega:+.4f}  "
        f"belt_speed={omega * RADIUS_PULLEY:+.4f}  "
        f"sample_patch=({sample.x:+.4f}, {sample.y:+.4f}, {sample.z:+.4f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=T_END)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: beltDriveReevingSystem.py -> PyChrono visual belt drive")
    if args.no_vis:
        system, left, right, patches = simulate(args.duration, args.step)
        print_state(system, patches)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

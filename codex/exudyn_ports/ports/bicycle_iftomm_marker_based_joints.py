import argparse
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from bicycle_iftomm_benchmark import (
    BCOM,
    HCOM,
    LAMBDA,
    OMEGA_FRONT_Y0,
    OMEGA_REAR_Y0,
    P1,
    P2,
    P3,
    R_FRONT,
    R_REAR,
    STEP,
    add_local_sphere,
    color,
    make_handlebar,
    make_material,
    make_rear_frame,
    make_wheel,
    make_wheel_height_constraint,
    print_state,
)


# Reproduces the intent of EXUDYN
# Examples/FurtherExamples/bicycleIftommBenchmarkMarkerBasedJoints.py:
# the same IFToMM uncontrolled bicycle, but with the wheel and steering joints
# built from explicit body-local marker frames.


def local_revolute_y(body_a, body_b, point_a_local, point_b_local):
    joint = chrono.ChLinkLockRevolute()
    frame_a = chrono.ChFramed(point_a_local, chrono.Q_ROTATE_Z_TO_Y)
    frame_b = chrono.ChFramed(point_b_local, chrono.Q_ROTATE_Z_TO_Y)
    joint.Initialize(body_a, body_b, True, frame_a, frame_b)
    return joint


def local_steering_revolute(handlebar, rear_frame):
    joint = chrono.ChLinkLockRevolute()
    steer_frame = chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleY(-LAMBDA))
    frame_h = chrono.ChFramed(P2 - HCOM, steer_frame.GetRot())
    frame_b = chrono.ChFramed(P2 - BCOM, steer_frame.GetRot())
    joint.Initialize(handlebar, rear_frame, True, frame_h, frame_b)
    return joint


def build_system():
    system = chrono.ChSystemSMC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    ground_mat = make_material(0.95, 0.01)
    wheel_mat = make_material(0.95, 0.01)

    ground = chrono.ChBodyEasyBox(12.0, 14.0, 0.05, 1000, False, True, ground_mat)
    ground.SetName("marker bicycle ground plane")
    ground.SetFixed(True)
    ground.SetPos(chrono.ChVector3d(4.0, 0, -0.025))
    strip = chrono.ChVisualShapeBox(6.0, 0.035, 0.012)
    strip.SetColor(color(0.42, 0.44, 0.46))
    ground.AddVisualShape(strip, chrono.ChFramed(chrono.ChVector3d(2.0, 0, 0.03)))
    system.AddBody(ground)

    rear_wheel = make_wheel(
        "marker rear rolling wheel",
        R_REAR,
        2.0,
        chrono.ChVector3d(0.0603, 0.12, 0.0603),
        P1,
        OMEGA_REAR_Y0,
        wheel_mat,
        color(0.12, 0.35, 0.88),
    )
    front_wheel = make_wheel(
        "marker front rolling wheel",
        R_FRONT,
        3.0,
        chrono.ChVector3d(0.1405, 0.28, 0.1405),
        P3,
        OMEGA_FRONT_Y0,
        wheel_mat,
        color(0.10, 0.48, 0.90),
    )
    rear_frame = make_rear_frame()
    handlebar = make_handlebar()

    add_local_sphere(rear_frame, P1 - BCOM, 0.050, color(1.0, 0.75, 0.12))
    add_local_sphere(rear_frame, P2 - BCOM, 0.050, color(1.0, 0.75, 0.12))
    add_local_sphere(handlebar, P2 - HCOM, 0.046, color(1.0, 0.75, 0.12))
    add_local_sphere(handlebar, P3 - HCOM, 0.046, color(1.0, 0.75, 0.12))

    for body in (rear_wheel, front_wheel, rear_frame, handlebar):
        system.AddBody(body)

    rear_joint = local_revolute_y(rear_wheel, rear_frame, chrono.ChVector3d(0, 0, 0), P1 - BCOM)
    front_joint = local_revolute_y(front_wheel, handlebar, chrono.ChVector3d(0, 0, 0), P3 - HCOM)
    steer_joint = local_steering_revolute(handlebar, rear_frame)
    rear_height = make_wheel_height_constraint(rear_wheel, ground, P1)
    front_height = make_wheel_height_constraint(front_wheel, ground, P3)

    for joint in (rear_joint, front_joint, steer_joint, rear_height, front_height):
        system.AddLink(joint)

    return system, rear_frame, handlebar, rear_wheel, front_wheel, (
        rear_joint,
        front_joint,
        steer_joint,
        rear_height,
        front_height,
    )


def simulate(duration, step):
    system, rear_frame, handlebar, rear_wheel, front_wheel, joints = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, rear_frame, handlebar, rear_wheel, front_wheel, joints


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, rear_frame, handlebar, rear_wheel, front_wheel, joints = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1120, 760)
    vis.SetWindowTitle("EXUDYN port: bicycleIftommBenchmarkMarkerBasedJoints.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(3.2, 5.0, 2.2), chrono.ChVector3d(1.0, 0, 0.55))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, rear_frame, handlebar, rear_wheel, front_wheel)
            next_log += 0.5


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print(
        "EXUDYN port: bicycleIftommBenchmarkMarkerBasedJoints.py -> "
        "PyChrono marker-frame bicycle benchmark"
    )
    if args.no_vis:
        system, rear_frame, handlebar, rear_wheel, front_wheel, joints = simulate(args.duration, args.step)
        print_state(system, rear_frame, handlebar, rear_wheel, front_wheel)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

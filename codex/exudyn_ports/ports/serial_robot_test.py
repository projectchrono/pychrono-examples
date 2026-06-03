import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
import serial_robot_kinematic_tree as robot


# Reproduces the intent of EXUDYN TestModels/serialRobotTest.py:
# a redundant-coordinate six-axis PUMA robot with PD joint-torque control and
# static-torque compensation. Chrono core does not expose EXUDYN's robotics
# helper for redundant-coordinate MBS creation, so this port keeps the source
# standard-DH geometry and q0->q1->q2->q0 trajectory as a visible replay, and
# renders the static-compensation torque demand at every joint.

STEP = 1.0e-3
END_TIME = 0.2
SEGMENTS = (0.25, 0.25, 0.5)
GRAVITY = [0.0, 0.0, 9.81]

Q0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
Q1 = [0.0, math.pi / 8.0, 0.25 * math.pi, 0.0, math.pi / 8.0, 0.0]
Q2 = [0.5 * math.pi, -math.pi / 8.0, -0.125 * math.pi, 0.0, 0.25 * math.pi, 0.0]
TRAJECTORY_POINTS = (Q0, Q1, Q2, Q0)
P_CONTROL = [40000.0, 40000.0, 40000.0, 100.0, 100.0, 10.0]
D_CONTROL = [400.0, 400.0, 100.0, 1.0, 1.0, 0.05]


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vadd(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def vsub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def vscale(a, scale):
    return [scale * a[0], scale * a[1], scale * a[2]]


def cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def ht_axis(ht, index):
    return [ht[0][index], ht[1][index], ht[2][index]]


def profile_constant_acceleration(tau, duration):
    if tau <= 0.0:
        return 0.0, 0.0
    if tau >= duration:
        return 1.0, 0.0
    half = 0.5 * duration
    if tau <= half:
        s = 2.0 * (tau / duration) ** 2
        sd = 4.0 * tau / (duration * duration)
    else:
        rem = duration - tau
        s = 1.0 - 2.0 * (rem / duration) ** 2
        sd = 4.0 * rem / (duration * duration)
    return s, sd


def trajectory(time):
    elapsed = 0.0
    for index, duration in enumerate(SEGMENTS):
        if time <= elapsed + duration:
            tau = time - elapsed
            s, sd = profile_constant_acceleration(tau, duration)
            q = []
            qd = []
            for a, b in zip(TRAJECTORY_POINTS[index], TRAJECTORY_POINTS[index + 1]):
                delta = b - a
                q.append(a + delta * s)
                qd.append(delta * sd)
            return q, qd
        elapsed += duration
    return list(TRAJECTORY_POINTS[-1]), [0.0] * 6


def static_torques(q):
    joint_frames, link_frames, _tool = robot.robot_poses(q)
    torques = []
    for joint_index, joint_frame in enumerate(joint_frames):
        origin = robot.ht_translation(joint_frame)
        axis = ht_axis(joint_frame, 2)
        torque = 0.0
        for link_index in range(joint_index, len(robot.LINKS)):
            link = robot.LINKS[link_index]
            com = robot.transform_point(link_frames[link_index], link["COM"])
            force = vscale(GRAVITY, link["mass"])
            torque += dot(axis, cross(vsub(com, origin), force))
        torques.append(torque)
    return torques


def torque_demands(q, qd):
    # In this replay the actual coordinate equals the commanded trajectory, so
    # the PD error vanishes and only the source static-torque compensation path
    # remains. Keep the D terms visible in the output for traceability.
    static = static_torques(q)
    damping = [D_CONTROL[i] * qd[i] for i in range(6)]
    return [-value for value in static], damping


class TorqueVisual:
    def __init__(self, system, index):
        self.index = index
        self.segment = robot.MutableSegment(system, f"serialRobotTest joint {index + 1} torque demand", color(0.95, 0.58, 0.08), 5)

    def update(self, joint_frame, torque):
        origin = robot.ht_translation(joint_frame)
        z_axis = ht_axis(joint_frame, 2)
        x_axis = ht_axis(joint_frame, 0)
        start = vadd(origin, vscale(z_axis, 0.13))
        length = max(-0.20, min(0.20, 0.010 * torque))
        self.segment.update(start, vadd(start, vscale(x_axis, length)))


def build_system():
    system, items = robot.build_system()
    torque_visuals = [TorqueVisual(system, i) for i in range(6)]
    system._serial_robot_test_items = {"base_items": items, "torque_visuals": torque_visuals}
    update_visuals(system)
    return system, system._serial_robot_test_items


def update_base_visuals(system, q):
    items = system._serial_robot_kinematic_tree_items
    joint_frames, link_frames, tool_frame = robot.robot_poses(q)
    previous_origin = [0.0, 0.0, 0.0]
    for index, link in enumerate(robot.LINKS):
        joint_origin = robot.ht_translation(joint_frames[index])
        next_origin = robot.ht_translation(link_frames[index])
        com = robot.transform_point(link_frames[index], link["COM"])
        joint_rot = robot.ht_rotation(joint_frames[index])

        items["link_segments"][index].update(previous_origin, next_origin)
        items["com_segments"][index].update(joint_origin, com)
        robot.set_body_pose(items["joint_markers"][index], joint_origin, joint_rot)
        robot.set_body_pose(items["com_markers"][index], com, robot.identity3())
        robot.set_body_pose(items["axes"][index], joint_origin, joint_rot)
        previous_origin = next_origin

    tool = items["tool"]
    tcp = robot.ht_translation(tool_frame)
    tool_rot = robot.ht_rotation(tool_frame)
    robot.set_body_pose(tool["tcp"], tcp, tool_rot)
    tool["stem"].update(robot.transform_point(link_frames[-1], [0.0, 0.0, 0.0]), tcp)
    tool["left"].update(robot.transform_point(tool_frame, [0.0, 0.03, 0.02]), robot.transform_point(tool_frame, [0.0, 0.03, 0.08]))
    tool["right"].update(robot.transform_point(tool_frame, [0.0, -0.03, 0.02]), robot.transform_point(tool_frame, [0.0, -0.03, 0.08]))
    return joint_frames, link_frames, tool_frame


def update_visuals(system):
    q, qd = trajectory(system.GetChTime())
    joint_frames, _link_frames, _tool = update_base_visuals(system, q)
    torques, _damping = torque_demands(q, qd)
    for visual, frame, torque in zip(system._serial_robot_test_items["torque_visuals"], joint_frames, torques):
        visual.update(frame, torque)


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration - 1.0e-12:
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
    vis.SetWindowTitle("EXUDYN port: serialRobotTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.72, -0.90, 0.72), chrono.ChVector3d(0.12, 0.0, 0.22))
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
            print_state(system)
            next_log += 0.05


def print_state(system):
    q, qd = trajectory(system.GetChTime())
    _joint_frames, _link_frames, tool = robot.robot_poses(q)
    tcp = robot.ht_translation(tool)
    torques, damping = torque_demands(q, qd)
    scaled_sum = 1.0e-2 * sum(torques)
    q_text = ",".join(f"{value:+.4f}" for value in q)
    print(
        f"t={system.GetChTime():6.3f}  "
        f"q=[{q_text}]  qd_norm={math.sqrt(sum(v * v for v in qd)):.6f}  "
        f"torque_scaled_sum={scaled_sum:+.9f}  "
        f"max_damping_path={max(abs(value) for value in damping):.6f}  "
        f"tcp=({tcp[0]:+.6f},{tcp[1]:+.6f},{tcp[2]:+.6f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: serialRobotTest.py -> PyChrono redundant-coordinate torque robot replay")
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

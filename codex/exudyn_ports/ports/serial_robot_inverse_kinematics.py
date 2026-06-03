import argparse
import math

import numpy as np
import pychrono.core as chrono


# Reproduces EXUDYN Examples/serialRobotInverseKinematics.py:
# a PUMA 560 ObjectKinematicTree whose joint offsets are driven by
# InverseKinematicsNumerical while the TCP follows two workspace SE(3) moves.
# This PyChrono port keeps the source PUMA standard-DH model, HTtool=[0,0,0.08],
# motionCase=2 target frames, a numerical damped-least-squares IK solve,
# visible robot/joint axes/tool graphics, and TCP target/actual traces.

STEP = 2.0e-3
END_TIME = 5.0
SAMPLE_DT = 0.025
HT_TOOL = np.array([0.0, 0.0, 0.08], dtype=float)
SOURCE_Q0 = np.array([0.0, 0.0, 0.0, 0.01, 0.02, 0.03], dtype=float)

PUMA_LINKS = (
    {"stdDH": [0.0, 0.0, 0.0, 0.5 * math.pi], "COM": [0.0, 0.0, 0.0]},
    {"stdDH": [0.0, 0.0, 0.4318, 0.0], "COM": [-0.3638, 0.006, 0.2275]},
    {"stdDH": [0.0, 0.15005, 0.0203, -0.5 * math.pi], "COM": [-0.0203, -0.0141, 0.07]},
    {"stdDH": [0.0, 0.4318, 0.0, 0.5 * math.pi], "COM": [0.0, 0.019, 0.0]},
    {"stdDH": [0.0, 0.0, 0.0, -0.5 * math.pi], "COM": [0.0, 0.0, 0.0]},
    {"stdDH": [0.0, 0.0, 0.0, 0.0], "COM": [0.0, 0.0, 0.032]},
)

LINK_COLORS = (
    (0.78, 0.22, 0.18),
    (0.10, 0.46, 0.84),
    (0.14, 0.62, 0.28),
    (0.86, 0.56, 0.12),
    (0.48, 0.34, 0.78),
    (0.10, 0.60, 0.62),
)


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vector(values):
    return chrono.ChVector3d(float(values[0]), float(values[1]), float(values[2]))


def skew(vector3):
    x, y, z = vector3
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=float)


def ht_rot_x(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]], dtype=float)


def ht_rot_z(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)


def ht_trans_x(value):
    return np.array([[1, 0, 0, value], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)


def ht_trans_z(value):
    return np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, value], [0, 0, 0, 1]], dtype=float)


def ht_translate(values):
    ht = np.eye(4)
    ht[:3, 3] = values
    return ht


def std_dh(theta, d, a, alpha):
    return ht_rot_z(theta) @ ht_trans_z(d) @ ht_trans_x(a) @ ht_rot_x(alpha)


def fk(q):
    frames = []
    joints = []
    ht = np.eye(4)
    for index, link in enumerate(PUMA_LINKS):
        joint_ht = ht @ ht_rot_z(q[index])
        joints.append(joint_ht)
        ht = joint_ht @ std_dh(*link["stdDH"])
        frames.append(ht)
    tool = ht @ ht_translate(HT_TOOL)
    return joints, frames, tool


BASE_TARGET = fk(np.zeros(6))[2]
MOVE = np.eye(4)
MOVE[:3, :3] = ht_rot_x(0.3 * math.pi)[:3, :3]
MOVE[:3, 3] = [0.0, 0.0, -0.3]


def rotvec_from_matrix(rot):
    value = max(-1.0, min(1.0, 0.5 * (np.trace(rot) - 1.0)))
    angle = math.acos(value)
    if angle < 1e-10:
        return np.zeros(3)
    return 0.5 * angle / math.sin(angle) * np.array(
        [rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1]]
    )


def so3_exp(omega):
    theta = np.linalg.norm(omega)
    if theta < 1e-12:
        return np.eye(3) + skew(omega)
    wx = skew(omega)
    return np.eye(3) + math.sin(theta) / theta * wx + (1.0 - math.cos(theta)) / (theta * theta) * (wx @ wx)


def so3_log(rot):
    return rotvec_from_matrix(rot)


def se3_log(ht):
    rot = ht[:3, :3]
    trans = ht[:3, 3]
    omega = so3_log(rot)
    theta = np.linalg.norm(omega)
    if theta < 1e-12:
        v = trans
    else:
        wx = skew(omega)
        coefficient = 1.0 / (theta * theta) - (1.0 + math.cos(theta)) / (2.0 * theta * math.sin(theta))
        v = (np.eye(3) - 0.5 * wx + coefficient * (wx @ wx)) @ trans
    return v, omega


def se3_exp(v, omega):
    theta = np.linalg.norm(omega)
    rot = so3_exp(omega)
    if theta < 1e-12:
        trans = v
    else:
        wx = skew(omega)
        trans = (np.eye(3) + (1.0 - math.cos(theta)) / (theta * theta) * wx + (theta - math.sin(theta)) / (theta**3) * (wx @ wx)) @ v
    ht = np.eye(4)
    ht[:3, :3] = rot
    ht[:3, 3] = trans
    return ht


MOVE_V, MOVE_OMEGA = se3_log(MOVE)


def interpolated_move(fraction):
    return se3_exp(MOVE_V * fraction, MOVE_OMEGA * fraction)


def target_transform(time):
    if time < 2.0:
        return BASE_TARGET @ interpolated_move(min(time, 1.0))
    if time < 4.0:
        return BASE_TARGET @ MOVE @ interpolated_move(min(time - 2.0, 1.0))
    return BASE_TARGET @ MOVE @ MOVE


def pose_error(q, target):
    _, _, tool = fk(q)
    pos_error = target[:3, 3] - tool[:3, 3]
    rot_error = rotvec_from_matrix(target[:3, :3] @ tool[:3, :3].T)
    return np.concatenate((pos_error, rot_error))


def solve_ik(target, seed):
    q = seed.astype(float).copy()
    damping = 6.0e-3
    eps = 1.0e-6
    success = False
    for iteration in range(70):
        error = pose_error(q, target)
        if np.linalg.norm(error) < 1.5e-5:
            success = True
            break
        jac = np.zeros((6, len(q)))
        for col in range(len(q)):
            qp = q.copy()
            qp[col] += eps
            jac[:, col] = (pose_error(qp, target) - error) / eps
        lhs = jac.T @ jac + (damping * damping) * np.eye(len(q))
        rhs = -jac.T @ error
        delta = np.linalg.solve(lhs, rhs)
        norm = np.linalg.norm(delta)
        if norm > 0.30:
            delta *= 0.30 / norm
        q += delta
    return q, success, np.linalg.norm(pose_error(q, target)), iteration + 1


def precompute_path():
    times = np.arange(0.0, END_TIME + 0.5 * SAMPLE_DT, SAMPLE_DT)
    qs = []
    targets = []
    errors = []
    successes = []
    iterations = []
    seed = SOURCE_Q0.copy()
    for time in times:
        target = target_transform(float(time))
        seed, success, error_norm, count = solve_ik(target, seed)
        qs.append(seed.copy())
        targets.append(target)
        errors.append(error_norm)
        successes.append(success)
        iterations.append(count)
    return times, np.array(qs), targets, np.array(errors), successes, iterations


PATH_TIMES, PATH_QS, PATH_TARGETS, PATH_ERRORS, PATH_SUCCESSES, PATH_ITERS = precompute_path()


def q_at_time(time):
    if time <= PATH_TIMES[0]:
        return PATH_QS[0].copy()
    if time >= PATH_TIMES[-1]:
        return PATH_QS[-1].copy()
    index = int(np.searchsorted(PATH_TIMES, time) - 1)
    alpha = (time - PATH_TIMES[index]) / (PATH_TIMES[index + 1] - PATH_TIMES[index])
    return (1.0 - alpha) * PATH_QS[index] + alpha * PATH_QS[index + 1]


def quat_from_matrix(m):
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    q = chrono.ChQuaterniond(w, x, y, z)
    q.Normalize()
    return q


class MutableSegment:
    def __init__(self, system, name, tint, thickness=4):
        self.body = chrono.ChBody()
        self.body.SetName(name)
        self.body.SetFixed(True)
        self.body.EnableCollision(False)
        self.shape = chrono.ChVisualShapeSegment()
        self.shape.SetMutable(True)
        self.shape.SetColor(tint)
        self.shape.SetThickness(thickness)
        self.body.AddVisualShape(self.shape)
        system.AddBody(self.body)

    def update(self, start, end):
        self.shape.SetLineGeometry(chrono.ChLineSegment(vector(start), vector(end)))
        self.body.UpdateVisualModel()


class MutableFrame:
    def __init__(self, system, prefix, length):
        self.length = length
        self.axes = (
            MutableSegment(system, f"{prefix} x axis", color(0.92, 0.10, 0.08), 4),
            MutableSegment(system, f"{prefix} y axis", color(0.10, 0.62, 0.18), 4),
            MutableSegment(system, f"{prefix} z axis", color(0.10, 0.24, 0.88), 4),
        )

    def update(self, ht):
        origin = ht[:3, 3]
        for index, axis in enumerate(self.axes):
            axis.update(origin, origin + self.length * ht[:3, index])


def set_body_pose(body, position, rotation):
    body.SetPos(vector(position))
    body.SetRot(quat_from_matrix(rotation))


def make_marker(system, name, radius, tint):
    body = chrono.ChBodyEasySphere(radius, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def make_axis(system, name):
    body = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.007, 0.15, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(color(0.02, 0.02, 0.025))
    system.AddBody(body)
    return body


def make_box(system, name, center, size, tint, opacity=1.0):
    body = chrono.ChBodyEasyBox(size[0], size[1], size[2], 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(vector(center))
    body.GetVisualShape(0).SetColor(tint)
    body.GetVisualShape(0).SetOpacity(opacity)
    system.AddBody(body)
    return body


def add_static_segment(system, name, start, end, tint, thickness=3):
    segment = MutableSegment(system, name, tint, thickness)
    segment.update(np.array(start, dtype=float), np.array(end, dtype=float))
    return segment


def make_ground(system):
    make_box(system, "PUMA IK base pedestal", [0, 0, -0.35], [0.12, 0.12, 0.50], color(0.48, 0.48, 0.50))
    make_box(system, "PUMA IK checker floor", [0, 0, -0.61], [2.0, 2.0, 0.018], color(0.68, 0.70, 0.68), 0.35)
    for value in np.linspace(-1.0, 1.0, 11):
        add_static_segment(system, f"PUMA IK floor x grid {value:.1f}", [value, -1.0, -0.596], [value, 1.0, -0.596], color(0.42, 0.42, 0.42), 1)
        add_static_segment(system, f"PUMA IK floor y grid {value:.1f}", [-1.0, value, -0.595], [1.0, value, -0.595], color(0.42, 0.42, 0.42), 1)
    add_static_segment(system, "PUMA IK base x axis", [0, 0, 0], [0.5, 0, 0], color(0.92, 0.10, 0.08), 3)
    add_static_segment(system, "PUMA IK base y axis", [0, 0, 0], [0, 0.5, 0], color(0.10, 0.62, 0.18), 3)
    add_static_segment(system, "PUMA IK base z axis", [0, 0, 0], [0, 0, 0.5], color(0.10, 0.24, 0.88), 3)


def make_tool(system):
    tcp = make_marker(system, "PUMA IK actual TCP", 0.024, color(0.96, 0.24, 0.10))
    target = make_marker(system, "PUMA IK moving target TCP", 0.026, color(0.04, 0.04, 0.045))
    stem = MutableSegment(system, "PUMA IK red tool stem", color(0.92, 0.10, 0.08), 5)
    left = MutableSegment(system, "PUMA IK left gripper finger", color(0.55, 0.55, 0.57), 5)
    right = MutableSegment(system, "PUMA IK right gripper finger", color(0.55, 0.55, 0.57), 5)
    return {"tcp": tcp, "target": target, "stem": stem, "left": left, "right": right}


def add_reference_frames(system):
    for name, ht, length in (
        ("PUMA IK source start frame", BASE_TARGET, 0.20),
        ("PUMA IK source move 1 frame", BASE_TARGET @ MOVE, 0.20),
        ("PUMA IK source move 2 frame", BASE_TARGET @ MOVE @ MOVE, 0.20),
    ):
        frame = MutableFrame(system, name, length)
        frame.update(ht)


def add_target_trace(system):
    segments = []
    for index in range(1, len(PATH_TARGETS)):
        start = PATH_TARGETS[index - 1][:3, 3]
        end = PATH_TARGETS[index][:3, 3]
        seg = MutableSegment(system, f"PUMA IK prescribed TCP trace {index}", color(0.04, 0.04, 0.045), 2)
        seg.update(start, end)
        segments.append(seg)
    return segments


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
    make_ground(system)
    add_reference_frames(system)
    add_target_trace(system)

    link_segments = []
    com_segments = []
    joint_markers = []
    com_markers = []
    axes = []
    for index, rgb in enumerate(LINK_COLORS):
        tint = color(*rgb)
        link_segments.append(MutableSegment(system, f"PUMA IK solved link {index + 1}", tint, 7))
        com_segments.append(MutableSegment(system, f"PUMA IK link {index + 1} COM arm", tint, 4))
        joint_markers.append(make_marker(system, f"PUMA IK joint {index + 1}", 0.045, tint))
        com_markers.append(make_marker(system, f"PUMA IK COM marker {index + 1}", 0.017 + 0.002 * index, color(0.04, 0.04, 0.045)))
        axes.append(make_axis(system, f"PUMA IK visible joint {index + 1} z axis"))

    moving_target_frame = MutableFrame(system, "PUMA IK moving target frame", 0.15)
    tool = make_tool(system)
    items = {
        "link_segments": link_segments,
        "com_segments": com_segments,
        "joint_markers": joint_markers,
        "com_markers": com_markers,
        "axes": axes,
        "tool": tool,
        "moving_target_frame": moving_target_frame,
    }
    system._serial_robot_inverse_kinematics_items = items
    update_visuals(system)
    return system, items


def update_visuals(system):
    items = system._serial_robot_inverse_kinematics_items
    time = system.GetChTime()
    q = q_at_time(time)
    target = target_transform(time)
    joints, frames, tool = fk(q)
    previous = np.array([0.0, 0.0, 0.0])

    for index, link in enumerate(PUMA_LINKS):
        joint_origin = joints[index][:3, 3]
        next_origin = frames[index][:3, 3]
        com = frames[index] @ np.array(list(link["COM"]) + [1.0])
        items["link_segments"][index].update(previous, next_origin)
        items["com_segments"][index].update(joint_origin, com[:3])
        set_body_pose(items["joint_markers"][index], joint_origin, joints[index][:3, :3])
        set_body_pose(items["com_markers"][index], com[:3], np.eye(3))
        set_body_pose(items["axes"][index], joint_origin, joints[index][:3, :3])
        previous = next_origin

    tool_items = items["tool"]
    set_body_pose(tool_items["tcp"], tool[:3, 3], tool[:3, :3])
    set_body_pose(tool_items["target"], target[:3, 3], target[:3, :3])
    tool_items["stem"].update(frames[-1][:3, 3], tool[:3, 3])
    tool_items["left"].update(tool[:3, 3] + tool[:3, :3] @ np.array([0.0, 0.03, -0.02]), tool[:3, 3] + tool[:3, :3] @ np.array([0.0, 0.03, 0.04]))
    tool_items["right"].update(tool[:3, 3] + tool[:3, :3] @ np.array([0.0, -0.03, -0.02]), tool[:3, 3] + tool[:3, :3] @ np.array([0.0, -0.03, 0.04]))
    items["moving_target_frame"].update(target)


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, items


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1120, 760)
    vis.SetWindowTitle("EXUDYN port: serialRobotInverseKinematics.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.15, -1.55, 1.00), chrono.ChVector3d(0.18, -0.02, 0.10))
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
            next_log += 0.5


def print_state(system):
    time = system.GetChTime()
    q = q_at_time(time)
    target = target_transform(time)
    _, _, tool = fk(q)
    error = pose_error(q, target)
    pos_error = np.linalg.norm(error[:3])
    rot_error = np.linalg.norm(error[3:])
    q_text = ",".join(f"{value:+.4f}" for value in q)
    print(
        f"t={time:6.3f}  q=[{q_text}]  q_sum={float(np.sum(q)):+.9f}  "
        f"pos_error={pos_error:.3e}  rot_error={rot_error:.3e}  "
        f"path_max_error={float(np.max(PATH_ERRORS)):.3e}  path_success={all(PATH_SUCCESSES)}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: serialRobotInverseKinematics.py -> PyChrono PUMA workspace IK replay")
    if args.no_vis:
        system, _ = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

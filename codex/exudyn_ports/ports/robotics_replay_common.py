import argparse
import math

import numpy as np
import pychrono.core as chrono


def color(r, g, b):
    return chrono.ChColor(float(r), float(g), float(b))


def vec(x, y=None, z=None):
    if y is None:
        return chrono.ChVector3d(float(x[0]), float(x[1]), float(x[2]))
    return chrono.ChVector3d(float(x), float(y), float(z))


def np_vec(values):
    return np.array(values, dtype=float)


def add_np(a, b):
    return np.asarray(a, dtype=float) + np.asarray(b, dtype=float)


def smoothstep(u):
    u = min(max(float(u), 0.0), 1.0)
    return u * u * (3.0 - 2.0 * u)


def smoothstep_t(u):
    u = min(max(float(u), 0.0), 1.0)
    return 6.0 * u * (1.0 - u)


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
        remaining = duration - tau
        s = 1.0 - 2.0 * (remaining / duration) ** 2
        sd = 4.0 * remaining / (duration * duration)
    return s, sd


def piecewise_profile(points, durations, time):
    points = [np.asarray(point, dtype=float) for point in points]
    if not durations:
        return points[0].copy(), np.zeros_like(points[0])
    elapsed = 0.0
    for index, duration in enumerate(durations):
        if time <= elapsed + duration:
            tau = time - elapsed
            s, sd = profile_constant_acceleration(tau, duration)
            delta = points[index + 1] - points[index]
            return points[index] + s * delta, sd * delta
        elapsed += duration
    return points[-1].copy(), np.zeros_like(points[-1])


def ht_identity():
    return np.eye(4)


def ht_translate(values):
    ht = np.eye(4)
    ht[:3, 3] = np.asarray(values, dtype=float)
    return ht


def ht_rot_x(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]], dtype=float)


def ht_rot_y(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]], dtype=float)


def ht_rot_z(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)


def ht_trans_x(value):
    return np.array([[1, 0, 0, value], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)


def ht_trans_z(value):
    return np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, value], [0, 0, 0, 1]], dtype=float)


def std_dh(theta, d, a, alpha):
    return ht_rot_z(theta) @ ht_trans_z(d) @ ht_trans_x(a) @ ht_rot_x(alpha)


def transform_point(ht, point):
    point4 = np.array([point[0], point[1], point[2], 1.0], dtype=float)
    return (ht @ point4)[:3]


def quat_from_matrix(m):
    trace = float(m[0, 0] + m[1, 1] + m[2, 2])
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = chrono.ChQuaterniond(w, x, y, z)
    q.Normalize()
    return q


def rot2(phi):
    c = math.cos(phi)
    s = math.sin(phi)
    return np.array([[c, -s], [s, c]], dtype=float)


def rz3(phi):
    c = math.cos(phi)
    s = math.sin(phi)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


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
        self.shape.SetLineGeometry(chrono.ChLineSegment(vec(start), vec(end)))
        self.body.UpdateVisualModel()


class MutablePolyline:
    def __init__(self, system, name, tint, thickness=3):
        self.body = chrono.ChBody()
        self.body.SetName(name)
        self.body.SetFixed(True)
        self.body.EnableCollision(False)
        self.shape = chrono.ChVisualShapeLine()
        self.shape.SetMutable(True)
        self.shape.SetColor(tint)
        self.shape.SetThickness(thickness)
        self.body.AddVisualShape(self.shape)
        system.AddBody(self.body)

    def update(self, points):
        line = chrono.ChLinePoly(len(points))
        for index, point in enumerate(points):
            line.SetPoint(index, vec(point))
        self.shape.SetLineGeometry(line)
        self.body.UpdateVisualModel()


def make_marker(system, name, radius, tint, opacity=1.0):
    body = chrono.ChBodyEasySphere(radius, 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    shape = body.GetVisualShape(0)
    shape.SetColor(tint)
    shape.SetOpacity(float(opacity))
    system.AddBody(body)
    return body


def make_box(system, name, size, tint, pos=(0, 0, 0), opacity=1.0):
    body = chrono.ChBodyEasyBox(size[0], size[1], size[2], 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(vec(pos))
    shape = body.GetVisualShape(0)
    shape.SetColor(tint)
    shape.SetOpacity(float(opacity))
    system.AddBody(body)
    return body


def make_cylinder(system, name, axis, radius, height, tint, pos=(0, 0, 0), opacity=1.0):
    body = chrono.ChBodyEasyCylinder(axis, radius, height, 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(vec(pos))
    shape = body.GetVisualShape(0)
    shape.SetColor(tint)
    shape.SetOpacity(float(opacity))
    system.AddBody(body)
    return body


def add_grid_ground(system, name, center, size, z=0.0, tile_count=8, tint=None):
    tint = tint or color(0.70, 0.72, 0.69)
    make_box(system, name + " slab", (size[0], size[1], 0.025), tint, (center[0], center[1], z - 0.0125), 0.34)
    x0 = center[0] - 0.5 * size[0]
    x1 = center[0] + 0.5 * size[0]
    y0 = center[1] - 0.5 * size[1]
    y1 = center[1] + 0.5 * size[1]
    for i in range(tile_count + 1):
        x = x0 + size[0] * i / tile_count
        MutableSegment(system, f"{name} x-grid {i}", color(0.36, 0.36, 0.36), 1).update((x, y0, z + 0.002), (x, y1, z + 0.002))
    for j in range(tile_count + 1):
        y = y0 + size[1] * j / tile_count
        MutableSegment(system, f"{name} y-grid {j}", color(0.36, 0.36, 0.36), 1).update((x0, y, z + 0.003), (x1, y, z + 0.003))


def set_body_pose(body, position, rotation=None):
    body.SetPos(vec(position))
    if rotation is not None:
        body.SetRot(quat_from_matrix(np.asarray(rotation, dtype=float)))
    body.UpdateVisualModel()


UR5_LINKS = (
    {"stdDH": [0.0, 0.089159, 0.0, 0.5 * math.pi], "COM": [0.0, 0.0, 0.045]},
    {"stdDH": [0.0, 0.0, -0.425, 0.0], "COM": [-0.2125, 0.0, 0.0]},
    {"stdDH": [0.0, 0.0, -0.39225, 0.0], "COM": [-0.196, 0.0, 0.0]},
    {"stdDH": [0.0, 0.10915, 0.0, 0.5 * math.pi], "COM": [0.0, 0.0, 0.055]},
    {"stdDH": [0.0, 0.09465, 0.0, -0.5 * math.pi], "COM": [0.0, 0.0, 0.047]},
    {"stdDH": [0.0, 0.0823, 0.0, 0.0], "COM": [0.0, 0.0, 0.041]},
)

PUMA_LINKS = (
    {"stdDH": [0.0, 0.0, 0.0, 0.5 * math.pi], "COM": [0.0, 0.0, 0.0]},
    {"stdDH": [0.0, 0.0, 0.4318, 0.0], "COM": [-0.3638, 0.006, 0.2275]},
    {"stdDH": [0.0, 0.15, 0.0203, -0.5 * math.pi], "COM": [-0.0203, -0.0141, 0.07]},
    {"stdDH": [0.0, 0.4318, 0.0, 0.5 * math.pi], "COM": [0.0, 0.019, 0.0]},
    {"stdDH": [0.0, 0.0, 0.0, -0.5 * math.pi], "COM": [0.0, 0.0, 0.0]},
    {"stdDH": [0.0, 0.0, 0.0, 0.0], "COM": [0.0, 0.0, 0.032]},
)

ROBOT_COLORS = (
    (0.78, 0.22, 0.18),
    (0.10, 0.46, 0.84),
    (0.14, 0.62, 0.28),
    (0.86, 0.56, 0.12),
    (0.48, 0.34, 0.78),
    (0.10, 0.60, 0.62),
)


def serial_fk(links, q, base_ht=None, tool_offset=(0, 0, 0.08)):
    frames = []
    joint_frames = []
    ht = np.eye(4) if base_ht is None else np.asarray(base_ht, dtype=float).copy()
    for index, link in enumerate(links):
        joint_ht = ht @ ht_rot_z(float(q[index]))
        joint_frames.append(joint_ht)
        ht = joint_ht @ std_dh(*link["stdDH"])
        frames.append(ht)
    tool = ht @ ht_translate(tool_offset)
    return joint_frames, frames, tool


def make_serial_arm(system, prefix, links, base_ht, tool_offset, base_size=(0.18, 0.18, 0.10), link_thickness=7):
    base = make_box(system, prefix + " visible base", base_size, color(0.50, 0.49, 0.47), transform_point(base_ht, (0, 0, -0.05)), 0.90)
    base.SetRot(quat_from_matrix(base_ht[:3, :3]))
    segments = []
    com_segments = []
    joints = []
    axes = []
    com_markers = []
    for index in range(len(links)):
        tint = color(*ROBOT_COLORS[index % len(ROBOT_COLORS)])
        segments.append(MutableSegment(system, f"{prefix} link {index + 1} visible body", tint, link_thickness))
        com_segments.append(MutableSegment(system, f"{prefix} link {index + 1} COM cue", color(0.04, 0.04, 0.045), 3))
        joints.append(make_marker(system, f"{prefix} joint {index + 1}", 0.034, tint))
        axes.append(make_cylinder(system, f"{prefix} joint {index + 1} z-axis", chrono.ChAxis_Z, 0.007, 0.16, color(0.03, 0.03, 0.035)))
        com_markers.append(make_marker(system, f"{prefix} link {index + 1} COM marker", 0.014, color(0.02, 0.02, 0.025)))
    tcp = make_marker(system, prefix + " TCP contact/tool sphere", 0.030, color(0.96, 0.06, 0.04))
    tool_stem = MutableSegment(system, prefix + " tool stem", color(0.96, 0.06, 0.04), 4)
    return {
        "prefix": prefix,
        "links": links,
        "base_ht": base_ht,
        "tool_offset": np.asarray(tool_offset, dtype=float),
        "base": base,
        "segments": segments,
        "com_segments": com_segments,
        "joints": joints,
        "axes": axes,
        "com_markers": com_markers,
        "tcp": tcp,
        "tool_stem": tool_stem,
        "last_tcp": np.zeros(3),
        "last_q": np.zeros(len(links)),
    }


def update_serial_arm(items, q):
    q = np.asarray(q, dtype=float)
    joint_frames, link_frames, tool = serial_fk(items["links"], q, items["base_ht"], items["tool_offset"])
    previous = transform_point(items["base_ht"], (0, 0, 0))
    for index, link in enumerate(items["links"]):
        joint = joint_frames[index]
        frame = link_frames[index]
        origin = transform_point(joint, (0, 0, 0))
        end = transform_point(frame, (0, 0, 0))
        com = transform_point(frame, link["COM"])
        rot = joint[:3, :3]
        items["segments"][index].update(previous, end)
        items["com_segments"][index].update(origin, com)
        set_body_pose(items["joints"][index], origin)
        set_body_pose(items["axes"][index], origin, rot)
        set_body_pose(items["com_markers"][index], com)
        previous = end
    tcp = transform_point(tool, (0, 0, 0))
    flange = transform_point(link_frames[-1], (0, 0, 0))
    set_body_pose(items["tcp"], tcp, tool[:3, :3])
    items["tool_stem"].update(flange, tcp)
    items["last_tcp"] = tcp
    items["last_q"] = q


SERIAL_URDF_Q0 = np.zeros(6)
SERIAL_URDF_Q1 = np.array([0.5 * math.pi, 0.0, -0.25 * math.pi, 0.0, 0.0, 0.0])
SERIAL_URDF_Q2 = np.array([0.0, -math.pi / 8.0, -0.5 * math.pi, 0.0, math.pi / 8.0, 0.0])
SERIAL_URDF_Q3 = np.array([0.8 * math.pi, -0.8 * math.pi, -0.4 * math.pi, 0.75 * math.pi, -0.4 * math.pi, 0.4 * math.pi])
SERIAL_URDF_POINTS = (SERIAL_URDF_Q0, SERIAL_URDF_Q1, SERIAL_URDF_Q2, SERIAL_URDF_Q3, SERIAL_URDF_Q0)
SERIAL_URDF_DURATIONS = (0.25, 0.25, 0.25, 0.25)
SERIAL_URDF_END = 1.5


def serial_urdf_state(time):
    return piecewise_profile(SERIAL_URDF_POINTS, SERIAL_URDF_DURATIONS, time)


def build_serial_robot_urdf_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, "UR5 URDF checker ground", (0.0, 0.0), (2.5, 2.5), z=-0.015, tile_count=10)
    arm = make_serial_arm(system, "UR5 URDF", UR5_LINKS, np.eye(4), (0.0, 0.0, 0.10), base_size=(0.22, 0.22, 0.12), link_thickness=8)
    path = MutablePolyline(system, "UR5 URDF TCP source trajectory trace", color(0.95, 0.16, 0.08), 4)
    points = []
    for i in range(80):
        q, _ = serial_urdf_state(i / 79 * sum(SERIAL_URDF_DURATIONS))
        points.append(transform_point(serial_fk(UR5_LINKS, q, np.eye(4), (0, 0, 0.10))[2], (0, 0, 0)))
    path.update(points)
    system._serial_robot_urdf = {"arm": arm, "path": path}
    update_serial_robot_urdf_visuals(system)
    return system, system._serial_robot_urdf


def update_serial_robot_urdf_visuals(system):
    items = system._serial_robot_urdf
    q, qd = serial_urdf_state(system.GetChTime())
    update_serial_arm(items["arm"], q)
    items["last"] = {"q": q, "qd": qd}


def print_serial_robot_urdf_state(system):
    items = system._serial_robot_urdf
    data = items["last"]
    tcp = items["arm"]["last_tcp"]
    print(
        f"t={system.GetChTime():.4f} source=serialRobotURDF.py model=UR5 nJoints=6 "
        f"q_sum={float(np.sum(data['q'])):+.9f} qd_norm={float(np.linalg.norm(data['qd'])):.6f} "
        f"tcp=({tcp[0]:+.6f},{tcp[1]:+.6f},{tcp[2]:+.6f}) "
        f"trajectory=4x ProfileConstantAcceleration(0.25s) P=[1e5,1e5,1e5,1e3,1e3,1e3]"
    )


LEFT_SHOULDER = np.array([0.086, -0.185, 1.383])
LEFT_ELBOW0 = np.array([0.109, -0.206, 1.122])
LEFT_HAND0 = np.array([0.0945, -0.2520, 0.8536])
HUMAN_Q0 = np.zeros(7)
HUMAN_Q1 = np.array([0.0, math.pi * 0.125, -0.15 * math.pi, math.pi * (0.375 - 0.03), 0.0, 0.0, 0.0])
HUMAN_DURATION = 1.25
PUMA_BASE = ht_translate((-1.0, -0.2, 0.75))
PUMA_Q0 = np.array([0.0, 0.5 * math.pi, -0.5 * math.pi, 0.0, 0.0, 0.0])
PUMA_Q3 = np.array([-0.35 * math.pi, 0.25 * math.pi, -0.75 * math.pi, 0.0, 0.0, 0.0])
PUMA_Q4 = np.array([0.07 * math.pi, 0.38 * math.pi, -0.88 * math.pi, 0.0, 0.0, 0.0])


def human_arm_state(time):
    if time <= 0.25:
        return HUMAN_Q0.copy(), np.zeros(7)
    if time <= 0.65:
        s, sd = profile_constant_acceleration(time - 0.25, 0.4)
        return HUMAN_Q0 + s * (HUMAN_Q1 - HUMAN_Q0), sd * (HUMAN_Q1 - HUMAN_Q0)
    return HUMAN_Q1.copy(), np.zeros(7)


def puma_interaction_state(time):
    return piecewise_profile((PUMA_Q0, PUMA_Q0, PUMA_Q3, PUMA_Q4), (0.25, 0.25, 0.2), time)


def rotate3_xyz(rx, ry, rz):
    return (ht_rot_x(rx) @ ht_rot_y(ry) @ ht_rot_z(rz))[:3, :3]


def human_points(q):
    upper0 = LEFT_ELBOW0 - LEFT_SHOULDER
    lower0 = LEFT_HAND0 - LEFT_ELBOW0
    shoulder_rot = rotate3_xyz(q[0], q[1], q[2])
    elbow = LEFT_SHOULDER + shoulder_rot @ upper0
    elbow_rot = shoulder_rot @ ht_rot_y(q[3])[:3, :3]
    hand = elbow + elbow_rot @ lower0
    wrist_rot = elbow_rot @ rotate3_xyz(q[4], q[5], q[6])
    return LEFT_SHOULDER, elbow, hand, shoulder_rot, elbow_rot, wrist_rot


def build_human_body(system):
    make_box(system, "human interaction torso STL substitute", (0.26, 0.42, 0.62), color(0.62, 0.62, 0.62), (0.02, 0.0, 1.12), 0.62)
    make_box(system, "human interaction pelvis STL substitute", (0.32, 0.36, 0.22), color(0.50, 0.50, 0.50), (0.0, 0.0, 0.72), 0.55)
    make_marker(system, "human interaction head STL substitute", 0.105, color(0.72, 0.72, 0.70), 0.65).SetPos(vec(0.03, 0.0, 1.55))
    for side, y in (("left", -0.10), ("right", 0.10)):
        make_box(system, f"human interaction {side} leg", (0.09, 0.075, 0.58), color(0.54, 0.54, 0.54), (-0.02, y, 0.36), 0.50)
        make_box(system, f"human interaction {side} foot", (0.19, 0.08, 0.05), color(0.42, 0.42, 0.42), (0.06, y, 0.045), 0.58)


def build_human_robot_interaction_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, "human robot interaction checker ground", (-0.25, 0.0), (3.0, 2.4), z=0.0, tile_count=8)
    build_human_body(system)
    human_items = {
        "upper": MutableSegment(system, "human left upper arm visible body", color(0.10, 0.25, 0.92), 10),
        "lower": MutableSegment(system, "human left lower arm plus hand visible body", color(0.05, 0.54, 0.98), 10),
        "hand": make_marker(system, "human hand contact sphere", 0.025, color(0.52, 0.26, 0.12)),
        "shoulder": make_marker(system, "human shoulder spherical joint", 0.042, color(0.95, 0.76, 0.15)),
        "elbow": make_marker(system, "human elbow revolute joint", 0.032, color(0.95, 0.56, 0.12)),
        "hand_contact": make_marker(system, "human hand contact envelope", 0.0275, color(0.92, 0.62, 0.12), 0.24),
    }
    puma = make_serial_arm(system, "human interaction PUMA560", PUMA_LINKS, PUMA_BASE, (0.0, 0.0, 0.12), base_size=(0.65, 0.65, 0.65), link_thickness=8)
    robot_contact = make_marker(system, "PUMA TCP contact envelope", 0.041, color(0.96, 0.05, 0.04), 0.28)
    gap = MutableSegment(system, "human-robot contact gap line", color(0.98, 0.05, 0.04), 4)
    system._human_robot_interaction = {"human": human_items, "puma": puma, "robot_contact": robot_contact, "gap": gap}
    update_human_robot_interaction_visuals(system)
    return system, system._human_robot_interaction


def update_human_robot_interaction_visuals(system):
    items = system._human_robot_interaction
    qh, qh_t = human_arm_state(system.GetChTime())
    shoulder, elbow, hand, _rs, _re, _rw = human_points(qh)
    human = items["human"]
    human["upper"].update(shoulder, elbow)
    human["lower"].update(elbow, hand)
    set_body_pose(human["shoulder"], shoulder)
    set_body_pose(human["elbow"], elbow)
    set_body_pose(human["hand"], hand)
    set_body_pose(human["hand_contact"], hand)
    qp, qp_t = puma_interaction_state(system.GetChTime())
    update_serial_arm(items["puma"], qp)
    tcp = items["puma"]["last_tcp"]
    set_body_pose(items["robot_contact"], tcp)
    items["gap"].update(hand, tcp)
    items["last"] = {"qh": qh, "qh_t": qh_t, "qp": qp, "qp_t": qp_t, "hand": hand, "tcp": tcp}


def print_human_robot_interaction_state(system):
    data = system._human_robot_interaction["last"]
    gap = float(np.linalg.norm(data["hand"] - data["tcp"]))
    print(
        f"t={system.GetChTime():.4f} source=humanRobotInteraction.py "
        f"humanKT=7 PUMA560=6 contactSpheres=2 gap={gap:.6f} "
        f"hand=({data['hand'][0]:+.5f},{data['hand'][1]:+.5f},{data['hand'][2]:+.5f}) "
        f"tcp=({data['tcp'][0]:+.5f},{data['tcp'][1]:+.5f},{data['tcp'][2]:+.5f}) "
        "external STL body replaced by visible articulated-dummy primitives"
    )


DD_WHEEL_RADIUS = 0.04
DD_WHEEL_DISTANCE = 0.4
DD_PLATFORM_HEIGHT = 0.1
DD_PLATFORM_RADIUS = 0.22
DD_MAX_WHEEL_SPEED = 2.0 * math.pi
DD_TARGET = np.array([1.2, 1.2], dtype=float)
DD_START = np.array([-1.35, -1.0], dtype=float)
DD_REPLAY_END = 4.0


def dd_pose(time):
    u = min(max(time / DD_REPLAY_END, 0.0), 1.0)
    s = smoothstep(u)
    st = smoothstep_t(u) / DD_REPLAY_END
    xy = DD_START + s * (DD_TARGET - DD_START)
    xy_t = st * (DD_TARGET - DD_START)
    target_angle = math.atan2((DD_TARGET - DD_START)[1], (DD_TARGET - DD_START)[0]) - 0.5 * math.pi
    phi = -0.8 + smoothstep(min(time / 1.2, 1.0)) * (target_angle + 0.8)
    phi_t = 0.0
    if 0.0 < time < 1.2:
        phi_t = smoothstep_t(time / 1.2) * (target_angle + 0.8) / 1.2
    return np.array([xy[0], xy[1], phi]), np.array([xy_t[0], xy_t[1], phi_t])


def dd_wheel_velocities(forward_velocity, yaw_rate):
    v_left = -forward_velocity / DD_WHEEL_RADIUS + yaw_rate * DD_WHEEL_DISTANCE * 0.5 / DD_WHEEL_RADIUS
    v_right = -forward_velocity / DD_WHEEL_RADIUS - yaw_rate * DD_WHEEL_DISTANCE * 0.5 / DD_WHEEL_RADIUS
    return np.array([v_left, v_right], dtype=float)


def dd_reward(pose, pose_t):
    direction_to_target = DD_TARGET - pose[:2]
    dist = float(np.linalg.norm(direction_to_target))
    if dist > 1e-12:
        v0 = direction_to_target / dist
    else:
        v0 = np.array([0.0, 1.0])
    v_dir = rot2(pose[2]) @ np.array([0.0, 1.0])
    local_speed = rot2(pose[2]).T @ pose_t[:2]
    reward = 1.0 - 0.5 * abs(float(local_speed[1])) - 0.5 * float(np.linalg.norm(v_dir - v0))
    return max(0.0, reward), dist, local_speed


def build_reinforcement_learning_robot_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, "RL differential drive checker ground", (0.0, 0.0), (4.0, 4.0), z=0.0, tile_count=8)
    platform = make_cylinder(system, "RL differential-drive platform cylinder", chrono.ChAxis_Z, DD_PLATFORM_RADIUS, DD_PLATFORM_HEIGHT, color(0.12, 0.36, 0.72), (0, 0, 0.08), 0.78)
    nose = make_cylinder(system, "RL platform heading cue", chrono.ChAxis_Y, 0.022, 0.22, color(0.82, 0.82, 0.84), (0, 0, 0.15), 1.0)
    wheels = [
        make_cylinder(system, "RL left controlled wheel", chrono.ChAxis_X, DD_WHEEL_RADIUS, 0.012, color(0.90, 0.12, 0.08)),
        make_cylinder(system, "RL right controlled wheel", chrono.ChAxis_X, DD_WHEEL_RADIUS, 0.012, color(0.90, 0.12, 0.08)),
    ]
    target = make_marker(system, "RL target destination from observation", 0.055, color(0.95, 0.05, 0.04))
    target.SetPos(vec(DD_TARGET[0], DD_TARGET[1], 0.06))
    path = MutablePolyline(system, "RL heuristic policy path", color(0.05, 0.44, 0.92), 4)
    path.update([np.r_[DD_START + (DD_TARGET - DD_START) * smoothstep(i / 80), 0.025] for i in range(81)])
    bars = [
        MutableSegment(system, "RL left wheel action bar", color(0.92, 0.52, 0.08), 5),
        MutableSegment(system, "RL right wheel action bar", color(0.92, 0.52, 0.08), 5),
        MutableSegment(system, "RL reward bar", color(0.12, 0.70, 0.20), 5),
    ]
    system._reinforcement_learning_robot = {
        "platform": platform,
        "nose": nose,
        "wheels": wheels,
        "target": target,
        "path": path,
        "bars": bars,
    }
    update_reinforcement_learning_robot_visuals(system)
    return system, system._reinforcement_learning_robot


def update_reinforcement_learning_robot_visuals(system):
    items = system._reinforcement_learning_robot
    time = system.GetChTime()
    pose, pose_t = dd_pose(time)
    yaw = chrono.QuatFromAngleZ(float(pose[2]))
    center = np.array([pose[0], pose[1], DD_WHEEL_RADIUS + 0.5 * DD_PLATFORM_HEIGHT])
    items["platform"].SetPos(vec(center))
    items["platform"].SetRot(yaw)
    items["nose"].SetPos(vec(center + np.r_[rot2(pose[2]) @ np.array([0.0, DD_PLATFORM_RADIUS * 0.55]), 0.045]))
    items["nose"].SetRot(yaw * chrono.QuatFromAngleX(0.5 * math.pi))
    forward_speed = float((rot2(pose[2]).T @ pose_t[:2])[1])
    wheel_speeds = dd_wheel_velocities(forward_speed, pose_t[2])
    for index, side in enumerate((-1.0, 1.0)):
        local = np.array([side * DD_WHEEL_DISTANCE * 0.5, 0.0])
        xy = pose[:2] + rot2(pose[2]) @ local
        wheel = items["wheels"][index]
        wheel.SetPos(vec(xy[0], xy[1], DD_WHEEL_RADIUS))
        wheel.SetRot(yaw * chrono.QuatFromAngleX(float(wheel_speeds[index] * time)))
        wheel.UpdateVisualModel()
    for body in (items["platform"], items["nose"]):
        body.UpdateVisualModel()
    reward, dist, local_speed = dd_reward(pose, pose_t)
    bar_x = -1.85
    for index, value in enumerate((wheel_speeds[0] / DD_MAX_WHEEL_SPEED, wheel_speeds[1] / DD_MAX_WHEEL_SPEED, reward)):
        base = np.array([bar_x + 0.18 * index, 1.72, 0.03])
        top = base + np.array([0.0, 0.0, 0.28 * max(-1.0, min(1.0, float(value)))])
        items["bars"][index].update(base, top)
    items["last"] = {"pose": pose, "pose_t": pose_t, "wheel_speeds": wheel_speeds, "reward": reward, "dist": dist, "local_speed": local_speed}


def print_reinforcement_learning_robot_state(system):
    data = system._reinforcement_learning_robot["last"]
    pose = data["pose"]
    print(
        f"t={system.GetChTime():.4f} source=reinforcementLearningRobot.py "
        f"RobotEnv states=8 actions=2 actionRange=+-2pi target=({DD_TARGET[0]:+.2f},{DD_TARGET[1]:+.2f}) "
        f"pose=({pose[0]:+.5f},{pose[1]:+.5f},{pose[2]:+.5f}) "
        f"wheelOmega=({data['wheel_speeds'][0]:+.5f},{data['wheel_speeds'][1]:+.5f}) "
        f"rewardProxy={data['reward']:.6f} distance={data['dist']:.6f}"
    )


SPOT_LEGS_INIT = np.array([0.0, 36.0 * math.pi / 180.0, -54.0 * math.pi / 180.0])
SPOT_Z_CONTACT_SOURCE = -0.7 + 0.01214 + 6.45586829e-02
SPOT_Z_OFFSET = -SPOT_Z_CONTACT_SOURCE
SPOT_BODY_SIZE = np.array([0.70, 0.40, 0.12])
SPOT_UPPER = 0.25
SPOT_LOWER = 0.37
SPOT_HIPS = {
    "fl": np.array([0.26, 0.16, -0.03]),
    "fr": np.array([0.26, -0.16, -0.03]),
    "hl": np.array([-0.26, 0.16, -0.03]),
    "hr": np.array([-0.26, -0.16, -0.03]),
}
SPOT_Q0 = np.r_[np.zeros(6), SPOT_LEGS_INIT, SPOT_LEGS_INIT, SPOT_LEGS_INIT, SPOT_LEGS_INIT]
SPOT_Q1 = np.array([0, 0, 0, 0, 0, 0, 0.25 * math.pi, 0, 0, -0.25 * math.pi, 0, 0, 0.25 * math.pi, 0, 0, -0.25 * math.pi, 0, 0], dtype=float)
SPOT_Q2 = np.array([0, 0, 0, 0, 0, 0, 0, 0.5 * math.pi, -0.9 * math.pi, 0, 0.5 * math.pi, -0.9 * math.pi, 0, 0.5 * math.pi, -0.9 * math.pi, 0, 0.5 * math.pi, -0.9 * math.pi], dtype=float)
SPOT_Q3 = SPOT_Q2.copy()
SPOT_Q4 = np.array([0, 0, 0, 0, 0, 0, 0, 0.5 * math.pi, -0.9 * math.pi, 0, 0.5 * math.pi, -0.9 * math.pi, 0, 0, 0, 0, 0, 0], dtype=float)
SPOT_MODEL_POINTS = (SPOT_Q0, SPOT_Q0, SPOT_Q1, SPOT_Q2, SPOT_Q0, SPOT_Q3, SPOT_Q4, SPOT_Q0)
SPOT_MODEL_DURATIONS = (0.3, 1.0, 1.0, 0.7, 0.7, 0.7, 0.4)
SPOT_MODEL_END = 6.0


def spot_model_state(time):
    return piecewise_profile(SPOT_MODEL_POINTS, SPOT_MODEL_DURATIONS, time)


def spot_rl_state(time):
    tmax = 5.0
    progress = smoothstep(min(max(time / tmax, 0.0), 1.0))
    q = SPOT_Q0.copy()
    q[0] = 2.4 * progress
    q[2] = 0.03 * math.sin(2.0 * math.pi * time / 1.2)
    q[4] = 0.06 * math.sin(2.0 * math.pi * time / 2.0)
    phases = (0.0, math.pi, math.pi, 0.0)
    max_angle = 25.0 * math.pi / 180.0
    for leg_index, phase in enumerate(phases):
        base = 6 + 3 * leg_index
        q[base] = (0.22 if leg_index in (0, 2) else -0.22) * max_angle * math.sin(2.0 * math.pi * time / 0.8 + phase)
        q[base + 1] = SPOT_LEGS_INIT[1] + 0.65 * max_angle * math.sin(2.0 * math.pi * time / 0.8 + phase)
        q[base + 2] = SPOT_LEGS_INIT[2] - 0.85 * max_angle * max(0.0, math.sin(2.0 * math.pi * time / 0.8 + phase))
    qd = np.zeros_like(q)
    h = 1.0e-4
    if time > h:
        q_prev, _ = spot_rl_state_no_velocity(time - h)
        qd = (q - q_prev) / h
    return q, qd


def spot_rl_state_no_velocity(time):
    tmax = 5.0
    progress = smoothstep(min(max(time / tmax, 0.0), 1.0))
    q = SPOT_Q0.copy()
    q[0] = 2.4 * progress
    q[2] = 0.03 * math.sin(2.0 * math.pi * time / 1.2)
    q[4] = 0.06 * math.sin(2.0 * math.pi * time / 2.0)
    phases = (0.0, math.pi, math.pi, 0.0)
    max_angle = 25.0 * math.pi / 180.0
    for leg_index, phase in enumerate(phases):
        base = 6 + 3 * leg_index
        q[base] = (0.22 if leg_index in (0, 2) else -0.22) * max_angle * math.sin(2.0 * math.pi * time / 0.8 + phase)
        q[base + 1] = SPOT_LEGS_INIT[1] + 0.65 * max_angle * math.sin(2.0 * math.pi * time / 0.8 + phase)
        q[base + 2] = SPOT_LEGS_INIT[2] - 0.85 * max_angle * max(0.0, math.sin(2.0 * math.pi * time / 0.8 + phase))
    return q, np.zeros_like(q)


def spot_leg_points(body_pos, yaw, hip_name, leg_q):
    sign_y = 1.0 if hip_name.endswith("l") else -1.0
    rot = rz3(yaw)
    hip = body_pos + rot @ SPOT_HIPS[hip_name]
    side, hip_pitch, knee_pitch = leg_q
    lateral = np.array([0.0, sign_y * 0.13 * math.sin(side), 0.0])
    upper_dir = np.array([0.30 * math.sin(hip_pitch), 0.0, -math.cos(hip_pitch)])
    upper_dir = upper_dir / max(np.linalg.norm(upper_dir), 1.0e-12)
    knee = hip + rot @ (lateral + SPOT_UPPER * upper_dir)
    lower_dir = np.array([0.26 * math.sin(hip_pitch + knee_pitch), 0.0, -math.cos(hip_pitch + knee_pitch)])
    lower_dir = lower_dir / max(np.linalg.norm(lower_dir), 1.0e-12)
    foot = knee + rot @ (lateral * 0.25 + SPOT_LOWER * lower_dir)
    return hip, knee, foot


def build_spot_items(system, prefix, with_target=False, with_bars=False):
    add_grid_ground(system, prefix + " ground", (1.4 if with_target else 0.0, 0.0), (5.2 if with_target else 2.6, 2.0), z=0.0, tile_count=10)
    body = make_box(system, prefix + " floating base/URDF body", SPOT_BODY_SIZE, color(0.86, 0.08, 0.06), (0, 0, SPOT_Z_OFFSET), 0.88)
    legs = {}
    for name in ("fl", "fr", "hl", "hr"):
        legs[name] = {
            "upper": MutableSegment(system, prefix + f" {name} hip-to-knee visible link", color(0.16, 0.18, 0.22), 9),
            "lower": MutableSegment(system, prefix + f" {name} lower-leg visible link", color(0.08, 0.36, 0.82), 9),
            "hip": make_marker(system, prefix + f" {name} hip marker", 0.030, color(0.94, 0.54, 0.08)),
            "knee": make_marker(system, prefix + f" {name} knee marker", 0.026, color(0.05, 0.05, 0.05)),
            "foot": make_marker(system, prefix + f" {name} leg contact sphere", 0.050, color(0.04, 0.04, 0.04)),
        }
    items = {"body": body, "legs": legs}
    if with_target:
        target = make_marker(system, prefix + " RL target sphere", 0.10, color(0.95, 0.06, 0.04), 0.42)
        target.SetPos(vec(4.0, 0.0, 0.10))
        items["target"] = target
        items["body_path"] = MutableSegment(system, prefix + " target progress trace", color(0.04, 0.42, 0.95), 4)
    if with_bars:
        items["bars"] = [MutableSegment(system, prefix + f" action bar {i:02d}", color(0.90, 0.52, 0.08), 3) for i in range(12)]
    return items


def update_spot_items(system, items, q, qd=None, rl=False):
    body_pos = np.array([q[0], q[1], SPOT_Z_OFFSET + q[2]])
    yaw = float(q[5])
    body_rot = chrono.QuatFromAngleZ(yaw) * chrono.QuatFromAngleY(float(q[4])) * chrono.QuatFromAngleX(float(q[3]))
    items["body"].SetPos(vec(body_pos))
    items["body"].SetRot(body_rot)
    items["body"].UpdateVisualModel()
    feet = {}
    for leg_index, name in enumerate(("fl", "fr", "hl", "hr")):
        base = 6 + 3 * leg_index
        hip, knee, foot = spot_leg_points(body_pos, yaw, name, q[base : base + 3])
        leg = items["legs"][name]
        leg["upper"].update(hip, knee)
        leg["lower"].update(knee, foot)
        set_body_pose(leg["hip"], hip)
        set_body_pose(leg["knee"], knee)
        set_body_pose(leg["foot"], foot)
        feet[name] = foot
    if "body_path" in items:
        items["body_path"].update((0.0, 0.0, 0.035), (body_pos[0], body_pos[1], 0.035))
    if "bars" in items and qd is not None:
        actions = q[6:18] - np.tile(SPOT_LEGS_INIT, 4)
        for i, bar in enumerate(items["bars"]):
            base = np.array([0.35 + i * 0.055, -0.88, 0.04])
            top = base + np.array([0.0, 0.0, 0.36 * max(-1.0, min(1.0, actions[i] / (25.0 * math.pi / 180.0)))])
            bar.update(base, top)
    items["last"] = {"q": q, "qd": qd if qd is not None else np.zeros_like(q), "feet": feet, "body_pos": body_pos}


def build_spot_model_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    items = build_spot_items(system, "SpotModel", with_target=False, with_bars=False)
    system._spot_model = items
    update_spot_model_visuals(system)
    return system, items


def update_spot_model_visuals(system):
    q, qd = spot_model_state(system.GetChTime())
    update_spot_items(system, system._spot_model, q, qd, rl=False)


def print_spot_model_state(system):
    data = system._spot_model["last"]
    foot_min = min(float(point[2]) for point in data["feet"].values())
    print(
        f"t={system.GetChTime():.4f} source=FurtherExamples/spotModel.py "
        f"floatingBase=6 actuatedJoints=12 legsInit=({SPOT_LEGS_INIT[0]:+.4f},{SPOT_LEGS_INIT[1]:+.4f},{SPOT_LEGS_INIT[2]:+.4f}) "
        f"zContactSource={SPOT_Z_CONTACT_SOURCE:+.6f} q_sum={float(np.sum(data['q'])):+.6f} minFootZ={foot_min:+.6f}"
    )


def build_spot_reinforcement_learning_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    items = build_spot_items(system, "SpotRL", with_target=True, with_bars=True)
    system._spot_reinforcement_learning = items
    update_spot_reinforcement_learning_visuals(system)
    return system, items


def spot_rl_reward(q, qd):
    target = np.array([4.0, 0.0])
    pos = q[:3]
    vel = qd[:3]
    dist = float(np.linalg.norm(target - pos[:2]))
    vel_target = float(np.dot(vel[:2], (target - pos[:2]) / max(dist, 1.0e-12)))
    reward = vel_target
    if dist < 1.0:
        reward += 1.0 - dist
    angle_penalty = abs(q[3]) + abs(q[4]) + 0.5 * abs(q[5]) + max(0.0, 0.70 - (SPOT_Z_OFFSET + q[2]))
    reward -= 0.5 * angle_penalty
    return reward, dist, vel_target


def update_spot_reinforcement_learning_visuals(system):
    q, qd = spot_rl_state(system.GetChTime())
    update_spot_items(system, system._spot_reinforcement_learning, q, qd, rl=True)
    reward, dist, vel_target = spot_rl_reward(q, qd)
    system._spot_reinforcement_learning["last"]["reward"] = reward
    system._spot_reinforcement_learning["last"]["dist"] = dist
    system._spot_reinforcement_learning["last"]["vel_target"] = vel_target


def print_spot_reinforcement_learning_state(system):
    data = system._spot_reinforcement_learning["last"]
    print(
        f"t={system.GetChTime():.4f} source=FurtherExamples/spotReinforcementLearning.py "
        f"SpotEnv stateSize=40 actionSize=12 stepUpdate=0.02 episodeMaxLen=300 "
        f"body=({data['body_pos'][0]:+.5f},{data['body_pos'][1]:+.5f},{data['body_pos'][2]:+.5f}) "
        f"target=(+4.00,+0.00) dist={data['dist']:.6f} velTarget={data['vel_target']:+.6f} rewardProxy={data['reward']:+.6f}"
    )


def simulate(builder, updater, duration, step):
    system, items = builder()
    while system.GetChTime() < duration - 1.0e-14:
        updater(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    updater(system)
    return system, items


def run_visual(builder, updater, printer, title, camera, target, duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = builder()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1100, 760)
    vis.SetWindowTitle(title)
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(vec(camera), vec(target))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        updater(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if system.GetChTime() >= next_log:
            printer(system)
            next_log += max(0.25, 0.2 * duration)
    updater(system)


def generic_main(builder, updater, printer, title, default_duration, default_step, camera, target, intro):
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=default_duration)
    parser.add_argument("--step", type=float, default=default_step)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()
    print(intro)
    if args.no_vis:
        system, _items = simulate(builder, updater, args.duration, args.step)
        printer(system)
    else:
        run_visual(builder, updater, printer, title, camera, target, args.duration, args.step)

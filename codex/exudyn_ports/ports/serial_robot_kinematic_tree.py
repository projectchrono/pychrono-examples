import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN Examples/serialRobotKinematicTree.py:
# a six-axis serial robot built from standard DH parameters and driven by a
# four-segment joint-space trajectory.  EXUDYN solves the PD-controlled
# ObjectKinematicTree dynamically; this PyChrono port keeps the source DH
# geometry, commanded trajectory, visible base/link/tool bodies, COM markers,
# and moving joint-axis markers as a robust kinematic replay.

STEP = 1.0e-3
END_TIME = 1.25
SEGMENT_TIME = 0.25
JOINT_RADIUS = 0.055
JOINT_AXIS_LENGTH = 0.18

LINK_COLORS = (
    (0.78, 0.22, 0.18),
    (0.10, 0.46, 0.84),
    (0.14, 0.62, 0.28),
    (0.86, 0.56, 0.12),
    (0.48, 0.34, 0.78),
    (0.10, 0.60, 0.62),
)

LINKS = (
    {"stdDH": [0.0, 0.0, 0.0, 0.5 * math.pi], "mass": 20.0, "COM": [0.0, 0.0, 0.0]},
    {"stdDH": [0.0, 0.0, 0.4318, 0.0], "mass": 17.4, "COM": [-0.3638, 0.006, 0.2275]},
    {"stdDH": [0.0, 0.15, 0.0203, -0.5 * math.pi], "mass": 4.8, "COM": [-0.0203, -0.0141, 0.07]},
    {"stdDH": [0.0, 0.4318, 0.0, 0.5 * math.pi], "mass": 0.82, "COM": [0.0, 0.019, 0.0]},
    {"stdDH": [0.0, 0.0, 0.0, -0.5 * math.pi], "mass": 0.34, "COM": [0.0, 0.0, 0.0]},
    {"stdDH": [0.0, 0.0, 0.0, 0.0], "mass": 0.09, "COM": [0.0, 0.0, 0.032]},
)

Q0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
Q1 = [0.0, math.pi / 8.0, 0.5 * math.pi, 0.0, math.pi / 8.0, 0.0]
Q2 = [0.8 * math.pi, -0.8 * math.pi, -0.5 * math.pi, 0.75 * math.pi, -0.4 * math.pi, 0.4 * math.pi]
Q3 = [0.5 * math.pi, 0.0, -0.25 * math.pi, 0.0, 0.0, 0.0]
TRAJECTORY_POINTS = (Q0, Q3, Q1, Q2, Q0)


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vector(values):
    return chrono.ChVector3d(values[0], values[1], values[2])


def vadd(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def vscale(a, factor):
    return [factor * a[0], factor * a[1], factor * a[2]]


def matmul3(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matvec3(a, v):
    return [sum(a[i][j] * v[j] for j in range(3)) for i in range(3)]


def identity3():
    return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]


def ht_identity():
    return [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def ht_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def ht_rot_x(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return [[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]]


def ht_rot_z(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def ht_trans_x(value):
    return [[1, 0, 0, value], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def ht_trans_z(value):
    return [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, value], [0, 0, 0, 1]]


def ht_translate(values):
    return [[1, 0, 0, values[0]], [0, 1, 0, values[1]], [0, 0, 1, values[2]], [0, 0, 0, 1]]


def std_dh(theta, d, a, alpha):
    return ht_mul(ht_mul(ht_mul(ht_rot_z(theta), ht_trans_z(d)), ht_trans_x(a)), ht_rot_x(alpha))


def ht_rotation(ht):
    return [[ht[i][j] for j in range(3)] for i in range(3)]


def ht_translation(ht):
    return [ht[0][3], ht[1][3], ht[2][3]]


def transform_point(ht, point):
    return [
        ht[0][0] * point[0] + ht[0][1] * point[1] + ht[0][2] * point[2] + ht[0][3],
        ht[1][0] * point[0] + ht[1][1] * point[1] + ht[1][2] * point[2] + ht[1][3],
        ht[2][0] * point[0] + ht[2][1] * point[1] + ht[2][2] * point[2] + ht[2][3],
    ]


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


def interpolate(q_start, q_end, s, sd):
    q = []
    qd = []
    for a, b in zip(q_start, q_end):
        delta = b - a
        q.append(a + delta * s)
        qd.append(delta * sd)
    return q, qd


def trajectory(t):
    segment_count = len(TRAJECTORY_POINTS) - 1
    if t >= segment_count * SEGMENT_TIME:
        return list(TRAJECTORY_POINTS[-1]), [0.0] * 6
    index = max(0, min(segment_count - 1, int(t / SEGMENT_TIME)))
    tau = t - index * SEGMENT_TIME
    s, sd = profile_constant_acceleration(tau, SEGMENT_TIME)
    return interpolate(TRAJECTORY_POINTS[index], TRAJECTORY_POINTS[index + 1], s, sd)


def robot_poses(q):
    frames = []
    joint_frames = []
    ht = ht_identity()
    for index, link in enumerate(LINKS):
        joint_ht = ht_mul(ht, ht_rot_z(q[index]))
        joint_frames.append(joint_ht)
        local = std_dh(*link["stdDH"])
        ht = ht_mul(joint_ht, local)
        frames.append(ht)
    tool = ht_mul(ht, ht_translate([0.0, 0.0, 0.1]))
    return joint_frames, frames, tool


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


def make_marker(system, name, radius, tint):
    body = chrono.ChBodyEasySphere(radius, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def make_axis(system, name, tint):
    body = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.010, JOINT_AXIS_LENGTH, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def add_static_segment(system, name, start, end, tint, thickness=3):
    segment = MutableSegment(system, name, tint, thickness)
    segment.update(start, end)
    return segment


def make_base(system):
    base = chrono.ChBodyEasyBox(0.12, 0.12, 0.10, 1000, True, False)
    base.SetName("serial robot source base")
    base.SetFixed(True)
    base.EnableCollision(False)
    base.SetPos(chrono.ChVector3d(0, 0, -0.15))
    base.GetVisualShape(0).SetColor(color(0.48, 0.48, 0.50))
    system.AddBody(base)

    add_static_segment(system, "serial robot base x axis", [0, 0, 0], [0.5, 0, 0], color(0.92, 0.10, 0.08), 3)
    add_static_segment(system, "serial robot base y axis", [0, 0, 0], [0, 0.5, 0], color(0.12, 0.62, 0.18), 3)
    add_static_segment(system, "serial robot base z axis", [0, 0, 0], [0, 0, 0.5], color(0.10, 0.24, 0.88), 3)
    return base


def make_tool(system):
    tcp = make_marker(system, "serial robot TCP marker", 0.025, color(0.92, 0.10, 0.08))
    left = MutableSegment(system, "serial robot left gripper finger", color(0.55, 0.55, 0.57), 5)
    right = MutableSegment(system, "serial robot right gripper finger", color(0.55, 0.55, 0.57), 5)
    stem = MutableSegment(system, "serial robot tool stem", color(0.92, 0.10, 0.08), 4)
    return {"tcp": tcp, "left": left, "right": right, "stem": stem}


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    base = make_base(system)
    link_segments = []
    com_segments = []
    joint_markers = []
    com_markers = []
    axes = []
    for index, rgb in enumerate(LINK_COLORS):
        tint = color(*rgb)
        link_segments.append(MutableSegment(system, f"serial robot link {index + 1} DH span", tint, 7))
        com_segments.append(MutableSegment(system, f"serial robot link {index + 1} COM arm", tint, 4))
        joint_markers.append(make_marker(system, f"serial robot joint {index + 1}", JOINT_RADIUS, tint))
        com_markers.append(make_marker(system, f"serial robot link {index + 1} COM", 0.020 + 0.002 * index, color(0.04, 0.04, 0.045)))
        axes.append(make_axis(system, f"serial robot visible joint {index + 1} z axis", color(0.02, 0.02, 0.025)))

    tool = make_tool(system)
    items = {
        "base": base,
        "link_segments": link_segments,
        "com_segments": com_segments,
        "joint_markers": joint_markers,
        "com_markers": com_markers,
        "axes": axes,
        "tool": tool,
    }
    system._serial_robot_kinematic_tree_items = items
    update_visuals(system)
    return system, items


def set_body_pose(body, position, rotation):
    body.SetPos(vector(position))
    body.SetRot(quat_from_matrix(rotation))


def update_visuals(system):
    items = system._serial_robot_kinematic_tree_items
    q, _ = trajectory(system.GetChTime())
    joint_frames, link_frames, tool_frame = robot_poses(q)
    previous_origin = [0.0, 0.0, 0.0]

    for index, link in enumerate(LINKS):
        joint_origin = ht_translation(joint_frames[index])
        next_origin = ht_translation(link_frames[index])
        com = transform_point(link_frames[index], link["COM"])
        joint_rot = ht_rotation(joint_frames[index])

        items["link_segments"][index].update(previous_origin, next_origin)
        items["com_segments"][index].update(joint_origin, com)
        set_body_pose(items["joint_markers"][index], joint_origin, joint_rot)
        set_body_pose(items["com_markers"][index], com, identity3())
        set_body_pose(items["axes"][index], joint_origin, joint_rot)
        previous_origin = next_origin

    tool = items["tool"]
    tcp = ht_translation(tool_frame)
    tool_rot = ht_rotation(tool_frame)
    set_body_pose(tool["tcp"], tcp, tool_rot)

    stem_start = transform_point(link_frames[-1], [0.0, 0.0, 0.0])
    stem_end = tcp
    tool["stem"].update(stem_start, stem_end)
    tool["left"].update(transform_point(tool_frame, [0.0, 0.03, 0.02]), transform_point(tool_frame, [0.0, 0.03, 0.08]))
    tool["right"].update(transform_point(tool_frame, [0.0, -0.03, 0.02]), transform_point(tool_frame, [0.0, -0.03, 0.08]))


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
    vis.SetWindowSize(1100, 760)
    vis.SetWindowTitle("EXUDYN port: serialRobotKinematicTree.py")
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
            next_log += 0.25


def print_state(system):
    q, qd = trajectory(system.GetChTime())
    _, _, tool = robot_poses(q)
    tcp = ht_translation(tool)
    q_text = ",".join(f"{value:+.4f}" for value in q)
    print(
        f"t={system.GetChTime():6.3f}  "
        f"q=[{q_text}]  q_sum={sum(q):+.9f}  qd_norm={math.sqrt(sum(v*v for v in qd)):.6f}  "
        f"tcp=({tcp[0]:+.6f},{tcp[1]:+.6f},{tcp[2]:+.6f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: serialRobotKinematicTree.py -> PyChrono DH robot trajectory replay")
    if args.no_vis:
        system, _ = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

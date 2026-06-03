import argparse
import math

import numpy as np
import pychrono.core as chrono


# Reproduces EXUDYN Examples/InverseKinematicsNumericalExample.py:
# a UR5 serial manipulator solved with InverseKinematicsNumerical for a target
# pose T3 = [RotX(pi), t=[0.4526,-0.1488,0.5275]].  This PyChrono port keeps
# the source UR5 standard-DH model and solves the same target with a local
# damped least-squares numerical IK iteration, then renders the solved robot,
# target frame, initial TCP, and iteration path.

STEP = 1.0e-3
END_TIME = 0.6
TARGET_POS = np.array([0.4526, -0.1488, 0.5275], dtype=float)
TARGET_ROT = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=float)
INITIAL_Q = np.array([0.0, -0.25 * math.pi, -0.25 * math.pi, -0.25 * math.pi, 0.25 * math.pi, 0.5 * math.pi], dtype=float)

UR5_LINKS = (
    {"stdDH": [0.0, 0.089459, 0.0, 0.5 * math.pi], "COM": [0.0, -0.02561, 0.00193]},
    {"stdDH": [0.0, 0.0, -0.4250, 0.0], "COM": [0.2125, 0.0, 0.11336]},
    {"stdDH": [0.0, 0.0, -0.39225, 0.0], "COM": [0.150, 0.0, 0.02650]},
    {"stdDH": [0.0, 0.10915, 0.0, 0.5 * math.pi], "COM": [0.0, -0.00180, 0.016340]},
    {"stdDH": [0.0, 0.09465, 0.0, -0.5 * math.pi], "COM": [0.0, -0.00180, 0.016340]},
    {"stdDH": [0.0, 0.0823, 0.0, 0.0], "COM": [0.0, 0.0, -0.0011590]},
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


def std_dh(theta, d, a, alpha):
    return ht_rot_z(theta) @ ht_trans_z(d) @ ht_trans_x(a) @ ht_rot_x(alpha)


def fk(q):
    frames = []
    joints = []
    ht = np.eye(4)
    for index, link in enumerate(UR5_LINKS):
        joint_ht = ht @ ht_rot_z(q[index])
        joints.append(joint_ht)
        ht = joint_ht @ std_dh(*link["stdDH"])
        frames.append(ht)
    return joints, frames, ht


def rotvec_from_matrix(rot):
    value = max(-1.0, min(1.0, 0.5 * (np.trace(rot) - 1.0)))
    angle = math.acos(value)
    if angle < 1e-10:
        return np.zeros(3)
    skew = np.array([rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1]])
    return 0.5 * angle / math.sin(angle) * skew


def pose_error(q):
    _, _, tool = fk(q)
    pos_error = TARGET_POS - tool[:3, 3]
    rot_error = rotvec_from_matrix(TARGET_ROT @ tool[:3, :3].T)
    return np.concatenate((pos_error, rot_error))


def solve_ik(q0):
    q = q0.astype(float).copy()
    history = []
    damping = 4.0e-3
    eps = 1.0e-6
    success = False
    for _ in range(80):
        error = pose_error(q)
        _, _, tool = fk(q)
        history.append(tool[:3, 3].copy())
        if np.linalg.norm(error) < 2.0e-6:
            success = True
            break
        jac = np.zeros((6, len(q)))
        for col in range(len(q)):
            qp = q.copy()
            qp[col] += eps
            jac[:, col] = (pose_error(qp) - error) / eps
        lhs = jac.T @ jac + (damping * damping) * np.eye(len(q))
        rhs = -jac.T @ error
        delta = np.linalg.solve(lhs, rhs)
        norm = np.linalg.norm(delta)
        if norm > 0.35:
            delta *= 0.35 / norm
        q += delta
    _, _, tool = fk(q)
    history.append(tool[:3, 3].copy())
    return q, success, history, pose_error(q)


IK_Q, IK_SUCCESS, IK_HISTORY, IK_ERROR = solve_ik(INITIAL_Q)


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


def make_axis_body(system, name, tint):
    body = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.006, 0.13, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def add_frame(system, prefix, origin, rotation, length):
    colors = (color(0.92, 0.10, 0.08), color(0.10, 0.62, 0.18), color(0.10, 0.24, 0.88))
    axes = []
    for axis_index, tint in enumerate(colors):
        direction = rotation[:, axis_index]
        seg = MutableSegment(system, f"{prefix} axis {axis_index}", tint, 4)
        seg.update(origin, origin + length * direction)
        axes.append(seg)
    return axes


def make_ground(system):
    base = chrono.ChBodyEasyBox(0.12, 0.12, 0.10, 1000, True, False)
    base.SetName("UR5 IK base")
    base.SetFixed(True)
    base.EnableCollision(False)
    base.SetPos(chrono.ChVector3d(0, 0, -0.15))
    base.GetVisualShape(0).SetColor(color(0.48, 0.48, 0.50))
    system.AddBody(base)
    add_frame(system, "UR5 IK world", np.array([0.0, 0.0, 0.0]), np.eye(3), 0.28)
    add_frame(system, "UR5 IK target frame", TARGET_POS, TARGET_ROT, 0.18)
    return base


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
    make_ground(system)

    joints, frames, tool = fk(IK_Q)
    initial_tool = fk(INITIAL_Q)[2]
    link_segments = []
    com_segments = []
    joint_markers = []
    axes = []
    previous = np.array([0.0, 0.0, 0.0])
    for index, rgb in enumerate(LINK_COLORS):
        tint = color(*rgb)
        link_segments.append(MutableSegment(system, f"UR5 IK solved link {index + 1}", tint, 7))
        com_segments.append(MutableSegment(system, f"UR5 IK solved COM arm {index + 1}", tint, 3))
        joint_markers.append(make_marker(system, f"UR5 IK joint {index + 1}", 0.040, tint))
        axes.append(make_axis_body(system, f"UR5 IK visible joint {index + 1} z axis", color(0.02, 0.02, 0.025)))
        origin = joints[index][:3, 3]
        end = frames[index][:3, 3]
        com = frames[index] @ np.array(list(UR5_LINKS[index]["COM"]) + [1.0])
        link_segments[-1].update(previous, end)
        com_segments[-1].update(origin, com[:3])
        set_body_pose(joint_markers[-1], origin, joints[index][:3, :3])
        set_body_pose(axes[-1], origin, joints[index][:3, :3])
        previous = end

    tcp = make_marker(system, "UR5 IK solved TCP", 0.030, color(0.95, 0.24, 0.10))
    target = make_marker(system, "UR5 IK target point", 0.034, color(0.06, 0.06, 0.065))
    initial = make_marker(system, "UR5 IK initial TCP", 0.026, color(0.55, 0.55, 0.57))
    set_body_pose(tcp, tool[:3, 3], tool[:3, :3])
    set_body_pose(target, TARGET_POS, TARGET_ROT)
    set_body_pose(initial, initial_tool[:3, 3], initial_tool[:3, :3])

    path_segments = []
    for i in range(1, len(IK_HISTORY)):
        seg = MutableSegment(system, f"UR5 IK iteration path {i}", color(0.05, 0.05, 0.055), 2)
        seg.update(IK_HISTORY[i - 1], IK_HISTORY[i])
        path_segments.append(seg)

    items = {"tcp": tcp, "target": target, "initial": initial, "path_segments": path_segments}
    system._inverse_kinematics_numerical_items = items
    return system, items


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, items


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: InverseKinematicsNumericalExample.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.2, -1.45, 0.95), chrono.ChVector3d(0.05, -0.05, 0.20))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system)
            next_log += 0.25


def print_state(system):
    _, _, tool = fk(IK_Q)
    pos_error = np.linalg.norm(TARGET_POS - tool[:3, 3])
    rot_error = np.linalg.norm(rotvec_from_matrix(TARGET_ROT @ tool[:3, :3].T))
    q_text = ",".join(f"{v:+.4f}" for v in IK_Q)
    print(
        f"t={system.GetChTime():6.3f}  success={IK_SUCCESS}  "
        f"q=[{q_text}]  q_sum={float(np.sum(IK_Q)):+.9f}  "
        f"pos_error={pos_error:.3e}  rot_error={rot_error:.3e}  iterations={len(IK_HISTORY)}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: InverseKinematicsNumericalExample.py -> PyChrono UR5 numerical IK")
    if args.no_vis:
        system, _ = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

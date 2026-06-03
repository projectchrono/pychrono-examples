import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN TestModels/createKinematicTreeTest.py:
# two ObjectKinematicTree systems created with the newer TreeLink helper.  The
# first is a two-link Rz/Ry chain with PD control; the second is a seven-link
# branched tree with one prismatic coordinate and smooth joint-position
# offsets.  PyChrono does not provide the same ObjectKinematicTree helper, so
# this port visualizes the same source tree topology and commanded joint motion
# with explicit visible Chrono bodies and moving joint-frame markers.

LENGTH = 0.8
WIDTH = 0.05
STEP = 4.0e-3
END_TIME = 1.2


def color(r, g, b):
    return chrono.ChColor(r, g, b)


COLORS = [
    color(0.95, 0.56, 0.08),
    color(0.12, 0.42, 0.85),
    color(0.92, 0.22, 0.12),
    color(0.12, 0.42, 0.85),
    color(0.92, 0.22, 0.12),
    color(0.12, 0.42, 0.85),
    color(0.92, 0.22, 0.12),
]


def vadd(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matvec(a, v):
    return [sum(a[i][j] * v[j] for j in range(3)) for i in range(3)]


def rot_x(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def rot_y(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def rot_z(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


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


def vector(values):
    return chrono.ChVector3d(values[0], values[1], values[2])


def smooth_step(t, t0, t1, y0, y1):
    if t <= t0:
        return y0
    if t >= t1:
        return y1
    s = (t - t0) / (t1 - t0)
    s = s * s * (3.0 - 2.0 * s)
    return y0 + (y1 - y0) * s


def make_ground(system):
    ground = chrono.ChBody()
    ground.SetName("createKinematicTreeTest ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    for pos, size, tint in (
        ([1.0, 0.5, -0.05], [1.6, 0.08, 0.05], color(0.42, 0.42, 0.44)),
        ([-1.0, -0.5, -0.05], [1.5, 0.08, 0.05], color(0.12, 0.42, 0.85)),
    ):
        rail = chrono.ChVisualShapeBox(size[0], size[1], size[2])
        rail.SetColor(tint)
        ground.AddVisualShape(rail, chrono.ChFramed(vector(pos)))

    system.AddBody(ground)
    return ground


def add_link_visuals(body):
    shape = chrono.ChVisualShapeBox(LENGTH, WIDTH, WIDTH)
    shape.SetColor(color(0.12, 0.42, 0.85))
    body.AddVisualShape(shape, chrono.ChFramed(chrono.ChVector3d(0.5 * LENGTH, 0, 0)))
    for local, tint in (
        (chrono.ChVector3d(0, 0, 0), color(0.04, 0.04, 0.045)),
        (chrono.ChVector3d(0.5 * LENGTH, 0, 0), color(0.12, 0.12, 0.12)),
        (chrono.ChVector3d(LENGTH, 0, 0), color(0.95, 0.56, 0.08)),
    ):
        marker = chrono.ChVisualShapeSphere(0.040)
        marker.SetColor(tint)
        body.AddVisualShape(marker, chrono.ChFramed(local))


def make_body(system, name, tint):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    add_link_visuals(body)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def make_axis_body(system, name, tint=color(0.08, 0.08, 0.09)):
    marker = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.028, 0.32, 1000, True, False)
    marker.SetName(name)
    marker.SetFixed(True)
    marker.EnableCollision(False)
    marker.GetVisualShape(0).SetColor(tint)
    system.AddBody(marker)
    return marker


def tree0_links():
    return [
        {"parent": -1, "joint_type": "Rz", "joint_t": [0, 0, 0.5]},
        {"parent": 0, "joint_type": "Ry", "joint_t": [0.5, 0, 0]},
    ]


def tree1_links():
    return [
        {"parent": -1, "joint_type": "Px", "joint_t": [0, 0, 0.1]},
        {"parent": 0, "joint_type": "Rz", "joint_t": [-0.25, 0, 0.4]},
        {"parent": 0, "joint_type": "Rz", "joint_t": [0.25, 0, 0.4]},
        {"parent": 1, "joint_type": "Rx", "joint_t": [0, 0.5, 0]},
        {"parent": 3, "joint_type": "Rx", "joint_t": [0, 0.5, 0]},
        {"parent": 2, "joint_type": "Rx", "joint_t": [0, 0.5, 0]},
        {"parent": 5, "joint_type": "Rx", "joint_t": [0, 0.5, 0]},
    ]


def apply_joint(joint_type, base_p, base_r, q):
    if joint_type == "Px":
        return vadd(base_p, matvec(base_r, [q, 0, 0])), base_r
    if joint_type == "Rx":
        return base_p, matmul(base_r, rot_x(q))
    if joint_type == "Ry":
        return base_p, matmul(base_r, rot_y(q))
    if joint_type == "Rz":
        return base_p, matmul(base_r, rot_z(q))
    raise ValueError(joint_type)


def compute_poses(links, base_offset, q_values):
    poses = []
    base_r = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    for index, link in enumerate(links):
        if link["parent"] < 0:
            parent_p = base_offset
            parent_r = base_r
        else:
            parent = poses[link["parent"]]
            parent_p = parent["origin"]
            parent_r = parent["rotation"]
        joint_p = vadd(parent_p, matvec(parent_r, link["joint_t"]))
        origin, rotation = apply_joint(link["joint_type"], joint_p, parent_r, q_values[index])
        poses.append({"joint": joint_p, "origin": origin, "rotation": rotation, "joint_type": link["joint_type"]})
    return poses


def tree0_coordinates(t):
    # The source starts at [pi, -pi/2] and has PD control toward zero offsets.
    decay = 1.0 - smooth_step(t, 0.0, 0.45, 0.0, 1.0)
    return [math.pi * decay, -0.5 * math.pi * decay]


def tree1_offsets(t):
    return [
        smooth_step(t, 0.1, 0.2, 0, 0.25),
        smooth_step(t, 0.2, 0.3, 0, 0.25 * math.pi),
        smooth_step(t, 0.2, 0.3, 0, -0.25 * math.pi),
        smooth_step(t, 0.4, 0.5, 0, 0.5 * math.pi),
        smooth_step(t, 0.6, 0.7, 0, 0.5 * math.pi),
        smooth_step(t, 0.8, 0.9, 0, 0.5 * math.pi),
        smooth_step(t, 1.0, 1.1, 0, 0.5 * math.pi),
    ]


def tree1_coordinates(t):
    initial = [0.1, 0.125 * math.pi, -0.125 * math.pi, 0, 0, 0, 0]
    follow = smooth_step(t, 0.0, 0.1, 0.0, 1.0)
    offsets = tree1_offsets(t)
    return [(1.0 - follow) * q0 + follow * u for q0, u in zip(initial, offsets)]


def set_body_pose(body, pose):
    body.SetPos(vector(pose["origin"]))
    body.SetRot(quat_from_matrix(pose["rotation"]))


def set_axis_pose(body, pose):
    body.SetPos(vector(pose["joint"]))
    body.SetRot(quat_from_matrix(pose["rotation"]))


def update_kinematics(system):
    items = system._create_kinematic_tree_items
    t = system.GetChTime()
    q0 = tree0_coordinates(t)
    q1 = tree1_coordinates(t)

    poses0 = compute_poses(tree0_links(), [1, 0.5, 0], q0)
    poses1 = compute_poses(tree1_links(), [-1, -0.5, 0], q1)

    for body, axis, pose in zip(items["tree0_bodies"], items["tree0_axes"], poses0):
        set_body_pose(body, pose)
        set_axis_pose(axis, pose)
    for body, axis, pose in zip(items["tree1_bodies"], items["tree1_axes"], poses1):
        set_body_pose(body, pose)
        set_axis_pose(axis, pose)

    items["tree0_q"] = q0
    items["tree1_q"] = q1
    items["tree0_poses"] = poses0
    items["tree1_poses"] = poses1


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    ground = make_ground(system)

    tree0_bodies = [make_body(system, f"CreateKinematicTree tree0 link {i + 1}", tint) for i, tint in enumerate((color(0.12, 0.42, 0.85), color(0.92, 0.22, 0.12)))]
    tree0_axes = [make_axis_body(system, f"CreateKinematicTree tree0 visible joint axis {i + 1}") for i in range(2)]
    tree1_bodies = [make_body(system, f"CreateKinematicTree branched tree link {i + 1}", COLORS[i]) for i in range(7)]
    tree1_axes = [make_axis_body(system, f"CreateKinematicTree branched tree visible joint axis {i + 1}") for i in range(7)]

    items = {
        "ground": ground,
        "tree0_bodies": tree0_bodies,
        "tree0_axes": tree0_axes,
        "tree1_bodies": tree1_bodies,
        "tree1_axes": tree1_axes,
        "tree0_q": [],
        "tree1_q": [],
        "tree0_poses": [],
        "tree1_poses": [],
    }
    system._create_kinematic_tree_items = items
    update_kinematics(system)
    return system, items


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration:
        update_kinematics(system)
        system.DoStepDynamics(step)
    update_kinematics(system)
    return system, items


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: createKinematicTreeTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.2, -5.0, 3.1), chrono.ChVector3d(0.0, 0.0, 0.55))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        update_kinematics(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, items)
            next_log += 0.25


def norm(values):
    return math.sqrt(sum(v * v for v in values))


def tip_position(poses):
    pose = poses[-1]
    return vector(vadd(pose["origin"], matvec(pose["rotation"], [LENGTH, 0, 0])))


def print_state(system, items):
    q0 = items["tree0_q"]
    q1 = items["tree1_q"]
    tip0 = tip_position(items["tree0_poses"])
    tip1 = tip_position(items["tree1_poses"])
    print(
        f"t={system.GetChTime():6.3f}  "
        f"tree0_q_norm={norm(q0):.6f}  tree1_q_norm={norm(q1):.6f}  "
        f"tree1_offsets_norm={norm(tree1_offsets(system.GetChTime())):.6f}  "
        f"tip0=({tip0.x:+.4f},{tip0.y:+.4f},{tip0.z:+.4f})  "
        f"tip1=({tip1.x:+.4f},{tip1.y:+.4f},{tip1.z:+.4f})  "
        f"bodies={len(items['tree0_bodies']) + len(items['tree1_bodies'])}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: createKinematicTreeTest.py -> PyChrono TreeLink kinematic replay")
    if args.no_vis:
        system, items = simulate(args.duration, args.step)
        print_state(system, items)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

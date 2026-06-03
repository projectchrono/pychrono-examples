import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN TestModels/kinematicTreeTest.py:
# a five-link 3D ObjectKinematicTree with RevoluteZ joints, nontrivial joint
# transformations, and gravity [0,-10,0].  PyChrono represents the tree as
# explicit visible rigid bodies connected by revolute joints whose frames use
# the same source offsets and rotated joint axes.

LENGTH = 2.0
WIDTH = 0.1
DENSITY = 1000.0
GRAVITY_Y = -10.0
N_LINKS = 5
STEP = 2.5e-3
END_TIME = 1.0
SOURCE_REFERENCE_SUM_Q = -1.309383960216414


def color(r, g, b):
    return chrono.ChColor(r, g, b)


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


def rot_xyz(angles):
    rx, ry, rz = angles
    return matmul(matmul(rot_x(rx), rot_y(ry)), rot_z(rz))


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


def joint_transform(index):
    transform = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    if index % 2 != 0:
        transform = rot_xyz([0.0, 0.25 * math.pi, 0.0])
    if index % 3 >= 1:
        transform = rot_xyz([0.5 * math.pi, 0.25 * math.pi, 0.0])
    return transform


def source_tree_poses():
    poses = []
    parent_origin = [0.0, 0.0, 0.0]
    parent_rot = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    for index in range(N_LINKS):
        offset = [0.0, 0.0, 0.0] if index == 0 else [LENGTH, 0.0, 0.0]
        joint_origin = vadd(parent_origin, matvec(parent_rot, offset))
        link_rot = matmul(parent_rot, joint_transform(index))
        com = vadd(joint_origin, matvec(link_rot, [0.5 * LENGTH, 0.0, 0.0]))
        poses.append({"joint_origin": joint_origin, "rotation": link_rot, "com": com})
        parent_origin = joint_origin
        parent_rot = link_rot
    return poses


def make_ground(system):
    ground = chrono.ChBody()
    ground.SetName("kinematicTreeTest ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    rail = chrono.ChVisualShapeBox(7.0, 0.04, 0.04)
    rail.SetColor(color(0.42, 0.42, 0.44))
    ground.AddVisualShape(rail, chrono.ChFramed(chrono.ChVector3d(2.6, -0.18, -0.30)))

    origin = chrono.ChVisualShapeSphere(0.070)
    origin.SetColor(color(0.04, 0.04, 0.045))
    ground.AddVisualShape(origin, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))

    system.AddBody(ground)
    return ground


def add_link_markers(body):
    for local, tint in (
        (chrono.ChVector3d(-0.5 * LENGTH, 0, 0), color(0.04, 0.04, 0.045)),
        (chrono.ChVector3d(0, 0, 0), color(0.12, 0.12, 0.12)),
        (chrono.ChVector3d(0.5 * LENGTH, 0, 0), color(0.95, 0.56, 0.08)),
    ):
        marker = chrono.ChVisualShapeSphere(0.050)
        marker.SetColor(tint)
        body.AddVisualShape(marker, chrono.ChFramed(local))

    axis = chrono.ChVisualShapeCylinder(0.020, 0.35)
    axis.SetColor(color(0.70, 0.70, 0.72))
    body.AddVisualShape(axis, chrono.ChFramed(chrono.ChVector3d(-0.5 * LENGTH, 0, 0), chrono.QUNIT))


def make_link(system, index, pose):
    body = chrono.ChBodyEasyBox(LENGTH, WIDTH, WIDTH, DENSITY, True, False)
    body.SetName(f"kinematicTreeTest link {index + 1}")
    body.EnableCollision(False)
    body.SetPos(vector(pose["com"]))
    body.SetRot(quat_from_matrix(pose["rotation"]))
    body.GetVisualShape(0).SetColor(color(0.12, 0.42, 0.85))
    add_link_markers(body)
    system.AddBody(body)
    return body


def add_revolute(system, name, child, parent, pose):
    frame = chrono.ChFramed(vector(pose["joint_origin"]), quat_from_matrix(pose["rotation"]))
    joint = chrono.ChLinkLockRevolute()
    joint.SetName(name)
    joint.Initialize(child, parent, frame)
    system.AddLink(joint)
    return joint


def make_axis_marker(system, name, pose):
    marker = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.035, 0.48, 1000, True, False)
    marker.SetName(name)
    marker.SetFixed(True)
    marker.EnableCollision(False)
    marker.SetPos(vector(pose["joint_origin"]))
    marker.SetRot(quat_from_matrix(pose["rotation"]))
    marker.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.09))
    system.AddBody(marker)
    return marker


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, GRAVITY_Y, 0))

    ground = make_ground(system)
    poses = source_tree_poses()
    links = [make_link(system, index, pose) for index, pose in enumerate(poses)]
    joints = []
    for index, pose in enumerate(poses):
        parent = ground if index == 0 else links[index - 1]
        joints.append(add_revolute(system, f"source rotated RevoluteZ joint {index + 1}", links[index], parent, pose))
        make_axis_marker(system, f"visible rotated joint axis {index + 1}", pose)

    items = {"ground": ground, "links": links, "joints": joints, "poses": poses}
    system._kinematic_tree_test_items = items
    return system, items


def joint_angles(items):
    return [joint.GetRelAngle() for joint in items["joints"]]


def joint_gap(items):
    error = (items["links"][0].TransformPointLocalToParent(chrono.ChVector3d(-0.5 * LENGTH, 0, 0)) - vector(items["poses"][0]["joint_origin"])).Length()
    for index in range(1, N_LINKS):
        child = items["links"][index].TransformPointLocalToParent(chrono.ChVector3d(-0.5 * LENGTH, 0, 0))
        parent = items["links"][index - 1].TransformPointLocalToParent(chrono.ChVector3d(0.5 * LENGTH, 0, 0))
        error = max(error, (child - parent).Length())
    return error


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
    vis.SetWindowTitle("EXUDYN port: kinematicTreeTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(4.6, -8.0, 6.0), chrono.ChVector3d(2.8, 0.0, 1.8))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, items)
            next_log += 0.25


def print_state(system, items):
    angles = joint_angles(items)
    total = sum(angles)
    tip = items["links"][-1].TransformPointLocalToParent(chrono.ChVector3d(0.5 * LENGTH, 0, 0))
    angle_text = ",".join(f"{a:+.4f}" for a in angles)
    print(
        f"t={system.GetChTime():6.3f}  "
        f"joint_angles=[{angle_text}]  "
        f"sum_q={total:+.9f}  source_delta={total - SOURCE_REFERENCE_SUM_Q:+.3e}  "
        f"tip=({tip.x:+.6f},{tip.y:+.6f},{tip.z:+.6f})  "
        f"joint_gap={joint_gap(items):.3e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: kinematicTreeTest.py -> PyChrono explicit rotated-axis tree")
    if args.no_vis:
        system, items = simulate(args.duration, args.step)
        print_state(system, items)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

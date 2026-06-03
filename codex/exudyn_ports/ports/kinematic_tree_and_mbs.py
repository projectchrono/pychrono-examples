import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN Examples/kinematicTreeAndMBS.py:
# a five-link 3D kinematic-tree comparison using the same source joint
# transformations for the GenericODE2/KinematicTree66 and Robot/ObjectKinematicTree
# variants.  PyChrono represents both active EXUDYN cases as explicit visible
# rigid-body chains with identical rotated RevoluteZ joint frames.

LENGTH = 2.0
WIDTH = 0.1
DENSITY = 1000.0
GRAVITY_Y = -10.0
N_LINKS = 5
STEP = 2.5e-3
END_TIME = 1.0

CASE_SPECS = (
    {
        "name": "KT T66",
        "offset": [0.0, 0.0, 0.0],
        "link_color": (0.10, 0.42, 0.86),
        "joint_color": (0.04, 0.07, 0.10),
        "rail_color": (0.10, 0.26, 0.48),
    },
    {
        "name": "KT cpp",
        "offset": [6.4, 0.0, 0.0],
        "link_color": (0.86, 0.32, 0.18),
        "joint_color": (0.16, 0.05, 0.03),
        "rail_color": (0.52, 0.18, 0.10),
    },
)


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vector(values):
    return chrono.ChVector3d(values[0], values[1], values[2])


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


def joint_transform(index):
    transform = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    if index % 2 != 0:
        transform = rot_xyz([0.0, 0.25 * math.pi, 0.0])
    if index % 3 >= 1:
        transform = rot_xyz([0.5 * math.pi, 0.25 * math.pi, 0.0])
    return transform


def source_tree_poses(base_offset):
    poses = []
    parent_origin = list(base_offset)
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
    ground.SetName("kinematicTreeAndMBS ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    for spec in CASE_SPECS:
        offset = spec["offset"]
        rail = chrono.ChVisualShapeBox(5.4, 0.035, 0.035)
        rail.SetColor(color(*spec["rail_color"]))
        ground.AddVisualShape(rail, chrono.ChFramed(vector(vadd(offset, [2.55, -0.22, -0.30]))))

        origin = chrono.ChVisualShapeSphere(0.075)
        origin.SetColor(color(*spec["joint_color"]))
        ground.AddVisualShape(origin, chrono.ChFramed(vector(offset)))

        pedestal = chrono.ChVisualShapeCylinder(0.060, 0.28)
        pedestal.SetColor(color(0.52, 0.52, 0.54))
        ground.AddVisualShape(pedestal, chrono.ChFramed(vector(vadd(offset, [0.0, -0.12, 0.0]))))

    system.AddBody(ground)
    return ground


def add_link_markers(body, spec):
    for local, tint in (
        (chrono.ChVector3d(-0.5 * LENGTH, 0, 0), color(*spec["joint_color"])),
        (chrono.ChVector3d(0, 0, 0), color(0.12, 0.12, 0.12)),
        (chrono.ChVector3d(0.5 * LENGTH, 0, 0), color(0.96, 0.64, 0.08)),
    ):
        marker = chrono.ChVisualShapeSphere(0.050)
        marker.SetColor(tint)
        body.AddVisualShape(marker, chrono.ChFramed(local))

    axis = chrono.ChVisualShapeCylinder(0.020, 0.42)
    axis.SetColor(color(0.72, 0.72, 0.74))
    body.AddVisualShape(axis, chrono.ChFramed(chrono.ChVector3d(-0.5 * LENGTH, 0, 0), chrono.QUNIT))


def make_link(system, spec, index, pose):
    body = chrono.ChBodyEasyBox(LENGTH, WIDTH, WIDTH, DENSITY, True, False)
    body.SetName(f"{spec['name']} link {index + 1}")
    body.EnableCollision(False)
    body.SetPos(vector(pose["com"]))
    body.SetRot(quat_from_matrix(pose["rotation"]))
    body.GetVisualShape(0).SetColor(color(*spec["link_color"]))
    add_link_markers(body, spec)
    system.AddBody(body)
    return body


def add_revolute(system, spec, index, child, parent, pose):
    frame = chrono.ChFramed(vector(pose["joint_origin"]), quat_from_matrix(pose["rotation"]))
    joint = chrono.ChLinkLockRevolute()
    joint.SetName(f"{spec['name']} source rotated RevoluteZ joint {index + 1}")
    joint.Initialize(child, parent, frame)
    system.AddLink(joint)
    return joint


def make_axis_marker(system, spec, index, pose):
    marker = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.034, 0.50, 1000, True, False)
    marker.SetName(f"{spec['name']} visible rotated joint axis {index + 1}")
    marker.SetFixed(True)
    marker.EnableCollision(False)
    marker.SetPos(vector(pose["joint_origin"]))
    marker.SetRot(quat_from_matrix(pose["rotation"]))
    marker.GetVisualShape(0).SetColor(color(*spec["joint_color"]))
    system.AddBody(marker)
    return marker


def make_case(system, ground, spec):
    poses = source_tree_poses(spec["offset"])
    links = [make_link(system, spec, index, pose) for index, pose in enumerate(poses)]
    joints = []
    axes = []
    for index, pose in enumerate(poses):
        parent = ground if index == 0 else links[index - 1]
        joints.append(add_revolute(system, spec, index, links[index], parent, pose))
        axes.append(make_axis_marker(system, spec, index, pose))
    return {"spec": spec, "poses": poses, "links": links, "joints": joints, "axes": axes}


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, GRAVITY_Y, 0))

    ground = make_ground(system)
    cases = [make_case(system, ground, spec) for spec in CASE_SPECS]
    items = {"ground": ground, "cases": cases}
    system._kinematic_tree_and_mbs_items = items
    return system, items


def joint_angles(case):
    return [joint.GetRelAngle() for joint in case["joints"]]


def joint_gap(case):
    error = (
        case["links"][0].TransformPointLocalToParent(chrono.ChVector3d(-0.5 * LENGTH, 0, 0))
        - vector(case["poses"][0]["joint_origin"])
    ).Length()
    for index in range(1, N_LINKS):
        child = case["links"][index].TransformPointLocalToParent(chrono.ChVector3d(-0.5 * LENGTH, 0, 0))
        parent = case["links"][index - 1].TransformPointLocalToParent(chrono.ChVector3d(0.5 * LENGTH, 0, 0))
        error = max(error, (child - parent).Length())
    return error


def tip_position(case):
    return case["links"][-1].TransformPointLocalToParent(chrono.ChVector3d(0.5 * LENGTH, 0, 0))


def case_delta(items):
    first = joint_angles(items["cases"][0])
    second = joint_angles(items["cases"][1])
    return max(abs(a - b) for a, b in zip(first, second))


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
    vis.SetWindowSize(1200, 720)
    vis.SetWindowTitle("EXUDYN port: kinematicTreeAndMBS.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(6.0, -11.5, 6.3), chrono.ChVector3d(5.6, 0.2, 1.9))
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
    parts = [f"t={system.GetChTime():6.3f}"]
    for case in items["cases"]:
        angles = joint_angles(case)
        angle_text = ",".join(f"{a:+.4f}" for a in angles)
        tip = tip_position(case)
        parts.append(
            f"{case['spec']['name']}: q=[{angle_text}] sum_q={sum(angles):+.9f} "
            f"tip=({tip.x:+.6f},{tip.y:+.6f},{tip.z:+.6f}) gap={joint_gap(case):.3e}"
        )
    parts.append(f"case_angle_delta={case_delta(items):.3e}")
    print("  ".join(parts))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: kinematicTreeAndMBS.py -> PyChrono explicit KT T66 / KT cpp chains")
    if args.no_vis:
        system, items = simulate(args.duration, args.step)
        print_state(system, items)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

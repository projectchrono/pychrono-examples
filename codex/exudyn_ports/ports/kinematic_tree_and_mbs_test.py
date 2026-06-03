import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN TestModels/kinematicTreeAndMBStest.py:
# the test runs three Robot/ObjectKinematicTree comparison cases
# (3D mechanism, inverted pendulum, and tree structure) with both redundant-MBS
# and kinematic-tree formulations.  PyChrono represents the two formulations as
# paired explicit rigid-body mechanisms with the source link sizes, joint
# transforms, initial reference offset, gravity, and visible joint axes.

LENGTH = 0.5
WIDTH = 0.1
DENSITY_CART = 5000.0
DENSITY_LINK = 1000.0
GRAVITY = 9.81
STEP = 1.0e-3
END_TIME = 0.5
SMALL_ANGLE = 5.0 * 2.0 * math.pi / 360.0

CASE_LAYOUTS = (
    {
        "kind": "3D mechanism",
        "x": -1.15,
        "z": 0.0,
        "duration": 0.5,
        "colors": ((0.12, 0.42, 0.84), (0.10, 0.64, 0.32)),
    },
    {
        "kind": "inverted pendulum",
        "x": 0.70,
        "z": 0.0,
        "duration": 0.5,
        "colors": ((0.26, 0.54, 0.90), (0.86, 0.35, 0.18)),
    },
    {
        "kind": "tree structure",
        "x": 2.65,
        "z": 0.0,
        "duration": 0.25,
        "colors": ((0.44, 0.34, 0.84), (0.10, 0.62, 0.58)),
    },
)

FORMULATION_OFFSETS = (
    ("redundant-MBS", -0.22, 0),
    ("kinematic-tree", 0.22, 1),
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


def rot_y(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def rot_z(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def identity():
    return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]


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


def make_ground(system):
    ground = chrono.ChBody()
    ground.SetName("kinematicTreeAndMBStest ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    for layout in CASE_LAYOUTS:
        for _, z_row, color_index in FORMULATION_OFFSETS:
            base = [layout["x"], 0.0, layout["z"] + z_row]
            rail = chrono.ChVisualShapeBox(1.55, 0.035, 0.035)
            rail.SetColor(color(*layout["colors"][color_index]))
            ground.AddVisualShape(rail, chrono.ChFramed(vector(vadd(base, [0.0, -0.10, 0.0]))))

            origin = chrono.ChVisualShapeSphere(0.045)
            origin.SetColor(color(0.04, 0.04, 0.045))
            ground.AddVisualShape(origin, chrono.ChFramed(vector(base)))

    system.AddBody(ground)
    return ground


def add_marker(body, local, radius, tint):
    shape = chrono.ChVisualShapeSphere(radius)
    shape.SetColor(tint)
    body.AddVisualShape(shape, chrono.ChFramed(local))


def make_box_body(system, name, size, density, pos, rot, tint, link_axis):
    body = chrono.ChBodyEasyBox(size[0], size[1], size[2], density, True, False)
    body.SetName(name)
    body.EnableCollision(False)
    body.SetPos(vector(pos))
    body.SetRot(quat_from_matrix(rot))
    body.GetVisualShape(0).SetColor(tint)

    if link_axis == "x":
        add_marker(body, chrono.ChVector3d(-0.5 * size[0], 0, 0), 0.026, color(0.04, 0.04, 0.045))
        add_marker(body, chrono.ChVector3d(0.5 * size[0], 0, 0), 0.026, color(0.90, 0.55, 0.08))
    else:
        add_marker(body, chrono.ChVector3d(0, 0, 0), 0.024, color(0.14, 0.14, 0.14))
        add_marker(body, chrono.ChVector3d(0, 0.5 * size[1], 0), 0.026, color(0.90, 0.55, 0.08))

    system.AddBody(body)
    return body


def add_prismatic(system, name, cart, ground, base):
    joint = chrono.ChLinkLockPrismatic()
    joint.SetName(f"{name} Px cart")
    joint.Initialize(cart, ground, chrono.ChFramed(vector(base), chrono.QUNIT))
    system.AddLink(joint)
    return joint


def add_revolute(system, name, child, parent, origin, rotation):
    joint = chrono.ChLinkLockRevolute()
    joint.SetName(name)
    joint.Initialize(child, parent, chrono.ChFramed(vector(origin), quat_from_matrix(rotation)))
    system.AddLink(joint)
    return joint


def add_axis_marker(system, name, origin, rotation, tint):
    axis = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.015, 0.22, 1000, True, False)
    axis.SetName(name)
    axis.SetFixed(True)
    axis.EnableCollision(False)
    axis.SetPos(vector(origin))
    axis.SetRot(quat_from_matrix(rotation))
    axis.GetVisualShape(0).SetColor(tint)
    system.AddBody(axis)
    return axis


def add_rz_link(system, name, parent, parent_frame, pre_translation, pre_rotation, q0, tint):
    parent_origin, parent_rot = parent_frame
    joint_origin = vadd(parent_origin, matvec(parent_rot, pre_translation))
    pre_rot = matmul(parent_rot, pre_rotation)
    link_rot = matmul(pre_rot, rot_z(q0))
    com = vadd(joint_origin, matvec(link_rot, [0.0, 0.5 * LENGTH, 0.0]))

    body = make_box_body(
        system,
        f"{name} link",
        [WIDTH, LENGTH, WIDTH],
        DENSITY_LINK,
        com,
        link_rot,
        tint,
        "y",
    )
    joint = add_revolute(system, f"{name} RevoluteZ", body, parent, joint_origin, link_rot)
    add_axis_marker(system, f"{name} visible RevoluteZ axis", joint_origin, link_rot, color(0.05, 0.05, 0.06))
    return body, joint, (joint_origin, link_rot)


def make_cart(system, ground, name, base, tint):
    cart = make_box_body(
        system,
        f"{name} cart",
        [LENGTH, WIDTH, WIDTH],
        DENSITY_CART,
        base,
        identity(),
        tint,
        "x",
    )
    cart_joint = add_prismatic(system, name, cart, ground, base)
    return cart, cart_joint, (list(base), identity())


def build_3d_mechanism(system, ground, name, base, tint):
    cart, cart_joint, frame = make_cart(system, ground, name, base, tint)
    links = []
    joints = [cart_joint]
    body, joint, frame = add_rz_link(system, f"{name} first vertical", cart, frame, [0, 0, 0], identity(), 0.0, tint)
    links.append(body)
    joints.append(joint)
    body, joint, frame = add_rz_link(
        system,
        f"{name} rotated-Y branch",
        body,
        frame,
        [0, LENGTH, 0],
        rot_y(0.25 * math.pi),
        0.0,
        tint,
    )
    links.append(body)
    joints.append(joint)
    body, joint, frame = add_rz_link(
        system,
        f"{name} rotated-Z branch",
        body,
        frame,
        [0, LENGTH, 0],
        rot_z(-0.5 * math.pi),
        0.0,
        tint,
    )
    links.append(body)
    joints.append(joint)
    return {"name": name, "cart": cart, "links": links, "joints": joints, "tip": links[-1], "kind": "3D mechanism"}


def build_inverted_pendulum(system, ground, name, base, tint):
    cart, cart_joint, frame = make_cart(system, ground, name, base, tint)
    links = []
    joints = [cart_joint]
    parent = cart
    for index in range(5):
        pre_translation = [0, 0, 0] if index == 0 else [0, LENGTH, 0]
        q0 = -SMALL_ANGLE if index == 0 else 0.0
        parent, joint, frame = add_rz_link(
            system,
            f"{name} inverted link {index + 1}",
            parent,
            frame,
            pre_translation,
            identity(),
            q0,
            tint,
        )
        links.append(parent)
        joints.append(joint)
    return {"name": name, "cart": cart, "links": links, "joints": joints, "tip": links[-1], "kind": "inverted pendulum"}


def add_tree_branch(system, name, parent, parent_frame, side, sign, tint):
    links = []
    joints = []
    frame = parent_frame
    body_parent = parent
    for index in range(5):
        if index == 0:
            pre_translation = [side * 0.5 * LENGTH, 0.0, 0.0]
            pre_rotation = identity()
        else:
            pre_translation = [0.0, LENGTH, 0.0]
            pre_rotation = rot_z(sign * SMALL_ANGLE)
        body_parent, joint, frame = add_rz_link(
            system,
            f"{name} branch {side:+.0f} link {index + 1}",
            body_parent,
            frame,
            pre_translation,
            pre_rotation,
            0.0,
            tint,
        )
        links.append(body_parent)
        joints.append(joint)
    return links, joints


def build_tree_structure(system, ground, name, base, tint):
    cart, cart_joint, root_frame = make_cart(system, ground, name, base, tint)
    plus_links, plus_joints = add_tree_branch(system, name, cart, root_frame, 1.0, -1.0, tint)
    minus_links, minus_joints = add_tree_branch(system, name, cart, root_frame, -1.0, 1.0, tint)
    links = plus_links + minus_links
    joints = [cart_joint] + plus_joints + minus_joints
    return {"name": name, "cart": cart, "links": links, "joints": joints, "tip": minus_links[-1], "kind": "tree structure"}


def build_pair(system, ground, layout):
    pair = {}
    for label, z_row, color_index in FORMULATION_OFFSETS:
        base = [layout["x"], 0.0, layout["z"] + z_row]
        tint = color(*layout["colors"][color_index])
        name = f"{layout['kind']} {label}"
        if layout["kind"] == "3D mechanism":
            pair[label] = build_3d_mechanism(system, ground, name, base, tint)
        elif layout["kind"] == "inverted pendulum":
            pair[label] = build_inverted_pendulum(system, ground, name, base, tint)
        else:
            pair[label] = build_tree_structure(system, ground, name, base, tint)
    return {"layout": layout, "pair": pair}


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -GRAVITY, 0))

    ground = make_ground(system)
    cases = [build_pair(system, ground, layout) for layout in CASE_LAYOUTS]
    items = {"ground": ground, "cases": cases}
    system._kinematic_tree_and_mbs_test_items = items
    return system, items


def tip_position(chain):
    return chain["tip"].TransformPointLocalToParent(chrono.ChVector3d(0, 0.5 * LENGTH, 0))


def cart_x(chain):
    return chain["cart"].GetPos().x


def pair_tip_delta(case):
    mbs = tip_position(case["pair"]["redundant-MBS"])
    kt = tip_position(case["pair"]["kinematic-tree"])
    dz = abs(mbs.z - kt.z)
    kt_aligned = chrono.ChVector3d(kt.x, kt.y, kt.z - dz)
    return (mbs - kt_aligned).Length()


def sensor_sum(items):
    total = 0.0
    for case in items["cases"]:
        for chain in case["pair"].values():
            total += tip_position(chain).Length()
            total += abs(cart_x(chain))
    return total


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
    vis.SetWindowSize(1200, 760)
    vis.SetWindowTitle("EXUDYN port: kinematicTreeAndMBStest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.8, -5.2, 3.0), chrono.ChVector3d(0.8, 0.85, 0.0))
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
            next_log += 0.125


def print_state(system, items):
    fields = [f"t={system.GetChTime():6.3f}"]
    for case in items["cases"]:
        mbs = case["pair"]["redundant-MBS"]
        kt = case["pair"]["kinematic-tree"]
        mbs_tip = tip_position(mbs)
        kt_tip = tip_position(kt)
        fields.append(
            f"{case['layout']['kind']}: "
            f"mbs_tip=({mbs_tip.x:+.5f},{mbs_tip.y:+.5f},{mbs_tip.z:+.5f}) "
            f"kt_tip=({kt_tip.x:+.5f},{kt_tip.y:+.5f},{kt_tip.z:+.5f}) "
            f"tip_delta_xy={pair_tip_delta(case):.3e}"
        )
    fields.append(f"sensor_sum={sensor_sum(items):.9f}")
    print("  ".join(fields))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: kinematicTreeAndMBStest.py -> PyChrono explicit MBS/KT comparison set")
    if args.no_vis:
        system, items = simulate(args.duration, args.step)
        print_state(system, items)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

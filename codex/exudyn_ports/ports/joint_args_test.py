import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/jointArgsTest.py:
# two revolute-joint construction cases are compared against marker-derived
# joint arguments. Chrono does not have EXUDYN's GetJointArgs helper, so this
# port builds the same four revolute bodies explicitly. The direct and
# marker-style copies are separated slightly in depth so their visual bodies and
# joint axes can be inspected instead of being hidden on top of each other.

LENGTH = 1.0
WIDTH = 0.1
DENSITY = 5000.0
STEP = 2.0e-3
END_TIME = 0.5
ROTATION_VECTOR = chrono.ChVector3d(0.2, 0.3, 0.4)
COPY_Z_OFFSET = 0.17


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def add(a, b):
    return chrono.ChVector3d(a.x + b.x, a.y + b.y, a.z + b.z)


def scale(a, s):
    return chrono.ChVector3d(a.x * s, a.y * s, a.z * s)


def rotation_from_rotvec(rotvec):
    angle = rotvec.Length()
    if angle < 1e-14:
        return chrono.QUNIT
    axis = chrono.ChVector3d(rotvec.x / angle, rotvec.y / angle, rotvec.z / angle)
    return chrono.QuatFromAngleAxis(angle, axis)


def rotate_by_rotvec(vector, rotvec):
    angle = rotvec.Length()
    if angle < 1e-14:
        return chrono.ChVector3d(vector.x, vector.y, vector.z)
    axis = chrono.ChVector3d(rotvec.x / angle, rotvec.y / angle, rotvec.z / angle)
    c = math.cos(angle)
    s = math.sin(angle)
    return add(
        add(scale(vector, c), scale(axis.Cross(vector), s)),
        scale(axis, axis.Dot(vector) * (1.0 - c)),
    )


def joint_frame_at(point, axis):
    axis_v = chrono.ChVector3d(axis.x, axis.y, axis.z)
    axis_v.Normalize()
    z_axis = chrono.ChVector3d(0, 0, 1)
    dot = max(-1.0, min(1.0, z_axis.Dot(axis_v)))
    cross = z_axis.Cross(axis_v)
    if cross.Length() < 1e-12:
        rot = chrono.QUNIT if dot > 0 else chrono.QuatFromAngleX(math.pi)
    else:
        cross.Normalize()
        rot = chrono.QuatFromAngleAxis(math.acos(dot), cross)
    return chrono.ChFramed(point, rot)


def add_local_cylinder(body, point_a, point_b, radius, tint):
    segment = chrono.ChLineSegment(point_a, point_b)
    shape = chrono.ChVisualShapeCylinder(radius, segment.GetLength())
    shape.SetColor(tint)
    body.AddVisualShape(shape, segment.GetFrame())


def add_body_basis(body):
    length = 0.22
    radius = 0.006
    add_local_cylinder(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(length, 0, 0), radius, color(0.92, 0.12, 0.08))
    add_local_cylinder(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, length, 0), radius, color(0.08, 0.68, 0.18))
    add_local_cylinder(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, length), radius, color(0.12, 0.28, 0.92))


def make_ground(system):
    ground = chrono.ChBodyEasyBox(1.65, 1.65, 0.035, 1000, True, False)
    ground.SetName("joint args fixed ground reference")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    ground.SetPos(chrono.ChVector3d(0.48, -0.52, -0.28))
    ground.GetVisualShape(0).SetColor(color(0.78, 0.78, 0.74))
    ground.GetVisualShape(0).SetOpacity(0.32)
    system.AddBody(ground)
    return ground


def make_link(system, name, center, rotation, tint):
    body = chrono.ChBodyEasyBox(LENGTH, WIDTH, WIDTH, DENSITY, True, False)
    body.SetName(name)
    body.EnableCollision(False)
    body.SetPos(center)
    body.SetRot(rotation)
    body.GetVisualShape(0).SetColor(tint)
    add_body_basis(body)
    system.AddBody(body)
    return body


def make_joint_axis_visual(system, name, point, axis, tint):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)

    axis_v = chrono.ChVector3d(axis.x, axis.y, axis.z)
    axis_v.Normalize()
    half = scale(axis_v, 0.22)
    segment = chrono.ChLineSegment(add(point, scale(half, -1.0)), add(point, half))
    cylinder = chrono.ChVisualShapeCylinder(0.018, segment.GetLength())
    cylinder.SetColor(tint)
    body.AddVisualShape(cylinder, segment.GetFrame())

    sphere = chrono.ChVisualShapeSphere(0.045)
    sphere.SetColor(color(0.04, 0.04, 0.05))
    body.AddVisualShape(sphere, chrono.ChFramed(point))
    system.AddBody(body)
    return body


def add_revolute(system, name, body, ground, point, axis, tint):
    joint = chrono.ChLinkLockRevolute()
    joint.SetName(name)
    joint.Initialize(body, ground, joint_frame_at(point, axis))
    system.AddLink(joint)
    make_joint_axis_visual(system, f"{name} visible axis", point, axis, tint)
    return joint


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
    ground = make_ground(system)

    rotation = rotation_from_rotvec(ROTATION_VECTOR)
    local_axis_y = rotate_by_rotvec(chrono.ChVector3d(0, 1, 0), ROTATION_VECTOR)

    cases = []
    pair_specs = [
        {
            "base_center": chrono.ChVector3d(0.5 * LENGTH, 0.0, 0.0),
            "joint_point": chrono.ChVector3d(0.25, -0.1, 0.1),
            "axis": chrono.ChVector3d(0, 0, 1),
            "names": ("reference global-Z revolute", "marker-derived global-Z revolute"),
            "colors": (color(0.88, 0.12, 0.08), color(0.10, 0.42, 0.90)),
        },
        {
            "base_center": chrono.ChVector3d(0.5 * LENGTH, -LENGTH, 0.0),
            "joint_point": chrono.ChVector3d(0.1, -0.05, 0.05),
            "axis": local_axis_y,
            "names": ("reference local-Y revolute", "marker-derived local-Y revolute"),
            "colors": (color(0.32, 0.82, 0.18), color(0.95, 0.52, 0.08)),
        },
    ]

    for pair_index, spec in enumerate(pair_specs):
        for copy_index, copy_sign in enumerate((-1.0, 1.0)):
            offset = chrono.ChVector3d(0.0, 0.0, copy_sign * COPY_Z_OFFSET)
            body = make_link(
                system,
                spec["names"][copy_index],
                add(spec["base_center"], offset),
                rotation,
                spec["colors"][copy_index],
            )
            joint = add_revolute(
                system,
                spec["names"][copy_index],
                body,
                ground,
                add(spec["joint_point"], offset),
                spec["axis"],
                spec["colors"][copy_index],
            )
            cases.append({"body": body, "joint": joint, "pair": pair_index, "copy": copy_index})

    system._joint_args_items = {"cases": cases, "ground": ground}
    return system, [item["body"] for item in cases], [item["joint"] for item in cases]


def simulate(duration, step):
    system, bodies, joints = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, bodies, joints


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, bodies, joints = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: jointArgsTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.25, 1.25, 2.65), chrono.ChVector3d(0.45, -0.55, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, bodies)
            next_log += 0.25


def print_state(system, bodies):
    totals = []
    for body in bodies:
        pos = body.GetPos()
        rot = body.GetRot()
        totals.append(pos.x + pos.y + pos.z + rot.e0 + rot.e1 + rot.e2 + rot.e3)
    print(
        f"t={system.GetChTime():6.3f}  "
        f"case_checksum={sum(totals):+.6f}  "
        f"body0_y={bodies[0].GetPos().y:+.5f}  "
        f"body3_y={bodies[-1].GetPos().y:+.5f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: jointArgsTest.py -> PyChrono explicit revolute joint-argument comparison")
    if args.no_vis:
        system, bodies, joints = simulate(args.duration, args.step)
        print_state(system, bodies)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

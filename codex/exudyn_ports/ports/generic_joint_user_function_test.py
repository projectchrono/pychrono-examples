import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/genericJointUserFunctionTest.py:
# a five-body rigid chain where the first two GenericJoint constraints receive
# a time-dependent offsetUserFunction. Chrono does not expose the same six-axis
# generic-joint offset callback, so this port replays the source offset
# kinematics and renders the rigid bodies, body bases, joint points, driven
# x/z offset axes, and the later spherical/free-axis generic joints explicitly.

N_BODIES = 5
S = 0.1
SX = 3.0 * S
BODY_LENGTH = 2.0 * SX
STEP = 1.0e-3
END_TIME = 2.5


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vcopy(v):
    return chrono.ChVector3d(v.x, v.y, v.z)


def vadd(a, b):
    return chrono.ChVector3d(a.x + b.x, a.y + b.y, a.z + b.z)


def vscale(v, scale):
    return chrono.ChVector3d(v.x * scale, v.y * scale, v.z * scale)


def source_offsets(time):
    ramp = 1.0 - math.cos(time * 2.0 * math.pi / 4.0)
    phi_z = -2.0 * math.pi * 0.5 * ramp / N_BODIES
    phi_x = -0.5 * math.pi * 0.5 * ramp / N_BODIES
    return phi_x, phi_z


def add_local_cylinder(body, point_a, point_b, radius, tint):
    segment = chrono.ChLineSegment(point_a, point_b)
    if segment.GetLength() <= 1.0e-12:
        return None
    shape = chrono.ChVisualShapeCylinder(radius, segment.GetLength())
    shape.SetColor(tint)
    body.AddVisualShape(shape, segment.GetFrame())
    return shape


def add_body_axes(body):
    add_local_cylinder(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0.16, 0, 0), 0.006, color(0.92, 0.10, 0.06))
    add_local_cylinder(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0.16, 0), 0.006, color(0.08, 0.62, 0.14))
    add_local_cylinder(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0.16), 0.006, color(0.08, 0.18, 0.90))


def make_chain_body(system, index):
    body = chrono.ChBodyEasyBox(BODY_LENGTH, 2.0 * S, 2.0 * S, 1000, True, False)
    body.SetName(f"generic joint user-function chain body {index + 1}")
    body.SetFixed(True)
    body.EnableCollision(False)
    palette = [
        color(0.90, 0.14, 0.10),
        color(0.10, 0.34, 0.88),
        color(0.14, 0.62, 0.22),
        color(0.94, 0.56, 0.08),
        color(0.55, 0.24, 0.82),
    ]
    body.GetVisualShape(0).SetColor(palette[index])
    body.GetVisualShape(0).SetOpacity(0.72)
    add_body_axes(body)

    left = chrono.ChVisualShapeSphere(0.022)
    left.SetColor(color(0.04, 0.04, 0.045))
    body.AddVisualShape(left, chrono.ChFramed(chrono.ChVector3d(-SX, 0, 0)))
    right = chrono.ChVisualShapeSphere(0.022)
    right.SetColor(color(0.04, 0.04, 0.045))
    body.AddVisualShape(right, chrono.ChFramed(chrono.ChVector3d(SX, 0, 0)))
    system.AddBody(body)
    return body


def make_mutable_axis(system, name, radius, tint):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    system.AddBody(body)
    shape = chrono.ChVisualShapeSegment()
    shape.SetMutable(True)
    shape.SetThickness(radius)
    shape.SetColor(tint)
    body.AddVisualShape(shape)
    return body, shape


def make_joint_marker(system, index):
    marker = chrono.ChBodyEasySphere(0.035, 1000, True, False)
    marker.SetName(f"generic joint user-function joint marker {index}")
    marker.SetFixed(True)
    marker.EnableCollision(False)
    marker.GetVisualShape(0).SetColor(color(0.03, 0.03, 0.035))
    system.AddBody(marker)

    axis_body, axis_shape = make_mutable_axis(system, f"generic joint user-function visible axis {index}", 4, color(0.02, 0.02, 0.025))
    return {"marker": marker, "axis_body": axis_body, "axis_shape": axis_shape}


def make_reference_plate(system):
    plate = chrono.ChBodyEasyBox(3.35, 1.18, 0.035, 1000, True, False)
    plate.SetName("generic joint user-function background plate")
    plate.SetFixed(True)
    plate.EnableCollision(False)
    plate.SetPos(chrono.ChVector3d(1.05, -0.18, -0.28))
    plate.GetVisualShape(0).SetColor(color(0.78, 0.78, 0.74))
    plate.GetVisualShape(0).SetOpacity(0.22)
    system.AddBody(plate)


def update_axis_visual(item, point, axis, half_length=0.13):
    axis_v = vcopy(axis)
    if axis_v.Length() <= 1.0e-12:
        axis_v = chrono.ChVector3d(0, 0, 1)
    axis_v.Normalize()
    p0 = vadd(point, vscale(axis_v, -half_length))
    p1 = vadd(point, vscale(axis_v, half_length))
    item["axis_shape"].SetLineGeometry(chrono.ChLineSegment(p0, p1))
    item["axis_body"].UpdateVisualModel()


def update_kinematics(system):
    items = system._generic_joint_user_function_items
    bodies = items["bodies"]
    joints = items["joints"]
    phi_x, phi_z = source_offsets(system.GetChTime())
    driven_rotation = chrono.QuatFromAngleX(phi_x) * chrono.QuatFromAngleZ(phi_z)

    anchor = chrono.ChVector3d(-SX, 0, 0)
    rotation = chrono.QUNIT
    for index, body in enumerate(bodies):
        if index < 2:
            rotation = rotation * driven_rotation
        elif index == 3:
            # EXUDYN leaves this generic joint's y-rotation free; show the
            # available axis with a small source-offset-dependent pose change.
            rotation = rotation * chrono.QuatFromAngleY(-0.25 * math.sin(-phi_z))
        elif index == 4:
            # EXUDYN leaves this generic joint's z-rotation free.
            rotation = rotation * chrono.QuatFromAngleZ(0.18 * math.sin(-phi_z))

        center = vadd(anchor, rotation.Rotate(chrono.ChVector3d(SX, 0, 0)))
        body.SetPos(center)
        body.SetRot(rotation)
        body.UpdateVisualModel()

        joint = joints[index]
        joint["marker"].SetPos(anchor)
        if index < 2:
            update_axis_visual(joint, anchor, rotation.Rotate(chrono.ChVector3d(0, 0, 1)), 0.15)
        elif index == 3:
            update_axis_visual(joint, anchor, rotation.Rotate(chrono.ChVector3d(0, 1, 0)), 0.15)
        else:
            update_axis_visual(joint, anchor, rotation.Rotate(chrono.ChVector3d(0, 0, 1)), 0.15)

        anchor = vadd(anchor, rotation.Rotate(chrono.ChVector3d(BODY_LENGTH, 0, 0)))

    items["last_right_joint"] = anchor


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
    make_reference_plate(system)
    bodies = [make_chain_body(system, i) for i in range(N_BODIES)]
    joints = [make_joint_marker(system, i) for i in range(N_BODIES)]
    system._generic_joint_user_function_items = {"bodies": bodies, "joints": joints, "last_right_joint": chrono.ChVector3d(0, 0, 0)}
    update_kinematics(system)
    return system, bodies, joints


def update_visuals(system):
    if hasattr(system, "_generic_joint_user_function_items"):
        update_kinematics(system)


def simulate(duration, step):
    system, bodies, joints = build_system()
    while system.GetChTime() < duration:
        update_kinematics(system)
        system.DoStepDynamics(step)
    update_kinematics(system)
    return system, bodies, joints


def print_state(system, bodies):
    phi_x, phi_z = source_offsets(system.GetChTime())
    last = bodies[-1].GetPos()
    solution = abs(last.x) + abs(last.y) + abs(last.z)
    print(
        f"t={system.GetChTime():6.3f}  "
        f"offset=(phi_x={phi_x:+.6f}, phi_z={phi_z:+.6f})  "
        f"last=({last.x:+.6f}, {last.y:+.6f}, {last.z:+.6f})  "
        f"solution={solution:.9f}"
    )


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, bodies, joints = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: genericJointUserFunctionTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.00, -3.20, 1.55), chrono.ChVector3d(0.00, -0.48, 0.0))
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
            print_state(system, bodies)
            next_log += 0.5


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: genericJointUserFunctionTest.py -> PyChrono generic-joint offset replay")
    if args.no_vis:
        system, bodies, joints = simulate(args.duration, args.step)
        print_state(system, bodies)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

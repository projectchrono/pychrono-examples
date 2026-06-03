import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/bricardMechanism.py:
# a closed Bricard-style 3D linkage made from five moving rectangular links and
# one ground closure, connected by revolute axes Y/Z/X/Y/Z/X. Chrono carries the
# loop with revolute constraints; explicit axis cylinders and pin spheres make
# the joint frame sequence inspectable in visualization.

LENGTH = 1.0
WIDTH = 0.1
DENSITY = 100.0
STEP = 1.0e-3
END_TIME = 1.0

JOINT_POINTS = [
    (0.0, 0.0, 0.0),
    (LENGTH, 0.0, 0.0),
    (LENGTH, -LENGTH, 0.0),
    (LENGTH, -LENGTH, LENGTH),
    (0.0, -LENGTH, LENGTH),
    (0.0, 0.0, LENGTH),
]

JOINT_AXES = [
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
    (1.0, 0.0, 0.0),
]


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def v3(values):
    return chrono.ChVector3d(values[0], values[1], values[2])


def add(a, b):
    return chrono.ChVector3d(a.x + b.x, a.y + b.y, a.z + b.z)


def sub(a, b):
    return chrono.ChVector3d(a.x - b.x, a.y - b.y, a.z - b.z)


def scale(v, factor):
    return chrono.ChVector3d(v.x * factor, v.y * factor, v.z * factor)


def axis_quat(axis):
    axis_v = v3(axis)
    axis_v.Normalize()
    z_axis = chrono.ChVector3d(0, 0, 1)
    dot = max(-1.0, min(1.0, z_axis.Dot(axis_v)))
    cross = z_axis.Cross(axis_v)
    if cross.Length() < 1.0e-12:
        return chrono.QUNIT if dot > 0 else chrono.QuatFromAngleX(math.pi)
    cross.Normalize()
    return chrono.QuatFromAngleAxis(math.acos(dot), cross)


def joint_frame(point, axis):
    return chrono.ChFramed(v3(point), axis_quat(axis))


def link_rotation(index):
    if index == 0:
        return chrono.QUNIT
    if index == 1:
        return chrono.QuatFromAngleZ(-0.5 * math.pi)
    if index == 2:
        return chrono.QuatFromAngleY(-0.5 * math.pi)
    if index == 3:
        return chrono.QuatFromAngleZ(math.pi)
    if index == 4:
        return chrono.QuatFromAngleZ(0.5 * math.pi)
    raise ValueError(index)


def link_direction(index):
    p0 = v3(JOINT_POINTS[index])
    p1 = v3(JOINT_POINTS[index + 1])
    direction = sub(p1, p0)
    direction.Normalize()
    return direction


def add_body_axes(body):
    axis_specs = [
        (chrono.ChVector3d(0.16, 0, 0), color(0.95, 0.12, 0.08), chrono.Q_ROTATE_Z_TO_X),
        (chrono.ChVector3d(0, 0.16, 0), color(0.10, 0.68, 0.18), chrono.Q_ROTATE_Z_TO_Y),
        (chrono.ChVector3d(0, 0, 0.16), color(0.08, 0.28, 0.92), chrono.QUNIT),
    ]
    for half_axis, tint, rot in axis_specs:
        shape = chrono.ChVisualShapeCylinder(0.010, 2.0 * half_axis.Length())
        shape.SetColor(tint)
        body.AddVisualShape(shape, chrono.ChFramed(half_axis, rot))


def add_joint_visual(system, point, axis, index):
    marker = chrono.ChBody()
    marker.SetName(f"bricard revolute axis marker {index + 1}")
    marker.SetFixed(True)
    marker.EnableCollision(False)
    system.AddBody(marker)

    center = v3(point)
    axis_v = v3(axis)
    axis_v.Normalize()
    segment = chrono.ChLineSegment(add(center, scale(axis_v, -0.08)), add(center, scale(axis_v, 0.08)))
    cylinder = chrono.ChVisualShapeCylinder(0.018, segment.GetLength())
    cylinder.SetColor(color(0.04, 0.04, 0.05))
    marker.AddVisualShape(cylinder, segment.GetFrame())

    sphere = chrono.ChVisualShapeSphere(0.045)
    sphere.SetColor(color(0.96, 0.72, 0.08))
    marker.AddVisualShape(sphere, chrono.ChFramed(center))
    return marker


def add_reference_plate(system):
    plate = chrono.ChBodyEasyBox(1.9, 0.025, 1.55, 1000, True, False)
    plate.SetName("bricard vertical reference plate")
    plate.SetFixed(True)
    plate.EnableCollision(False)
    plate.SetPos(chrono.ChVector3d(0.5, -2.1, 0.5))
    plate.GetVisualShape(0).SetColor(color(0.72, 0.74, 0.76))
    plate.GetVisualShape(0).SetOpacity(0.28)
    system.AddBody(plate)
    return plate


def make_link(index):
    start = v3(JOINT_POINTS[index])
    center = add(start, scale(link_direction(index), 0.5 * LENGTH))
    body = chrono.ChBodyEasyBox(LENGTH, WIDTH, WIDTH, DENSITY, True, False)
    body.SetName(f"bricard link {index + 1}")
    body.SetPos(center)
    body.SetRot(link_rotation(index))
    body.GetVisualShape(0).SetColor(color(0.10, 0.34 + 0.07 * index, 0.82))
    body.EnableCollision(False)
    add_body_axes(body)
    return body


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    ground = chrono.ChBody()
    ground.SetName("bricard ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)
    add_reference_plate(system)

    bodies = [make_link(i) for i in range(5)]
    for body in bodies:
        system.AddBody(body)

    previous = ground
    joints = []
    for i in range(5):
        joint = chrono.ChLinkLockRevolute()
        joint.SetName(f"bricard revolute joint {i + 1}")
        joint.Initialize(bodies[i], previous, joint_frame(JOINT_POINTS[i], JOINT_AXES[i]))
        system.AddLink(joint)
        joints.append(joint)
        add_joint_visual(system, JOINT_POINTS[i], JOINT_AXES[i], i)
        previous = bodies[i]

    closing_joint = chrono.ChLinkLockRevolute()
    closing_joint.SetName("bricard closing revolute joint 6")
    closing_joint.Initialize(ground, bodies[-1], joint_frame(JOINT_POINTS[5], JOINT_AXES[5]))
    system.AddLink(closing_joint)
    joints.append(closing_joint)
    add_joint_visual(system, JOINT_POINTS[5], JOINT_AXES[5], 5)

    return system, bodies, joints


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
    vis.SetWindowTitle("EXUDYN port: bricardMechanism.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(2.2, -4.0, 2.2), chrono.ChVector3d(0.45, -0.55, 0.55))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, bodies, joints)
            next_log += 0.25


def print_state(system, bodies, joints):
    norm = math.sqrt(sum(body.GetPos().Length2() for body in bodies))
    last = bodies[-1]
    print(
        f"t={system.GetChTime():6.3f}  "
        f"last_pos=({last.GetPos().x:+.4f}, {last.GetPos().y:+.4f}, {last.GetPos().z:+.4f})  "
        f"norm={norm:.6f}  links={len(bodies)} joints={len(joints)}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: bricardMechanism.py -> PyChrono closed-loop Bricard linkage")
    if args.no_vis:
        system, bodies, joints = simulate(args.duration, args.step)
        print_state(system, bodies, joints)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

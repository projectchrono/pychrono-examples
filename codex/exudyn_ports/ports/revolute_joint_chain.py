import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN Examples/addRevoluteJoint.py:
# a four-link rigid-body chain under gravity with revolute joints whose axes are
# specified in the global frame.

LENGTH = 0.4
WIDTH = 0.1
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def v3(values):
    return chrono.ChVector3d(values[0], values[1], values[2])


def add(a, b):
    return chrono.ChVector3d(a.x + b.x, a.y + b.y, a.z + b.z)


def scale(a, s):
    return chrono.ChVector3d(a.x * s, a.y * s, a.z * s)


def joint_frame_at(point, axis):
    axis_v = v3(axis)
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


def make_link(index, center, rotation):
    body = chrono.ChBodyEasyBox(LENGTH, WIDTH, WIDTH, 1000, True, False)
    body.SetName(f"link {index}")
    body.SetPos(center)
    body.SetRot(rotation)
    body.GetVisualShape(0).SetColor(color(0.1, 0.38 + 0.1 * index, 0.75))
    return body


def build_system():
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground.SetPos(chrono.ChVector3d(-0.5 * LENGTH, 0, 0))
    sys.AddBody(ground)

    rotations = [
        chrono.QUNIT,
        chrono.QuatFromAngleY(0.5 * math.pi),
        chrono.QuatFromAngleZ(0.5 * math.pi),
        chrono.QuatFromAngleX(-0.5 * math.pi),
    ]
    directions = [
        chrono.ChVector3d(1, 0, 0),
        chrono.ChVector3d(0, 0, -1),
        chrono.ChVector3d(0, 1, 0),
        chrono.ChVector3d(1, 0, 0),
    ]
    axes = [
        (0, 0, 1),
        (1, 1, 1),
        (1, 0, 0),
        (0, 0, 1),
    ]

    bodies = []
    previous = ground
    joint_point = chrono.ChVector3d(-0.5 * LENGTH, 0, 0)
    for i, (rot, direction, axis) in enumerate(zip(rotations, directions, axes)):
        center = add(joint_point, scale(direction, 0.5 * LENGTH))
        body = make_link(i, center, rot)
        sys.AddBody(body)

        joint = chrono.ChLinkLockRevolute()
        joint.Initialize(body, previous, joint_frame_at(joint_point, axis))
        sys.AddLink(joint)

        bodies.append(body)
        previous = body
        joint_point = add(joint_point, scale(direction, LENGTH))

    return sys, bodies


def simulate(duration, step):
    sys, bodies = build_system()
    while sys.GetChTime() < duration:
        sys.DoStepDynamics(step)
    return sys, bodies


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    sys, bodies = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: addRevoluteJoint.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.8, 1.1, 2.4), chrono.ChVector3d(0.25, 0.1, -0.2))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and sys.GetChTime() < duration:
        t = sys.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(step)
        if t >= next_log:
            print_state(sys, bodies)
            next_log += 0.5


def print_state(sys, bodies):
    last = bodies[-1]
    print(
        f"t={sys.GetChTime():6.3f}  "
        f"last_pos=({last.GetPos().x:+.4f}, {last.GetPos().y:+.4f}, {last.GetPos().z:+.4f})  "
        f"links={len(bodies)}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: addRevoluteJoint.py -> PyChrono revolute joint chain")
    if args.no_vis:
        sys, bodies = simulate(args.duration, args.step)
        print_state(sys, bodies)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

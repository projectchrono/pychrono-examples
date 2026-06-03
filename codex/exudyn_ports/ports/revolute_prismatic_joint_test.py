import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN
# TestModels/revoluteJointPrismaticJointTest.py:
# a chain of five rigid bodies connected by revolute joints except for one
# prismatic joint, with a spring-damper limiting the prismatic motion.

LENGTH = 0.4
WIDTH = 0.1
SPRING = 500.0
DAMPING = SPRING * 0.02
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def v3(values):
    return chrono.ChVector3d(values[0], values[1], values[2])


def add(a, b):
    return chrono.ChVector3d(a.x + b.x, a.y + b.y, a.z + b.z)


def scale(a, s):
    return chrono.ChVector3d(a.x * s, a.y * s, a.z * s)


def diagonal_matrix(values):
    matrix = chrono.ChMatrix66d()
    matrix.SetZero()
    for i, value in enumerate(values):
        matrix.SetItem(i, i, value)
    return matrix


def joint_frame_at(point, axis):
    axis_v = axis
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


def update_visuals(sys):
    update_system_visuals(sys)


def build_system():
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    ground = chrono.ChBody()
    ground.SetFixed(True)
    sys.AddBody(ground)

    axes = [
        chrono.ChVector3d(0, 0, 1),
        chrono.ChVector3d(1, 0, 0),
        chrono.ChVector3d(0, 1, 0),
        chrono.ChVector3d(0, 0, 1),
        chrono.ChVector3d(0, 0, 1),
    ]

    bodies = []
    previous = ground
    joint_point = chrono.ChVector3d(0, 0, 0)
    visual_spring = None
    for i in range(5):
        center = add(joint_point, chrono.ChVector3d(0.5 * LENGTH, 0, 0))
        body = chrono.ChBodyEasyBox(LENGTH, WIDTH, WIDTH, 1000, True, False)
        body.SetName(f"link {i}")
        body.SetPos(center)
        body.GetVisualShape(0).SetColor(color(0.10, 0.34 + 0.08 * i, 0.78))
        sys.AddBody(body)

        frame = joint_frame_at(joint_point, axes[i])
        if i == 2:
            joint = chrono.ChLinkLockPrismatic()
            joint.Initialize(body, previous, frame)
            sys.AddLink(joint)

            k = diagonal_matrix([SPRING, SPRING, SPRING, 0, 0, 0])
            r = diagonal_matrix([DAMPING, DAMPING, DAMPING, 0, 0, 0])
            bushing = chrono.ChLinkBushing()
            bushing.Initialize(body, previous, frame, k, r)
            sys.AddLink(bushing)

            visual_spring = chrono.ChLinkTSDA()
            visual_spring.Initialize(
                body,
                previous,
                True,
                chrono.ChVector3d(-0.5 * LENGTH, 0.16, 0),
                chrono.ChVector3d(0.5 * LENGTH, -0.16, 0),
            )
            visual_spring.SetRestLength(0.32)
            visual_spring.SetSpringCoefficient(0)
            visual_spring.SetDampingCoefficient(0)
            sys.AddLink(visual_spring)
            spring_shape = chrono.ChVisualShapeSpring(0.045, 80, 10)
            spring_shape.SetColor(color(0.85, 0.18, 0.12))
            visual_spring.AddVisualShape(spring_shape)
            attach_spring_visual(sys, visual_spring, 0.045, 80, 10, color(0.85, 0.18, 0.12))
        else:
            joint = chrono.ChLinkLockRevolute()
            joint.Initialize(body, previous, frame)
            sys.AddLink(joint)

        bodies.append(body)
        previous = body
        joint_point = add(joint_point, chrono.ChVector3d(LENGTH, 0, 0))

    force = chrono.ChForce()
    force.SetF_z(chrono.ChFunctionConst(20.0))
    bodies[-1].AddForce(force)

    return sys, bodies, visual_spring


def simulate(duration, step):
    sys, bodies, visual_spring = build_system()
    while sys.GetChTime() < duration:
        sys.DoStepDynamics(step)
    return sys, bodies, visual_spring


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    sys, bodies, visual_spring = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: revoluteJointPrismaticJointTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.2, 1.0, 2.6), chrono.ChVector3d(0.9, 0.0, 0.15))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and sys.GetChTime() < duration:
        t = sys.GetChTime()
        vis.BeginScene()
        update_visuals(sys)
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(step)
        if t >= next_log:
            print_state(sys, bodies)
            next_log += 0.1


def print_state(sys, bodies):
    last = bodies[-1]
    print(
        f"t={sys.GetChTime():6.3f}  "
        f"last_pos=({last.GetPos().x:+.4f},{last.GetPos().y:+.4f},{last.GetPos().z:+.4f})  "
        f"links={len(bodies)}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0.25)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print(
        "EXUDYN port: revoluteJointPrismaticJointTest.py -> "
        "PyChrono revolute/prismatic chain"
    )
    if args.no_vis:
        sys, bodies, _ = simulate(args.duration, args.step)
        print_state(sys, bodies)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

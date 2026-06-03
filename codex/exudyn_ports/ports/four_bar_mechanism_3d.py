import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN Examples/fourBarMechanism3D.py:
# a simple 3D four-bar linkage with rigid links and revolute joints. The EXUDYN
# example discusses redundant loop constraints; this PyChrono version uses
# revolute joints at the four physical pins and a small x gravity disturbance.

LENGTH = 1.0
WIDTH = 0.1
DENSITY = 5000.0
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_link(name, position, rotation, tint):
    body = chrono.ChBodyEasyBox(LENGTH, WIDTH, WIDTH, DENSITY, True, False)
    body.SetName(name)
    body.SetPos(position)
    body.SetRot(rotation)
    body.GetVisualShape(0).SetColor(tint)
    return body


def add_pin(sys, position, tint):
    pin = chrono.ChBodyEasySphere(0.065, 1000, True, False)
    pin.SetFixed(True)
    pin.SetPos(position)
    pin.GetVisualShape(0).SetColor(tint)
    sys.AddBody(pin)
    return pin


def add_revolute(sys, body_a, body_b, position):
    joint = chrono.ChLinkLockRevolute()
    joint.Initialize(body_a, body_b, chrono.ChFramed(position))
    sys.AddLink(joint)
    return joint


def build_system():
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0.1, -9.81, 0))

    ground = chrono.ChBody()
    ground.SetFixed(True)
    sys.AddBody(ground)

    left = make_link(
        "left link",
        chrono.ChVector3d(0, 0.5 * LENGTH, 0),
        chrono.QuatFromAngleZ(0.5 * math.pi),
        color(0.85, 0.15, 0.1),
    )
    top = make_link(
        "top link",
        chrono.ChVector3d(0.5 * LENGTH, LENGTH, 0),
        chrono.QUNIT,
        color(0.1, 0.65, 0.25),
    )
    right = make_link(
        "right link",
        chrono.ChVector3d(LENGTH, 0.5 * LENGTH, 0),
        chrono.QuatFromAngleZ(-0.5 * math.pi),
        color(0.12, 0.42, 0.85),
    )

    for body in (left, top, right):
        sys.AddBody(body)

    ground_bar = chrono.ChBodyEasyBox(LENGTH + 0.25, 0.05, 0.05, 1000, True, False)
    ground_bar.SetFixed(True)
    ground_bar.SetPos(chrono.ChVector3d(0.5 * LENGTH, -0.04, 0))
    ground_bar.GetVisualShape(0).SetColor(color(0.35, 0.35, 0.35))
    sys.AddBody(ground_bar)

    p0 = chrono.ChVector3d(0, 0, 0)
    p1 = chrono.ChVector3d(0, LENGTH, 0)
    p2 = chrono.ChVector3d(LENGTH, LENGTH, 0)
    p3 = chrono.ChVector3d(LENGTH, 0, 0)

    joints = [
        add_revolute(sys, left, ground, p0),
        add_revolute(sys, top, left, p1),
        add_revolute(sys, right, top, p2),
        add_revolute(sys, right, ground, p3),
    ]

    for point in (p0, p1, p2, p3):
        add_pin(sys, point, color(0.12, 0.12, 0.12))

    return sys, (left, top, right), joints


def simulate(duration, step):
    sys, bodies, joints = build_system()
    while sys.GetChTime() < duration:
        sys.DoStepDynamics(step)
    return sys, bodies, joints


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    sys, bodies, joints = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: fourBarMechanism3D.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.5, 0.65, 3.0), chrono.ChVector3d(0.5, 0.45, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and sys.GetChTime() < duration:
        t = sys.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(step)
        if t >= next_log:
            print_state(sys, bodies, joints)
            next_log += 0.5


def print_state(sys, bodies, joints):
    top = bodies[1]
    print(
        f"t={sys.GetChTime():6.3f}  "
        f"top=({top.GetPos().x:+.4f}, {top.GetPos().y:+.4f}, {top.GetPos().z:+.4f})  "
        f"links={len(bodies)}  joints={len(joints)}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: fourBarMechanism3D.py -> PyChrono four-bar linkage")
    if args.no_vis:
        sys, bodies, joints = simulate(args.duration, args.step)
        print_state(sys, bodies, joints)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

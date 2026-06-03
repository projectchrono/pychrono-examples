import argparse

import pychrono.core as chrono


# Reproduces the intent of EXUDYN Examples/rigidPendulum.py:
# a 2D rigid rectangular pendulum with a revolute support, gravity, and initial
# angular velocity.

LENGTH = 1.0
WIDTH = 0.1
MASS = 12.0
INITIAL_ANGULAR_SPEED = 2.0
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def build_system():
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    ground = chrono.ChBody()
    ground.SetFixed(True)
    sys.AddBody(ground)

    pendulum = chrono.ChBodyEasyBox(LENGTH, WIDTH, WIDTH, 1000, True, False)
    pendulum.SetName("rigid pendulum")
    pendulum.SetMass(MASS)
    inertia_zz = MASS * LENGTH * LENGTH / 12.0
    pendulum.SetInertiaXX(chrono.ChVector3d(0.02, inertia_zz, inertia_zz))
    pendulum.SetPos(chrono.ChVector3d(-0.5 * LENGTH, LENGTH, 0))
    pendulum.SetAngVelLocal(chrono.ChVector3d(0, 0, INITIAL_ANGULAR_SPEED))
    pendulum.GetVisualShape(0).SetColor(color(0.12, 0.42, 0.85))
    sys.AddBody(pendulum)

    pivot = chrono.ChVector3d(-LENGTH, LENGTH, 0)
    joint = chrono.ChLinkLockRevolute()
    joint.Initialize(pendulum, ground, chrono.ChFramed(pivot))
    sys.AddLink(joint)

    pivot_body = chrono.ChBodyEasySphere(0.06, 1000, True, False)
    pivot_body.SetFixed(True)
    pivot_body.SetPos(pivot)
    pivot_body.GetVisualShape(0).SetColor(color(0.15, 0.15, 0.15))
    sys.AddBody(pivot_body)

    reference = chrono.ChBodyEasyBox(1.4, 0.02, 0.02, 1000, True, False)
    reference.SetFixed(True)
    reference.SetPos(chrono.ChVector3d(-0.55, -0.08, 0))
    reference.GetVisualShape(0).SetColor(color(0.5, 0.5, 0.5))
    sys.AddBody(reference)

    return sys, pendulum, joint


def simulate(duration, step):
    sys, pendulum, joint = build_system()
    while sys.GetChTime() < duration:
        sys.DoStepDynamics(step)
    return sys, pendulum, joint


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    sys, pendulum, joint = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: rigidPendulum.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(-0.55, 1.0, 2.5), chrono.ChVector3d(-0.55, 0.55, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and sys.GetChTime() < duration:
        t = sys.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(step)
        if t >= next_log:
            print_state(sys, pendulum, joint)
            next_log += 0.5


def print_state(sys, pendulum, joint):
    angle_z = pendulum.GetRot().GetCardanAnglesXYZ().z
    print(
        f"t={sys.GetChTime():6.3f}  "
        f"pos=({pendulum.GetPos().x:+.4f}, {pendulum.GetPos().y:+.4f})  "
        f"angle_z={angle_z:+.4f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: rigidPendulum.py -> PyChrono rigid pendulum")
    if args.no_vis:
        sys, pendulum, joint = simulate(args.duration, args.step)
        print_state(sys, pendulum, joint)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

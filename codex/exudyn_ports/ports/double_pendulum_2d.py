import argparse

import pychrono.core as chrono


# Reproduces the intent of EXUDYN Examples/doublePendulum2D.py:
# two unit-mass rigid bars connected by planar revolute joints and gravity.

LENGTH = 1.0
WIDTH = 0.06
MASS = 1.0
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_bar(name, position, tint):
    body = chrono.ChBodyEasyBox(LENGTH, WIDTH, WIDTH, 1000, True, False)
    body.SetName(name)
    body.SetMass(MASS)
    body.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, 1.0))
    body.SetPos(position)
    body.GetVisualShape(0).SetColor(tint)
    return body


def build_system():
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    ground = chrono.ChBody()
    ground.SetFixed(True)
    sys.AddBody(ground)

    bar0 = make_bar("upper bar", chrono.ChVector3d(0, 0, 0), color(0.86, 0.18, 0.12))
    bar1 = make_bar("lower bar", chrono.ChVector3d(1, 0, 0), color(0.1, 0.34, 0.85))
    sys.AddBody(bar0)
    sys.AddBody(bar1)

    joint_ground = chrono.ChLinkLockRevolute()
    joint_ground.Initialize(bar0, ground, chrono.ChFramed(chrono.ChVector3d(-0.5, 0, 0)))
    sys.AddLink(joint_ground)

    joint_mid = chrono.ChLinkLockRevolute()
    joint_mid.Initialize(bar1, bar0, chrono.ChFramed(chrono.ChVector3d(0.5, 0, 0)))
    sys.AddLink(joint_mid)

    return sys, bar0, bar1


def simulate(duration, step):
    sys, bar0, bar1 = build_system()
    while sys.GetChTime() < duration:
        sys.DoStepDynamics(step)
    return sys, bar0, bar1


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    sys, bar0, bar1 = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: doublePendulum2D.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.8, 1.0, 3.0), chrono.ChVector3d(0.45, -0.35, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and sys.GetChTime() < duration:
        t = sys.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(step)
        if t >= next_log:
            print_state(sys, bar0, bar1)
            next_log += 0.5


def print_state(sys, bar0, bar1):
    print(
        f"t={sys.GetChTime():6.3f}  "
        f"bar0=({bar0.GetPos().x:+.4f}, {bar0.GetPos().y:+.4f})  "
        f"bar1=({bar1.GetPos().x:+.4f}, {bar1.GetPos().y:+.4f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: doublePendulum2D.py -> PyChrono revolute double pendulum")
    if args.no_vis:
        sys, bar0, bar1 = simulate(args.duration, args.step)
        print_state(sys, bar0, bar1)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

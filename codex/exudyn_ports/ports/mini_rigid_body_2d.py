import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN
# TestModels/MiniExamples/ObjectRigidBody2D.py:
# a planar rigid body starts at x=1.5 and angle pi/4 with velocities 0.5 and
# 0.75*pi. After 1 s, x=2 and angle=pi.

STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def build_system():
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    body = chrono.ChBodyEasyBox(0.42, 0.12, 0.035, 1000, True, False)
    body.SetMass(1.0)
    body.SetInertiaXX(chrono.ChVector3d(1, 1, 2))
    body.SetPos(chrono.ChVector3d(1.5, 1.0, 0))
    body.SetRot(chrono.QuatFromAngleZ(0.25 * math.pi))
    body.SetPosDt(chrono.ChVector3d(0.5, 0, 0))
    body.SetAngVelParent(chrono.ChVector3d(0, 0, 0.75 * math.pi))
    body.GetVisualShape(0).SetColor(color(0.12, 0.38, 0.88))
    sys.AddBody(body)

    marker = chrono.ChVisualShapeSphere(0.045)
    marker.SetColor(color(0.92, 0.22, 0.12))
    body.AddVisualShape(marker, chrono.ChFramed(chrono.ChVector3d(0.23, 0, 0)))

    return sys, body


def simulate(duration, step):
    sys, body = build_system()
    while sys.GetChTime() < duration:
        sys.DoStepDynamics(step)
    return sys, body


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    sys, body = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ObjectRigidBody2D.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.8, 1.35, 1.55), chrono.ChVector3d(1.75, 1.0, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and sys.GetChTime() < duration:
        t = sys.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(step)
        if t >= next_log:
            print_state(sys, body)
            next_log += 0.25


def print_state(sys, body):
    rz = body.GetRot().GetCardanAnglesXYZ().z
    theta = 0.25 * math.pi + 0.75 * math.pi * sys.GetChTime()
    print(
        f"t={sys.GetChTime():6.3f}  x={body.GetPos().x:+.8f}  "
        f"theta={theta:+.8f}  wrapped_rz={rz:+.8f}  result={body.GetPos().x + theta:+.8f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ObjectRigidBody2D.py -> PyChrono planar rigid body")
    if args.no_vis:
        sys, body = simulate(args.duration, args.step)
        print_state(sys, body)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

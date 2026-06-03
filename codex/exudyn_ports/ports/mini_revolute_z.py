import argparse

import pychrono.core as chrono


# Reproduces the intent of EXUDYN
# TestModels/MiniExamples/ObjectJointRevoluteZ.py:
# a rigid body constrained by a revolute-Z joint and driven by a constant
# torque around that joint axis.

TORQUE_Z = 1.0
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def build_system():
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    ground = chrono.ChBody()
    ground.SetFixed(True)
    sys.AddBody(ground)

    body = chrono.ChBody()
    body.SetMass(1.0)
    body.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
    body.SetPos(chrono.ChVector3d(0, 0, 0))
    body.EnableCollision(False)

    arm = chrono.ChVisualShapeBox(0.72, 0.08, 0.08)
    arm.SetColor(color(0.15, 0.42, 0.90))
    body.AddVisualShape(arm, chrono.ChFramed(chrono.ChVector3d(0.36, 0, 0)))
    hub = chrono.ChVisualShapeCylinder(0.10, 0.08)
    hub.SetColor(color(0.10, 0.10, 0.10))
    body.AddVisualShape(hub, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.Q_ROTATE_Y_TO_Z))
    tip = chrono.ChVisualShapeSphere(0.055)
    tip.SetColor(color(0.92, 0.22, 0.12))
    body.AddVisualShape(tip, chrono.ChFramed(chrono.ChVector3d(0.74, 0, 0)))
    sys.AddBody(body)

    joint = chrono.ChLinkLockRevolute()
    joint.Initialize(body, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
    joint.ForceRz().SetActive(True)
    joint.ForceRz().SetActuatorForceTorque(TORQUE_Z)
    sys.AddLink(joint)

    return sys, body, joint


def simulate(duration, step):
    sys, body, joint = build_system()
    while sys.GetChTime() < duration:
        sys.DoStepDynamics(step)
    return sys, body, joint


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    sys, body, joint = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ObjectJointRevoluteZ.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.6, 0.55, 1.75), chrono.ChVector3d(0.25, 0, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and sys.GetChTime() < duration:
        t = sys.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(step)
        if t >= next_log:
            print_state(sys, body, joint)
            next_log += 0.25


def print_state(sys, body, joint):
    angles = body.GetRot().GetCardanAnglesXYZ()
    print(
        f"t={sys.GetChTime():6.3f}  rz={angles.z:+.8f}  "
        f"wz={body.GetAngVelLocal().z:+.8f}  torque={TORQUE_Z:+.3f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ObjectJointRevoluteZ.py -> PyChrono revolute-Z torque test")
    if args.no_vis:
        sys, body, joint = simulate(args.duration, args.step)
        print_state(sys, body, joint)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

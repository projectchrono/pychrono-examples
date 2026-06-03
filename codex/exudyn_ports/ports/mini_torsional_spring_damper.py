import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN
# TestModels/MiniExamples/ObjectConnectorTorsionalSpringDamper.py:
# a rigid body rotates about a revolute-Z joint, resisted by a torsional
# spring-damper and loaded by a unit torque.

STIFFNESS = 2000.0
DAMPING = STIFFNESS * 0.01
TORQUE = 1.0
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def add_torsion_coil(sys):
    body = chrono.ChBody()
    body.SetFixed(True)
    body.EnableCollision(False)
    line = chrono.ChLinePoly(140)
    for i in range(140):
        s = i / 139.0
        angle = 7.0 * 2.0 * chrono.CH_PI * s
        radius = 0.07 + 0.12 * s
        line.SetPoint(i, chrono.ChVector3d(radius * math.cos(angle), radius * math.sin(angle), 0.06))
    shape = chrono.ChVisualShapeLine()
    shape.SetLineGeometry(line)
    shape.SetThickness(3)
    shape.SetColor(color(0.02, 0.02, 0.02))
    body.AddVisualShape(shape)
    sys.AddBody(body)
    return body


def build_system():
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    ground = chrono.ChBody()
    ground.SetFixed(True)
    sys.AddBody(ground)

    body = chrono.ChBody()
    body.SetMass(1.0)
    body.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
    body.EnableCollision(False)
    arm = chrono.ChVisualShapeBox(0.70, 0.08, 0.08)
    arm.SetColor(color(0.12, 0.38, 0.88))
    body.AddVisualShape(arm, chrono.ChFramed(chrono.ChVector3d(0.35, 0, 0)))
    tip = chrono.ChVisualShapeSphere(0.055)
    tip.SetColor(color(0.92, 0.22, 0.12))
    body.AddVisualShape(tip, chrono.ChFramed(chrono.ChVector3d(0.72, 0, 0)))
    sys.AddBody(body)

    frame = chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)
    revolute = chrono.ChLinkLockRevolute()
    revolute.Initialize(body, ground, frame)
    revolute.ForceRz().SetActive(True)
    revolute.ForceRz().SetActuatorForceTorque(TORQUE)
    sys.AddLink(revolute)

    spring = chrono.ChLinkRSDA()
    spring.Initialize(body, ground, frame)
    spring.SetRestAngle(0)
    spring.SetSpringCoefficient(STIFFNESS)
    spring.SetDampingCoefficient(DAMPING)
    rot_shape = chrono.ChVisualShapeRotSpring(0.22, 40)
    rot_shape.SetColor(color(0.02, 0.02, 0.02))
    spring.AddVisualShape(rot_shape)
    sys.AddLink(spring)
    add_torsion_coil(sys)

    return sys, body, revolute, spring


def simulate(duration, step):
    sys, body, revolute, spring = build_system()
    while sys.GetChTime() < duration:
        sys.DoStepDynamics(step)
    return sys, body, revolute, spring


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    sys, body, revolute, spring = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ObjectConnectorTorsionalSpringDamper.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.6, 0.6, 1.7), chrono.ChVector3d(0.25, 0, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and sys.GetChTime() < duration:
        t = sys.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(step)
        if t >= next_log:
            print_state(sys, body, spring)
            next_log += 0.25


def print_state(sys, body, spring):
    rz = body.GetRot().GetCardanAnglesXYZ().z
    print(f"t={sys.GetChTime():6.3f}  rz={rz:+.8f}  spring_torque={spring.GetTorque():+.6f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ObjectConnectorTorsionalSpringDamper.py -> PyChrono RSDA")
    if args.no_vis:
        sys, body, revolute, spring = simulate(args.duration, args.step)
        print_state(sys, body, spring)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

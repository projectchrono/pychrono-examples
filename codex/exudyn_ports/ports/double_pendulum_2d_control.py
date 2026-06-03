import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/doublePendulum2DControl.py:
# a two-link planar pendulum with controller torques driving the absolute first
# angle and relative second angle. This port uses Chrono angle motors to impose
# the same desired joint-angle profiles robustly.

LENGTH = 1.0
WIDTH = 0.055
MASS = 1.0
INERTIA = 1.0
OMEGA = 0.5
STEP = 1e-3
END_TIME = 5.0


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def desired_angle(sign, time):
    return sign * 0.5 * math.pi * math.sin(OMEGA * math.pi * time)


def desired_angle_function(sign):
    return chrono.ChFunctionSine(sign * 0.5 * math.pi, 0.5 * OMEGA)


def make_link(name, center, tint):
    body = chrono.ChBodyEasyBox(LENGTH, WIDTH, WIDTH, 1000, True, False)
    body.SetName(name)
    body.SetMass(MASS)
    body.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, INERTIA))
    body.SetPos(center)
    body.GetVisualShape(0).SetColor(tint)
    for x, marker_color in ((-0.5 * LENGTH, color(0.05, 0.05, 0.05)), (0.5 * LENGTH, color(0.96, 0.82, 0.08))):
        marker = chrono.ChVisualShapeSphere(0.045)
        marker.SetColor(marker_color)
        body.AddVisualShape(marker, chrono.ChFramed(chrono.ChVector3d(x, 0, 0)))
    return body


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    ground = chrono.ChBody()
    ground.SetName("controlled double pendulum ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)

    plate = chrono.ChBodyEasyBox(2.4, 2.4, 0.025, 1000, True, False)
    plate.SetName("visible controlled double pendulum reference plate")
    plate.SetFixed(True)
    plate.SetPos(chrono.ChVector3d(0.45, -0.75, -0.11))
    plate.GetVisualShape(0).SetColor(color(0.80, 0.80, 0.76))
    plate.GetVisualShape(0).SetOpacity(0.28)
    system.AddBody(plate)

    support = chrono.ChBodyEasySphere(0.055, 1000, True, False)
    support.SetName("controlled double pendulum support marker")
    support.SetFixed(True)
    support.SetPos(chrono.ChVector3d(-0.5, 0, 0))
    support.GetVisualShape(0).SetColor(color(0.05, 0.05, 0.05))
    system.AddBody(support)

    link0 = make_link("controlled double pendulum link 1", chrono.ChVector3d(0, 0, 0), color(0.10, 0.35, 0.90))
    link1 = make_link("controlled double pendulum link 2", chrono.ChVector3d(1, 0, 0), color(0.88, 0.16, 0.10))
    system.AddBody(link0)
    system.AddBody(link1)

    motor0 = chrono.ChLinkMotorRotationAngle()
    motor0.SetName("controlled double pendulum absolute angle motor")
    motor0.Initialize(link0, ground, chrono.ChFramed(chrono.ChVector3d(-0.5, 0, 0), chrono.QUNIT))
    motor0.SetAngleFunction(desired_angle_function(+1.0))
    system.AddLink(motor0)

    motor1 = chrono.ChLinkMotorRotationAngle()
    motor1.SetName("controlled double pendulum relative angle motor")
    motor1.Initialize(link1, link0, chrono.ChFramed(chrono.ChVector3d(0.5, 0, 0), chrono.QUNIT))
    motor1.SetAngleFunction(desired_angle_function(-1.0))
    system.AddLink(motor1)

    system._double_pendulum_control_items = {"link0": link0, "link1": link1, "motor0": motor0, "motor1": motor1}
    return system, link0, link1, (motor0, motor1)


def simulate(duration, step):
    system, link0, link1, motors = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, link0, link1, motors


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, link0, link1, motors = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: doublePendulum2DControl.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.7, 0.8, 3.3), chrono.ChVector3d(0.45, -0.75, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, link0, link1)
            next_log += 0.5


def yaw_z(body):
    q = body.GetRot()
    return math.atan2(2.0 * (q.e0 * q.e3 + q.e1 * q.e2), 1.0 - 2.0 * (q.e2 * q.e2 + q.e3 * q.e3))


def print_state(system, link0, link1):
    phi0 = yaw_z(link0)
    phi1 = yaw_z(link1)
    desired0 = desired_angle(+1.0, system.GetChTime())
    desired_rel = desired_angle(-1.0, system.GetChTime())
    tip = link1.TransformPointLocalToParent(chrono.ChVector3d(0.5 * LENGTH, 0, 0))
    print(
        f"t={system.GetChTime():6.3f}  "
        f"phi0={phi0:+.5f}  rel={phi1 - phi0:+.5f}  "
        f"desired=({desired0:+.5f}, {desired_rel:+.5f})  "
        f"tip=({tip.x:+.4f}, {tip.y:+.4f}, {tip.z:+.4f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: doublePendulum2DControl.py -> PyChrono controlled double pendulum")
    if args.no_vis:
        system, link0, link1, motors = simulate(args.duration, args.step)
        print_state(system, link0, link1)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

import argparse

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/sphericalJointTest.py:
# four 3D rigid bodies connected as a chain with spherical joints. The first
# connector follows the source's constrainedAxes=[0,1,1] behavior with a
# point-on-line joint, while the remaining connectors are spherical joints.

N_BODIES = 4
S = 0.1
HALF_LENGTH = 3.0 * S
BODY_LENGTH = 2.0 * HALF_LENGTH
BODY_WIDTH = 2.0 * S
JOINT_Z_OFFSET = 0.1
MASS = 2.0
INERTIA = chrono.ChVector3d(6.0, 1.0, 6.0)
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_box_body(index, center):
    body = chrono.ChBodyEasyBox(BODY_LENGTH, BODY_WIDTH, BODY_WIDTH, 1000, True, False)
    body.SetName(f"spherical joint chain body {index + 1}")
    body.SetMass(MASS)
    body.SetInertiaXX(INERTIA)
    body.SetPos(center)
    body.GetVisualShape(0).SetColor(color(0.82, 0.14, 0.10))

    left_marker = chrono.ChVisualShapeSphere(0.035)
    left_marker.SetColor(color(0.96, 0.82, 0.08))
    body.AddVisualShape(left_marker, chrono.ChFramed(chrono.ChVector3d(-HALF_LENGTH, 0, JOINT_Z_OFFSET)))

    right_marker = chrono.ChVisualShapeSphere(0.035)
    right_marker.SetColor(color(0.06, 0.06, 0.06))
    body.AddVisualShape(right_marker, chrono.ChFramed(chrono.ChVector3d(HALF_LENGTH, 0, JOINT_Z_OFFSET)))
    return body


def make_joint_marker(system, point, name, tint):
    marker = chrono.ChBodyEasySphere(0.045, 1000, True, False)
    marker.SetName(name)
    marker.SetFixed(True)
    marker.SetPos(point)
    marker.GetVisualShape(0).SetColor(tint)
    system.AddBody(marker)
    return marker


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    ground = chrono.ChBody()
    ground.SetName("spherical chain ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)

    plate = chrono.ChBodyEasyBox(3.2, 1.2, 0.025, 1000, True, False)
    plate.SetName("visible spherical joint chain reference plate")
    plate.SetFixed(True)
    plate.SetPos(chrono.ChVector3d(0.65, -0.12, -0.24))
    plate.GetVisualShape(0).SetColor(color(0.80, 0.80, 0.76))
    plate.GetVisualShape(0).SetOpacity(0.28)
    system.AddBody(plate)

    bodies = []
    for i in range(N_BODIES):
        center = chrono.ChVector3d(-HALF_LENGTH + i * 2.0 * HALF_LENGTH, 0.0, 0.0)
        body = make_box_body(i, center)
        system.AddBody(body)
        bodies.append(body)

    support_point = chrono.ChVector3d(-HALF_LENGTH, 0.0, 0.0)
    make_joint_marker(system, support_point, "point-line support marker", color(0.05, 0.05, 0.05))

    support = chrono.ChLinkLockPointLine()
    support.SetName("first body point-line support")
    support.Initialize(bodies[0], ground, chrono.ChFramed(support_point, chrono.QUNIT))
    system.AddLink(support)

    joints = [support]
    for i in range(1, N_BODIES):
        joint_point = chrono.ChVector3d(-HALF_LENGTH + i * 2.0 * HALF_LENGTH - HALF_LENGTH, 0.0, JOINT_Z_OFFSET)
        joint = chrono.ChLinkLockSpherical()
        joint.SetName(f"spherical joint {i}")
        joint.Initialize(bodies[i], bodies[i - 1], chrono.ChFramed(joint_point, chrono.QUNIT))
        system.AddLink(joint)
        joints.append(joint)

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
    vis.SetWindowTitle("EXUDYN port: sphericalJointTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.9, -1.7, 1.1), chrono.ChVector3d(0.6, -0.2, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, bodies)
            next_log += 0.25


def print_state(system, bodies):
    first = bodies[0].GetPos()
    last = bodies[-1].GetPos()
    total = 0.0
    for body in bodies[:2]:
        pos = body.GetPos()
        rot = body.GetRot()
        total += abs(pos.x) + abs(pos.y) + abs(pos.z) + abs(rot.e0) + abs(rot.e1) + abs(rot.e2) + abs(rot.e3)
    print(
        f"t={system.GetChTime():6.3f}  "
        f"first=({first.x:+.4f}, {first.y:+.4f}, {first.z:+.4f})  "
        f"last=({last.x:+.4f}, {last.y:+.4f}, {last.z:+.4f})  "
        f"chain_metric={total:.6f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: sphericalJointTest.py -> PyChrono spherical joint chain")
    if args.no_vis:
        system, bodies, joints = simulate(args.duration, args.step)
        print_state(system, bodies)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

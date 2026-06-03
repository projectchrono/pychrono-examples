import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN Examples/FurtherExamples/fourBarKinematicTreeRobot.py:
# a four-bar mechanism generated through EXUDYN's Robot helper, then converted
# to ObjectKinematicTree with a closing RevoluteJointZ at the tip.  PyChrono
# represents the same source geometry as explicit rigid links with revolute
# loop joints, source gravity loads, and the constant base torque.

L1 = 0.5
L2 = 1.0
L3 = 1.0
WIDTH = 0.05
DENSITY = 1000.0
GRAVITY = -9.81
TORQUE_Z = 1.0
STEP = 2.0e-3
END_TIME = 8.0


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def midpoint(a, b):
    return chrono.ChVector3d(0.5 * (a.x + b.x), 0.5 * (a.y + b.y), 0.5 * (a.z + b.z))


def angle_between(a, b):
    return math.atan2(b.y - a.y, b.x - a.x)


def source_points():
    # Robot preHT rotations [0, -pi/2, -pi/2] produce the same rectangle as
    # the manual kinematic-tree four-bar: vertical crank, horizontal coupler,
    # and vertical closing rocker to the ground marker at [1, -0.5, 0].
    p0 = chrono.ChVector3d(0, 0, 0)
    p1 = chrono.ChVector3d(0, L1, 0)
    p2 = chrono.ChVector3d(L2, L1, 0)
    p3 = chrono.ChVector3d(L2, L1 - L3, 0)
    return p0, p1, p2, p3


def make_ground(system, points):
    ground = chrono.ChBody()
    ground.SetName("robot-helper kinematic-tree ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    p0, _, _, p3 = points
    base = chrono.ChVisualShapeBox(1.35, 0.055, 0.055)
    base.SetColor(color(0.36, 0.36, 0.38))
    ground.AddVisualShape(base, chrono.ChFramed(midpoint(p0, p3)))

    base_cube = chrono.ChVisualShapeBox(0.05, 0.05, 0.05)
    base_cube.SetColor(color(0.50, 0.50, 0.52))
    ground.AddVisualShape(base_cube, chrono.ChFramed(chrono.ChVector3d(0, 0, -0.07)))

    for point in (p0, p3):
        sphere = chrono.ChVisualShapeSphere(0.055)
        sphere.SetColor(color(0.04, 0.04, 0.045))
        ground.AddVisualShape(sphere, chrono.ChFramed(point))

        axis = chrono.ChVisualShapeCylinder(0.018, 0.18)
        axis.SetColor(color(0.72, 0.72, 0.74))
        ground.AddVisualShape(axis, chrono.ChFramed(point, chrono.QUNIT))

    system.AddBody(ground)
    return ground


def add_body_markers(body, length):
    for x, tint in ((-0.5 * length, color(0.04, 0.04, 0.045)), (0.5 * length, color(0.96, 0.72, 0.08))):
        sphere = chrono.ChVisualShapeSphere(0.040)
        sphere.SetColor(tint)
        body.AddVisualShape(sphere, chrono.ChFramed(chrono.ChVector3d(x, 0, 0)))

    axis = chrono.ChVisualShapeCylinder(0.014, 0.16)
    axis.SetColor(color(0.70, 0.70, 0.72))
    body.AddVisualShape(axis, chrono.ChFramed(chrono.ChVector3d(-0.5 * length, 0, 0), chrono.QUNIT))


def make_link(system, name, p0, p1, length, tint):
    body = chrono.ChBodyEasyBox(length, WIDTH, WIDTH, DENSITY, True, False)
    body.SetName(name)
    body.EnableCollision(False)
    body.SetPos(midpoint(p0, p1))
    body.SetRot(chrono.QuatFromAngleZ(angle_between(p0, p1)))
    body.GetVisualShape(0).SetColor(tint)
    add_body_markers(body, length)
    system.AddBody(body)
    return body


def add_revolute(system, name, body_a, body_b, point):
    joint = chrono.ChLinkLockRevolute()
    joint.SetName(name)
    joint.Initialize(body_a, body_b, chrono.ChFramed(point, chrono.QUNIT))
    system.AddLink(joint)
    return joint


def make_axis_marker(system, name, point, radius=0.024):
    marker = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, radius, 0.22, 1000, True, False)
    marker.SetName(name)
    marker.SetFixed(True)
    marker.EnableCollision(False)
    marker.SetPos(point)
    marker.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.09))
    system.AddBody(marker)
    return marker


def make_torque_indicator(system, point):
    indicator = chrono.ChBody()
    indicator.SetName("constant base-torque indicator")
    indicator.SetFixed(True)
    indicator.EnableCollision(False)

    hub = chrono.ChVisualShapeCylinder(0.032, 0.28)
    hub.SetColor(color(0.92, 0.22, 0.12))
    indicator.AddVisualShape(hub, chrono.ChFramed(point + chrono.ChVector3d(0, 0, 0.02), chrono.QUNIT))

    arrow = chrono.ChVisualShapeBox(0.15, 0.024, 0.024)
    arrow.SetColor(color(0.92, 0.22, 0.12))
    indicator.AddVisualShape(arrow, chrono.ChFramed(point + chrono.ChVector3d(0.10, 0.08, 0.14), chrono.QuatFromAngleZ(0.55)))

    tip = chrono.ChVisualShapeCone(0.048, 0.08)
    tip.SetColor(color(0.92, 0.22, 0.12))
    indicator.AddVisualShape(tip, chrono.ChFramed(point + chrono.ChVector3d(0.17, 0.14, 0.14), chrono.QuatFromAngleZ(0.55)))

    system.AddBody(indicator)
    return indicator


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, GRAVITY, 0))

    points = source_points()
    p0, p1, p2, p3 = points
    ground = make_ground(system, points)

    link1 = make_link(system, "robot-helper red crank", p0, p1, L1, color(0.92, 0.22, 0.12))
    link2 = make_link(system, "robot-helper blue coupler", p1, p2, L2, color(0.12, 0.42, 0.85))
    link3 = make_link(system, "robot-helper blue rocker", p2, p3, L3, color(0.12, 0.42, 0.85))

    base_joint = add_revolute(system, "constant-torque base revolute Z", link1, ground, p0)
    base_joint.ForceRz().SetActive(True)
    base_joint.ForceRz().SetActuatorForceTorque(TORQUE_Z)
    joints = [
        base_joint,
        add_revolute(system, "tree joint 1 revolute Z", link2, link1, p1),
        add_revolute(system, "tree joint 2 revolute Z", link3, link2, p2),
        add_revolute(system, "robot-helper closing RevoluteJointZ analogue", link3, ground, p3),
    ]

    for index, point in enumerate(points):
        make_axis_marker(system, f"visible robot-helper pin axis {index + 1}", point)
    torque_indicator = make_torque_indicator(system, p0)

    items = {
        "ground": ground,
        "links": (link1, link2, link3),
        "joints": joints,
        "base_joint": base_joint,
        "points": points,
        "torque_indicator": torque_indicator,
    }
    system._four_bar_robot_items = items
    return system, items


def local_endpoint(body, local_x):
    return body.TransformPointLocalToParent(chrono.ChVector3d(local_x, 0, 0))


def loop_error(items):
    link3 = items["links"][2]
    p3 = items["points"][3]
    current = local_endpoint(link3, 0.5 * L3)
    return (current - p3).Length()


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, items


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: fourBarKinematicTreeRobot.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.50, -2.35, 2.10), chrono.ChVector3d(0.50, 0.0, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, items)
            next_log += 0.5


def print_state(system, items):
    link1 = items["links"][0]
    link2 = items["links"][1]
    omega_z = link1.GetAngVelParent().z
    top = link2.GetPos()
    print(
        f"t={system.GetChTime():6.3f}  "
        f"omega_z={omega_z:+.6f}  "
        f"torque_z={TORQUE_Z:+.6f}  "
        f"coupler_mid=({top.x:+.6f},{top.y:+.6f},{top.z:+.6f})  "
        f"loop_error={loop_error(items):.3e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: fourBarKinematicTreeRobot.py -> PyChrono explicit torque-driven four-bar")
    if args.no_vis:
        system, items = simulate(args.duration, args.step)
        print_state(system, items)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

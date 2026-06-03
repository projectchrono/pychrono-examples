import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN Examples/FurtherExamples/fourBarKinematicTreeGravity.py:
# a three-link four-bar mechanism represented in EXUDYN as an ObjectKinematicTree
# with a closing-loop GenericJoint.  PyChrono does not need the tree helper here:
# the same mechanism is represented as explicit rigid bodies with revolute pin
# joints at the physical loop points, preserving the source link dimensions,
# initial pose, and gravity/disturbance vector.

L1 = 0.5
L2 = 1.0
L3 = 1.0
WIDTH = 0.05
DENSITY = 1000.0
GRAVITY_X = 0.5
GRAVITY_Y = -9.81
STEP = 5.0e-3
END_TIME = 10.0


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def midpoint(a, b):
    return chrono.ChVector3d(0.5 * (a.x + b.x), 0.5 * (a.y + b.y), 0.5 * (a.z + b.z))


def angle_between(a, b):
    return math.atan2(b.y - a.y, b.x - a.x)


def make_ground(system, points):
    ground = chrono.ChBody()
    ground.SetName("kinematic-tree four-bar ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    p0, _, _, p3 = points
    base = chrono.ChVisualShapeBox(1.35, 0.055, 0.055)
    base.SetColor(color(0.36, 0.36, 0.38))
    ground.AddVisualShape(base, chrono.ChFramed(midpoint(p0, p3)))

    for name, point in (("left ground pin", p0), ("right ground pin", p3)):
        sphere = chrono.ChVisualShapeSphere(0.055)
        sphere.SetColor(color(0.04, 0.04, 0.045))
        ground.AddVisualShape(sphere, chrono.ChFramed(point))

        axis = chrono.ChVisualShapeCylinder(0.018, 0.18)
        axis.SetColor(color(0.72, 0.72, 0.74))
        ground.AddVisualShape(axis, chrono.ChFramed(point, chrono.QUNIT))

        marker = chrono.ChVisualShapeBox(0.05, 0.05, 0.012)
        marker.SetColor(color(0.96, 0.72, 0.08))
        ground.AddVisualShape(marker, chrono.ChFramed(point + chrono.ChVector3d(0, 0, 0.105)))

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


def source_points():
    p0 = chrono.ChVector3d(0, 0, 0)
    p1 = chrono.ChVector3d(0, L1, 0)
    p2 = chrono.ChVector3d(L2, L1, 0)
    p3 = chrono.ChVector3d(L2, L1 - L3, 0)
    return p0, p1, p2, p3


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(GRAVITY_X, GRAVITY_Y, 0))

    points = source_points()
    p0, p1, p2, p3 = points
    ground = make_ground(system, points)

    link1 = make_link(system, "kinematic-tree link 1 short crank", p0, p1, L1, color(0.12, 0.42, 0.85))
    link2 = make_link(system, "kinematic-tree link 2 top coupler", p1, p2, L2, color(0.10, 0.62, 0.28))
    link3 = make_link(system, "kinematic-tree link 3 closing rocker", p2, p3, L3, color(0.92, 0.22, 0.12))

    joints = [
        add_revolute(system, "tree base revolute Z", link1, ground, p0),
        add_revolute(system, "tree joint 1 revolute Z", link2, link1, p1),
        add_revolute(system, "tree joint 2 revolute Z", link3, link2, p2),
        add_revolute(system, "tree closing-loop generic joint analogue", link3, ground, p3),
    ]

    for index, point in enumerate(points):
        make_axis_marker(system, f"visible kinematic-tree joint axis {index + 1}", point)

    system._four_bar_kt_items = {"ground": ground, "links": (link1, link2, link3), "joints": joints, "points": points}
    return system, system._four_bar_kt_items


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
    vis.SetWindowTitle("EXUDYN port: fourBarKinematicTreeGravity.py")
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
    link2 = items["links"][1]
    top = link2.GetPos()
    print(
        f"t={system.GetChTime():6.3f}  "
        f"coupler_mid=({top.x:+.6f},{top.y:+.6f},{top.z:+.6f})  "
        f"loop_error={loop_error(items):.3e}  links={len(items['links'])}  joints={len(items['joints'])}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: fourBarKinematicTreeGravity.py -> PyChrono explicit four-bar")
    if args.no_vis:
        system, items = simulate(args.duration, args.step)
        print_state(system, items)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

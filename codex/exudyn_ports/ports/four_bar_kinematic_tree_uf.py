import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN Examples/FurtherExamples/fourBarKinematicTreeUF.py:
# a Robot-generated ObjectKinematicTree with three slender links, a fourth cube
# link, a GenericJoint that fixes an off-center TCP marker back to ground, and a
# torque user function at the first joint.  PyChrono represents the same closed
# loop with explicit visible bodies and revolute joints; the TCP marker closure
# is represented by a rigid lock to ground at the source marker point.

L1 = 0.5
L2 = 1.0
L3 = 1.0
WIDTH = 0.05
DENSITY = 1000.0
GRAVITY = -9.81
TARGET_OMEGA_Z = -20.0
CONTROL_GAIN = 100.0
STEP = 1.0e-3
END_TIME = 2.0


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def midpoint(a, b):
    return chrono.ChVector3d(0.5 * (a.x + b.x), 0.5 * (a.y + b.y), 0.5 * (a.z + b.z))


def angle_between(a, b):
    return math.atan2(b.y - a.y, b.x - a.x)


def source_points():
    # Using EXUDYN's Robot preHT chain with q=[0, pi/2, pi/2, 0] gives a
    # left-closing rectangular chain.  The fourth cube's TCP marker is at local
    # [-1, -0.5, 0], which returns to the ground origin from the cube center.
    p0 = chrono.ChVector3d(0.0, 0.0, 0.0)
    p1 = chrono.ChVector3d(0.0, L1, 0.0)
    p2 = chrono.ChVector3d(-L2, L1, 0.0)
    p3 = chrono.ChVector3d(-L2, L1 - L3, 0.0)
    tcp = chrono.ChVector3d(0.0, 0.0, 0.0)
    return p0, p1, p2, p3, tcp


def make_ground(system, points):
    ground = chrono.ChBody()
    ground.SetName("UF kinematic-tree ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    p0, _, _, p3, tcp = points
    base = chrono.ChVisualShapeBox((p3 - p0).Length(), 0.050, 0.050)
    base.SetColor(color(0.36, 0.36, 0.38))
    base_angle = angle_between(p0, p3)
    ground.AddVisualShape(base, chrono.ChFramed(midpoint(p0, p3), chrono.QuatFromAngleZ(base_angle)))

    base_cube = chrono.ChVisualShapeBox(0.038, 0.038, 0.038)
    base_cube.SetColor(color(0.50, 0.50, 0.52))
    ground.AddVisualShape(base_cube, chrono.ChFramed(chrono.ChVector3d(0, 0, -0.07)))

    for point in (p0, p3, tcp):
        sphere = chrono.ChVisualShapeSphere(0.052)
        sphere.SetColor(color(0.04, 0.04, 0.045))
        ground.AddVisualShape(sphere, chrono.ChFramed(point))

        axis = chrono.ChVisualShapeCylinder(0.017, 0.18)
        axis.SetColor(color(0.72, 0.72, 0.74))
        ground.AddVisualShape(axis, chrono.ChFramed(point, chrono.QUNIT))

    system.AddBody(ground)
    return ground


def add_body_markers(body, length):
    for x, tint in ((-0.5 * length, color(0.04, 0.04, 0.045)), (0.5 * length, color(0.96, 0.72, 0.08))):
        sphere = chrono.ChVisualShapeSphere(0.038)
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


def make_tcp_cube(system, center):
    body = chrono.ChBodyEasyBox(1.0, 1.0, 1.0, DENSITY, True, False)
    body.SetName("UF fourth cube link and arm")
    body.EnableCollision(False)
    body.SetPos(center)
    body.SetRot(chrono.QuatFromAngleZ(math.pi))
    body.GetVisualShape(0).SetColor(color(0.12, 0.42, 0.85))
    body.GetVisualShape(0).SetOpacity(0.28)

    arm = chrono.ChVisualShapeBox(math.sqrt(1.25), 0.030, 0.030)
    arm.SetColor(color(0.95, 0.56, 0.08))
    local_mid = chrono.ChVector3d(-0.5, -0.25, 0.0)
    local_angle = math.atan2(-0.5, -1.0)
    body.AddVisualShape(arm, chrono.ChFramed(local_mid, chrono.QuatFromAngleZ(local_angle)))

    for local, tint in (
        (chrono.ChVector3d(0, 0, 0), color(0.04, 0.04, 0.045)),
        (chrono.ChVector3d(-1, -0.5, 0), color(0.96, 0.72, 0.08)),
    ):
        marker = chrono.ChVisualShapeSphere(0.045)
        marker.SetColor(tint)
        body.AddVisualShape(marker, chrono.ChFramed(local))

    system.AddBody(body)
    return body


def add_revolute(system, name, body_a, body_b, point):
    joint = chrono.ChLinkLockRevolute()
    joint.SetName(name)
    joint.Initialize(body_a, body_b, chrono.ChFramed(point, chrono.QUNIT))
    system.AddLink(joint)
    return joint


def add_lock(system, name, body_a, body_b, point, rotation):
    lock = chrono.ChLinkLockLock()
    lock.SetName(name)
    lock.Initialize(body_a, body_b, chrono.ChFramed(point, rotation))
    system.AddLink(lock)
    return lock


def make_axis_marker(system, name, point, radius=0.022):
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
    indicator.SetName("UF negative-speed torque indicator")
    indicator.SetFixed(True)
    indicator.EnableCollision(False)

    hub = chrono.ChVisualShapeCylinder(0.032, 0.28)
    hub.SetColor(color(0.92, 0.22, 0.12))
    indicator.AddVisualShape(hub, chrono.ChFramed(point + chrono.ChVector3d(0, 0, 0.02), chrono.QUNIT))

    arrow = chrono.ChVisualShapeBox(0.17, 0.026, 0.026)
    arrow.SetColor(color(0.92, 0.22, 0.12))
    indicator.AddVisualShape(arrow, chrono.ChFramed(point + chrono.ChVector3d(-0.12, 0.08, 0.14), chrono.QuatFromAngleZ(-0.55)))

    tip = chrono.ChVisualShapeCone(0.052, 0.085)
    tip.SetColor(color(0.92, 0.22, 0.12))
    indicator.AddVisualShape(tip, chrono.ChFramed(point + chrono.ChVector3d(-0.21, 0.13, 0.14), chrono.QuatFromAngleZ(-0.55)))

    system.AddBody(indicator)
    return indicator


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, GRAVITY, 0))

    points = source_points()
    p0, p1, p2, p3, tcp = points
    ground = make_ground(system, points)

    link1 = make_link(system, "UF robot link 1", p0, p1, L1, color(0.12, 0.42, 0.85))
    link2 = make_link(system, "UF robot link 2", p1, p2, L2, color(0.12, 0.42, 0.85))
    link3 = make_link(system, "UF robot link 3", p2, p3, L3, color(0.12, 0.42, 0.85))
    cube = make_tcp_cube(system, p3)

    base_joint = add_revolute(system, "UF base revolute Z with user torque", link1, ground, p0)
    base_joint.ForceRz().SetActive(True)
    joints = [
        base_joint,
        add_revolute(system, "UF tree joint 1 revolute Z", link2, link1, p1),
        add_revolute(system, "UF tree joint 2 revolute Z", link3, link2, p2),
        add_revolute(system, "UF fourth-link revolute Z", cube, link3, p3),
    ]

    tcp_lock = add_lock(system, "UF GenericJoint x/y/Rz TCP closure analogue", cube, ground, tcp, chrono.QuatFromAngleZ(math.pi))
    for index, point in enumerate((p0, p1, p2, p3, tcp)):
        make_axis_marker(system, f"visible UF joint/TCP axis {index + 1}", point)
    torque_indicator = make_torque_indicator(system, p0)

    items = {
        "ground": ground,
        "links": (link1, link2, link3, cube),
        "joints": joints,
        "tcp_lock": tcp_lock,
        "points": points,
        "torque_indicator": torque_indicator,
        "last_torque": 0.0,
    }
    system._four_bar_uf_items = items
    update_user_torque(items)
    return system, items


def update_user_torque(items):
    omega_z = items["links"][0].GetAngVelParent().z
    torque_z = CONTROL_GAIN * (TARGET_OMEGA_Z - omega_z)
    items["joints"][0].ForceRz().SetActuatorForceTorque(torque_z)
    items["last_torque"] = torque_z
    return torque_z


def local_endpoint(body, local_x):
    return body.TransformPointLocalToParent(chrono.ChVector3d(local_x, 0, 0))


def tcp_position(items):
    cube = items["links"][3]
    return cube.TransformPointLocalToParent(chrono.ChVector3d(-1, -0.5, 0))


def loop_error(items):
    tcp = items["points"][4]
    return (tcp_position(items) - tcp).Length()


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration:
        update_user_torque(items)
        system.DoStepDynamics(step)
    update_user_torque(items)
    return system, items


def update_visuals(system):
    update_user_torque(system._four_bar_uf_items)


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: fourBarKinematicTreeUF.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(-0.55, -2.45, 2.25), chrono.ChVector3d(-0.55, 0.0, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        update_user_torque(items)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, items)
            next_log += 0.25


def print_state(system, items):
    link1 = items["links"][0]
    cube = items["links"][3]
    omega_z = link1.GetAngVelParent().z
    cube_pos = cube.GetPos()
    tcp = tcp_position(items)
    print(
        f"t={system.GetChTime():6.3f}  "
        f"omega_z={omega_z:+.6f}  "
        f"torque_z={items['last_torque']:+.6f}  "
        f"cube=({cube_pos.x:+.6f},{cube_pos.y:+.6f},{cube_pos.z:+.6f})  "
        f"tcp=({tcp.x:+.6f},{tcp.y:+.6f},{tcp.z:+.6f})  "
        f"loop_error={loop_error(items):.3e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: fourBarKinematicTreeUF.py -> PyChrono explicit user-torque four-bar")
    if args.no_vis:
        system, items = simulate(args.duration, args.step)
        print_state(system, items)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

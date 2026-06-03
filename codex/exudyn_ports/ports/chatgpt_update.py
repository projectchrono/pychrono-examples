import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_segment_visual, attach_spring_visual, update_system_visuals


# Reproduces the executable model embedded in EXUDYN Examples/chatGPTupdate.py:
# a documented mainSystemExtensions scene with rigid bodies, mass points,
# distance constraints, revolute/prismatic joints, loads, and spring-dampers.
# The PyChrono port keeps those ingredients in one inspectable scene. Springs
# use native ChVisualShapeSpring coils plus the shared helical capture fallback.

STEP = 1.0e-3
END_TIME = 5.0


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def add(a, b):
    return chrono.ChVector3d(a.x + b.x, a.y + b.y, a.z + b.z)


def scale(v, s):
    return chrono.ChVector3d(v.x * s, v.y * s, v.z * s)


def joint_frame_at(point, axis):
    axis_v = chrono.ChVector3d(axis.x, axis.y, axis.z)
    axis_v.Normalize()
    z_axis = chrono.ChVector3d(0, 0, 1)
    dot = max(-1.0, min(1.0, z_axis.Dot(axis_v)))
    cross = z_axis.Cross(axis_v)
    if cross.Length() < 1e-12:
        rot = chrono.QUNIT if dot > 0 else chrono.QuatFromAngleX(math.pi)
    else:
        cross.Normalize()
        rot = chrono.QuatFromAngleAxis(math.acos(dot), cross)
    return chrono.ChFramed(point, rot)


def diagonal_matrix(values):
    matrix = chrono.ChMatrix66d()
    matrix.SetZero()
    for i, value in enumerate(values):
        matrix.SetItem(i, i, value)
    return matrix


def add_local_cylinder(body, point_a, point_b, radius, tint):
    segment = chrono.ChLineSegment(point_a, point_b)
    shape = chrono.ChVisualShapeCylinder(radius, segment.GetLength())
    shape.SetColor(tint)
    body.AddVisualShape(shape, segment.GetFrame())


def add_body_axes(body, length=0.18, radius=0.006):
    add_local_cylinder(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(length, 0, 0), radius, color(0.92, 0.12, 0.08))
    add_local_cylinder(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, length, 0), radius, color(0.08, 0.68, 0.18))
    add_local_cylinder(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, length), radius, color(0.12, 0.28, 0.92))


def make_mutable_segment(system, name, tint, thickness):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    shape = chrono.ChVisualShapeSegment()
    shape.SetMutable(True)
    shape.SetColor(tint)
    shape.SetThickness(thickness)
    body.AddVisualShape(shape)
    system.AddBody(body)
    return body, shape


def set_segment(body, shape, point_a, point_b):
    shape.SetLineGeometry(chrono.ChLineSegment(point_a, point_b))
    body.UpdateVisualModel()


def make_box(system, name, size, density, pos, rot, tint):
    body = chrono.ChBodyEasyBox(size[0], size[1], size[2], density, True, False)
    body.SetName(name)
    body.EnableCollision(False)
    body.SetPos(pos)
    body.SetRot(rot)
    body.GetVisualShape(0).SetColor(tint)
    add_body_axes(body)
    system.AddBody(body)
    return body


def make_sphere(system, name, radius, mass, pos, tint, fixed=False):
    body = chrono.ChBodyEasySphere(radius, 1000, True, False)
    body.SetName(name)
    body.SetFixed(fixed)
    body.EnableCollision(False)
    body.SetPos(pos)
    if not fixed:
        body.SetMass(mass)
        body.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def distance_between(body_a, local_a, body_b, local_b):
    pa = body_a.TransformPointLocalToParent(local_a)
    pb = body_b.TransformPointLocalToParent(local_b)
    return math.sqrt((pa.x - pb.x) ** 2 + (pa.y - pb.y) ** 2 + (pa.z - pb.z) ** 2)


def add_distance(system, name, body_a, body_b, local_a, local_b):
    link = chrono.ChLinkDistance()
    link.SetName(name)
    link.Initialize(
        body_a,
        body_b,
        True,
        local_a,
        local_b,
        False,
        distance_between(body_a, local_a, body_b, local_b),
    )
    system.AddLink(link)
    attach_segment_visual(system, link, color(0.05, 0.05, 0.06), 3)
    return link


def add_tsda(system, name, body_a, body_b, local_a, local_b, stiffness, damping, radius, turns, tint):
    spring = chrono.ChLinkTSDA()
    spring.SetName(name)
    spring.Initialize(body_a, body_b, True, local_a, local_b)
    spring.SetRestLength(spring.GetLength())
    spring.SetSpringCoefficient(stiffness)
    spring.SetDampingCoefficient(damping)
    system.AddLink(spring)
    shape = chrono.ChVisualShapeSpring(radius, 96, turns)
    shape.SetColor(tint)
    spring.AddVisualShape(shape)
    fallback = attach_spring_visual(system, spring, radius, 96, turns, tint)
    fallback.shape.SetThickness(4)
    return spring


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    ground = chrono.ChBody()
    ground.SetName("ChatGPT update ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)

    plate = chrono.ChBodyEasyBox(8.8, 4.6, 0.035, 1000, True, False)
    plate.SetName("ChatGPT update checkerboard reference")
    plate.SetFixed(True)
    plate.EnableCollision(False)
    plate.SetPos(chrono.ChVector3d(3.5, 0.0, -0.23))
    plate.GetVisualShape(0).SetColor(color(0.78, 0.78, 0.74))
    plate.GetVisualShape(0).SetOpacity(0.28)
    system.AddBody(plate)

    ground_sphere = make_sphere(system, "ChatGPT update red ground marker", 0.10, 1.0, chrono.ChVector3d(3, 1, 0), color(0.90, 0.05, 0.04), True)

    b0 = make_box(
        system,
        "ChatGPT update loaded rigid body b0",
        (1.0, 0.1, 0.1),
        5000,
        chrono.ChVector3d(0.5, 0, 0),
        chrono.QuatFromAngleZ(0.5 * math.pi),
        color(0.95, 0.52, 0.08),
    )
    b0.SetPosDt(chrono.ChVector3d(0, 4, 0))
    b0.SetAngVelParent(chrono.ChVector3d(2, 0, 0))

    m1 = make_sphere(system, "ChatGPT update free mass point m1", 0.20, 1.0, chrono.ChVector3d(1, -1, 0), color(0.36, 0.36, 0.40))
    m1.SetPosDt(chrono.ChVector3d(2, 5, 0))

    dist_ground_b0 = add_distance(
        system,
        "ChatGPT update ground-to-b0 distance constraint",
        b0,
        ground,
        chrono.ChVector3d(-0.5, 0, 0),
        chrono.ChVector3d(0, 0, 0),
    )
    dist_b0_m1 = add_distance(
        system,
        "ChatGPT update b0-to-m1 distance constraint",
        b0,
        m1,
        chrono.ChVector3d(0.5, 0, 0),
        chrono.ChVector3d(0, 0, 0),
    )

    b1 = make_box(system, "ChatGPT update prismatic body b1", (1.0, 0.1, 0.1), 5000, chrono.ChVector3d(2.5, 0, 0), chrono.QUNIT, color(0.12, 0.42, 0.90))
    b2 = make_box(system, "ChatGPT update revolute body b2", (1.0, 0.1, 0.1), 5000, chrono.ChVector3d(3.5, 0, 0), chrono.QUNIT, color(0.12, 0.42, 0.90))

    prismatic = chrono.ChLinkLockPrismatic()
    prismatic.SetName("ChatGPT update ground-b1 prismatic joint")
    prismatic.Initialize(b1, ground, chrono.ChFramed(chrono.ChVector3d(2, 0, 0), chrono.Q_ROTATE_Z_TO_X))
    system.AddLink(prismatic)
    guide = chrono.ChBodyEasyBox(1.35, 0.025, 0.025, 1000, True, False)
    guide.SetName("ChatGPT update visible prismatic rail")
    guide.SetFixed(True)
    guide.EnableCollision(False)
    guide.SetPos(chrono.ChVector3d(2.5, -0.18, 0))
    guide.GetVisualShape(0).SetColor(color(0.06, 0.06, 0.07))
    system.AddBody(guide)

    revolute = chrono.ChLinkLockRevolute()
    revolute.SetName("ChatGPT update b1-b2 revolute joint")
    revolute.Initialize(b2, b1, joint_frame_at(chrono.ChVector3d(3, 0, 0), chrono.ChVector3d(0, 0, 1)))
    system.AddLink(revolute)
    make_sphere(system, "ChatGPT update revolute joint marker", 0.055, 1.0, chrono.ChVector3d(3, 0, 0), color(0.04, 0.04, 0.05), True)

    m2 = make_sphere(system, "ChatGPT update spring mass m2", 0.22, 10.0, chrono.ChVector3d(7, 2, 0), color(0.08, 0.30, 0.92))
    anchor = make_sphere(system, "ChatGPT update spring ground anchor", 0.07, 1.0, chrono.ChVector3d(6, 0, 0), color(0.04, 0.04, 0.05), True)
    spring = add_tsda(
        system,
        "ChatGPT update CreateSpringDamper coil",
        m2,
        ground,
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(6, 0, 0),
        1.0e3,
        1.0e1,
        0.12,
        13,
        color(0.88, 0.14, 0.08),
    )

    k = diagonal_matrix([20.0, 0.0, 1.0e4, 0.0, 0.0, 0.0])
    r = diagonal_matrix([0.1, 0.0, 10.0, 0.0, 0.0, 0.0])
    cartesian = chrono.ChLinkBushing()
    cartesian.SetName("ChatGPT update Cartesian spring-damper")
    cartesian.Initialize(m2, ground, chrono.ChFramed(chrono.ChVector3d(7, 2, 0)), k, r)
    system.AddLink(cartesian)

    cart_x_anchor = make_sphere(system, "ChatGPT update Cartesian x spring anchor", 0.045, 1.0, chrono.ChVector3d(7.50, 2.33, 0.0), color(0.04, 0.04, 0.05), True)
    cart_z_anchor = make_sphere(system, "ChatGPT update Cartesian z spring anchor", 0.045, 1.0, chrono.ChVector3d(7.00, 1.66, 0.50), color(0.04, 0.04, 0.05), True)
    cart_x_visual = add_tsda(system, "ChatGPT update Cartesian x coil visual", m2, cart_x_anchor, chrono.ChVector3d(0.0, 0.33, 0.0), chrono.ChVector3d(0, 0, 0), 0.0, 0.0, 0.060, 9, color(0.88, 0.14, 0.08))
    cart_z_visual = add_tsda(system, "ChatGPT update Cartesian z coil visual", m2, cart_z_anchor, chrono.ChVector3d(0.0, -0.34, 0.0), chrono.ChVector3d(0, 0, 0), 0.0, 0.0, 0.060, 9, color(0.88, 0.14, 0.08))

    load_container = chrono.ChLoadContainer()
    system.Add(load_container)
    force_b0 = chrono.ChLoadBodyForce(b0, chrono.ChVector3d(0, 0, 0), False, chrono.ChVector3d(-0.5, 0, 0), True)
    torque_b0 = chrono.ChLoadBodyTorque(b0, chrono.ChVector3d(0, 1, 0), True)
    load_container.Add(force_b0)
    load_container.Add(torque_b0)

    force_body, force_shape = make_mutable_segment(system, "ChatGPT update b0 force vector", color(0.90, 0.12, 0.08), 5)
    torque_body, torque_shape = make_mutable_segment(system, "ChatGPT update b0 torque vector", color(0.96, 0.76, 0.08), 4)

    system._chatgpt_update_items = {
        "ground": ground,
        "b0": b0,
        "m1": m1,
        "b1": b1,
        "b2": b2,
        "m2": m2,
        "spring": spring,
        "cart_x_visual": cart_x_visual,
        "cart_z_visual": cart_z_visual,
        "distances": [dist_ground_b0, dist_b0_m1],
        "force_b0": force_b0,
        "torque_b0": torque_b0,
        "force_visual": (force_body, force_shape),
        "torque_visual": (torque_body, torque_shape),
    }
    update_visuals(system)
    return system, [b0, m1, b1, b2, m2], [spring, cart_x_visual, cart_z_visual]


def update_visuals(system):
    items = getattr(system, "_chatgpt_update_items", None)
    if items is None:
        return

    time = system.GetChTime()
    b0 = items["b0"]
    force = chrono.ChVector3d(10.0 + 5.0 * math.sin(time * 10.0 * 2.0 * math.pi), 0, 0)
    items["force_b0"].SetForce(force, False)
    items["torque_b0"].SetTorque(chrono.ChVector3d(0, 1, 0), True)

    force_origin = b0.TransformPointLocalToParent(chrono.ChVector3d(-0.5, 0, 0))
    force_tip = add(force_origin, scale(force, 0.030))
    force_body, force_shape = items["force_visual"]
    set_segment(force_body, force_shape, force_origin, force_tip)

    torque_origin = b0.TransformPointLocalToParent(chrono.ChVector3d(0.5, 0, 0))
    torque_axis = b0.TransformDirectionLocalToParent(chrono.ChVector3d(0, 1, 0))
    torque_body, torque_shape = items["torque_visual"]
    set_segment(torque_body, torque_shape, torque_origin, add(torque_origin, scale(torque_axis, 0.30)))

    update_system_visuals(system)


def simulate(duration, step):
    system, bodies, springs = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, bodies, springs


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, bodies, springs = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: chatGPTupdate.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(4.6, -6.7, 4.3), chrono.ChVector3d(3.8, 0.3, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, bodies, springs)
            next_log += 0.5


def print_state(system, bodies, springs):
    b0, m1, b1, b2, m2 = bodies
    print(
        f"t={system.GetChTime():6.3f}  "
        f"b0=({b0.GetPos().x:+.4f}, {b0.GetPos().y:+.4f}, {b0.GetPos().z:+.4f})  "
        f"m1=({m1.GetPos().x:+.4f}, {m1.GetPos().y:+.4f}, {m1.GetPos().z:+.4f})  "
        f"b1_x={b1.GetPos().x:+.4f}  "
        f"m2=({m2.GetPos().x:+.4f}, {m2.GetPos().y:+.4f}, {m2.GetPos().z:+.4f})  "
        f"spring_L={springs[0].GetLength():.5f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: chatGPTupdate.py -> PyChrono mainSystemExtensions scene")
    if args.no_vis:
        system, bodies, springs = simulate(args.duration, args.step)
        print_state(system, bodies, springs)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

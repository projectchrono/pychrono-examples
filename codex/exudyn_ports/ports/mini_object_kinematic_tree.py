import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN TestModels/MiniExamples/ObjectKinematicTree.py:
# a one-link ObjectKinematicTree with baseOffset=[0.5, 0, 0], one RevoluteZ
# joint, link COM at local [0.5, 0, 0], and gravity in -y.  The source checks a
# final generalized coordinate near -3.1340185518 after the default 1 s RK67
# solve.  PyChrono represents the same link as a visible rigid body connected
# to ground by a revolute joint.

LENGTH = 1.0
WIDTH = 0.1
DENSITY = 1000.0
GRAVITY = 9.81
BASE_OFFSET_X = 0.5
INITIAL_Q = 0.0
INITIAL_QD = 0.0
EXPECTED_EXUDYN_Q = -3.134018551808591
STEP = 5.0e-4
END_TIME = 1.0


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def link_mass():
    return DENSITY * LENGTH * WIDTH * WIDTH


def inertia_about_pivot_z():
    mass = link_mass()
    inertia_com_z = mass * (LENGTH * LENGTH + WIDTH * WIDTH) / 12.0
    return inertia_com_z + mass * (0.5 * LENGTH) ** 2


def pendulum_rhs(q, qd):
    coeff = link_mass() * GRAVITY * 0.5 * LENGTH / inertia_about_pivot_z()
    return qd, -coeff * math.cos(q)


def scalar_step(q, qd, h):
    k1q, k1v = pendulum_rhs(q, qd)
    k2q, k2v = pendulum_rhs(q + 0.5 * h * k1q, qd + 0.5 * h * k1v)
    k3q, k3v = pendulum_rhs(q + 0.5 * h * k2q, qd + 0.5 * h * k2v)
    k4q, k4v = pendulum_rhs(q + h * k3q, qd + h * k3v)
    q_new = q + h * (k1q + 2.0 * k2q + 2.0 * k3q + k4q) / 6.0
    qd_new = qd + h * (k1v + 2.0 * k2v + 2.0 * k3v + k4v) / 6.0
    return q_new, qd_new


def scalar_integrate(duration, step):
    q = INITIAL_Q
    qd = INITIAL_QD
    time = 0.0
    while time < duration:
        q, qd = scalar_step(q, qd, step)
        time += step
    return q, qd


def make_ground(system, pivot):
    ground = chrono.ChBody()
    ground.SetName("mini ObjectKinematicTree ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    rail = chrono.ChVisualShapeBox(1.65, 0.055, 0.055)
    rail.SetColor(color(0.44, 0.44, 0.46))
    ground.AddVisualShape(rail, chrono.ChFramed(chrono.ChVector3d(BASE_OFFSET_X + 0.25, -0.13, 0)))

    base_marker = chrono.ChVisualShapeSphere(0.036)
    base_marker.SetColor(color(0.50, 0.50, 0.52))
    ground.AddVisualShape(base_marker, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))

    pivot_sphere = chrono.ChVisualShapeSphere(0.060)
    pivot_sphere.SetColor(color(0.04, 0.04, 0.045))
    ground.AddVisualShape(pivot_sphere, chrono.ChFramed(pivot))

    axis = chrono.ChVisualShapeCylinder(0.018, 0.22)
    axis.SetColor(color(0.72, 0.72, 0.74))
    ground.AddVisualShape(axis, chrono.ChFramed(pivot, chrono.QUNIT))

    system.AddBody(ground)
    return ground


def add_link_markers(body):
    for local, tint in (
        (chrono.ChVector3d(-0.5 * LENGTH, 0, 0), color(0.04, 0.04, 0.045)),
        (chrono.ChVector3d(0, 0, 0), color(0.10, 0.10, 0.10)),
        (chrono.ChVector3d(0.5 * LENGTH, 0, 0), color(0.95, 0.56, 0.08)),
    ):
        marker = chrono.ChVisualShapeSphere(0.040)
        marker.SetColor(tint)
        body.AddVisualShape(marker, chrono.ChFramed(local))

    axis = chrono.ChVisualShapeCylinder(0.014, 0.18)
    axis.SetColor(color(0.70, 0.70, 0.72))
    body.AddVisualShape(axis, chrono.ChFramed(chrono.ChVector3d(-0.5 * LENGTH, 0, 0), chrono.QUNIT))


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -GRAVITY, 0))

    pivot = chrono.ChVector3d(BASE_OFFSET_X, 0, 0)
    ground = make_ground(system, pivot)

    com = pivot + chrono.ChVector3d(0.5 * LENGTH * math.cos(INITIAL_Q), 0.5 * LENGTH * math.sin(INITIAL_Q), 0)
    link = chrono.ChBodyEasyBox(LENGTH, WIDTH, WIDTH, DENSITY, True, False)
    link.SetName("mini ObjectKinematicTree single link")
    link.EnableCollision(False)
    link.SetPos(com)
    link.SetRot(chrono.QuatFromAngleZ(INITIAL_Q))
    link.SetAngVelParent(chrono.ChVector3d(0, 0, INITIAL_QD))
    link.GetVisualShape(0).SetColor(color(0.12, 0.42, 0.85))
    add_link_markers(link)
    system.AddBody(link)

    joint = chrono.ChLinkLockRevolute()
    joint.SetName("mini ObjectKinematicTree RevoluteZ analogue")
    joint.Initialize(link, ground, chrono.ChFramed(pivot, chrono.QUNIT))
    system.AddLink(joint)

    expected_marker = chrono.ChBodyEasySphere(0.025, 1000.0, True, False)
    expected_marker.SetName("EXUDYN final-angle tip reference")
    expected_marker.SetFixed(True)
    expected_marker.EnableCollision(False)
    expected_marker.SetPos(
        pivot
        + chrono.ChVector3d(
            LENGTH * math.cos(EXPECTED_EXUDYN_Q),
            LENGTH * math.sin(EXPECTED_EXUDYN_Q),
            0,
        )
    )
    expected_marker.GetVisualShape(0).SetColor(color(0.92, 0.22, 0.12))
    system.AddBody(expected_marker)

    items = {"ground": ground, "link": link, "joint": joint, "pivot": pivot, "expected_marker": expected_marker}
    system._mini_object_kinematic_tree_items = items
    return system, items


def link_q(items):
    return items["link"].GetRot().GetCardanAnglesXYZ().z


def pivot_error(items):
    current = items["link"].TransformPointLocalToParent(chrono.ChVector3d(-0.5 * LENGTH, 0, 0))
    return (current - items["pivot"]).Length()


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
    vis.SetWindowTitle("EXUDYN port: ObjectKinematicTree.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.55, -1.65, 1.15), chrono.ChVector3d(0.55, -0.25, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, items, scalar_integrate(time, step))
            next_log += 0.2


def print_state(system, items, scalar_reference):
    q = link_q(items)
    qd = items["link"].GetAngVelParent().z
    q_rk4, qd_rk4 = scalar_reference
    tip = items["link"].TransformPointLocalToParent(chrono.ChVector3d(0.5 * LENGTH, 0, 0))
    print(
        f"t={system.GetChTime():6.3f}  "
        f"q_chrono={q:+.9f}  q_rk4={q_rk4:+.9f}  "
        f"exudyn_delta={q - EXPECTED_EXUDYN_Q:+.3e}  rk4_delta={q - q_rk4:+.3e}  "
        f"qd={qd:+.6f}  qd_rk4={qd_rk4:+.6f}  "
        f"tip=({tip.x:+.6f},{tip.y:+.6f},{tip.z:+.6f})  "
        f"pivot_error={pivot_error(items):.3e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ObjectKinematicTree.py -> PyChrono mini one-link kinematic tree")
    scalar_reference = scalar_integrate(args.duration, args.step)
    if args.no_vis:
        system, items = simulate(args.duration, args.step)
        print_state(system, items, scalar_reference)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

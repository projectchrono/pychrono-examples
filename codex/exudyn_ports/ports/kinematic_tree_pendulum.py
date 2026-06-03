import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN Examples/kinematicTreePendulum.py:
# a Robot/ObjectKinematicTree one-link pendulum with a RevoluteZ joint,
# initial reference configuration -90 degrees, gravity in -y, and explicit
# solver comparisons.  PyChrono represents the kinematic-tree link as an
# explicit visible rigid body with a revolute support; a compact scalar
# pendulum integration mirrors the EXUDYN explicit-solver comparison.

LENGTH = 0.5
WIDTH = 0.1
DENSITY = 1000.0
GRAVITY = 9.81
INITIAL_Q = -0.5 * math.pi
INITIAL_QD = 0.0
STEP = 1.0e-3
END_TIME = 7.0


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def link_mass():
    return DENSITY * LENGTH * WIDTH * WIDTH


def inertia_about_pivot_z():
    mass = link_mass()
    inertia_com_z = mass * (LENGTH * LENGTH + WIDTH * WIDTH) / 12.0
    return inertia_com_z + mass * (0.5 * LENGTH) ** 2


def q_to_body_angle(q):
    # EXUDYN's q rotates the link local y-axis; Chrono's box is drawn along x.
    return q + 0.5 * math.pi


def body_angle_to_q(angle):
    return angle - 0.5 * math.pi


def pendulum_rhs(q, qd):
    coeff = link_mass() * GRAVITY * 0.5 * LENGTH / inertia_about_pivot_z()
    return qd, coeff * math.sin(q)


def scalar_step(q, qd, h, method):
    if method == "euler":
        q1, qd1 = pendulum_rhs(q, qd)
        return q + h * q1, qd + h * qd1

    if method == "midpoint":
        k1q, k1v = pendulum_rhs(q, qd)
        k2q, k2v = pendulum_rhs(q + 0.5 * h * k1q, qd + 0.5 * h * k1v)
        return q + h * k2q, qd + h * k2v

    if method == "rk4":
        k1q, k1v = pendulum_rhs(q, qd)
        k2q, k2v = pendulum_rhs(q + 0.5 * h * k1q, qd + 0.5 * h * k1v)
        k3q, k3v = pendulum_rhs(q + 0.5 * h * k2q, qd + 0.5 * h * k2v)
        k4q, k4v = pendulum_rhs(q + h * k3q, qd + h * k3v)
        q_new = q + h * (k1q + 2.0 * k2q + 2.0 * k3q + k4q) / 6.0
        qd_new = qd + h * (k1v + 2.0 * k2v + 2.0 * k3v + k4v) / 6.0
        return q_new, qd_new

    raise ValueError(method)


def scalar_integrate(duration, step):
    results = {}
    for method in ("euler", "midpoint", "rk4"):
        q = INITIAL_Q
        qd = INITIAL_QD
        time = 0.0
        while time < duration:
            q, qd = scalar_step(q, qd, step, method)
            time += step
        results[method] = (q, qd)
    return results


def make_ground(system):
    ground = chrono.ChBody()
    ground.SetName("kinematic-tree pendulum ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    rail = chrono.ChVisualShapeBox(4.0 * LENGTH, 0.8 * WIDTH, 0.8 * WIDTH)
    rail.SetColor(color(0.44, 0.44, 0.46))
    ground.AddVisualShape(rail, chrono.ChFramed(chrono.ChVector3d(0.5 * LENGTH, -0.12, 0)))

    pivot = chrono.ChVisualShapeSphere(0.060)
    pivot.SetColor(color(0.04, 0.04, 0.045))
    ground.AddVisualShape(pivot, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))

    axis = chrono.ChVisualShapeCylinder(0.018, 0.22)
    axis.SetColor(color(0.72, 0.72, 0.74))
    ground.AddVisualShape(axis, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))

    system.AddBody(ground)
    return ground


def add_link_markers(body):
    for local, tint in (
        (chrono.ChVector3d(-0.5 * LENGTH, 0, 0), color(0.04, 0.04, 0.045)),
        (chrono.ChVector3d(0.5 * LENGTH, 0, 0), color(0.95, 0.56, 0.08)),
        (chrono.ChVector3d(0, 0, 0), color(0.10, 0.10, 0.10)),
    ):
        marker = chrono.ChVisualShapeSphere(0.038)
        marker.SetColor(tint)
        body.AddVisualShape(marker, chrono.ChFramed(local))

    axis = chrono.ChVisualShapeCylinder(0.014, 0.16)
    axis.SetColor(color(0.70, 0.70, 0.72))
    body.AddVisualShape(axis, chrono.ChFramed(chrono.ChVector3d(-0.5 * LENGTH, 0, 0), chrono.QUNIT))


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -GRAVITY, 0))

    ground = make_ground(system)
    body_angle = q_to_body_angle(INITIAL_Q)
    pivot = chrono.ChVector3d(0, 0, 0)
    com = pivot + chrono.ChVector3d(0.5 * LENGTH * math.cos(body_angle), 0.5 * LENGTH * math.sin(body_angle), 0)

    link = chrono.ChBodyEasyBox(LENGTH, WIDTH, WIDTH, DENSITY, True, False)
    link.SetName("kinematic-tree one-link pendulum")
    link.EnableCollision(False)
    link.SetPos(com)
    link.SetRot(chrono.QuatFromAngleZ(body_angle))
    link.SetAngVelParent(chrono.ChVector3d(0, 0, INITIAL_QD))
    link.GetVisualShape(0).SetColor(color(0.12, 0.42, 0.85))
    add_link_markers(link)
    system.AddBody(link)

    joint = chrono.ChLinkLockRevolute()
    joint.SetName("kinematic-tree RevoluteZ analogue")
    joint.Initialize(link, ground, chrono.ChFramed(pivot, chrono.QUNIT))
    system.AddLink(joint)

    tip_reference = chrono.ChBodyEasySphere(0.025, 1000.0, True, False)
    tip_reference.SetName("initial tip reference")
    tip_reference.SetFixed(True)
    tip_reference.EnableCollision(False)
    tip_reference.SetPos(chrono.ChVector3d(LENGTH, 0, 0))
    tip_reference.GetVisualShape(0).SetColor(color(0.92, 0.22, 0.12))
    system.AddBody(tip_reference)

    items = {"ground": ground, "link": link, "joint": joint, "pivot": pivot, "tip_reference": tip_reference}
    system._kinematic_tree_pendulum_items = items
    return system, items


def pivot_error(items):
    current = items["link"].TransformPointLocalToParent(chrono.ChVector3d(-0.5 * LENGTH, 0, 0))
    return (current - items["pivot"]).Length()


def link_q(items):
    angle_z = items["link"].GetRot().GetCardanAnglesXYZ().z
    return body_angle_to_q(angle_z)


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
    vis.SetWindowTitle("EXUDYN port: kinematicTreePendulum.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.28, -1.25, 1.15), chrono.ChVector3d(0.12, 0.0, 0.0))
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
            next_log += 0.5


def print_state(system, items, scalar_results):
    q = link_q(items)
    qd = items["link"].GetAngVelParent().z
    midpoint = items["link"].GetPos()
    rk4_q, rk4_qd = scalar_results["rk4"]
    print(
        f"t={system.GetChTime():6.3f}  "
        f"q_chrono={q:+.6f}  qd={qd:+.6f}  "
        f"q_rk4={rk4_q:+.6f}  rk4_delta={q - rk4_q:+.3e}  "
        f"mid=({midpoint.x:+.6f},{midpoint.y:+.6f},{midpoint.z:+.6f})  "
        f"pivot_error={pivot_error(items):.3e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: kinematicTreePendulum.py -> PyChrono one-link kinematic-tree pendulum")
    if args.no_vis:
        system, items = simulate(args.duration, args.step)
        print_state(system, items, scalar_integrate(args.duration, args.step))
        for method, (q, qd) in scalar_integrate(args.duration, args.step).items():
            print(f"{method:>8s}: q={q:+.6f} qd={qd:+.6f}")
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

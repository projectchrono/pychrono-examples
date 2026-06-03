import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/ballBearingTest.py:
# a single 6010/C3-style ball bearing with a rotating inner shaft, fixed outer
# ring, cage, balls, and the source force/torque timing reported for checks.
# EXUDYN's ContactSphereTorus helper is represented here with explicit bearing
# geometry and kinematic ball/cage motion for robust PyChrono visualization.

OUTSIDE_DIAMETER = 0.080
BORE_DIAMETER = 0.050
WIDTH = 0.016
N_BALLS = 14
RADIUS_CAGE = 0.0325
RADIUS_BALL = 0.00873 / 2.0
INNER_RING_RADIUS = 0.5 * BORE_DIAMETER
OUTER_RING_RADIUS = 0.5 * OUTSIDE_DIAMETER
SHAFT_RADIUS = 0.5 * BORE_DIAMETER
SHAFT_LENGTH = 0.120
OMEGA_SHAFT = 2.0 * math.pi * 2.0
CAGE_SPEED_RATIO = 0.42
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def force_z(time):
    time_start = 0.0
    max_force = 200.0
    if time < time_start:
        return 0.0
    if time < 2.0 * time_start:
        return min(max_force * (time - time_start), max_force)
    if time < 3.0 * time_start:
        return max(-max_force, max_force - 2.0 * max_force * (time - 2.0 * time_start))
    return min(0.0, -max_force + max_force * (time - 3.0 * time_start))


def torque_x(time):
    time_start = 3.5 * 0.0
    max_torque = 8.0
    if time < time_start or time > time_start + 2.0:
        return 0.0
    if time < time_start + 1.0:
        return min(max_torque * (time - time_start), max_torque)
    return max(max_torque * (time_start + 2.0 - time), 0.0)


def make_marker_sphere(radius, tint):
    shape = chrono.ChVisualShapeSphere(radius)
    shape.SetColor(tint)
    return shape


def add_ring_beads(body, ring_radius, bead_radius, z_offset, tint, count=64):
    for i in range(count):
        angle = 2.0 * math.pi * i / count
        point = chrono.ChVector3d(ring_radius * math.cos(angle), ring_radius * math.sin(angle), z_offset)
        body.AddVisualShape(make_marker_sphere(bead_radius, tint), chrono.ChFramed(point))


def add_ring_visuals(body, inner):
    if inner:
        tint = color(0.12, 0.42, 0.85)
        add_ring_beads(body, INNER_RING_RADIUS, 0.0015, -0.5 * WIDTH, tint, 44)
        add_ring_beads(body, INNER_RING_RADIUS, 0.0015, 0.5 * WIDTH, tint, 44)
        add_ring_beads(body, INNER_RING_RADIUS + 0.004, 0.0012, 0, tint, 44)
    else:
        tint = color(0.40, 0.40, 0.42)
        add_ring_beads(body, OUTER_RING_RADIUS, 0.0018, -0.5 * WIDTH, tint, 60)
        add_ring_beads(body, OUTER_RING_RADIUS, 0.0018, 0.5 * WIDTH, tint, 60)
        add_ring_beads(body, OUTER_RING_RADIUS - 0.004, 0.0014, 0, tint, 60)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    background = chrono.ChBodyEasyBox(0.13, 0.006, 0.16, 1000, True, False)
    background.SetName("ball bearing checkerboard plate")
    background.SetFixed(True)
    background.SetPos(chrono.ChVector3d(0, -0.055, -0.012))
    background.GetVisualShape(0).SetColor(color(0.82, 0.82, 0.78))
    background.GetVisualShape(0).SetOpacity(0.35)
    system.AddBody(background)

    outer_ring = chrono.ChBody()
    outer_ring.SetName("fixed outer ball bearing ring")
    outer_ring.SetFixed(True)
    outer_ring.EnableCollision(False)
    add_ring_visuals(outer_ring, inner=False)
    for x in (-0.6 * OUTSIDE_DIAMETER, 0.6 * OUTSIDE_DIAMETER):
        lug = chrono.ChVisualShapeSphere(0.006)
        lug.SetColor(color(0.58, 0.34, 0.16))
        outer_ring.AddVisualShape(lug, chrono.ChFramed(chrono.ChVector3d(x, 0, 0)))
    system.AddBody(outer_ring)

    shaft = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, SHAFT_RADIUS, SHAFT_LENGTH, 7800, True, False)
    shaft.SetName("rotating inner bearing shaft")
    shaft.SetFixed(True)
    shaft.GetVisualShape(0).SetColor(color(0.12, 0.42, 0.85))
    add_ring_visuals(shaft, inner=True)
    system.AddBody(shaft)

    cage = chrono.ChBody()
    cage.SetName("ball bearing cage")
    cage.SetFixed(True)
    cage.EnableCollision(False)
    add_ring_beads(cage, RADIUS_CAGE, 0.0012, 0, color(0.95, 0.64, 0.16), N_BALLS)
    system.AddBody(cage)

    balls = []
    for i in range(N_BALLS):
        ball = chrono.ChBodyEasySphere(RADIUS_BALL * 1.01, 7800, True, False)
        ball.SetName(f"bearing test ball {i + 1:02d}")
        ball.SetFixed(True)
        ball.GetVisualShape(0).SetColor(color(0.88, 0.88, 0.86))
        patch = chrono.ChVisualShapeSphere(0.0013)
        patch.SetColor(color(0.12, 0.12, 0.12))
        ball.AddVisualShape(patch, chrono.ChFramed(chrono.ChVector3d(RADIUS_BALL, 0, 0)))
        system.AddBody(ball)
        balls.append(ball)

    force_marker = chrono.ChBodyEasyBox(0.010, 0.010, 0.050, 1000, True, False)
    force_marker.SetName("bearing axial force marker")
    force_marker.SetFixed(True)
    force_marker.SetPos(chrono.ChVector3d(0.058, 0.0, 0.035))
    force_marker.GetVisualShape(0).SetColor(color(0.90, 0.18, 0.12))
    system.AddBody(force_marker)

    system._bearing_test_items = {"shaft": shaft, "cage": cage, "balls": balls}
    update_bearing_kinematics(system)
    return system, shaft, outer_ring, cage, balls


def update_bearing_kinematics(system):
    items = getattr(system, "_bearing_test_items", None)
    if items is None:
        return

    time = system.GetChTime()
    shaft_angle = OMEGA_SHAFT * time
    cage_angle = CAGE_SPEED_RATIO * shaft_angle
    ball_spin = -OMEGA_SHAFT * SHAFT_RADIUS / RADIUS_BALL

    items["shaft"].SetRot(chrono.QuatFromAngleZ(shaft_angle))
    items["shaft"].SetAngVelParent(chrono.ChVector3d(0, 0, OMEGA_SHAFT))
    items["cage"].SetRot(chrono.QuatFromAngleZ(cage_angle))

    for i, ball in enumerate(items["balls"]):
        angle = cage_angle + 2.0 * math.pi * i / N_BALLS
        position = chrono.ChVector3d(RADIUS_CAGE * math.cos(angle), RADIUS_CAGE * math.sin(angle), 0)
        tangent = chrono.ChVector3d(-math.sin(angle), math.cos(angle), 0)
        ball.SetPos(position)
        ball.SetRot(chrono.QuatFromAngleAxis(ball_spin * time, tangent) * chrono.QuatFromAngleZ(angle))
        ball.SetPosDt(
            chrono.ChVector3d(
                -CAGE_SPEED_RATIO * OMEGA_SHAFT * RADIUS_CAGE * math.sin(angle),
                CAGE_SPEED_RATIO * OMEGA_SHAFT * RADIUS_CAGE * math.cos(angle),
                0,
            )
        )
        ball.SetAngVelParent(chrono.ChVector3d(tangent.x * ball_spin, tangent.y * ball_spin, 0))


def update_visuals(system):
    update_bearing_kinematics(system)


def simulate(duration, step):
    system, shaft, outer_ring, cage, balls = build_system()
    while system.GetChTime() < duration:
        update_bearing_kinematics(system)
        system.DoStepDynamics(step)
    update_bearing_kinematics(system)
    return system, shaft, outer_ring, cage, balls


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, shaft, outer_ring, cage, balls = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ballBearingTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.13, -0.18, 0.13), chrono.ChVector3d(0, 0, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        update_bearing_kinematics(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, shaft, balls)
            next_log += 0.25


def print_state(system, shaft, balls):
    sample = balls[0].GetPos()
    print(
        f"t={system.GetChTime():6.3f}  "
        f"shaft_wz={OMEGA_SHAFT:+.4f}  "
        f"sample_ball=({sample.x:+.5f}, {sample.y:+.5f}, {sample.z:+.5f})  "
        f"force_z={force_z(system.GetChTime()):+.2f}  "
        f"torque_x={torque_x(system.GetChTime()):+.2f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0.5)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ballBearingTest.py -> PyChrono visual ball bearing test")
    if args.no_vis:
        system, shaft, outer_ring, cage, balls = simulate(args.duration, args.step)
        print_state(system, shaft, balls)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN Examples/ballBearningModel.py:
# two ball bearings support a rotating shaft. EXUDYN builds a detailed contact
# bearing with helper functions from exudyn.machines; PyChrono core has no
# matching bearing helper here, so this port keeps the bearing kinematics
# explicit and visual: fixed outer rings, a spinning shaft/inner rings, cages,
# and all balls as separate visible Chrono bodies.

OUTSIDE_DIAMETER = 0.080
BORE_DIAMETER = 0.050
WIDTH = 0.016
N_BALLS = 14
RADIUS_CAGE = 0.0325
RADIUS_BALL = 0.00873 / 2.0
OUTER_RING_RADIUS = 0.5 * OUTSIDE_DIAMETER
INNER_RING_RADIUS = 0.5 * BORE_DIAMETER
SHAFT_RADIUS = 0.5 * BORE_DIAMETER
SHAFT_LENGTH = 0.120
OMEGA_SHAFT = 2.0 * math.pi * 2.0
CAGE_SPEED_RATIO = 0.42
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def force_z(time):
    ramp_time = 2.0
    max_force = 200.0
    if time < ramp_time:
        return 0.0
    if time < 2.0 * ramp_time:
        return min(max_force * (time - ramp_time), max_force)
    if time < 3.0 * ramp_time:
        return max(-max_force, max_force - 2.0 * max_force * (time - 2.0 * ramp_time))
    return min(0.0, -max_force + max_force * (time - 3.0 * ramp_time))


def torque_x(time):
    ramp_time = 7.0
    max_torque = 8.0
    if time < ramp_time or time > ramp_time + 2.0:
        return 0.0
    if time < ramp_time + 1.0:
        return min(max_torque * (time - ramp_time), max_torque)
    return max(max_torque * (ramp_time + 2.0 - time), 0.0)


def make_marker_sphere(radius, tint):
    shape = chrono.ChVisualShapeSphere(radius)
    shape.SetColor(tint)
    return shape


def add_ring_beads(body, ring_radius, bead_radius, z_offset, tint, count=64):
    for i in range(count):
        angle = 2.0 * math.pi * i / count
        point = chrono.ChVector3d(ring_radius * math.cos(angle), ring_radius * math.sin(angle), z_offset)
        body.AddVisualShape(make_marker_sphere(bead_radius, tint), chrono.ChFramed(point))


def add_bearing_ring_visuals(body, z_offset, inner=False):
    if inner:
        tint = color(0.12, 0.42, 0.85)
        add_ring_beads(body, INNER_RING_RADIUS, 0.0015, z_offset - 0.5 * WIDTH, tint, 40)
        add_ring_beads(body, INNER_RING_RADIUS, 0.0015, z_offset + 0.5 * WIDTH, tint, 40)
        add_ring_beads(body, INNER_RING_RADIUS + 0.004, 0.0012, z_offset, tint, 40)
    else:
        tint = color(0.40, 0.40, 0.42)
        add_ring_beads(body, OUTER_RING_RADIUS, 0.0018, -0.5 * WIDTH, tint, 56)
        add_ring_beads(body, OUTER_RING_RADIUS, 0.0018, 0.5 * WIDTH, tint, 56)
        add_ring_beads(body, OUTER_RING_RADIUS - 0.004, 0.0014, 0, tint, 56)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    ground = chrono.ChBody()
    ground.SetName("bearing ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)

    plate = chrono.ChBodyEasyBox(0.15, 0.006, 0.18, 1000, True, False)
    plate.SetName("bearing background plate")
    plate.SetFixed(True)
    plate.SetPos(chrono.ChVector3d(0, -0.055, 0))
    plate.GetVisualShape(0).SetColor(color(0.82, 0.82, 0.78))
    plate.GetVisualShape(0).SetOpacity(0.35)
    system.AddBody(plate)

    shaft = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, SHAFT_RADIUS, SHAFT_LENGTH, 7800, True, False)
    shaft.SetName("rotating bearing shaft and inner rings")
    shaft.SetFixed(True)
    shaft.GetVisualShape(0).SetColor(color(0.12, 0.42, 0.85))
    system.AddBody(shaft)

    bearing_offsets = [-0.3 * SHAFT_LENGTH, 0.3 * SHAFT_LENGTH]
    outer_rings = []
    cages = []
    balls = []

    for bearing_index, z_offset in enumerate(bearing_offsets):
        add_bearing_ring_visuals(shaft, z_offset, inner=True)

        outer = chrono.ChBody()
        outer.SetName(f"fixed outer bearing ring {bearing_index + 1}")
        outer.SetFixed(True)
        outer.EnableCollision(False)
        outer.SetPos(chrono.ChVector3d(0, 0, z_offset))
        add_bearing_ring_visuals(outer, 0, inner=False)

        lug_a = chrono.ChVisualShapeSphere(0.006)
        lug_a.SetColor(color(0.58, 0.34, 0.16))
        outer.AddVisualShape(lug_a, chrono.ChFramed(chrono.ChVector3d(1.18 * OUTER_RING_RADIUS, 0, 0)))
        lug_b = chrono.ChVisualShapeSphere(0.006)
        lug_b.SetColor(color(0.58, 0.34, 0.16))
        outer.AddVisualShape(lug_b, chrono.ChFramed(chrono.ChVector3d(-1.18 * OUTER_RING_RADIUS, 0, 0)))
        system.AddBody(outer)
        outer_rings.append(outer)

        cage = chrono.ChBody()
        cage.SetName(f"bearing cage {bearing_index + 1}")
        cage.SetFixed(True)
        cage.EnableCollision(False)
        cage.SetPos(chrono.ChVector3d(0, 0, z_offset))
        add_ring_beads(cage, RADIUS_CAGE, 0.0012, 0, color(0.95, 0.64, 0.16), N_BALLS)
        system.AddBody(cage)
        cages.append(cage)

        bearing_balls = []
        for i in range(N_BALLS):
            ball = chrono.ChBodyEasySphere(RADIUS_BALL, 7800, True, False)
            ball.SetName(f"bearing {bearing_index + 1} ball {i + 1:02d}")
            ball.SetFixed(True)
            ball.GetVisualShape(0).SetColor(color(0.88, 0.88, 0.86))
            patch = chrono.ChVisualShapeSphere(0.0013)
            patch.SetColor(color(0.12, 0.12, 0.12))
            ball.AddVisualShape(patch, chrono.ChFramed(chrono.ChVector3d(RADIUS_BALL, 0, 0)))
            system.AddBody(ball)
            bearing_balls.append(ball)
        balls.append(bearing_balls)

    system._bearing_items = {
        "shaft": shaft,
        "cages": cages,
        "balls": balls,
        "offsets": bearing_offsets,
    }
    update_bearing_kinematics(system)
    return system, shaft, outer_rings, cages, balls


def update_bearing_kinematics(system):
    items = getattr(system, "_bearing_items", None)
    if items is None:
        return

    time = system.GetChTime()
    shaft_angle = OMEGA_SHAFT * time
    cage_angle = CAGE_SPEED_RATIO * shaft_angle
    ball_spin = -OMEGA_SHAFT * SHAFT_RADIUS / RADIUS_BALL

    items["shaft"].SetRot(chrono.QuatFromAngleZ(shaft_angle))
    items["shaft"].SetAngVelParent(chrono.ChVector3d(0, 0, OMEGA_SHAFT))

    for bearing_index, z_offset in enumerate(items["offsets"]):
        items["cages"][bearing_index].SetRot(chrono.QuatFromAngleZ(cage_angle))
        items["cages"][bearing_index].SetAngVelParent(chrono.ChVector3d(0, 0, CAGE_SPEED_RATIO * OMEGA_SHAFT))

        for i, ball in enumerate(items["balls"][bearing_index]):
            angle = cage_angle + 2.0 * math.pi * i / N_BALLS
            position = chrono.ChVector3d(
                RADIUS_CAGE * math.cos(angle),
                RADIUS_CAGE * math.sin(angle),
                z_offset,
            )
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
    system, shaft, outer_rings, cages, balls = build_system()
    while system.GetChTime() < duration:
        update_bearing_kinematics(system)
        system.DoStepDynamics(step)
    update_bearing_kinematics(system)
    return system, shaft, outer_rings, cages, balls


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, shaft, outer_rings, cages, balls = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ballBearningModel.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.13, -0.18, 0.13), chrono.ChVector3d(0, 0, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        update_bearing_kinematics(system)
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, shaft, balls)
            next_log += 0.25


def print_state(system, shaft, balls):
    sample = balls[0][0].GetPos()
    print(
        f"t={system.GetChTime():6.3f}  "
        f"shaft_wz={OMEGA_SHAFT:+.4f}  "
        f"sample_ball=({sample.x:+.5f}, {sample.y:+.5f}, {sample.z:+.5f})  "
        f"load_z={force_z(system.GetChTime()):+.2f}  "
        f"torque_x={torque_x(system.GetChTime()):+.2f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ballBearningModel.py -> PyChrono visual two-bearing shaft")
    if args.no_vis:
        system, shaft, outer_rings, cages, balls = simulate(args.duration, args.step)
        print_state(system, shaft, balls)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

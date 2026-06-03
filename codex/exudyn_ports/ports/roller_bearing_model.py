import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN Examples/rollerBearningModel.py:
# eight rolling elements between a fixed outer race and rotating inner race.
# The EXUDYN example focuses on sphere-sphere contact in an annular bearing;
# this PyChrono port keeps the annular geometry and rolling-element kinematics
# explicit for robust visualization.

WIDTH = 0.12
ROLLER_RADIUS = 0.10
OUTER_RADIUS = 0.60
INNER_RADIUS = 0.40
N_ROLLERS = 8
OMEGA_INNER = 2.0 * math.pi * 2.0
CAGE_RADIUS = OUTER_RADIUS - ROLLER_RADIUS
CAGE_SPEED_RATIO = INNER_RADIUS / (INNER_RADIUS + OUTER_RADIUS)
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def marker_sphere(radius, tint):
    shape = chrono.ChVisualShapeSphere(radius)
    shape.SetColor(tint)
    return shape


def add_ring_beads(body, ring_radius, bead_radius, z_offset, tint, count=96):
    for i in range(count):
        angle = 2.0 * math.pi * i / count
        body.AddVisualShape(
            marker_sphere(bead_radius, tint),
            chrono.ChFramed(chrono.ChVector3d(ring_radius * math.cos(angle), ring_radius * math.sin(angle), z_offset)),
        )


def add_spoke(body, length, tint):
    spoke = chrono.ChVisualShapeBox(length, 0.018, 0.018)
    spoke.SetColor(tint)
    body.AddVisualShape(spoke)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -10.0, 0))

    background = chrono.ChBodyEasyBox(1.55, 1.55, 0.016, 1000, True, False)
    background.SetName("roller bearing background")
    background.SetFixed(True)
    background.SetPos(chrono.ChVector3d(0, 0, -0.18))
    background.GetVisualShape(0).SetColor(color(0.82, 0.82, 0.78))
    background.GetVisualShape(0).SetOpacity(0.30)
    system.AddBody(background)

    outer_ring = chrono.ChBody()
    outer_ring.SetName("fixed roller bearing outer race")
    outer_ring.SetFixed(True)
    outer_ring.EnableCollision(False)
    add_ring_beads(outer_ring, OUTER_RADIUS + ROLLER_RADIUS, 0.014, -0.5 * WIDTH, color(0.55, 0.55, 0.55), 120)
    add_ring_beads(outer_ring, OUTER_RADIUS + ROLLER_RADIUS, 0.014, 0.5 * WIDTH, color(0.55, 0.55, 0.55), 120)
    add_ring_beads(outer_ring, OUTER_RADIUS, 0.010, 0, color(0.38, 0.38, 0.40), 120)
    system.AddBody(outer_ring)

    inner = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, INNER_RADIUS, 2.0 * WIDTH, 7800, True, False)
    inner.SetName("rotating roller bearing inner race")
    inner.SetFixed(True)
    inner.GetVisualShape(0).SetColor(color(0.12, 0.35, 0.90))
    add_spoke(inner, 1.7 * INNER_RADIUS, color(0.95, 0.55, 0.10))
    system.AddBody(inner)

    cage = chrono.ChBody()
    cage.SetName("roller bearing cage")
    cage.SetFixed(True)
    cage.EnableCollision(False)
    add_ring_beads(cage, CAGE_RADIUS, 0.010, 0, color(0.95, 0.64, 0.16), N_ROLLERS)
    system.AddBody(cage)

    rollers = []
    for i in range(N_ROLLERS):
        roller = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, ROLLER_RADIUS, WIDTH, 7800, True, False)
        roller.SetName(f"roller bearing element {i + 1:02d}")
        roller.SetFixed(True)
        roller.GetVisualShape(0).SetColor(color(0.92, 0.45, 0.12))
        add_spoke(roller, 1.8 * ROLLER_RADIUS, color(0.08, 0.08, 0.08))
        system.AddBody(roller)
        rollers.append(roller)

    system._roller_bearing_items = {"inner": inner, "cage": cage, "rollers": rollers}
    update_bearing_kinematics(system)
    return system, inner, outer_ring, cage, rollers


def update_bearing_kinematics(system):
    items = getattr(system, "_roller_bearing_items", None)
    if items is None:
        return

    time = system.GetChTime()
    inner_angle = OMEGA_INNER * time
    cage_angle = CAGE_SPEED_RATIO * inner_angle
    roller_spin = -OMEGA_INNER * INNER_RADIUS / ROLLER_RADIUS

    items["inner"].SetRot(chrono.QuatFromAngleZ(inner_angle))
    items["inner"].SetAngVelParent(chrono.ChVector3d(0, 0, OMEGA_INNER))
    items["cage"].SetRot(chrono.QuatFromAngleZ(cage_angle))

    for i, roller in enumerate(items["rollers"]):
        angle = cage_angle + 2.0 * math.pi * i / N_ROLLERS
        roller.SetPos(chrono.ChVector3d(CAGE_RADIUS * math.cos(angle), CAGE_RADIUS * math.sin(angle), 0))
        roller.SetRot(chrono.QuatFromAngleZ(angle + roller_spin * time))
        roller.SetPosDt(
            chrono.ChVector3d(
                -CAGE_SPEED_RATIO * OMEGA_INNER * CAGE_RADIUS * math.sin(angle),
                CAGE_SPEED_RATIO * OMEGA_INNER * CAGE_RADIUS * math.cos(angle),
                0,
            )
        )
        roller.SetAngVelParent(chrono.ChVector3d(0, 0, roller_spin))


def update_visuals(system):
    update_bearing_kinematics(system)


def simulate(duration, step):
    system, inner, outer_ring, cage, rollers = build_system()
    while system.GetChTime() < duration:
        update_bearing_kinematics(system)
        system.DoStepDynamics(step)
    update_bearing_kinematics(system)
    return system, inner, outer_ring, cage, rollers


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, inner, outer_ring, cage, rollers = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: rollerBearningModel.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.0, -2.1, 1.45), chrono.ChVector3d(0, 0, 0))
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
            print_state(system, inner, rollers)
            next_log += 0.25


def print_state(system, inner, rollers):
    sample = rollers[0].GetPos()
    print(
        f"t={system.GetChTime():6.3f}  "
        f"inner_wz={OMEGA_INNER:+.4f}  "
        f"sample_roller=({sample.x:+.4f}, {sample.y:+.4f}, {sample.z:+.4f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: rollerBearningModel.py -> PyChrono visual roller bearing")
    if args.no_vis:
        system, inner, outer_ring, cage, rollers = simulate(args.duration, args.step)
        print_state(system, inner, rollers)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

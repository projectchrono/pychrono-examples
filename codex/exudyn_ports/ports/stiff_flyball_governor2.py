import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN Examples/stiffFlyballGovernor2.py:
# a stiff IFToMM flyball governor with a spinning vertical shaft, a prismatic
# slider, two hinged rods carrying heavy flyballs, and two slider-to-rod
# spring-dampers.

LENGTH_SHAFT = 1.0
WIDTH_SHAFT = 0.01
LENGTH_ROD = 1.0
WIDTH_ROD = 0.01
SHAFT_VISUAL_RADIUS = 0.025
ROD_VISUAL_WIDTH = 0.035
DIM_SLIDER = 0.1
SLIDER_Z0 = 0.5
X_AB = 0.1
BETA0 = math.radians(30.0)
OMEGA_Z0 = 0.16 * 2.0 * math.pi

M_SHAFT = 0.3
M_ROD = 0.3
M_SLIDER = 3.0
M_FLYBALL = 5.0
M_ROD_TOTAL = M_ROD + M_FLYBALL

SPRING_K = 8.0e5 * 0.005
SPRING_C = 4.0e4 * 0.005
SPRING_L0 = 0.5
STEP = 1e-4

ROD_COM_SHIFT = -(M_FLYBALL * (0.5 * LENGTH_ROD)) / M_ROD_TOTAL
ROD_HINGE_DISTANCE = 0.5 * LENGTH_ROD - ROD_COM_SHIFT
ROD_FAR_END = 0.5 * LENGTH_ROD + ROD_COM_SHIFT


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def box_shape(size_x, size_y, size_z, tint):
    shape = chrono.ChVisualShapeBox(size_x, size_y, size_z)
    shape.SetColor(tint)
    return shape


def add_pin_marker(body, local_position, radius=0.035, tint=None):
    marker = chrono.ChVisualShapeSphere(radius)
    marker.SetColor(tint or color(0.06, 0.06, 0.06))
    body.AddVisualShape(marker, chrono.ChFramed(local_position))


def make_ground_support(system):
    ground = chrono.ChBody()
    ground.SetName("flyball ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)

    base = chrono.ChBodyEasyBox(0.34, 0.34, 0.035, 1000, True, False)
    base.SetName("visible governor base")
    base.SetFixed(True)
    base.SetPos(chrono.ChVector3d(0, 0, -0.02))
    base.GetVisualShape(0).SetColor(color(0.42, 0.42, 0.42))
    system.AddBody(base)
    return ground


def make_shaft():
    shaft = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, SHAFT_VISUAL_RADIUS, LENGTH_SHAFT, 3000, True, False)
    shaft.SetName("governor shaft")
    shaft.SetMass(M_SHAFT)
    shaft.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, 0.0001))
    shaft.SetPos(chrono.ChVector3d(0, 0, 0.5 * LENGTH_SHAFT))
    shaft.SetAngVelParent(chrono.ChVector3d(0, 0, OMEGA_Z0))
    shaft.GetVisualShape(0).SetColor(color(0.12, 0.28, 0.86))

    for x in (-0.5 * X_AB, 0.5 * X_AB):
        add_pin_marker(shaft, chrono.ChVector3d(x, 0, 0.5 * LENGTH_SHAFT), 0.028)
    return shaft


def make_slider():
    slider = chrono.ChBodyEasyBox(DIM_SLIDER, DIM_SLIDER, DIM_SLIDER, 3000, True, False)
    slider.SetName("governor slider")
    slider.SetMass(M_SLIDER)
    slider.SetInertiaXX(chrono.ChVector3d(0.005, 0.005, 0.005))
    slider.SetPos(chrono.ChVector3d(0, 0, SLIDER_Z0))
    slider.SetAngVelParent(chrono.ChVector3d(0, 0, OMEGA_Z0))
    slider.GetVisualShape(0).SetColor(color(0.95, 0.66, 0.08))
    add_pin_marker(slider, chrono.ChVector3d(0.5 * DIM_SLIDER, 0, 0), 0.024)
    add_pin_marker(slider, chrono.ChVector3d(-0.5 * DIM_SLIDER, 0, 0), 0.024)
    return slider


def rod_inertia():
    ix = M_ROD * (WIDTH_ROD**2 + WIDTH_ROD**2) / 12.0
    iy_rod_center = M_ROD * (LENGTH_ROD**2 + WIDTH_ROD**2) / 12.0
    iz_rod_center = iy_rod_center
    rod_offset = ROD_COM_SHIFT
    ball_offset = ROD_FAR_END
    iy = iy_rod_center + M_ROD * rod_offset**2 + M_FLYBALL * ball_offset**2
    iz = iz_rod_center + M_ROD * rod_offset**2 + M_FLYBALL * ball_offset**2
    return chrono.ChVector3d(ix + 0.001, iy, iz)


def make_rod(name, side):
    sign = 1.0 if side > 0 else -1.0
    x_pos = sign * (0.5 * X_AB + ROD_HINGE_DISTANCE * math.cos(BETA0))
    z_pos = LENGTH_SHAFT - ROD_HINGE_DISTANCE * math.sin(BETA0)
    beta = sign * BETA0

    rod = chrono.ChBody()
    rod.SetName(name)
    rod.SetMass(M_ROD_TOTAL)
    rod.SetInertiaXX(rod_inertia())
    rod.SetPos(chrono.ChVector3d(x_pos, 0, z_pos))
    rod.SetRot(chrono.QuatFromAngleY(beta))
    rod.SetPosDt(chrono.ChVector3d(0, OMEGA_Z0 * x_pos, 0))
    rod.SetAngVelParent(chrono.ChVector3d(0, 0, OMEGA_Z0))
    rod.EnableCollision(False)

    if side > 0:
        rod_center = ROD_COM_SHIFT
        hinge_local = chrono.ChVector3d(-ROD_HINGE_DISTANCE, 0, 0)
        ball_local = chrono.ChVector3d(ROD_FAR_END, 0, 0)
        spring_local = chrono.ChVector3d(ROD_COM_SHIFT, 0, 0)
        tint = color(0.16, 0.38, 0.88)
    else:
        rod_center = -ROD_COM_SHIFT
        hinge_local = chrono.ChVector3d(ROD_HINGE_DISTANCE, 0, 0)
        ball_local = chrono.ChVector3d(-ROD_FAR_END, 0, 0)
        spring_local = chrono.ChVector3d(-ROD_COM_SHIFT, 0, 0)
        tint = color(0.12, 0.58, 0.24)

    rod.AddVisualShape(
        box_shape(LENGTH_ROD, ROD_VISUAL_WIDTH, ROD_VISUAL_WIDTH, tint),
        chrono.ChFramed(chrono.ChVector3d(rod_center, 0, 0)),
    )
    ball = chrono.ChVisualShapeSphere(0.085)
    ball.SetColor(color(0.82, 0.18, 0.12))
    rod.AddVisualShape(ball, chrono.ChFramed(ball_local))
    add_pin_marker(rod, hinge_local, 0.030)
    add_pin_marker(rod, spring_local, 0.024, color(0.95, 0.95, 0.95))

    rod._flyball_hinge_local = hinge_local
    rod._flyball_spring_local = spring_local
    rod._flyball_ball_local = ball_local
    return rod


def make_revolute_y(body_a, body_b, position):
    joint = chrono.ChLinkLockRevolute()
    joint.Initialize(body_a, body_b, chrono.ChFramed(position, chrono.Q_ROTATE_Z_TO_Y))
    return joint


def add_governor_spring(system, rod, slider, rod_local, slider_local, name):
    spring = chrono.ChLinkTSDA()
    spring.SetName(name)
    spring.Initialize(rod, slider, True, rod_local, slider_local)
    spring.SetRestLength(SPRING_L0)
    spring.SetSpringCoefficient(SPRING_K)
    spring.SetDampingCoefficient(SPRING_C)
    system.AddLink(spring)

    spring_shape = chrono.ChVisualShapeSpring(0.035, 90, 9)
    spring_shape.SetColor(color(0.88, 0.12, 0.10))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, spring, 0.035, 90, 9, color(0.88, 0.12, 0.10))
    return spring


def update_visuals(system):
    update_system_visuals(system)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    ground = make_ground_support(system)
    shaft = make_shaft()
    slider = make_slider()
    rod_ac = make_rod("governor rod AC with flyball", side=1)
    rod_bd = make_rod("governor rod BD with flyball", side=-1)

    for body in (shaft, slider, rod_ac, rod_bd):
        system.AddBody(body)

    shaft_ground = chrono.ChLinkLockRevolute()
    shaft_ground.Initialize(shaft, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
    system.AddLink(shaft_ground)

    slider_joint = chrono.ChLinkLockPrismatic()
    slider_joint.Initialize(slider, shaft, chrono.ChFramed(chrono.ChVector3d(0, 0, SLIDER_Z0), chrono.QUNIT))
    system.AddLink(slider_joint)

    point_a = chrono.ChVector3d(0.5 * X_AB, 0, LENGTH_SHAFT)
    point_b = chrono.ChVector3d(-0.5 * X_AB, 0, LENGTH_SHAFT)
    joint_ac = make_revolute_y(rod_ac, shaft, point_a)
    joint_bd = make_revolute_y(rod_bd, shaft, point_b)
    system.AddLink(joint_ac)
    system.AddLink(joint_bd)

    spring_ac = add_governor_spring(
        system,
        rod_ac,
        slider,
        rod_ac._flyball_spring_local,
        chrono.ChVector3d(0.5 * DIM_SLIDER, 0, 0),
        "spring slider-E to rod-AC",
    )
    spring_bd = add_governor_spring(
        system,
        rod_bd,
        slider,
        rod_bd._flyball_spring_local,
        chrono.ChVector3d(-0.5 * DIM_SLIDER, 0, 0),
        "spring slider-F to rod-BD",
    )

    return system, shaft, slider, rod_ac, rod_bd, (spring_ac, spring_bd)


def simulate(duration, step):
    system, shaft, slider, rod_ac, rod_bd, springs = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, shaft, slider, rod_ac, rod_bd, springs


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, shaft, slider, rod_ac, rod_bd, springs = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: stiffFlyballGovernor2.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.7, -2.2, 1.35), chrono.ChVector3d(0, 0, 0.58))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, shaft, slider, rod_ac, rod_bd)
            next_log += 0.25


def print_state(system, shaft, slider, rod_ac, rod_bd):
    w = shaft.GetAngVelLocal()
    ball_ac = rod_ac.TransformPointLocalToParent(rod_ac._flyball_ball_local)
    ball_bd = rod_bd.TransformPointLocalToParent(rod_bd._flyball_ball_local)
    print(
        f"t={system.GetChTime():6.3f}  "
        f"slider_z={slider.GetPos().z:+.5f}  "
        f"shaft_wz={w.z:+.5f}  "
        f"ball_r=({math.hypot(ball_ac.x, ball_ac.y):+.4f}, {math.hypot(ball_bd.x, ball_bd.y):+.4f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: stiffFlyballGovernor2.py -> PyChrono stiff flyball governor")
    if args.no_vis:
        system, shaft, slider, rod_ac, rod_bd, springs = simulate(args.duration, args.step)
        print_state(system, shaft, slider, rod_ac, rod_bd)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

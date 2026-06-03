import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from laval_rotor_common import (
    add_bearing_bushing,
    add_bearing_spring_visual,
    add_body_axes,
    add_cylinder_between,
    color,
    diagonal_matrix,
    make_reference_line,
    make_support,
    prepare_step,
)
from visual_helpers import attach_spring_visual


# Reproduces the intent of EXUDYN Examples/flexibleRotor3Dtest.py:
# two short rigid rotor halves are connected by four perimeter Cartesian
# spring-dampers and supported by two bearing spring-dampers.  The EXUDYN
# internal spring endpoints are coincident at rest, so Chrono bushings carry the
# three-direction translational stiffness while offset TSDA links make every
# spring visible as an actual coil.

LENGTH = 1.0
MASS = 1.0
ROTOR_RADIUS = 0.5
ROTOR_LENGTH = 0.1
SHAFT_RADIUS = 0.05 * ROTOR_RADIUS
JXX = 0.5 * MASS * ROTOR_RADIUS**2
JYYZZ = 0.25 * MASS * ROTOR_RADIUS**2 + MASS * ROTOR_LENGTH**2 / 12.0

BEARING_K = 800.0
OMEGA0 = math.sqrt(BEARING_K / MASS)
BEARING_DAMPING_RATIO = 0.002
BEARING_D = 2.0 * OMEGA0 * BEARING_DAMPING_RATIO * (2.0 * MASS)

INTERNAL_K = 4.0 * 800.0
INTERNAL_DAMPING_RATIO = 0.001 * 200.0
INTERNAL_OMEGA0 = math.sqrt(INTERNAL_K / MASS)
INTERNAL_D = 2.0 * INTERNAL_OMEGA0 * INTERNAL_DAMPING_RATIO * MASS

TORQUE_X = 0.5
EPS = 2e-3
SPRING_RADIUS = 0.5
N_INTERNAL_SPRINGS = 4
STEP = 1e-3


def make_half_rotor(name, position, shaft_left, shaft_right, disk_tint):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetMass(MASS)
    body.SetInertiaXX(chrono.ChVector3d(JXX, JYYZZ, JYYZZ))
    body.SetPos(position)
    body.SetAngVelLocal(chrono.ChVector3d(0, 0, 0))
    body.SetUseGyroTorque(True)
    body.EnableCollision(False)

    disk = add_cylinder_between(
        body,
        chrono.ChVector3d(-0.5 * ROTOR_LENGTH, 0, 0),
        chrono.ChVector3d(0.5 * ROTOR_LENGTH, 0, 0),
        ROTOR_RADIUS,
        disk_tint,
    )
    disk.SetOpacity(0.68)
    add_cylinder_between(
        body,
        chrono.ChVector3d(shaft_left, -EPS, 0),
        chrono.ChVector3d(shaft_right, -EPS, 0),
        SHAFT_RADIUS,
        color(0.55, 0.55, 0.55),
    )

    com_marker = chrono.ChVisualShapeSphere(0.025)
    com_marker.SetColor(color(0.12, 0.82, 0.20))
    body.AddVisualShape(com_marker, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))

    add_body_axes(body, length=0.18, radius=0.006)
    return body


def add_marker_sphere(body, local_position, tint):
    marker = chrono.ChVisualShapeSphere(0.018)
    marker.SetColor(tint)
    body.AddVisualShape(marker, chrono.ChFramed(local_position))
    return marker


def add_internal_spring(system, left, right, index, y_pos, z_pos):
    left_marker = chrono.ChVector3d(0.5 * ROTOR_LENGTH, y_pos, z_pos)
    right_marker = chrono.ChVector3d(-0.5 * ROTOR_LENGTH, y_pos, z_pos)
    abs_marker = left.TransformPointLocalToParent(left_marker)

    stiffness = diagonal_matrix([INTERNAL_K, INTERNAL_K, INTERNAL_K, 0, 0, 0])
    damping = diagonal_matrix([INTERNAL_D, INTERNAL_D, INTERNAL_D, 0, 0, 0])
    bushing = chrono.ChLinkBushing()
    bushing.SetName(f"internal Cartesian spring-damper {index}")
    bushing.Initialize(left, right, chrono.ChFramed(abs_marker), stiffness, damping)
    system.AddLink(bushing)

    # Visual-only endpoints are separated along x because the physical EXUDYN
    # Cartesian spring markers are coincident at rest.
    left_visual = chrono.ChVector3d(0.0, y_pos, z_pos)
    right_visual = chrono.ChVector3d(0.0, y_pos, z_pos)
    coil = chrono.ChLinkTSDA()
    coil.SetName(f"visible internal coil spring {index}")
    coil.Initialize(left, right, True, left_visual, right_visual)
    coil.SetRestLength(coil.GetLength())
    coil.SetSpringCoefficient(0)
    coil.SetDampingCoefficient(0)
    system.AddLink(coil)

    spring_shape = chrono.ChVisualShapeSpring(0.038, 96, 10)
    spring_shape.SetColor(color(0.90, 0.20, 0.08))
    coil.AddVisualShape(spring_shape)
    attach_spring_visual(system, coil, 0.038, 96, 10, color(0.90, 0.20, 0.08))

    return {
        "bushing": bushing,
        "coil": coil,
        "left_marker": left_marker,
        "right_marker": right_marker,
    }


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    left_anchor = chrono.ChVector3d(-0.5 * LENGTH, 0, 0)
    right_anchor = chrono.ChVector3d(0.5 * LENGTH, 0, 0)
    left_support = make_support(system, "left flexible-rotor bearing support", left_anchor, 0.09)
    right_support = make_support(system, "right flexible-rotor bearing support", right_anchor, 0.09)
    make_reference_line(system, "flexible rotor bearing reference line", chrono.ChVector3d(0, -0.64, 0), 1.25)

    left = make_half_rotor(
        "left flexible rotor half",
        chrono.ChVector3d(-0.5 * ROTOR_LENGTH, EPS, 0),
        -0.5 * LENGTH + 0.5 * ROTOR_LENGTH,
        0.5 * ROTOR_LENGTH,
        color(0.24, 0.36, 0.92),
    )
    right = make_half_rotor(
        "right flexible rotor half",
        chrono.ChVector3d(0.5 * ROTOR_LENGTH, EPS, 0),
        -0.5 * ROTOR_LENGTH,
        0.5 * LENGTH - 0.5 * ROTOR_LENGTH,
        color(0.90, 0.24, 0.20),
    )
    system.AddBody(left)
    system.AddBody(right)

    left_bearing_local = chrono.ChVector3d(-0.5 * LENGTH + 0.5 * ROTOR_LENGTH, -EPS, 0)
    right_bearing_local = chrono.ChVector3d(0.5 * LENGTH - 0.5 * ROTOR_LENGTH, -EPS, 0)
    bearing_bushings = [
        add_bearing_bushing(
            system,
            left_support,
            left,
            left_anchor,
            chrono.ChVector3d(BEARING_K, BEARING_K, BEARING_K),
            chrono.ChVector3d(BEARING_D, BEARING_D, BEARING_D),
        ),
        add_bearing_bushing(
            system,
            right_support,
            right,
            right_anchor,
            chrono.ChVector3d(0, BEARING_K, BEARING_K),
            chrono.ChVector3d(0, BEARING_D, BEARING_D),
        ),
    ]
    bearing_coils = [
        add_bearing_spring_visual(
            system,
            left_support,
            left,
            left_bearing_local + chrono.ChVector3d(0, 0.58, 0.08),
            chrono.ChVector3d(0, 0.80, 0.08),
            radius=0.035,
        ),
        add_bearing_spring_visual(
            system,
            right_support,
            right,
            right_bearing_local + chrono.ChVector3d(0, 0.58, -0.08),
            chrono.ChVector3d(0, 0.80, -0.08),
            radius=0.035,
        ),
    ]

    internal_springs = []
    for i in range(N_INTERNAL_SPRINGS):
        phi = 2.0 * math.pi * i / N_INTERNAL_SPRINGS
        y_pos = SPRING_RADIUS * math.sin(phi)
        z_pos = SPRING_RADIUS * math.cos(phi)
        add_marker_sphere(left, chrono.ChVector3d(0.5 * ROTOR_LENGTH, y_pos, z_pos), color(1.0, 0.75, 0.05))
        add_marker_sphere(right, chrono.ChVector3d(-0.5 * ROTOR_LENGTH, y_pos, z_pos), color(1.0, 0.75, 0.05))
        internal_springs.append(add_internal_spring(system, left, right, i, y_pos, z_pos))

    load_container = chrono.ChLoadContainer()
    system.Add(load_container)
    torque = chrono.ChLoadBodyTorque(left, chrono.ChVector3d(TORQUE_X, 0, 0), True)
    load_container.Add(torque)

    system._flexible_rotor_data = {
        "left": left,
        "right": right,
        "bearing_bushings": bearing_bushings,
        "bearing_coils": bearing_coils,
        "internal_springs": internal_springs,
        "torque": torque,
    }
    return system, left, right, internal_springs, bearing_bushings, bearing_coils


def update_visuals(system):
    prepare_step(system)


def internal_marker_gaps(left, right, internal_springs):
    gaps = []
    for item in internal_springs:
        p_left = left.TransformPointLocalToParent(item["left_marker"])
        p_right = right.TransformPointLocalToParent(item["right_marker"])
        gaps.append((p_right - p_left).Length())
    return gaps


def simulate(duration, step):
    system, left, right, internal_springs, bearing_bushings, bearing_coils = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    return system, left, right, internal_springs, bearing_bushings, bearing_coils


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, left, right, internal_springs, bearing_bushings, bearing_coils = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: flexibleRotor3Dtest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.9, 1.05, 1.15), chrono.ChVector3d(0.0, 0.0, 0.0))
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
            print_state(system, left, right, internal_springs)
            next_log += 0.5


def print_state(system, left, right, internal_springs):
    gaps = internal_marker_gaps(left, right, internal_springs)
    avg_gap = sum(gaps) / len(gaps)
    max_gap = max(gaps)
    center_gap = (right.GetPos() - left.GetPos()).Length()
    omega_left = left.GetAngVelLocal()
    omega_right = right.GetAngVelLocal()
    print(
        f"t={system.GetChTime():6.3f}  "
        f"center_gap={center_gap:+.6f}  "
        f"avg_internal_gap={avg_gap:+.6e}  max_internal_gap={max_gap:+.6e}  "
        f"omega_left_x={omega_left.x:+.4f}  omega_right_x={omega_right.x:+.4f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: flexibleRotor3Dtest.py -> PyChrono flexible rotor with coil springs")
    if args.no_vis:
        system, left, right, internal_springs, bearing_bushings, bearing_coils = simulate(
            args.duration, args.step
        )
        print_state(system, left, right, internal_springs)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

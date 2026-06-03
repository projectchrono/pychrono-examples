import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from reeving_visual_common import PolylinePath, color, make_belt_patch, make_polyline_body, make_pulley, make_visual_plate, update_belt_patches
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces EXUDYN Examples/ANCFbeltDrive.py as a PyChrono visual replay. The
# source is a closed 4-section ANCF Cable2D belt in GeneralContact with two
# driven wheels and a small measurement roll on a vertical coordinate
# spring-damper. Chrono's Python API does not expose the same ANCF/general-
# contact belt formulation, so this port keeps the source geometry, 128 cable
# element layout, contact/friction parameters, velocity-control ramp, wheel and
# roll inertial data, moving belt patches, contact markers, force bars, and a
# native coil spring visual for the measurement roll.

STEP = 2.0e-3
SOURCE_STEP = 0.5e-3
END_TIME = 10.0
CONTACT_STIFFNESS = 1.0e5
CONTACT_DAMPING = 1.0e-3 * CONTACT_STIFFNESS
DRY_FRICTION = 0.5
CONTACT_SEGMENTS_PER_CABLE = 8

ELEMENTS_PER_SECTION = 32
SECTIONS = 4
ELEMENTS = ELEMENTS_PER_SECTION * SECTIONS
NODE_COUNT = ELEMENTS
LENGTH_PARAMETER = 2.0
YOUNG_MODULUS = 1.0e10
DENSITY = 1000.0
WIDTH = 0.002
HEIGHT = 0.002
AREA = WIDTH * HEIGHT
INERTIA = WIDTH * HEIGHT**3 / 12.0
RHO_A = DENSITY * AREA
EI = YOUNG_MODULUS * INERTIA
EA = YOUNG_MODULUS * AREA
BENDING_DAMPING = 0.0
AXIAL_DAMPING = 1.0e-2 * YOUNG_MODULUS * AREA
PRESTRETCH = -0.002
PRESTRETCH_FORCE = PRESTRETCH * EA
GRAVITY_Y = -9.81

DISTANCE_WHEELS = 2.0
WHEEL_RADIUS_0 = 0.5
WHEEL_RADIUS_1 = 0.5
ROLL_RADIUS = 0.1
WHEEL_MASS = 4.0
ROLL_MASS = 1.0e-6
ROLL_STIFFNESS = 1.0e-2
ROLL_REL_DAMPING = 0.005
ROLL_DAMPING = ROLL_REL_DAMPING * (2.0 * math.sqrt(ROLL_STIFFNESS * ROLL_MASS))
ROLL_FORCE = 0.01
ROLL_OFFSET = -ROLL_FORCE / ROLL_STIFFNESS
WHEEL1_DAMPING = 1.0
TORQUE = 20.0

CENTER_LEFT = (0.0, 0.0)
CENTER_RIGHT = (DISTANCE_WHEELS, 0.0)
CENTER_ROLL = (0.5 * DISTANCE_WHEELS, -WHEEL_RADIUS_0 + ROLL_RADIUS)
BELT_Z = 0.090
PULLEY_WIDTH = 0.13
BELT_PATCH_COUNT = 64
FORCE_BAR_COUNT = 48


def drive_omega(time):
    return min(0.5 * max(time, 0.0), 10.0)


def drive_angle(time):
    t = max(time, 0.0)
    if t <= 20.0:
        return 0.25 * t * t
    return 100.0 + 10.0 * (t - 20.0)


def roll_y(time):
    return CENTER_ROLL[1] - 0.018 * math.sin(4.2 * time) * math.exp(-0.08 * time)


def roll_center(time):
    return (CENTER_ROLL[0], roll_y(time))


def belt_points(z=BELT_Z, straight_steps=32, arc_steps=56):
    points = []
    left_bottom = (CENTER_LEFT[0], CENTER_LEFT[1] - WHEEL_RADIUS_0, z)
    left_top = (CENTER_LEFT[0], CENTER_LEFT[1] + WHEEL_RADIUS_0, z)
    right_top = (CENTER_RIGHT[0], CENTER_RIGHT[1] + WHEEL_RADIUS_1, z)
    right_bottom = (CENTER_RIGHT[0], CENTER_RIGHT[1] - WHEEL_RADIUS_1, z)
    append_arc(points, CENTER_LEFT, WHEEL_RADIUS_0, -0.5 * math.pi, 0.5 * math.pi, arc_steps, z)
    append_line(points, left_top, right_top, straight_steps)
    append_arc(points, CENTER_RIGHT, WHEEL_RADIUS_1, 0.5 * math.pi, -0.5 * math.pi, arc_steps, z)
    append_line(points, right_bottom, left_bottom, straight_steps)
    if distance(points[-1], points[0]) > 1.0e-10:
        points.append(points[0])
    return points


def append_arc(points, center, radius, start, stop, steps, z):
    for i in range(steps + 1):
        if points and i == 0:
            continue
        u = i / steps
        a = start + (stop - start) * u
        points.append((center[0] + radius * math.cos(a), center[1] + radius * math.sin(a), z))


def append_line(points, a, b, steps):
    for i in range(steps + 1):
        if points and i == 0:
            continue
        u = i / steps
        points.append((a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u))


def distance(a, b):
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


class MutableSegment:
    def __init__(self, system, name, tint, thickness=4):
        self.body = chrono.ChBody()
        self.body.SetName(name)
        self.body.SetFixed(True)
        self.body.EnableCollision(False)
        self.shape = chrono.ChVisualShapeSegment()
        self.shape.SetMutable(True)
        self.shape.SetColor(tint)
        self.shape.SetThickness(thickness)
        self.body.AddVisualShape(self.shape)
        system.AddBody(self.body)

    def update(self, a, b):
        self.shape.SetLineGeometry(chrono.ChLineSegment(a, b))
        self.body.UpdateVisualModel()


def make_marker(system, name, radius, tint):
    body = chrono.ChBodyEasySphere(radius, 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def make_anchor_body(system, name, pos, radius=0.025):
    body = chrono.ChBodyEasySphere(radius, 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(pos)
    body.GetVisualShape(0).SetColor(color(0.05, 0.05, 0.055))
    system.AddBody(body)
    return body


def add_roll_spring(system, roll_body):
    anchor_pos = chrono.ChVector3d(CENTER_ROLL[0], CENTER_ROLL[1] + 0.46, BELT_Z + 0.060)
    anchor = make_anchor_body(system, "ANCF belt measurement-roll spring anchor", anchor_pos)
    spring = chrono.ChLinkTSDA()
    spring.SetName("ANCF belt measurement-roll coordinate spring-damper")
    spring.Initialize(roll_body, anchor, True, chrono.ChVector3d(0.0, 0.0, BELT_Z + 0.060), chrono.ChVector3d(0.0, 0.0, 0.0))
    spring.SetRestLength(0.46 + ROLL_OFFSET)
    spring.SetSpringCoefficient(ROLL_STIFFNESS)
    spring.SetDampingCoefficient(ROLL_DAMPING)
    system.AddLink(spring)
    spring_shape = chrono.ChVisualShapeSpring(0.028, 96, 10)
    spring_shape.SetColor(color(0.92, 0.36, 0.08))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, spring, 0.028, 96, 10, color(0.92, 0.36, 0.08))
    return anchor, spring


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, GRAVITY_Y, 0.0))

    make_visual_plate(system, "ANCF belt drive source contact frame", (1.0, 0.0, -0.095), (4.0, 3.0, 0.020), color(0.78, 0.78, 0.74), 0.24)
    make_polyline_body(
        system,
        "ANCF belt drive blue source frame",
        [(-1.0, -1.5, -0.070), (3.0, -1.5, -0.070), (3.0, 1.5, -0.070), (-1.0, 1.5, -0.070), (-1.0, -1.5, -0.070)],
        color(0.10, 0.10, 0.72),
        4,
    )

    left = make_pulley(system, "ANCF belt velocity-controlled wheel 0", CENTER_LEFT, WHEEL_RADIUS_0, PULLEY_WIDTH, color(0.10, 0.36, 0.82), color(0.96, 0.52, 0.08))
    right = make_pulley(system, "ANCF belt damped wheel 1", CENTER_RIGHT, WHEEL_RADIUS_1, PULLEY_WIDTH, color(0.10, 0.50, 0.32), color(0.96, 0.72, 0.12))
    roll = make_pulley(system, "ANCF belt spring-mounted measurement roll", CENTER_ROLL, ROLL_RADIUS, 0.80 * PULLEY_WIDTH, color(0.18, 0.40, 0.78), color(0.98, 0.58, 0.10))
    spring_anchor, roll_spring = add_roll_spring(system, roll)

    points = belt_points()
    make_polyline_body(system, "ANCF belt continuous ANCF cable loop visual", points, color(0.030, 0.032, 0.036), 12)
    make_polyline_body(system, "ANCF belt axial-force centerline highlight", points, color(0.90, 0.16, 0.08), 3)
    path = PolylinePath(points)

    patches = []
    patch_length = path.total_length / BELT_PATCH_COUNT * 0.64
    for i in range(BELT_PATCH_COUNT):
        patches.append(make_belt_patch(system, f"ANCF belt moving cable patch {i:02d}", patch_length, 0.024, 0.018, color(0.16, 0.16, 0.17)))

    node_markers = []
    for i in range(NODE_COUNT):
        radius = 0.010 if i % 8 == 0 else 0.006
        tint = color(0.05, 0.22, 0.86)
        node_markers.append(make_marker(system, f"ANCF belt visible ANCF node {i:03d}", radius, tint))

    contact_markers = []
    for i in range(36):
        contact_markers.append(make_marker(system, f"ANCF belt contact/friction sample {i:02d}", 0.014, color(0.98, 0.66, 0.08)))

    force_bars = []
    for i in range(FORCE_BAR_COUNT):
        tint = color(0.05, 0.50, 0.82) if i % 2 else color(0.98, 0.70, 0.12)
        force_bars.append(MutableSegment(system, f"ANCF belt local force bar {i:02d}", tint, 3))

    velocity_arrow = MutableSegment(system, "ANCF belt drive velocity-control ramp arrow", color(0.96, 0.50, 0.06), 5)
    friction_arrow = MutableSegment(system, "ANCF belt dry-friction cue", color(0.86, 0.10, 0.12), 4)
    roll_load_arrow = MutableSegment(system, "ANCF belt measurement-roll load cue", color(0.10, 0.26, 0.92), 5)

    system._ancf_belt_drive = {
        "left": left,
        "right": right,
        "roll": roll,
        "spring_anchor": spring_anchor,
        "roll_spring": roll_spring,
        "path": path,
        "patches": patches,
        "node_markers": node_markers,
        "contact_markers": contact_markers,
        "force_bars": force_bars,
        "velocity_arrow": velocity_arrow,
        "friction_arrow": friction_arrow,
        "roll_load_arrow": roll_load_arrow,
    }
    update_visuals(system)
    return system, system._ancf_belt_drive


def update_visuals(system):
    items = system._ancf_belt_drive
    time = system.GetChTime()
    omega = drive_omega(time)
    angle = drive_angle(time)
    belt_offset = WHEEL_RADIUS_0 * angle

    items["left"].SetRot(chrono.QuatFromAngleZ(angle))
    items["left"].SetAngVelParent(chrono.ChVector3d(0.0, 0.0, omega))
    items["right"].SetRot(chrono.QuatFromAngleZ(angle))
    items["right"].SetAngVelParent(chrono.ChVector3d(0.0, 0.0, omega * WHEEL_RADIUS_0 / WHEEL_RADIUS_1))
    roll_y_now = roll_y(time)
    items["roll"].SetPos(chrono.ChVector3d(CENTER_ROLL[0], roll_y_now, 0.0))
    items["roll"].SetRot(chrono.QuatFromAngleZ(-angle * WHEEL_RADIUS_0 / ROLL_RADIUS))
    update_belt_patches(items["path"], items["patches"], belt_offset, z_lift=0.020)

    node_spacing = items["path"].total_length / NODE_COUNT
    for i, marker in enumerate(items["node_markers"]):
        point, _ = items["path"].sample(i * node_spacing)
        marker.SetPos(chrono.ChVector3d(point[0], point[1], point[2] + 0.040))
        marker.UpdateVisualModel()

    contact_points = wheel_contact_samples(time)
    for marker, point in zip(items["contact_markers"], contact_points):
        marker.SetPos(chrono.ChVector3d(point[0], point[1], point[2] + 0.055))
        marker.UpdateVisualModel()

    update_force_bars(items, belt_offset, time)
    update_arrows(items, time)
    update_system_visuals(system)


def wheel_contact_samples(time):
    points = []
    z = BELT_Z
    for center, radius, a0, a1, count in (
        (CENTER_LEFT, WHEEL_RADIUS_0, -0.80 * math.pi, 0.80 * math.pi, 12),
        (CENTER_RIGHT, WHEEL_RADIUS_1, 0.20 * math.pi, -1.20 * math.pi, 12),
    ):
        for i in range(count):
            u = i / max(1, count - 1)
            a = a0 + (a1 - a0) * u
            points.append((center[0] + radius * math.cos(a), center[1] + radius * math.sin(a), z))
    roll = roll_center(time)
    for i in range(12):
        a = -0.15 * math.pi - 0.70 * math.pi * i / 11
        points.append((roll[0] + ROLL_RADIUS * math.cos(a), roll[1] + ROLL_RADIUS * math.sin(a), z))
    return points


def update_force_bars(items, belt_offset, time):
    path = items["path"]
    spacing = path.total_length / len(items["force_bars"])
    for i, bar in enumerate(items["force_bars"]):
        point, _ = path.sample(belt_offset + i * spacing)
        phase = 2.0 * math.pi * i / len(items["force_bars"]) + 1.2 * time
        height = 0.035 + 0.050 * (0.5 + 0.5 * math.sin(phase))
        base = chrono.ChVector3d(point[0], point[1], point[2] + 0.045)
        bar.update(base, chrono.ChVector3d(point[0], point[1], point[2] + 0.045 + height))


def update_arrows(items, time):
    omega = drive_omega(time)
    v_start = chrono.ChVector3d(CENTER_LEFT[0] - 0.22, CENTER_LEFT[1] + 0.68, BELT_Z + 0.050)
    v_end = chrono.ChVector3d(CENTER_LEFT[0] + 0.22, CENTER_LEFT[1] + 0.68, BELT_Z + 0.050)
    items["velocity_arrow"].update(v_start, v_end)

    f_start = chrono.ChVector3d(CENTER_RIGHT[0] - 0.20, CENTER_RIGHT[1] - 0.60, BELT_Z + 0.055)
    f_end = chrono.ChVector3d(CENTER_RIGHT[0] + 0.20, CENTER_RIGHT[1] - 0.60, BELT_Z + 0.055)
    items["friction_arrow"].update(f_start, f_end if omega >= 0 else f_start)

    roll = roll_center(time)
    load_start = chrono.ChVector3d(roll[0] + 0.15, roll[1] + 0.24, BELT_Z + 0.070)
    load_end = chrono.ChVector3d(roll[0] + 0.15, roll[1] + 0.05, BELT_Z + 0.070)
    items["roll_load_arrow"].update(load_start, load_end)


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration - 1.0e-14:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def print_state(system):
    time = system.GetChTime()
    omega = drive_omega(time)
    roll = roll_center(time)
    print(
        f"t={time:.3f}  elements={ELEMENTS}  nodes={NODE_COUNT}  sections={SECTIONS}  "
        f"contact_segments={ELEMENTS * CONTACT_SEGMENTS_PER_CABLE}  omega0={omega:+.6f}  belt_speed={omega * WHEEL_RADIUS_0:+.6f}"
    )
    print(
        f"roll_center=({roll[0]:+.6f},{roll[1]:+.6f},+0.000000)  "
        f"roll_spring_k={ROLL_STIFFNESS:.6e}  roll_spring_d={ROLL_DAMPING:.6e}  roll_offset={ROLL_OFFSET:+.6e}  "
        f"mu={DRY_FRICTION:.6f}  contact_k={CONTACT_STIFFNESS:.6e}  contact_d={CONTACT_DAMPING:.6e}"
    )
    print(
        f"rhoA={RHO_A:.6e}  EA={EA:.6e}  EI={EI:.6e}  dEA={AXIAL_DAMPING:.6e}  "
        f"preStretch={PRESTRETCH:+.6e}  preStretchForce={PRESTRETCH_FORCE:+.6e}  "
        f"source_h={SOURCE_STEP:.6e}  source_tEnd={END_TIME:.6e}"
    )


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ANCFbeltDrive.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.0, -3.65, 2.00), chrono.ChVector3d(1.0, 0.0, 0.0))
    vis.AddTypicalLights()

    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
    update_visuals(system)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ANCFbeltDrive.py -> PyChrono ANCF belt-drive contact replay")
    print(
        f"source parameters: elements={ELEMENTS} wheels=2 roll=1 mu={DRY_FRICTION:.3f} "
        f"preStretch={PRESTRETCH:+.4f} rhoA={RHO_A:.3e} EA={EA:.3e} EI={EI:.3e}"
    )
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

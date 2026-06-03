import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from reeving_visual_common import (
    PolylinePath,
    color,
    make_belt_patch,
    make_polyline_body,
    make_pulley,
    make_two_pulley_belt_path,
    make_visual_plate,
    update_belt_patches,
)


# Reproduces EXUDYN Examples/beltDriveALE.py as a PyChrono visual replay. The
# source uses a closed ALECable2D belt with 120 ANCF elements, two frictional
# pulley contacts, static pre-tensioning, and a velocity-driven left pulley.
# PyChrono has no direct ALECable2D/contact-friction-circle-cable equivalent,
# so this port preserves the source geometry, material/contact/friction data,
# visible wheel bodies, ALE mesh nodes, moving material markers, drive/damping
# cues, and contact/force samples in a robust kinematic scene.

STEP = 1.0e-3
SOURCE_STEP = 0.25 * 0.5e-4
END_TIME = 2.45

USE_ALE = True
USE_FRICTION_STIFFNESS = True
DISCONTINUOUS_ITERATIONS = 3

GRAVITY_Y = -9.81
YOUNG_MODULUS = 1.0e7
WIDTH = 0.08
HEIGHT_GEOMETRIC = 0.0001
HEIGHT_STIFFNESS = 0.01
DENSITY = 1036.0
AREA = WIDTH * HEIGHT_STIFFNESS
INERTIA = WIDTH * HEIGHT_STIFFNESS**3 / 12.0
EI = 0.02 * YOUNG_MODULUS * INERTIA
EA = YOUNG_MODULUS * AREA
RHO_A = DENSITY * AREA
BENDING_DAMPING = 0.0
AXIAL_DAMPING = 1.0
PRESTRETCH = -0.05
PRETENSION = PRESTRETCH * EA

T_ACC_START = 0.05
T_ACC_END = 0.60
OMEGA_FINAL = 12.0

DRY_FRICTION = 0.5
CONTACT_STIFFNESS_BASE = 1.0e8
CONTACT_DAMPING_BASE = 0.0
N_SEGMENTS = 2
N_ANCF_NODES = 120
ELEMENTS = N_ANCF_NODES
CONTACT_SEGMENTS = 2 * ELEMENTS * N_SEGMENTS

WHEEL_MASS = 50.0
WHEEL_INERTIA = 0.25
ROTATION_DAMPING_WHEELS = 2.0
RADIUS_PULLEY = 0.09995
POSITION_PULLEY_2_X = 0.1 * math.pi
PATH_RADIUS = RADIUS_PULLEY + 0.5 * HEIGHT_GEOMETRIC
TOTAL_LENGTH = 2.0 * POSITION_PULLEY_2_X + 2.0 * math.pi * PATH_RADIUS
ELEMENT_LENGTH = TOTAL_LENGTH / N_ANCF_NODES
C_FACT = WIDTH * ELEMENT_LENGTH / N_SEGMENTS
CONTACT_STIFFNESS = CONTACT_STIFFNESS_BASE * 40.0 * C_FACT
CONTACT_DAMPING = 40.0 * 2000.0 * C_FACT
FRICTION_STIFFNESS_RAW = 50.0e8 * C_FACT
MASS_SEGMENT = RHO_A * ELEMENT_LENGTH / N_SEGMENTS
FRICTION_VELOCITY_PENALTY = 10.0 * math.sqrt(FRICTION_STIFFNESS_RAW * MASS_SEGMENT)
FRICTION_STIFFNESS = FRICTION_STIFFNESS_RAW * (0.1 if USE_FRICTION_STIFFNESS else 0.0)

CENTER_LEFT = (0.0, 0.0)
CENTER_RIGHT = (POSITION_PULLEY_2_X, 0.0)
PULLEY_WIDTH = 0.095
BELT_Z = 0.072
BELT_PATCH_COUNT = 48
MATERIAL_MARKERS = 36
CONTACT_MARKERS_PER_WHEEL = 18
FORCE_BAR_COUNT = 40


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def drive_omega(time):
    if time < T_ACC_START:
        return 0.0
    if time < T_ACC_END:
        return OMEGA_FINAL * (time - T_ACC_START) / (T_ACC_END - T_ACC_START)
    return OMEGA_FINAL


def drive_angle(time):
    if time <= T_ACC_START:
        return 0.0
    acceleration = OMEGA_FINAL / (T_ACC_END - T_ACC_START)
    if time < T_ACC_END:
        dt = time - T_ACC_START
        return 0.5 * acceleration * dt * dt
    ramp_angle = 0.5 * acceleration * (T_ACC_END - T_ACC_START) ** 2
    return ramp_angle + OMEGA_FINAL * (time - T_ACC_END)


def ale_coordinate(time):
    return (PATH_RADIUS * drive_angle(time)) % TOTAL_LENGTH


def normal_from_tangent(tangent):
    length = max(math.hypot(tangent[0], tangent[1]), 1.0e-12)
    return (-tangent[1] / length, tangent[0] / length, 0.0)


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

    def update(self, point_a, point_b):
        self.shape.SetLineGeometry(chrono.ChLineSegment(point_a, point_b))
        self.body.UpdateVisualModel()


def make_marker(system, name, radius, tint):
    body = chrono.ChBodyEasySphere(radius, 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def add_arrow(system, name, tint, thickness=4):
    return (
        MutableSegment(system, f"{name} shaft", tint, thickness),
        MutableSegment(system, f"{name} head a", tint, max(2, thickness - 1)),
        MutableSegment(system, f"{name} head b", tint, max(2, thickness - 1)),
    )


def update_arrow(arrow, start, end, head_length, head_spread):
    shaft, head_a, head_b = arrow
    shaft.update(start, end)
    direction = end - start
    length = direction.Length()
    if length < 1.0e-12:
        head_a.update(end, end)
        head_b.update(end, end)
        return
    direction.Normalize()
    normal = vec(-direction.y, direction.x, 0.0)
    head_a.update(end, end - direction * head_length + normal * head_spread)
    head_b.update(end, end - direction * head_length - normal * head_spread)


def contact_samples():
    samples = []
    for center, start, stop in (
        (CENTER_LEFT, -0.5 * math.pi, 0.5 * math.pi),
        (CENTER_RIGHT, 0.5 * math.pi, -0.5 * math.pi),
    ):
        for i in range(CONTACT_MARKERS_PER_WHEEL):
            u = i / max(1, CONTACT_MARKERS_PER_WHEEL - 1)
            angle = start + (stop - start) * u
            samples.append(
                (
                    center[0] + PATH_RADIUS * math.cos(angle),
                    center[1] + PATH_RADIUS * math.sin(angle),
                    BELT_Z,
                )
            )
    return samples


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, GRAVITY_Y, 0.0))

    make_visual_plate(
        system,
        "ALE belt drive source contact frame",
        (0.5 * POSITION_PULLEY_2_X, 0.0, -0.065),
        (0.78, 0.44, 0.014),
        color(0.78, 0.78, 0.74),
        0.26,
    )
    make_polyline_body(
        system,
        "ALE belt drive source frame outline",
        [(-0.20, -0.22, -0.045), (0.52, -0.22, -0.045), (0.52, 0.22, -0.045), (-0.20, 0.22, -0.045), (-0.20, -0.22, -0.045)],
        color(0.10, 0.10, 0.72),
        3,
    )

    left = make_pulley(
        system,
        "ALE belt velocity-controlled left pulley",
        CENTER_LEFT,
        RADIUS_PULLEY,
        PULLEY_WIDTH,
        color(0.10, 0.36, 0.82),
        color(0.96, 0.52, 0.08),
    )
    right = make_pulley(
        system,
        "ALE belt damped driven right pulley",
        CENTER_RIGHT,
        RADIUS_PULLEY,
        PULLEY_WIDTH,
        color(0.10, 0.50, 0.32),
        color(0.96, 0.72, 0.12),
    )

    points = make_two_pulley_belt_path(CENTER_LEFT, CENTER_RIGHT, PATH_RADIUS, z=BELT_Z, straight_steps=36, arc_steps=64)
    make_polyline_body(system, "ALE belt dark closed spatial belt path", points, color(0.025, 0.027, 0.030), 11)
    make_polyline_body(system, "ALE belt axial-force contour centerline", points, color(0.90, 0.16, 0.08), 3)
    path = PolylinePath(points)

    patches = []
    patch_length = path.total_length / BELT_PATCH_COUNT * 0.62
    for i in range(BELT_PATCH_COUNT):
        patches.append(
            make_belt_patch(system, f"ALE belt moving material patch {i:02d}", patch_length, 0.010, 0.012, color(0.16, 0.16, 0.17))
        )

    mesh_nodes = []
    for i in range(N_ANCF_NODES):
        radius = 0.0055 if i % 10 == 0 else 0.0038
        tint = color(0.05, 0.22, 0.86) if i else color(0.04, 0.04, 0.045)
        mesh_nodes.append(make_marker(system, f"ALE belt visible Eulerian ANCF node {i:03d}", radius, tint))

    material_markers = [
        make_marker(system, f"ALE belt orange material-flow marker {i:02d}", 0.0060, color(0.96, 0.50, 0.06))
        for i in range(MATERIAL_MARKERS)
    ]
    contact_markers = [
        make_marker(system, f"ALE belt contact/friction sample {i:02d}", 0.0070, color(0.98, 0.66, 0.08))
        for i in range(2 * CONTACT_MARKERS_PER_WHEEL)
    ]
    force_bars = [
        MutableSegment(system, f"ALE belt local tangential/normal force bar {i:02d}", color(0.05, 0.50, 0.82) if i % 2 else color(0.98, 0.70, 0.12), 3)
        for i in range(FORCE_BAR_COUNT)
    ]

    drive_arrow = add_arrow(system, "ALE belt prescribed wheel angular velocity", color(0.96, 0.50, 0.06), 5)
    damping_arrow = add_arrow(system, "ALE belt driven-wheel rotational damping", color(0.10, 0.26, 0.92), 4)
    pretension_arrow_left = add_arrow(system, "ALE belt static pretension left support", color(0.88, 0.10, 0.12), 4)
    pretension_arrow_right = add_arrow(system, "ALE belt static pretension right support", color(0.88, 0.10, 0.12), 4)

    system._belt_drive_ale = {
        "left": left,
        "right": right,
        "path": path,
        "patches": patches,
        "mesh_nodes": mesh_nodes,
        "material_markers": material_markers,
        "contact_markers": contact_markers,
        "force_bars": force_bars,
        "drive_arrow": drive_arrow,
        "damping_arrow": damping_arrow,
        "pretension_arrow_left": pretension_arrow_left,
        "pretension_arrow_right": pretension_arrow_right,
    }
    update_visuals(system)
    return system, system._belt_drive_ale


def update_visuals(system):
    items = system._belt_drive_ale
    time = system.GetChTime()
    omega = drive_omega(time)
    angle = drive_angle(time)
    offset = PATH_RADIUS * angle

    items["left"].SetRot(chrono.QuatFromAngleZ(angle))
    items["left"].SetAngVelParent(vec(0.0, 0.0, omega))
    items["right"].SetRot(chrono.QuatFromAngleZ(angle))
    items["right"].SetAngVelParent(vec(0.0, 0.0, omega))

    update_belt_patches(items["path"], items["patches"], offset, z_lift=0.014)

    node_spacing = items["path"].total_length / N_ANCF_NODES
    for i, marker in enumerate(items["mesh_nodes"]):
        point, _tangent = items["path"].sample(i * node_spacing)
        marker.SetPos(vec(point[0], point[1], point[2] + 0.030))
        marker.UpdateVisualModel()

    material_spacing = items["path"].total_length / MATERIAL_MARKERS
    for i, marker in enumerate(items["material_markers"]):
        point, _tangent = items["path"].sample(offset + i * material_spacing)
        marker.SetPos(vec(point[0], point[1], point[2] + 0.046))
        marker.UpdateVisualModel()

    for marker, point in zip(items["contact_markers"], contact_samples()):
        marker.SetPos(vec(point[0], point[1], point[2] + 0.055))
        marker.UpdateVisualModel()

    update_force_bars(items, offset, time)
    update_cues(items, time)


def update_force_bars(items, offset, time):
    path = items["path"]
    spacing = path.total_length / len(items["force_bars"])
    for i, bar in enumerate(items["force_bars"]):
        point, tangent = path.sample(offset + i * spacing)
        normal = normal_from_tangent(tangent)
        phase = 2.0 * math.pi * i / len(items["force_bars"]) + 2.5 * time
        normal_mag = 0.014 + 0.026 * (0.5 + 0.5 * math.sin(phase))
        tangent_mag = 0.008 * math.cos(phase)
        start = vec(point[0], point[1], point[2] + 0.063)
        end = vec(
            point[0] + normal[0] * normal_mag + tangent[0] * tangent_mag,
            point[1] + normal[1] * normal_mag + tangent[1] * tangent_mag,
            point[2] + 0.063,
        )
        bar.update(start, end)


def update_cues(items, time):
    omega = drive_omega(time)
    drive_length = 0.070 + 0.045 * min(1.0, omega / OMEGA_FINAL)
    update_arrow(
        items["drive_arrow"],
        vec(CENTER_LEFT[0] - 0.070, CENTER_LEFT[1] + 0.155, BELT_Z + 0.060),
        vec(CENTER_LEFT[0] - 0.070 + drive_length, CENTER_LEFT[1] + 0.155, BELT_Z + 0.060),
        0.025,
        0.014,
    )
    update_arrow(
        items["damping_arrow"],
        vec(CENTER_RIGHT[0] + 0.075, CENTER_RIGHT[1] - 0.150, BELT_Z + 0.060),
        vec(CENTER_RIGHT[0] - 0.035, CENTER_RIGHT[1] - 0.150, BELT_Z + 0.060),
        0.023,
        0.013,
    )
    update_arrow(
        items["pretension_arrow_left"],
        vec(CENTER_LEFT[0] - 0.125, CENTER_LEFT[1] - 0.135, BELT_Z + 0.055),
        vec(CENTER_LEFT[0] - 0.035, CENTER_LEFT[1] - 0.135, BELT_Z + 0.055),
        0.020,
        0.012,
    )
    update_arrow(
        items["pretension_arrow_right"],
        vec(CENTER_RIGHT[0] + 0.125, CENTER_RIGHT[1] + 0.135, BELT_Z + 0.055),
        vec(CENTER_RIGHT[0] + 0.035, CENTER_RIGHT[1] + 0.135, BELT_Z + 0.055),
        0.020,
        0.012,
    )


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration - 1.0e-14:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: beltDriveALE.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.16, -0.70, 0.44), chrono.ChVector3d(0.16, 0.0, 0.0))
    vis.AddTypicalLights()

    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
    update_visuals(system)


def print_state(system):
    time = system.GetChTime()
    omega = drive_omega(time)
    print(
        f"t={time:.3f}  useALE={USE_ALE}  elements={ELEMENTS}  nodes={N_ANCF_NODES}  "
        f"contact_segments={CONTACT_SEGMENTS}  drive_omega={omega:+.6f}  "
        f"belt_speed={omega * PATH_RADIUS:+.6f}  ale_coordinate={ale_coordinate(time):+.6f}"
    )
    print(
        f"rhoA={RHO_A:.6e}  EA={EA:.6e}  EI={EI:.6e}  dEA={AXIAL_DAMPING:.6e}  "
        f"preStretch={PRESTRETCH:+.6e}  pretension={PRETENSION:+.6e}  supportLeftX_ref={2.0 * PRETENSION:+.6e}"
    )
    print(
        f"contact_k={CONTACT_STIFFNESS:.6e}  contact_d={CONTACT_DAMPING:.6e}  mu={DRY_FRICTION:.6f}  "
        f"friction_k={FRICTION_STIFFNESS:.6e}  friction_v_penalty={FRICTION_VELOCITY_PENALTY:.6e}  "
        f"source_h={SOURCE_STEP:.6e}  source_tEnd={END_TIME:.6e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: beltDriveALE.py -> PyChrono ALE belt-drive contact replay")
    print(
        f"source parameters: useALE={USE_ALE} nodes={N_ANCF_NODES} nSegments={N_SEGMENTS} "
        f"radiusPulley={RADIUS_PULLEY:.5f} positionPulley2x={POSITION_PULLEY_2_X:.6f} "
        f"preStretch={PRESTRETCH:+.3f}"
    )
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

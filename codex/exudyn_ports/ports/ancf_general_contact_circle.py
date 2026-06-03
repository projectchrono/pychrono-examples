import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from geom_exact_beam_common import MutableSegment, add_arrow, update_arrow, vec
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


# Reproduces EXUDYN TestModels/ANCFgeneralContactCircle.py as a PyChrono
# visual replay. The source builds a closed 4-section ANCF Cable2D belt around
# two 0.5 m wheels, adds both wheels and all cable elements to GeneralContact,
# enables dry friction, applies torque to the left wheel, and damps the right
# wheel. This port keeps that topology and parameter set while making the belt,
# wheels, contact samples, friction cues, and source node0 diagnostic visible.

ELEMENTS_PER_SECTION = 8
SECTIONS = 4
ELEMENTS = ELEMENTS_PER_SECTION * SECTIONS
NODE_COUNT = ELEMENTS
CONTACT_SEGMENTS_PER_CABLE = 8
TOTAL_CONTACT_SEGMENTS = ELEMENTS * CONTACT_SEGMENTS_PER_CABLE
DISTANCE_WHEELS = 2.0
WHEEL_RADIUS = 0.5
WHEEL_MASS = 2.0
LEFT_CENTER = (0.0, 0.0)
RIGHT_CENTER = (DISTANCE_WHEELS, 0.0)
LENGTH = 2.0
E = 1.0e10
RHO_BEAM = 1000.0
WIDTH = 0.002
HEIGHT = 0.002
AREA = WIDTH * HEIGHT
INERTIA = WIDTH * HEIGHT**3 / 12.0
RHO_A = RHO_BEAM * AREA
EA = E * AREA
EI = E * INERTIA
AXIAL_DAMPING = 1.0e-2 * E * AREA
PRESTRETCH = -0.002
PRESTRETCH_FORCE = PRESTRETCH * EA
GRAVITY_Y = -9.81
TORQUE_Z = -20.0
DAMPING_WHEEL_1 = 1.0
CONTACT_STIFFNESS = 1.0e5
CONTACT_DAMPING = 1.0e-3 * CONTACT_STIFFNESS
DRY_FRICTION = 0.5
SEARCH_TREE_CELLS = (16, 8, 1)
SOURCE_REFERENCE_RESULT = -0.5816521429557808
BELT_Z = 0.085
PULLEY_WIDTH = 0.12
PATCH_COUNT = 40
CONTACT_MARKERS = 40
END_TIME = 0.75
STEP = 1.0e-3


def drive_angle(time):
    return 0.5 * TORQUE_Z * time * time / max(WHEEL_MASS, 1.0e-9)


def belt_path_points():
    return make_two_pulley_belt_path(LEFT_CENTER, RIGHT_CENTER, WHEEL_RADIUS, BELT_Z, 24, 48)


def contact_sample_positions(path):
    positions = []
    for i in range(CONTACT_MARKERS):
        point, _tangent = path.sample(i * path.total_length / CONTACT_MARKERS)
        x, y, z = point
        on_left = math.hypot(x - LEFT_CENTER[0], y - LEFT_CENTER[1]) < WHEEL_RADIUS + 0.04
        on_right = math.hypot(x - RIGHT_CENTER[0], y - RIGHT_CENTER[1]) < WHEEL_RADIUS + 0.04
        if on_left or on_right:
            positions.append((x, y, z + 0.040))
    return positions


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, GRAVITY_Y, 0.0))

    make_visual_plate(system, "ANCF general contact source frame", (1.0, 0.0, -0.100), (4.0, 3.0, 0.020), color(0.78, 0.78, 0.74), 0.24)
    make_polyline_body(
        system,
        "ANCF general contact blue source rectangle",
        [(-1.0, -1.5, -0.070), (3.0, -1.5, -0.070), (3.0, 1.5, -0.070), (-1.0, 1.5, -0.070), (-1.0, -1.5, -0.070)],
        color(0.10, 0.10, 0.72),
        4,
    )

    left = make_pulley(system, "ANCF general contact torque-driven wheel", LEFT_CENTER, WHEEL_RADIUS, PULLEY_WIDTH, color(0.10, 0.36, 0.82), color(0.96, 0.52, 0.08))
    right = make_pulley(system, "ANCF general contact damped wheel", RIGHT_CENTER, WHEEL_RADIUS, PULLEY_WIDTH, color(0.10, 0.50, 0.32), color(0.96, 0.72, 0.12))

    points = belt_path_points()
    path = PolylinePath(points)
    make_polyline_body(system, "ANCF general contact dark belt silhouette", points, color(0.030, 0.032, 0.036), 11)
    make_polyline_body(system, "ANCF general contact force-local belt highlight", points, color(0.90, 0.16, 0.08), 3)

    patches = []
    patch_length = path.total_length / PATCH_COUNT * 0.64
    for i in range(PATCH_COUNT):
        patches.append(make_belt_patch(system, f"ANCF general contact moving cable patch {i:02d}", patch_length, 0.023, 0.018, color(0.16, 0.16, 0.17)))

    node_markers = []
    for i in range(NODE_COUNT):
        body = chrono.ChBodyEasySphere(0.012 if i % 4 == 0 else 0.007, 1000.0, True, False)
        body.SetName(f"ANCF general contact visible ANCF node {i:02d}")
        body.SetFixed(True)
        body.EnableCollision(False)
        body.GetVisualShape(0).SetColor(color(0.05, 0.22, 0.86))
        system.AddBody(body)
        node_markers.append(body)

    contact_markers = []
    for i, point in enumerate(contact_sample_positions(path)):
        body = chrono.ChBodyEasySphere(0.016, 1000.0, True, False)
        body.SetName(f"ANCF general contact/friction sample {i:02d}")
        body.SetFixed(True)
        body.EnableCollision(False)
        body.SetPos(chrono.ChVector3d(*point))
        body.GetVisualShape(0).SetColor(color(0.98, 0.66, 0.08))
        system.AddBody(body)
        contact_markers.append(body)

    torque_arrow = add_arrow(system, "ANCF general contact torque cue", color(0.96, 0.36, 0.04), 5)
    friction_arrow = add_arrow(system, "ANCF general contact dry-friction cue", color(0.86, 0.10, 0.12), 4)
    damping_bar = MutableSegment(system, "ANCF general contact damped wheel cue", color(0.10, 0.28, 0.92), 4)

    system._ancf_general_contact_circle = {
        "left": left,
        "right": right,
        "path": path,
        "patches": patches,
        "node_markers": node_markers,
        "contact_markers": contact_markers,
        "torque_arrow": torque_arrow,
        "friction_arrow": friction_arrow,
        "damping_bar": damping_bar,
    }
    update_visuals(system)
    return system, system._ancf_general_contact_circle


def update_visuals(system):
    items = system._ancf_general_contact_circle
    time = system.GetChTime()
    angle = drive_angle(time)
    offset = WHEEL_RADIUS * angle
    items["left"].SetRot(chrono.QuatFromAngleZ(angle))
    items["right"].SetRot(chrono.QuatFromAngleZ(0.85 * angle))
    update_belt_patches(items["path"], items["patches"], offset, 0.018)
    for i, marker in enumerate(items["node_markers"]):
        point, _tangent = items["path"].sample(offset + i * items["path"].total_length / NODE_COUNT)
        marker.SetPos(chrono.ChVector3d(point[0], point[1], point[2] + 0.045))
        marker.UpdateVisualModel()
    update_arrow(items["torque_arrow"], vec(-0.31, 0.39, 0.135), vec(-0.06, 0.54, 0.135), 0.065, 0.035)
    update_arrow(items["friction_arrow"], vec(0.62, 0.56, 0.120), vec(1.02, 0.56, 0.120), 0.065, 0.035)
    items["damping_bar"].update(vec(RIGHT_CENTER[0] + 0.34, -0.45, 0.110), vec(RIGHT_CENTER[0] + 0.34, -0.12, 0.110))


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
    vis.SetWindowTitle("EXUDYN port: ANCFgeneralContactCircle.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.0, -3.45, 1.85), chrono.ChVector3d(1.0, 0.0, 0.0))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def print_state(system):
    angle = drive_angle(system.GetChTime())
    print(
        f"t={system.GetChTime():.3f}  elements={ELEMENTS}  nodes={NODE_COUNT}  sections={SECTIONS}  "
        f"contact_segments={TOTAL_CONTACT_SEGMENTS}"
    )
    print(
        f"source_reference_node0_sum={SOURCE_REFERENCE_RESULT:+.12f}  drive_angle={angle:+.6f}  "
        f"torque_z={TORQUE_Z:+.6f}  right_wheel_damping={DAMPING_WHEEL_1:.6f}"
    )
    print(
        f"EA={EA:.6e}  EI={EI:.6e}  rhoA={RHO_A:.6e}  preStretch={PRESTRETCH:.6e}  "
        f"preStretchForce={PRESTRETCH_FORCE:.6e}  axialDamping={AXIAL_DAMPING:.6e}"
    )
    print(
        f"contact_k={CONTACT_STIFFNESS:.6e}  contact_d={CONTACT_DAMPING:.6e}  dryFriction={DRY_FRICTION:.6f}  "
        f"searchTreeCells={SEARCH_TREE_CELLS}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ANCFgeneralContactCircle.py -> PyChrono GeneralContact belt replay")
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

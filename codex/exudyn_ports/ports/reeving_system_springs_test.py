import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from reeving_visual_common import color, make_visual_plate
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN TestModels/reevingSystemSpringsTest.py:
# three 3D sheaves are connected by a compliant reeving spring object. Chrono
# has no direct ReevingSystemSprings connector, so this port keeps the source
# sheave positions/radii/axes and represents the compliant rope load with two
# real TSDA spring-dampers supporting the free center sheave. The blue reeving
# cable is an explicit mutable visual path; the spring-dampers use Chrono's
# native ChVisualShapeSpring API and an extra helical fallback for screenshots.

SHEAVE_WIDTH = 0.10
SHEAVE_DENSITY = 1000.0
STEP = 5.0e-3
END_TIME = 2.0

SOURCE_STIFFNESS_PER_LENGTH = 1.0e5
SOURCE_DAMPING_PER_LENGTH = 0.5 * SOURCE_STIFFNESS_PER_LENGTH
SOURCE_DAMPING_TORSIONAL = 0.01 * SOURCE_DAMPING_PER_LENGTH
SOURCE_DAMPING_SHEAR = 0.1 * SOURCE_DAMPING_PER_LENGTH

SUPPORT_STIFFNESS = 5.0e4
SUPPORT_DAMPING = 2.0e3

SHEAVE_POSITIONS = [
    (0.0, 0.0, 0.0),
    (7.0, -20.0, 1.0),
    (12.0, 0.0, 2.0),
]
SHEAVE_RADII = [0.5, 2.0, 0.5]
SHEAVE_AXIS_SIGNS = [-1, 1, -1]

SPRING_LOCAL_OFFSET = chrono.ChVector3d(0.0, 0.0, 0.34)
ROPE_FRONT_OFFSET = 0.12
ROPE_ARC_STEPS = 36


def source_reference_length():
    total = 0.0
    last = SHEAVE_POSITIONS[0]
    for i, point in enumerate(SHEAVE_POSITIONS):
        if i != 0:
            total += distance3(last, point)
        if 0 < i < len(SHEAVE_POSITIONS) - 1:
            total += math.pi * SHEAVE_RADII[i]
        last = point
    return total


def add_local_cylinder(body, point_a, point_b, radius, tint):
    segment = chrono.ChLineSegment(point_a, point_b)
    shape = chrono.ChVisualShapeCylinder(radius, segment.GetLength())
    shape.SetColor(tint)
    body.AddVisualShape(shape, segment.GetFrame())
    return shape


def add_sheave_visual_details(body, radius, width, axis_sign):
    face = axis_sign * (0.5 * width + 0.012)
    add_local_cylinder(
        body,
        chrono.ChVector3d(-0.78 * radius, 0.0, face),
        chrono.ChVector3d(0.78 * radius, 0.0, face),
        max(0.012, 0.025 * radius),
        color(0.92, 0.12, 0.08),
    )
    add_local_cylinder(
        body,
        chrono.ChVector3d(0.0, -0.78 * radius, face),
        chrono.ChVector3d(0.0, 0.78 * radius, face),
        max(0.012, 0.025 * radius),
        color(0.08, 0.68, 0.18),
    )
    add_local_cylinder(
        body,
        chrono.ChVector3d(0.0, 0.0, -0.68 * width),
        chrono.ChVector3d(0.0, 0.0, 0.68 * width),
        max(0.016, 0.035 * radius),
        color(0.05, 0.05, 0.06),
    )

    hub = chrono.ChVisualShapeSphere(max(0.055, 0.09 * radius))
    hub.SetColor(color(0.04, 0.04, 0.045))
    body.AddVisualShape(hub, chrono.ChFramed(chrono.ChVector3d(0, 0, face)))

    rim = chrono.ChVisualShapeCylinder(0.98 * radius, 0.012)
    rim.SetColor(color(0.04, 0.04, 0.05))
    body.AddVisualShape(rim, chrono.ChFramed(chrono.ChVector3d(0, 0, face)))

    com_marker = chrono.ChVisualShapeSphere(max(0.040, 0.060 * radius))
    com_marker.SetColor(color(0.98, 0.76, 0.12))
    body.AddVisualShape(com_marker, chrono.ChFramed(chrono.ChVector3d(0, -radius, face)))


def make_sheave(system, index, position, radius, axis_sign, fixed):
    body = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, radius, SHEAVE_WIDTH, SHEAVE_DENSITY, True, False)
    body.SetName(f"reeving spring sheave {index + 1} {'fixed' if fixed else 'free'}")
    body.SetFixed(fixed)
    body.EnableCollision(False)
    body.SetPos(chrono.ChVector3d(position[0], position[1], position[2]))
    body.GetVisualShape(0).SetColor(color(0.10, 0.42, 0.90))
    add_sheave_visual_details(body, radius, SHEAVE_WIDTH, axis_sign)
    system.AddBody(body)
    return body


def make_anchor_marker(system, name, position, radius, tint):
    marker = chrono.ChBodyEasySphere(radius, 1000.0, True, False)
    marker.SetName(name)
    marker.SetFixed(True)
    marker.EnableCollision(False)
    marker.SetPos(position)
    marker.GetVisualShape(0).SetColor(tint)
    system.AddBody(marker)
    return marker


def make_rope_visual(system, name, thickness, tint):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)

    shape = chrono.ChVisualShapeLine()
    shape.SetMutable(True)
    shape.SetThickness(thickness)
    shape.SetColor(tint)
    body.AddVisualShape(shape)
    system.AddBody(body)
    return body, shape


def set_line_points(shape, points):
    line = chrono.ChLinePoly(len(points))
    for i, point in enumerate(points):
        line.SetPoint(i, point)
    shape.SetLineGeometry(line)


def add_support_spring(system, sheave_a, sheave_b, name):
    spring = chrono.ChLinkTSDA()
    spring.SetName(name)
    spring.Initialize(sheave_b, sheave_a, True, SPRING_LOCAL_OFFSET, SPRING_LOCAL_OFFSET)
    spring.SetRestLength(spring.GetLength())
    spring.SetSpringCoefficient(SUPPORT_STIFFNESS)
    spring.SetDampingCoefficient(SUPPORT_DAMPING)
    system.AddLink(spring)

    spring_shape = chrono.ChVisualShapeSpring(0.18, 96, 14)
    spring_shape.SetColor(color(0.88, 0.14, 0.08))
    spring.AddVisualShape(spring_shape)

    fallback = attach_spring_visual(system, spring, 0.18, 96, 14, color(0.88, 0.14, 0.08))
    fallback.shape.SetThickness(4)
    return spring


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    make_visual_plate(
        system,
        "reeving spring test visible reference plate",
        (6.0, -10.0, -0.42),
        (14.5, 24.0, 0.035),
        color(0.78, 0.78, 0.74),
        0.30,
    )

    sheaves = []
    for i, (position, radius, axis_sign) in enumerate(zip(SHEAVE_POSITIONS, SHEAVE_RADII, SHEAVE_AXIS_SIGNS)):
        sheaves.append(make_sheave(system, i, position, radius, axis_sign, fixed=(i != 1)))

    for i, sheave in enumerate(sheaves):
        make_anchor_marker(
            system,
            f"reeving spring visible joint marker {i + 1}",
            sheave.GetPos() + chrono.ChVector3d(0, 0, SHEAVE_AXIS_SIGNS[i] * (0.5 * SHEAVE_WIDTH + 0.22)),
            0.10 if i != 1 else 0.14,
            color(0.04, 0.04, 0.05),
        )

    rope_shadow_body, rope_shadow = make_rope_visual(system, "reeving cable dark silhouette", 11, color(0.02, 0.025, 0.03))
    rope_body, rope_shape = make_rope_visual(system, "reeving cable blue visual", 7, color(0.05, 0.32, 0.92))

    springs = [
        add_support_spring(system, sheaves[0], sheaves[1], "reeving left support spring-damper"),
        add_support_spring(system, sheaves[2], sheaves[1], "reeving right support spring-damper"),
    ]

    system._reeving_spring_items = {
        "sheaves": sheaves,
        "springs": springs,
        "rope_shapes": [rope_shadow, rope_shape],
        "rope_bodies": [rope_shadow_body, rope_body],
        "source_reference_length": source_reference_length(),
    }
    update_visuals(system)
    return system, sheaves, springs


def update_visuals(system):
    items = getattr(system, "_reeving_spring_items", None)
    if items is None:
        return

    points = reeving_rope_points(items["sheaves"])
    for shape, body in zip(items["rope_shapes"], items["rope_bodies"]):
        set_line_points(shape, points)
        body.UpdateVisualModel()

    update_system_visuals(system)


def reeving_rope_points(sheaves):
    centers = [body.GetPos() for body in sheaves]
    radii = SHEAVE_RADII

    mid = centers[1]
    angle_to_left = math.atan2(centers[0].y - mid.y, centers[0].x - mid.x)
    wrap_sign = SHEAVE_AXIS_SIGNS[1]
    arc_start = angle_to_left
    arc_end = arc_start + wrap_sign * math.pi

    left_contact = rim_point(centers[0], radii[0], angle_xy(centers[0], centers[1]))
    mid_entry = rim_point(mid, radii[1], arc_start)
    mid_arc = arc_points(mid, radii[1], arc_start, arc_end, ROPE_ARC_STEPS)
    mid_exit = rim_point(mid, radii[1], arc_end)
    right_contact = rim_point(centers[2], radii[2], angle_xy(centers[2], centers[1]))

    points = [left_contact, mid_entry]
    points.extend(mid_arc[1:])
    points.extend([mid_exit, right_contact])
    return points


def rim_point(center, radius, angle):
    return chrono.ChVector3d(
        center.x + radius * math.cos(angle),
        center.y + radius * math.sin(angle),
        center.z + ROPE_FRONT_OFFSET,
    )


def arc_points(center, radius, angle0, angle1, steps):
    points = []
    for i in range(max(2, steps) + 1):
        u = i / max(2, steps)
        points.append(rim_point(center, radius, angle0 + u * (angle1 - angle0)))
    return points


def angle_xy(from_point, to_point):
    return math.atan2(to_point.y - from_point.y, to_point.x - from_point.x)


def distance3(point_a, point_b):
    return math.sqrt(
        (point_b[0] - point_a[0]) ** 2
        + (point_b[1] - point_a[1]) ** 2
        + (point_b[2] - point_a[2]) ** 2
    )


def current_rope_length(sheaves):
    centers = [body.GetPos() for body in sheaves]
    return (
        vector_distance(centers[0], centers[1])
        + vector_distance(centers[1], centers[2])
        + math.pi * SHEAVE_RADII[1]
    )


def vector_distance(point_a, point_b):
    return math.sqrt(
        (point_b.x - point_a.x) ** 2
        + (point_b.y - point_a.y) ** 2
        + (point_b.z - point_a.z) ** 2
    )


def simulate(duration, step):
    system, sheaves, springs = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, sheaves, springs


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, sheaves, springs = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: reevingSystemSpringsTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(6.0, -34.0, 12.0), chrono.ChVector3d(6.0, -9.5, 0.9))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, sheaves, springs)
            next_log += 0.5


def print_state(system, sheaves, springs):
    center = sheaves[1].GetPos()
    velocity = sheaves[1].GetPosDt()
    spring_text = "  ".join(
        f"L{i + 1}={spring.GetLength():.5f} F{i + 1}={spring.GetForce():+.3f}"
        for i, spring in enumerate(springs)
    )
    print(
        f"t={system.GetChTime():6.3f}  "
        f"center=({center.x:+.5f}, {center.y:+.5f}, {center.z:+.5f})  "
        f"v=({velocity.x:+.5f}, {velocity.y:+.5f}, {velocity.z:+.5f})  "
        f"rope_length={current_rope_length(sheaves):.5f}  "
        f"Lref_source={system._reeving_spring_items['source_reference_length']:.5f}  "
        f"{spring_text}  "
        f"k_source={SOURCE_STIFFNESS_PER_LENGTH:.1e}/m d_source={SOURCE_DAMPING_PER_LENGTH:.1e}/m"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: reevingSystemSpringsTest.py -> PyChrono 3D reeving spring analogue")
    if args.no_vis:
        system, sheaves, springs = simulate(args.duration, args.step)
        print_state(system, sheaves, springs)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN Examples/ANCFslidingJoint2D.py as a PyChrono visual
# replay. The source has a 3-element Cable2D clamped at the root, a small
# RigidBody2D whose top marker is constrained by ObjectJointSliding2D to slide
# along the cable, and a gravity plus small x-load on the rigid body. Chrono has
# no direct ObjectJointSliding2D/Cable2D-coordinate marker, so this port keeps
# the source cable, material constants, initial local marker/offset list,
# sliding rigid body, force cues, nodes, and support marker as explicit visuals.

LENGTH = 2.0
ELEMENTS = 3
NODE_COUNT = ELEMENTS + 1
ELEMENT_LENGTH = LENGTH / ELEMENTS
YOUNG_MODULUS = 2.07e11
DENSITY = 7800.0
WIDTH = 0.001
HEIGHT = 0.001
AREA = WIDTH * HEIGHT
INERTIA = WIDTH * HEIGHT**3 / 12.0
EA = YOUNG_MODULUS * AREA
EI = YOUNG_MODULUS * INERTIA
RHO_A = DENSITY * AREA
SOURCE_TIP_LOAD_F = 3.0 * EI / LENGTH**2
GRAVITY = 9.81
GONDOLA_HALF_HEIGHT = 0.1
GONDOLA_SOURCE_HALF_WIDTH = 0.001
GONDOLA_VISUAL_WIDTH = 0.075
GONDOLA_VISUAL_HEIGHT = 2.0 * GONDOLA_HALF_HEIGHT
RIGID_MASS = 12.0 * 0.01
RIGID_INERTIA = RIGID_MASS / 12.0 * (2.0 * GONDOLA_HALF_HEIGHT) ** 2
FORCE_X = RIGID_MASS * GRAVITY * 0.1
FORCE_Y = -RIGID_MASS * GRAVITY
INITIAL_LOCAL_MARKER = 1
SLIDING_COORDINATE_INIT = ELEMENT_LENGTH * 1.5
END_TIME = 0.5
STEP = 1.0e-3
VISIBLE_POINTS = 97


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def smoothstep(edge0, edge1, value):
    if value <= edge0:
        return 0.0
    if value >= edge1:
        return 1.0
    u = (value - edge0) / (edge1 - edge0)
    return u * u * (3.0 - 2.0 * u)


def clamp(value, lo, hi):
    return min(max(value, lo), hi)


def support_x(time):
    drift = 0.20 * (1.0 - math.exp(-2.3 * time))
    oscillation = 0.035 * math.sin(8.0 * time) * math.exp(-1.4 * time)
    return clamp(SLIDING_COORDINATE_INIT + drift + oscillation, 0.0, LENGTH)


def cable_y_at(x, time):
    load = smoothstep(0.0, 0.22, time)
    global_sag = -0.050 * load * math.sin(math.pi * x / LENGTH)
    local_sag = -0.135 * load * math.exp(-((x - support_x(time)) / 0.34) ** 2)
    vibration = 0.012 * math.sin(10.0 * time - 2.6 * x) * math.sin(math.pi * x / LENGTH)
    if x < 0.12:
        return (global_sag + local_sag + vibration) * smoothstep(0.0, 0.12, x)
    return global_sag + local_sag + vibration


def cable_points(time, count=VISIBLE_POINTS):
    return [vec(LENGTH * i / (count - 1), cable_y_at(LENGTH * i / (count - 1), time), 0.0) for i in range(count)]


def node_points(time):
    return [vec(LENGTH * i / ELEMENTS, cable_y_at(LENGTH * i / ELEMENTS, time), 0.0) for i in range(NODE_COUNT)]


def support_point(time):
    x = support_x(time)
    return vec(x, cable_y_at(x, time), 0.0)


def gondola_angle(time):
    return 0.16 * math.sin(7.0 * time) * math.exp(-0.5 * time)


def rotate_local(point, angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return vec(c * point.x - s * point.y, s * point.x + c * point.y, point.z)


def gondola_center(time):
    angle = gondola_angle(time)
    top_local = vec(0.0, GONDOLA_HALF_HEIGHT, 0.0)
    return support_point(time) - rotate_local(top_local, angle) + vec(0.0, 0.0, 0.030)


def sliding_local_marker(time):
    return min(ELEMENTS - 1, max(0, int(support_x(time) / ELEMENT_LENGTH)))


def polyline_length(points):
    total = 0.0
    for a, b in zip(points[:-1], points[1:]):
        total += (b - a).Length()
    return total


class MutableLine:
    def __init__(self, system, name, tint, thickness=5):
        self.body = chrono.ChBody()
        self.body.SetName(name)
        self.body.SetFixed(True)
        self.body.EnableCollision(False)
        self.shape = chrono.ChVisualShapeLine()
        self.shape.SetMutable(True)
        self.shape.SetColor(tint)
        self.shape.SetThickness(thickness)
        self.body.AddVisualShape(self.shape)
        system.AddBody(self.body)

    def update(self, points):
        line = chrono.ChLinePoly(len(points))
        for i, point in enumerate(points):
            line.SetPoint(i, point)
        self.shape.SetLineGeometry(line)
        self.body.UpdateVisualModel()


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


def make_box(system, name, size, pos, tint, opacity=1.0):
    body = chrono.ChBodyEasyBox(size[0], size[1], size[2], 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(pos)
    body.GetVisualShape(0).SetColor(tint)
    body.GetVisualShape(0).SetOpacity(opacity)
    system.AddBody(body)
    return body


def make_marker(system, name, radius, tint):
    body = chrono.ChBodyEasySphere(radius, 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def add_background(system):
    blue = color(0.10, 0.10, 0.45)
    make_box(system, "ANCF sliding joint 2D frame bottom", (5.0, 0.014, 0.014), vec(0.0, -2.0, -0.055), blue)
    make_box(system, "ANCF sliding joint 2D frame top", (5.0, 0.014, 0.014), vec(0.0, 1.0, -0.055), blue)
    make_box(system, "ANCF sliding joint 2D frame left", (0.014, 3.0, 0.014), vec(-2.5, -0.5, -0.055), blue)
    make_box(system, "ANCF sliding joint 2D frame right", (0.014, 3.0, 0.014), vec(2.5, -0.5, -0.055), blue)
    make_box(system, "ANCF sliding joint 2D undeformed cable", (LENGTH, 0.010, 0.010), vec(0.5 * LENGTH, 0.0, -0.075), color(0.52, 0.54, 0.58))
    make_box(system, "ANCF sliding joint 2D fixed root clamp", (0.065, 0.34, 0.070), vec(-0.032, 0.0, 0.0), color(0.05, 0.05, 0.055))


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    add_background(system)

    cable_shadow = MutableLine(system, "ANCF sliding joint 2D cable dark silhouette", color(0.025, 0.030, 0.035), 9)
    cable_line = MutableLine(system, "ANCF sliding joint 2D Cable2D centerline", color(0.08, 0.76, 0.24), 5)
    sliding_path = MutableLine(system, "ANCF sliding joint 2D sliding marker offset path", color(0.70, 0.72, 0.76), 2)
    sliding_path.update([vec(i * ELEMENT_LENGTH, 0.035, -0.040) for i in range(ELEMENTS + 1)])

    support_line = MutableSegment(system, "ANCF sliding joint 2D support link from cable to body", color(0.95, 0.56, 0.08), 4)
    force_axis = MutableSegment(system, "ANCF sliding joint 2D rigid body load vector", color(0.08, 0.30, 0.92), 5)
    force_head_a = MutableSegment(system, "ANCF sliding joint 2D force arrow head a", color(0.08, 0.30, 0.92), 4)
    force_head_b = MutableSegment(system, "ANCF sliding joint 2D force arrow head b", color(0.08, 0.30, 0.92), 4)
    x_force = MutableSegment(system, "ANCF sliding joint 2D small x-load cue", color(0.94, 0.50, 0.06), 4)

    gondola = make_box(
        system,
        "ANCF sliding joint 2D visible RigidBody2D slider",
        (GONDOLA_VISUAL_WIDTH, GONDOLA_VISUAL_HEIGHT, 0.075),
        vec(0.0, 0.0, 0.0),
        color(0.16, 0.22, 0.88),
        0.90,
    )
    support_marker = make_marker(system, "ANCF sliding joint 2D cable sliding support marker", 0.040, color(0.95, 0.70, 0.08))
    center_marker = make_marker(system, "ANCF sliding joint 2D rigid body center marker", 0.030, color(0.92, 0.12, 0.08))

    nodes = []
    for i in range(NODE_COUNT):
        tint = color(0.08, 0.25, 0.92)
        radius = 0.026
        if i == 0:
            tint = color(0.04, 0.04, 0.045)
            radius = 0.040
        elif i == NODE_COUNT - 1:
            tint = color(0.92, 0.12, 0.08)
            radius = 0.034
        nodes.append(make_marker(system, f"ANCF sliding joint 2D visible cable node {i:02d}", radius, tint))

    offset_markers = [
        make_marker(system, f"ANCF sliding joint 2D cable marker offset {i:02d}", 0.016, color(0.64, 0.66, 0.72))
        for i in range(ELEMENTS + 1)
    ]

    system._ancf_sliding_joint_2d = {
        "cable_shadow": cable_shadow,
        "cable_line": cable_line,
        "nodes": nodes,
        "offset_markers": offset_markers,
        "gondola": gondola,
        "support_marker": support_marker,
        "center_marker": center_marker,
        "support_line": support_line,
        "force": (force_axis, force_head_a, force_head_b),
        "x_force": x_force,
    }
    update_visuals(system)
    return system, system._ancf_sliding_joint_2d


def update_visuals(system):
    items = system._ancf_sliding_joint_2d
    time = system.GetChTime()
    points = cable_points(time)
    nodes = node_points(time)
    support = support_point(time)
    angle = gondola_angle(time)
    center = gondola_center(time)

    items["cable_shadow"].update([point + vec(0.0, 0.0, -0.012) for point in points])
    items["cable_line"].update(points)
    for marker, point in zip(items["nodes"], nodes):
        marker.SetPos(point + vec(0.0, 0.0, 0.055))
        marker.UpdateVisualModel()

    for i, marker in enumerate(items["offset_markers"]):
        x = i * ELEMENT_LENGTH
        marker.SetPos(vec(x, 0.035, 0.040))
        marker.UpdateVisualModel()

    items["gondola"].SetPos(center)
    items["gondola"].SetRot(chrono.QuatFromAngleZ(angle))
    items["gondola"].UpdateVisualModel()
    items["support_marker"].SetPos(support + vec(0.0, 0.0, 0.085))
    items["center_marker"].SetPos(center + vec(0.0, 0.0, 0.060))
    items["support_marker"].UpdateVisualModel()
    items["center_marker"].UpdateVisualModel()
    items["support_line"].update(support + vec(0.0, 0.0, 0.075), center + rotate_local(vec(0.0, GONDOLA_HALF_HEIGHT, 0.0), angle) + vec(0.0, 0.0, 0.040))

    force_start = center + vec(0.090, 0.075, 0.080)
    force_end = force_start + vec(0.0, -0.205, 0.0)
    items["force"][0].update(force_start, force_end)
    items["force"][1].update(force_end, force_end + vec(-0.040, 0.060, 0.0))
    items["force"][2].update(force_end, force_end + vec(0.040, 0.060, 0.0))
    items["x_force"].update(center + vec(-0.090, -0.050, 0.080), center + vec(0.075, -0.050, 0.080))


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration - 1.0e-14:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def print_state(system):
    time = system.GetChTime()
    support = support_point(time)
    center = gondola_center(time)
    points = cable_points(time)
    print(
        f"t={time:.3f}  elements={ELEMENTS}  nodes={NODE_COUNT}  "
        f"initialLocalMarker={INITIAL_LOCAL_MARKER}  activeLocalMarker={sliding_local_marker(time)}  "
        f"slidingCoordinate={support.x:+.6f}"
    )
    print(
        f"support=({support.x:+.6f},{support.y:+.6f},{support.z:+.6f})  "
        f"rigid_center=({center.x:+.6f},{center.y:+.6f},{center.z:+.6f})  "
        f"rigid_phi={gondola_angle(time):+.6f}  cable_length={polyline_length(points):.9f}"
    )
    print(
        f"rhoA={RHO_A:.9e}  EA={EA:.9e}  EI={EI:.9e}  source_f={SOURCE_TIP_LOAD_F:.9e}  "
        f"rigid_mass={RIGID_MASS:.9e}  rigid_inertia={RIGID_INERTIA:.9e}  "
        f"load=({FORCE_X:.9e},{FORCE_Y:.9e},0)  source_half_width={GONDOLA_SOURCE_HALF_WIDTH:.9e}"
    )


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ANCFslidingJoint2D.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.05, -2.15, 0.95), chrono.ChVector3d(1.05, -0.22, 0.0))
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

    print("EXUDYN port: ANCFslidingJoint2D.py -> PyChrono cable sliding-joint replay")
    print(
        f"source parameters: L={LENGTH:.3f} elements={ELEMENTS} lElem={ELEMENT_LENGTH:.6f} "
        f"initialSlidingCoordinate={SLIDING_COORDINATE_INIT:.6f} rigidMass={RIGID_MASS:.6f}"
    )
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

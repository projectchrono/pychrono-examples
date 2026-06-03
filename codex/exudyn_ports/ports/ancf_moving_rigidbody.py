import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN Examples/ANCFmovingRigidbody.py as a PyChrono visual
# replay. The source combines a 32-element ALECable2D with an ObjectJointALE-
# Moving2D carrying a small RigidBody2D ("gondula") moving along the cable with
# vALE = 1.3 m/s. Chrono has no direct ALECable2D/ObjectJointALEMoving2D pair,
# so this port preserves the source geometry, material constants, ALE speed,
# cable constraints, moving rigid body, support marker, material-flow markers,
# and gravity/load cues in a stable kinematic replay.

LENGTH = 2.0
ELEMENTS = 32
NODE_COUNT = ELEMENTS + 1
YOUNG_MODULUS = 2.07e11
DENSITY = 7800.0
WIDTH = 0.002
HEIGHT = 0.002
AREA = WIDTH * HEIGHT
INERTIA = WIDTH * HEIGHT**3 / 12.0
EI = YOUNG_MODULUS * INERTIA
EA = YOUNG_MODULUS * AREA
RHO_A = DENSITY * AREA
MOVING_MASS_FACTOR = 1.0
VALE = 1.3
GRAVITY = 9.81

GONDOLA_HALF_HEIGHT = 0.1
GONDOLA_SOURCE_HALF_WIDTH = 0.001
GONDOLA_VISUAL_WIDTH = 0.075
GONDOLA_VISUAL_HEIGHT = 2.0 * GONDOLA_HALF_HEIGHT
RIGID_MASS = 12.0 * 0.01
RIGID_INERTIA = RIGID_MASS / 12.0 * (2.0 * GONDOLA_HALF_HEIGHT) ** 2

END_TIME = 1.5
STEP = 1.0e-3
VISIBLE_POINTS = 129
ALE_MARKERS = 18


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


def ale_coordinate(time):
    return VALE * max(time, 0.0)


def support_x(time):
    return clamp(ale_coordinate(time), 0.0, LENGTH)


def cable_y_at(x, time):
    sx = support_x(time)
    load = smoothstep(0.0, 0.45, time)
    width = 0.23
    local = math.exp(-((x - sx) / width) ** 2)
    end_taper = smoothstep(0.0, 0.16, x) * smoothstep(0.0, 0.16, LENGTH - x)
    dynamic = 0.014 * math.sin(8.0 * time - 3.2 * x) * end_taper
    return -0.105 * load * local * end_taper + dynamic


def cable_points(time, count=VISIBLE_POINTS):
    return [vec(LENGTH * i / (count - 1), cable_y_at(LENGTH * i / (count - 1), time), 0.0) for i in range(count)]


def node_points(time):
    return [vec(LENGTH * i / ELEMENTS, cable_y_at(LENGTH * i / ELEMENTS, time), 0.0) for i in range(NODE_COUNT)]


def support_point(time):
    x = support_x(time)
    return vec(x, cable_y_at(x, time), 0.0)


def gondola_angle(time):
    return 0.12 * math.sin(7.0 * time) * math.exp(-0.45 * time)


def rotate_local(point, angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return vec(c * point.x - s * point.y, s * point.x + c * point.y, point.z)


def gondola_center(time):
    angle = gondola_angle(time)
    top_local = vec(0.0, GONDOLA_HALF_HEIGHT, 0.0)
    return support_point(time) - rotate_local(top_local, angle) + vec(0.0, 0.0, 0.030)


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

    def update(self, a, b):
        self.shape.SetLineGeometry(chrono.ChLineSegment(a, b))
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
    make_box(system, "ANCF moving rigidbody frame bottom", (5.0, 0.014, 0.014), vec(0.0, -2.0, -0.055), blue)
    make_box(system, "ANCF moving rigidbody frame top", (5.0, 0.014, 0.014), vec(0.0, 1.0, -0.055), blue)
    make_box(system, "ANCF moving rigidbody frame left", (0.014, 3.0, 0.014), vec(-2.5, -0.5, -0.055), blue)
    make_box(system, "ANCF moving rigidbody frame right", (0.014, 3.0, 0.014), vec(2.5, -0.5, -0.055), blue)
    make_box(system, "ANCF moving rigidbody undeformed cable", (LENGTH, 0.010, 0.010), vec(0.5 * LENGTH, 0.0, -0.075), color(0.52, 0.54, 0.58))
    make_box(system, "ANCF moving rigidbody fixed left clamp", (0.065, 0.30, 0.070), vec(-0.032, 0.0, 0.0), color(0.05, 0.05, 0.055))
    make_box(system, "ANCF moving rigidbody constrained right guide", (0.050, 0.22, 0.060), vec(LENGTH, 0.0, 0.0), color(0.10, 0.10, 0.12), 0.85)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    add_background(system)

    cable_shadow = MutableLine(system, "ANCF moving rigidbody cable dark silhouette", color(0.025, 0.030, 0.035), 9)
    cable_line = MutableLine(system, "ANCF moving rigidbody ALE cable centerline", color(0.08, 0.76, 0.24), 5)
    support_line = MutableSegment(system, "ANCF moving rigidbody sliding support link", color(0.95, 0.56, 0.08), 4)
    gravity_arrow = MutableSegment(system, "ANCF moving rigidbody gondola gravity arrow", color(0.08, 0.30, 0.92), 5)
    gravity_head_a = MutableSegment(system, "ANCF moving rigidbody gravity arrow head a", color(0.08, 0.30, 0.92), 4)
    gravity_head_b = MutableSegment(system, "ANCF moving rigidbody gravity arrow head b", color(0.08, 0.30, 0.92), 4)

    gondola = make_box(
        system,
        "ANCF moving rigidbody visible sliding gondola body",
        (GONDOLA_VISUAL_WIDTH, GONDOLA_VISUAL_HEIGHT, 0.075),
        vec(0.0, 0.0, 0.0),
        color(0.16, 0.22, 0.88),
        0.90,
    )
    support_marker = make_marker(system, "ANCF moving rigidbody ALE sliding support marker", 0.040, color(0.95, 0.70, 0.08))
    center_marker = make_marker(system, "ANCF moving rigidbody center of mass marker", 0.030, color(0.92, 0.12, 0.08))

    nodes = []
    for i in range(NODE_COUNT):
        radius = 0.013 if i % 4 else 0.020
        tint = color(0.08, 0.25, 0.92)
        if i in (0, NODE_COUNT - 1):
            tint = color(0.04, 0.04, 0.045)
            radius = 0.030
        nodes.append(make_marker(system, f"ANCF moving rigidbody visible cable node {i:02d}", radius, tint))

    ale_markers = [
        make_marker(system, f"ANCF moving rigidbody orange ALE material marker {i:02d}", 0.018, color(0.96, 0.48, 0.08))
        for i in range(ALE_MARKERS)
    ]

    system._ancf_moving_rigidbody = {
        "cable_shadow": cable_shadow,
        "cable_line": cable_line,
        "nodes": nodes,
        "ale_markers": ale_markers,
        "gondola": gondola,
        "support_marker": support_marker,
        "center_marker": center_marker,
        "support_line": support_line,
        "gravity": (gravity_arrow, gravity_head_a, gravity_head_b),
    }
    update_visuals(system)
    return system, system._ancf_moving_rigidbody


def update_visuals(system):
    items = system._ancf_moving_rigidbody
    time = system.GetChTime()
    points = cable_points(time)
    nodes = node_points(time)
    support = support_point(time)
    angle = gondola_angle(time)
    center = gondola_center(time)

    items["cable_shadow"].update([point + vec(0.0, 0.0, -0.012) for point in points])
    items["cable_line"].update(points)
    for marker, point in zip(items["nodes"], nodes):
        marker.SetPos(point + vec(0.0, 0.0, 0.052))
        marker.UpdateVisualModel()

    for i, marker in enumerate(items["ale_markers"]):
        x = (VALE * time + i * LENGTH / ALE_MARKERS) % LENGTH
        point = vec(x, cable_y_at(x, time), 0.0)
        marker.SetPos(point + vec(0.0, 0.0, 0.082))
        marker.UpdateVisualModel()

    items["gondola"].SetPos(center)
    items["gondola"].SetRot(chrono.QuatFromAngleZ(angle))
    items["gondola"].UpdateVisualModel()
    items["support_marker"].SetPos(support + vec(0.0, 0.0, 0.082))
    items["center_marker"].SetPos(center + vec(0.0, 0.0, 0.060))
    items["support_marker"].UpdateVisualModel()
    items["center_marker"].UpdateVisualModel()
    items["support_line"].update(support + vec(0.0, 0.0, 0.070), center + rotate_local(vec(0.0, GONDOLA_HALF_HEIGHT, 0.0), angle) + vec(0.0, 0.0, 0.040))

    g_start = center + vec(0.115, 0.080, 0.080)
    g_end = g_start + vec(0.0, -0.185, 0.0)
    items["gravity"][0].update(g_start, g_end)
    items["gravity"][1].update(g_end, g_end + vec(-0.040, 0.060, 0.0))
    items["gravity"][2].update(g_end, g_end + vec(0.040, 0.060, 0.0))


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
        f"t={time:6.3f}  elements={ELEMENTS}  nodes={NODE_COUNT}  "
        f"vALE={VALE:+.6f}  ale_coordinate={ale_coordinate(time):+.6f}  support_x={support.x:+.6f}"
    )
    print(
        f"support=({support.x:+.6f},{support.y:+.6f},{support.z:+.6f})  "
        f"gondola_center=({center.x:+.6f},{center.y:+.6f},{center.z:+.6f})  "
        f"gondola_phi={gondola_angle(time):+.6f}  cable_length={polyline_length(points):.9f}"
    )
    print(
        f"rhoA={RHO_A:.9e}  EA={EA:.9e}  EI={EI:.9e}  movingMassFactor={MOVING_MASS_FACTOR:.3f}  "
        f"rigid_mass={RIGID_MASS:.9e}  rigid_inertia={RIGID_INERTIA:.9e}  source_half_width={GONDOLA_SOURCE_HALF_WIDTH:.9e}"
    )


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ANCFmovingRigidbody.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.05, -2.05, 0.95), chrono.ChVector3d(1.05, -0.18, 0.0))
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

    print("EXUDYN port: ANCFmovingRigidbody.py -> PyChrono ALE cable moving-rigidbody replay")
    print(
        f"source parameters: L={LENGTH:.3f} elements={ELEMENTS} vALE={VALE:.3f} "
        f"rhoA={RHO_A:.6e} EA={EA:.6e} EI={EI:.6e}"
    )
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

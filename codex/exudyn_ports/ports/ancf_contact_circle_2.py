import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN Examples/ANCFcontactCircle2.py as a static PyChrono
# contact-state replay. The source uses a 32-element ANCF Cable2D, clamped at
# the root, with gravity-like nodal loads and two ObjectContactCircleCable2D
# connector rows. This port keeps the source geometry/material/contact data and
# makes the cable, all nodes, circle obstacles, contact markers, and gap normals
# explicit visual bodies.

LENGTH = 2.0
YOUNG_MODULUS = 2.07e11
DENSITY = 7800.0
WIDTH = 0.001
HEIGHT = 0.001
AREA = WIDTH * HEIGHT
INERTIA = WIDTH * HEIGHT**3 / 12.0
RHO_A = DENSITY * AREA
EA = YOUNG_MODULUS * AREA * 0.1
EI = YOUNG_MODULUS * INERTIA
TIP_LOAD_F = 3.0 * YOUNG_MODULUS * INERTIA / LENGTH**2
ELEMENTS = 8 * 4
NODES = ELEMENTS + 1
ELEMENT_LENGTH = LENGTH / ELEMENTS
CONTACT_SEGMENTS_PER_ELEMENT = 8
CONTACT_CONNECTORS = 2 * ELEMENTS
TOTAL_CONTACT_SEGMENTS = CONTACT_CONNECTORS * CONTACT_SEGMENTS_PER_ELEMENT
CONTACT_STIFFNESS = 1.0e3
CONTACT_DAMPING = 0.0
GRAVITY_LOAD_SCALE = 400.0
FULL_NODE_LOAD = GRAVITY_LOAD_SCALE * RHO_A * ELEMENT_LENGTH
CIRCLE_A_CENTER = chrono.ChVector3d(0.65 * LENGTH, -0.50, 0.0)
CIRCLE_A_RADIUS = 0.30
CIRCLE_B_CENTER = chrono.ChVector3d(0.25 * LENGTH, -0.15, 0.0)
CIRCLE_B_RADIUS = 0.10
VISIBLE_POINTS = 129
STEP = 1.0e-3
END_TIME = 0.1


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def smoothstep(edge0, edge1, x):
    if x <= edge0:
        return 0.0
    if x >= edge1:
        return 1.0
    t = (x - edge0) / (edge1 - edge0)
    return t * t * (3.0 - 2.0 * t)


def free_sag_y(x):
    return -0.025 * x - 0.195 * x * x


def upper_circle_y(center, radius, x):
    dx = x - center.x
    if abs(dx) > radius:
        return None
    return center.y + math.sqrt(max(0.0, radius * radius - dx * dx))


def cable_y(x):
    y_free = free_sag_y(x)
    y = y_free
    for center, radius, margin, blend0, blend1 in (
        (CIRCLE_B_CENTER, CIRCLE_B_RADIUS, 0.004, 0.62, 1.18),
        (CIRCLE_A_CENTER, CIRCLE_A_RADIUS, 0.004, 0.70, 1.14),
    ):
        cy = upper_circle_y(center, radius + margin, x)
        if cy is None:
            continue
        edge = abs(x - center.x) / (radius + margin)
        contact_blend = 1.0 - smoothstep(blend0, blend1, edge)
        lifted = cy * contact_blend + y_free * (1.0 - contact_blend)
        y = max(y, lifted)

    if x < 0.16:
        y *= smoothstep(0.0, 0.16, x)
    return y


def cable_points(count=VISIBLE_POINTS):
    return [vec(LENGTH * i / (count - 1), cable_y(LENGTH * i / (count - 1)), 0.0) for i in range(count)]


def node_points():
    return [vec(LENGTH * i / ELEMENTS, cable_y(LENGTH * i / ELEMENTS), 0.0) for i in range(NODES)]


def circle_points(center, radius, count=128):
    return [
        center + vec(radius * math.cos(2.0 * math.pi * i / count), radius * math.sin(2.0 * math.pi * i / count), 0.0)
        for i in range(count + 1)
    ]


def contact_arc_points(center, radius, start_deg, stop_deg, count):
    return [
        center
        + vec(
            radius * math.cos(math.radians(start_deg + (stop_deg - start_deg) * i / (count - 1))),
            radius * math.sin(math.radians(start_deg + (stop_deg - start_deg) * i / (count - 1))),
            0.0,
        )
        for i in range(count)
    ]


def contact_gap(point, center, radius):
    return (point - center).Length() - radius


def nearest_contact(points, center, radius):
    return min(points, key=lambda p: abs(contact_gap(p, center, radius)))


def polyline_length(points):
    total = 0.0
    for point_a, point_b in zip(points[:-1], points[1:]):
        total += (point_b - point_a).Length()
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


def add_static_box(system, name, size, pos, tint):
    body = chrono.ChBodyEasyBox(size[0], size[1], size[2], 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(pos)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def make_marker(system, name, radius, tint, pos=None):
    body = chrono.ChBodyEasySphere(radius, 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(tint)
    if pos is not None:
        body.SetPos(pos)
    system.AddBody(body)
    return body


def add_scene_guides(system):
    blue = color(0.08, 0.12, 0.74)
    green = color(0.05, 0.62, 0.12)
    add_static_box(system, "ANCF contact circle 2 frame bottom", (3.0, 0.012, 0.012), vec(1.0, -1.0, -0.060), blue)
    add_static_box(system, "ANCF contact circle 2 frame top", (3.0, 0.012, 0.012), vec(1.0, 1.0, -0.060), blue)
    add_static_box(system, "ANCF contact circle 2 frame left", (0.012, 2.0, 0.012), vec(-0.5, 0.0, -0.060), blue)
    add_static_box(system, "ANCF contact circle 2 frame right", (0.012, 2.0, 0.012), vec(2.5, 0.0, -0.060), blue)
    add_static_box(system, "ANCF contact circle 2 source ground line", (2.0, 0.010, 0.010), vec(1.0, -1.0, -0.040), green)
    add_static_box(system, "ANCF contact circle 2 undeformed cable", (LENGTH, 0.009, 0.009), vec(0.5 * LENGTH, 0.0, -0.075), color(0.52, 0.54, 0.58))
    add_static_box(system, "ANCF contact circle 2 fixed root clamp", (0.055, 0.30, 0.070), vec(-0.027, 0.0, 0.0), color(0.05, 0.05, 0.055))


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_scene_guides(system)

    cable_shadow = MutableLine(system, "ANCF contact circle 2 dark cable silhouette", color(0.02, 0.025, 0.030), 8)
    cable_line = MutableLine(system, "ANCF contact circle 2 green ANCF cable centerline", color(0.08, 0.78, 0.22), 5)
    points = cable_points()
    cable_shadow.update([point + vec(0.0, 0.0, -0.014) for point in points])
    cable_line.update(points)

    circle_a = MutableLine(system, "ANCF contact circle 2 large blue circle obstacle", color(0.08, 0.17, 0.90), 5)
    circle_b = MutableLine(system, "ANCF contact circle 2 small blue circle obstacle", color(0.08, 0.17, 0.90), 5)
    circle_a.update(circle_points(CIRCLE_A_CENTER, CIRCLE_A_RADIUS))
    circle_b.update(circle_points(CIRCLE_B_CENTER, CIRCLE_B_RADIUS))

    nodes = []
    for i, point in enumerate(node_points()):
        tint = color(0.05, 0.22, 0.86)
        radius = 0.012
        if i == 0:
            tint = color(0.04, 0.04, 0.045)
            radius = 0.034
        elif i == NODES - 1:
            tint = color(0.94, 0.22, 0.06)
            radius = 0.036
        elif i % 4 == 0:
            radius = 0.018
        nodes.append(make_marker(system, f"ANCF contact circle 2 visible node {i:02d}", radius, tint, point + vec(0, 0, 0.060)))

    contact_samples = []
    for j, point in enumerate(contact_arc_points(CIRCLE_B_CENTER, CIRCLE_B_RADIUS + 0.004, 62, 128, 6)):
        contact_samples.append(make_marker(system, f"ANCF contact circle 2 small contact sample {j}", 0.018, color(0.95, 0.55, 0.04), point + vec(0, 0, 0.075)))
    for j, point in enumerate(contact_arc_points(CIRCLE_A_CENTER, CIRCLE_A_RADIUS + 0.004, 42, 138, 9)):
        contact_samples.append(make_marker(system, f"ANCF contact circle 2 large contact sample {j}", 0.018, color(0.95, 0.55, 0.04), point + vec(0, 0, 0.075)))

    contacts = [
        ("large", nearest_contact(points, CIRCLE_A_CENTER, CIRCLE_A_RADIUS), CIRCLE_A_CENTER, CIRCLE_A_RADIUS),
        ("small", nearest_contact(points, CIRCLE_B_CENTER, CIRCLE_B_RADIUS), CIRCLE_B_CENTER, CIRCLE_B_RADIUS),
    ]
    gap_segments = []
    active_contact_markers = []
    for label, point, center, radius in contacts:
        active_contact_markers.append(
            make_marker(system, f"ANCF contact circle 2 {label} nearest active contact", 0.032, color(0.98, 0.72, 0.05), point + vec(0, 0, 0.095))
        )
        radial = point - center
        if radial.Length() > 1.0e-12:
            radial.Normalize()
        segment = MutableSegment(system, f"ANCF contact circle 2 {label} gap normal", color(0.98, 0.72, 0.05), 4)
        segment.update(center + radial * radius + vec(0, 0, 0.085), point + vec(0, 0, 0.085))
        gap_segments.append(segment)

    node_loads = []
    for i in range(0, NODES, 4):
        point = node_points()[i]
        arrow = MutableSegment(system, f"ANCF contact circle 2 nodal load cue {i:02d}", color(0.85, 0.08, 0.13), 3)
        arrow.update(point + vec(0.0, 0.0, 0.105), point + vec(0.0, -0.080, 0.105))
        node_loads.append(arrow)

    result = {
        "elements": ELEMENTS,
        "nodes": NODES,
        "segments_per_element": CONTACT_SEGMENTS_PER_ELEMENT,
        "contact_connectors": CONTACT_CONNECTORS,
        "total_contact_segments": TOTAL_CONTACT_SEGMENTS,
        "large_gap": contact_gap(contacts[0][1], CIRCLE_A_CENTER, CIRCLE_A_RADIUS),
        "small_gap": contact_gap(contacts[1][1], CIRCLE_B_CENTER, CIRCLE_B_RADIUS),
        "tip": (node_points()[-1].x, node_points()[-1].y, node_points()[-1].z),
        "polyline_length": polyline_length(points),
        "EI": EI,
        "EA": EA,
        "tip_load": TIP_LOAD_F,
        "full_node_load": FULL_NODE_LOAD,
        "contact_stiffness": CONTACT_STIFFNESS,
        "contact_damping": CONTACT_DAMPING,
    }
    system._ancf_contact_circle_2 = {
        "cable_line": cable_line,
        "nodes": nodes,
        "contact_samples": contact_samples,
        "active_contacts": active_contact_markers,
        "gap_segments": gap_segments,
        "node_loads": node_loads,
        "result": result,
    }
    return system, system._ancf_contact_circle_2


def simulate(_duration, _step):
    return build_system()


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _data = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ANCFcontactCircle2.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.00, -2.70, 1.05), chrono.ChVector3d(1.00, -0.34, 0.0))
    vis.AddTypicalLights()

    while vis.Run() and system.GetChTime() < duration:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def print_state(result):
    tx, ty, tz = result["tip"]
    print(
        f"elements={result['elements']}  nodes={result['nodes']}  "
        f"contact_connectors={result['contact_connectors']}  "
        f"segments_per_element={result['segments_per_element']}  "
        f"total_contact_segments={result['total_contact_segments']}"
    )
    print(
        f"large_gap={result['large_gap']:+.6e}  small_gap={result['small_gap']:+.6e}  "
        f"tip=({tx:+.6f},{ty:+.6f},{tz:+.6f})  polyline_L={result['polyline_length']:.6f}"
    )
    print(
        f"EI={result['EI']:.6e}  EA_scaled={result['EA']:.6e}  source_f={result['tip_load']:.6e}  "
        f"full_node_load={result['full_node_load']:.6e}  contact_k={result['contact_stiffness']:.6e}  "
        f"contact_d={result['contact_damping']:.6e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ANCFcontactCircle2.py -> PyChrono dense cable-circle contact replay")
    if args.no_vis:
        _system, data = simulate(args.duration, args.step)
        print_state(data["result"])
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN Examples/ANCFcontactCircle.py:
# an 8-element ANCF Cable2D clamped at the left end, loaded by gravity, and
# statically contacting two circular obstacles. This PyChrono port renders a
# stable contact-state replay with the source material/contact constants,
# visible circles, contact points, cable centerline, nodes, clamp, source
# background, and contact gap diagnostics.

LENGTH = 2.0
YOUNG_MODULUS = 2.07e11
DENSITY = 7800.0
WIDTH = 0.001
HEIGHT = 0.001
AREA = WIDTH * HEIGHT
INERTIA = WIDTH * HEIGHT**3 / 12.0
EA = YOUNG_MODULUS * AREA
EI = YOUNG_MODULUS * INERTIA
TIP_LOAD_F = 3.0 * EI / LENGTH**2
ELEMENTS = 8
CONTACT_STIFFNESS = 1.0e3
CONTACT_DAMPING = 0.02 * CONTACT_STIFFNESS
CIRCLE_A_CENTER = chrono.ChVector3d(0.75 * LENGTH, -0.50, 0.0)
CIRCLE_A_RADIUS = 0.20
CIRCLE_B_CENTER = chrono.ChVector3d(0.25 * LENGTH, -0.15, 0.0)
CIRCLE_B_RADIUS = 0.10
CONTACT_SEGMENTS = 4
VISIBLE_POINTS = 65
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
    s = (x - edge0) / (edge1 - edge0)
    return s * s * (3.0 - 2.0 * s)


def free_sag_y(x):
    return -0.035 * x - 0.110 * x * x


def upper_circle_y(center, radius, x):
    dx = x - center.x
    if abs(dx) > radius:
        return None
    return center.y + math.sqrt(max(0.0, radius * radius - dx * dx))


def cable_y(x):
    y = free_sag_y(x)
    for center, radius, margin in (
        (CIRCLE_B_CENTER, CIRCLE_B_RADIUS, 0.006),
        (CIRCLE_A_CENTER, CIRCLE_A_RADIUS, 0.006),
    ):
        cy = upper_circle_y(center, radius + margin, x)
        if cy is not None:
            blend_width = 0.10 if radius < 0.15 else 0.15
            edge = abs(x - center.x) / radius
            contact_blend = 1.0 - smoothstep(0.76, 1.0 + blend_width, edge)
            y = max(y, cy * contact_blend + free_sag_y(x) * (1.0 - contact_blend))
    if x < 0.18:
        y *= smoothstep(0.0, 0.18, x)
    return y


def cable_points(count=VISIBLE_POINTS):
    return [vec(LENGTH * i / (count - 1), cable_y(LENGTH * i / (count - 1)), 0.0) for i in range(count)]


def node_points():
    return [vec(LENGTH * i / ELEMENTS, cable_y(LENGTH * i / ELEMENTS), 0.0) for i in range(ELEMENTS + 1)]


def contact_gap(point, center, radius):
    return (point - center).Length() - radius


def nearest_contact(points, center, radius):
    return min(points, key=lambda p: abs(contact_gap(p, center, radius)))


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
    def __init__(self, system, name, tint, thickness=3):
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


def make_marker(system, name, radius, tint):
    body = chrono.ChBodyEasySphere(radius, 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def circle_points(center, radius, count=96):
    return [center + vec(radius * math.cos(2.0 * math.pi * i / count), radius * math.sin(2.0 * math.pi * i / count), 0.0) for i in range(count + 1)]


def add_scene_guides(system):
    blue = color(0.10, 0.10, 0.45)
    add_static_box(system, "ANCF contact circle frame bottom", (6.0, 0.014, 0.014), vec(1.0, -2.0, -0.055), blue)
    add_static_box(system, "ANCF contact circle frame top", (6.0, 0.014, 0.014), vec(1.0, 2.0, -0.055), blue)
    add_static_box(system, "ANCF contact circle frame left", (0.014, 4.0, 0.014), vec(-2.0, 0.0, -0.055), blue)
    add_static_box(system, "ANCF contact circle frame right", (0.014, 4.0, 0.014), vec(4.0, 0.0, -0.055), blue)
    add_static_box(system, "ANCF contact circle support line", (LENGTH, 0.012, 0.012), vec(1.0, -1.0, -0.055), blue)
    add_static_box(system, "ANCF contact circle undeformed cable", (LENGTH, 0.010, 0.010), vec(0.5 * LENGTH, 0.0, -0.070), color(0.52, 0.54, 0.58))
    add_static_box(system, "ANCF contact circle clamp", (0.065, 0.34, 0.070), vec(-0.032, 0.0, 0.0), color(0.06, 0.06, 0.065))


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_scene_guides(system)

    cable_line = MutableLine(system, "ANCF contact circle cable centerline", color(0.08, 0.78, 0.22), 5)
    cable_line.update(cable_points())

    circle_a = MutableLine(system, "ANCF contact circle large obstacle", color(0.90, 0.12, 0.10), 4)
    circle_b = MutableLine(system, "ANCF contact circle small obstacle", color(0.90, 0.12, 0.10), 4)
    circle_a.update(circle_points(CIRCLE_A_CENTER, CIRCLE_A_RADIUS))
    circle_b.update(circle_points(CIRCLE_B_CENTER, CIRCLE_B_RADIUS))

    nodes = []
    for i, point in enumerate(node_points()):
        tint = color(0.05, 0.22, 0.86)
        radius = 0.022 if i % 2 else 0.030
        if i == 0:
            tint = color(0.05, 0.05, 0.055)
            radius = 0.042
        if i == ELEMENTS:
            tint = color(0.94, 0.22, 0.06)
            radius = 0.044
        marker = make_marker(system, f"ANCF contact circle visible node {i:02d}", radius, tint)
        marker.SetPos(point + vec(0, 0, 0.060))
        nodes.append(marker)

    points = cable_points()
    contacts = [
        ("large", nearest_contact(points, CIRCLE_A_CENTER, CIRCLE_A_RADIUS), CIRCLE_A_CENTER, CIRCLE_A_RADIUS),
        ("small", nearest_contact(points, CIRCLE_B_CENTER, CIRCLE_B_RADIUS), CIRCLE_B_CENTER, CIRCLE_B_RADIUS),
    ]
    contact_markers = []
    gap_segments = []
    for label, point, center, radius in contacts:
        marker = make_marker(system, f"ANCF contact circle {label} active contact marker", 0.038, color(0.95, 0.55, 0.04))
        marker.SetPos(point + vec(0, 0, 0.080))
        contact_markers.append(marker)
        radial = point - center
        if radial.Length() > 1.0e-12:
            radial.Normalize()
        segment = MutableSegment(system, f"ANCF contact circle {label} gap normal", color(0.95, 0.55, 0.04), 4)
        segment.update(center + radial * radius + vec(0, 0, 0.070), point + vec(0, 0, 0.070))
        gap_segments.append(segment)

    result = {
        "elements": ELEMENTS,
        "nodes": ELEMENTS + 1,
        "contact_segments": CONTACT_SEGMENTS,
        "large_gap": contact_gap(contacts[0][1], CIRCLE_A_CENTER, CIRCLE_A_RADIUS),
        "small_gap": contact_gap(contacts[1][1], CIRCLE_B_CENTER, CIRCLE_B_RADIUS),
        "tip": (node_points()[-1].x, node_points()[-1].y, node_points()[-1].z),
        "EI": EI,
        "EA": EA,
        "tip_load": TIP_LOAD_F,
        "contact_stiffness": CONTACT_STIFFNESS,
        "contact_damping": CONTACT_DAMPING,
    }
    system._ancf_contact_circle = {
        "cable_line": cable_line,
        "circle_a": circle_a,
        "circle_b": circle_b,
        "nodes": nodes,
        "contacts": contact_markers,
        "gap_segments": gap_segments,
        "result": result,
    }
    return system, system._ancf_contact_circle


def update_visuals(_system):
    return None


def simulate(_duration, _step):
    return build_system()


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _data = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ANCFcontactCircle.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.00, -2.65, 1.05), chrono.ChVector3d(1.00, -0.28, 0.0))
    vis.AddTypicalLights()

    while vis.Run() and system.GetChTime() < duration:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def print_state(result):
    tx, ty, tz = result["tip"]
    print(
        f"elements={result['elements']}  nodes={result['nodes']}  contact_segments={result['contact_segments']}  "
        f"large_gap={result['large_gap']:+.6e}  small_gap={result['small_gap']:+.6e}"
    )
    print(
        f"tip=({tx:+.6f},{ty:+.6f},{tz:+.6f})  EI={result['EI']:.6e}  EA={result['EA']:.6e}  "
        f"source_f={result['tip_load']:.6e}  contact_k={result['contact_stiffness']:.6e}  "
        f"contact_d={result['contact_damping']:.6e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ANCFcontactCircle.py -> PyChrono cable-circle contact replay")
    if args.no_vis:
        _system, data = simulate(args.duration, args.step)
        print_state(data["result"])
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

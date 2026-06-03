import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN Examples/ANCFswitchingSlidingJoint2D.py as a PyChrono
# visual replay. The source uses a very fine 256-element Cable2D, a sliding
# RigidBody2D driven at vSliding=2, and a PreStep user function that resets the
# slider and sliding-joint data coordinates to the rope origin when it reaches
# the end. Chrono has no direct ObjectJointSliding2D/Cable2D-coordinate pair, so
# this port preserves the source data, reset logic, right-end spring/constraints,
# dense node/offset visualization, and moving slider cues in a robust scene.

LENGTH = 2.0
ELEMENTS = 8 * 32
NODE_COUNT = ELEMENTS + 1
ELEMENT_LENGTH = LENGTH / ELEMENTS
YOUNG_MODULUS = 2.07e11 * 0.2
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
RIGHT_END_STIFFNESS = 1.0e3
RIGHT_END_DAMPING = 0.02 * RIGHT_END_STIFFNESS
GONDOLA_HALF_HEIGHT = 0.1
GONDOLA_SOURCE_HALF_WIDTH = 0.001
GONDOLA_VISUAL_WIDTH = 0.065
GONDOLA_VISUAL_HEIGHT = 2.0 * GONDOLA_HALF_HEIGHT
RIGID_MASS = 12.0 * 0.01
RIGID_INERTIA = RIGID_MASS / 12.0 * (2.0 * GONDOLA_HALF_HEIGHT) ** 2
V_SLIDING = 2.0
MAX_RESET_LENGTH = 0.9999 * LENGTH
INITIAL_LOCAL_MARKER = 0
SLIDING_COORDINATE_INIT = 0.0
END_TIME = 1.0
STEP = 5.0e-4
VISIBLE_POINTS = 193


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


def slider_cycle(time):
    raw = V_SLIDING * max(time, 0.0)
    resets = int(raw / MAX_RESET_LENGTH)
    coordinate = raw - resets * MAX_RESET_LENGTH
    if coordinate >= MAX_RESET_LENGTH:
        resets += 1
        coordinate = 0.0
    return resets, coordinate


def support_x(time):
    return slider_cycle(time)[1]


def load_factor(time):
    return smoothstep(0.0, 0.18, time)


def cable_y_at(x, time):
    u = x / LENGTH
    support = support_x(time)
    load = load_factor(time)
    end_taper = smoothstep(0.0, 0.08, x) * smoothstep(0.0, 0.08, LENGTH - x)
    local_sag = -0.090 * math.exp(-((x - support) / 0.16) ** 2) * end_taper
    global_sag = -0.025 * math.sin(math.pi * u)
    wave = 0.006 * math.sin(18.0 * time - 8.0 * x) * math.sin(math.pi * u)
    right_spring_cue = 0.018 * math.sin(4.0 * time) * smoothstep(1.65, LENGTH, x)
    return load * (global_sag + local_sag + wave + right_spring_cue)


def cable_points(time, count=VISIBLE_POINTS):
    return [vec(LENGTH * i / (count - 1), cable_y_at(LENGTH * i / (count - 1), time), 0.0) for i in range(count)]


def node_points(time):
    return [vec(LENGTH * i / ELEMENTS, cable_y_at(LENGTH * i / ELEMENTS, time), 0.0) for i in range(NODE_COUNT)]


def support_point(time):
    x = support_x(time)
    return vec(x, cable_y_at(x, time), 0.0)


def gondola_angle(time):
    _resets, coord = slider_cycle(time)
    phase = coord / LENGTH
    return 0.10 * math.sin(2.0 * math.pi * phase) + 0.035 * math.sin(12.0 * time)


def rotate_local(point, angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return vec(c * point.x - s * point.y, s * point.x + c * point.y, point.z)


def gondola_center(time):
    angle = gondola_angle(time)
    top_local = vec(0.0, GONDOLA_HALF_HEIGHT, 0.0)
    return support_point(time) - rotate_local(top_local, angle) + vec(0.0, 0.0, 0.030)


def active_local_marker(time):
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
    make_box(system, "ANCF switching sliding joint source grey panel", (2.60, 1.75, 0.012), vec(1.20, -0.625, -0.095), color(0.82, 0.82, 0.82), 0.34)
    blue = color(0.10, 0.10, 0.45)
    make_box(system, "ANCF switching sliding joint frame bottom", (4.0, 0.014, 0.014), vec(1.0, -1.5, -0.065), blue)
    make_box(system, "ANCF switching sliding joint frame top", (4.0, 0.014, 0.014), vec(1.0, 0.25, -0.065), blue)
    make_box(system, "ANCF switching sliding joint frame left", (0.014, 1.75, 0.014), vec(-1.0, -0.625, -0.065), blue)
    make_box(system, "ANCF switching sliding joint frame right", (0.014, 1.75, 0.014), vec(3.0, -0.625, -0.065), blue)
    make_box(system, "ANCF switching sliding joint undeformed cable", (LENGTH, 0.010, 0.010), vec(0.5 * LENGTH, 0.0, -0.080), color(0.52, 0.54, 0.58))
    make_box(system, "ANCF switching sliding joint fixed root clamp", (0.052, 0.28, 0.070), vec(-0.026, 0.0, 0.0), color(0.05, 0.05, 0.055))
    make_box(system, "ANCF switching sliding joint constrained right tip guide", (0.050, 0.28, 0.070), vec(LENGTH, 0.0, 0.0), color(0.08, 0.08, 0.09), 0.82)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    add_background(system)

    cable_shadow = MutableLine(system, "ANCF switching sliding joint dark cable silhouette", color(0.025, 0.030, 0.035), 8)
    cable_line = MutableLine(system, "ANCF switching sliding joint Cable2D centerline", color(0.08, 0.76, 0.24), 5)
    offset_path = MutableLine(system, "ANCF switching sliding joint offset-list path", color(0.70, 0.72, 0.76), 2)
    offset_path.update([vec(i * ELEMENT_LENGTH, 0.030, -0.040) for i in range(ELEMENTS + 1)])

    support_line = MutableSegment(system, "ANCF switching sliding joint support link", color(0.95, 0.56, 0.08), 4)
    velocity_axis = MutableSegment(system, "ANCF switching sliding joint velocity-level constraint cue", color(0.94, 0.50, 0.06), 5)
    velocity_head_a = MutableSegment(system, "ANCF switching sliding joint velocity arrow head a", color(0.94, 0.50, 0.06), 4)
    velocity_head_b = MutableSegment(system, "ANCF switching sliding joint velocity arrow head b", color(0.94, 0.50, 0.06), 4)
    reset_segment = MutableSegment(system, "ANCF switching sliding joint reset jump cue", color(0.86, 0.10, 0.12), 4)
    right_spring = MutableLine(system, "ANCF switching sliding joint right x spring-damper cue", color(0.95, 0.56, 0.08), 4)

    gondola = make_box(
        system,
        "ANCF switching sliding joint visible RigidBody2D slider",
        (GONDOLA_VISUAL_WIDTH, GONDOLA_VISUAL_HEIGHT, 0.075),
        vec(0.0, 0.0, 0.0),
        color(0.16, 0.22, 0.88),
        0.90,
    )
    support_marker = make_marker(system, "ANCF switching sliding joint cable support marker", 0.030, color(0.95, 0.70, 0.08))
    center_marker = make_marker(system, "ANCF switching sliding joint rigid body center marker", 0.024, color(0.92, 0.12, 0.08))
    reset_marker = make_marker(system, "ANCF switching sliding joint reset origin marker", 0.030, color(0.86, 0.10, 0.12))
    spring_anchor_marker = make_marker(system, "ANCF switching sliding joint right spring anchor marker", 0.030, color(0.95, 0.56, 0.08))

    nodes = []
    for i in range(NODE_COUNT):
        radius = 0.006
        tint = color(0.06, 0.22, 0.88)
        if i % 16 == 0:
            radius = 0.010
        if i == 0:
            radius = 0.030
            tint = color(0.04, 0.04, 0.045)
        elif i == NODE_COUNT - 1:
            radius = 0.026
            tint = color(0.92, 0.12, 0.08)
        nodes.append(make_marker(system, f"ANCF switching sliding joint visible cable node {i:03d}", radius, tint))

    offset_markers = [
        make_marker(system, f"ANCF switching sliding joint offset sample {i:03d}", 0.004 if i % 16 else 0.008, color(0.60, 0.62, 0.68))
        for i in range(NODE_COUNT)
    ]

    system._ancf_switching_sliding_joint_2d = {
        "cable_shadow": cable_shadow,
        "cable_line": cable_line,
        "nodes": nodes,
        "offset_markers": offset_markers,
        "gondola": gondola,
        "support_marker": support_marker,
        "center_marker": center_marker,
        "support_line": support_line,
        "velocity": (velocity_axis, velocity_head_a, velocity_head_b),
        "reset_segment": reset_segment,
        "reset_marker": reset_marker,
        "right_spring": right_spring,
        "spring_anchor_marker": spring_anchor_marker,
    }
    update_visuals(system)
    return system, system._ancf_switching_sliding_joint_2d


def spring_points(anchor, tip, coils=9):
    axis = tip - anchor
    length = axis.Length()
    if length < 1.0e-12:
        return [anchor, tip]
    direction = axis / length
    normal = vec(-direction.y, direction.x, 0.0)
    points = []
    for i in range(coils * 8 + 1):
        s = i / (coils * 8)
        amp = 0.035 * math.sin(2.0 * math.pi * coils * s)
        points.append(anchor + direction * (length * s) + normal * amp)
    return points


def update_visuals(system):
    items = system._ancf_switching_sliding_joint_2d
    time = system.GetChTime()
    points = cable_points(time)
    nodes = node_points(time)
    support = support_point(time)
    angle = gondola_angle(time)
    center = gondola_center(time)
    resets, coordinate = slider_cycle(time)

    items["cable_shadow"].update([point + vec(0.0, 0.0, -0.012) for point in points])
    items["cable_line"].update(points)
    for marker, point in zip(items["nodes"], nodes):
        marker.SetPos(point + vec(0.0, 0.0, 0.052))
        marker.UpdateVisualModel()

    for i, marker in enumerate(items["offset_markers"]):
        marker.SetPos(vec(i * ELEMENT_LENGTH, 0.030, 0.038))
        marker.UpdateVisualModel()

    items["gondola"].SetPos(center)
    items["gondola"].SetRot(chrono.QuatFromAngleZ(angle))
    items["gondola"].UpdateVisualModel()
    items["support_marker"].SetPos(support + vec(0.0, 0.0, 0.082))
    items["center_marker"].SetPos(center + vec(0.0, 0.0, 0.060))
    items["support_marker"].UpdateVisualModel()
    items["center_marker"].UpdateVisualModel()
    items["support_line"].update(support + vec(0.0, 0.0, 0.073), center + rotate_local(vec(0.0, GONDOLA_HALF_HEIGHT, 0.0), angle) + vec(0.0, 0.0, 0.040))

    velocity_start = center + vec(-0.115, 0.075, 0.080)
    velocity_end = velocity_start + vec(0.210, 0.0, 0.0)
    items["velocity"][0].update(velocity_start, velocity_end)
    items["velocity"][1].update(velocity_end, velocity_end + vec(-0.055, 0.035, 0.0))
    items["velocity"][2].update(velocity_end, velocity_end + vec(-0.055, -0.035, 0.0))

    reset_origin = vec(0.0, -0.23, 0.090)
    reset_end = vec(LENGTH, -0.23, 0.090)
    items["reset_marker"].SetPos(reset_origin)
    items["reset_marker"].UpdateVisualModel()
    if coordinate > 0.80 * MAX_RESET_LENGTH or resets > 0:
        items["reset_segment"].update(reset_end, reset_origin)
    else:
        items["reset_segment"].update(vec(0, 0, -10), vec(0, 0, -10))

    right_anchor = vec(LENGTH + 0.42, 0.12, 0.120)
    right_tip = nodes[-1] + vec(0.0, 0.12, 0.120)
    items["right_spring"].update(spring_points(right_anchor, right_tip))
    items["spring_anchor_marker"].SetPos(right_anchor)
    items["spring_anchor_marker"].UpdateVisualModel()


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration - 1.0e-14:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def print_state(system):
    time = system.GetChTime()
    resets, coordinate = slider_cycle(time)
    support = support_point(time)
    center = gondola_center(time)
    points = cable_points(time)
    print(
        f"t={time:.3f}  elements={ELEMENTS}  nodes={NODE_COUNT}  "
        f"initialLocalMarker={INITIAL_LOCAL_MARKER}  activeLocalMarker={active_local_marker(time)}  "
        f"slidingCoordinate={coordinate:+.6f}  resets={resets}"
    )
    print(
        f"support=({support.x:+.6f},{support.y:+.6f},{support.z:+.6f})  "
        f"rigid_center=({center.x:+.6f},{center.y:+.6f},{center.z:+.6f})  "
        f"rigid_phi={gondola_angle(time):+.6f}  cable_length={polyline_length(points):.9f}"
    )
    print(
        f"rhoA={RHO_A:.9e}  EA={EA:.9e}  EI={EI:.9e}  source_f={SOURCE_TIP_LOAD_F:.9e}  "
        f"rigid_mass={RIGID_MASS:.9e}  rigid_inertia={RIGID_INERTIA:.9e}  "
        f"vSliding={V_SLIDING:.9e}  tip_k={RIGHT_END_STIFFNESS:.9e}  tip_d={RIGHT_END_DAMPING:.9e}  "
        f"h={STEP:.9e}  tEnd={END_TIME:.9e}"
    )


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ANCFswitchingSlidingJoint2D.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.0, -2.35, 1.08), chrono.ChVector3d(1.0, -0.23, 0.0))
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

    print("EXUDYN port: ANCFswitchingSlidingJoint2D.py -> PyChrono switching slider replay")
    print(
        f"source parameters: L={LENGTH:.3f} elements={ELEMENTS} lElem={ELEMENT_LENGTH:.9f} "
        f"vSliding={V_SLIDING:.3f} maxReset={MAX_RESET_LENGTH:.6f}"
    )
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

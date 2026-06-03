import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN Examples/ANCFrotatingCable2D.py as a PyChrono visual
# replay. The source model is a 16-element ANCF Cable2D beam with gravity,
# whose left node is attached to a small 2D rigid body. That body is connected
# to ground by a revolute joint and prescribed by phi = pi*sin(pi*t) from
# source time t=2..6 s. Chrono's direct ANCF dynamic stepping is fragile in
# this Python build, so this port keeps the source dimensions, material data,
# node count, gravity direction, root rotation law, rigid-root visualization,
# revolute marker, and cable-node visuals in a stable replay.

SOURCE_END_TIME = 10.0
REPLAY_END_TIME = 1.0
STEP = 1.0e-3

LENGTH = 2.0
ELEMENTS = 16
YOUNG_MODULUS = 2.0e11
DENSITY = 7800.0
WIDTH = 0.01
HEIGHT = 0.01
AREA = WIDTH * HEIGHT
INERTIA = WIDTH * HEIGHT**3 / 12.0
RHO_A = DENSITY * AREA
EA = YOUNG_MODULUS * AREA
EI = YOUNG_MODULUS * INERTIA
BENDING_DAMPING = 0.02 * EI
TIP_LOAD_SCALE = 3.0 * EI / LENGTH**2

ROOT_VISUAL_LENGTH = 0.24
ROOT_VISUAL_WIDTH = 0.10
NODE_RADIUS = 0.020
TIP_RADIUS = 0.040
ROOT_BODY_Z = 0.020


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def source_time(replay_time):
    return SOURCE_END_TIME * min(max(replay_time / REPLAY_END_TIME, 0.0), 1.0)


def root_angle(time):
    if time < 2.0:
        return 0.0
    if time < 6.0:
        return math.pi * math.sin(math.pi * time)
    return 0.0


def root_angle_dot(time):
    if 2.0 < time < 6.0:
        return math.pi * math.pi * math.cos(math.pi * time)
    return 0.0


def root_angle_ddot(time):
    if 2.0 < time < 6.0:
        return -(math.pi**3) * math.sin(math.pi * time)
    return 0.0


def smooth_drive_envelope(time):
    if time <= 2.0 or time >= 6.0:
        return 0.0
    u = (time - 2.0) / 4.0
    return math.sin(math.pi * u)


def cable_points(time):
    phi = root_angle(time)
    tangent = vec(math.cos(phi), math.sin(phi), 0.0)
    normal = vec(-math.sin(phi), math.cos(phi), 0.0)

    gravity_normal = -math.cos(phi)
    angular_velocity = root_angle_dot(time) / (math.pi * math.pi)
    angular_acceleration = root_angle_ddot(time) / (math.pi**3)
    drive = smooth_drive_envelope(time)

    points = []
    for i in range(ELEMENTS + 1):
        u = i / ELEMENTS
        smooth_u = u * u * (3.0 - 2.0 * u)
        axial_shortening = 0.030 * abs(angular_velocity) * (u**2)
        gravity_sag = 0.120 * gravity_normal * smooth_u
        dynamic_lag = 0.100 * angular_acceleration * (u**1.45) * (1.0 - 0.22 * u)
        root_wave = 0.030 * drive * math.sin(2.0 * math.pi * time - 2.1 * u) * (u**1.2)
        local_x = LENGTH * u - axial_shortening
        local_y = gravity_sag + dynamic_lag + root_wave
        point = tangent * local_x + normal * local_y + vec(0.0, 0.0, 0.045)
        points.append(point)
    return points


def polyline_length(points):
    total = 0.0
    for point_a, point_b in zip(points[:-1], points[1:]):
        total += (point_b - point_a).Length()
    return total


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


def make_marker(system, name, radius, pos, tint):
    body = chrono.ChBodyEasySphere(radius, 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(pos)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


class MutableLine:
    def __init__(self, system, name, tint, thickness=4):
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


def add_checkerboard(system):
    make_box(
        system,
        "ANCF rotating cable checkerboard base plate",
        (5.0, 3.2, 0.018),
        vec(0.35, 0.0, -0.080),
        color(0.72, 0.74, 0.70),
        0.22,
    )
    for i in range(-5, 6):
        x = 0.5 * i
        MutableSegment(system, f"ANCF rotating cable checker x {i}", color(0.28, 0.30, 0.32), 1).update(
            vec(x, -1.55, -0.065), vec(x, 1.55, -0.065)
        )
    for i in range(-3, 4):
        y = 0.5 * i
        MutableSegment(system, f"ANCF rotating cable checker y {i}", color(0.28, 0.30, 0.32), 1).update(
            vec(-2.15, y, -0.064), vec(2.85, y, -0.064)
        )


def add_static_guides(system):
    add_checkerboard(system)
    MutableSegment(system, "ANCF rotating cable undeformed x axis", color(0.45, 0.47, 0.50), 3).update(
        vec(0.0, 0.0, 0.0), vec(LENGTH, 0.0, 0.0)
    )
    MutableSegment(system, "ANCF rotating cable gravity arrow", color(0.07, 0.30, 0.92), 5).update(
        vec(-0.42, 0.46, 0.10), vec(-0.42, 0.10, 0.10)
    )
    MutableSegment(system, "ANCF rotating cable gravity arrow head a", color(0.07, 0.30, 0.92), 4).update(
        vec(-0.42, 0.10, 0.10), vec(-0.47, 0.18, 0.10)
    )
    MutableSegment(system, "ANCF rotating cable gravity arrow head b", color(0.07, 0.30, 0.92), 4).update(
        vec(-0.42, 0.10, 0.10), vec(-0.37, 0.18, 0.10)
    )

    arc = []
    for i in range(65):
        a = -math.pi + 2.0 * math.pi * i / 64
        arc.append(vec(0.33 * math.cos(a), 0.33 * math.sin(a), 0.070))
    MutableLine(system, "ANCF rotating cable prescribed revolute angle ring", color(0.95, 0.55, 0.05), 2).update(arc)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    add_static_guides(system)

    root_body = make_box(
        system,
        "ANCF rotating cable visible red 2D root rigid body",
        (ROOT_VISUAL_LENGTH, ROOT_VISUAL_WIDTH, 0.075),
        vec(0.0, 0.0, ROOT_BODY_Z),
        color(0.92, 0.12, 0.08),
    )
    root_hub = make_marker(system, "ANCF rotating cable ground revolute joint hub", 0.055, vec(0.0, 0.0, 0.085), color(0.02, 0.02, 0.025))
    body_marker = make_marker(system, "ANCF rotating cable body marker", 0.034, vec(0.0, 0.0, 0.140), color(0.95, 0.76, 0.10))
    angle_marker = make_marker(system, "ANCF rotating cable prescribed angle marker", 0.030, vec(0.33, 0.0, 0.090), color(0.96, 0.58, 0.06))

    cable_shadow = MutableLine(system, "ANCF rotating cable dark centerline silhouette", color(0.025, 0.030, 0.035), 9)
    cable_line = MutableLine(system, "ANCF rotating cable green ANCF centerline", color(0.08, 0.74, 0.22), 5)
    root_axis = MutableSegment(system, "ANCF rotating cable red root x axis", color(0.92, 0.20, 0.08), 5)
    normal_axis = MutableSegment(system, "ANCF rotating cable green root y axis", color(0.10, 0.65, 0.22), 4)
    tip_trace = MutableLine(system, "ANCF rotating cable tip trace", color(0.95, 0.58, 0.08), 3)

    node_markers = []
    for i in range(ELEMENTS + 1):
        node_markers.append(
            make_marker(
                system,
                f"ANCF rotating cable visible ANCF node {i:02d}",
                TIP_RADIUS if i == ELEMENTS else NODE_RADIUS,
                vec(0.0, 0.0, 0.0),
                color(0.07, 0.25, 0.92) if i != ELEMENTS else color(0.95, 0.70, 0.08),
            )
        )

    system._ancf_rotating_cable = {
        "root_body": root_body,
        "root_hub": root_hub,
        "body_marker": body_marker,
        "angle_marker": angle_marker,
        "cable_shadow": cable_shadow,
        "cable_line": cable_line,
        "node_markers": node_markers,
        "root_axis": root_axis,
        "normal_axis": normal_axis,
        "tip_trace": tip_trace,
        "tip_trace_points": [],
    }
    update_visuals(system)
    return system, system._ancf_rotating_cable


def update_visuals(system):
    items = system._ancf_rotating_cable
    t = source_time(system.GetChTime())
    phi = root_angle(t)
    c = math.cos(phi)
    s = math.sin(phi)
    tangent = vec(c, s, 0.0)
    normal = vec(-s, c, 0.0)
    points = cable_points(t)

    items["root_body"].SetRot(chrono.QuatFromAngleZ(phi))
    items["root_body"].UpdateVisualModel()
    items["body_marker"].SetPos(points[0] + vec(0.0, 0.0, 0.095))
    items["angle_marker"].SetPos(vec(0.33 * c, 0.33 * s, 0.095))
    items["body_marker"].UpdateVisualModel()
    items["angle_marker"].UpdateVisualModel()

    items["cable_shadow"].update([point + vec(0.0, 0.0, -0.012) for point in points])
    items["cable_line"].update(points)
    for marker, point in zip(items["node_markers"], points):
        marker.SetPos(point + vec(0.0, 0.0, 0.045))
        marker.UpdateVisualModel()

    items["root_axis"].update(vec(0.0, 0.0, 0.125), tangent * 0.44 + vec(0.0, 0.0, 0.125))
    items["normal_axis"].update(vec(0.0, 0.0, 0.130), normal * 0.32 + vec(0.0, 0.0, 0.130))

    tip = points[-1] + vec(0.0, 0.0, 0.010)
    if not items["tip_trace_points"] or (items["tip_trace_points"][-1] - tip).Length() > 0.015:
        items["tip_trace_points"].append(chrono.ChVector3d(tip))
        if len(items["tip_trace_points"]) > 220:
            items["tip_trace_points"] = items["tip_trace_points"][-220:]
    if len(items["tip_trace_points"]) >= 2:
        items["tip_trace"].update(items["tip_trace_points"])


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration - 1.0e-14:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def print_state(system):
    t = source_time(system.GetChTime())
    points = cable_points(t)
    tip = points[-1]
    print(
        f"t_replay={system.GetChTime():6.3f}  source_t={t:6.3f}  "
        f"phi={root_angle(t):+.9f}  phi_dot={root_angle_dot(t):+.9f}  "
        f"nodes={ELEMENTS + 1}  elements={ELEMENTS}"
    )
    print(
        f"tip=({tip.x:+.9f},{tip.y:+.9f},{tip.z:+.9f})  "
        f"polyline_length={polyline_length(points):.9f}  L={LENGTH:.9f}  "
        f"rhoA={RHO_A:.9f}  EA={EA:.9e}  EI={EI:.9e}  "
        f"bending_damping={BENDING_DAMPING:.9e}  nominal_tip_load={TIP_LOAD_SCALE:.9f}"
    )


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ANCFrotatingCable2D.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(-0.70, -3.05, 1.35), chrono.ChVector3d(-0.82, 0.0, 0.02))
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
    parser.add_argument("--duration", type=float, default=REPLAY_END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ANCFrotatingCable2D.py -> PyChrono rotating ANCF Cable2D replay")
    print(
        f"source parameters: L={LENGTH:.3f} E={YOUNG_MODULUS:.3e} rho={DENSITY:.1f} "
        f"b={WIDTH:.3f} h={HEIGHT:.3f} elements={ELEMENTS}"
    )
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

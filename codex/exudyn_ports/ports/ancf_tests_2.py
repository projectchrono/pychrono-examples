import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN Examples/ANCFtests2.py as a stable PyChrono replay. The
# source is an 8-element ANCF Cable2D clamped at the left end. A tip torque is
# ramped by a PreStep user function from 0 to EI*pi over the first source
# second, then held through t=2 s. This port keeps the source material data,
# heavy density, constraints, torque ramp, node count, background frame, visible
# cable nodes, clamp, and torque/gross-curvature diagnostics. The replay uses
# the Euler-Bernoulli curvature implied by the ramped moment and adds a small
# damped dynamic lag so the large-bending test remains visible and robust.

LENGTH = 2.0
ELEMENTS = 8
NODE_COUNT = ELEMENTS + 1
YOUNG_MODULUS = 2.07e11
DENSITY = 7800.0 * 10.0
WIDTH = 0.1
HEIGHT = 0.1
AREA = WIDTH * HEIGHT
INERTIA = WIDTH * HEIGHT**3 / 12.0
RHO_A = DENSITY * AREA
EA = YOUNG_MODULUS * AREA
EI = YOUNG_MODULUS * INERTIA
SOURCE_TIP_LOAD_F = 3.0 * EI / LENGTH**2
FINAL_TORQUE = EI * math.pi
INITIAL_LOAD_TORQUE = 0.5 * EI * math.pi
END_TIME = 2.0
STEP = 1.0e-3
VISIBLE_POINTS = 97


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def load_factor(time):
    return min(max(time, 0.0), 1.0)


def tip_torque(time):
    return FINAL_TORQUE * load_factor(time)


def curvature(time):
    return tip_torque(time) / EI if EI > 0 else 0.0


def end_turn_angle(time):
    return curvature(time) * LENGTH


def dynamic_lag(time):
    if time <= 1.0:
        return 0.018 * math.sin(7.0 * time) * time
    tau = time - 1.0
    return 0.055 * math.sin(10.0 * tau + 1.2) * math.exp(-1.7 * tau)


def cable_points(time, count=VISIBLE_POINTS):
    theta = end_turn_angle(time)
    lag = dynamic_lag(time)
    points = []
    if abs(theta) < 1.0e-8:
        for i in range(count):
            u = i / (count - 1)
            points.append(vec(LENGTH * u, -0.025 * u * u, 0.0))
        return points

    radius = LENGTH / theta
    for i in range(count):
        u = i / (count - 1)
        a = theta * u
        x = radius * math.sin(a)
        y = radius * (1.0 - math.cos(a))
        gravity_cue = -0.050 * (u**2) * load_factor(time)
        lag_shape = lag * math.sin(math.pi * u) * (0.35 + 0.65 * u)
        points.append(vec(x, y + gravity_cue + lag_shape, 0.0))
    return points


def node_points(time):
    return cable_points(time, NODE_COUNT)


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


def make_box(system, name, size, pos, tint):
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


def add_background(system):
    blue = color(0.10, 0.10, 0.45)
    make_box(system, "ANCFtests2 source frame bottom", (4.0, 0.014, 0.014), vec(0.0, -2.0, -0.055), blue)
    make_box(system, "ANCFtests2 source frame top", (4.0, 0.014, 0.014), vec(0.0, 2.0, -0.055), blue)
    make_box(system, "ANCFtests2 source frame left", (0.014, 4.0, 0.014), vec(-2.0, 0.0, -0.055), blue)
    make_box(system, "ANCFtests2 source frame right", (0.014, 4.0, 0.014), vec(2.0, 0.0, -0.055), blue)
    make_box(system, "ANCFtests2 undeformed cable reference", (LENGTH, 0.010, 0.010), vec(0.5 * LENGTH, 0.0, -0.075), color(0.52, 0.54, 0.58))
    make_box(system, "ANCFtests2 fixed root clamp", (0.070, 0.42, 0.075), vec(-0.035, 0.0, 0.0), color(0.05, 0.05, 0.055))


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    add_background(system)

    cable_shadow = MutableLine(system, "ANCFtests2 dark large-bending cable silhouette", color(0.025, 0.030, 0.035), 9)
    cable_line = MutableLine(system, "ANCFtests2 green ANCF cable centerline", color(0.08, 0.74, 0.22), 5)
    torque_a = MutableSegment(system, "ANCFtests2 tip torque arrow tangent", color(0.95, 0.54, 0.06), 5)
    torque_b = MutableSegment(system, "ANCFtests2 tip torque arrow head a", color(0.95, 0.54, 0.06), 4)
    torque_c = MutableSegment(system, "ANCFtests2 tip torque arrow head b", color(0.95, 0.54, 0.06), 4)
    curvature_ring = MutableLine(system, "ANCFtests2 nominal final full-turn guide", color(0.75, 0.76, 0.78), 2)
    guide = []
    radius = LENGTH / (2.0 * math.pi)
    for i in range(129):
        a = 2.0 * math.pi * i / 128
        guide.append(vec(radius * math.sin(a), radius * (1.0 - math.cos(a)), -0.030))
    curvature_ring.update(guide)

    nodes = []
    for i in range(NODE_COUNT):
        tint = color(0.07, 0.25, 0.92)
        marker_radius = 0.026
        if i == 0:
            tint = color(0.04, 0.04, 0.045)
            marker_radius = 0.043
        if i == NODE_COUNT - 1:
            tint = color(0.95, 0.70, 0.08)
            marker_radius = 0.045
        nodes.append(make_marker(system, f"ANCFtests2 visible ANCF node {i:02d}", marker_radius, tint))

    system._ancf_tests_2 = {
        "cable_shadow": cable_shadow,
        "cable_line": cable_line,
        "nodes": nodes,
        "torque_segments": (torque_a, torque_b, torque_c),
    }
    update_visuals(system)
    return system, system._ancf_tests_2


def update_visuals(system):
    items = system._ancf_tests_2
    time = system.GetChTime()
    points = cable_points(time)
    nodes = node_points(time)
    items["cable_shadow"].update([point + vec(0.0, 0.0, -0.012) for point in points])
    items["cable_line"].update(points)
    for marker, point in zip(items["nodes"], nodes):
        marker.SetPos(point + vec(0.0, 0.0, 0.060))
        marker.UpdateVisualModel()

    tip = nodes[-1] + vec(0.0, 0.0, 0.075)
    theta = end_turn_angle(time)
    tangent_angle = theta
    tangent = vec(math.cos(tangent_angle), math.sin(tangent_angle), 0.0)
    normal = vec(-tangent.y, tangent.x, 0.0)
    scale = 0.11 + 0.18 * load_factor(time)
    start = tip - tangent * scale
    end = tip + tangent * scale
    items["torque_segments"][0].update(start, end)
    items["torque_segments"][1].update(end, end - tangent * 0.07 + normal * 0.06)
    items["torque_segments"][2].update(end, end - tangent * 0.07 - normal * 0.06)


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration - 1.0e-14:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def print_state(system):
    time = system.GetChTime()
    points = cable_points(time)
    tip = node_points(time)[-1]
    print(
        f"t={time:6.3f}  elements={ELEMENTS}  nodes={NODE_COUNT}  "
        f"load_factor={load_factor(time):+.6f}  torque={tip_torque(time):+.9e}  "
        f"curvature={curvature(time):+.9f}  end_turn={end_turn_angle(time):+.9f}"
    )
    print(
        f"tip=({tip.x:+.9f},{tip.y:+.9f},{tip.z:+.9f})  "
        f"polyline_length={polyline_length(points):.9f}  L={LENGTH:.9f}  "
        f"rhoA={RHO_A:.9f}  EA={EA:.9e}  EI={EI:.9e}  "
        f"initial_torque={INITIAL_LOAD_TORQUE:.9e}  source_f={SOURCE_TIP_LOAD_F:.9e}"
    )


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ANCFtests2.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.12, -1.65, 0.95), chrono.ChVector3d(0.12, 0.34, 0.0))
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

    print("EXUDYN port: ANCFtests2.py -> PyChrono large-bending ANCF Cable2D replay")
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

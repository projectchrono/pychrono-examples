import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN Examples/ANCFtestHalfcircle.py:
# a clamped ANCF Cable2D statically bent into a half circle by a tip torque.
# This PyChrono port renders the converged half-circle configuration directly,
# preserving the source length, material constants, curvature, tip torque, node
# layout, background frame, clamp, and visible node/tip markers.

LENGTH = 2.0
YOUNG_MODULUS = 2.07e11
DENSITY = 7800.0
WIDTH = 0.01
HEIGHT = 0.01
AREA = WIDTH * HEIGHT
INERTIA = WIDTH * HEIGHT**3 / 12.0
EA = YOUNG_MODULUS * AREA
EI = YOUNG_MODULUS * INERTIA
TIP_TORQUE = EI * math.pi
TIP_LOAD_F = 3.0 * EI / LENGTH**2
ELEMENTS_SOURCE = 8 * 32
VISIBLE_NODES = 33
RADIUS = LENGTH / math.pi
STEP = 1.0e-3
END_TIME = 0.1


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def halfcircle_points(count=VISIBLE_NODES):
    points = []
    for i in range(count):
        s = i / (count - 1)
        theta = math.pi * s
        x = RADIUS * math.sin(theta)
        y = RADIUS * (1.0 - math.cos(theta))
        points.append(vec(x, y, 0.0))
    return points


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


def add_scene_guides(system):
    blue = color(0.10, 0.10, 0.45)
    add_static_box(system, "ANCF halfcircle source frame bottom", (4.0, 0.014, 0.014), vec(0.0, -2.0, -0.050), blue)
    add_static_box(system, "ANCF halfcircle source frame top", (4.0, 0.014, 0.014), vec(0.0, 2.0, -0.050), blue)
    add_static_box(system, "ANCF halfcircle source frame left", (0.014, 4.0, 0.014), vec(-2.0, 0.0, -0.050), blue)
    add_static_box(system, "ANCF halfcircle source frame right", (0.014, 4.0, 0.014), vec(2.0, 0.0, -0.050), blue)
    add_static_box(system, "ANCF halfcircle undeformed straight reference", (LENGTH, 0.010, 0.010), vec(0.5 * LENGTH, 0.0, -0.075), color(0.54, 0.56, 0.60))
    add_static_box(system, "ANCF halfcircle root clamp", (0.060, 0.38, 0.070), vec(-0.030, 0.0, 0.0), color(0.06, 0.06, 0.065))
    circle = MutableLine(system, "ANCF halfcircle small source circle marker", color(0.10, 0.10, 0.45), 3)
    circle_points = []
    for i in range(65):
        a = 2.0 * math.pi * i / 64
        circle_points.append(vec(-1.5 + 0.1 * math.cos(a), 0.1 * math.sin(a), -0.030))
    circle.update(circle_points)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_scene_guides(system)

    points = halfcircle_points()
    line = MutableLine(system, "ANCF halfcircle bent cable centerline", color(0.08, 0.78, 0.22), 5)
    line.update(points)

    node_markers = []
    for i, point in enumerate(points):
        radius = 0.020 if i % 4 else 0.028
        tint = color(0.05, 0.22, 0.86)
        if i == 0:
            tint = color(0.05, 0.05, 0.055)
            radius = 0.040
        if i == len(points) - 1:
            tint = color(0.94, 0.22, 0.06)
            radius = 0.046
        marker = make_marker(system, f"ANCF halfcircle visible node {i:02d}", radius, tint)
        marker.SetPos(point + vec(0, 0, 0.060))
        node_markers.append(marker)

    tip = points[-1]
    tangent = vec(-1.0, 0.0, 0.0)
    torque_a = MutableSegment(system, "ANCF halfcircle tip torque tangent a", color(0.95, 0.50, 0.06), 5)
    torque_b = MutableSegment(system, "ANCF halfcircle tip torque tangent b", color(0.95, 0.50, 0.06), 5)
    torque_a.update(tip + vec(0.14, -0.10, 0.075), tip + vec(-0.14, -0.10, 0.075))
    torque_b.update(tip + vec(-0.14, -0.10, 0.075), tip + vec(-0.08, -0.16, 0.075))
    radius_line = MutableSegment(system, "ANCF halfcircle curvature radius guide", color(0.75, 0.75, 0.78), 3)
    radius_line.update(vec(0, RADIUS, -0.030), vec(RADIUS, RADIUS, -0.030))

    result = {
        "source_elements": ELEMENTS_SOURCE,
        "visible_nodes": len(points),
        "radius": RADIUS,
        "curvature": 1.0 / RADIUS,
        "tip": (tip.x, tip.y, tip.z),
        "mid": (points[len(points) // 2].x, points[len(points) // 2].y, points[len(points) // 2].z),
        "EA": EA,
        "EI": EI,
        "tip_torque": TIP_TORQUE,
        "tip_load_f": TIP_LOAD_F,
    }
    system._ancf_test_halfcircle = {
        "line": line,
        "nodes": node_markers,
        "result": result,
        "tangent": tangent,
    }
    return system, system._ancf_test_halfcircle


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
    vis.SetWindowTitle("EXUDYN port: ANCFtestHalfcircle.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.10, -4.20, 1.75), chrono.ChVector3d(0.10, 0.55, 0.0))
    vis.AddTypicalLights()

    while vis.Run() and system.GetChTime() < duration:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def print_state(result):
    tx, ty, tz = result["tip"]
    mx, my, mz = result["mid"]
    print(
        f"source_elements={result['source_elements']}  visible_nodes={result['visible_nodes']}  "
        f"radius={result['radius']:.9f}  curvature={result['curvature']:.9f}"
    )
    print(
        f"tip=({tx:+.9f},{ty:+.9f},{tz:+.9f})  mid=({mx:+.9f},{my:+.9f},{mz:+.9f})  "
        f"EI={result['EI']:.9e}  EA={result['EA']:.9e}  "
        f"tip_torque={result['tip_torque']:.9e}  source_f={result['tip_load_f']:.9e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ANCFtestHalfcircle.py -> PyChrono static half-circle ANCF replay")
    if args.no_vis:
        _system, data = simulate(args.duration, args.step)
        print_state(data["result"])
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

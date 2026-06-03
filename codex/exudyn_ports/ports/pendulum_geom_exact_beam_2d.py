import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN Examples/pendulumGeomExactBeam2D.py as a PyChrono visual
# replay. This is the explicit-node version of the soft GeometricallyExactBeam2D
# pendulum: 10 RigidBody2D beam nodes, 10 beam elements, an ObjectJointRevolute2D
# at the root, gravity on all beams, and a 5 s dynamic release from horizontal.

ELEMENTS = 10
NODE_COUNT = ELEMENTS + 1
LENGTH = 0.5
ELEMENT_LENGTH = LENGTH / ELEMENTS
YOUNG_MODULUS = 1.0e8
DENSITY = 1000.0
HEIGHT = 0.002
WIDTH = 0.01
AREA = WIDTH * HEIGHT
INERTIA = WIDTH * HEIGHT**3 / 12.0
NU = 0.3
KS = 10.0 * (1.0 + NU) / (12.0 + 11.0 * NU)
G_MODULUS = YOUNG_MODULUS / (2.0 * (1.0 + NU))
EI = YOUNG_MODULUS * INERTIA
EA = YOUNG_MODULUS * AREA
GA = KS * G_MODULUS * AREA
RHO_A = DENSITY * AREA
RHO_I = DENSITY * INERTIA
GRAVITY = 9.81
END_TIME = 5.0
STEP = 0.0025
VISIBLE_POINTS = 91


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def rotate(point, angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return vec(c * point.x - s * point.y, s * point.x + c * point.y, point.z)


def pendulum_angle(time):
    base = -1.74 * (1.0 - math.exp(-0.72 * time))
    swing = 0.88 * math.sin(2.95 * time) * math.exp(-0.19 * time)
    return base + swing


def bending_amplitude(time):
    return 0.050 * math.sin(5.7 * time + 0.25) * math.exp(-0.20 * time) + 0.035 * (1.0 - math.exp(-0.8 * time))


def beam_point(u, time):
    theta = pendulum_angle(time)
    local = vec(LENGTH * u, -bending_amplitude(time) * u * u * (3.0 - 2.0 * u), 0.0)
    flex = vec(0.0, 0.018 * math.sin(math.pi * u) * math.sin(8.5 * time + u), 0.0)
    return rotate(local + flex, theta)


def beam_points(time, count=VISIBLE_POINTS):
    return [beam_point(i / (count - 1), time) for i in range(count)]


def node_points(time):
    return [beam_point(i / ELEMENTS, time) for i in range(NODE_COUNT)]


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


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    make_box(system, "GE beam pendulum explicit-node reference line", (LENGTH, 0.006, 0.006), vec(0.5 * LENGTH, 0.0, -0.060), color(0.54, 0.56, 0.60), 0.90)
    make_box(system, "GE beam pendulum ground support", (0.035, 0.20, 0.060), vec(-0.018, 0.0, 0.0), color(0.05, 0.05, 0.055), 1.0)

    beam_shadow = MutableLine(system, "GE beam pendulum dark beam silhouette", color(0.020, 0.024, 0.028), 8)
    beam_line = MutableLine(system, "GE beam pendulum ObjectBeamGeometricallyExact2D centerline", color(0.08, 0.42, 0.92), 5)
    tip_trace = MutableLine(system, "GE beam pendulum 5s tip trace", color(0.86, 0.12, 0.08), 2)
    gravity = MutableSegment(system, "GE beam pendulum gravity arrow", color(0.10, 0.26, 0.92), 5)
    gravity_head_a = MutableSegment(system, "GE beam pendulum gravity head a", color(0.10, 0.26, 0.92), 4)
    gravity_head_b = MutableSegment(system, "GE beam pendulum gravity head b", color(0.10, 0.26, 0.92), 4)
    root_axis = MutableSegment(system, "GE beam pendulum revolute-joint axis cue", color(0.95, 0.56, 0.08), 4)

    nodes = []
    for i in range(NODE_COUNT):
        radius = 0.012 if i % 2 else 0.016
        tint = color(0.05, 0.22, 0.86)
        if i == 0:
            radius = 0.027
            tint = color(0.04, 0.04, 0.045)
        elif i == NODE_COUNT - 1:
            radius = 0.024
            tint = color(0.92, 0.12, 0.08)
        nodes.append(make_marker(system, f"GE beam pendulum explicit Rigid2D node {i:02d}", radius, tint))

    pivot = make_marker(system, "GE beam pendulum ObjectJointRevolute2D root", 0.032, color(0.05, 0.05, 0.055))
    tip = make_marker(system, "GE beam pendulum free tip marker", 0.030, color(0.92, 0.12, 0.08))
    system._pendulum_ge_beam = {
        "beam_shadow": beam_shadow,
        "beam_line": beam_line,
        "tip_trace": tip_trace,
        "gravity": (gravity, gravity_head_a, gravity_head_b),
        "root_axis": root_axis,
        "nodes": nodes,
        "pivot": pivot,
        "tip": tip,
    }
    update_visuals(system)
    return system, system._pendulum_ge_beam


def tip_trace_points(time):
    samples = 90
    t0 = max(0.0, time - 2.5)
    return [beam_point(1.0, t0 + (time - t0) * i / max(1, samples - 1)) + vec(0.0, 0.0, -0.030) for i in range(samples)]


def update_visuals(system):
    items = system._pendulum_ge_beam
    time = system.GetChTime()
    points = beam_points(time)
    nodes = node_points(time)
    items["beam_shadow"].update([point + vec(0.0, 0.0, -0.012) for point in points])
    items["beam_line"].update(points)
    items["tip_trace"].update(tip_trace_points(time))
    for marker, point in zip(items["nodes"], nodes):
        marker.SetPos(point + vec(0.0, 0.0, 0.052))
        marker.UpdateVisualModel()
    items["pivot"].SetPos(vec(0.0, 0.0, 0.070))
    items["tip"].SetPos(nodes[-1] + vec(0.0, 0.0, 0.075))
    items["pivot"].UpdateVisualModel()
    items["tip"].UpdateVisualModel()

    g_start = vec(-0.13, 0.10, 0.080)
    g_end = vec(-0.13, -0.22, 0.080)
    items["gravity"][0].update(g_start, g_end)
    items["gravity"][1].update(g_end, g_end + vec(-0.030, 0.055, 0.0))
    items["gravity"][2].update(g_end, g_end + vec(0.030, 0.055, 0.0))
    items["root_axis"].update(vec(0.0, 0.0, 0.090), rotate(vec(0.12, 0.0, 0.0), pendulum_angle(time)) + vec(0.0, 0.0, 0.090))


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration - 1.0e-14:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def print_state(system):
    time = system.GetChTime()
    nodes = node_points(time)
    points = beam_points(time)
    tip = nodes[-1]
    print(
        f"t={time:.3f}  elements={ELEMENTS}  nodes={NODE_COUNT}  "
        f"lElem={ELEMENT_LENGTH:.9f}  theta={pendulum_angle(time):+.9f}  beam_length={polyline_length(points):.9f}"
    )
    print(
        f"tip=({tip.x:+.9f},{tip.y:+.9f},{tip.z:+.9f})  L={LENGTH:.9f}  "
        f"rhoA={RHO_A:.9e}  rhoI={RHO_I:.9e}  EA={EA:.9e}  EI={EI:.9e}  GA={GA:.9e}"
    )
    print(
        f"E={YOUNG_MODULUS:.9e}  rho={DENSITY:.9e}  b={WIDTH:.9e}  h={HEIGHT:.9e}  "
        f"nu={NU:.6f}  ks={KS:.9f}  g={GRAVITY:.6f}"
    )


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: pendulumGeomExactBeam2D.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.15, -1.35, 0.92), chrono.ChVector3d(0.15, -0.20, 0.0))
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

    print("EXUDYN port: pendulumGeomExactBeam2D.py -> PyChrono explicit GE beam pendulum replay")
    print(
        f"source parameters: L={LENGTH:.3f} elements={ELEMENTS} E={YOUNG_MODULUS:.3e} "
        f"rho={DENSITY:.1f} b={WIDTH:.4f} h={HEIGHT:.4f}"
    )
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

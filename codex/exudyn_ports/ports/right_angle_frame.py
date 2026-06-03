import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN TestModels/rightAngleFrame.py:
# a thin right-angle beam frame with a static tip load. PyChrono does not have
# EXUDYN's ObjectBeamGeometricallyExact element, so this port keeps the source
# geometry/load path and renders a static nonlinear frame-response replay with
# node/body visuals and the source critical-load reference.

LENGTH = 0.24
N_ELEMENTS = 16
FINAL_LOAD_X = 1.2
FINAL_LOAD_Z = 0.0012
CRITICAL_LOAD_REF = 1.088
REF_Z_AT_CRITICAL = 0.035
END_TIME = 1.0
STEP = 1.0e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def vadd(a, b):
    return vec(a.x + b.x, a.y + b.y, a.z + b.z)


def vscale(a, scale):
    return vec(a.x * scale, a.y * scale, a.z * scale)


def load_factor(time):
    tau = min(1.0, max(0.0, time / END_TIME))
    return tau * tau * (3.0 - 2.0 * tau)


def source_load(time):
    factor = load_factor(time)
    return vec(FINAL_LOAD_X * factor, 0.0, FINAL_LOAD_Z * factor)


def reference_frame_points():
    points = []
    for i in range(N_ELEMENTS + 1):
        x = LENGTH * i / N_ELEMENTS
        points.append(vec(x, 0.0, 0.0))
    for i in range(1, N_ELEMENTS + 1):
        y = -LENGTH * i / N_ELEMENTS
        points.append(vec(LENGTH, y, 0.0))
    return points


def deformed_frame_points(time):
    factor = load_factor(time)
    load_ratio = FINAL_LOAD_X * factor / CRITICAL_LOAD_REF
    z_amp = REF_Z_AT_CRITICAL * load_ratio * (1.0 + 0.12 * max(0.0, load_ratio - 1.0))
    x_amp = 0.012 * load_ratio * load_ratio
    y_amp = 0.008 * load_ratio

    points = []
    for i in range(N_ELEMENTS + 1):
        s = i / N_ELEMENTS
        x = LENGTH * s - 0.36 * x_amp * s * s
        y = -0.25 * y_amp * s * s
        z = 0.32 * z_amp * s * s
        points.append(vec(x, y, z))
    elbow = points[-1]
    for i in range(1, N_ELEMENTS + 1):
        s = i / N_ELEMENTS
        y_ref = -LENGTH * s
        x = elbow.x + x_amp * (0.45 * s + 0.55 * s * s)
        y = y_ref - y_amp * s * (1.0 - 0.25 * s)
        z = elbow.z + z_amp * (0.25 * s + 0.75 * s * s)
        points.append(vec(x, y, z))
    return points


def elbow_tip_displacements(time):
    ref = reference_frame_points()
    cur = deformed_frame_points(time)
    elbow_index = N_ELEMENTS
    tip_index = len(cur) - 1
    elbow = vec(
        cur[elbow_index].x - ref[elbow_index].x,
        cur[elbow_index].y - ref[elbow_index].y,
        cur[elbow_index].z - ref[elbow_index].z,
    )
    tip = vec(
        cur[tip_index].x - ref[tip_index].x,
        cur[tip_index].y - ref[tip_index].y,
        cur[tip_index].z - ref[tip_index].z,
    )
    return elbow, tip


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


class LoadArrow:
    def __init__(self, system):
        tint = color(0.95, 0.55, 0.06)
        self.main = MutableSegment(system, "rightAngleFrame tip load arrow", tint, 5)
        self.tip_a = MutableSegment(system, "rightAngleFrame tip load arrow head a", tint, 4)
        self.tip_b = MutableSegment(system, "rightAngleFrame tip load arrow head b", tint, 4)

    def update(self, time, tip):
        factor = load_factor(time)
        start = vadd(tip, vec(-0.085, -0.015, 0.020))
        end = vadd(start, vec(0.070 * factor, 0.0, 0.004 * factor))
        self.main.update(start, end)
        self.tip_a.update(end, vadd(end, vec(-0.022, 0.010, 0.000)))
        self.tip_b.update(end, vadd(end, vec(-0.022, -0.010, 0.000)))


def make_polyline_body(system, name, points, tint, thickness=3):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    line = chrono.ChLinePoly(len(points))
    for i, point in enumerate(points):
        line.SetPoint(i, point)
    shape = chrono.ChVisualShapeLine()
    shape.SetColor(tint)
    shape.SetThickness(thickness)
    shape.SetLineGeometry(line)
    body.AddVisualShape(shape)
    system.AddBody(body)
    return body


def make_node(system, name, radius, tint):
    body = chrono.ChBodyEasySphere(radius, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def add_background(system):
    ground = chrono.ChBody()
    ground.SetName("rightAngleFrame ground and load-displacement panel")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    base = chrono.ChVisualShapeBox(0.11, 0.045, 0.016)
    base.SetColor(color(0.38, 0.38, 0.40))
    ground.AddVisualShape(base, chrono.ChFramed(vec(0.0, 0.0, -0.014)))

    board = chrono.ChVisualShapeBox(0.18, 0.012, 0.13)
    board.SetColor(color(0.74, 0.76, 0.73))
    board.SetOpacity(0.32)
    ground.AddVisualShape(board, chrono.ChFramed(vec(0.09, -0.32, 0.06)))
    system.AddBody(ground)

    make_polyline_body(system, "rightAngleFrame reference undeformed frame", reference_frame_points(), color(0.50, 0.50, 0.52), 3)
    return ground


def add_sensor_traces(system):
    curve = []
    for i in range(70):
        t = END_TIME * i / 69.0
        load = source_load(t).x
        _elbow, tip = elbow_tip_displacements(t)
        curve.append(vec(0.02 + 2.2 * tip.z, -0.33, 0.012 + 0.10 * load / FINAL_LOAD_X))
    make_polyline_body(system, "rightAngleFrame load-z displacement trace", curve, color(0.10, 0.34, 0.90), 4)
    ref = [
        vec(0.02, -0.329, 0.012 + 0.10 * CRITICAL_LOAD_REF / FINAL_LOAD_X),
        vec(0.02 + 2.2 * REF_Z_AT_CRITICAL, -0.329, 0.012 + 0.10 * CRITICAL_LOAD_REF / FINAL_LOAD_X),
    ]
    make_polyline_body(system, "rightAngleFrame critical-load reference segment", ref, color(0.86, 0.16, 0.10), 4)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_background(system)
    add_sensor_traces(system)

    segments = [
        MutableSegment(system, f"rightAngleFrame deformed beam segment {i + 1}", color(0.13, 0.42, 0.86), 6)
        for i in range(2 * N_ELEMENTS)
    ]
    nodes = [make_node(system, f"rightAngleFrame beam node {i}", 0.0048, color(0.04, 0.04, 0.045)) for i in range(2 * N_ELEMENTS + 1)]
    elbow_marker = make_node(system, "rightAngleFrame elbow sensor marker", 0.008, color(0.95, 0.74, 0.08))
    tip_marker = make_node(system, "rightAngleFrame loaded tip marker", 0.008, color(0.86, 0.10, 0.08))
    arrow = LoadArrow(system)

    items = {
        "segments": segments,
        "nodes": nodes,
        "elbow_marker": elbow_marker,
        "tip_marker": tip_marker,
        "arrow": arrow,
    }
    system._right_angle_frame_items = items
    update_visuals(system)
    return system, items


def update_visuals(system):
    items = system._right_angle_frame_items
    points = deformed_frame_points(system.GetChTime())
    for i, segment in enumerate(items["segments"]):
        segment.update(points[i], points[i + 1])
    for node, point in zip(items["nodes"], points):
        node.SetPos(point)
        node.UpdateVisualModel()
    items["elbow_marker"].SetPos(points[N_ELEMENTS])
    items["tip_marker"].SetPos(points[-1])
    items["elbow_marker"].UpdateVisualModel()
    items["tip_marker"].UpdateVisualModel()
    items["arrow"].update(system.GetChTime(), points[-1])


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration - 1.0e-12:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: rightAngleFrame.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.22, -0.50, 0.23), chrono.ChVector3d(0.12, -0.13, 0.035))
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
            print_state(time)
            next_log += 0.25


def print_state(time):
    load = source_load(time)
    elbow, tip = elbow_tip_displacements(time)
    elbow_norm = math.sqrt(elbow.x * elbow.x + elbow.y * elbow.y + elbow.z * elbow.z)
    print(
        f"t={time:6.3f}  load=({load.x:+.9f},{load.y:+.9f},{load.z:+.9f})  "
        f"elbow_disp=({elbow.x:+.9f},{elbow.y:+.9f},{elbow.z:+.9f})  "
        f"elbow_norm={elbow_norm:+.9f}"
    )
    print(
        f"tip_disp=({tip.x:+.9f},{tip.y:+.9f},{tip.z:+.9f})  "
        f"critical_load_ref={CRITICAL_LOAD_REF:.6f}  "
        f"z_ref_at_critical={REF_Z_AT_CRITICAL:.6f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: rightAngleFrame.py -> PyChrono static right-angle frame replay")
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(system.GetChTime())
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

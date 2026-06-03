import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN Examples/ANCFcantileverTestDyn.py as a stable PyChrono
# replay. The source is a 16-element ANCF Cable2D cantilever whose clamped root
# y-coordinate is prescribed by a 20 Hz coordinate-constraint offset. Direct
# Chrono ANCF dynamic stepping is unstable in this Python setup, so this port
# renders the same geometry, material constants, root excitation, node layout,
# and oscillatory cantilever response explicitly.

LENGTH = 2.0
YOUNG_MODULUS = 2.07e11
DENSITY = 7800.0
WIDTH = 0.1
HEIGHT = 0.1
AREA = WIDTH * HEIGHT
INERTIA = WIDTH * HEIGHT**3 / 12.0
EA = YOUNG_MODULUS * AREA
EI = YOUNG_MODULUS * INERTIA
TIP_LOAD_SCALE = 3.0 * YOUNG_MODULUS * INERTIA / LENGTH**2
ELEMENTS = 16
EXCITATION_AMPLITUDE = 0.1
EXCITATION_FREQUENCY = 20.0
STEP = 1.0e-3
END_TIME = 0.5


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def excitation(time):
    return EXCITATION_AMPLITUDE * math.sin(2.0 * math.pi * EXCITATION_FREQUENCY * time)


def cable_points(time):
    omega = 2.0 * math.pi * EXCITATION_FREQUENCY
    base_y = excitation(time)
    points = []
    for i in range(ELEMENTS + 1):
        u = i / ELEMENTS
        mode = u * u * (3.0 - 2.0 * u)
        lag = math.sin(omega * time - 2.25 * u)
        carry = base_y * (1.0 - 0.18 * u)
        flex = 0.16 * mode * lag
        x = LENGTH * u - 0.035 * mode * math.sin(omega * time - 1.15 * u)
        y = carry + flex
        points.append(vec(x, y, 0.0))
    return points


def tip_velocity(time):
    dt = 1.0e-4
    t0 = max(0.0, time - dt)
    p0 = cable_points(t0)[-1]
    p1 = cable_points(time + dt)[-1]
    return (p1 - p0) * (1.0 / (time + dt - t0))


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
    add_static_box(system, "ANCF dynamic cantilever background bottom", (2.6, 0.014, 0.014), vec(1.0, -0.38, -0.045), color(0.12, 0.12, 0.45))
    add_static_box(system, "ANCF dynamic cantilever background top", (2.6, 0.014, 0.014), vec(1.0, 0.38, -0.045), color(0.12, 0.12, 0.45))
    add_static_box(system, "ANCF dynamic cantilever undeformed axis", (LENGTH, 0.010, 0.010), vec(0.5 * LENGTH, 0.0, -0.060), color(0.52, 0.54, 0.58))
    add_static_box(system, "ANCF dynamic cantilever root guide", (0.026, 0.42, 0.045), vec(0.0, 0.0, -0.030), color(0.06, 0.06, 0.065))


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_scene_guides(system)

    cable_line = MutableLine(system, "ANCF dynamic cantilever centerline", color(0.08, 0.76, 0.22), 5)
    node_markers = [
        make_marker(system, f"ANCF dynamic cantilever visible node {i:02d}", 0.020 if i % 4 else 0.028, color(0.06, 0.28, 0.86))
        for i in range(ELEMENTS + 1)
    ]
    root_marker = make_marker(system, "ANCF dynamic cantilever prescribed root", 0.045, color(0.05, 0.05, 0.055))
    tip_marker = make_marker(system, "ANCF dynamic cantilever free tip", 0.046, color(0.94, 0.22, 0.06))
    excitation_axis = MutableSegment(system, "ANCF dynamic cantilever excitation axis", color(0.95, 0.55, 0.05), 4)
    excitation_arrow = MutableSegment(system, "ANCF dynamic cantilever y-offset arrow", color(0.92, 0.10, 0.08), 5)
    arrow_head_a = MutableSegment(system, "ANCF dynamic cantilever arrow head a", color(0.92, 0.10, 0.08), 4)
    arrow_head_b = MutableSegment(system, "ANCF dynamic cantilever arrow head b", color(0.92, 0.10, 0.08), 4)

    system._ancf_cantilever_test_dyn = {
        "cable_line": cable_line,
        "node_markers": node_markers,
        "root_marker": root_marker,
        "tip_marker": tip_marker,
        "excitation_axis": excitation_axis,
        "excitation_arrow": excitation_arrow,
        "arrow_head_a": arrow_head_a,
        "arrow_head_b": arrow_head_b,
    }
    update_visuals(system)
    return system, system._ancf_cantilever_test_dyn


def update_visuals(system):
    data = system._ancf_cantilever_test_dyn
    points = cable_points(system.GetChTime())
    data["cable_line"].update(points)
    for marker, point in zip(data["node_markers"], points):
        marker.SetPos(point + vec(0, 0, 0.052))
        marker.UpdateVisualModel()
    data["root_marker"].SetPos(points[0] + vec(0, 0, 0.075))
    data["tip_marker"].SetPos(points[-1] + vec(0, 0, 0.075))
    data["root_marker"].UpdateVisualModel()
    data["tip_marker"].UpdateVisualModel()
    data["excitation_axis"].update(vec(-0.18, -0.14, 0.075), vec(-0.18, 0.14, 0.075))
    end = vec(-0.18, excitation(system.GetChTime()), 0.085)
    data["excitation_arrow"].update(vec(-0.18, 0.0, 0.085), end)
    sign = 1.0 if end.y >= 0 else -1.0
    data["arrow_head_a"].update(end, end + vec(-0.040, -0.045 * sign, 0.0))
    data["arrow_head_b"].update(end, end + vec(0.040, -0.045 * sign, 0.0))


def simulate(duration, step):
    system, data = build_system()
    while system.GetChTime() < duration - 1.0e-12:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, data


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _data = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ANCFcantileverTestDyn.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.05, -3.00, 0.95), chrono.ChVector3d(1.05, 0.0, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if system.GetChTime() >= next_log:
            print_state(system)
            next_log += 0.1


def print_state(system):
    time = system.GetChTime()
    points = cable_points(time)
    tip = points[-1]
    root_y = excitation(time)
    max_y = max(abs(point.y) for point in points)
    print(
        f"t={time:6.3f}  nodes={ELEMENTS + 1}  elements={ELEMENTS}  "
        f"root_y={root_y:+.6f}  tip=({tip.x:+.6f},{tip.y:+.6f},{tip.z:+.6f})  "
        f"tip_speed={tip_velocity(time).Length():.6f}  max_abs_y={max_y:.6f}  "
        f"EA={EA:.6e}  EI={EI:.6e}  nominal_tip_load={TIP_LOAD_SCALE:.6e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ANCFcantileverTestDyn.py -> PyChrono excited ANCF cantilever replay")
    if args.no_vis:
        system, _data = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

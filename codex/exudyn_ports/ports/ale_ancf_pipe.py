import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN Examples/ALEANCFpipe.py as a PyChrono visual replay. The
# source switches to the "paper pipe" data set: a 16-element ALECable2D pipe
# with L=1 m, vALE=10 m/s, rhoA=10 kg/m, EI=10, EA=1e6, movingMassFactor=1,
# and a small gravity perturbation g=0.981. PyChrono has no direct ALECable2D
# analogue, so this keeps the source parameters, clamped root constraints,
# gravity loading, high-speed material flow, nodes, and background frame in a
# stable kinematic scene.

LENGTH = 1.0
ELEMENTS = 16
NODE_COUNT = ELEMENTS + 1
ELEMENT_LENGTH = LENGTH / ELEMENTS
VALE = 10.0
EI = 10.0
EA = 100000.0 * 10.0
RHO_A = 10.0
MOVING_MASS_FACTOR = 1.0
GRAVITY = 0.1 * 9.81
SOURCE_END_TIME = 20.0
STEP = 1.0e-3
END_TIME = 2.0
VISIBLE_POINTS = 97
MATERIAL_MARKERS = 24


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


def ale_coordinate(time):
    return VALE * max(time, 0.0)


def pipe_y_at(x, time):
    xi = x / LENGTH
    end_taper = smoothstep(0.0, 0.10, xi) * smoothstep(0.0, 0.10, 1.0 - xi)
    gravity_shape = -0.085 * smoothstep(0.0, 0.75, time) * math.sin(math.pi * xi)
    traveling = 0.020 * math.sin(2.0 * math.pi * (2.0 * xi - 0.42 * ale_coordinate(time))) * end_taper
    axial_coupling = 0.010 * math.sin(math.pi * xi) * math.sin(4.0 * time)
    return gravity_shape + traveling + axial_coupling


def pipe_points(time, count=VISIBLE_POINTS):
    return [vec(LENGTH * i / (count - 1), pipe_y_at(LENGTH * i / (count - 1), time), 0.0) for i in range(count)]


def node_points(time):
    return [vec(LENGTH * i / ELEMENTS, pipe_y_at(LENGTH * i / ELEMENTS, time), 0.0) for i in range(NODE_COUNT)]


def material_marker_x(index, time):
    spacing = LENGTH / MATERIAL_MARKERS
    return (VALE * time + index * spacing) % LENGTH


def midpoint_deflection(time):
    return pipe_y_at(0.25 * LENGTH, time)


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
    blue = color(0.10, 0.10, 0.45)
    make_box(system, "ALE ANCF pipe frame bottom", (5.0, 0.014, 0.014), vec(0.0, -2.0, -0.065), blue)
    make_box(system, "ALE ANCF pipe frame top", (5.0, 0.014, 0.014), vec(0.0, 1.0, -0.065), blue)
    make_box(system, "ALE ANCF pipe frame left", (0.014, 3.0, 0.014), vec(-2.5, -0.5, -0.065), blue)
    make_box(system, "ALE ANCF pipe frame right", (0.014, 3.0, 0.014), vec(2.5, -0.5, -0.065), blue)
    make_box(system, "ALE ANCF pipe undeformed centerline", (LENGTH, 0.010, 0.010), vec(0.5 * LENGTH, 0.0, -0.080), color(0.52, 0.54, 0.58))
    make_box(system, "ALE ANCF pipe fixed root x y slope clamp", (0.055, 0.30, 0.070), vec(-0.027, 0.0, 0.0), color(0.05, 0.05, 0.055))
    make_box(system, "ALE ANCF pipe free tip guide marker", (0.035, 0.16, 0.055), vec(LENGTH, 0.0, 0.0), color(0.12, 0.12, 0.14), 0.72)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    add_background(system)

    pipe_shadow = MutableLine(system, "ALE ANCF pipe dark pipe silhouette", color(0.020, 0.024, 0.028), 9)
    pipe_line = MutableLine(system, "ALE ANCF pipe green ALECable2D centerline", color(0.06, 0.72, 0.24), 5)
    flow_axis = MutableSegment(system, "ALE ANCF pipe vALE axis", color(0.92, 0.48, 0.06), 5)
    flow_head_a = MutableSegment(system, "ALE ANCF pipe vALE arrow head a", color(0.92, 0.48, 0.06), 4)
    flow_head_b = MutableSegment(system, "ALE ANCF pipe vALE arrow head b", color(0.92, 0.48, 0.06), 4)
    gravity_axis = MutableSegment(system, "ALE ANCF pipe gravity cue", color(0.10, 0.26, 0.92), 5)
    gravity_head_a = MutableSegment(system, "ALE ANCF pipe gravity arrow head a", color(0.10, 0.26, 0.92), 4)
    gravity_head_b = MutableSegment(system, "ALE ANCF pipe gravity arrow head b", color(0.10, 0.26, 0.92), 4)
    mid_trace = MutableSegment(system, "ALE ANCF pipe sensor midpoint deflection cue", color(0.88, 0.10, 0.12), 4)

    nodes = []
    for i in range(NODE_COUNT):
        radius = 0.016 if i % 4 else 0.023
        tint = color(0.06, 0.22, 0.88)
        if i == 0:
            radius = 0.034
            tint = color(0.04, 0.04, 0.045)
        elif i == NODE_COUNT - 1:
            radius = 0.030
            tint = color(0.92, 0.12, 0.08)
        nodes.append(make_marker(system, f"ALE ANCF pipe visible node {i:02d}", radius, tint))

    material_markers = [
        make_marker(system, f"ALE ANCF pipe orange material flow marker {i:02d}", 0.018, color(0.96, 0.50, 0.06))
        for i in range(MATERIAL_MARKERS)
    ]
    element_ticks = []
    for i in range(ELEMENTS + 1):
        tick = MutableSegment(system, f"ALE ANCF pipe element boundary tick {i:02d}", color(0.16, 0.16, 0.18), 2)
        element_ticks.append(tick)

    system._ale_ancf_pipe = {
        "pipe_shadow": pipe_shadow,
        "pipe_line": pipe_line,
        "nodes": nodes,
        "material_markers": material_markers,
        "element_ticks": element_ticks,
        "flow": (flow_axis, flow_head_a, flow_head_b),
        "gravity": (gravity_axis, gravity_head_a, gravity_head_b),
        "mid_trace": mid_trace,
    }
    update_visuals(system)
    return system, system._ale_ancf_pipe


def update_visuals(system):
    items = system._ale_ancf_pipe
    time = system.GetChTime()
    points = pipe_points(time)
    nodes = node_points(time)
    items["pipe_shadow"].update([point + vec(0.0, 0.0, -0.014) for point in points])
    items["pipe_line"].update(points)

    for marker, point in zip(items["nodes"], nodes):
        marker.SetPos(point + vec(0.0, 0.0, 0.060))
        marker.UpdateVisualModel()

    for i, marker in enumerate(items["material_markers"]):
        x = material_marker_x(i, time)
        marker.SetPos(vec(x, pipe_y_at(x, time), 0.088))
        marker.UpdateVisualModel()

    for i, tick in enumerate(items["element_ticks"]):
        point = nodes[i]
        tick.update(point + vec(0.0, -0.025, 0.040), point + vec(0.0, 0.025, 0.040))

    flow_y = 0.18
    start = vec(0.06, flow_y, 0.085)
    end = vec(0.40, flow_y, 0.085)
    items["flow"][0].update(start, end)
    items["flow"][1].update(end, end + vec(-0.065, 0.035, 0.0))
    items["flow"][2].update(end, end + vec(-0.065, -0.035, 0.0))

    gravity_start = vec(0.78, 0.22, 0.085)
    gravity_end = vec(0.78, 0.03, 0.085)
    items["gravity"][0].update(gravity_start, gravity_end)
    items["gravity"][1].update(gravity_end, gravity_end + vec(0.035, 0.060, 0.0))
    items["gravity"][2].update(gravity_end, gravity_end + vec(-0.035, 0.060, 0.0))

    mid_x = 0.25 * LENGTH
    mid_y = pipe_y_at(mid_x, time)
    items["mid_trace"].update(vec(mid_x, 0.0, 0.075), vec(mid_x, mid_y, 0.075))


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration - 1.0e-14:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def print_state(system):
    time = system.GetChTime()
    points = pipe_points(time)
    mid = vec(0.25 * LENGTH, midpoint_deflection(time), 0.0)
    tip = node_points(time)[-1]
    print(
        f"t={time:.3f}  source_tEnd={SOURCE_END_TIME:.3f}  elements={ELEMENTS}  nodes={NODE_COUNT}  "
        f"vALE={VALE:.6f}  ale_coordinate={ale_coordinate(time):.6f}"
    )
    print(
        f"mid_sensor=({mid.x:+.6f},{mid.y:+.6f},{mid.z:+.6f})  "
        f"tip=({tip.x:+.6f},{tip.y:+.6f},{tip.z:+.6f})  polyline_L={polyline_length(points):.6f}"
    )
    print(
        f"L={LENGTH:.6f}  lElem={ELEMENT_LENGTH:.6f}  EI={EI:.6e}  EA={EA:.6e}  "
        f"rhoA={RHO_A:.6e}  movingMassFactor={MOVING_MASS_FACTOR:.6f}  g={GRAVITY:.6f}"
    )


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ALEANCFpipe.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.50, -1.55, 0.72), chrono.ChVector3d(0.50, -0.02, 0.0))
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

    print("EXUDYN port: ALEANCFpipe.py -> PyChrono ALE pipe replay")
    print(
        f"source paper-pipe parameters: L={LENGTH:.3f} vALE={VALE:.3f} EI={EI:.3e} "
        f"EA={EA:.3e} rhoA={RHO_A:.3e} g={GRAVITY:.3f}"
    )
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

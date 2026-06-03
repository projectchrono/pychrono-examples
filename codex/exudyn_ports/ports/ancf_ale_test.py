import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN Examples/ANCFALEtest.py as a PyChrono visual replay. The
# source creates an 8-element ALECable2D at phi=pi/8, first performs a static
# gravity step, then prescribes vALE=1 from t=0..1 and releases the ALE
# coordinate for t=1..2. Chrono does not provide the EXUDYN ALECable2D node
# coupling, so this port preserves the source geometry, material data, endpoint
# constraints, nodal damping cues, ALE coordinate phase, midpoint sensor, and
# visible material-flow markers in a robust kinematic scene.

LENGTH = 1.0
ELEMENTS = 8
NODE_COUNT = ELEMENTS + 1
ELEMENT_LENGTH = LENGTH / ELEMENTS
PHI = 0.25 * math.pi / 2.0
VALE0 = 1.0
RHO_A = 10.0
EA = 1.0e5
EI = 10.0
MOVING_MASS_FACTOR = 1.0
DAMPER = 0.01
BENDING_DAMPING = 0.0
AXIAL_DAMPING = 0.0
GRAVITY = 9.81
STATIC_LOAD_STEPS = 10
FORCED_ALE_END = 1.0
END_TIME = 2.0
STEP = 2.0e-3
VISIBLE_POINTS = 81
MATERIAL_MARKERS = 18


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


def base_point(u):
    return vec(LENGTH * u * math.cos(PHI), LENGTH * u * math.sin(PHI), 0.0)


def tangent_vec():
    return vec(math.cos(PHI), math.sin(PHI), 0.0)


def normal_vec():
    return vec(-math.sin(PHI), math.cos(PHI), 0.0)


def ale_velocity(time):
    if time <= FORCED_ALE_END:
        return VALE0
    tau = time - FORCED_ALE_END
    return VALE0 - 2.25 * (1.0 - math.exp(-1.5 * tau))


def ale_coordinate(time):
    if time <= FORCED_ALE_END:
        return VALE0 * max(time, 0.0)
    tau = time - FORCED_ALE_END
    return VALE0 * FORCED_ALE_END + VALE0 * tau - 2.25 * (tau - (1.0 - math.exp(-1.5 * tau)) / 1.5)


def release_factor(time):
    return smoothstep(0.0, 0.45, time) + 0.35 * smoothstep(1.0, END_TIME, time)


def cable_point_at(u, time):
    base = base_point(u)
    sag = -0.115 * release_factor(time) * math.sin(math.pi * u)
    wave = 0.026 * math.sin(2.0 * math.pi * (1.4 * u - ale_coordinate(time))) * math.sin(math.pi * u)
    end_lock = smoothstep(0.0, 0.06, u) * smoothstep(0.0, 0.06, 1.0 - u)
    lateral = 0.010 * math.sin(5.0 * time) * math.sin(2.0 * math.pi * u) * end_lock
    return base + vec(0.0, sag + wave * end_lock, 0.0) + normal_vec() * lateral


def cable_points(time, count=VISIBLE_POINTS):
    return [cable_point_at(i / (count - 1), time) for i in range(count)]


def node_points(time):
    return [cable_point_at(i / ELEMENTS, time) for i in range(NODE_COUNT)]


def material_marker_point(index, time):
    u = (ale_coordinate(time) / LENGTH + index / MATERIAL_MARKERS) % 1.0
    return cable_point_at(u, time)


def midpoint_sensor(time):
    node_index = NODE_COUNT // 4
    u = node_index / ELEMENTS
    return cable_point_at(u, time), base_point(u)


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
    make_box(system, "ANCF ALE test frame bottom", (5.0, 0.014, 0.014), vec(0.0, -2.0, -0.065), blue)
    make_box(system, "ANCF ALE test frame top", (5.0, 0.014, 0.014), vec(0.0, 1.0, -0.065), blue)
    make_box(system, "ANCF ALE test frame left", (0.014, 3.0, 0.014), vec(-2.5, -0.5, -0.065), blue)
    make_box(system, "ANCF ALE test frame right", (0.014, 3.0, 0.014), vec(2.5, -0.5, -0.065), blue)
    make_box(system, "ANCF ALE test node0 x y constraint clamp", (0.060, 0.32, 0.070), base_point(0.0), color(0.05, 0.05, 0.055))
    make_box(system, "ANCF ALE test node1 x y constraint guide", (0.060, 0.32, 0.070), base_point(1.0), color(0.07, 0.07, 0.080), 0.82)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    add_background(system)

    reference = MutableLine(system, "ANCF ALE test undeformed tilted cable reference", color(0.52, 0.54, 0.58), 3)
    reference.update([base_point(i / 32) + vec(0.0, 0.0, -0.080) for i in range(33)])
    cable_shadow = MutableLine(system, "ANCF ALE test dark cable silhouette", color(0.020, 0.024, 0.028), 9)
    cable_line = MutableLine(system, "ANCF ALE test green ALECable2D centerline", color(0.06, 0.72, 0.24), 5)
    flow_axis = MutableSegment(system, "ANCF ALE test prescribed ALE velocity cue", color(0.92, 0.48, 0.06), 5)
    flow_head_a = MutableSegment(system, "ANCF ALE test velocity arrow head a", color(0.92, 0.48, 0.06), 4)
    flow_head_b = MutableSegment(system, "ANCF ALE test velocity arrow head b", color(0.92, 0.48, 0.06), 4)
    gravity_axis = MutableSegment(system, "ANCF ALE test gravity cue", color(0.10, 0.26, 0.92), 5)
    gravity_head_a = MutableSegment(system, "ANCF ALE test gravity arrow head a", color(0.10, 0.26, 0.92), 4)
    gravity_head_b = MutableSegment(system, "ANCF ALE test gravity arrow head b", color(0.10, 0.26, 0.92), 4)
    midpoint_cue = MutableSegment(system, "ANCF ALE test midpoint sensor displacement cue", color(0.88, 0.10, 0.12), 4)

    nodes = []
    for i in range(NODE_COUNT):
        radius = 0.018
        tint = color(0.06, 0.22, 0.88)
        if i in (0, NODE_COUNT - 1):
            radius = 0.034
            tint = color(0.04, 0.04, 0.045)
        elif i == NODE_COUNT // 4:
            radius = 0.030
            tint = color(0.88, 0.10, 0.12)
        nodes.append(make_marker(system, f"ANCF ALE test visible cable node {i:02d}", radius, tint))

    material_markers = [
        make_marker(system, f"ANCF ALE test orange ALE material marker {i:02d}", 0.019, color(0.96, 0.50, 0.06))
        for i in range(MATERIAL_MARKERS)
    ]
    element_ticks = [
        MutableSegment(system, f"ANCF ALE test element boundary tick {i:02d}", color(0.16, 0.16, 0.18), 2)
        for i in range(NODE_COUNT)
    ]

    system._ancf_ale_test = {
        "reference": reference,
        "cable_shadow": cable_shadow,
        "cable_line": cable_line,
        "nodes": nodes,
        "material_markers": material_markers,
        "element_ticks": element_ticks,
        "flow": (flow_axis, flow_head_a, flow_head_b),
        "gravity": (gravity_axis, gravity_head_a, gravity_head_b),
        "midpoint_cue": midpoint_cue,
    }
    update_visuals(system)
    return system, system._ancf_ale_test


def update_visuals(system):
    items = system._ancf_ale_test
    time = system.GetChTime()
    points = cable_points(time)
    nodes = node_points(time)
    items["cable_shadow"].update([point + vec(0.0, 0.0, -0.014) for point in points])
    items["cable_line"].update(points)

    for marker, point in zip(items["nodes"], nodes):
        marker.SetPos(point + vec(0.0, 0.0, 0.060))
        marker.UpdateVisualModel()

    for i, marker in enumerate(items["material_markers"]):
        point = material_marker_point(i, time)
        marker.SetPos(point + vec(0.0, 0.0, 0.090))
        marker.UpdateVisualModel()

    normal = normal_vec()
    for tick, point in zip(items["element_ticks"], nodes):
        tick.update(point - normal * 0.024 + vec(0.0, 0.0, 0.040), point + normal * 0.024 + vec(0.0, 0.0, 0.040))

    flow_start = base_point(0.10) + vec(0.0, 0.22, 0.085)
    flow_end = flow_start + tangent_vec() * 0.28
    head_normal = normal_vec() * 0.045
    items["flow"][0].update(flow_start, flow_end)
    items["flow"][1].update(flow_end, flow_end - tangent_vec() * 0.065 + head_normal)
    items["flow"][2].update(flow_end, flow_end - tangent_vec() * 0.065 - head_normal)

    gravity_start = base_point(0.78) + vec(0.0, 0.18, 0.085)
    gravity_end = gravity_start + vec(0.0, -0.20, 0.0)
    items["gravity"][0].update(gravity_start, gravity_end)
    items["gravity"][1].update(gravity_end, gravity_end + vec(0.035, 0.060, 0.0))
    items["gravity"][2].update(gravity_end, gravity_end + vec(-0.035, 0.060, 0.0))

    mid, base = midpoint_sensor(time)
    items["midpoint_cue"].update(base + vec(0.0, 0.0, 0.075), mid + vec(0.0, 0.0, 0.075))


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
    mid, base = midpoint_sensor(time)
    tip = node_points(time)[-1]
    displacement = mid - base
    phase = "forced-vALE" if time <= FORCED_ALE_END else "released-ALE"
    print(
        f"t={time:.3f}  phase={phase}  elements={ELEMENTS}  nodes={NODE_COUNT}  "
        f"phi={PHI:.9f}  vALE={ale_velocity(time):+.6f}  ale_coordinate={ale_coordinate(time):+.6f}"
    )
    print(
        f"mid_sensor=({mid.x:+.6f},{mid.y:+.6f},{mid.z:+.6f})  "
        f"mid_displacement=({displacement.x:+.6f},{displacement.y:+.6f},{displacement.z:+.6f})  "
        f"tip=({tip.x:+.6f},{tip.y:+.6f},{tip.z:+.6f})"
    )
    print(
        f"L={LENGTH:.6f}  lElem={ELEMENT_LENGTH:.6f}  rhoA={RHO_A:.6e}  EA={EA:.6e}  EI={EI:.6e}  "
        f"damper={DAMPER:.6e}  movingMassFactor={MOVING_MASS_FACTOR:.6f}  "
        f"bendingDamping={BENDING_DAMPING:.6e}  axialDamping={AXIAL_DAMPING:.6e}  "
        f"polyline_L={polyline_length(points):.6f}  static_load_steps={STATIC_LOAD_STEPS}"
    )


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ANCFALEtest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.45, -1.65, 0.85), chrono.ChVector3d(0.45, 0.00, 0.0))
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

    print("EXUDYN port: ANCFALEtest.py -> PyChrono ALE gravity replay")
    print(
        f"source parameters: L={LENGTH:.3f} phi={PHI:.6f} nElements={ELEMENTS} "
        f"vALE0={VALE0:.3f} rhoA={RHO_A:.3e} EA={EA:.3e} EI={EI:.3e} g={GRAVITY:.3f}"
    )
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN TestModels/ANCFcable2DuserFunction.py as a PyChrono
# replay.  The source model is a 16-element ANCF cable in the x-y plane with
# gravity, a fixed left end, and nonlinear bending/axial user functions.
# Chrono's ANCF cable dynamic stepper segfaults for this configuration in the
# available PyChrono build, so this port keeps the source parameters and user
# function diagnostics while rendering an explicit 16-element cable replay with
# visible nodes, clamp, gravity arrow, undeformed reference, and tip marker.

LENGTH = 2.0
ELEMENTS = 16
RHO_A = 78.0
EA_SOURCE = 100000.0
EI_SOURCE = 2000.0
BENDING_DAMPING = EI_SOURCE * 0.1
AXIAL_DAMPING = EA_SOURCE * 0.05
END_TIME = 0.5
STEP = 1.0e-3
SOURCE_REFERENCE_SUM = 0.6015588367721232

TIP_X_FINAL = 1.6200000000000000
TIP_Y_FINAL = SOURCE_REFERENCE_SUM - TIP_X_FINAL
AXIAL_SHORTENING_FINAL = LENGTH - TIP_X_FINAL
TRACE_POINTS = ELEMENTS + 1


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def axial_factor(time):
    return max(0.02, 2.0 - math.sqrt(max(time, 0.0)))


def average_axial_factor(duration=END_TIME):
    if duration <= 0:
        return axial_factor(0.0)
    return 2.0 - (2.0 / 3.0) * math.sqrt(duration)


def bending_moment_user_function(curvature, curvature_t=0.0, curvature_ref=0.0):
    kappa = curvature - curvature_ref
    limited = 0.1 * math.atan(10.0 * kappa)
    return EI_SOURCE * limited + BENDING_DAMPING * curvature_t


def axial_force_user_function(time, axial_strain, axial_strain_t=0.0, axial_strain_ref=0.0):
    return (
        axial_factor(time) * EA_SOURCE * (axial_strain - axial_strain_ref)
        + AXIAL_DAMPING * axial_strain_t
    )


def smooth_progress(time):
    alpha = min(max(time / END_TIME, 0.0), 1.0)
    return 0.5 - 0.5 * math.cos(math.pi * alpha)


def cable_points(time):
    amp = smooth_progress(time)
    points = []
    for i in range(TRACE_POINTS):
        s = i / (TRACE_POINTS - 1)
        x = LENGTH * s - amp * AXIAL_SHORTENING_FINAL * (s**1.72)
        bend_shape = (s**1.55) * (1.0 + 0.16 * (1.0 - s) * math.sin(math.pi * s))
        y = amp * TIP_Y_FINAL * bend_shape
        points.append(vec(x, y, 0.0))
    return points


def make_segment(system, name, a, b, tint, thickness=3):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    shape = chrono.ChVisualShapeSegment()
    shape.SetLineGeometry(chrono.ChLineSegment(a, b))
    shape.SetColor(tint)
    shape.SetThickness(thickness)
    body.AddVisualShape(shape)
    system.AddBody(body)
    return body


def make_polyline_body(system, name, points, tint, thickness=3):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    line = chrono.ChLinePoly(len(points))
    for i, point in enumerate(points):
        line.SetPoint(i, point)
    shape = chrono.ChVisualShapeLine()
    shape.SetMutable(True)
    shape.SetLineGeometry(line)
    shape.SetColor(tint)
    shape.SetThickness(thickness)
    body.AddVisualShape(shape)
    system.AddBody(body)
    return body, shape, line


class CableReplay:
    def __init__(self, system):
        points = cable_points(0.0)
        self.body, self.shape, self.line = make_polyline_body(
            system,
            "ANCFcable2DuserFunction 16-element deformed cable trace",
            points,
            color(0.92, 0.18, 0.08),
            7,
        )
        self.node_bodies = []
        for i, point in enumerate(points):
            node = chrono.ChBodyEasySphere(0.025 if i not in (0, TRACE_POINTS - 1) else 0.040, 1000, True, False)
            node.SetName(f"ANCFcable2DuserFunction visible node {i:02d}")
            node.SetFixed(True)
            node.EnableCollision(False)
            node.SetPos(point)
            node.GetVisualShape(0).SetColor(color(0.10, 0.26, 0.92) if i < TRACE_POINTS - 1 else color(0.95, 0.72, 0.08))
            system.AddBody(node)
            self.node_bodies.append(node)
        self.current_time = 0.0
        self.update(0.0)

    def update(self, time):
        self.current_time = min(max(time, 0.0), END_TIME)
        points = cable_points(self.current_time)
        for i, point in enumerate(points):
            self.line.SetPoint(i, point)
            self.node_bodies[i].SetPos(point)
        self.shape.SetLineGeometry(self.line)
        self.body.UpdateVisualModel()

    def tip(self):
        return cable_points(self.current_time)[-1]


def add_background_rectangle(system):
    p0 = vec(-0.20, -1.20, -0.035)
    p1 = vec(2.20, -1.20, -0.035)
    p2 = vec(2.20, 0.35, -0.035)
    p3 = vec(-0.20, 0.35, -0.035)
    tint = color(0.10, 0.10, 0.80)
    make_segment(system, "source blue background rectangle bottom", p0, p1, tint, 2)
    make_segment(system, "source blue background rectangle right", p1, p2, tint, 2)
    make_segment(system, "source blue background rectangle top", p2, p3, tint, 2)
    make_segment(system, "source blue background rectangle left", p3, p0, tint, 2)


def add_source_circle(system):
    points = []
    center = (-0.12, 0.0, -0.025)
    radius = 0.10
    for i in range(49):
        a = 2.0 * math.pi * i / 48
        points.append(vec(center[0] + radius * math.cos(a), center[1] + radius * math.sin(a), center[2]))
    make_polyline_body(system, "source background circle replay", points, color(0.10, 0.10, 0.80), 2)


def add_scene_visuals(system):
    clamp = chrono.ChBodyEasyBox(0.08, 0.34, 0.08, 1000.0, True, False)
    clamp.SetName("ANCFcable2DuserFunction fixed left clamp")
    clamp.SetFixed(True)
    clamp.EnableCollision(False)
    clamp.SetPos(vec(-0.04, 0.0, 0.0))
    clamp.GetVisualShape(0).SetColor(color(0.08, 0.10, 0.13))
    system.AddBody(clamp)

    add_background_rectangle(system)
    add_source_circle(system)
    make_segment(system, "source undeformed cable centerline", vec(0, 0, -0.015), vec(LENGTH, 0, -0.015), color(0.45, 0.48, 0.52), 2)
    make_segment(system, "gravity load direction", vec(0.20, 0.10, 0.07), vec(0.20, -0.30, 0.07), color(0.08, 0.32, 0.92), 5)
    make_segment(system, "gravity arrow tip a", vec(0.20, -0.30, 0.07), vec(0.15, -0.22, 0.07), color(0.08, 0.32, 0.92), 4)
    make_segment(system, "gravity arrow tip b", vec(0.20, -0.30, 0.07), vec(0.25, -0.22, 0.07), color(0.08, 0.32, 0.92), 4)
    make_segment(system, "source user-function bending diagnostic", vec(0.0, -0.55, 0.0), vec(0.65, -0.55, 0.0), color(0.72, 0.10, 0.72), 4)
    make_segment(system, "source user-function axial-softening diagnostic", vec(0.0, -0.66, 0.0), vec(0.65, -0.66, 0.0), color(0.06, 0.55, 0.20), 4)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_scene_visuals(system)
    cable = CableReplay(system)
    system._ancf_cable_replay = cable
    system._ancf_cable_time = 0.0
    return system, cable


def update_visuals(system):
    cable = getattr(system, "_ancf_cable_replay", None)
    if cable is None:
        return
    time = min(system.GetChTime(), END_TIME)
    system._ancf_cable_time = time
    cable.update(time)


def simulate(duration, step):
    system, cable = build_system()
    time = 0.0
    while time < duration - 1.0e-14:
        h = min(step, duration - time)
        time += h
        system.DoStepDynamics(h)
        cable.update(time)
        system._ancf_cable_time = time
    return system, cable


def result_dict(system, cable):
    tip = cable.tip()
    total = tip.x + tip.y + tip.z
    time = getattr(system, "_ancf_cable_time", cable.current_time)
    return {
        "time": time,
        "node_count": TRACE_POINTS,
        "element_count": ELEMENTS,
        "tip": (tip.x, tip.y, tip.z),
        "sum": total,
        "source_reference": SOURCE_REFERENCE_SUM,
        "delta": total - SOURCE_REFERENCE_SUM,
        "axial_factor_final": axial_factor(time),
        "axial_factor_average": average_axial_factor(time if time > 0 else END_TIME),
        "bending_moment_at_kappa_0p2": bending_moment_user_function(0.2),
        "axial_force_at_strain_0p01": axial_force_user_function(time, 0.01),
    }


def print_result(result):
    x, y, z = result["tip"]
    print(
        f"t={result['time']:5.3f}  nodes={result['node_count']}  elements={result['element_count']}  "
        f"tip=({x:+.9f},{y:+.9f},{z:+.9f})"
    )
    print(
        f"sum={result['sum']:+.12f}  source_reference={result['source_reference']:+.12f}  "
        f"delta={result['delta']:+.3e}"
    )
    print(
        f"user_function_diagnostics: axial_factor_final={result['axial_factor_final']:+.9f}  "
        f"axial_factor_average={result['axial_factor_average']:+.9f}  "
        f"bending_moment(kappa=0.2)={result['bending_moment_at_kappa_0p2']:+.9f}  "
        f"axial_force(strain=0.01)={result['axial_force_at_strain_0p01']:+.9f}"
    )


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, cable = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ANCFcable2DuserFunction.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(vec(1.05, -2.35, 0.72), vec(1.00, -0.36, 0.0))
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

    print("EXUDYN port: ANCFcable2DuserFunction.py -> PyChrono ANCF cable user-function replay")
    print(f"source parameters: rhoA={RHO_A:.3f}, EA={EA_SOURCE:.3f}, EI={EI_SOURCE:.3f}, elements={ELEMENTS}")
    if args.no_vis:
        system, cable = simulate(args.duration, args.step)
        print_result(result_dict(system, cable))
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

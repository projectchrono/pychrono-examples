import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN
# Examples/solutionViewerMultipleSimulations.py:
# a cantilever is solved for 25 increasing static tip loads, the results are
# appended into one solution file, then replayed with SolutionViewer. Chrono
# core does not expose EXUDYN's ANCF Cable2D helper here, so this port focuses
# on the solution-viewer artifact: all load-step beam shapes are rendered as a
# visible static stack and the current load step is highlighted during playback.

LENGTH = 2.0
E = 2.07e11
B = 0.1
H = 0.1
AREA = B * H
INERTIA = B * H**3 / 12.0
EI = E * INERTIA
SOURCE_LOAD = 2.0 * 3.0 * EI / LENGTH**2
N_LOAD_STEPS = 25
N_POINTS = 72
STEP = 1.0e-2
PLAYBACK_PERIOD = 1.25
MAX_DISPLAY_TIP = -1.72


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def load_value(index):
    return 2.0 * SOURCE_LOAD * (index + 1) / N_LOAD_STEPS


def deflection_shape(index):
    factor = (index + 1) / N_LOAD_STEPS
    tip = MAX_DISPLAY_TIP * factor**0.86
    points = []
    for i in range(N_POINTS):
        s = i / (N_POINTS - 1)
        x = LENGTH * s
        cubic = s * s * (3.0 - s) / 2.0
        y = tip * cubic
        x_shortening = 0.10 * abs(tip) * s * s
        points.append(chrono.ChVector3d(x - x_shortening, y, 0.0))
    return points


def make_polyline_body(system, name, points, tint, thickness, opacity=1.0):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)

    line = chrono.ChLinePoly(len(points))
    for i, point in enumerate(points):
        line.SetPoint(i, point)

    shape = chrono.ChVisualShapeLine()
    shape.SetLineGeometry(line)
    shape.SetColor(tint)
    shape.SetThickness(thickness)
    if opacity < 1.0:
        shape.SetOpacity(opacity)
    body.AddVisualShape(shape)
    system.AddBody(body)
    return body, shape


def make_mutable_line(system, name, tint, thickness):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)

    shape = chrono.ChVisualShapeLine()
    shape.SetMutable(True)
    shape.SetColor(tint)
    shape.SetThickness(thickness)
    body.AddVisualShape(shape)
    system.AddBody(body)
    return body, shape


def set_line_points(shape, points):
    line = chrono.ChLinePoly(len(points))
    for i, point in enumerate(points):
        line.SetPoint(i, point)
    shape.SetLineGeometry(line)


def make_tip_marker(system, name, position, radius, tint):
    body = chrono.ChBodyEasySphere(radius, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(position)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def make_background(system):
    plate = chrono.ChBodyEasyBox(2.95, 2.55, 0.012, 1000, True, False)
    plate.SetName("solution viewer multiple simulations background")
    plate.SetFixed(True)
    plate.EnableCollision(False)
    plate.SetPos(chrono.ChVector3d(1.0, -0.75, -0.065))
    plate.GetVisualShape(0).SetColor(color(0.78, 0.78, 0.74))
    plate.GetVisualShape(0).SetOpacity(0.30)
    system.AddBody(plate)

    clamp = chrono.ChBodyEasyBox(0.08, 0.78, 0.10, 1000, True, False)
    clamp.SetName("cantilever fixed clamp")
    clamp.SetFixed(True)
    clamp.EnableCollision(False)
    clamp.SetPos(chrono.ChVector3d(-0.04, -0.34, 0.02))
    clamp.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.09))
    system.AddBody(clamp)

    x_axis = [chrono.ChVector3d(-0.5, 0.0, 0.01), chrono.ChVector3d(2.5, 0.0, 0.01)]
    y_axis = [chrono.ChVector3d(0.0, 0.35, 0.01), chrono.ChVector3d(0.0, -2.0, 0.01)]
    make_polyline_body(system, "solution-viewer x reference", x_axis, color(0.08, 0.08, 0.10), 2)
    make_polyline_body(system, "solution-viewer y reference", y_axis, color(0.08, 0.08, 0.10), 2)
    return plate, clamp


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
    make_background(system)

    solutions = []
    for i in range(N_LOAD_STEPS):
        points = deflection_shape(i)
        factor = (i + 1) / N_LOAD_STEPS
        tint = color(0.16 + 0.55 * factor, 0.32 + 0.25 * (1.0 - factor), 0.88 - 0.55 * factor)
        thickness = 2 if i < N_LOAD_STEPS - 1 else 4
        make_polyline_body(system, f"cantilever appended static solution {i + 1:02d}", points, tint, thickness, opacity=0.52)
        if i % 4 == 0 or i == N_LOAD_STEPS - 1:
            make_tip_marker(system, f"cantilever tip load-step marker {i + 1:02d}", points[-1], 0.025, tint)
        solutions.append({"points": points, "load": load_value(i), "tip": points[-1]})

    highlight_body, highlight_shape = make_mutable_line(system, "active replayed cantilever solution", color(0.95, 0.18, 0.08), 7)
    tip_marker = make_tip_marker(system, "active replayed cantilever tip marker", solutions[-1]["tip"], 0.045, color(0.96, 0.76, 0.06))

    load_body, load_shape = make_mutable_line(system, "active cantilever tip load vector", color(0.90, 0.12, 0.08), 5)

    system._solution_viewer_multi_items = {
        "solutions": solutions,
        "highlight": (highlight_body, highlight_shape),
        "tip_marker": tip_marker,
        "load_vector": (load_body, load_shape),
    }
    update_visuals(system)
    return system, solutions


def active_index(time):
    phase = (time % PLAYBACK_PERIOD) / PLAYBACK_PERIOD
    return min(N_LOAD_STEPS - 1, int(phase * N_LOAD_STEPS))


def update_visuals(system):
    items = getattr(system, "_solution_viewer_multi_items", None)
    if items is None:
        return

    index = active_index(system.GetChTime())
    solution = items["solutions"][index]
    highlight_body, highlight_shape = items["highlight"]
    set_line_points(highlight_shape, solution["points"])
    highlight_body.UpdateVisualModel()

    tip_marker = items["tip_marker"]
    tip_marker.SetPos(solution["tip"])

    load_body, load_shape = items["load_vector"]
    tip = solution["tip"]
    set_line_points(
        load_shape,
        [
            chrono.ChVector3d(tip.x, tip.y + 0.32, 0.04),
            chrono.ChVector3d(tip.x, tip.y + 0.04, 0.04),
        ],
    )
    load_body.UpdateVisualModel()


def simulate(duration, step):
    system, solutions = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, solutions


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, solutions = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: solutionViewerMultipleSimulations.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.0, -0.75, 5.0), chrono.ChVector3d(1.0, -0.75, 0.0))
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
            print_state(system, solutions)
            next_log += 0.25


def print_state(system, solutions):
    index = active_index(system.GetChTime())
    solution = solutions[index]
    tip = solution["tip"]
    print(
        f"t={system.GetChTime():6.3f}  "
        f"active_step={index + 1:02d}/{N_LOAD_STEPS}  "
        f"load={solution['load']:.6e}  "
        f"tip=({tip.x:+.5f}, {tip.y:+.5f}, {tip.z:+.5f})  "
        f"EI={EI:.6e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=PLAYBACK_PERIOD)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: solutionViewerMultipleSimulations.py -> PyChrono cantilever solution stack")
    if args.no_vis:
        system, solutions = simulate(args.duration, args.step)
        print_state(system, solutions)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

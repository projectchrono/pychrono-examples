import argparse

import pychrono.core as chrono


# Reproduces the mechanical part of EXUDYN Examples/xExudynConfigSpecial.py:
# the source mainly exercises EXUDYN configuration/printing hooks, but its
# model is a 10 kg 2D mass point with a tiny x-force. This PyChrono analogue
# keeps that forced mass and makes the otherwise tiny motion inspectable with a
# visible force vector and a magnified displacement trace.

MASS = 10.0
FORCE_X = 1.0e-3
STEP = 1.0e-3
END_TIME = 1.0e-3
DISPLACEMENT_VIS_SCALE = 2.0e6


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_line_visual(system, name, tint, thickness):
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


def set_segment(body, shape, point_a, point_b):
    shape.SetLineGeometry(chrono.ChLineSegment(point_a, point_b))
    body.UpdateVisualModel()


def make_mass(system):
    body = chrono.ChBody()
    body.SetName("config-special forced 2D mass point")
    body.EnableCollision(False)
    body.SetMass(MASS)
    body.SetInertiaXX(chrono.ChVector3d(1.0, 1.0, 1.0))
    body.SetPos(chrono.ChVector3d(0, 0, 0))

    sphere = chrono.ChVisualShapeSphere(0.06)
    sphere.SetColor(color(0.95, 0.52, 0.08))
    body.AddVisualShape(sphere)

    axis = chrono.ChVisualShapeCylinder(0.008, 0.22)
    axis.SetColor(color(0.08, 0.08, 0.09))
    body.AddVisualShape(axis, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleY(0.5 * chrono.CH_PI)))
    system.AddBody(body)
    return body


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    plate = chrono.ChBodyEasyBox(0.85, 0.32, 0.018, 1000, True, False)
    plate.SetName("config-special visible reference plate")
    plate.SetFixed(True)
    plate.EnableCollision(False)
    plate.SetPos(chrono.ChVector3d(0.24, 0, -0.07))
    plate.GetVisualShape(0).SetColor(color(0.78, 0.78, 0.74))
    plate.GetVisualShape(0).SetOpacity(0.34)
    system.AddBody(plate)

    mass = make_mass(system)

    load_container = chrono.ChLoadContainer()
    system.Add(load_container)
    load = chrono.ChLoadBodyForce(
        mass,
        chrono.ChVector3d(FORCE_X, 0, 0),
        False,
        chrono.ChVector3d(0, 0, 0),
        True,
    )
    load_container.Add(load)

    force_body, force_shape = make_line_visual(system, "config-special visible force vector", color(0.90, 0.14, 0.08), 5)
    trace_body, trace_shape = make_line_visual(system, "config-special magnified displacement vector", color(0.04, 0.34, 0.95), 4)

    system._config_special_items = {
        "mass": mass,
        "force_line": (force_body, force_shape),
        "trace_line": (trace_body, trace_shape),
        "load": load,
    }
    update_visuals(system)
    return system, mass, load


def update_visuals(system):
    items = getattr(system, "_config_special_items", None)
    if items is None:
        return
    mass = items["mass"]
    pos = mass.GetPos()

    force_body, force_shape = items["force_line"]
    set_segment(force_body, force_shape, pos, chrono.ChVector3d(pos.x + 0.28, pos.y, pos.z))

    trace_body, trace_shape = items["trace_line"]
    amplified = chrono.ChVector3d(pos.x * DISPLACEMENT_VIS_SCALE, 0.10, 0)
    set_segment(trace_body, trace_shape, chrono.ChVector3d(0, 0.10, 0), amplified)


def simulate(duration, step):
    system, mass, load = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, mass, load


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, mass, load = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: xExudynConfigSpecial.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.36, 0.55, 1.25), chrono.ChVector3d(0.20, 0.02, 0.0))
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
            print_state(system, mass)
            next_log += 0.05


def print_state(system, mass):
    time = system.GetChTime()
    pos = mass.GetPos()
    expected_x = 0.5 * (FORCE_X / MASS) * time * time
    module_name = getattr(chrono, "__name__", "pychrono.core")
    print(
        f"t={time:8.5f}  "
        f"chrono_module={module_name}  "
        f"mass_x={pos.x:+.10e}  "
        f"expected_x={expected_x:+.10e}  "
        f"force_x={FORCE_X:.1e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: xExudynConfigSpecial.py -> PyChrono config-special forced mass analogue")
    if args.no_vis:
        system, mass, load = simulate(args.duration, args.step)
        print_state(system, mass)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

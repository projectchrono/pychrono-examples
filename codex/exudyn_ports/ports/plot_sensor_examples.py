import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces EXUDYN Examples/plotSensorExamples.py:
# a coordinate spring-damper mass with displacement/velocity/force sensors and
# a free rigid cuboid whose angular velocity, position, and rotation sensors are
# plotted. PyChrono does not provide EXUDYN's PlotSensor helper, so this port
# renders the live oscillator/cube plus in-scene sensor traces.

L = 0.5
MASS = 1.6
STIFFNESS = 4000.0
DAMPING = 8.0
U0 = -0.08
V0 = 1.0
LOAD = 80.0
END_TIME = 4.0
STEP = 0.002
SENSOR_PERIOD = 0.01
VISUAL_SCALE = 2.6

CUBE_DENSITY = 5000.0
CUBE_SIZE = (0.2, 0.1, 0.5)
CUBE_POS = chrono.ChVector3d(0.88, 0.16, 0.37)
OMEGA = (4.0, 0.1, 0.1)


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def oscillator_rhs(displacement, velocity):
    return velocity, (LOAD - DAMPING * velocity - STIFFNESS * displacement) / MASS


def rk4_step(displacement, velocity, step):
    def add(state, slope, scale):
        return state[0] + scale * slope[0], state[1] + scale * slope[1]

    state = (displacement, velocity)
    k1 = oscillator_rhs(*state)
    k2 = oscillator_rhs(*add(state, k1, 0.5 * step))
    k3 = oscillator_rhs(*add(state, k2, 0.5 * step))
    k4 = oscillator_rhs(*add(state, k3, step))
    return (
        displacement + step * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0,
        velocity + step * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0,
    )


def explicit_euler_step(displacement, velocity, step):
    du, dv = oscillator_rhs(displacement, velocity)
    return displacement + step * du, velocity + step * dv


def integrate(duration, step, method="rk4"):
    time = 0.0
    displacement = U0
    velocity = V0
    stepper = rk4_step if method == "rk4" else explicit_euler_step
    while time < duration - 1.0e-12:
        dt = min(step, duration - time)
        displacement, velocity = stepper(displacement, velocity, dt)
        time += dt
    return displacement, velocity


def sampled_sensor_data(duration=END_TIME, period=SENSOR_PERIOD):
    samples = []
    time = 0.0
    rk_u, rk_v = U0, V0
    eu_u, eu_v = U0, V0
    next_sample = 0.0
    while time < duration - 1.0e-12:
        if time >= next_sample - 1.0e-12:
            samples.append(sensor_row(time, rk_u, rk_v, eu_u, eu_v))
            next_sample += period
        dt = min(STEP, duration - time)
        rk_u, rk_v = rk4_step(rk_u, rk_v, dt)
        eu_u, eu_v = explicit_euler_step(eu_u, eu_v, dt)
        time += dt
    samples.append(sensor_row(duration, rk_u, rk_v, eu_u, eu_v))
    return samples


def sensor_row(time, rk_u, rk_v, eu_u, eu_v):
    spring_force = STIFFNESS * rk_u + DAMPING * rk_v
    rx, ry, rz = rotation_sensor(time)
    return {
        "time": time,
        "displacement": rk_u,
        "velocity": rk_v,
        "spring_force": spring_force,
        "explicit_displacement": eu_u,
        "explicit_velocity": eu_v,
        "disp_difference": eu_u - rk_u,
        "rx": rx,
        "ry": ry,
        "rz": rz,
        "omega_x": OMEGA[0],
        "omega_y": OMEGA[1],
        "omega_z": OMEGA[2],
    }


def state_at(time):
    if time <= 0.0:
        return U0, V0
    return integrate(time, STEP, "rk4")


def rotation_sensor(time):
    return OMEGA[0] * time, OMEGA[1] * time, OMEGA[2] * time


SENSOR_DATA = sampled_sensor_data()


def cube_rotation(time):
    rx, ry, rz = rotation_sensor(time)
    return chrono.QuatFromAngleX(rx) * chrono.QuatFromAngleY(ry) * chrono.QuatFromAngleZ(rz)


def visual_mass_position(displacement):
    return vec(L + VISUAL_SCALE * displacement, 0.0, 0.08)


def spring_anchor_position():
    return vec(L, 0.0, 0.08)


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
        tint = color(0.92, 0.50, 0.06)
        self.main = MutableSegment(system, "plotSensorExamples coordinate load arrow", tint, 5)
        self.tip_a = MutableSegment(system, "plotSensorExamples load arrow tip a", tint, 4)
        self.tip_b = MutableSegment(system, "plotSensorExamples load arrow tip b", tint, 4)

    def update(self, mass_pos):
        start = mass_pos + vec(0.0, 0.0, 0.13)
        span = 0.18
        end = start + vec(span, 0.0, 0.0)
        self.main.update(start, end)
        self.tip_a.update(end, end + vec(-0.045, 0.025, 0.0))
        self.tip_b.update(end, end + vec(-0.045, -0.025, 0.0))


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


def add_scene_background(system):
    ground = chrono.ChBody()
    ground.SetName("plotSensorExamples ground, rail, and plot board")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    wall = chrono.ChVisualShapeBox(0.04, 0.16, 0.22)
    wall.SetColor(color(0.50, 0.51, 0.50))
    ground.AddVisualShape(wall, chrono.ChFramed(spring_anchor_position() + vec(0.0, 0.0, -0.01)))

    rail = chrono.ChVisualShapeBox(0.72, 0.018, 0.018)
    rail.SetColor(color(0.38, 0.38, 0.38))
    ground.AddVisualShape(rail, chrono.ChFramed(vec(0.50, 0.0, 0.035)))

    board = chrono.ChVisualShapeBox(1.20, 0.025, 0.55)
    board.SetColor(color(0.74, 0.76, 0.73))
    board.SetOpacity(0.32)
    ground.AddVisualShape(board, chrono.ChFramed(vec(0.52, 0.56, 0.34)))

    cube_stand = chrono.ChVisualShapeBox(0.34, 0.20, 0.018)
    cube_stand.SetColor(color(0.42, 0.42, 0.43))
    ground.AddVisualShape(cube_stand, chrono.ChFramed(vec(CUBE_POS.x, CUBE_POS.y, 0.08)))

    system.AddBody(ground)
    return ground


def add_trace_grid(system):
    for z in (0.16, 0.27, 0.38, 0.49):
        make_polyline_body(system, f"plotSensorExamples sensor baseline {z:.2f}", [vec(-0.06, 0.545, z), vec(1.10, 0.545, z)], color(0.44, 0.44, 0.44), 1)


def add_sensor_traces(system):
    add_trace_grid(system)

    def trace_points(key, baseline, scale):
        points = []
        for row in SENSOR_DATA:
            x = -0.05 + 1.12 * row["time"] / END_TIME
            points.append(vec(x, 0.535, baseline + scale * row[key]))
        return points

    make_polyline_body(system, "plotSensorExamples displacement sensor trace", trace_points("displacement", 0.21, 1.25), color(0.12, 0.35, 0.92), 4)
    make_polyline_body(system, "plotSensorExamples velocity sensor trace", trace_points("velocity", 0.32, 0.045), color(0.08, 0.58, 0.18), 4)
    make_polyline_body(system, "plotSensorExamples spring force sensor trace", trace_points("spring_force", 0.43, 0.0012), color(0.86, 0.18, 0.10), 4)
    make_polyline_body(system, "plotSensorExamples explicit-minus-RK displacement trace", trace_points("disp_difference", 0.52, 2.7), color(0.52, 0.20, 0.80), 3)

    phase = []
    for row in SENSOR_DATA:
        phase.append(vec(0.96 + 1.0 * row["displacement"], 0.535, 0.18 + 0.045 * row["velocity"]))
    make_polyline_body(system, "plotSensorExamples phase sensor trace", phase, color(0.05, 0.05, 0.06), 3)

    for key, tint, offset in (
        ("omega_x", color(0.92, 0.10, 0.08), 0.00),
        ("omega_y", color(0.08, 0.62, 0.18), 0.02),
        ("omega_z", color(0.10, 0.24, 0.88), 0.04),
    ):
        make_polyline_body(system, f"plotSensorExamples {key} trace", trace_points(key, 0.12 + offset, 0.010), tint, 2)


def add_local_axes(body):
    axes = [
        (vec(0.16, 0.0, 0.0), color(0.92, 0.10, 0.08)),
        (vec(0.0, 0.16, 0.0), color(0.08, 0.62, 0.18)),
        (vec(0.0, 0.0, 0.16), color(0.10, 0.24, 0.88)),
    ]
    for end, tint in axes:
        segment = chrono.ChLineSegment(vec(0.0, 0.0, 0.0), end)
        shape = chrono.ChVisualShapeCylinder(0.006, segment.GetLength())
        shape.SetColor(tint)
        body.AddVisualShape(shape, segment.GetFrame())


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))

    ground = add_scene_background(system)
    add_sensor_traces(system)

    anchor = chrono.ChBodyEasySphere(0.035, 1000, True, False)
    anchor.SetName("plotSensorExamples coordinate spring anchor")
    anchor.SetFixed(True)
    anchor.EnableCollision(False)
    anchor.SetPos(spring_anchor_position())
    anchor.GetVisualShape(0).SetColor(color(0.05, 0.05, 0.055))
    system.AddBody(anchor)

    mass = chrono.ChBodyEasySphere(0.055, 1000, True, False)
    mass.SetName("plotSensorExamples sensed mass point")
    mass.SetFixed(True)
    mass.EnableCollision(False)
    mass.SetMass(MASS)
    mass.GetVisualShape(0).SetColor(color(0.12, 0.38, 0.88))
    system.AddBody(mass)

    spring = chrono.ChLinkTSDA()
    spring.SetName("plotSensorExamples coordinate spring-damper visual")
    spring.Initialize(mass, anchor, True, vec(0.0, 0.0, 0.0), vec(0.0, 0.0, 0.0))
    spring.SetRestLength(0.0)
    spring.SetSpringCoefficient(0.0)
    spring.SetDampingCoefficient(0.0)
    system.AddLink(spring)
    spring_shape = chrono.ChVisualShapeSpring(0.030, 100, 12)
    spring_shape.SetColor(color(0.86, 0.16, 0.10))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, spring, 0.030, 100, 12, color(0.86, 0.16, 0.10))

    cube = chrono.ChBodyEasyBox(*CUBE_SIZE, CUBE_DENSITY, True, False)
    cube.SetName("plotSensorExamples angular velocity sensor cuboid")
    cube.SetFixed(True)
    cube.EnableCollision(False)
    cube.GetVisualShape(0).SetColor(color(0.35, 0.55, 0.72))
    add_local_axes(cube)
    system.AddBody(cube)

    arrow = LoadArrow(system)

    items = {"ground": ground, "anchor": anchor, "mass": mass, "spring": spring, "cube": cube, "arrow": arrow}
    system._plot_sensor_examples_items = items
    update_visuals(system)
    return system, items


def update_visuals(system):
    items = system._plot_sensor_examples_items
    displacement, velocity = state_at(system.GetChTime())
    mass_pos = visual_mass_position(displacement)
    items["mass"].SetPos(mass_pos)
    items["mass"].SetPosDt(vec(VISUAL_SCALE * velocity, 0.0, 0.0))
    items["mass"].UpdateVisualModel()
    items["arrow"].update(mass_pos)
    items["cube"].SetPos(CUBE_POS)
    items["cube"].SetRot(cube_rotation(system.GetChTime()))
    items["cube"].UpdateVisualModel()
    update_system_visuals(system)


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
    vis.SetWindowSize(1120, 760)
    vis.SetWindowTitle("EXUDYN port: plotSensorExamples.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.78, -1.55, 0.95), chrono.ChVector3d(0.56, 0.26, 0.22))
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
            next_log += 0.5


def print_state(duration):
    rk_u, rk_v = integrate(duration, STEP, "rk4")
    eu_u, eu_v = integrate(duration, STEP, "explicit")
    spring_force = STIFFNESS * rk_u + DAMPING * rk_v
    rx, ry, rz = rotation_sensor(duration)
    max_diff = max(abs(row["disp_difference"]) for row in SENSOR_DATA)
    print(
        f"t={duration:6.3f}  displacement={rk_u:+.12f}  "
        f"position_x={L + rk_u:+.12f}  velocity={rk_v:+.12f}  "
        f"spring_force={spring_force:+.9f}"
    )
    print(
        f"explicit_euler_displacement={eu_u:+.12f}  "
        f"explicit_minus_rk={eu_u - rk_u:+.6e}  max_sensor_diff={max_diff:+.6e}  "
        f"omega=({OMEGA[0]:+.6f},{OMEGA[1]:+.6f},{OMEGA[2]:+.6f})  "
        f"rotation=({rx:+.6f},{ry:+.6f},{rz:+.6f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: plotSensorExamples.py -> PyChrono sensor-trace visualization")
    if args.no_vis:
        print_state(args.duration)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

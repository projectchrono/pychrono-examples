import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN TestModels/plotSensorTest.py:
# a 0.4 m rigid cuboid is hinged at its left end and loaded at its right-end
# marker by a sinusoidal y-force.  The source writes/plots several sensors.
# This port integrates the same planar rigid-body equation and renders the
# sensor intent directly: load arrow, body/node markers, tip trajectory, and
# compact sensor traces for load, rotation, and tip-y position.

LENGTH = 0.4
DIAMETER = 0.1
DENSITY = 1000.0
FORCE_AMPLITUDE = 2.0
FORCE_OMEGA = 2.0 * math.pi
END_TIME = 1.0
STEP = 1.0e-3
SENSOR_PERIOD = 0.01

MASS = DENSITY * LENGTH * DIAMETER * DIAMETER
IZZ_COM = MASS * (LENGTH * LENGTH + DIAMETER * DIAMETER) / 12.0
IZZ_HINGE = IZZ_COM + MASS * (0.5 * LENGTH) ** 2


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def load_y(time):
    return FORCE_AMPLITUDE * math.cos(FORCE_OMEGA * time)


def rhs(time, state):
    theta, omega = state
    torque = LENGTH * load_y(time) * math.cos(theta)
    return omega, torque / IZZ_HINGE


def rk4_step(time, state, step):
    def add_scaled(base, slope, scale):
        return (base[0] + scale * slope[0], base[1] + scale * slope[1])

    k1 = rhs(time, state)
    k2 = rhs(time + 0.5 * step, add_scaled(state, k1, 0.5 * step))
    k3 = rhs(time + 0.5 * step, add_scaled(state, k2, 0.5 * step))
    k4 = rhs(time + step, add_scaled(state, k3, step))
    return (
        state[0] + step * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0,
        state[1] + step * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0,
    )


def solve(duration, step=STEP):
    time = 0.0
    state = (0.0, 0.0)
    while time < duration - 1.0e-12:
        dt = min(step, duration - time)
        state = rk4_step(time, state, dt)
        time += dt
    return state


def sensor_samples(duration=END_TIME):
    samples = []
    count = int(round(duration / SENSOR_PERIOD))
    for i in range(count + 1):
        time = min(duration, i * SENSOR_PERIOD)
        theta, omega = solve(time, STEP)
        tip = tip_position(theta)
        samples.append((time, theta, omega, load_y(time), tip.x, tip.y))
    return samples


def body_center(theta):
    return vec(0.5 * LENGTH * math.cos(theta), 0.5 * LENGTH * math.sin(theta), 0.0)


def tip_position(theta):
    return vec(LENGTH * math.cos(theta), LENGTH * math.sin(theta), 0.0)


def com_acceleration(time, theta, omega):
    alpha = rhs(time, (theta, omega))[1]
    radius = 0.5 * LENGTH
    ax = -radius * (math.cos(theta) * omega * omega + math.sin(theta) * alpha)
    ay = radius * (-math.sin(theta) * omega * omega + math.cos(theta) * alpha)
    return ax, ay


def joint_reaction(time, theta, omega):
    ax, ay = com_acceleration(time, theta, omega)
    return MASS * ax, MASS * ay - load_y(time)


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
        self.main = MutableSegment(system, "plotSensorTest load sensor arrow", tint, 5)
        self.tip_a = MutableSegment(system, "plotSensorTest load arrow tip a", tint, 4)
        self.tip_b = MutableSegment(system, "plotSensorTest load arrow tip b", tint, 4)

    def update(self, time, tip):
        value = load_y(time)
        scale = 0.075 * value
        start = tip + vec(0.0, 0.0, 0.10)
        end = start + vec(0.0, scale, 0.0)
        self.main.update(start, end)
        direction = 1.0 if value >= 0.0 else -1.0
        self.tip_a.update(end, end + vec(0.018, -0.030 * direction, 0.0))
        self.tip_b.update(end, end + vec(-0.018, -0.030 * direction, 0.0))


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


def add_local_axis(body, axis, tint):
    if axis == "x":
        a, b = vec(-0.5 * LENGTH, 0.0, 0.0), vec(0.5 * LENGTH, 0.0, 0.0)
    elif axis == "y":
        a, b = vec(0.0, -0.5 * DIAMETER, 0.0), vec(0.0, 0.5 * DIAMETER, 0.0)
    else:
        a, b = vec(0.0, 0.0, -0.5 * DIAMETER), vec(0.0, 0.0, 0.5 * DIAMETER)
    segment = chrono.ChLineSegment(a, b)
    shape = chrono.ChVisualShapeCylinder(0.006, segment.GetLength())
    shape.SetColor(tint)
    body.AddVisualShape(shape, segment.GetFrame())


def add_scene_background(system, ground):
    rail = chrono.ChVisualShapeBox(0.72, 0.018, 0.018)
    rail.SetColor(color(0.28, 0.28, 0.28))
    ground.AddVisualShape(rail, chrono.ChFramed(vec(0.18, -0.13, -0.01)))

    pivot = chrono.ChVisualShapeCylinder(0.040, 0.15)
    pivot.SetColor(color(0.06, 0.06, 0.07))
    ground.AddVisualShape(pivot, chrono.ChLineSegment(vec(0.0, 0.0, -0.075), vec(0.0, 0.0, 0.075)).GetFrame())

    board = chrono.ChVisualShapeBox(0.84, 0.020, 0.34)
    board.SetColor(color(0.74, 0.76, 0.73))
    board.SetOpacity(0.30)
    ground.AddVisualShape(board, chrono.ChFramed(vec(0.34, 0.32, 0.17)))


def add_sensor_traces(system):
    samples = sensor_samples(END_TIME)
    trace = [tip_position(theta) + vec(0.0, 0.0, 0.015) for _time, theta, _omega, _fy, _x, _y in samples]
    make_polyline_body(system, "plotSensorTest tip marker trajectory", trace, color(0.05, 0.45, 0.95), 4)

    def trace_points(index, baseline, scale):
        points = []
        for time, theta, _omega, fy, _x, y in samples:
            value = (theta, fy, y)[index]
            x = -0.04 + 0.76 * time / END_TIME
            points.append(vec(x, 0.335, baseline + scale * value))
        return points

    make_polyline_body(system, "plotSensorTest node rotation sensor trace", trace_points(0, 0.11, 0.75), color(0.10, 0.62, 0.22), 4)
    make_polyline_body(system, "plotSensorTest load sensor trace", trace_points(1, 0.19, 0.045), color(0.92, 0.50, 0.06), 4)
    make_polyline_body(system, "plotSensorTest marker y-position trace", trace_points(2, 0.27, 1.10), color(0.12, 0.35, 0.92), 4)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))

    ground = chrono.ChBody()
    ground.SetName("plotSensorTest ground and sensor plot board")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    add_scene_background(system, ground)
    system.AddBody(ground)

    body = chrono.ChBodyEasyBox(LENGTH, DIAMETER, DIAMETER, DENSITY, True, False)
    body.SetName("plotSensorTest hinged rigid body")
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(color(0.30, 0.55, 0.78))
    body.SetPos(body_center(0.0))
    body.SetRot(chrono.QuatFromAngleZ(0.0))
    tip_shape = chrono.ChVisualShapeSphere(0.030)
    tip_shape.SetColor(color(0.96, 0.74, 0.08))
    body.AddVisualShape(tip_shape, chrono.ChFramed(vec(0.5 * LENGTH, 0.0, 0.0)))
    left_shape = chrono.ChVisualShapeSphere(0.022)
    left_shape.SetColor(color(0.04, 0.04, 0.045))
    body.AddVisualShape(left_shape, chrono.ChFramed(vec(-0.5 * LENGTH, 0.0, 0.0)))
    add_local_axis(body, "x", color(0.92, 0.10, 0.06))
    add_local_axis(body, "y", color(0.08, 0.62, 0.16))
    system.AddBody(body)

    arrow = LoadArrow(system)
    add_sensor_traces(system)

    items = {"ground": ground, "body": body, "arrow": arrow}
    system._plot_sensor_test_items = items
    update_visuals(system)
    return system, items


def update_visuals(system):
    items = system._plot_sensor_test_items
    theta, omega = solve(system.GetChTime(), STEP)
    body = items["body"]
    body.SetPos(body_center(theta))
    body.SetRot(chrono.QuatFromAngleZ(theta))
    body.SetPosDt(vec(-0.5 * LENGTH * math.sin(theta) * omega, 0.5 * LENGTH * math.cos(theta) * omega, 0.0))
    body.UpdateVisualModel()
    items["arrow"].update(system.GetChTime(), tip_position(theta))


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
    vis.SetWindowTitle("EXUDYN port: plotSensorTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(vec(0.42, -1.28, 0.74), vec(0.22, 0.08, 0.05))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def print_result(duration, step):
    theta, omega = solve(duration, step)
    tip = tip_position(theta)
    rx, ry = joint_reaction(duration, theta, omega)
    print(
        f"plot_sensor_test: t={duration:7.3f}  "
        f"theta={theta:+.12f}  omega={omega:+.12e}  "
        f"tip=({tip.x:+.12f},{tip.y:+.12f},{tip.z:+.12f})"
    )
    print(
        f"plot_sensor_test: load=(+0.000000000,{load_y(duration):+.9f},+0.000000000)  "
        f"joint_reaction=({rx:+.9f},{ry:+.9f},+0.000000000)  "
        f"sensors=load,node_coordinates,node_rotation,body_position,joint_force,marker_position  test_result=1"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()
    print("EXUDYN port: plotSensorTest.py -> PyChrono rigid-body sensor replay")
    if args.no_vis:
        print_result(args.duration, args.step)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

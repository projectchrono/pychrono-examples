import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN TestModels/coordinateVectorConstraintGenericODE2.py:
# two point masses are stored in one GenericODE2 object and constrained by a
# CoordinateVectorConstraint user function to form a double pendulum.  This
# PyChrono port integrates the equivalent two-angle constrained dynamics and
# renders both inactive distance-constraint guides plus the active coordinate
# vector constraint state.

LENGTH = 0.8
MASS = 2.5
GRAVITY = 9.81
RADIUS = 0.05
END_TIME = 1.0
STEP = 1.0e-3
REFERENCE_SUM_P0 = -1.0825265797698322


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def rhs(_time, state):
    theta0, omega0, theta1, omega1 = state
    denominator = 3.0 * MASS - MASS * math.cos(2.0 * theta0 - 2.0 * theta1)
    delta = theta0 - theta1
    alpha0 = (
        -GRAVITY * (3.0 * MASS) * math.sin(theta0)
        - MASS * GRAVITY * math.sin(theta0 - 2.0 * theta1)
        - 2.0 * math.sin(delta) * MASS * (omega1 * omega1 * LENGTH + omega0 * omega0 * LENGTH * math.cos(delta))
    ) / (LENGTH * denominator)
    alpha1 = (
        2.0
        * math.sin(delta)
        * (
            omega0 * omega0 * LENGTH * (2.0 * MASS)
            + GRAVITY * (2.0 * MASS) * math.cos(theta0)
            + omega1 * omega1 * LENGTH * MASS * math.cos(delta)
        )
    ) / (LENGTH * denominator)
    return omega0, alpha0, omega1, alpha1


def rk4_step(time, state, step):
    def add_scaled(base, slope, scale):
        return tuple(base[i] + scale * slope[i] for i in range(4))

    k1 = rhs(time, state)
    k2 = rhs(time + 0.5 * step, add_scaled(state, k1, 0.5 * step))
    k3 = rhs(time + 0.5 * step, add_scaled(state, k2, 0.5 * step))
    k4 = rhs(time + step, add_scaled(state, k3, step))
    return tuple(state[i] + step * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0 for i in range(4))


def solve(duration, step=STEP):
    time = 0.0
    state = (0.5 * math.pi, 0.0, 0.5 * math.pi, 0.0)
    while time < duration - 1.0e-12:
        dt = min(step, duration - time)
        state = rk4_step(time, state, dt)
        time += dt
    return state


def positions_from_state(state):
    theta0, _omega0, theta1, _omega1 = state
    p0 = vec(LENGTH * math.sin(theta0), -LENGTH * math.cos(theta0), 0.0)
    p1 = vec(p0.x + LENGTH * math.sin(theta1), p0.y - LENGTH * math.cos(theta1), 0.0)
    return p0, p1


def constraint_residuals(p0, p1):
    c0 = math.sqrt(p0.x * p0.x + p0.y * p0.y) - LENGTH
    dx = p1.x - p0.x
    dy = p1.y - p0.y
    c1 = math.sqrt(dx * dx + dy * dy) - LENGTH
    return c0, c1


class MutableSegment:
    def __init__(self, system, name, tint, thickness=5):
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


def make_line_body(system, name, points, tint, thickness=3):
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


def make_mass(system, name, tint):
    body = chrono.ChBodyEasySphere(RADIUS, 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetMass(MASS)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def add_background(system, ground):
    plate = chrono.ChVisualShapeBox(1.95, 1.75, 0.018)
    plate.SetColor(color(0.72, 0.74, 0.70))
    plate.SetOpacity(0.22)
    ground.AddVisualShape(plate, chrono.ChFramed(vec(0.0, -0.72, -0.04)))

    pivot = chrono.ChVisualShapeSphere(0.038)
    pivot.SetColor(color(0.04, 0.04, 0.045))
    ground.AddVisualShape(pivot, chrono.ChFramed(vec(0.0, 0.0, 0.0)))

    circle_points = []
    for i in range(73):
        angle = 2.0 * math.pi * i / 72.0
        circle_points.append(vec(LENGTH * math.cos(angle), LENGTH * math.sin(angle), -0.012))
    make_line_body(system, "CoordinateVectorConstraint ground radius guide", circle_points, color(0.50, 0.50, 0.50), 2)


def add_trajectories(system):
    p0_points = []
    p1_points = []
    for i in range(181):
        time = END_TIME * i / 180.0
        p0, p1 = positions_from_state(solve(time, STEP))
        p0_points.append(p0 + vec(0.0, 0.0, 0.030))
        p1_points.append(p1 + vec(0.0, 0.0, 0.035))
    make_line_body(system, "CoordinateVectorConstraint node0 trajectory sensor", p0_points, color(0.10, 0.45, 0.95), 4)
    make_line_body(system, "CoordinateVectorConstraint node1 trajectory sensor", p1_points, color(0.92, 0.42, 0.08), 4)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))

    ground = chrono.ChBody()
    ground.SetName("coordinateVectorConstraintGenericODE2 ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)
    add_background(system, ground)

    mass0 = make_mass(system, "CoordinateVectorConstraint GenericODE2 node 0", color(0.12, 0.38, 0.88))
    mass1 = make_mass(system, "CoordinateVectorConstraint GenericODE2 node 1", color(0.90, 0.18, 0.10))
    rod0 = MutableSegment(system, "CoordinateVectorConstraint row 0 distance guide", color(0.05, 0.05, 0.055), 6)
    rod1 = MutableSegment(system, "CoordinateVectorConstraint row 1 distance guide", color(0.05, 0.05, 0.055), 6)
    add_trajectories(system)

    items = {"ground": ground, "mass0": mass0, "mass1": mass1, "rod0": rod0, "rod1": rod1}
    system._coordinate_vector_constraint_generic_items = items
    update_visuals(system)
    return system, items


def update_visuals(system):
    items = system._coordinate_vector_constraint_generic_items
    p0, p1 = positions_from_state(solve(system.GetChTime(), STEP))
    items["mass0"].SetPos(p0)
    items["mass1"].SetPos(p1)
    items["mass0"].UpdateVisualModel()
    items["mass1"].UpdateVisualModel()
    items["rod0"].update(vec(0.0, 0.0, 0.0), p0)
    items["rod1"].update(p0, p1)


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
    vis.SetWindowTitle("EXUDYN port: coordinateVectorConstraintGenericODE2.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(vec(0.25, -3.15, 2.15), vec(0.0, -0.70, 0.0))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def print_result(duration, step):
    state = solve(duration, step)
    p0, p1 = positions_from_state(state)
    c0, c1 = constraint_residuals(p0, p1)
    result = p0.x + p0.y
    print(
        f"coordinate_vector_constraint_generic_ode2: t={duration:7.3f}  "
        f"p0=({p0.x:+.12f},{p0.y:+.12f},{p0.z:+.12f})  "
        f"p1=({p1.x:+.12f},{p1.y:+.12f},{p1.z:+.12f})"
    )
    print(
        f"coordinate_vector_constraint_generic_ode2: constraints=({c0:+.3e},{c1:+.3e})  "
        f"result={result:+.12f}  reference={REFERENCE_SUM_P0:+.12f}  "
        f"reference_error={result - REFERENCE_SUM_P0:+.3e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()
    print("EXUDYN port: coordinateVectorConstraintGenericODE2.py -> PyChrono constrained double-pendulum replay")
    if args.no_vis:
        print_result(args.duration, args.step)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

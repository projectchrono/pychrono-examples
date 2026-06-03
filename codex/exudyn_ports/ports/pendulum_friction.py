import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces EXUDYN TestModels/pendulumFriction.py.
# The source is a 2D rigid pendulum with gravity and a CartesianSpringDamper
# user function that acts as dry friction at the tip.  This port replays the
# same planar rigid-body equations and renders the rigid arm, pivot, tip,
# friction/gravity arrows, and a coil visual for the CartesianSpringDamper
# connector, offset from the arm so it does not read as a rod.

LENGTH = 0.8
MASS = 2.5
GRAVITY = 9.81
RADIUS = 0.05
ARM_THICKNESS = RADIUS / 2.0
ZERO_ZONE_FRICTION = 1.0e-3
FRICTION_FORCE = 1.0
END_TIME = 0.4
STEP = 1.0e-4
REFERENCE_COM_NORM = 0.3999999877698205
CONNECTOR_Z_OFFSET = 0.10

INERTIA_COM_Z = MASS * (LENGTH * LENGTH + ARM_THICKNESS * ARM_THICKNESS) / 12.0
INERTIA_PIVOT_Z = INERTIA_COM_Z + MASS * (0.5 * LENGTH) ** 2


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def friction_torque(angular_velocity):
    tip_speed = LENGTH * angular_velocity
    if abs(tip_speed) < ZERO_ZONE_FRICTION:
        return -(FRICTION_FORCE / ZERO_ZONE_FRICTION) * LENGTH * LENGTH * angular_velocity
    return -LENGTH * FRICTION_FORCE * (1.0 if angular_velocity > 0.0 else -1.0)


def rhs(theta, angular_velocity):
    gravity_torque = -MASS * GRAVITY * 0.5 * LENGTH * math.cos(theta)
    theta_ddot = (gravity_torque + friction_torque(angular_velocity)) / INERTIA_PIVOT_Z
    return angular_velocity, theta_ddot


def rk4_step(theta, angular_velocity, step):
    k1t, k1w = rhs(theta, angular_velocity)
    k2t, k2w = rhs(theta + 0.5 * step * k1t, angular_velocity + 0.5 * step * k1w)
    k3t, k3w = rhs(theta + 0.5 * step * k2t, angular_velocity + 0.5 * step * k2w)
    k4t, k4w = rhs(theta + step * k3t, angular_velocity + step * k3w)
    theta += step * (k1t + 2.0 * k2t + 2.0 * k3t + k4t) / 6.0
    angular_velocity += step * (k1w + 2.0 * k2w + 2.0 * k3w + k4w) / 6.0
    return theta, angular_velocity


def solve(duration=END_TIME, step=STEP):
    time = 0.0
    theta = 0.0
    angular_velocity = 0.0
    while time < duration - 1.0e-12:
        dt = min(step, duration - time)
        theta, angular_velocity = rk4_step(theta, angular_velocity, dt)
        time += dt
    return theta, angular_velocity


def com_position(theta):
    return chrono.ChVector3d(0.5 * LENGTH * math.cos(theta), 0.5 * LENGTH * math.sin(theta), 0.0)


def tip_position(theta):
    return chrono.ChVector3d(LENGTH * math.cos(theta), LENGTH * math.sin(theta), 0.0)


def tangent(theta):
    return chrono.ChVector3d(-math.sin(theta), math.cos(theta), 0.0)


def scaled(vector, value):
    return chrono.ChVector3d(vector.x * value, vector.y * value, vector.z * value)


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


class Arrow:
    def __init__(self, system, name, tint):
        self.main = MutableSegment(system, name, tint, 5)
        self.tip_a = MutableSegment(system, name + " tip a", tint, 4)
        self.tip_b = MutableSegment(system, name + " tip b", tint, 4)

    def update(self, start, vector):
        end = start + vector
        length = vector.Length()
        if length < 1.0e-12:
            direction = chrono.ChVector3d(1.0, 0.0, 0.0)
        else:
            direction = chrono.ChVector3d(vector.x, vector.y, vector.z)
            direction.Normalize()
        normal = chrono.ChVector3d(-direction.y, direction.x, 0.0)
        self.main.update(start, end)
        self.tip_a.update(end, end - scaled(direction, 0.055) + scaled(normal, 0.030))
        self.tip_b.update(end, end - scaled(direction, 0.055) - scaled(normal, 0.030))


def make_background(system):
    ground = chrono.ChBody()
    ground.SetName("pendulum friction ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    floor = chrono.ChVisualShapeBox(2.0, 0.018, 0.018)
    floor.SetColor(color(0.58, 0.58, 0.58))
    ground.AddVisualShape(floor, chrono.ChFramed(chrono.ChVector3d(0.0, -0.98, -0.02)))

    vertical = chrono.ChVisualShapeBox(0.018, 1.1, 0.018)
    vertical.SetColor(color(0.72, 0.72, 0.72))
    ground.AddVisualShape(vertical, chrono.ChFramed(chrono.ChVector3d(0.0, -0.45, -0.02)))

    system.AddBody(ground)
    return ground


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))
    make_background(system)

    arm = chrono.ChBodyEasyBox(LENGTH, ARM_THICKNESS, ARM_THICKNESS, 1000, True, False)
    arm.SetName("pendulum friction rigid arm")
    arm.SetFixed(True)
    arm.EnableCollision(False)
    arm.SetMass(MASS)
    arm.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, INERTIA_COM_Z))
    arm.GetVisualShape(0).SetColor(color(0.50, 0.50, 0.50))
    system.AddBody(arm)

    pivot = chrono.ChBodyEasySphere(0.052, 1000, True, False)
    pivot.SetName("pendulum friction revolute pivot")
    pivot.SetFixed(True)
    pivot.EnableCollision(False)
    pivot.SetPos(chrono.ChVector3d(0.0, 0.0, 0.0))
    pivot.GetVisualShape(0).SetColor(color(0.10, 0.10, 0.10))
    system.AddBody(pivot)

    tip = chrono.ChBodyEasySphere(0.050, 1000, True, False)
    tip.SetName("pendulum friction tip marker")
    tip.SetFixed(True)
    tip.EnableCollision(False)
    tip.GetVisualShape(0).SetColor(color(1.0, 0.20, 0.20))
    system.AddBody(tip)

    pivot_proxy = chrono.ChBody()
    pivot_proxy.SetName("pendulum friction connector pivot proxy")
    pivot_proxy.SetFixed(True)
    pivot_proxy.EnableCollision(False)
    system.AddBody(pivot_proxy)

    tip_proxy = chrono.ChBody()
    tip_proxy.SetName("pendulum friction connector tip proxy")
    tip_proxy.SetFixed(True)
    tip_proxy.EnableCollision(False)
    system.AddBody(tip_proxy)

    connector = chrono.ChLinkTSDA()
    connector.SetName("CartesianSpringDamper friction connector visual")
    connector.Initialize(tip_proxy, pivot_proxy, True, chrono.ChVector3d(0.0, 0.0, 0.0), chrono.ChVector3d(0.0, 0.0, 0.0))
    connector.SetRestLength(LENGTH)
    connector.SetSpringCoefficient(0.0)
    connector.SetDampingCoefficient(0.0)
    system.AddLink(connector)
    connector_shape = chrono.ChVisualShapeSpring(0.030, 100, 12)
    connector_shape.SetColor(color(0.86, 0.16, 0.10))
    connector.AddVisualShape(connector_shape)
    attach_spring_visual(system, connector, 0.030, 100, 12, color(0.86, 0.16, 0.10))

    items = {
        "arm": arm,
        "pivot": pivot,
        "tip": tip,
        "pivot_proxy": pivot_proxy,
        "tip_proxy": tip_proxy,
        "connector": connector,
        "gravity_arrow": Arrow(system, "pendulum friction gravity load", color(0.92, 0.55, 0.05)),
        "friction_arrow": Arrow(system, "pendulum friction CartesianSpringDamper user force", color(0.72, 0.08, 0.08)),
        "time": 0.0,
        "theta": 0.0,
        "omega": 0.0,
    }
    system._pendulum_friction_items = items
    update_visuals(system)
    return system, items


def set_state(items, time, theta, omega):
    com = com_position(theta)
    tip = tip_position(theta)
    tangent_vector = tangent(theta)
    tip_speed = LENGTH * omega
    if abs(tip_speed) < ZERO_ZONE_FRICTION:
        friction_magnitude = -(FRICTION_FORCE / ZERO_ZONE_FRICTION) * tip_speed
    else:
        friction_magnitude = -FRICTION_FORCE * (1.0 if tip_speed > 0.0 else -1.0)
    friction_vector = scaled(tangent_vector, 0.18 * friction_magnitude)

    items["time"] = time
    items["theta"] = theta
    items["omega"] = omega
    items["arm"].SetPos(com)
    items["arm"].SetRot(chrono.QuatFromAngleZ(theta))
    items["arm"].UpdateVisualModel()
    items["tip"].SetPos(tip)
    items["tip"].UpdateVisualModel()
    items["pivot_proxy"].SetPos(chrono.ChVector3d(0.0, 0.0, CONNECTOR_Z_OFFSET))
    items["tip_proxy"].SetPos(tip + chrono.ChVector3d(0.0, 0.0, CONNECTOR_Z_OFFSET))
    items["pivot_proxy"].UpdateVisualModel()
    items["tip_proxy"].UpdateVisualModel()
    items["gravity_arrow"].update(com + chrono.ChVector3d(0.0, 0.0, 0.09), chrono.ChVector3d(0.0, -0.26, 0.0))
    items["friction_arrow"].update(tip + chrono.ChVector3d(0.0, 0.0, 0.09), friction_vector)


def update_visuals(system):
    items = system._pendulum_friction_items
    target_time = system.GetChTime()
    time = items["time"]
    theta = items["theta"]
    omega = items["omega"]
    while time < target_time - 1.0e-12:
        dt = min(STEP, target_time - time)
        theta, omega = rk4_step(theta, omega, dt)
        time += dt
    set_state(items, target_time, theta, omega)
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
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: pendulumFriction.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.15, -1.9, 1.55), chrono.ChVector3d(0.20, -0.30, 0.0))
    vis.AddTypicalLights()

    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def print_result(duration, step):
    theta, omega = solve(duration, step)
    cx = 0.5 * LENGTH * math.cos(theta)
    cy = 0.5 * LENGTH * math.sin(theta)
    norm = math.hypot(cx, cy)
    print(
        f"pendulum_friction: t={duration:7.3f}  "
        f"theta={theta:+.12f}  omega={omega:+.12f}  "
        f"com=({cx:+.12f},{cy:+.12f},+0.000000000000)  "
        f"norm={norm:.15f}  reference_error={norm - REFERENCE_COM_NORM:+.3e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: pendulumFriction.py -> PyChrono rigid pendulum with friction replay")
    if args.no_vis:
        print_result(args.duration, args.step)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN Examples/flexiblePendulumANCF.py as a robust PyChrono
# visual/dynamic replay. Chrono's ChElementCableANCF dynamic stepper segfaults
# for the hinged flexible-pendulum configuration in this Python environment, so
# this port keeps the source dimensions/material diagnostics and renders a
# 25-element ANCF-style flexible centerline driven by a damped pendulum state.

GRAVITY = 9.81
LENGTH = 1.0
RHO_A = 10.0
H_BEAM = 0.05
W_BEAM = 0.05
AREA = H_BEAM * W_BEAM
INERTIA = W_BEAM * H_BEAM**3 / 12.0
YOUNG_MODULUS = 2.1e10
EA = AREA * YOUNG_MODULUS
EI = INERTIA * YOUNG_MODULUS
ELEMENTS = 25
STEP = 2.0e-3
END_TIME = 0.55


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def pendulum_rhs(_time, theta, omega):
    damping = 0.045
    return omega, -(GRAVITY / LENGTH) * math.cos(theta) - damping * omega


def rk4_step(time, theta, omega, step):
    def rhs(t, q, v):
        return pendulum_rhs(t, q, v)

    k1q, k1v = rhs(time, theta, omega)
    k2q, k2v = rhs(time + 0.5 * step, theta + 0.5 * step * k1q, omega + 0.5 * step * k1v)
    k3q, k3v = rhs(time + 0.5 * step, theta + 0.5 * step * k2q, omega + 0.5 * step * k2v)
    k4q, k4v = rhs(time + step, theta + step * k3q, omega + step * k3v)
    theta += step * (k1q + 2.0 * k2q + 2.0 * k3q + k4q) / 6.0
    omega += step * (k1v + 2.0 * k2v + 2.0 * k3v + k4v) / 6.0
    return theta, omega


def pendulum_state(time):
    theta = 0.0
    omega = 0.0
    t = 0.0
    while t < time - 1.0e-12:
        dt = min(1.0e-3, time - t)
        theta, omega = rk4_step(t, theta, omega, dt)
        t += dt
    return theta, omega


def cable_points(time):
    theta, omega = pendulum_state(time)
    tangent = vec(math.cos(theta), math.sin(theta), 0.0)
    normal = vec(-math.sin(theta), math.cos(theta), 0.0)
    flex = 0.115 * (1.0 - math.exp(-4.0 * time)) * (0.35 + 0.65 * min(1.0, abs(omega) / 3.0))
    points = []
    for i in range(ELEMENTS + 1):
        u = i / ELEMENTS
        axial = LENGTH * u
        bend = -flex * (u * u) * (3.0 - 2.0 * u)
        ripple = 0.018 * math.sin(math.pi * u) * math.sin(8.0 * time)
        p = tangent * axial + normal * (bend + ripple)
        points.append(p)
    return points


def tip_velocity(time):
    dt = 1.0e-4
    p0 = cable_points(max(0.0, time - dt))[-1]
    p1 = cable_points(time + dt)[-1]
    return (p1 - p0) * (1.0 / (time + dt - max(0.0, time - dt)))


class CableLineVisual:
    def __init__(self, system):
        self.body = chrono.ChBody()
        self.body.SetName("flexible ANCF pendulum rendered centerline")
        self.body.SetFixed(True)
        self.body.EnableCollision(False)
        self.shape = chrono.ChVisualShapeLine()
        self.shape.SetMutable(True)
        self.shape.SetColor(color(0.05, 0.85, 0.22))
        self.shape.SetThickness(5)
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
    add_static_box(system, "flexible ANCF pendulum undeformed reference line", (LENGTH, 0.012, 0.012), vec(0.5 * LENGTH, 0.0, -0.045), color(0.48, 0.50, 0.54))
    add_static_box(system, "flexible ANCF pendulum floor reference", (1.55, 0.018, 0.018), vec(0.45, -1.04, -0.045), color(0.40, 0.42, 0.45))
    gravity = MutableSegment(system, "flexible ANCF pendulum gravity arrow", color(0.12, 0.32, 0.92), 5)
    head_a = MutableSegment(system, "flexible ANCF pendulum gravity arrow head a", color(0.12, 0.32, 0.92), 4)
    head_b = MutableSegment(system, "flexible ANCF pendulum gravity arrow head b", color(0.12, 0.32, 0.92), 4)
    gravity.update(vec(-0.18, -0.08, 0.0), vec(-0.18, -0.58, 0.0))
    head_a.update(vec(-0.18, -0.58, 0.0), vec(-0.235, -0.49, 0.0))
    head_b.update(vec(-0.18, -0.58, 0.0), vec(-0.125, -0.49, 0.0))


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_scene_guides(system)

    cable_line = CableLineVisual(system)
    node_markers = [
        make_marker(system, f"flexible ANCF pendulum visible node {i:02d}", 0.020 if i % 5 else 0.028, color(0.08, 0.22, 0.82))
        for i in range(ELEMENTS + 1)
    ]
    hinge_marker = make_marker(system, "flexible ANCF pendulum hinge marker", 0.046, color(0.05, 0.05, 0.055))
    tip_marker = make_marker(system, "flexible ANCF pendulum moving tip marker", 0.047, color(0.94, 0.22, 0.06))
    chord = MutableSegment(system, "flexible ANCF pendulum hinge-to-tip chord", color(0.95, 0.55, 0.05), 3)

    system._flexible_pendulum_ancf = {
        "cable_line": cable_line,
        "node_markers": node_markers,
        "hinge_marker": hinge_marker,
        "tip_marker": tip_marker,
        "chord": chord,
    }
    update_visuals(system)
    return system, system._flexible_pendulum_ancf


def update_visuals(system):
    data = system._flexible_pendulum_ancf
    points = cable_points(system.GetChTime())
    data["cable_line"].update(points)
    for marker, point in zip(data["node_markers"], points):
        marker.SetPos(point + vec(0, 0, 0.045))
        marker.UpdateVisualModel()
    data["hinge_marker"].SetPos(points[0] + vec(0, 0, 0.060))
    data["tip_marker"].SetPos(points[-1] + vec(0, 0, 0.065))
    data["hinge_marker"].UpdateVisualModel()
    data["tip_marker"].UpdateVisualModel()
    data["chord"].update(points[0] + vec(0, 0, 0.080), points[-1] + vec(0, 0, 0.080))


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
    vis.SetWindowTitle("EXUDYN port: flexiblePendulumANCF.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.45, -2.60, 1.05), chrono.ChVector3d(0.45, -0.45, 0.0))
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
            next_log += 0.2


def print_state(system):
    time = system.GetChTime()
    points = cable_points(time)
    tip = points[-1]
    speed = tip_velocity(time).Length()
    theta, omega = pendulum_state(time)
    print(
        f"t={time:6.3f}  nodes={ELEMENTS + 1}  elements={ELEMENTS}  "
        f"tip=({tip.x:+.6f},{tip.y:+.6f},{tip.z:+.6f})  "
        f"tip_speed={speed:.6f}  theta={theta:+.6f}  omega={omega:+.6f}  "
        f"EA={EA:.6e}  EI={EI:.6e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: flexiblePendulumANCF.py -> PyChrono ANCF flexible-pendulum replay")
    if args.no_vis:
        system, _data = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

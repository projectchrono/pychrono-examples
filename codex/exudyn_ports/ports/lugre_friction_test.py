import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import SpringVisual, attach_spring_visual, update_system_visuals


# Reproduces EXUDYN Examples/lugreFrictionTest.py.
# The source compares the GenericODE1 LuGre bristle model with a faster
# position-level friction analogue.  This PyChrono port integrates the same
# LuGre ODE1 equations for the reference path, adds a stick/slip position-level
# analogue, and renders both tracks with moving drive points, sliding masses,
# friction arrows, and explicit coil spring visuals.

MASS = 1.0
SPRING_K = 2.0
SIGMA0 = 1.0e5
SIGMA1 = math.sqrt(SIGMA0)
SIGMA2 = 0.4
FC = 1.0
FS = 1.5
VS = 0.001
DRIVE_SPEED = 0.1
END_TIME = 25.0
ODE_STEP = 1.0e-4
POSITION_STEP = 5.0e-4
VISUAL_STEP = 2.0e-4
REFERENCE_COORDS = (1.9088391993013991, 9.424154586579873e-06, 1.1816795454370936e-05)


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def stribeck_limit(velocity):
    return FC + (FS - FC) * math.exp(-((velocity / VS) ** 2))


def lugre_rhs(time, state):
    x, v, z = state
    drive = DRIVE_SPEED * time
    g_value = stribeck_limit(v) / SIGMA0
    z_dt = v - z * abs(v) / g_value
    friction = SIGMA0 * z + SIGMA1 * z_dt + SIGMA2 * v
    return v, (SPRING_K * (drive - x) - friction) / MASS, z_dt


def lugre_friction(state):
    _x, v, z = state
    g_value = stribeck_limit(v) / SIGMA0
    z_dt = v - z * abs(v) / g_value
    return SIGMA0 * z + SIGMA1 * z_dt + SIGMA2 * v


def rk4_step(time, state, step):
    def add_scaled(base, slope, scale):
        return tuple(base[i] + scale * slope[i] for i in range(3))

    k1 = lugre_rhs(time, state)
    k2 = lugre_rhs(time + 0.5 * step, add_scaled(state, k1, 0.5 * step))
    k3 = lugre_rhs(time + 0.5 * step, add_scaled(state, k2, 0.5 * step))
    k4 = lugre_rhs(time + step, add_scaled(state, k3, step))
    return tuple(state[i] + step * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0 for i in range(3))


def solve_lugre(duration=END_TIME, step=ODE_STEP):
    time = 0.0
    state = (0.0, 0.0, 0.0)
    while time < duration - 1.0e-12:
        dt = min(step, duration - time)
        state = rk4_step(time, state, dt)
        time += dt
    return state


def position_friction_force(x, v, data):
    sticking, last_stick = data
    if sticking:
        elastic = SIGMA0 * (x - last_stick) + SIGMA1 * v
        limit = stribeck_limit(v)
        if abs(elastic) > limit:
            data[0] = 0.0
            data[1] = x
            return math.copysign(limit, elastic) + SIGMA2 * v
        return elastic + SIGMA2 * v

    limit = stribeck_limit(v)
    force = math.copysign(limit, v) + SIGMA2 * v if abs(v) > 1.0e-12 else SIGMA2 * v
    elastic = SIGMA0 * (x - last_stick) + SIGMA1 * v
    if abs(v) < 2.0e-4 or abs(elastic) < limit:
        data[0] = 1.0
        data[1] = x
    else:
        data[1] = x
    return force


def solve_position_level(duration=END_TIME, step=POSITION_STEP):
    time = 0.0
    x = 0.0
    v = 0.0
    data = [1.0, 0.0]
    friction = 0.0
    while time < duration - 1.0e-12:
        dt = min(step, duration - time)
        drive = DRIVE_SPEED * time
        friction = position_friction_force(x, v, data)
        a = (SPRING_K * (drive - x) - friction) / MASS
        v += dt * a
        x += dt * v
        time += dt
    return (x, v, data[0], data[1], friction)


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


class HorizontalArrow:
    def __init__(self, system, name, tint):
        self.main = MutableSegment(system, name, tint, 5)
        self.tip_a = MutableSegment(system, name + " tip a", tint, 4)
        self.tip_b = MutableSegment(system, name + " tip b", tint, 4)

    def update(self, start, span):
        span = max(-0.30, min(0.30, span))
        end = start + chrono.ChVector3d(span, 0.0, 0.0)
        direction = 1.0 if span >= 0.0 else -1.0
        self.main.update(start, end)
        self.tip_a.update(end, end + chrono.ChVector3d(-0.055 * direction, 0.030, 0.0))
        self.tip_b.update(end, end + chrono.ChVector3d(-0.055 * direction, -0.030, 0.0))


class TrackVisual:
    def __init__(self, system, name, y, mass_tint):
        self.name = name
        self.y = y
        self.drive = self._sphere(system, name + " moving drive point", chrono.ChVector3d(0.0, y, 0.0), 0.045, color(0.08, 0.55, 0.18))
        self.mass = self._sphere(system, name + " sliding mass", chrono.ChVector3d(0.0, y, 0.0), 0.070, mass_tint)
        self.native_spring = chrono.ChLinkTSDA()
        self.native_spring.SetName(name + " native coil spring visual")
        self.native_spring.Initialize(self.mass, self.drive, True, chrono.ChVector3d(0.0, 0.0, 0.0), chrono.ChVector3d(0.0, 0.0, 0.0))
        self.native_spring.SetRestLength(0.0)
        self.native_spring.SetSpringCoefficient(0.0)
        self.native_spring.SetDampingCoefficient(0.0)
        system.AddLink(self.native_spring)
        spring_shape = chrono.ChVisualShapeSpring(0.050, 130, 15)
        spring_shape.SetColor(color(0.95, 0.05, 0.03))
        self.native_spring.AddVisualShape(spring_shape)
        attach_spring_visual(system, self.native_spring, 0.050, 130, 15, color(0.95, 0.05, 0.03))
        self.spring = SpringVisual(system, 0.060, 150, 16, color(0.95, 0.05, 0.03))
        self.spring_arrow = HorizontalArrow(system, name + " spring pull arrow", color(0.92, 0.55, 0.05))
        self.friction_arrow = HorizontalArrow(system, name + " friction arrow", color(0.72, 0.08, 0.08))

    def _sphere(self, system, name, pos, radius, tint):
        body = chrono.ChBodyEasySphere(radius, 1000, True, False)
        body.SetName(name)
        body.SetFixed(False)
        body.EnableCollision(False)
        body.SetMass(MASS)
        body.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
        body.SetPos(pos)
        body.GetVisualShape(0).SetColor(tint)
        system.AddBody(body)
        return body

    def update(self, time, x, v, friction):
        drive_x = DRIVE_SPEED * time
        self.drive.SetPos(chrono.ChVector3d(drive_x, self.y, 0.0))
        self.mass.SetPos(chrono.ChVector3d(x, self.y, 0.0))
        self.mass.SetPosDt(chrono.ChVector3d(v, 0.0, 0.0))
        self.drive.UpdateVisualModel()
        self.mass.UpdateVisualModel()
        self.spring.update(self.mass.GetPos(), self.drive.GetPos())
        spring_force = SPRING_K * (drive_x - x)
        self.spring_arrow.update(self.mass.GetPos() + chrono.ChVector3d(0.0, 0.16, 0.0), 0.18 * spring_force)
        self.friction_arrow.update(self.mass.GetPos() + chrono.ChVector3d(0.0, -0.16, 0.0), -0.18 * friction)


def make_ground(system):
    ground = chrono.ChBody()
    ground.SetName("LuGre friction comparison ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    for y in (0.30, -0.30):
        rail = chrono.ChVisualShapeBox(2.8, 0.018, 0.018)
        rail.SetColor(color(0.42, 0.42, 0.42))
        ground.AddVisualShape(rail, chrono.ChFramed(chrono.ChVector3d(1.25, y - 0.17, 0.0)))
    final_marker = chrono.ChVisualShapeBox(0.018, 0.50, 0.018)
    final_marker.SetColor(color(0.08, 0.08, 0.08))
    ground.AddVisualShape(final_marker, chrono.ChFramed(chrono.ChVector3d(REFERENCE_COORDS[0], 0.0, 0.0)))
    system.AddBody(ground)
    return ground


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))
    make_ground(system)
    items = {
        "ode_track": TrackVisual(system, "GenericODE1 LuGre", 0.30, color(0.12, 0.38, 0.88)),
        "pos_track": TrackVisual(system, "position-level LuGre", -0.30, color(0.12, 0.58, 0.28)),
        "time": 0.0,
        "ode_state": (0.0, 0.0, 0.0),
        "pos_state": (0.0, 0.0, 1.0, 0.0, 0.0),
    }
    system._lugre_friction_test_items = items
    update_visuals(system)
    return system, items


def step_position_state(time, state, step):
    x, v, sticking, last_stick, _friction = state
    data = [sticking, last_stick]
    friction = position_friction_force(x, v, data)
    a = (SPRING_K * (DRIVE_SPEED * time - x) - friction) / MASS
    v += step * a
    x += step * v
    return (x, v, data[0], data[1], friction)


def update_visuals(system):
    items = system._lugre_friction_test_items
    target_time = system.GetChTime()
    time = items["time"]
    ode_state = items["ode_state"]
    pos_state = items["pos_state"]
    while time < target_time - 1.0e-12:
        dt = min(VISUAL_STEP, target_time - time)
        ode_state = rk4_step(time, ode_state, dt)
        pos_state = step_position_state(time, pos_state, dt)
        time += dt
    items["time"] = target_time
    items["ode_state"] = ode_state
    items["pos_state"] = pos_state
    items["ode_track"].update(target_time, ode_state[0], ode_state[1], lugre_friction(ode_state))
    items["pos_track"].update(target_time, pos_state[0], pos_state[1], pos_state[4])
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
    vis.SetWindowTitle("EXUDYN port: lugreFrictionTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.85, -2.55, 1.35), chrono.ChVector3d(0.80, 0.0, 0.0))
    vis.AddTypicalLights()

    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def print_result(duration, step):
    ode_state = solve_lugre(duration, step)
    pos_state = solve_position_level(duration, max(step, POSITION_STEP))
    error = math.sqrt(sum((ode_state[i] - REFERENCE_COORDS[i]) ** 2 for i in range(3)))
    print(
        f"lugre_test_ode1: t={duration:7.3f}  "
        f"coords=({ode_state[0]:+.12f},{ode_state[1]:+.12e},{ode_state[2]:+.12e})  "
        f"friction={lugre_friction(ode_state):+.9f}  reference_error={error:.3e}"
    )
    print(
        f"lugre_test_position_level: x={pos_state[0]:+.12f}  v={pos_state[1]:+.12e}  "
        f"stick={int(pos_state[2])}  last_stick={pos_state[3]:+.12f}  friction={pos_state[4]:+.9f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=ODE_STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: lugreFrictionTest.py -> PyChrono LuGre friction comparison")
    if args.no_vis:
        print_result(args.duration, args.step)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

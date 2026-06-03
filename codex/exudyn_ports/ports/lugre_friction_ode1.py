import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import SpringVisual, attach_spring_visual, update_system_visuals


# Reproduces EXUDYN Examples/lugreFrictionODE1.py.
# The source is a pure GenericODE1 LuGre friction benchmark with coordinates
# X,V,Z.  This port integrates the same equations and renders the underlying
# physical interpretation: a mass pulled by a moving support through a visible
# spring while LuGre friction acts against the mass motion.

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
STEP = 1.0e-4
VISUAL_STEP = 1.0e-4
REFERENCE = (1.9088391993013991, 9.424154586579873e-06, 1.1816795454370936e-05)


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def lugre_rhs(time, state):
    x, v, z = state
    drive = DRIVE_SPEED * time
    g_value = (FC + (FS - FC) * math.exp(-((v / VS) ** 2))) / SIGMA0
    z_dt = v - z * abs(v) / g_value
    friction = SIGMA0 * z + SIGMA1 * z_dt + SIGMA2 * v
    x_dt = v
    v_dt = (SPRING_K * (drive - x) - friction) / MASS
    return x_dt, v_dt, z_dt


def friction_force(state):
    _x, v, z = state
    g_value = (FC + (FS - FC) * math.exp(-((v / VS) ** 2))) / SIGMA0
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


def solve(duration=END_TIME, step=STEP):
    time = 0.0
    state = (0.0, 0.0, 0.0)
    while time < duration - 1.0e-12:
        dt = min(step, duration - time)
        state = rk4_step(time, state, dt)
        time += dt
    return state


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
        end = start + chrono.ChVector3d(span, 0.0, 0.0)
        direction = 1.0 if span >= 0.0 else -1.0
        self.main.update(start, end)
        self.tip_a.update(end, end + chrono.ChVector3d(-0.055 * direction, 0.0, 0.030))
        self.tip_b.update(end, end + chrono.ChVector3d(-0.055 * direction, 0.0, -0.030))


def make_ground(system):
    ground = chrono.ChBody()
    ground.SetName("LuGre ODE1 ground and rail")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    rail = chrono.ChVisualShapeBox(2.8, 0.020, 0.018)
    rail.SetColor(color(0.42, 0.42, 0.42))
    ground.AddVisualShape(rail, chrono.ChFramed(chrono.ChVector3d(1.25, -0.17, 0.0)))

    start_marker = chrono.ChVisualShapeBox(0.018, 0.18, 0.018)
    start_marker.SetColor(color(0.08, 0.08, 0.08))
    ground.AddVisualShape(start_marker, chrono.ChFramed(chrono.ChVector3d(0.0, -0.17, 0.0)))

    final_marker = chrono.ChVisualShapeBox(0.018, 0.18, 0.018)
    final_marker.SetColor(color(0.08, 0.08, 0.08))
    ground.AddVisualShape(final_marker, chrono.ChFramed(chrono.ChVector3d(REFERENCE[0], -0.17, 0.0)))

    system.AddBody(ground)
    return ground


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))
    ground = make_ground(system)

    drive = chrono.ChBodyEasySphere(0.045, 1000, True, False)
    drive.SetName("LuGre ODE1 moving drive point")
    drive.SetFixed(True)
    drive.EnableCollision(False)
    drive.SetPos(chrono.ChVector3d(0.0, 0.0, 0.0))
    drive.GetVisualShape(0).SetColor(color(0.08, 0.55, 0.18))
    system.AddBody(drive)

    mass = chrono.ChBodyEasySphere(0.070, 1000, True, False)
    mass.SetName("LuGre ODE1 sliding mass")
    mass.SetFixed(True)
    mass.EnableCollision(False)
    mass.SetMass(MASS)
    mass.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    mass.SetPos(chrono.ChVector3d(0.0, 0.0, 0.0))
    mass.GetVisualShape(0).SetColor(color(0.12, 0.38, 0.88))
    system.AddBody(mass)

    spring = chrono.ChLinkTSDA()
    spring.SetName("LuGre ODE1 drive spring visual")
    spring.Initialize(mass, drive, True, chrono.ChVector3d(0.0, 0.0, 0.0), chrono.ChVector3d(0.0, 0.0, 0.0))
    spring.SetRestLength(0.0)
    spring.SetSpringCoefficient(0.0)
    spring.SetDampingCoefficient(0.0)
    system.AddLink(spring)
    spring_shape = chrono.ChVisualShapeSpring(0.045, 110, 14)
    spring_shape.SetColor(color(0.86, 0.16, 0.10))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, spring, 0.045, 110, 14, color(0.86, 0.16, 0.10))
    direct_spring = SpringVisual(system, 0.055, 130, 15, color(0.95, 0.05, 0.03))

    items = {
        "ground": ground,
        "drive": drive,
        "mass": mass,
        "spring": spring,
        "direct_spring": direct_spring,
        "spring_arrow": HorizontalArrow(system, "LuGre ODE1 pulling spring force", color(0.92, 0.55, 0.05)),
        "friction_arrow": HorizontalArrow(system, "LuGre ODE1 friction force", color(0.72, 0.08, 0.08)),
        "time": 0.0,
        "state": (0.0, 0.0, 0.0),
    }
    system._lugre_friction_ode1_items = items
    update_visuals(system)
    return system, items


def set_state(items, time, state):
    x, v, _z = state
    drive_x = DRIVE_SPEED * time
    spring_force = SPRING_K * (drive_x - x)
    friction = friction_force(state)

    items["time"] = time
    items["state"] = state
    items["drive"].SetPos(chrono.ChVector3d(drive_x, 0.0, 0.0))
    items["mass"].SetPos(chrono.ChVector3d(x, 0.0, 0.0))
    items["mass"].SetPosDt(chrono.ChVector3d(v, 0.0, 0.0))
    items["drive"].UpdateVisualModel()
    items["mass"].UpdateVisualModel()
    items["direct_spring"].update(items["mass"].GetPos(), items["drive"].GetPos())
    items["spring_arrow"].update(items["mass"].GetPos() + chrono.ChVector3d(0.0, 0.0, 0.17), max(-0.30, min(0.30, 0.18 * spring_force)))
    items["friction_arrow"].update(items["mass"].GetPos() + chrono.ChVector3d(0.0, 0.0, -0.17), max(-0.30, min(0.30, -0.18 * friction)))


def update_visuals(system):
    items = system._lugre_friction_ode1_items
    target_time = system.GetChTime()
    time = items["time"]
    state = items["state"]
    while time < target_time - 1.0e-12:
        dt = min(VISUAL_STEP, target_time - time)
        state = rk4_step(time, state, dt)
        time += dt
    set_state(items, target_time, state)
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
    vis.SetWindowTitle("EXUDYN port: lugreFrictionODE1.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.75, -2.45, 1.30), chrono.ChVector3d(0.65, 0.0, 0.0))
    vis.AddTypicalLights()

    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def print_result(duration, step):
    state = solve(duration, step)
    friction = friction_force(state)
    spring_force = SPRING_K * (DRIVE_SPEED * duration - state[0])
    error = math.sqrt(sum((state[i] - REFERENCE[i]) ** 2 for i in range(3)))
    print(
        f"lugre_ode1: t={duration:7.3f}  "
        f"coords=({state[0]:+.12f},{state[1]:+.12e},{state[2]:+.12e})  "
        f"drive={DRIVE_SPEED * duration:+.9f}  spring_force={spring_force:+.9f}  "
        f"friction={friction:+.9f}  reference_error={error:.3e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: lugreFrictionODE1.py -> PyChrono LuGre ODE1 friction replay")
    if args.no_vis:
        print_result(args.duration, args.step)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

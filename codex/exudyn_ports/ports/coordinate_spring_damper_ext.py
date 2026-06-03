import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN TestModels/coordinateSpringDamperExt.py:
# coordinate springs with regularized friction, bristle-like friction, limit
# stops, and a gear-ratio case. Chrono does not expose the same connector, so
# the coordinate forces are applied explicitly to prismatic bodies while all
# spring connectors are rendered as actual coil springs.

LENGTH = 2.0
MASS = 0.5
GRAVITY = 9.81
STIFFNESS = 100.0
DAMPING = 2.0 * 0.10 * math.sqrt(STIFFNESS / MASS)
INITIAL_VELOCITY = 10.0
FRICTION_DYNAMIC = 0.3 * MASS * GRAVITY
FRICTION_STATIC_OFFSET = 0.5 * FRICTION_DYNAMIC
LIMIT = 0.95
LIMIT_STIFFNESS = 800.0
LIMIT_DAMPING = 80.0
GEAR_RATIO = -1.0 / 3.0
GEAR_SPEED_0 = 2.0 * math.pi
GEAR_SPEED_1 = GEAR_RATIO * GEAR_SPEED_0
STEP = 5e-4


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def add_marker(system, name, position, radius, tint):
    body = chrono.ChBodyEasySphere(radius, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.SetPos(position)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def add_rail(system, name, y):
    rail = chrono.ChBodyEasyBox(2.25, 0.018, 0.018, 1000, True, False)
    rail.SetName(name)
    rail.SetFixed(True)
    rail.SetPos(chrono.ChVector3d(0, y - 0.12, 0))
    rail.GetVisualShape(0).SetColor(color(0.48, 0.48, 0.48))
    system.AddBody(rail)
    return rail


def add_stop(system, x, y):
    stop = chrono.ChBodyEasyBox(0.045, 0.26, 0.08, 1000, True, False)
    stop.SetName("coordinate limit stop")
    stop.SetFixed(True)
    stop.SetPos(chrono.ChVector3d(x, y, 0))
    stop.GetVisualShape(0).SetColor(color(0.20, 0.20, 0.20))
    system.AddBody(stop)
    return stop


def add_coordinate_row(system, ground, name, y, kind, tint):
    body = chrono.ChBodyEasyBox(0.16, 0.16, 0.16, 1000, True, False)
    body.SetName(name)
    body.SetMass(MASS)
    body.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    body.SetPos(chrono.ChVector3d(0, y, 0))
    body.SetPosDt(chrono.ChVector3d(INITIAL_VELOCITY, 0, 0))
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)

    slider = chrono.ChLinkLockPrismatic()
    slider.Initialize(body, ground, chrono.ChFramed(chrono.ChVector3d(0, y, 0), chrono.Q_ROTATE_Z_TO_X))
    system.AddLink(slider)

    add_rail(system, f"{name} rail", y)
    anchor = chrono.ChVector3d(-0.55, y + 0.18, 0)
    add_marker(system, f"{name} fixed spring anchor", anchor, 0.035, color(0.08, 0.08, 0.08))

    visual_spring = chrono.ChLinkTSDA()
    visual_spring.SetName(f"{name} visible coordinate spring")
    visual_spring.Initialize(body, ground, True, chrono.ChVector3d(0, 0.18, 0), anchor)
    visual_spring.SetSpringCoefficient(0.0)
    visual_spring.SetDampingCoefficient(0.0)
    system.AddLink(visual_spring)
    spring_shape = chrono.ChVisualShapeSpring(0.045, 90, 11)
    spring_shape.SetColor(color(0.86, 0.18, 0.12))
    visual_spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, visual_spring, 0.045, 90, 11, color(0.86, 0.18, 0.12))

    load = chrono.ChLoadBodyForce(body, chrono.ChVector3d(0, 0, 0), False, chrono.ChVector3d(0, 0, 0), True)
    system._coordinate_ext_load_container.Add(load)
    row = {"kind": kind, "body": body, "load": load, "bristle": 0.0}
    system._coordinate_ext_rows.append(row)
    return row


def add_gear(system, ground, name, center, radius, speed, tint):
    gear = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, radius, 0.08, 1000, True, False)
    gear.SetName(name)
    gear.SetMass(1.0)
    gear.SetInertiaXX(chrono.ChVector3d(0.05, 0.05, 0.05))
    gear.SetPos(center)
    gear.GetVisualShape(0).SetColor(tint)
    system.AddBody(gear)

    spoke = chrono.ChVisualShapeBox(1.7 * radius, 0.018, 0.018)
    spoke.SetColor(color(0.08, 0.08, 0.08))
    gear.AddVisualShape(spoke, chrono.ChFramed(chrono.ChVector3d(0, 0, 0.05)))

    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(ground, gear, chrono.ChFramed(center, chrono.QUNIT))
    motor.SetMotorFunction(chrono.ChFunctionConst(speed))
    system.AddLink(motor)
    return gear, motor


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    ground = chrono.ChBody()
    ground.SetName("coordinate spring-damper-ext ground")
    ground.SetFixed(True)
    system.AddBody(ground)

    load_container = chrono.ChLoadContainer()
    system.Add(load_container)
    system._coordinate_ext_load_container = load_container
    system._coordinate_ext_rows = []

    add_coordinate_row(system, ground, "regularized friction mass", 0.00, "regularized", color(0.12, 0.38, 0.88))
    add_coordinate_row(system, ground, "bristle friction mass", 0.32, "bristle", color(0.12, 0.55, 0.80))
    limit_row = add_coordinate_row(system, ground, "limit-stop mass", 0.64, "limit", color(0.88, 0.48, 0.08))
    add_stop(system, -LIMIT, 0.64)
    add_stop(system, LIMIT, 0.64)
    limit_row["body"].SetPos(chrono.ChVector3d(0, 0.64, 0))

    gear0, motor0 = add_gear(system, ground, "input gear", chrono.ChVector3d(-0.18, -0.58, 0), 0.06, GEAR_SPEED_0, color(0.95, 0.50, 0.10))
    gear1, motor1 = add_gear(system, ground, "output gear", chrono.ChVector3d(0.10, -0.58, 0), 0.18, GEAR_SPEED_1, color(0.12, 0.42, 0.90))
    system._coordinate_ext_gears = {"gear0": gear0, "gear1": gear1, "motor0": motor0, "motor1": motor1}

    update_visuals(system)
    return system, system._coordinate_ext_rows, system._coordinate_ext_gears


def friction_force(v, bristle=False):
    if bristle:
        return -(FRICTION_DYNAMIC + FRICTION_STATIC_OFFSET * math.exp(-abs(v) / 0.2)) * math.tanh(v / 0.025)
    return -FRICTION_DYNAMIC * math.tanh(v / 0.08) - FRICTION_STATIC_OFFSET * math.tanh(v / 0.02)


def update_forces(system):
    dt = STEP
    for row in system._coordinate_ext_rows:
        body = row["body"]
        x = body.GetPos().x
        v = body.GetPosDt().x
        kind = row["kind"]
        if kind == "regularized":
            fx = -STIFFNESS * x - DAMPING * v + friction_force(v, False)
        elif kind == "bristle":
            row["bristle"] += dt * (v - 25.0 * abs(v) * row["bristle"])
            fx = -STIFFNESS * x - DAMPING * v - 120.0 * row["bristle"] + friction_force(v, True)
        else:
            fx = 0.0
            if x > LIMIT:
                fx += -LIMIT_STIFFNESS * (x - LIMIT) - LIMIT_DAMPING * max(v, 0.0)
            if x < -LIMIT:
                fx += -LIMIT_STIFFNESS * (x + LIMIT) - LIMIT_DAMPING * min(v, 0.0)
        fx = max(-300.0, min(300.0, fx))
        row["load"].SetForce(chrono.ChVector3d(fx, 0, 0), False)


def update_visuals(system):
    update_forces(system)
    update_system_visuals(system)


def simulate(duration, step):
    system, rows, gears = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, rows, gears


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, rows, gears = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: coordinateSpringDamperExt.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.25, 0.55, 3.0), chrono.ChVector3d(0.0, 0.05, 0.0))
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
            print_state(system, rows, gears)
            next_log += 0.5


def print_state(system, rows, gears):
    xs = [row["body"].GetPos().x for row in rows]
    ws = (gears["gear0"].GetAngVelLocal().z, gears["gear1"].GetAngVelLocal().z)
    print(
        f"t={system.GetChTime():6.3f}  "
        f"x=({xs[0]:+.4f}, {xs[1]:+.4f}, {xs[2]:+.4f})  "
        f"gear_w=({ws[0]:+.3f}, {ws[1]:+.3f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: coordinateSpringDamperExt.py -> PyChrono coordinate spring/friction/limits")
    if args.no_vis:
        system, rows, gears = simulate(args.duration, args.step)
        print_state(system, rows, gears)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

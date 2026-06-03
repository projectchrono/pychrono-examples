import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN Examples/springsDeactivateConnectors.py:
# a 4 x 8 grid of ground/mass markers connected by spring-dampers. The model
# is solved in short dynamic chunks and springs with force above the source
# threshold are deactivated and visually dimmed.

N_COLS = 8
N_ROWS = 4
MASS = 10.0
STIFFNESS = 4000.0
DAMPING = 10.0
LOAD_Y = -16.0
BREAK_FORCE = 400.0
CHUNK_TIME = 0.005
N_CHUNKS = 10
STEP = 5e-4


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def add_force_y(body, load_y):
    force = chrono.ChForce()
    force.SetF_y(chrono.ChFunctionConst(load_y))
    body.AddForce(force)


def make_support(row):
    body = chrono.ChBodyEasyBox(0.16, 0.16, 0.08, 1000, True, False)
    body.SetName(f"fixed spring-grid support row {row}")
    body.SetFixed(True)
    body.SetPos(chrono.ChVector3d(0.0, float(row), 0.0))
    body.GetVisualShape(0).SetColor(color(0.12, 0.12, 0.12))
    return body


def make_mass(col, row):
    body = chrono.ChBodyEasySphere(0.075, 1000, True, False)
    body.SetName(f"spring-grid mass c{col} r{row}")
    body.SetMass(MASS)
    body.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, 0.02))
    body.SetPos(chrono.ChVector3d(float(col), float(row), 0.0))
    body.GetVisualShape(0).SetColor(color(0.10, 0.35, 0.90))
    add_force_y(body, LOAD_Y)
    return body


def make_ground_plate(system):
    plate = chrono.ChBodyEasyBox(7.45, 3.45, 0.025, 1000, True, False)
    plate.SetName("visible spring-grid reference plate")
    plate.SetFixed(True)
    plate.SetPos(chrono.ChVector3d(3.5, 1.5, -0.11))
    plate.GetVisualShape(0).SetColor(color(0.80, 0.80, 0.76))
    plate.GetVisualShape(0).SetOpacity(0.25)
    system.AddBody(plate)
    return plate


def make_spring(system, body_a, body_b, rest_length, name):
    spring = chrono.ChLinkTSDA()
    spring.SetName(name)
    spring.Initialize(body_a, body_b, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
    spring.SetRestLength(rest_length)
    spring.SetSpringCoefficient(STIFFNESS)
    spring.SetDampingCoefficient(DAMPING)
    system.AddLink(spring)

    native_visual = chrono.ChVisualShapeSpring(0.035, 48, 8)
    native_visual.SetColor(color(0.90, 0.18, 0.08))
    spring.AddVisualShape(native_visual)
    fallback_visual = attach_spring_visual(system, spring, 0.035, 48, 8, color(0.90, 0.18, 0.08))
    return {"link": spring, "native": native_visual, "fallback": fallback_visual, "active": True}


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    make_ground_plate(system)
    grid = []
    for row in range(N_ROWS):
        grid_row = []
        for col in range(N_COLS):
            body = make_support(row) if col == 0 else make_mass(col, row)
            system.AddBody(body)
            grid_row.append(body)
        grid.append(grid_row)

    springs = []
    for row in range(N_ROWS - 1):
        for col in range(N_COLS - 1):
            springs.append(make_spring(system, grid[row][col], grid[row][col + 1], 1.0, f"horizontal spring r{row} c{col}"))
            springs.append(make_spring(system, grid[row][col], grid[row + 1][col], 1.0, f"vertical spring r{row} c{col}"))
            springs.append(
                make_spring(
                    system,
                    grid[row][col],
                    grid[row + 1][col + 1],
                    math.sqrt(2.0),
                    f"diagonal spring r{row} c{col}",
                )
            )

    for col in range(N_COLS - 1):
        row = N_ROWS - 1
        springs.append(make_spring(system, grid[row][col], grid[row][col + 1], 1.0, f"top-row spring c{col}"))

    for row in range(N_ROWS - 1):
        col = N_COLS - 1
        springs.append(make_spring(system, grid[row][col], grid[row + 1][col], 1.0, f"right-column spring r{row}"))

    system._spring_deactivation_items = {"grid": grid, "springs": springs, "completed_chunks": 0}
    update_visuals(system)
    return system, grid, springs


def update_visuals(system):
    update_system_visuals(system)


def deactivate_overloaded_springs(system):
    items = system._spring_deactivation_items
    broken = 0
    for item in items["springs"]:
        if not item["active"]:
            continue
        force = abs(item["link"].GetForce())
        if force > BREAK_FORCE:
            item["link"].SetDisabled(True)
            item["native"].SetColor(color(0.45, 0.45, 0.45))
            item["fallback"].shape.SetColor(color(0.45, 0.45, 0.45))
            item["fallback"].shape.SetThickness(1)
            item["active"] = False
            broken += 1
    return broken


def simulate(duration, step):
    system, grid, springs = build_system()
    target_chunks = max(1, int(math.ceil(duration / CHUNK_TIME)))
    for _ in range(target_chunks):
        target = min(system.GetChTime() + CHUNK_TIME, duration)
        while system.GetChTime() < target - 1e-12:
            update_visuals(system)
            system.DoStepDynamics(min(step, target - system.GetChTime()))
        deactivate_overloaded_springs(system)
        system._spring_deactivation_items["completed_chunks"] += 1
    update_visuals(system)
    return system, grid, springs


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, grid, springs = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: springsDeactivateConnectors.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(3.5, -5.3, 4.1), chrono.ChVector3d(3.5, 1.35, 0.0))
    vis.AddTypicalLights()

    next_chunk = CHUNK_TIME
    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if system.GetChTime() >= next_chunk - 1e-12:
            deactivate_overloaded_springs(system)
            system._spring_deactivation_items["completed_chunks"] += 1
            next_chunk += CHUNK_TIME
        if time >= next_log:
            print_state(system, grid, springs)
            next_log += 0.02


def active_count(springs):
    return sum(1 for item in springs if item["active"])


def print_state(system, grid, springs):
    tip = grid[0][N_COLS - 2].GetPos()
    print(
        f"t={system.GetChTime():7.4f}  "
        f"tip_y={tip.y:+.6f}  "
        f"active_springs={active_count(springs)}/{len(springs)}  "
        f"chunks={system._spring_deactivation_items['completed_chunks']}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=N_CHUNKS * CHUNK_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: springsDeactivateConnectors.py -> PyChrono spring deactivation grid")
    if args.no_vis:
        system, grid, springs = simulate(args.duration, args.step)
        print_state(system, grid, springs)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

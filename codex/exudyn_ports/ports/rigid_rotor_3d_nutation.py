import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from laval_rotor_common import (
    add_bearing_bushing,
    add_bearing_spring_visual,
    color,
    make_reference_line,
    make_rotor_body,
    make_support,
    prepare_step,
)


# Reproduces the intent of EXUDYN Examples/rigidRotor3Dnutation.py:
# a spinning 3D rotor on a Cartesian support receives a short off-center force
# pulse from t=10.0 to t=10.05, exciting nutation.

MASS = 2.0
RADIUS = 0.5
DISK_LENGTH = 0.2
STIFFNESS = 8000.0
JXX = 0.5 * MASS * RADIUS**2
JYYZZ = 0.25 * MASS * RADIUS**2 + MASS * DISK_LENGTH**2 / 12.0
OMEGA0 = math.sqrt(2.0 * STIFFNESS / MASS)
DAMPING_RATIO = 0.002
DAMPING = 2.0 * OMEGA0 * DAMPING_RATIO * MASS
OMEGA_INITIAL = 0.1 * OMEGA0
PULSE_START = 10.0
PULSE_END = 10.05
STEP = 1e-3


def pulse_function():
    pulse = chrono.ChFunctionInterp()
    pulse.AddPoint(0, 0)
    pulse.AddPoint(PULSE_START - 1e-3, 0)
    pulse.AddPoint(PULSE_START, 1)
    pulse.AddPoint(PULSE_END, 1)
    pulse.AddPoint(PULSE_END + 1e-3, 0)
    pulse.AddPoint(50, 0)
    pulse.SetExtrapolate(True)
    return pulse


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    support = make_support(system, "nutation bearing support", chrono.ChVector3d(0, 0, 0), 0.09)
    make_reference_line(system, "nutation force reference", chrono.ChVector3d(0, -0.22, 0), 0.9)

    rotor = make_rotor_body(
        "nutation Laval rotor",
        MASS,
        chrono.ChVector3d(JXX, JYYZZ, JYYZZ),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(OMEGA_INITIAL, 0, 0),
        disk_radius=0.16,
        disk_length=DISK_LENGTH,
        shaft_left=-0.34,
        shaft_right=0.34,
        shaft_y=0,
        shaft_radius=0.014,
    )
    system.AddBody(rotor)

    bushing = add_bearing_bushing(
        system,
        support,
        rotor,
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(STIFFNESS, STIFFNESS, STIFFNESS),
        chrono.ChVector3d(DAMPING, DAMPING, DAMPING),
    )
    visual_spring = add_bearing_spring_visual(
        system,
        support,
        rotor,
        chrono.ChVector3d(0, 0.18, 0),
        chrono.ChVector3d(0, 0.38, 0),
        radius=0.032,
    )

    load_container = chrono.ChLoadContainer()
    system.Add(load_container)
    force = chrono.ChLoadBodyForce(
        rotor,
        chrono.ChVector3d(0.3, 0.2, 0.1),
        False,
        chrono.ChVector3d(0, RADIUS, 0),
        True,
    )
    force.SetModulationFunction(pulse_function())
    load_container.Add(force)

    arrow = chrono.ChBodyEasyBox(0.38, 0.018, 0.018, 1000, True, False)
    arrow.SetName("nutation pulse force marker")
    arrow.SetFixed(True)
    arrow.SetPos(chrono.ChVector3d(0.28, 0.46, 0.10))
    arrow.GetVisualShape(0).SetColor(color(0.90, 0.18, 0.12))
    system.AddBody(arrow)

    system._laval_rotor_data = {
        "omega0": OMEGA0,
        "pulse": (PULSE_START, PULSE_END),
    }
    return system, rotor, bushing, visual_spring, force


def update_visuals(system):
    prepare_step(system)


def simulate(duration, step):
    system, rotor, bushing, visual_spring, force = build_system()
    while system.GetChTime() < duration:
        prepare_step(system)
        system.DoStepDynamics(step)
    return system, rotor, bushing, visual_spring, force


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, rotor, bushing, visual_spring, force = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: rigidRotor3Dnutation.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.42, 0.82, 1.45), chrono.ChVector3d(0.0, 0.05, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        prepare_step(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, rotor, bushing)
            next_log += 1.0


def print_state(system, rotor, bushing):
    omega = rotor.GetAngVelLocal()
    pos = rotor.GetPos()
    active = PULSE_START <= system.GetChTime() <= PULSE_END
    print(
        f"t={system.GetChTime():6.3f}  "
        f"pos=({pos.x:+.5f}, {pos.y:+.5f}, {pos.z:+.5f})  "
        f"omega=({omega.x:+.3f}, {omega.y:+.3f}, {omega.z:+.3f})  "
        f"pulse_active={active}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: rigidRotor3Dnutation.py -> PyChrono nutating Laval rotor")
    if args.no_vis:
        system, rotor, bushing, visual_spring, force = simulate(args.duration, args.step)
        print_state(system, rotor, bushing)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

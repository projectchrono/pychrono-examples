import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from laval_rotor_common import (
    add_laval_bearings,
    make_reference_line,
    make_rotor_body,
    make_support,
    prepare_step,
)


# Reproduces the intent of EXUDYN Examples/rigidRotor3Drunup.py:
# a torque-driven symmetric Laval rotor with eccentricity and two Cartesian
# bearing spring-dampers. The full EXUDYN resonance pass is long; the default
# duration is shortened, while --duration 200 --step 0.005 gives the source
# runup time scale.

LENGTH = 1.0
L0 = 0.5
L1 = LENGTH - L0
MASS = 2.0
RADIUS = 0.75
DISK_LENGTH = 0.2
STIFFNESS = 800.0
JXX = 0.5 * MASS * RADIUS**2
JYYZZ = 0.25 * MASS * RADIUS**2 + MASS * DISK_LENGTH**2 / 12.0
OMEGA0 = math.sqrt(2.0 * STIFFNESS / MASS)
DAMPING_RATIO = 0.002
DAMPING = 2.0 * OMEGA0 * DAMPING_RATIO * MASS
TORQUE_X = 0.2
EPS = 2e-3 * 0.74
STEP = 1e-3


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    anchors = [
        chrono.ChVector3d(-0.5 * LENGTH, 0, 0),
        chrono.ChVector3d(0.5 * LENGTH, 0, 0),
    ]
    supports = [
        make_support(system, "left runup bearing support", anchors[0], 0.09),
        make_support(system, "right runup bearing support", anchors[1], 0.09),
    ]
    make_reference_line(system, "runup bearing reference line", chrono.ChVector3d(0, -0.22, 0), 1.25)

    rotor = make_rotor_body(
        "runup Laval rotor",
        MASS,
        chrono.ChVector3d(JXX, JYYZZ, JYYZZ),
        chrono.ChVector3d(L0 - 0.5 * LENGTH, EPS, 0),
        chrono.ChVector3d(0, 0, 0),
        disk_radius=0.18,
        disk_length=DISK_LENGTH,
        shaft_left=-L0,
        shaft_right=L1,
        shaft_y=-EPS,
        shaft_radius=0.018,
    )
    system.AddBody(rotor)

    rotor_locals = [
        chrono.ChVector3d(-L0, -EPS, 0),
        chrono.ChVector3d(L1, -EPS, 0),
    ]
    stiffnesses = [
        chrono.ChVector3d(STIFFNESS, STIFFNESS, STIFFNESS),
        chrono.ChVector3d(0, STIFFNESS, STIFFNESS),
    ]
    dampings = [
        chrono.ChVector3d(DAMPING, DAMPING, DAMPING),
        chrono.ChVector3d(0, DAMPING, DAMPING),
    ]
    bushings, visual_springs = add_laval_bearings(
        system, rotor, supports, anchors, rotor_locals, stiffnesses, dampings
    )

    load_container = chrono.ChLoadContainer()
    system.Add(load_container)
    torque = chrono.ChLoadBodyTorque(rotor, chrono.ChVector3d(TORQUE_X, 0, 0), True)
    load_container.Add(torque)

    system._laval_rotor_data = {
        "eps": EPS,
        "omega0": OMEGA0,
        "torque": TORQUE_X,
    }
    return system, rotor, bushings, visual_springs, torque


def update_visuals(system):
    prepare_step(system)


def simulate(duration, step):
    system, rotor, bushings, visual_springs, torque = build_system()
    while system.GetChTime() < duration:
        prepare_step(system)
        system.DoStepDynamics(step)
    return system, rotor, bushings, visual_springs, torque


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, rotor, bushings, visual_springs, torque = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: rigidRotor3Drunup.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.34, 0.84, 1.55), chrono.ChVector3d(0.0, 0.02, 0.0))
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
            print_state(system, rotor, bushings)
            next_log += 1.0


def print_state(system, rotor, bushings):
    data = system._laval_rotor_data
    shaft = rotor.TransformPointLocalToParent(chrono.ChVector3d(0, -data["eps"], 0))
    omega = rotor.GetAngVelLocal()
    print(
        f"t={system.GetChTime():6.3f}  "
        f"shaft=({shaft.x:+.5f}, {shaft.y:+.5f}, {shaft.z:+.5f})  "
        f"omega_x={omega.x:+.4f} rad/s  omega_x_hz={omega.x / (2 * math.pi):+.4f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: rigidRotor3Drunup.py -> PyChrono torque-driven Laval rotor")
    if args.no_vis:
        system, rotor, bushings, visual_springs, torque = simulate(args.duration, args.step)
        print_state(system, rotor, bushings)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

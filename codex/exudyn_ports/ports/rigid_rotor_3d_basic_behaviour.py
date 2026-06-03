import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from laval_rotor_common import (
    add_laval_bearings,
    color,
    enforce_constant_spin,
    make_reference_line,
    make_rotor_body,
    make_support,
    prepare_step,
)


# Reproduces the intent of EXUDYN Examples/rigidRotor3DbasicBehaviour.py:
# a Laval rotor with eccentric mass center, Cartesian bearing stiffness/damping,
# and a velocity-level x-spin constraint. The --mode option selects slow,
# critical, or fast initial spin as in the EXUDYN source.

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
BASE_DAMPING = 2.0 * OMEGA0 * DAMPING_RATIO * MASS
BASE_EPS = 10e-3
STEP = 1e-3


def mode_parameters(mode):
    eps = BASE_EPS
    damping = BASE_DAMPING
    if mode == 0:
        omega = 0.5 * OMEGA0
    elif mode == 1:
        omega = OMEGA0
        eps *= 0.1
        damping *= 10.0
    elif mode == 2:
        omega = 2.0 * OMEGA0
    else:
        raise ValueError("mode must be 0, 1, or 2")
    return eps, damping, omega


def build_system(mode=2):
    eps, damping, omega_initial = mode_parameters(mode)
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    anchors = [
        chrono.ChVector3d(-0.5 * LENGTH, 0, 0),
        chrono.ChVector3d(0.5 * LENGTH, 0, 0),
    ]
    supports = [
        make_support(system, "left rotor bearing support", anchors[0], 0.09),
        make_support(system, "right rotor bearing support", anchors[1], 0.09),
    ]
    make_reference_line(system, "bearing reference line", chrono.ChVector3d(0, -0.18, 0), 1.25)

    rotor = make_rotor_body(
        "basic Laval rotor",
        MASS,
        chrono.ChVector3d(JXX, JYYZZ, JYYZZ),
        chrono.ChVector3d(L0 - 0.5 * LENGTH, eps, 0),
        chrono.ChVector3d(omega_initial, 0, 0),
        disk_radius=0.10,
        disk_length=DISK_LENGTH,
        shaft_left=-L0,
        shaft_right=L1,
        shaft_y=-eps,
        shaft_radius=0.015,
    )
    system.AddBody(rotor)

    rotor_locals = [
        chrono.ChVector3d(-L0, -eps, 0),
        chrono.ChVector3d(L1, -eps, 0),
    ]
    stiffnesses = [
        chrono.ChVector3d(STIFFNESS, STIFFNESS, STIFFNESS),
        chrono.ChVector3d(0, STIFFNESS, STIFFNESS),
    ]
    dampings = [
        chrono.ChVector3d(damping, damping, damping),
        chrono.ChVector3d(0, damping, damping),
    ]
    bushings, visual_springs = add_laval_bearings(
        system, rotor, supports, anchors, rotor_locals, stiffnesses, dampings
    )

    system._laval_rotor_constant_spin = {"rotor": rotor, "omega_x": omega_initial}
    system._laval_rotor_data = {
        "eps": eps,
        "omega0": OMEGA0,
        "omega_initial": omega_initial,
        "mode": mode,
    }
    return system, rotor, bushings, visual_springs


def update_visuals(system):
    prepare_step(system)


def simulate(duration, step, mode):
    system, rotor, bushings, visual_springs = build_system(mode)
    while system.GetChTime() < duration:
        prepare_step(system)
        system.DoStepDynamics(step)
    return system, rotor, bushings, visual_springs


def run_visual(duration, step, mode):
    import pychrono.irrlicht as chronoirr

    system, rotor, bushings, visual_springs = build_system(mode)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: rigidRotor3DbasicBehaviour.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.28, 0.78, 1.45), chrono.ChVector3d(0.0, 0.02, 0.0))
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
            next_log += 0.5


def print_state(system, rotor, bushings):
    data = system._laval_rotor_data
    shaft = rotor.TransformPointLocalToParent(chrono.ChVector3d(0, -data["eps"], 0))
    omega = rotor.GetAngVelLocal()
    print(
        f"t={system.GetChTime():6.3f}  mode={data['mode']}  "
        f"shaft=({shaft.x:+.5f}, {shaft.y:+.5f}, {shaft.z:+.5f})  "
        f"omega_x={omega.x:+.4f} rad/s  resonance={data['omega0']:+.4f} rad/s"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2.5)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--mode", type=int, default=2, choices=(0, 1, 2))
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: rigidRotor3DbasicBehaviour.py -> PyChrono Laval rotor")
    if args.no_vis:
        system, rotor, bushings, visual_springs = simulate(args.duration, args.step, args.mode)
        print_state(system, rotor, bushings)
    else:
        run_visual(args.duration, args.step, args.mode)


if __name__ == "__main__":
    main()

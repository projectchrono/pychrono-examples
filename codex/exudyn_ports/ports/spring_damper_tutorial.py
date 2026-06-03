import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN Examples/springDamperTutorial.py:
# a 3D mass point with only its x coordinate connected to ground by a
# coordinate spring-damper, unit initial velocity, and constant coordinate load.

LENGTH = 0.5
MASS = 1.6
STIFFNESS = 4000.0
DAMPING = 8.0
INITIAL_DISPLACEMENT = -0.08
INITIAL_VELOCITY = 1.0
FORCE = 80.0
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def update_visuals(system):
    update_system_visuals(system)


def exact_displacement(t):
    omega0 = math.sqrt(STIFFNESS / MASS)
    damping_ratio = DAMPING / (2.0 * math.sqrt(STIFFNESS * MASS))
    omega = omega0 * math.sqrt(1.0 - damping_ratio**2)
    static_displacement = FORCE / STIFFNESS
    c1 = INITIAL_DISPLACEMENT - static_displacement
    c2 = (INITIAL_VELOCITY + omega0 * damping_ratio * c1) / omega
    return (
        math.exp(-omega0 * damping_ratio * t)
        * (c1 * math.cos(omega * t) + c2 * math.sin(omega * t))
        + static_displacement
    )


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)

    mass = chrono.ChBodyEasySphere(0.055, 1000, True, False)
    mass.SetMass(MASS)
    mass.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    mass.SetPos(chrono.ChVector3d(LENGTH + INITIAL_DISPLACEMENT, 0, 0))
    mass.SetPosDt(chrono.ChVector3d(INITIAL_VELOCITY, 0, 0))
    mass.GetVisualShape(0).SetColor(color(0.12, 0.36, 0.90))
    system.AddBody(mass)

    slider = chrono.ChLinkLockPrismatic()
    slider.Initialize(mass, ground, chrono.ChFramed(chrono.ChVector3d(LENGTH, 0, 0), chrono.Q_ROTATE_Z_TO_X))
    system.AddLink(slider)

    spring = chrono.ChLinkTSDA()
    spring.Initialize(mass, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
    spring.SetRestLength(LENGTH)
    spring.SetSpringCoefficient(STIFFNESS)
    spring.SetDampingCoefficient(DAMPING)
    system.AddLink(spring)
    spring_shape = chrono.ChVisualShapeSpring(0.055, 100, 15)
    spring_shape.SetColor(color(0.85, 0.18, 0.12))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, spring, 0.055, 100, 15, color(0.85, 0.18, 0.12))

    anchor = chrono.ChBodyEasySphere(0.04, 1000, True, False)
    anchor.SetFixed(True)
    anchor.SetPos(chrono.ChVector3d(0, 0, 0))
    anchor.GetVisualShape(0).SetColor(color(0.12, 0.12, 0.12))
    system.AddBody(anchor)

    rail = chrono.ChBodyEasyBox(1.2, 0.015, 0.015, 1000, True, False)
    rail.SetFixed(True)
    rail.SetPos(chrono.ChVector3d(0.55, -0.14, 0))
    rail.GetVisualShape(0).SetColor(color(0.45, 0.45, 0.45))
    system.AddBody(rail)

    force = chrono.ChForce()
    force.SetF_x(chrono.ChFunctionConst(FORCE))
    mass.AddForce(force)

    return system, mass, spring


def simulate(duration, step):
    system, mass, spring = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, mass, spring


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, mass, spring = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: springDamperTutorial.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.5, 0.6, 1.8), chrono.ChVector3d(0.5, 0, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        update_visuals(system)
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, mass, spring)
            next_log += 0.25


def print_state(system, mass, spring):
    displacement = mass.GetPos().x - LENGTH
    reference = exact_displacement(system.GetChTime())
    print(
        f"t={system.GetChTime():6.3f}  u={displacement:+.8f}  "
        f"exact={reference:+.8f}  error={displacement - reference:+.3e}  "
        f"spring_force={spring.GetForce():+.3f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: springDamperTutorial.py -> PyChrono coordinate spring-damper")
    if args.no_vis:
        system, mass, spring = simulate(args.duration, args.step)
        print_state(system, mass, spring)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN TestModels/velocityVerletTest.py:
# compare Velocity Verlet step sizes on a conservative 1D mass-spring oscillator.
# PyChrono does not expose EXUDYN's VelocityVerlet solver, so this port keeps the
# integrator test explicitly and uses PyChrono bodies/spring visuals for the
# rendered oscillator.

REST_LENGTH = 1.0
MASS = 10.0
STIFFNESS = 1000.0
INITIAL_POSITION = 1.0
INITIAL_VELOCITY = 1.0
REFERENCE_T_END = 2.0
REFERENCE_POSITION = 1.091294525072764
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def acceleration(x):
    return -(STIFFNESS / MASS) * (x - REST_LENGTH)


def exact_position(t):
    omega = math.sqrt(STIFFNESS / MASS)
    return REST_LENGTH + (INITIAL_POSITION - REST_LENGTH) * math.cos(omega * t) + INITIAL_VELOCITY / omega * math.sin(omega * t)


def velocity_verlet(h, duration=REFERENCE_T_END):
    x = INITIAL_POSITION
    v = INITIAL_VELOCITY
    t = 0.0
    while t < duration - 1e-15:
        dt = min(h, duration - t)
        a0 = acceleration(x)
        x_new = x + v * dt + 0.5 * a0 * dt * dt
        a1 = acceleration(x_new)
        v = v + 0.5 * (a0 + a1) * dt
        x = x_new
        t += dt
    return x, v


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)

    anchor = chrono.ChBodyEasySphere(0.05, 1000, True, False)
    anchor.SetFixed(True)
    anchor.SetPos(chrono.ChVector3d(0, 0, 0))
    anchor.GetVisualShape(0).SetColor(color(0.10, 0.10, 0.10))
    system.AddBody(anchor)

    mass = chrono.ChBodyEasySphere(0.20, 1000, True, False)
    mass.SetFixed(True)
    mass.SetPos(chrono.ChVector3d(INITIAL_POSITION, 0, 0))
    mass.GetVisualShape(0).SetColor(color(0.90, 0.12, 0.08))
    system.AddBody(mass)

    spring = chrono.ChLinkTSDA()
    spring.Initialize(mass, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
    spring.SetRestLength(REST_LENGTH)
    spring.SetSpringCoefficient(STIFFNESS)
    spring.SetDampingCoefficient(0)
    system.AddLink(spring)
    spring_shape = chrono.ChVisualShapeSpring(0.10, 100, 14)
    spring_shape.SetColor(color(0.85, 0.18, 0.12))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, spring, 0.10, 100, 14, color(0.85, 0.18, 0.12))

    rail = chrono.ChBodyEasyBox(1.8, 0.025, 0.025, 1000, True, False)
    rail.SetFixed(True)
    rail.SetPos(chrono.ChVector3d(0.9, -0.30, 0))
    rail.GetVisualShape(0).SetColor(color(0.45, 0.45, 0.45))
    system.AddBody(rail)

    system._exudyn_port_mass_body = mass
    return system, mass, spring


def update_visuals(system):
    mass = getattr(system, "_exudyn_port_mass_body", None)
    if mass is not None:
        mass.SetPos(chrono.ChVector3d(exact_position(system.GetChTime()), 0, 0))
        mass.UpdateVisualModel()
    update_system_visuals(system)


def simulate():
    results = []
    for i in range(4):
        h = STEP / (2**i)
        x, v = velocity_verlet(h)
        results.append((h, x, x - REFERENCE_POSITION))
    return results


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, mass, spring = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: velocityVerletTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.65, 0.65, 2.4), chrono.ChVector3d(0.65, 0, 0))
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
            print(f"t={system.GetChTime():6.3f}  x_exact={exact_position(system.GetChTime()):+.8f}")
            next_log += 0.25


def print_state(results):
    total = sum(x for _, x, _ in results)
    for h, x, err in results:
        print(f"h={h:.8f}  x={x:+.12f}  err={err:+.3e}")
    print(f"velocityVerletTest result={total:+.12f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=REFERENCE_T_END)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: velocityVerletTest.py -> PyChrono visual spring plus Velocity Verlet check")
    if args.no_vis:
        print_state(simulate())
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN Examples/nMassOscillatorEigenmodes.py:
# build the same 12-mass 1D spring chain and compute/animate its eigenmodes.

N = 12
MASS = 1.0
STIFFNESS = 800.0
DAMPING = 2.0
REST = 0.2
MODE_AMPLITUDE = 0.08
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def eigenpairs():
    stiffness = np.zeros((N, N), dtype=float)
    stiffness[0, 0] += STIFFNESS
    for i in range(N - 1):
        stiffness[i, i] += STIFFNESS
        stiffness[i + 1, i + 1] += STIFFNESS
        stiffness[i, i + 1] -= STIFFNESS
        stiffness[i + 1, i] -= STIFFNESS
    values, vectors = np.linalg.eigh(stiffness / MASS)
    omegas = np.sqrt(np.maximum(values, 0.0))
    for i in range(vectors.shape[1]):
        vectors[:, i] /= np.max(np.abs(vectors[:, i]))
    return omegas, vectors


def update_visuals(system):
    bodies = getattr(system, "_exudyn_port_mode_bodies", [])
    if bodies:
        mode = system._exudyn_port_mode_shape
        omega = system._exudyn_port_mode_omega
        phase = math.sin(omega * system.GetChTime())
        for i, body in enumerate(bodies):
            x = (i + 1) * REST + MODE_AMPLITUDE * mode[i] * phase
            body.SetPos(chrono.ChVector3d(x, 0, 0))
    update_system_visuals(system)


def build_system(mode_index=0):
    omegas, vectors = eigenpairs()
    mode_index = max(0, min(mode_index, N - 1))

    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)

    anchor = chrono.ChBodyEasySphere(0.045, 1000, True, False)
    anchor.SetFixed(True)
    anchor.SetPos(chrono.ChVector3d(0, 0, 0))
    anchor.GetVisualShape(0).SetColor(color(0.12, 0.12, 0.12))
    system.AddBody(anchor)

    rail = chrono.ChBodyEasyBox((N + 1) * REST, 0.02, 0.02, 1000, True, False)
    rail.SetFixed(True)
    rail.SetPos(chrono.ChVector3d(0.5 * (N + 1) * REST, -0.13, 0))
    rail.GetVisualShape(0).SetColor(color(0.40, 0.40, 0.40))
    system.AddBody(rail)

    bodies = []
    previous = ground
    for i in range(N):
        body = chrono.ChBodyEasySphere(0.055, 1000, True, False)
        body.SetFixed(True)
        body.SetMass(MASS)
        body.SetPos(chrono.ChVector3d((i + 1) * REST, 0, 0))
        if i == 0:
            body.GetVisualShape(0).SetColor(color(0.10, 0.65, 0.25))
        elif i == N - 1:
            body.GetVisualShape(0).SetColor(color(0.85, 0.14, 0.10))
        else:
            body.GetVisualShape(0).SetColor(color(0.12, 0.42, 0.85))
        system.AddBody(body)
        bodies.append(body)

        spring = chrono.ChLinkTSDA()
        spring.Initialize(body, previous, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
        spring.SetRestLength(REST)
        spring.SetSpringCoefficient(STIFFNESS)
        spring.SetDampingCoefficient(DAMPING)
        system.AddLink(spring)
        spring_shape = chrono.ChVisualShapeSpring(0.025, 60, 8)
        spring_shape.SetColor(color(0.45, 0.45, 0.45))
        spring.AddVisualShape(spring_shape)
        attach_spring_visual(system, spring, 0.025, 60, 8, color(0.45, 0.45, 0.45))
        previous = body

    system._exudyn_port_mode_bodies = bodies
    system._exudyn_port_mode_shape = vectors[:, mode_index]
    system._exudyn_port_mode_omega = float(omegas[mode_index])
    system._exudyn_port_omegas = omegas
    system._exudyn_port_modes = vectors
    update_visuals(system)
    return system, bodies


def run_visual(duration, step, mode_index):
    import pychrono.irrlicht as chronoirr

    system, bodies = build_system(mode_index)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1180, 720)
    vis.SetWindowTitle("EXUDYN port: nMassOscillatorEigenmodes.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.35, 1.0, 2.8), chrono.ChVector3d(1.25, 0, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        update_visuals(system)
        vis.Render()
        vis.EndScene()
        system.SetChTime(t + step)
        if t >= next_log:
            print_state(system, mode_index)
            next_log += 0.5


def print_state(system, mode_index):
    omegas = system._exudyn_port_omegas
    first = " ".join(f"{omega:.6f}" for omega in omegas[:6])
    print(f"mode={mode_index + 1}  omega={omegas[mode_index]:.8f} rad/s  first_omegas=[{first}]")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--mode", type=int, default=1, help="1-based mode index")
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    mode_index = max(0, min(args.mode - 1, N - 1))
    print("EXUDYN port: nMassOscillatorEigenmodes.py -> PyChrono visual eigenmode model")
    system, bodies = build_system(mode_index)
    print_state(system, mode_index)
    if not args.no_vis:
        run_visual(args.duration, args.step, mode_index)


if __name__ == "__main__":
    main()

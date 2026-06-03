import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/generalContactImplicit1.py:
# a single rigid sphere falling onto a triangular floor with normal/frictional
# contact. Chrono uses native SMC contact for the same sphere-floor setup.

LENGTH = 1.0
BASE = 0.1
RADIUS = 0.5 * BASE
MASS = 0.025
GRAVITY = 10.0
STEP = 2e-4


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_material():
    material = chrono.ChContactMaterialSMC()
    material.SetFriction(0.8)
    material.SetKn(1e3)
    material.SetGn(0.2)
    material.SetKt(1e3)
    material.SetGt(0.2)
    material.SetRestitution(0.02)
    return material


def build_system():
    system = chrono.ChSystemSMC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -GRAVITY))
    material = make_material()

    ground = chrono.ChBodyEasyBox(LENGTH, LENGTH, 0.025, 1000, True, True, material)
    ground.SetFixed(True)
    ground.SetPos(chrono.ChVector3d(0, 0, -0.0125))
    ground.GetVisualShape(0).SetColor(color(0.84, 0.84, 0.68))
    ground.GetVisualShape(0).SetOpacity(0.55)
    system.AddBody(ground)

    sphere = chrono.ChBodyEasySphere(RADIUS, MASS / ((4.0 / 3.0) * chrono.CH_PI * RADIUS**3), True, True, material)
    sphere.SetMass(MASS)
    sphere.SetPos(chrono.ChVector3d(-0.1 * LENGTH, -0.1 * LENGTH, 0.5 * LENGTH + RADIUS))
    sphere.GetVisualShape(0).SetColor(color(0.90, 0.12, 0.10))
    system.AddBody(sphere)

    return system, sphere


def simulate(duration, step):
    system, sphere = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, sphere


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, sphere = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: generalContactImplicit1.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.55, -0.75, 0.65), chrono.ChVector3d(0, 0, 0.20))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, sphere)
            next_log += 0.25


def print_state(system, sphere):
    pos = sphere.GetPos()
    vel = sphere.GetPosDt()
    omega = sphere.GetAngVelParent()
    total = abs(pos.x) + abs(pos.y) + abs(pos.z) + abs(omega.x) + abs(omega.y) + abs(omega.z)
    print(
        f"t={system.GetChTime():6.3f}  "
        f"pos=({pos.x:+.6f}, {pos.y:+.6f}, {pos.z:+.6f})  "
        f"vz={vel.z:+.6f}  coordinate_sum={total:.8f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: generalContactImplicit1.py -> PyChrono SMC sphere-floor contact")
    if args.no_vis:
        system, sphere = simulate(args.duration, args.step)
        print_state(system, sphere)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

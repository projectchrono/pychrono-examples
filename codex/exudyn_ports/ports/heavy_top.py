import argparse

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/heavyTop.py:
# a heavy rigid top with a spherical fixed point, gravity acting at its center
# of mass, and a large initial spin.

MASS = 15.0
LENGTH = 1.0
RADIUS = 0.5
GRAVITY = 9.81
INERTIA_COM = chrono.ChVector3d(0.234375, 0.46875, 0.234375)
INITIAL_OMEGA = chrono.ChVector3d(0, 150, -4.61538)
STEP = 1e-4


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -GRAVITY))

    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)

    pivot = chrono.ChBodyEasySphere(0.045, 1000, True, False)
    pivot.SetFixed(True)
    pivot.SetPos(chrono.ChVector3d(0, 0, 0))
    pivot.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.08))
    system.AddBody(pivot)

    body = chrono.ChBody()
    body.SetMass(MASS)
    body.SetInertiaXX(INERTIA_COM)
    body.SetPos(chrono.ChVector3d(0, 1, 0))
    body.SetAngVelLocal(INITIAL_OMEGA)
    body.SetUseGyroTorque(True)
    body.EnableCollision(False)

    shape = chrono.ChVisualShapeBox(RADIUS, LENGTH, RADIUS)
    shape.SetColor(color(0.10, 0.10, 0.80))
    body.AddVisualShape(shape, chrono.ChFramed(chrono.ChVector3d(0, -0.5 * LENGTH, 0)))
    tip = chrono.ChVisualShapeSphere(0.055)
    tip.SetColor(color(0.95, 0.72, 0.05))
    body.AddVisualShape(tip, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    system.AddBody(body)

    joint = chrono.ChLinkLockSpherical()
    joint.Initialize(body, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    system.AddLink(joint)

    return system, body, joint


def simulate(duration, step):
    system, body, joint = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, body, joint


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, body, joint = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: heavyTop.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.6, 1.8, 1.6), chrono.ChVector3d(0, 0.45, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, body)
            next_log += 0.05


def print_state(system, body):
    omega = body.GetAngVelLocal()
    pivot_world = body.TransformPointLocalToParent(chrono.ChVector3d(0, -1, 0))
    print(
        f"t={system.GetChTime():6.3f}  "
        f"omega_local=({omega.x:+.4f}, {omega.y:+.4f}, {omega.z:+.4f})  "
        f"pivot=({pivot_world.x:+.3e}, {pivot_world.y:+.3e}, {pivot_world.z:+.3e})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0.2)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: heavyTop.py -> PyChrono spherical-joint heavy top")
    if args.no_vis:
        system, body, joint = simulate(args.duration, args.step)
        print_state(system, body)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

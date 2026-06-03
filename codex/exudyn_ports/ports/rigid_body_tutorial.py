import argparse

import pychrono.core as chrono


# Reproduces the intent of EXUDYN Examples/rigidBodyTutorial.py:
# a single 3D rigid link under gravity, constrained to ground by a generic
# joint equivalent to a revolute-Z support.

LENGTH = 1.0
WIDTH = 0.1
DENSITY = 5000.0
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)

    link = chrono.ChBodyEasyBox(LENGTH, WIDTH, WIDTH, DENSITY, True, False)
    link.SetName("rigid body tutorial link")
    link.SetPos(chrono.ChVector3d(0.5 * LENGTH, 0, 0))
    link.GetVisualShape(0).SetColor(color(0.95, 0.35, 0.30))
    system.AddBody(link)

    joint = chrono.ChLinkLockRevolute()
    joint.Initialize(link, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
    system.AddLink(joint)

    pivot = chrono.ChBodyEasySphere(0.045, 1000, True, False)
    pivot.SetFixed(True)
    pivot.SetPos(chrono.ChVector3d(0, 0, 0))
    pivot.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.08))
    system.AddBody(pivot)

    return system, link, joint


def simulate(duration, step):
    system, link, joint = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, link, joint


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, link, joint = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: rigidBodyTutorial.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.45, 0.45, 2.0), chrono.ChVector3d(0.35, -0.20, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, link)
            next_log += 0.5


def print_state(system, link):
    angle = link.GetRot().GetCardanAnglesXYZ().z
    tip = link.TransformPointLocalToParent(chrono.ChVector3d(0.5 * LENGTH, 0, 0))
    print(
        f"t={system.GetChTime():6.3f}  "
        f"angle_z={angle:+.6f}  tip=({tip.x:+.5f}, {tip.y:+.5f}, {tip.z:+.5f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: rigidBodyTutorial.py -> PyChrono revolute rigid link")
    if args.no_vis:
        system, link, joint = simulate(args.duration, args.step)
        print_state(system, link)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

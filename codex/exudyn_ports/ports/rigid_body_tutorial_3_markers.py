import argparse

import pychrono.core as chrono


# Reproduces the intent of EXUDYN Examples/rigidBodyTutorial3withMarkers.py:
# the marker-style version of the two-link 3D rigid body tutorial.

LENGTH = 1.0
WIDTH = 0.1
DENSITY = 5000.0
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    ground = chrono.ChBodyEasySphere(0.045, 1000, True, False)
    ground.SetName("fixed revolute support")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    ground.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.08))
    system.AddBody(ground)

    link0 = chrono.ChBodyEasyBox(LENGTH, WIDTH, WIDTH, DENSITY, True, False)
    link0.SetName("marker tutorial first link")
    link0.SetPos(chrono.ChVector3d(0.5 * LENGTH, 0, 0))
    link0.GetVisualShape(0).SetColor(color(0.92, 0.12, 0.08))
    system.AddBody(link0)

    link1 = chrono.ChBodyEasyBox(WIDTH, WIDTH, LENGTH, DENSITY, True, False)
    link1.SetName("marker tutorial second link")
    link1.SetPos(chrono.ChVector3d(LENGTH, 0, 0.5 * LENGTH))
    link1.GetVisualShape(0).SetColor(color(0.55, 0.85, 0.35))
    system.AddBody(link1)

    ground_joint = chrono.ChLinkLockRevolute()
    ground_joint.Initialize(link0, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
    system.AddLink(ground_joint)

    elbow = chrono.ChLinkLockRevolute()
    elbow.Initialize(link1, link0, chrono.ChFramed(chrono.ChVector3d(LENGTH, 0, 0), chrono.Q_ROTATE_Z_TO_X))
    system.AddLink(elbow)

    marker = chrono.ChBodyEasySphere(0.045, 1000, True, False)
    marker.SetFixed(True)
    marker.SetPos(chrono.ChVector3d(LENGTH, 0, 0))
    marker.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.08))
    system.AddBody(marker)

    return system, (link0, link1), (ground_joint, elbow)


def simulate(duration, step):
    system, links, joints = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, links, joints


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, links, joints = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: rigidBodyTutorial3withMarkers.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.2, 0.8, 2.5), chrono.ChVector3d(0.75, -0.2, 0.25))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, links)
            next_log += 0.5


def print_state(system, links):
    tip = links[1].TransformPointLocalToParent(chrono.ChVector3d(0, 0, 0.5 * LENGTH))
    print(f"t={system.GetChTime():6.3f}  tip=({tip.x:+.5f}, {tip.y:+.5f}, {tip.z:+.5f})")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: rigidBodyTutorial3withMarkers.py -> PyChrono marker-style two-link tutorial")
    if args.no_vis:
        system, links, joints = simulate(args.duration, args.step)
        print_state(system, links)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

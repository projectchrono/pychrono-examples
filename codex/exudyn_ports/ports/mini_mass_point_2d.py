import argparse

import pychrono.core as chrono


# Reproduces the intent of EXUDYN
# TestModels/MiniExamples/ObjectMassPoint2D.py:
# a free 2D mass point with the same x motion as ObjectMassPoint, constrained
# to the xy plane.

STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def build_system():
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    ground = chrono.ChBody()
    ground.SetFixed(True)
    sys.AddBody(ground)

    mass = chrono.ChBodyEasySphere(0.06, 1000, True, False)
    mass.SetMass(1.0)
    mass.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    mass.SetPos(chrono.ChVector3d(1.5, 1.0, 0))
    mass.SetPosDt(chrono.ChVector3d(0.5, 0, 0))
    mass.GetVisualShape(0).SetColor(color(0.12, 0.60, 0.25))
    sys.AddBody(mass)

    planar = chrono.ChLinkLockPlanar()
    planar.Initialize(mass, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
    sys.AddLink(planar)

    rail = chrono.ChBodyEasyBox(1.25, 0.015, 0.015, 1000, True, False)
    rail.SetFixed(True)
    rail.SetPos(chrono.ChVector3d(1.6, 0.88, 0))
    rail.GetVisualShape(0).SetColor(color(0.45, 0.45, 0.45))
    sys.AddBody(rail)

    return sys, mass


def simulate(duration, step):
    sys, mass = build_system()
    while sys.GetChTime() < duration:
        sys.DoStepDynamics(step)
    return sys, mass


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    sys, mass = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ObjectMassPoint2D.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.7, 1.35, 1.4), chrono.ChVector3d(1.7, 1.0, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and sys.GetChTime() < duration:
        t = sys.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(step)
        if t >= next_log:
            print_state(sys, mass)
            next_log += 0.25


def print_state(sys, mass):
    exact_x = 1.5 + 0.5 * sys.GetChTime()
    print(f"t={sys.GetChTime():6.3f}  x={mass.GetPos().x:+.8f}  z={mass.GetPos().z:+.3e}  exact={exact_x:+.8f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ObjectMassPoint2D.py -> PyChrono planar mass point")
    if args.no_vis:
        sys, mass = simulate(args.duration, args.step)
        print_state(sys, mass)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

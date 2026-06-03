import argparse

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/MiniExamples/ObjectMass1D.py:
# a 1D translational mass with reference coordinate 1, initial displacement
# 0.5, and initial velocity 0.5, so x=2 after 1 s.

STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def build_system():
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    ground = chrono.ChBody()
    ground.SetFixed(True)
    sys.AddBody(ground)

    mass = chrono.ChBodyEasyBox(0.12, 0.08, 0.08, 1000, True, False)
    mass.SetMass(1.0)
    mass.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    mass.SetPos(chrono.ChVector3d(1.5, 0, 0))
    mass.SetPosDt(chrono.ChVector3d(0.5, 0, 0))
    mass.GetVisualShape(0).SetColor(color(0.12, 0.38, 0.88))
    sys.AddBody(mass)

    slider = chrono.ChLinkLockPrismatic()
    slider.Initialize(mass, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.Q_ROTATE_Z_TO_X))
    sys.AddLink(slider)

    rail = chrono.ChBodyEasyBox(1.25, 0.012, 0.012, 1000, True, False)
    rail.SetFixed(True)
    rail.SetPos(chrono.ChVector3d(1.6, -0.12, 0))
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
    vis.SetWindowTitle("EXUDYN port: ObjectMass1D.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.7, 0.3, 1.4), chrono.ChVector3d(1.7, 0, 0))
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
    print(f"t={sys.GetChTime():6.3f}  x={mass.GetPos().x:+.8f}  exact={exact_x:+.8f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ObjectMass1D.py -> PyChrono prismatic 1D mass")
    if args.no_vis:
        sys, mass = simulate(args.duration, args.step)
        print_state(sys, mass)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

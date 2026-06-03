import argparse

import pychrono.core as chrono


# Reproduces the intent of EXUDYN
# TestModels/MiniExamples/LoadMassProportional.py:
# a 2 kg point mass receives a mass-proportional acceleration load of -9.81 in
# z, so the final z position after 1 s is approximately -g/2.

MASS = 2.0
ACCELERATION = -9.81
STEP = 1e-4


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def build_system():
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, ACCELERATION))

    mass = chrono.ChBodyEasySphere(0.06, 1000, True, False)
    mass.SetMass(MASS)
    mass.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    mass.SetPos(chrono.ChVector3d(1, 0, 0))
    mass.GetVisualShape(0).SetColor(color(0.12, 0.38, 0.88))
    sys.AddBody(mass)

    ground = chrono.ChBodyEasyBox(0.28, 0.02, 0.02, 1000, True, False)
    ground.SetFixed(True)
    ground.SetPos(chrono.ChVector3d(1, 0, 0))
    ground.GetVisualShape(0).SetColor(color(0.45, 0.45, 0.45))
    sys.AddBody(ground)

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
    vis.SetWindowTitle("EXUDYN port: LoadMassProportional.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.0, 1.2, 3.0), chrono.ChVector3d(1.0, 0, -1.7))
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
    exact_z = 0.5 * ACCELERATION * sys.GetChTime() ** 2
    print(f"t={sys.GetChTime():6.3f}  z={mass.GetPos().z:+.8f}  exact={exact_z:+.8f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: LoadMassProportional.py -> PyChrono gravity-equivalent load")
    if args.no_vis:
        sys, mass = simulate(args.duration, args.step)
        print_state(sys, mass)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN
# TestModels/MiniExamples/ObjectRotationalMass1D.py:
# a 1D rotational inertia with reference angle 1, initial displacement 0.5,
# and angular speed 0.5, so psi=2 after 1 s.

STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def update_visuals(sys):
    shaft = getattr(sys, "_exudyn_port_shaft", None)
    rotor = getattr(sys, "_exudyn_port_rotor_visual", None)
    if shaft is not None and rotor is not None:
        rotor.SetRot(chrono.QuatFromAngleZ(shaft.GetPos()))


def build_system():
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    shaft = chrono.ChShaft()
    shaft.SetInertia(1.0)
    shaft.SetPos(1.5)
    shaft.SetPosDt(0.5)
    sys.Add(shaft)

    rotor = chrono.ChBody()
    rotor.SetFixed(True)
    rotor.EnableCollision(False)
    disk = chrono.ChVisualShapeCylinder(0.22, 0.04)
    disk.SetColor(color(0.12, 0.38, 0.88))
    rotor.AddVisualShape(disk, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.Q_ROTATE_Y_TO_Z))
    mark = chrono.ChVisualShapeBox(0.32, 0.025, 0.025)
    mark.SetColor(color(0.92, 0.22, 0.12))
    rotor.AddVisualShape(mark, chrono.ChFramed(chrono.ChVector3d(0.13, 0, 0)))
    sys.AddBody(rotor)

    sys._exudyn_port_shaft = shaft
    sys._exudyn_port_rotor_visual = rotor
    update_visuals(sys)
    return sys, shaft, rotor


def simulate(duration, step):
    sys, shaft, rotor = build_system()
    while sys.GetChTime() < duration:
        sys.DoStepDynamics(step)
        update_visuals(sys)
    return sys, shaft, rotor


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    sys, shaft, rotor = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ObjectRotationalMass1D.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.35, 0.45, 1.25), chrono.ChVector3d(0, 0, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and sys.GetChTime() < duration:
        t = sys.GetChTime()
        vis.BeginScene()
        update_visuals(sys)
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(step)
        if t >= next_log:
            print_state(sys, shaft)
            next_log += 0.25


def print_state(sys, shaft):
    exact = 1.5 + 0.5 * sys.GetChTime()
    wrapped = math.atan2(math.sin(shaft.GetPos()), math.cos(shaft.GetPos()))
    print(f"t={sys.GetChTime():6.3f}  psi={shaft.GetPos():+.8f}  exact={exact:+.8f}  wrapped={wrapped:+.8f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ObjectRotationalMass1D.py -> PyChrono shaft inertia")
    if args.no_vis:
        sys, shaft, rotor = simulate(args.duration, args.step)
        print_state(sys, shaft)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

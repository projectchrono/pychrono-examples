import argparse
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from ancf_contact_circle import END_TIME, STEP, build_system, print_state, simulate, update_visuals


SOURCE_REFERENCE_TIP_Y = -0.4842698420787613


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _data = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ANCFcontactCircleTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.00, -2.65, 1.05), chrono.ChVector3d(1.00, -0.28, 0.0))
    vis.AddTypicalLights()

    while vis.Run() and system.GetChTime() < duration:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ANCFcontactCircleTest.py -> PyChrono ObjectContactCircleCable2D replay")
    if args.no_vis:
        _system, data = simulate(args.duration, args.step)
        print_state(data["result"])
        print(f"source_reference_tip_y={SOURCE_REFERENCE_TIP_Y:+.12f}")
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

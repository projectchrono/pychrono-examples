import argparse
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
import serial_robot_test as base


# The EXUDYN checkout currently has TestModels/movingGroundRobotTest.py as a
# byte-identical copy of TestModels/serialRobotTest.py. Keep a distinct runnable
# port and title so the inventory source is covered without hiding that fact.


def build_system():
    return base.build_system()


def update_visuals(system):
    base.update_visuals(system)


def simulate(duration, step):
    return base.simulate(duration, step)


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = base.build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1100, 760)
    vis.SetWindowTitle("EXUDYN port: movingGroundRobotTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.72, -0.90, 0.72), chrono.ChVector3d(0.12, 0.0, 0.22))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        base.update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            base.print_state(system)
            next_log += 0.05


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=base.END_TIME)
    parser.add_argument("--step", type=float, default=base.STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: movingGroundRobotTest.py -> PyChrono redundant-coordinate robot replay")
    if args.no_vis:
        system, _items = base.simulate(args.duration, args.step)
        base.print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

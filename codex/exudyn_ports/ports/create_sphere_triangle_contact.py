import argparse
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
import create_sphere_quad_contact as base


# Reproduces the intent of EXUDYN TestModels/createSphereTriangleContact.py.
# The source is the same triangle-vs-quad sphere-contact comparison as
# createSphereQuadContact.py, with only diagnostic scaling/comment differences.


def build_system():
    return base.build_system()


def simulate(duration, step):
    return base.simulate(duration, step)


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, spheres, floors = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: createSphereTriangleContact.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(2.35, -2.55, 1.25), chrono.ChVector3d(0.0, -0.05, 0.08))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            base.print_state(system, spheres)
            next_log += 0.15


def print_state(system, spheres, floors):
    base.print_state(system, spheres)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=base.END_TIME)
    parser.add_argument("--step", type=float, default=base.STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: createSphereTriangleContact.py -> PyChrono trigs-vs-quad contact")
    if args.no_vis:
        system, spheres, floors = simulate(args.duration, args.step)
        print_state(system, spheres, floors)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

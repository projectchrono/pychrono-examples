import argparse
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
import object_generic_ode2 as base


# Reproduces EXUDYN TestModels/MiniExamples/MarkerSuperElementPosition.py.
# The source builds the same two-node ObjectGenericODE2 matrix model as the
# ObjectGenericODE2 mini example, then applies a 10 N x-load through a
# MarkerSuperElementPosition on mesh node 1 with weighting factor 1.


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def add_super_element_marker_visual(items):
    mass1 = items["mass1"]
    mass1.SetName("MarkerSuperElementPosition mesh node 1 loaded mass")
    marker = chrono.ChVisualShapeSphere(0.034)
    marker.SetColor(color(0.96, 0.78, 0.10))
    mass1.AddVisualShape(marker, chrono.ChFramed(chrono.ChVector3d(0.0, 0.13, 0.0)))

    stem = chrono.ChVisualShapeSegment()
    stem.SetColor(color(0.96, 0.78, 0.10))
    stem.SetThickness(4)
    stem.SetLineGeometry(
        chrono.ChLineSegment(
            chrono.ChVector3d(0.0, 0.075, 0.0),
            chrono.ChVector3d(0.0, 0.13, 0.0),
        )
    )
    mass1.AddVisualShape(stem)


def build_system():
    system, items = base.build_system()
    items["ground"].SetName("MarkerSuperElementPosition ground")
    items["mass0"].SetName("MarkerSuperElementPosition mesh node 0 mass")
    items["anchor"].SetName("MarkerSuperElementPosition grounded reference marker")
    items["spring0"].SetName("MarkerSuperElementPosition ground-node0 matrix stiffness/damping")
    items["spring1"].SetName("MarkerSuperElementPosition node0-node1 matrix stiffness/damping")
    add_super_element_marker_visual(items)
    system._marker_super_element_position_items = items
    update_visuals(system)
    return system, items


def update_visuals(system):
    base.update_visuals(system)


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration - 1.0e-12:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: MarkerSuperElementPosition.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.12, -3.05, 1.18), chrono.ChVector3d(0.12, 0.0, 0.0))
    vis.AddTypicalLights()

    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def print_result(duration, step):
    u0, u1, v0, v1 = base.solve(duration, step)
    source_position_x = base.MASS1_X + u1
    static_position_x = base.REFERENCE_SOURCE_POSITION_X
    print(
        f"marker_super_element_position: t={duration:7.3f}  "
        f"node0_u={u0:+.12f}  node1_u={u1:+.12f}  "
        f"node1_v={v1:+.12e}"
    )
    print(
        f"marker_super_element_position: marker_mesh_node=1  weighting=1.000000000  "
        f"load=(+10.000000000,+0.000000000,+0.000000000)  "
        f"source_position_x={source_position_x:+.12f}  static_position_x={static_position_x:+.12f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=base.END_TIME)
    parser.add_argument("--step", type=float, default=base.STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: MarkerSuperElementPosition.py -> PyChrono super-element marker load analogue")
    if args.no_vis:
        print_result(args.duration, args.step)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

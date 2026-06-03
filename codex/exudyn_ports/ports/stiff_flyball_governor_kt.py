import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
import stiff_flyball_governor2 as body_port


# Reproduces the intent of EXUDYN Examples/stiffFlyballGovernorKT.py:
# a four-link kinematic-tree IFToMM flyball governor with a revolute shaft,
# prismatic slider, two revolute-Y flyball rods, and two slider-to-rod
# spring-dampers.  Chrono realizes the same tree topology with explicit
# constrained bodies so every link and both springs have inspectable visuals.

SOURCE_OMEGA_Z0 = 2.0 * math.pi
SOURCE_SPRING_K = 8.0e5
SOURCE_SPRING_C = 4.0e4
SOURCE_SPRING_L0 = 0.5
DEFAULT_STEP = 1.0e-4
DEFAULT_DURATION = 0.25

# Match the source kinematic-tree initial spin.  The spring coefficients are
# scaled like the existing Chrono flyball port to keep the explicit constrained
# body realization stable while preserving the source spring geometry.
body_port.OMEGA_Z0 = SOURCE_OMEGA_Z0
body_port.SPRING_K = SOURCE_SPRING_K * 0.005
body_port.SPRING_C = SOURCE_SPRING_C * 0.005
body_port.SPRING_L0 = SOURCE_SPRING_L0


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def add_background(system):
    plate = chrono.ChBodyEasyBox(2.0, 0.025, 2.0, 1000, True, False)
    plate.SetName("stiffFlyballGovernorKT blue source background")
    plate.SetFixed(True)
    plate.SetPos(chrono.ChVector3d(0, 0.64, 0.56))
    plate.GetVisualShape(0).SetColor(color(0.10, 0.10, 0.80))
    plate.GetVisualShape(0).SetOpacity(0.22)
    system.AddBody(plate)


def build_system():
    system, shaft, slider, rod_ac, rod_bd, springs = body_port.build_system()
    shaft.SetName("stiffFlyballGovernorKT revolute-Z shaft link")
    slider.SetName("stiffFlyballGovernorKT prismatic-Z slider link")
    rod_ac.SetName("stiffFlyballGovernorKT rod AC revolute-Y link")
    rod_bd.SetName("stiffFlyballGovernorKT rod BD revolute-Y link")
    springs[0].SetName("stiffFlyballGovernorKT spring E to rod AC")
    springs[1].SetName("stiffFlyballGovernorKT spring F to rod BD")
    add_background(system)
    body_port.update_visuals(system)
    return system, shaft, slider, rod_ac, rod_bd, springs


def simulate(duration, step):
    system, shaft, slider, rod_ac, rod_bd, springs = build_system()
    while system.GetChTime() < duration:
        body_port.update_visuals(system)
        system.DoStepDynamics(step)
    body_port.update_visuals(system)
    return system, shaft, slider, rod_ac, rod_bd, springs


def spring_lengths(springs):
    return tuple(spring.GetLength() for spring in springs)


def print_state(system, shaft, slider, rod_ac, rod_bd, springs):
    ball_ac = rod_ac.TransformPointLocalToParent(rod_ac._flyball_ball_local)
    ball_bd = rod_bd.TransformPointLocalToParent(rod_bd._flyball_ball_local)
    lengths = spring_lengths(springs)
    print(
        f"t={system.GetChTime():6.3f}  "
        f"slider_z={slider.GetPos().z:+.6f}  "
        f"shaft_wz={shaft.GetAngVelLocal().z:+.6f}  "
        f"spring_lengths=({lengths[0]:+.6f},{lengths[1]:+.6f})  "
        f"ball_r=({math.hypot(ball_ac.x, ball_ac.y):+.5f},{math.hypot(ball_bd.x, ball_bd.y):+.5f})"
    )


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, shaft, slider, rod_ac, rod_bd, springs = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: stiffFlyballGovernorKT.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.65, -2.05, 1.35), chrono.ChVector3d(0, 0, 0.58))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        body_port.update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, shaft, slider, rod_ac, rod_bd, springs)
            next_log += 0.05


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION)
    parser.add_argument("--step", type=float, default=DEFAULT_STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: stiffFlyballGovernorKT.py -> PyChrono kinematic-tree flyball governor")
    print(
        f"source parameters: omega0_z={SOURCE_OMEGA_Z0:.9f}, "
        f"k={SOURCE_SPRING_K:.1f}, c={SOURCE_SPRING_C:.1f}, l0={SOURCE_SPRING_L0:.3f}"
    )
    if args.no_vis:
        system, shaft, slider, rod_ac, rod_bd, springs = simulate(args.duration, args.step)
        print_state(system, shaft, slider, rod_ac, rod_bd, springs)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

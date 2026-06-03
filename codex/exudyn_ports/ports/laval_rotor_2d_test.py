import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from laval_rotor_common import (
    add_bearing_bushing,
    add_bearing_spring_visual,
    color,
    make_reference_line,
    make_support,
    prepare_step,
)


# Reproduces the intent of EXUDYN Examples/lavalRotor2Dtest.py:
# a planar Laval rotor with eccentric mass center, a Cartesian bearing
# spring-damper support, and forward/backward whirl excitation options. Chrono
# uses a 3D rigid body restricted by the bearing bushing; the bearing spring is
# represented by a visible offset coil rather than a rod.

MASS = 1.6
DISC_RADIUS = 0.5
VIS_DISC_RADIUS = 0.16
VIS_DISC_THICKNESS = 0.08
STIFFNESS = 4000.0
OMEGA0 = math.sqrt(STIFFNESS / MASS)
DAMPING_RATIO = 0.005
DAMPING = 2.0 * OMEGA0 * DAMPING_RATIO * MASS
EPS = 1.0e-2
OMEGA_INITIAL = 0.5 * OMEGA0
LOAD_AMPLITUDE = 0.35
STEP = 1e-3


def add_disc_visual(body):
    disc = chrono.ChVisualShapeCylinder(VIS_DISC_RADIUS, VIS_DISC_THICKNESS)
    disc.SetColor(color(0.86, 0.18, 0.12))
    body.AddVisualShape(disc, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleY(0.5 * math.pi)))

    shaft_marker = chrono.ChVisualShapeSphere(0.022)
    shaft_marker.SetColor(color(0.95, 0.72, 0.08))
    body.AddVisualShape(shaft_marker, chrono.ChFramed(chrono.ChVector3d(0, -EPS, 0)))

    spoke = chrono.ChVisualShapeBox(2.0 * VIS_DISC_RADIUS, 0.012, 0.012)
    spoke.SetColor(color(0.10, 0.10, 0.10))
    body.AddVisualShape(spoke, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))


def add_marker(body, local_position, radius, tint):
    marker = chrono.ChVisualShapeSphere(radius)
    marker.SetColor(tint)
    body.AddVisualShape(marker, chrono.ChFramed(local_position))


def build_system(mode="fw"):
    sign = -1.0 if mode == "fw" else 1.0
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    support = make_support(system, "2D Laval bearing support", chrono.ChVector3d(0, 0, 0), 0.08)
    make_reference_line(system, "2D Laval orbit reference x", chrono.ChVector3d(0, -0.24, 0), 0.55)

    rotor = chrono.ChBody()
    rotor.SetName("2D Laval rotor")
    rotor.SetMass(MASS)
    rotor.SetInertiaXX(chrono.ChVector3d(0.25 * MASS * DISC_RADIUS**2, 0.25 * MASS * DISC_RADIUS**2, MASS * DISC_RADIUS**2))
    rotor.SetPos(chrono.ChVector3d(0, EPS, 0))
    rotor.SetAngVelLocal(chrono.ChVector3d(0, 0, OMEGA_INITIAL))
    rotor.EnableCollision(False)
    add_disc_visual(rotor)
    add_marker(rotor, chrono.ChVector3d(0, 0.18, 0.04), 0.018, color(0.95, 0.72, 0.08))
    system.AddBody(rotor)

    bushing = add_bearing_bushing(
        system,
        support,
        rotor,
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(STIFFNESS, STIFFNESS, 0.0),
        chrono.ChVector3d(DAMPING, DAMPING, 0.0),
    )
    visual_spring = add_bearing_spring_visual(
        system,
        support,
        rotor,
        chrono.ChVector3d(0, 0.18, 0.04),
        chrono.ChVector3d(0, 0.40, 0.04),
        radius=0.028,
    )
    add_marker(support, chrono.ChVector3d(0, 0.40, 0.04), 0.022, color(0.08, 0.08, 0.08))

    load_container = chrono.ChLoadContainer()
    system.Add(load_container)
    force = chrono.ChLoadBodyForce(
        rotor,
        chrono.ChVector3d(0, 0, 0),
        False,
        chrono.ChVector3d(0, -EPS, 0),
        True,
    )
    load_container.Add(force)

    system._laval_2d_data = {
        "mode": mode,
        "sign": sign,
        "force": force,
        "rotor": rotor,
    }
    update_forces(system, rotor)
    return system, rotor, bushing, visual_spring, force


def update_forces(system, rotor):
    data = getattr(system, "_laval_2d_data", None)
    if data is None:
        return
    phi = rotor.GetRot().GetCardanAnglesXYZ().z
    data["force"].SetForce(
        chrono.ChVector3d(
            LOAD_AMPLITUDE * math.sin(OMEGA_INITIAL * system.GetChTime()),
            data["sign"] * LOAD_AMPLITUDE * math.cos(phi),
            0,
        ),
        False,
    )


def update_visuals(system, rotor=None):
    if rotor is None:
        rotor = system._laval_2d_data["rotor"]
    update_forces(system, rotor)
    prepare_step(system)


def simulate(duration, step, mode):
    system, rotor, bushing, visual_spring, force = build_system(mode)
    while system.GetChTime() < duration:
        update_visuals(system, rotor)
        system.DoStepDynamics(step)
    update_visuals(system, rotor)
    return system, rotor, bushing, visual_spring, force


def run_visual(duration, step, mode):
    import pychrono.irrlicht as chronoirr

    system, rotor, bushing, visual_spring, force = build_system(mode)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: lavalRotor2Dtest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.42, 0.55, 1.25), chrono.ChVector3d(0.0, 0.04, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        update_visuals(system, rotor)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, rotor, force)
            next_log += 0.5


def print_state(system, rotor, force):
    shaft = rotor.TransformPointLocalToParent(chrono.ChVector3d(0, -EPS, 0))
    omega = rotor.GetAngVelLocal()
    applied = force.GetForce()
    print(
        f"t={system.GetChTime():6.3f}  "
        f"shaft=({shaft.x:+.5f}, {shaft.y:+.5f}, {shaft.z:+.5f})  "
        f"omega_z={omega.z:+.4f}  "
        f"force=({applied.x:+.3f}, {applied.y:+.3f}, {applied.z:+.3f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--mode", choices=("fw", "bw"), default="fw")
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: lavalRotor2Dtest.py -> PyChrono planar Laval rotor")
    if args.no_vis:
        system, rotor, bushing, visual_spring, force = simulate(args.duration, args.step, args.mode)
        print_state(system, rotor, force)
    else:
        run_visual(args.duration, args.step, args.mode)


if __name__ == "__main__":
    main()

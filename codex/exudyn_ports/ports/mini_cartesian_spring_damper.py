import argparse
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN
# TestModels/MiniExamples/ObjectConnectorCartesianSpringDamper.py:
# a 5 kg mass connected to ground by a Cartesian spring-damper and loaded by
# a constant -y force. The physical connector is a Chrono bushing; the offset
# TSDA below is visual-only so the spring is visible as a coil in screenshots.

ANCHOR = chrono.ChVector3d(1, 1, 0)
MASS = 5.0
STIFFNESS = 5000.0
DAMPING_Y = STIFFNESS * 0.05
LOAD_Y = -5.0
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def diagonal_matrix(values):
    matrix = chrono.ChMatrix66d()
    matrix.SetZero()
    for i, value in enumerate(values):
        matrix.SetItem(i, i, value)
    return matrix


def update_visuals(sys):
    update_system_visuals(sys)


def build_system():
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    ground = chrono.ChBody()
    ground.SetFixed(True)
    sys.AddBody(ground)

    mass = chrono.ChBodyEasySphere(0.055, 1000, True, False)
    mass.SetMass(MASS)
    mass.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    mass.SetPos(ANCHOR)
    mass.GetVisualShape(0).SetColor(color(0.10, 0.35, 0.90))
    sys.AddBody(mass)

    k = diagonal_matrix([STIFFNESS, STIFFNESS, STIFFNESS, 0, 0, 0])
    r = diagonal_matrix([0, DAMPING_Y, 0, 0, 0, 0])
    bushing = chrono.ChLinkBushing()
    bushing.Initialize(ground, mass, chrono.ChFramed(ANCHOR), k, r)
    sys.AddLink(bushing)

    force = chrono.ChForce()
    force.SetF_y(chrono.ChFunctionConst(LOAD_Y))
    mass.AddForce(force)

    anchor = chrono.ChBodyEasySphere(0.035, 1000, True, False)
    anchor.SetFixed(True)
    anchor.SetPos(ANCHOR)
    anchor.GetVisualShape(0).SetColor(color(0.12, 0.12, 0.12))
    sys.AddBody(anchor)

    visual_spring = chrono.ChLinkTSDA()
    visual_spring.Initialize(
        mass,
        ground,
        True,
        chrono.ChVector3d(0.14, 0, 0),
        chrono.ChVector3d(ANCHOR.x + 0.14, ANCHOR.y + 0.30, ANCHOR.z),
    )
    visual_spring.SetRestLength(0.30)
    visual_spring.SetSpringCoefficient(0)
    visual_spring.SetDampingCoefficient(0)
    sys.AddLink(visual_spring)
    spring_shape = chrono.ChVisualShapeSpring(0.035, 70, 9)
    spring_shape.SetColor(color(0.85, 0.18, 0.12))
    visual_spring.AddVisualShape(spring_shape)
    attach_spring_visual(sys, visual_spring, 0.035, 70, 9, color(0.85, 0.18, 0.12))

    rail = chrono.ChBodyEasyBox(0.02, 0.72, 0.02, 1000, True, False)
    rail.SetFixed(True)
    rail.SetPos(chrono.ChVector3d(ANCHOR.x - 0.12, ANCHOR.y - 0.05, ANCHOR.z))
    rail.GetVisualShape(0).SetColor(color(0.48, 0.48, 0.48))
    sys.AddBody(rail)

    return sys, mass, bushing, visual_spring


def simulate(duration, step):
    sys, mass, bushing, visual_spring = build_system()
    while sys.GetChTime() < duration:
        sys.DoStepDynamics(step)
    return sys, mass, bushing, visual_spring


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    sys, mass, bushing, visual_spring = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ObjectConnectorCartesianSpringDamper.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.0, 1.25, 1.45), chrono.ChVector3d(1.0, 0.92, 0))
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
            print_state(sys, mass, bushing)
            next_log += 0.25


def print_state(sys, mass, bushing):
    displacement_y = mass.GetPos().y - ANCHOR.y
    reference = LOAD_Y / STIFFNESS
    force = bushing.GetForce()
    print(
        f"t={sys.GetChTime():6.3f}  uy={displacement_y:+.8f}  "
        f"static={reference:+.8f}  bushing_force_y={force.y:+.4f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print(
        "EXUDYN port: ObjectConnectorCartesianSpringDamper.py -> "
        "PyChrono Cartesian bushing spring-damper"
    )
    if args.no_vis:
        sys, mass, bushing, _ = simulate(args.duration, args.step)
        print_state(sys, mass, bushing)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

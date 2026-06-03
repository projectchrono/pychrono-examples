import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN
# TestModels/createRollingDiscPenaltyTest.py: a simple two-wheeler without
# steering, with two rolling wheels supporting a rigid board. EXUDYN uses
# CreateRollingDiscPenalty; this PyChrono port uses high-friction rigid contact.

RADIUS = 0.1
WIDTH = 0.01
BOARD_LENGTH = 0.8
OMEGA_Y = 4.0 * 2.0 * math.pi
STEP = 2e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_material(friction=0.7, restitution=0.02):
    mat = chrono.ChContactMaterialSMC()
    mat.SetFriction(friction)
    mat.SetRollingFriction(0.001)
    mat.SetSpinningFriction(0.001)
    mat.SetRestitution(restitution)
    return mat


def add_wheel_visual(wheel, tint):
    wheel.GetVisualShape(0).SetColor(tint)
    spoke = chrono.ChVisualShapeBox(1.8 * RADIUS, 0.012, 0.012)
    spoke.SetColor(color(0.95, 0.95, 0.95))
    wheel.AddVisualShape(spoke)


def build_system():
    sys = chrono.ChSystemSMC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    wheel_mat = make_material(0.8, 0.02)
    ground_mat = make_material(0.8, 0.02)

    ground = chrono.ChBodyEasyBox(6.0, 3.0, 0.06, 1000, True, True, ground_mat)
    ground.SetFixed(True)
    ground.SetPos(chrono.ChVector3d(0, 0, -0.03))
    ground.GetVisualShape(0).SetColor(color(0.42, 0.44, 0.46))
    ground.GetVisualShape(0).SetOpacity(0.45)
    sys.AddBody(ground)

    board = chrono.ChBodyEasyBox(1.1 * BOARD_LENGTH, 0.06, 0.06, 2800, True, False)
    board.SetPos(chrono.ChVector3d(0, 0, 1.20 * RADIUS))
    board.SetPosDt(chrono.ChVector3d(OMEGA_Y * RADIUS, 0, 0))
    board.GetVisualShape(0).SetColor(color(0.50, 0.50, 0.52))
    sys.AddBody(board)

    wheels = []
    for x, tint in [(-0.5 * BOARD_LENGTH, color(0.12, 0.35, 0.85)), (0.5 * BOARD_LENGTH, color(0.10, 0.62, 0.25))]:
        wheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, RADIUS, WIDTH, 5000, True, True, wheel_mat)
        wheel.SetPos(chrono.ChVector3d(x, 0, RADIUS))
        wheel.SetPosDt(chrono.ChVector3d(OMEGA_Y * RADIUS, 0, 0))
        wheel.SetAngVelParent(chrono.ChVector3d(0, OMEGA_Y, 0))
        add_wheel_visual(wheel, tint)
        sys.AddBody(wheel)
        wheels.append(wheel)

        joint = chrono.ChLinkLockRevolute()
        joint.Initialize(wheel, board, chrono.ChFramed(chrono.ChVector3d(x, 0, RADIUS), chrono.Q_ROTATE_Z_TO_Y))
        sys.AddLink(joint)

    return sys, board, wheels


def simulate(duration, step):
    sys, board, wheels = build_system()
    while sys.GetChTime() < duration:
        sys.DoStepDynamics(step)
    return sys, board, wheels


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    sys, board, wheels = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: createRollingDiscPenaltyTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.5, -2.2, 0.9), chrono.ChVector3d(0.2, 0, 0.12))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and sys.GetChTime() < duration:
        t = sys.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(step)
        if t >= next_log:
            print_state(sys, board, wheels)
            next_log += 0.25


def print_state(sys, board, wheels):
    p0 = wheels[0].GetPos()
    pb = board.GetPos()
    print(
        f"t={sys.GetChTime():6.3f}  board_x={pb.x:+.4f}  "
        f"wheel0=({p0.x:+.4f},{p0.z:+.4f})  speed={board.GetPosDt().x:+.4f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print(
        "EXUDYN port: createRollingDiscPenaltyTest.py -> "
        "PyChrono contact two-wheeler"
    )
    if args.no_vis:
        sys, board, wheels = simulate(args.duration, args.step)
        print_state(sys, board, wheels)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

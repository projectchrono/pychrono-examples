import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/createRollingDiscTest.py:
# a single wheel rolling on a horizontal plane. EXUDYN uses its ideal
# ObjectJointRollingDisc connector; this PyChrono reproduction uses a high
# friction rigid cylinder contact model.

RADIUS = 0.2
WIDTH = 0.02
OMEGA_X = -3.0 * 2.0 * math.pi
OMEGA_Y = 10.0
STEP = 2e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_material(friction=0.8, restitution=0.02):
    mat = chrono.ChContactMaterialNSC()
    mat.SetFriction(friction)
    mat.SetRollingFriction(0.002)
    mat.SetSpinningFriction(0.002)
    mat.SetRestitution(restitution)
    return mat


def add_spokes(wheel):
    spoke_a = chrono.ChVisualShapeBox(0.01, 1.95 * RADIUS, 0.012)
    spoke_a.SetColor(color(0.95, 0.95, 0.95))
    wheel.AddVisualShape(spoke_a)
    spoke_b = chrono.ChVisualShapeBox(0.01, 0.012, 1.95 * RADIUS)
    spoke_b.SetColor(color(0.95, 0.95, 0.95))
    wheel.AddVisualShape(spoke_b)


def build_system():
    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    ground_mat = make_material(0.9, 0.02)
    wheel_mat = make_material(0.9, 0.02)

    ground = chrono.ChBodyEasyBox(8.0, 8.0, 0.06, 1000, True, True, ground_mat)
    ground.SetFixed(True)
    ground.SetPos(chrono.ChVector3d(0, 0, -0.03))
    ground.GetVisualShape(0).SetColor(color(0.42, 0.44, 0.46))
    ground.GetVisualShape(0).SetOpacity(0.45)
    sys.AddBody(ground)

    wheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_X, RADIUS, WIDTH, 5000, True, True, wheel_mat)
    wheel.SetPos(chrono.ChVector3d(1, 0, RADIUS))
    wheel.SetPosDt(chrono.ChVector3d(0, -RADIUS * OMEGA_X, 0))
    wheel.SetAngVelParent(chrono.ChVector3d(OMEGA_X, OMEGA_Y, 0))
    wheel.GetVisualShape(0).SetColor(color(0.10, 0.32, 0.85))
    add_spokes(wheel)
    sys.AddBody(wheel)

    return sys, wheel


def simulate(duration, step):
    sys, wheel = build_system()
    while sys.GetChTime() < duration:
        sys.DoStepDynamics(step)
    return sys, wheel


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    sys, wheel = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: createRollingDiscTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(2.0, -2.8, 1.35), chrono.ChVector3d(1.0, 0.0, 0.20))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and sys.GetChTime() < duration:
        t = sys.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(step)
        if t >= next_log:
            print_state(sys, wheel)
            next_log += 0.25


def print_state(sys, wheel):
    pos = wheel.GetPos()
    omega = wheel.GetAngVelParent()
    print(
        f"t={sys.GetChTime():6.3f}  "
        f"pos=({pos.x:+.4f}, {pos.y:+.4f}, {pos.z:+.4f})  "
        f"|pos|={pos.Length():.6f}  omega=({omega.x:+.3f},{omega.y:+.3f},{omega.z:+.3f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: createRollingDiscTest.py -> PyChrono contact rolling disc")
    if args.no_vis:
        sys, wheel = simulate(args.duration, args.step)
        print_state(sys, wheel)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

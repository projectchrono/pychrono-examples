import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/rollingCoinPenaltyTest.py:
# a slightly tilted thin coin rolling on a plane with dry friction represented
# by a penalty contact formulation. This PyChrono port uses NSC contact because
# it is more robust for the very thin 1 mm coin geometry in the local renderer.

PHI0 = 5.0 / 180.0 * math.pi
RADIUS = 0.01
WIDTH = 0.001
MASS = 1.0
OMEGA0 = 40.0
STEP = 1e-4


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_material():
    material = chrono.ChContactMaterialNSC()
    material.SetFriction(0.8)
    material.SetRollingFriction(0.2)
    material.SetSpinningFriction(0.02)
    material.SetRestitution(0.01)
    return material


def add_spoke(coin):
    spoke = chrono.ChVisualShapeBox(WIDTH * 1.4, 0.70 * RADIUS, 0.70 * RADIUS)
    spoke.SetColor(color(0.95, 0.35, 0.25))
    coin.AddVisualShape(spoke)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    material = make_material()

    ground = chrono.ChBodyEasyBox(0.30, 0.30, 0.002, 1000, True, True, material)
    ground.SetFixed(True)
    ground.SetPos(chrono.ChVector3d(0, 0, -0.001))
    ground.GetVisualShape(0).SetColor(color(0.82, 0.82, 0.82))
    ground.GetVisualShape(0).SetOpacity(0.65)
    system.AddBody(ground)

    coin = chrono.ChBodyEasyCylinder(chrono.ChAxis_X, RADIUS, WIDTH, 7800, True, True, material)
    coin.SetMass(MASS)
    coin.SetInertiaXX(chrono.ChVector3d(0.5 * MASS * RADIUS**2, 0.25 * MASS * RADIUS**2, 0.25 * MASS * RADIUS**2))
    coin.SetPos(chrono.ChVector3d(RADIUS * math.sin(PHI0), 0, RADIUS * math.cos(PHI0) + 0.01))
    coin.SetRot(chrono.QuatFromAngleY(PHI0))
    coin.SetAngVelParent(chrono.ChVector3d(OMEGA0, 0, 0))
    coin.SetPosDt(chrono.ChVector3d(0, -OMEGA0 * RADIUS * math.cos(PHI0), 0))
    coin.GetVisualShape(0).SetColor(color(0.12, 0.35, 0.88))
    add_spoke(coin)
    system.AddBody(coin)

    return system, coin


def simulate(duration, step):
    system, coin = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, coin


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, coin = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: rollingCoinPenaltyTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.08, -0.12, 0.08), chrono.ChVector3d(0.02, 0, 0.01))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, coin)
            next_log += 0.1


def print_state(system, coin):
    pos = coin.GetPos()
    omega = coin.GetAngVelParent()
    print(
        f"t={system.GetChTime():6.3f}  "
        f"pos=({pos.x:+.6f}, {pos.y:+.6f}, {pos.z:+.6f})  "
        f"omega=({omega.x:+.3f}, {omega.y:+.3f}, {omega.z:+.3f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0.5)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: rollingCoinPenaltyTest.py -> PyChrono thin rolling coin contact")
    if args.no_vis:
        system, coin = simulate(args.duration, args.step)
        print_state(system, coin)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

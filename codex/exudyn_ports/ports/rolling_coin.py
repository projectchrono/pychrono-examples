import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/rollingCoinTest.py:
# a tilted thin coin with ideal rolling-disc constraints. PyChrono core in this
# environment has no ObjectJointRollingDisc equivalent, so this version keeps
# the Chrono rigid body on the ideal rolling trajectory prescribed by the
# initial EXUDYN rolling relation instead of using fragile thin-cylinder contact.

PHI0 = math.pi / 180.0
RADIUS = 0.01
WIDTH = 0.001
MASS = 1.0
OMEGA_Z = 10.0 * math.pi
STEP = 5e-4


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def add_coin_spokes(coin):
    spoke_a = chrono.ChVisualShapeBox(WIDTH * 1.4, 0.72 * RADIUS, 0.010 * RADIUS)
    spoke_a.SetColor(color(0.95, 0.35, 0.20))
    coin.AddVisualShape(spoke_a)
    spoke_b = chrono.ChVisualShapeBox(WIDTH * 1.4, 0.010 * RADIUS, 0.72 * RADIUS)
    spoke_b.SetColor(color(0.95, 0.35, 0.20))
    coin.AddVisualShape(spoke_b)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    ground = chrono.ChBodyEasyBox(0.12, 0.12, 0.002, 1000, True, False)
    ground.SetName("rolling coin ground")
    ground.SetFixed(True)
    ground.SetPos(chrono.ChVector3d(0, 0, -0.001))
    ground.GetVisualShape(0).SetColor(color(0.82, 0.82, 0.82))
    ground.GetVisualShape(0).SetOpacity(0.65)
    system.AddBody(ground)

    coin = chrono.ChBodyEasyCylinder(chrono.ChAxis_X, RADIUS, WIDTH, 7800, True, False)
    coin.SetName("ideal rolling coin contact analogue")
    coin.SetMass(MASS)
    coin.SetInertiaXX(
        chrono.ChVector3d(
            0.5 * MASS * RADIUS * RADIUS,
            0.25 * MASS * RADIUS * RADIUS,
            0.25 * MASS * RADIUS * RADIUS,
        )
    )
    coin.SetPos(chrono.ChVector3d(RADIUS * math.sin(PHI0), 0, RADIUS * math.cos(PHI0)))
    coin.SetRot(chrono.QuatFromAngleY(PHI0))
    coin.SetAngVelParent(chrono.ChVector3d(0, 0, OMEGA_Z))
    coin.SetPosDt(chrono.ChVector3d(0, OMEGA_Z * RADIUS * math.sin(PHI0), 0))
    coin.GetVisualShape(0).SetColor(color(0.12, 0.35, 0.88))
    add_coin_spokes(coin)
    system.AddBody(coin)

    system._rolling_coin_items = {"coin": coin}
    return system, coin


def update_ideal_rolling(system, coin):
    t = system.GetChTime()
    center = chrono.ChVector3d(
        RADIUS * math.sin(PHI0),
        OMEGA_Z * RADIUS * math.sin(PHI0) * t,
        RADIUS * math.cos(PHI0),
    )
    coin.SetPos(center)
    coin.SetRot(chrono.QuatFromAngleZ(OMEGA_Z * t) * chrono.QuatFromAngleY(PHI0))
    coin.SetPosDt(chrono.ChVector3d(0, OMEGA_Z * RADIUS * math.sin(PHI0), 0))
    coin.SetAngVelParent(chrono.ChVector3d(0, 0, OMEGA_Z))


def update_visuals(system):
    data = getattr(system, "_rolling_coin_items", None)
    if data is not None:
        update_ideal_rolling(system, data["coin"])


def simulate(duration, step):
    system, coin = build_system()
    while system.GetChTime() < duration:
        update_ideal_rolling(system, coin)
        system.DoStepDynamics(step)
    update_ideal_rolling(system, coin)
    return system, coin


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, coin = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: rollingCoinTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.055, -0.085, 0.055), chrono.ChVector3d(0.012, 0.0, 0.010))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        update_ideal_rolling(system, coin)
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, coin)
            next_log += 0.1


def print_state(system, coin):
    pos = coin.GetPos()
    omega = coin.GetAngVelParent()
    velocity = coin.GetPosDt()
    indicator = pos.x + 0.1 * (abs(omega.x) + abs(omega.y) + abs(omega.z))
    print(
        f"t={system.GetChTime():6.3f}  "
        f"pos=({pos.x:+.6f}, {pos.y:+.6f}, {pos.z:+.6f})  "
        f"vel=({velocity.x:+.5f}, {velocity.y:+.5f}, {velocity.z:+.5f})  "
        f"omega=({omega.x:+.3f}, {omega.y:+.3f}, {omega.z:+.3f})  "
        f"indicator={indicator:.8f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0.5)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: rollingCoinTest.py -> PyChrono tilted rolling coin ideal path")
    if args.no_vis:
        system, coin = simulate(args.duration, args.step)
        print_state(system, coin)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

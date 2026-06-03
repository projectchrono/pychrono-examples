import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/fourBarMechanismIftomm.py:
# a non-redundant IFToMM double four-bar mechanism represented with 5 rigid
# bars and seven revolute pin joints.

LENGTH = 1.0
WIDTH = 0.1
MASS = 1.0
INERTIA_Z = MASS * LENGTH * LENGTH / 12.0
V0 = 1.0
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_bar(name, x, y, theta, velocity, omega, tint):
    body = chrono.ChBodyEasyBox(LENGTH, WIDTH, WIDTH, 1000, True, False)
    body.SetName(name)
    body.SetMass(MASS)
    body.SetInertiaXX(chrono.ChVector3d(0.002, INERTIA_Z, INERTIA_Z))
    body.SetPos(chrono.ChVector3d(x, y, 0))
    body.SetRot(chrono.QuatFromAngleZ(theta))
    body.SetPosDt(chrono.ChVector3d(velocity[0], velocity[1], 0))
    body.SetAngVelLocal(chrono.ChVector3d(0, 0, omega))
    body.GetVisualShape(0).SetColor(tint)
    return body


def add_revolute(system, body_a, body_b, point):
    joint = chrono.ChLinkLockRevolute()
    joint.Initialize(body_a, body_b, chrono.ChFramed(point, chrono.QUNIT))
    system.AddLink(joint)
    return joint


def add_pin(system, point):
    pin = chrono.ChBodyEasySphere(0.045, 1000, True, False)
    pin.SetFixed(True)
    pin.SetPos(point)
    pin.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.08))
    system.AddBody(pin)
    return pin


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)

    refs = [
        (0, 0.5 * LENGTH, 0.5 * math.pi),
        (0.5 * LENGTH, LENGTH, 0),
        (LENGTH, 0.5 * LENGTH, 0.5 * math.pi),
        (1.5 * LENGTH, LENGTH, 0),
        (2 * LENGTH, 0.5 * LENGTH, 0.5 * math.pi),
    ]
    velocities = [
        (0.5 * V0, 0, -V0 / LENGTH),
        (V0, 0, 0),
        (0.5 * V0, 0, -V0 / LENGTH),
        (V0, 0, 0),
        (0.5 * V0, 0, -V0 / LENGTH),
    ]
    tints = [
        color(0.80, 0.18, 0.12),
        color(0.12, 0.42, 0.85),
        color(0.10, 0.65, 0.25),
        color(0.75, 0.18, 0.72),
        color(0.95, 0.72, 0.05),
    ]
    bars = []
    for i, ((x, y, theta), (vx, vy, omega), tint) in enumerate(zip(refs, velocities, tints)):
        bar = make_bar(f"iftomm bar {i}", x, y, theta, (vx, vy), omega, tint)
        system.AddBody(bar)
        bars.append(bar)

    ground_bar = chrono.ChBodyEasyBox(2.15, 0.04, 0.04, 1000, True, False)
    ground_bar.SetFixed(True)
    ground_bar.SetPos(chrono.ChVector3d(1.0, -0.06, 0))
    ground_bar.GetVisualShape(0).SetColor(color(0.40, 0.40, 0.40))
    system.AddBody(ground_bar)

    points = {
        "g0": chrono.ChVector3d(0, 0, 0),
        "g1": chrono.ChVector3d(1, 0, 0),
        "g2": chrono.ChVector3d(2, 0, 0),
        "p01": chrono.ChVector3d(0, 1, 0),
        "p123": chrono.ChVector3d(1, 1, 0),
        "p34": chrono.ChVector3d(2, 1, 0),
    }
    joints = [
        add_revolute(system, bars[0], ground, points["g0"]),
        add_revolute(system, bars[2], ground, points["g1"]),
        add_revolute(system, bars[4], ground, points["g2"]),
        add_revolute(system, bars[1], bars[0], points["p01"]),
        add_revolute(system, bars[3], bars[1], points["p123"]),
        add_revolute(system, bars[3], bars[2], points["p123"]),
        add_revolute(system, bars[4], bars[3], points["p34"]),
    ]
    for point in points.values():
        add_pin(system, point)

    return system, bars, joints


def simulate(duration, step):
    system, bars, joints = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, bars, joints


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, bars, joints = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: fourBarMechanismIftomm.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.0, 1.0, 5.2), chrono.ChVector3d(1.0, 0.55, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, bars, joints)
            next_log += 0.5


def total_energy(bars):
    energy = 0.0
    for bar in bars:
        v = bar.GetPosDt()
        omega = bar.GetAngVelLocal()
        energy += 0.5 * MASS * (v.x * v.x + v.y * v.y + v.z * v.z)
        energy += 0.5 * INERTIA_Z * omega.z * omega.z
        energy += MASS * 9.81 * bar.GetPos().y
    return energy


def print_state(system, bars, joints):
    print(
        f"t={system.GetChTime():6.3f}  "
        f"bar0=({bars[0].GetPos().x:+.5f}, {bars[0].GetPos().y:+.5f})  "
        f"energy={total_energy(bars):+.6f}  joints={len(joints)}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: fourBarMechanismIftomm.py -> PyChrono rigid double four-bar")
    if args.no_vis:
        system, bars, joints = simulate(args.duration, args.step)
        print_state(system, bars, joints)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

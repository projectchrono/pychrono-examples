import argparse

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/generalContactImplicit2.py:
# a small sphere is preloaded against a contact floor and launched tangentially
# while normal/friction contact is handled implicitly. Chrono uses native SMC
# contact for the same sphere-floor setup and renders both contact bodies.

LENGTH = 2.5
BASE = 0.1
RADIUS = 0.5 * BASE
MASS = 0.025
GRAVITY = 10.0
CONTACT_STIFFNESS = 1.0e3
CONTACT_DAMPING = 0.2
FRICTION = 0.1
STEP = 2e-4
END_TIME = 2.0


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_material():
    material = chrono.ChContactMaterialSMC()
    material.SetFriction(FRICTION)
    material.SetKn(CONTACT_STIFFNESS)
    material.SetGn(CONTACT_DAMPING)
    material.SetKt(CONTACT_STIFFNESS)
    material.SetGt(CONTACT_DAMPING)
    material.SetRestitution(0.02)
    return material


def build_system():
    system = chrono.ChSystemSMC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -GRAVITY))
    material = make_material()

    floor = chrono.ChBodyEasyBox(LENGTH, LENGTH, 0.025, 1000, False, True, material)
    floor.SetName("implicit contact sliding floor")
    floor.SetFixed(True)
    floor.SetPos(chrono.ChVector3d(0, 0, -0.0125))
    system.AddBody(floor)

    visible_floor = chrono.ChBodyEasyBox(1.0, 1.0, 0.018, 1000, True, False)
    visible_floor.SetName("visible source-scale contact floor patch")
    visible_floor.SetFixed(True)
    visible_floor.SetPos(chrono.ChVector3d(0, 0, -0.009))
    visible_floor.GetVisualShape(0).SetColor(color(0.84, 0.84, 0.68))
    visible_floor.GetVisualShape(0).SetOpacity(0.62)
    system.AddBody(visible_floor)

    rail = chrono.ChBodyEasyBox(1.8, 0.012, 0.012, 1000, True, False)
    rail.SetName("visible sliding direction guide")
    rail.SetFixed(True)
    rail.SetPos(chrono.ChVector3d(0, -0.16, 0.018))
    rail.GetVisualShape(0).SetColor(color(0.16, 0.16, 0.16))
    system.AddBody(rail)

    density = MASS / ((4.0 / 3.0) * chrono.CH_PI * RADIUS**3)
    sphere = chrono.ChBodyEasySphere(RADIUS, density, True, True, material)
    sphere.SetName("preloaded sliding contact sphere")
    sphere.SetMass(MASS)
    preload_z = RADIUS - MASS * GRAVITY / (0.5 * CONTACT_STIFFNESS)
    sphere.SetPos(chrono.ChVector3d(-0.4, 0.0, preload_z))
    sphere.SetPosDt(chrono.ChVector3d(1.0, 0.0, 0.0))
    sphere.GetVisualShape(0).SetColor(color(0.90, 0.12, 0.10))
    system.AddBody(sphere)

    axis = chrono.ChVisualShapeBox(1.8 * RADIUS, 0.010, 0.010)
    axis.SetColor(color(0.08, 0.08, 0.08))
    sphere.AddVisualShape(axis, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))

    return system, sphere, floor


def simulate(duration, step):
    system, sphere, floor = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, sphere, floor


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, sphere, floor = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: generalContactImplicit2.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.45, -0.70, 0.48), chrono.ChVector3d(0.0, 0.0, 0.08))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, sphere)
            next_log += 0.25


def print_state(system, sphere):
    pos = sphere.GetPos()
    vel = sphere.GetPosDt()
    omega = sphere.GetAngVelParent()
    total = abs(pos.x) + abs(pos.y) + abs(pos.z) + abs(omega.x) + abs(omega.y) + abs(omega.z)
    print(
        f"t={system.GetChTime():6.3f}  "
        f"pos=({pos.x:+.6f}, {pos.y:+.6f}, {pos.z:+.6f})  "
        f"vel=({vel.x:+.6f}, {vel.y:+.6f}, {vel.z:+.6f})  coordinate_sum={total:.8f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: generalContactImplicit2.py -> PyChrono SMC sliding sphere contact")
    if args.no_vis:
        system, sphere, floor = simulate(args.duration, args.step)
        print_state(system, sphere)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

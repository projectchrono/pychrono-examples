import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/contactCoordinateTest.py:
# a small mass/sphere settles onto a stiff one-dimensional contact stop. Chrono
# SMC sphere-floor contact provides the same penalty-contact behavior with
# explicit visible contact geometry.

RADIUS = 0.05
MASS = 0.25
GRAVITY = 9.81
CONTACT_STIFFNESS = 2.0e6
CONTACT_DAMPING = 200.0
STEP = 2e-4
END_TIME = 0.25


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_material():
    material = chrono.ChContactMaterialSMC()
    material.SetFriction(0.0)
    material.SetRestitution(0.02)
    material.SetKn(CONTACT_STIFFNESS)
    material.SetGn(CONTACT_DAMPING)
    material.SetKt(CONTACT_STIFFNESS)
    material.SetGt(CONTACT_DAMPING)
    return material


def sphere_density(mass, radius):
    return mass / ((4.0 / 3.0) * math.pi * radius**3)


def build_system():
    system = chrono.ChSystemSMC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -GRAVITY, 0))
    material = make_material()

    floor = chrono.ChBodyEasyBox(0.7, 0.018, 0.22, 1000, True, True, material)
    floor.SetName("contact-coordinate floor")
    floor.SetFixed(True)
    floor.SetPos(chrono.ChVector3d(0, -0.009, 0))
    floor.GetVisualShape(0).SetColor(color(0.55, 0.55, 0.55))
    floor.GetVisualShape(0).SetOpacity(0.75)
    system.AddBody(floor)

    sphere = chrono.ChBodyEasySphere(RADIUS, sphere_density(MASS, RADIUS), True, True, material)
    sphere.SetName("contact-coordinate mass sphere")
    sphere.SetMass(MASS)
    sphere.SetPos(chrono.ChVector3d(0, RADIUS + 0.05, 0))
    sphere.GetVisualShape(0).SetColor(color(0.88, 0.14, 0.10))
    system.AddBody(sphere)

    guide = chrono.ChBodyEasyBox(0.012, 0.30, 0.012, 1000, True, False)
    guide.SetName("visible contact-coordinate height guide")
    guide.SetFixed(True)
    guide.SetPos(chrono.ChVector3d(-0.16, 0.12, 0))
    guide.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.08))
    guide.GetVisualShape(0).SetOpacity(0.35)
    system.AddBody(guide)

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
    vis.SetWindowTitle("EXUDYN port: contactCoordinateTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.45, 0.55, 0.50), chrono.ChVector3d(0, 0.06, 0))
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
            next_log += 0.05


def print_state(system, sphere):
    pos = sphere.GetPos()
    vel = sphere.GetPosDt()
    penetration = max(0.0, RADIUS - pos.y)
    print(
        f"t={system.GetChTime():6.3f}  "
        f"y={pos.y:+.8f}  vy={vel.y:+.8f}  penetration={penetration:+.8f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: contactCoordinateTest.py -> PyChrono SMC contact stop")
    if args.no_vis:
        system, sphere, floor = simulate(args.duration, args.step)
        print_state(system, sphere)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

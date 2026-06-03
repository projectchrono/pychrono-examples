import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/contactSphereSphereTest.py:
# a moving rigid sphere bounces between two fixed spheres using a sphere-sphere
# contact law with restitution. Chrono's SMC contact provides the native
# penalty-contact analogue.

RADIUS = 0.1
MASS = 1.6
Y_INIT = 0.5
VY_INIT = 10.0
CONTACT_STIFFNESS = 1e6
CONTACT_DAMPING = 0.0
RESTITUTION = 0.7
STEP = 4e-4
END_TIME = 2.0

LOWER_CENTER = chrono.ChVector3d(0.0, -2.0 * RADIUS, 0.0)
UPPER_CENTER = chrono.ChVector3d(0.0, 1.0 + 2.0 * RADIUS, 0.0)


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_material():
    material = chrono.ChContactMaterialSMC()
    material.SetFriction(0.0)
    material.SetRestitution(RESTITUTION)
    material.SetKn(CONTACT_STIFFNESS)
    material.SetGn(CONTACT_DAMPING)
    material.SetKt(CONTACT_STIFFNESS)
    material.SetGt(CONTACT_DAMPING)
    return material


def sphere_density(mass, radius):
    return mass / ((4.0 / 3.0) * math.pi * radius**3)


def make_fixed_sphere(system, material, name, position):
    sphere = chrono.ChBodyEasySphere(RADIUS, 1000, True, True, material)
    sphere.SetName(name)
    sphere.SetFixed(True)
    sphere.SetPos(position)
    sphere.GetVisualShape(0).SetColor(color(0.12, 0.62, 0.24))
    system.AddBody(sphere)
    return sphere


def build_system():
    system = chrono.ChSystemSMC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
    material = make_material()

    lower = make_fixed_sphere(system, material, "lower fixed contact sphere", LOWER_CENTER)
    upper = make_fixed_sphere(system, material, "upper fixed contact sphere", UPPER_CENTER)

    moving = chrono.ChBodyEasySphere(RADIUS, sphere_density(MASS, RADIUS), True, True, material)
    moving.SetName("moving contact test sphere")
    moving.SetMass(MASS)
    moving.SetPos(chrono.ChVector3d(0, Y_INIT, 0))
    moving.SetPosDt(chrono.ChVector3d(0, VY_INIT, 0))
    moving.GetVisualShape(0).SetColor(color(0.92, 0.46, 0.06))
    system.AddBody(moving)

    guide = chrono.ChBodyEasyBox(0.018, 1.65, 0.018, 1000, True, False)
    guide.SetName("visible sphere-sphere contact travel guide")
    guide.SetFixed(True)
    guide.SetPos(chrono.ChVector3d(-0.22, 0.5, 0))
    guide.GetVisualShape(0).SetColor(color(0.10, 0.10, 0.10))
    guide.GetVisualShape(0).SetOpacity(0.35)
    system.AddBody(guide)

    return system, moving, (lower, upper)


def simulate(duration, step):
    system, moving, fixed = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, moving, fixed


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, moving, fixed = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: contactSphereSphereTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.55, 2.1, 1.05), chrono.ChVector3d(0, 0.5, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, moving)
            next_log += 0.25


def print_state(system, moving):
    pos = moving.GetPos()
    vel = moving.GetPosDt()
    gap_lower = pos.y - LOWER_CENTER.y - 2.0 * RADIUS
    gap_upper = UPPER_CENTER.y - pos.y - 2.0 * RADIUS
    print(
        f"t={system.GetChTime():6.3f}  "
        f"y={pos.y:+.6f}  vy={vel.y:+.6f}  "
        f"gaps=({gap_lower:+.6f}, {gap_upper:+.6f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: contactSphereSphereTest.py -> PyChrono SMC sphere-sphere contact")
    if args.no_vis:
        system, moving, fixed = simulate(args.duration, args.step)
        print_state(system, moving)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

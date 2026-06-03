import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN TestModels/contactSphereSphereTestEAPM.py:
# a moving sphere is slowly loaded into and unloaded from a fixed sphere while an
# adhesive sphere-sphere contact law is active. Chrono's SMC material supports a
# constant adhesive force; a real TSDA drive spring follows the EXUDYN
# set-position loading cycle and is rendered as a coil.

RADIUS = 0.1
MASS = 1.6
CONTACT_STIFFNESS = 1.0e5
CONTACT_DAMPING = 0.001
ADHESION_FORCE = 0.01
DRIVE_STIFFNESS = 1.0e5
DRIVE_DAMPING = 20.0
T_MOVE = 0.5
D_MOVE = 0.002
VISUAL_SPRING_X = 0.24
STEP = 2e-4
END_TIME = 1.0


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def sphere_density(mass, radius):
    return mass / ((4.0 / 3.0) * math.pi * radius**3)


def make_material():
    material = chrono.ChContactMaterialSMC()
    material.SetFriction(0.0)
    material.SetRestitution(0.0)
    material.SetKn(CONTACT_STIFFNESS)
    material.SetGn(CONTACT_DAMPING)
    material.SetKt(CONTACT_STIFFNESS)
    material.SetGt(CONTACT_DAMPING)
    material.SetAdhesion(ADHESION_FORCE)
    return material


def target_distance(time):
    if time < T_MOVE:
        return 2.0 * RADIUS - (D_MOVE / T_MOVE) * time
    return 2.0 * RADIUS - D_MOVE + (D_MOVE / T_MOVE) * (time - T_MOVE)


def build_system():
    system = chrono.ChSystemSMC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
    material = make_material()

    fixed = chrono.ChBodyEasySphere(RADIUS, 1000, True, True, material)
    fixed.SetName("fixed adhesive contact sphere")
    fixed.SetFixed(True)
    fixed.SetPos(chrono.ChVector3d(0, 0, 0))
    fixed.GetVisualShape(0).SetColor(color(0.88, 0.14, 0.10))
    system.AddBody(fixed)

    moving = chrono.ChBodyEasySphere(RADIUS, sphere_density(MASS, RADIUS), True, True, material)
    moving.SetName("moving adhesive contact sphere")
    moving.SetMass(MASS)
    moving.SetPos(chrono.ChVector3d(0, 2.0 * RADIUS, 0))
    moving.GetVisualShape(0).SetColor(color(0.12, 0.60, 0.26))
    system.AddBody(moving)

    drive_spring = chrono.ChLinkTSDA()
    drive_spring.SetName("adhesive contact loading spring")
    drive_spring.Initialize(moving, fixed, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
    drive_spring.SetSpringCoefficient(DRIVE_STIFFNESS)
    drive_spring.SetDampingCoefficient(DRIVE_DAMPING)
    drive_spring.SetRestLength(target_distance(0.0))
    system.AddLink(drive_spring)

    visual_spring = chrono.ChLinkTSDA()
    visual_spring.SetName("visible adhesive contact loading spring")
    visual_spring.Initialize(
        moving,
        fixed,
        True,
        chrono.ChVector3d(VISUAL_SPRING_X, 0, 0),
        chrono.ChVector3d(VISUAL_SPRING_X, 0, 0),
    )
    visual_spring.SetSpringCoefficient(0.0)
    visual_spring.SetDampingCoefficient(0.0)
    visual_spring.SetRestLength(target_distance(0.0))
    system.AddLink(visual_spring)
    spring_shape = chrono.ChVisualShapeSpring(0.035, 80, 9)
    spring_shape.SetColor(color(0.92, 0.46, 0.06))
    visual_spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, visual_spring, 0.035, 80, 9, color(0.92, 0.46, 0.06))

    guide = chrono.ChBodyEasyBox(0.018, 0.45, 0.018, 1000, True, False)
    guide.SetName("visible adhesive contact travel guide")
    guide.SetFixed(True)
    guide.SetPos(chrono.ChVector3d(-0.23, 0.10, 0))
    guide.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.08))
    guide.GetVisualShape(0).SetOpacity(0.35)
    system.AddBody(guide)

    system._adhesive_contact_items = {"drive_spring": drive_spring, "visual_spring": visual_spring}
    update_visuals(system)
    return system, moving, fixed, drive_spring


def update_visuals(system):
    items = getattr(system, "_adhesive_contact_items", None)
    if items is not None:
        distance = target_distance(system.GetChTime())
        items["drive_spring"].SetRestLength(distance)
        items["visual_spring"].SetRestLength(distance)
    update_system_visuals(system)


def simulate(duration, step):
    system, moving, fixed, spring = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, moving, fixed, spring


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, moving, fixed, spring = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: contactSphereSphereTestEAPM.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.55, 0.55, 0.45), chrono.ChVector3d(0, 0.10, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, moving, spring)
            next_log += 0.20


def print_state(system, moving, spring):
    pos = moving.GetPos()
    length = spring.GetLength()
    gap = length - 2.0 * RADIUS
    print(
        f"t={system.GetChTime():6.3f}  "
        f"y={pos.y:+.8f}  gap={gap:+.8f}  "
        f"target={target_distance(system.GetChTime()):.8f}  spring_force={spring.GetForce():+.4f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: contactSphereSphereTestEAPM.py -> PyChrono adhesive sphere contact")
    if args.no_vis:
        system, moving, fixed, spring = simulate(args.duration, args.step)
        print_state(system, moving, spring)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN Examples/tippeTop.py:
# a spinning two-sphere tippe-top shape in frictional floor contact. EXUDYN uses
# GeneralContact with a rigid-body marker for each sphere; this PyChrono port
# uses a single compound rigid body with two sphere contact shapes.

FLOOR_SIZE = 5.0
RADIUS = 0.2
STEM_RADIUS = 0.04
MASS = 0.01
HEIGHT = 2.4 * RADIUS
MAIN_CENTER_LOCAL = chrono.ChVector3d(0, 0, 0.04)
STEM_CENTER_LOCAL = chrono.ChVector3d(0, 0, HEIGHT - RADIUS - STEM_RADIUS + 0.04)
STEP = 2e-4


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_material():
    material = chrono.ChContactMaterialSMC()
    material.SetFriction(0.4)
    material.SetRestitution(0.0)
    material.SetYoungModulus(1e7)
    material.SetKn(1e3)
    material.SetGn(1.0)
    return material


def build_system():
    system = chrono.ChSystemSMC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    material = make_material()

    floor = chrono.ChBodyEasyBox(FLOOR_SIZE, FLOOR_SIZE, 0.04, 1000, True, True, material)
    floor.SetName("tippe top floor")
    floor.SetFixed(True)
    floor.SetPos(chrono.ChVector3d(0, 0, -0.02))
    floor.GetVisualShape(0).SetColor(color(0.78, 0.78, 0.74))
    floor.GetVisualShape(0).SetOpacity(0.55)
    system.AddBody(floor)

    top = chrono.ChBody()
    top.SetName("tippe top compound body")
    top.SetMass(MASS)
    top.SetInertiaXX(chrono.ChVector3d(1.0e-4, 1.0e-4, 5.0e-5))
    top.SetPos(chrono.ChVector3d(0, 0, RADIUS - 0.04))
    top.SetAngVelParent(chrono.ChVector3d(0, 0.4, 200.0))

    body_shape = chrono.ChVisualShapeSphere(RADIUS)
    body_shape.SetColor(color(0.12, 0.42, 0.85))
    top.AddVisualShape(body_shape, chrono.ChFramed(MAIN_CENTER_LOCAL))
    stem_shape = chrono.ChVisualShapeSphere(STEM_RADIUS)
    stem_shape.SetColor(color(0.12, 0.42, 0.85))
    top.AddVisualShape(stem_shape, chrono.ChFramed(STEM_CENTER_LOCAL))
    stripe = chrono.ChVisualShapeBox(0.02, 1.3 * RADIUS, 0.02)
    stripe.SetColor(color(0.95, 0.55, 0.12))
    top.AddVisualShape(stripe, chrono.ChFramed(MAIN_CENTER_LOCAL))

    top.AddCollisionShape(chrono.ChCollisionShapeSphere(material, RADIUS), chrono.ChFramed(MAIN_CENTER_LOCAL))
    top.AddCollisionShape(chrono.ChCollisionShapeSphere(material, STEM_RADIUS), chrono.ChFramed(STEM_CENTER_LOCAL))
    top.EnableCollision(True)
    system.AddBody(top)

    return system, top


def simulate(duration, step):
    system, top = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, top


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, top = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: tippeTop.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.0, -1.6, 0.85), chrono.ChVector3d(0, 0, 0.20))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, top)
            next_log += 0.5


def print_state(system, top):
    pos = top.GetPos()
    omega = top.GetAngVelParent()
    print(
        f"t={system.GetChTime():6.3f}  "
        f"pos=({pos.x:+.5f}, {pos.y:+.5f}, {pos.z:+.5f})  "
        f"omega=({omega.x:+.3f}, {omega.y:+.3f}, {omega.z:+.3f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: tippeTop.py -> PyChrono compound contact tippe top")
    if args.no_vis:
        system, top = simulate(args.duration, args.step)
        print_state(system, top)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/generalContactCylinderTest.py:
# a short cylinder rolling on a triangular ground contact surface. Chrono uses
# native SMC cylinder/box contact rather than EXUDYN's marker-sphere cylinder.

RADIUS = 0.25
WIDTH = 0.10
OMEGA_Y = 12.0
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_material(friction=0.2):
    material = chrono.ChContactMaterialSMC()
    material.SetFriction(friction)
    material.SetKn(1e4)
    material.SetGn(5.0)
    material.SetKt(1e4)
    material.SetGt(5.0)
    material.SetRestitution(0.02)
    return material


def add_spokes(cylinder):
    spoke_x = chrono.ChVisualShapeBox(1.85 * RADIUS, 0.012, 0.012)
    spoke_x.SetColor(color(0.95, 0.95, 0.95))
    cylinder.AddVisualShape(spoke_x, chrono.ChFramed(chrono.ChVector3d(0, -0.56 * WIDTH, 0)))
    spoke_z = chrono.ChVisualShapeBox(0.012, 0.012, 1.85 * RADIUS)
    spoke_z.SetColor(color(0.95, 0.95, 0.95))
    cylinder.AddVisualShape(spoke_z, chrono.ChFramed(chrono.ChVector3d(0, -0.56 * WIDTH, 0)))


def build_system():
    system = chrono.ChSystemSMC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    material = make_material(0.2)

    ground = chrono.ChBodyEasyBox(4.0, 4.0, 0.06, 1000, True, True, material)
    ground.SetFixed(True)
    ground.SetPos(chrono.ChVector3d(2.0, 0, -0.03))
    ground.GetVisualShape(0).SetColor(color(0.48, 0.50, 0.52))
    ground.GetVisualShape(0).SetOpacity(0.45)
    system.AddBody(ground)

    cylinder = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, RADIUS, WIDTH, 1000, True, True, material)
    cylinder.SetPos(chrono.ChVector3d(0, 0, RADIUS))
    cylinder.SetRot(chrono.QuatFromAngleX(0.1))
    cylinder.SetPosDt(chrono.ChVector3d(OMEGA_Y * RADIUS, 0, 0))
    cylinder.SetAngVelParent(chrono.ChVector3d(0, 0.5 * OMEGA_Y, 0))
    cylinder.GetVisualShape(0).SetColor(color(0.18, 0.38, 0.82))
    add_spokes(cylinder)
    system.AddBody(cylinder)

    return system, cylinder


def simulate(duration, step):
    system, cylinder = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, cylinder


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, cylinder = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: generalContactCylinderTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(2.2, -1.2, 0.9), chrono.ChVector3d(1.0, 0, 0.20))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, cylinder)
            next_log += 0.25


def print_state(system, cylinder):
    pos = cylinder.GetPos()
    vel = cylinder.GetPosDt()
    omega = cylinder.GetAngVelParent()
    rot = cylinder.GetRot().GetCardanAnglesXYZ()
    norm = math.sqrt(
        pos.x * pos.x
        + pos.y * pos.y
        + pos.z * pos.z
        + vel.x * vel.x
        + vel.y * vel.y
        + vel.z * vel.z
        + omega.x * omega.x
        + omega.y * omega.y
        + omega.z * omega.z
        + rot.x * rot.x
        + rot.y * rot.y
        + rot.z * rot.z
    )
    print(
        f"t={system.GetChTime():6.3f}  "
        f"pos=({pos.x:+.5f}, {pos.y:+.5f}, {pos.z:+.5f})  "
        f"omega=({omega.x:+.4f}, {omega.y:+.4f}, {omega.z:+.4f})  qnorm={norm:.8f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: generalContactCylinderTest.py -> PyChrono SMC rolling cylinder")
    if args.no_vis:
        system, cylinder = simulate(args.duration, args.step)
        print_state(system, cylinder)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

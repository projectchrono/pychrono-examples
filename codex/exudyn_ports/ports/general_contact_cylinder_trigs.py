import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/generalContactCylinderTrigsTest.py:
# a cylinder, represented by a triangulated surface in EXUDYN, rolling on a bed
# of spherical contact markers. Chrono uses a native cylinder against fixed
# contact spheres.

RADIUS = 0.25
WIDTH = 0.25
MARKER_RADIUS = 0.02
PLANE_LENGTH = 0.3
X_FACTOR = 5
N_PLANE = 6
OMEGA_Y = 10.0
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_material():
    material = chrono.ChContactMaterialSMC()
    material.SetFriction(0.1)
    material.SetKn(2e4)
    material.SetGn(10.0)
    material.SetKt(2e4)
    material.SetGt(10.0)
    material.SetRestitution(0.02)
    return material


def add_spokes(cylinder):
    spoke_x = chrono.ChVisualShapeBox(1.85 * RADIUS, 0.012, 0.012)
    spoke_x.SetColor(color(0.95, 0.95, 0.95))
    cylinder.AddVisualShape(spoke_x, chrono.ChFramed(chrono.ChVector3d(0, -0.54 * WIDTH, 0)))
    spoke_z = chrono.ChVisualShapeBox(0.012, 0.012, 1.85 * RADIUS)
    spoke_z.SetColor(color(0.95, 0.95, 0.95))
    cylinder.AddVisualShape(spoke_z, chrono.ChFramed(chrono.ChVector3d(0, -0.54 * WIDTH, 0)))


def build_system():
    system = chrono.ChSystemSMC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    material = make_material()

    bed_plate = chrono.ChBodyEasyBox(PLANE_LENGTH * X_FACTOR, PLANE_LENGTH, 0.01, 1000, True, False)
    bed_plate.SetFixed(True)
    bed_plate.SetPos(chrono.ChVector3d(0.5 * PLANE_LENGTH * X_FACTOR, 0, -0.04))
    bed_plate.GetVisualShape(0).SetColor(color(0.48, 0.50, 0.52))
    bed_plate.GetVisualShape(0).SetOpacity(0.35)
    system.AddBody(bed_plate)

    spheres = []
    for ix in range(N_PLANE * X_FACTOR + 1):
        for iy in range(N_PLANE + 1):
            x = (ix / N_PLANE) * PLANE_LENGTH
            y = (iy / N_PLANE - 0.5) * PLANE_LENGTH
            sphere = chrono.ChBodyEasySphere(MARKER_RADIUS, 1000, True, True, material)
            sphere.SetFixed(True)
            sphere.SetPos(chrono.ChVector3d(x, y, -MARKER_RADIUS))
            sphere.GetVisualShape(0).SetColor(color(0.62, 0.62, 0.64))
            system.AddBody(sphere)
            spheres.append(sphere)

    cylinder = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, RADIUS, WIDTH, 200, True, True, material)
    cylinder.SetPos(chrono.ChVector3d(0, 0, RADIUS))
    cylinder.SetPosDt(chrono.ChVector3d(OMEGA_Y * RADIUS, 0, 0))
    cylinder.SetAngVelParent(chrono.ChVector3d(0, OMEGA_Y, 0))
    cylinder.GetVisualShape(0).SetColor(color(0.18, 0.38, 0.82))
    add_spokes(cylinder)
    system.AddBody(cylinder)

    return system, cylinder, spheres


def simulate(duration, step):
    system, cylinder, spheres = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, cylinder, spheres


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, cylinder, spheres = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: generalContactCylinderTrigsTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.5, -0.9, 0.75), chrono.ChVector3d(0.75, 0, 0.12))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, cylinder, spheres)
            next_log += 0.25


def print_state(system, cylinder, spheres):
    pos = cylinder.GetPos()
    vel = cylinder.GetPosDt()
    omega = cylinder.GetAngVelParent()
    norm = math.sqrt(pos.Length2() + vel.Length2() + omega.Length2())
    print(
        f"t={system.GetChTime():6.3f}  "
        f"pos=({pos.x:+.5f}, {pos.y:+.5f}, {pos.z:+.5f})  "
        f"omega_y={omega.y:+.5f}  contact_spheres={len(spheres)}  qnorm={norm:.8f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: generalContactCylinderTrigsTest.py -> PyChrono cylinder on contact-sphere bed")
    if args.no_vis:
        system, cylinder, spheres = simulate(args.duration, args.step)
        print_state(system, cylinder, spheres)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

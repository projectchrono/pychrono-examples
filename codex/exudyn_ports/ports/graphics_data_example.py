import argparse

import pychrono.core as chrono


# Reproduces the intent of EXUDYN Examples/graphicsDataExample.py:
# a gallery of graphics primitives plus one rigid spinning sphere. The EXUDYN
# source focuses on visualization data, so this port uses fixed Chrono bodies
# with explicit visual shapes for every displayed primitive.

STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_body(system, name, position, fixed=True):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(fixed)
    body.EnableCollision(False)
    body.SetPos(position)
    system.AddBody(body)
    return body


def add_cylinder_between(body, p1, p2, radius, tint):
    segment = chrono.ChLineSegment(p1, p2)
    cylinder = chrono.ChVisualShapeCylinder(radius, segment.GetLength())
    cylinder.SetColor(tint)
    body.AddVisualShape(cylinder, segment.GetFrame())
    return cylinder


def add_box(body, center, size, tint):
    shape = chrono.ChVisualShapeBox(size.x, size.y, size.z)
    shape.SetColor(tint)
    body.AddVisualShape(shape, chrono.ChFramed(center))
    return shape


def add_sphere(body, center, radius, tint):
    shape = chrono.ChVisualShapeSphere(radius)
    shape.SetColor(tint)
    body.AddVisualShape(shape, chrono.ChFramed(center))
    return shape


def add_basis(body, origin, length):
    add_cylinder_between(body, origin, origin + chrono.ChVector3d(length, 0, 0), 0.018, color(0.90, 0.20, 0.16))
    add_cylinder_between(body, origin, origin + chrono.ChVector3d(0, length, 0), 0.018, color(0.14, 0.70, 0.20))
    add_cylinder_between(body, origin, origin + chrono.ChVector3d(0, 0, length), 0.018, color(0.16, 0.28, 0.90))


def add_checkerboard(body):
    tile = 0.5
    for ix in range(-5, 5):
        for iy in range(-5, 5):
            shade = 0.68 if (ix + iy) % 2 else 0.82
            add_box(
                body,
                chrono.ChVector3d((ix + 0.5) * tile, (iy + 0.5) * tile, -2.02),
                chrono.ChVector3d(tile, tile, 0.025),
                color(shade, shade, shade),
            )


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    gallery = make_body(system, "graphics primitive gallery", chrono.ChVector3d(0, 0, 0), True)

    add_checkerboard(gallery)

    add_cylinder_between(
        gallery,
        chrono.ChVector3d(-2, -2, -1),
        chrono.ChVector3d(-2, -2, 1),
        0.04,
        color(0.95, 0.80, 0.10),
    )
    add_basis(gallery, chrono.ChVector3d(-2, -1, -1), 0.5)
    add_cylinder_between(
        gallery,
        chrono.ChVector3d(-2, 0, -1),
        chrono.ChVector3d(-2, 0, 1),
        0.30,
        color(0.95, 0.95, 0.95),
    )
    add_sphere(gallery, chrono.ChVector3d(-2, 1, 0), 0.50, color(0.95, 0.95, 0.95))
    add_box(
        gallery,
        chrono.ChVector3d(-2, -3, -1),
        chrono.ChVector3d(0.40, 0.50, 0.60),
        color(0.12, 0.42, 0.92),
    )

    for z, radius in [(-1.0, 0.14), (-0.75, 0.22), (-0.50, 0.34), (-0.25, 0.22)]:
        add_cylinder_between(
            gallery,
            chrono.ChVector3d(0, -2, z),
            chrono.ChVector3d(0, -2, z + 0.22),
            radius,
            color(0.80, 0.10, 0.10),
        )
    add_box(
        gallery,
        chrono.ChVector3d(0.25, 2.0, -0.25),
        chrono.ChVector3d(0.82, 0.82, 1.50),
        color(0.28, 0.52, 0.72),
    )
    add_box(
        gallery,
        chrono.ChVector3d(0.25, 2.0, 0.55),
        chrono.ChVector3d(0.26, 0.26, 0.35),
        color(0.70, 0.82, 0.90),
    )

    sphere = chrono.ChBody()
    sphere.SetName("spinning graphics sphere")
    sphere.SetMass(1.0)
    sphere.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
    sphere.SetPos(chrono.ChVector3d(1, 1, 0))
    sphere.SetAngVelLocal(chrono.ChVector3d(1, 0, 0))
    sphere.EnableCollision(False)
    add_sphere(sphere, chrono.ChVector3d(0, 0, 0), 0.50, color(0.95, 0.50, 0.12))
    add_box(sphere, chrono.ChVector3d(0, 0, 0.50), chrono.ChVector3d(0.95, 0.025, 0.025), color(0.08, 0.08, 0.08))
    system.AddBody(sphere)

    system._graphics_data = {"sphere": sphere}
    return system, sphere, gallery


def update_visuals(system):
    pass


def simulate(duration, step):
    system, sphere, gallery = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, sphere, gallery


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, sphere, gallery = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: graphicsDataExample.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(4.4, -5.4, 3.2), chrono.ChVector3d(-0.5, -0.5, -0.6))
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
            next_log += 0.5


def print_state(system, sphere):
    omega = sphere.GetAngVelLocal()
    pos = sphere.GetPos()
    print(
        f"t={system.GetChTime():6.3f}  "
        f"sphere=({pos.x:+.3f}, {pos.y:+.3f}, {pos.z:+.3f})  "
        f"omega=({omega.x:+.3f}, {omega.y:+.3f}, {omega.z:+.3f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: graphicsDataExample.py -> PyChrono graphics primitives")
    if args.no_vis:
        system, sphere, gallery = simulate(args.duration, args.step)
        print_state(system, sphere)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

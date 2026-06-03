import argparse
import random

import pychrono.core as chrono


# Reproduces the intent of EXUDYN Examples/particleClusters.py:
# rigid bodies represented by clusters of contact spheres falling into a
# box-like container, plus one larger block-like body with corner spheres.

CLUSTERS = 96
RADIUS = 0.08
MASS = 0.08
STEP = 5e-4


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_material(friction=0.55, restitution=0.04):
    mat = chrono.ChContactMaterialNSC()
    mat.SetFriction(friction)
    mat.SetRestitution(restitution)
    return mat


def add_wall(sys, mat, size, pos, tint, opacity=0.32):
    wall = chrono.ChBodyEasyBox(size.x, size.y, size.z, 1000, True, True, mat)
    wall.SetFixed(True)
    wall.SetPos(pos)
    wall.GetVisualShape(0).SetColor(tint)
    wall.GetVisualShape(0).SetOpacity(opacity)
    sys.AddBody(wall)
    return wall


def sphere_offsets():
    drx = RADIUS * 0.75
    drz = RADIUS * 0.50
    return [
        chrono.ChVector3d(0, 0, -drz),
        chrono.ChVector3d(0, 0, drz),
        chrono.ChVector3d(-drx, 0, -drz),
        chrono.ChVector3d(drx, 0, -drz),
        chrono.ChVector3d(drx, 0, drz),
        chrono.ChVector3d(-drx, 0, drz),
    ]


def add_cluster(sys, mat, pos, tint, large=False):
    body = chrono.ChBody()
    body.SetMass(MASS * (10 if large else 1))
    body.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, 0.02) if large else chrono.ChVector3d(0.002, 0.002, 0.002))
    body.SetPos(pos)
    body.SetPosDt(chrono.ChVector3d(0, -1.0, 0))
    body.EnableCollision(True)

    offsets = sphere_offsets()
    if large:
        half = 4.0 * RADIUS
        box = chrono.ChCollisionShapeBox(mat, 2 * half, 2 * half, 2 * half)
        body.AddCollisionShape(box)
        vbox = chrono.ChVisualShapeBox(2 * half, 2 * half, 2 * half)
        vbox.SetColor(color(0.25, 0.48, 0.72))
        vbox.SetOpacity(0.55)
        body.AddVisualShape(vbox)
        offsets = [
            chrono.ChVector3d(sx * half, sy * half, sz * half)
            for sx in (-1, 1)
            for sy in (-1, 1)
            for sz in (-1, 1)
        ]

    for offset in offsets:
        body.AddCollisionShape(chrono.ChCollisionShapeSphere(mat, RADIUS), chrono.ChFramed(offset))
        shape = chrono.ChVisualShapeSphere(RADIUS)
        shape.SetColor(tint)
        body.AddVisualShape(shape, chrono.ChFramed(offset))

    sys.AddBody(body)
    return body


def build_system(count=CLUSTERS):
    random.seed(5)
    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    cluster_mat = make_material(0.55, 0.04)
    wall_mat = make_material(0.65, 0.02)

    add_wall(sys, wall_mat, chrono.ChVector3d(5.2, 0.12, 2.4), chrono.ChVector3d(2.4, -0.18, 0), color(0.50, 0.52, 0.55), 0.65)
    add_wall(sys, wall_mat, chrono.ChVector3d(0.12, 2.8, 2.4), chrono.ChVector3d(-0.25, 1.1, 0), color(0.55, 0.55, 0.38), 0.35)
    add_wall(sys, wall_mat, chrono.ChVector3d(0.12, 2.8, 2.4), chrono.ChVector3d(5.0, 1.1, 0), color(0.55, 0.55, 0.38), 0.35)
    add_wall(sys, wall_mat, chrono.ChVector3d(5.2, 2.8, 0.12), chrono.ChVector3d(2.4, 1.1, -1.25), color(0.55, 0.55, 0.38), 0.28)
    add_wall(sys, wall_mat, chrono.ChVector3d(5.2, 2.8, 0.12), chrono.ChVector3d(2.4, 1.1, 1.25), color(0.55, 0.55, 0.38), 0.28)

    clusters = []
    rows = 8
    spacing_x = 3.8 * RADIUS
    spacing_y = 2.4 * RADIUS
    spacing_z = 3.0 * RADIUS
    palette = [
        color(0.12, 0.42, 0.85),
        color(0.10, 0.62, 0.25),
        color(0.92, 0.48, 0.06),
        color(0.75, 0.18, 0.72),
    ]
    for i in range(count):
        layer = i // rows
        col = i % rows
        x = 0.25 + col * spacing_x + (0.5 * spacing_x if layer % 2 else 0)
        y = 0.45 + (layer % 10) * spacing_y
        z = -0.9 + (layer // 10) * spacing_z + random.uniform(-0.08, 0.08) * RADIUS
        clusters.append(add_cluster(sys, cluster_mat, chrono.ChVector3d(x, y, z), palette[layer % len(palette)]))

    clusters.append(add_cluster(sys, cluster_mat, chrono.ChVector3d(3.9, 1.45, 0.65), color(0.92, 0.80, 0.08), True))
    return sys, clusters


def simulate(duration, step, count=CLUSTERS):
    sys, clusters = build_system(count)
    while sys.GetChTime() < duration:
        sys.DoStepDynamics(step)
    return sys, clusters


def run_visual(duration, step, count):
    import pychrono.irrlicht as chronoirr

    sys, clusters = build_system(count)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1120, 760)
    vis.SetWindowTitle("EXUDYN port: particleClusters.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(4.2, 2.2, 4.8), chrono.ChVector3d(2.4, 0.5, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and sys.GetChTime() < duration:
        t = sys.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(step)
        if t >= next_log:
            print_state(sys, clusters)
            next_log += 0.5


def print_state(sys, clusters):
    p = clusters[min(len(clusters) - 1, 10)].GetPos()
    print(f"t={sys.GetChTime():6.3f}  clusters={len(clusters)}  sample=({p.x:+.4f},{p.y:+.4f},{p.z:+.4f})")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--count", type=int, default=CLUSTERS)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: particleClusters.py -> PyChrono rigid sphere clusters")
    if args.no_vis:
        sys, clusters = simulate(args.duration, args.step, args.count)
        print_state(sys, clusters)
    else:
        run_visual(args.duration, args.step, args.count)


if __name__ == "__main__":
    main()

import argparse

import pychrono.core as chrono


# Reproduces the intent of EXUDYN Examples/multiMbsTest.py:
# several independent MBS instances, each containing a single rigid link hinged
# to ground.  PyChrono visualizes the same four independent hinged links in one
# system while preserving the two EXUDYN groups by their offsets and colors.

LENGTH = 1.0
WIDTH = 0.10
DENSITY = 5000.0
MASS = DENSITY * LENGTH * WIDTH * WIDTH
STEP = 1e-3

PENDULUMS = [
    ("SC1 red link", chrono.ChVector3d(0.0, 0.0, 0.0), (0.85, 0.10, 0.08)),
    ("SC1 blue link", chrono.ChVector3d(1.2, 0.0, 0.0), (0.10, 0.30, 0.85)),
    ("SC2 red link", chrono.ChVector3d(0.0, -1.2, 0.0), (0.85, 0.10, 0.08)),
    ("SC2 green link", chrono.ChVector3d(0.6, -2.4, 0.0), (0.10, 0.62, 0.24)),
]


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def add_visual_axis(body, half_length):
    axis = chrono.ChVisualShapeCylinder(0.012, 2.0 * half_length)
    axis.SetColor(color(0.96, 0.82, 0.12))
    body.AddVisualShape(axis, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.Q_ROTATE_Z_TO_X))
    end = chrono.ChVisualShapeSphere(0.045)
    end.SetColor(color(0.08, 0.08, 0.08))
    body.AddVisualShape(end, chrono.ChFramed(chrono.ChVector3d(-half_length, 0, 0)))


def add_pendulum(system, ground, name, pivot, tint_tuple):
    tint = color(*tint_tuple)

    body = chrono.ChBodyEasyBox(LENGTH, WIDTH, WIDTH, DENSITY, True, False)
    body.SetName(name)
    body.SetMass(MASS)
    body.SetInertiaXX(chrono.ChVector3d(MASS * (WIDTH * WIDTH + WIDTH * WIDTH) / 12.0, MASS * LENGTH * LENGTH / 12.0, MASS * LENGTH * LENGTH / 12.0))
    body.SetPos(pivot + chrono.ChVector3d(0.5 * LENGTH, 0, 0))
    body.GetVisualShape(0).SetColor(tint)
    add_visual_axis(body, 0.5 * LENGTH)
    system.AddBody(body)

    joint = chrono.ChLinkLockRevolute()
    joint.SetName(f"{name} revolute joint")
    joint.Initialize(body, ground, chrono.ChFramed(pivot, chrono.QUNIT))
    system.AddLink(joint)

    pivot_body = chrono.ChBodyEasySphere(0.055, 1000, True, False)
    pivot_body.SetName(f"{name} pivot visual")
    pivot_body.SetFixed(True)
    pivot_body.SetPos(pivot)
    pivot_body.GetVisualShape(0).SetColor(color(0.10, 0.10, 0.10))
    system.AddBody(pivot_body)

    row = chrono.ChBodyEasyBox(1.25, 0.018, 0.018, 1000, True, False)
    row.SetName(f"{name} local ground reference")
    row.SetFixed(True)
    row.SetPos(pivot + chrono.ChVector3d(0.55, -0.18, -0.02))
    row.GetVisualShape(0).SetColor(color(0.50, 0.50, 0.50))
    system.AddBody(row)

    return body, joint, pivot_body


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    ground = chrono.ChBody()
    ground.SetName("multi-mbs shared hidden ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)

    bodies = []
    joints = []
    pivots = []
    for name, pivot, tint in PENDULUMS:
        body, joint, pivot_body = add_pendulum(system, ground, name, pivot, tint)
        bodies.append(body)
        joints.append(joint)
        pivots.append(pivot_body)

    system._multi_mbs_items = {
        "bodies": bodies,
        "joints": joints,
        "pivots": pivots,
    }
    return system, bodies, joints


def simulate(duration, step):
    system, bodies, joints = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, bodies, joints


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, bodies, joints = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: multiMbsTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.15, -0.95, 4.4), chrono.ChVector3d(0.85, -0.95, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, bodies)
            next_log += 0.5


def update_visuals(system):
    return None


def print_state(system, bodies):
    angles = [body.GetRot().GetCardanAnglesXYZ().z for body in bodies]
    positions = [body.GetPos() for body in bodies]
    print(
        f"t={system.GetChTime():6.3f}  "
        f"angles=({', '.join(f'{angle:+.4f}' for angle in angles)})  "
        f"first=({positions[0].x:+.4f}, {positions[0].y:+.4f})  "
        f"systems=2 pendulums={len(bodies)}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: multiMbsTest.py -> PyChrono multiple independent hinged links")
    if args.no_vis:
        system, bodies, joints = simulate(args.duration, args.step)
        print_state(system, bodies)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

import argparse

import pychrono.core as chrono


# Reproduces the intent of EXUDYN Examples/gyroStability.py:
# three identical free rigid bodies spin about the smallest, middle, and largest
# principal inertia axes with a small disturbance. The middle-axis case shows
# the unstable tennis-racket effect.

LENGTH = 0.1
WIDTH = 0.03
ANGULAR_SPEED = 5.0
EPS = 2e-4
STEP = 5e-4
INERTIA = chrono.ChVector3d(2.0e-4, 7.0e-4, 1.1e-3)


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def add_cylinder_between(body, p1, p2, radius, tint):
    segment = chrono.ChLineSegment(p1, p2)
    cylinder = chrono.ChVisualShapeCylinder(radius, segment.GetLength())
    cylinder.SetColor(tint)
    body.AddVisualShape(cylinder, segment.GetFrame())


def make_gyro(index, pos, omega, tint):
    body = chrono.ChBody()
    body.SetName(f"gyro {index}")
    body.SetMass(1.0)
    body.SetInertiaXX(INERTIA)
    body.SetPos(pos)
    body.SetAngVelLocal(omega)
    body.SetUseGyroTorque(True)
    body.EnableCollision(False)

    add_cylinder_between(
        body,
        chrono.ChVector3d(-LENGTH, 0, 0),
        chrono.ChVector3d(LENGTH, 0, 0),
        0.5 * WIDTH,
        tint,
    )
    add_cylinder_between(
        body,
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, LENGTH, 0),
        0.5 * WIDTH,
        tint,
    )
    marker = chrono.ChVisualShapeSphere(0.018)
    marker.SetColor(color(0.06, 0.06, 0.06))
    body.AddVisualShape(marker, chrono.ChFramed(chrono.ChVector3d(LENGTH, 0, 0)))
    return body


def build_system():
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    positions = [
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(3 * LENGTH, 0, 0),
        chrono.ChVector3d(6 * LENGTH, 0, 0),
    ]
    omegas = [
        chrono.ChVector3d(ANGULAR_SPEED, EPS * ANGULAR_SPEED, EPS * ANGULAR_SPEED),
        chrono.ChVector3d(EPS * ANGULAR_SPEED, ANGULAR_SPEED, EPS * ANGULAR_SPEED),
        chrono.ChVector3d(EPS * ANGULAR_SPEED, EPS * ANGULAR_SPEED, ANGULAR_SPEED),
    ]
    colors = [
        color(0.90, 0.12, 0.10),
        color(0.25, 0.75, 0.12),
        color(0.12, 0.32, 0.85),
    ]

    bodies = []
    for i, (pos, omega, tint) in enumerate(zip(positions, omegas, colors)):
        body = make_gyro(i, pos, omega, tint)
        sys.AddBody(body)
        bodies.append(body)

    return sys, bodies


def simulate(duration, step):
    sys, bodies = build_system()
    while sys.GetChTime() < duration:
        sys.DoStepDynamics(step)
    return sys, bodies


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    sys, bodies = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1120, 760)
    vis.SetWindowTitle("EXUDYN port: gyroStability.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.32, 0.42, 1.15), chrono.ChVector3d(0.30, 0.02, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and sys.GetChTime() < duration:
        t = sys.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(step)
        if t >= next_log:
            print_state(sys, bodies)
            next_log += 1.0


def print_state(sys, bodies):
    parts = []
    for i, body in enumerate(bodies):
        omega = body.GetAngVelLocal()
        parts.append(f"omega{i}=({omega.x:+.3f},{omega.y:+.3f},{omega.z:+.3f})")
    print(f"t={sys.GetChTime():6.3f}  " + "  ".join(parts))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: gyroStability.py -> PyChrono free rigid gyro stability")
    if args.no_vis:
        sys, bodies = simulate(args.duration, args.step)
        print_state(sys, bodies)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

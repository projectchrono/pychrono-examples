import argparse

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/rotatingTableTest.py:
# a torque-driven wheel rolling on a table that can rotate around a vertical
# axis. The EXUDYN source compares ideal and penalty rolling-disc connectors;
# this PyChrono version uses high-friction contact for the wheel/table pair.

MASS = 30.0
RADIUS = 0.5
FRAME_LENGTH = 3.0
WIDTH = 0.10
TORQUE_Z = 20.0
STEP = 5e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_material(friction=0.8, restitution=0.02):
    mat = chrono.ChContactMaterialNSC()
    mat.SetFriction(friction)
    mat.SetRollingFriction(0.001)
    mat.SetSpinningFriction(0.001)
    mat.SetRestitution(restitution)
    return mat


def build_system():
    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    contact_mat = make_material(0.9, 0.02)

    ground = chrono.ChBody()
    ground.SetFixed(True)
    sys.AddBody(ground)

    table = chrono.ChBodyEasyBox(6.0, 0.08, 6.0, 800, True, True, contact_mat)
    table.SetMass(100.0)
    table.SetInertiaXX(chrono.ChVector3d(1000, 1000, 1000))
    table.SetPos(chrono.ChVector3d(0, -RADIUS - 0.04, 0))
    table.GetVisualShape(0).SetColor(color(0.42, 0.44, 0.46))
    sys.AddBody(table)

    table_joint = chrono.ChLinkLockRevolute()
    table_joint.Initialize(table, ground, chrono.ChFramed(chrono.ChVector3d(0, -RADIUS, 0), chrono.Q_ROTATE_Z_TO_Y))
    sys.AddLink(table_joint)

    wheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, RADIUS, WIDTH, 1000, True, True, contact_mat)
    wheel.SetMass(MASS)
    wheel.SetPos(chrono.ChVector3d(0, 0, FRAME_LENGTH))
    wheel.GetVisualShape(0).SetColor(color(0.10, 0.35, 0.90))
    spoke = chrono.ChVisualShapeBox(1.15 * RADIUS, 1.15 * RADIUS, 0.03)
    spoke.SetColor(color(0.95, 0.38, 0.18))
    wheel.AddVisualShape(spoke)
    shaft = chrono.ChVisualShapeCylinder(0.5 * WIDTH, FRAME_LENGTH)
    shaft.SetColor(color(0.95, 0.55, 0.10))
    wheel.AddVisualShape(shaft, chrono.ChFramed(chrono.ChVector3d(0, 0, -0.5 * FRAME_LENGTH), chrono.QUNIT))
    sys.AddBody(wheel)

    support = chrono.ChBody()
    support.SetFixed(True)
    support.EnableCollision(False)
    pivot = chrono.ChVisualShapeSphere(0.08)
    pivot.SetColor(color(0.08, 0.08, 0.08))
    support.AddVisualShape(pivot, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    sys.AddBody(support)

    tether = chrono.ChLinkDistance()
    tether.Initialize(
        wheel,
        support,
        True,
        chrono.ChVector3d(0, 0, -FRAME_LENGTH),
        chrono.ChVector3d(0, RADIUS, 0),
        False,
        RADIUS,
    )
    sys.AddLink(tether)

    load_container = chrono.ChLoadContainer()
    sys.Add(load_container)
    torque = chrono.ChLoadBodyTorque(wheel, chrono.ChVector3d(0, 0, TORQUE_Z), True)
    load_container.Add(torque)

    return sys, table, wheel, tether, load_container


def simulate(duration, step):
    sys, table, wheel, tether, load_container = build_system()
    while sys.GetChTime() < duration:
        sys.DoStepDynamics(step)
    return sys, table, wheel, tether, load_container


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    sys, table, wheel, tether, load_container = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1120, 760)
    vis.SetWindowTitle("EXUDYN port: rotatingTableTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(3.5, 2.0, 5.8), chrono.ChVector3d(0, -0.25, 1.3))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and sys.GetChTime() < duration:
        t = sys.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(step)
        if t >= next_log:
            print_state(sys, table, wheel)
            next_log += 0.5


def print_state(sys, table, wheel):
    p = wheel.GetPos()
    omega = wheel.GetAngVelLocal()
    table_omega = table.GetAngVelParent()
    print(
        f"t={sys.GetChTime():6.3f}  "
        f"wheel=({p.x:+.4f},{p.y:+.4f},{p.z:+.4f})  "
        f"wheel_wz={omega.z:+.4f}  table_wy={table_omega.y:+.4f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: rotatingTableTest.py -> PyChrono rolling wheel on rotating table")
    if args.no_vis:
        sys, table, wheel, _, _ = simulate(args.duration, args.step)
        print_state(sys, table, wheel)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

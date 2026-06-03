import argparse

import pychrono.core as chrono


# Reproduces the intent of EXUDYN Examples/switchingConstraintsPendulum.py:
# a rigid pendulum starts pinned with an attached point mass; the mass coupling
# is released after 0.1 s and the support joint after 0.3 s.

A = 0.5
B = 0.05
LENGTH = 2.0 * A
WIDTH = 2.0 * B
MASS_RIGID = 12.0
MASS_POINT = 2.0
INERTIA_RIGID_ZZ = MASS_RIGID / 12.0 * LENGTH * LENGTH
OMEGA0 = 4.0
MASS_RELEASE_TIME = 0.1
SUPPORT_RELEASE_TIME = 0.3
STEP = 5e-4

PENDULUM_CENTER = chrono.ChVector3d(-0.5, 0, 0)
PIVOT = chrono.ChVector3d(-1.0, 0, 0)


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def add_pendulum_markers(body):
    for x, radius, tint in (
        (-A, 0.045, color(0.06, 0.06, 0.06)),
        (0, 0.035, color(0.95, 0.95, 0.95)),
        (A, 0.045, color(0.06, 0.06, 0.06)),
    ):
        marker = chrono.ChVisualShapeSphere(radius)
        marker.SetColor(tint)
        body.AddVisualShape(marker, chrono.ChFramed(chrono.ChVector3d(x, 0, 0)))


def make_pendulum():
    pendulum = chrono.ChBodyEasyBox(LENGTH, WIDTH, 0.07, 1000, True, False)
    pendulum.SetName("switching pendulum")
    pendulum.SetMass(MASS_RIGID)
    pendulum.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, INERTIA_RIGID_ZZ))
    pendulum.SetPos(PENDULUM_CENTER)
    pendulum.SetPosDt(chrono.ChVector3d(0, OMEGA0 * A, 0))
    pendulum.SetAngVelLocal(chrono.ChVector3d(0, 0, OMEGA0))
    pendulum.GetVisualShape(0).SetColor(color(0.10, 0.32, 0.86))
    add_pendulum_markers(pendulum)
    return pendulum


def build_system():
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    support = chrono.ChBodyEasyBox(0.18, 0.18, 0.08, 1000, True, False)
    support.SetName("switching support")
    support.SetFixed(True)
    support.SetPos(PIVOT)
    support.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.08))
    sys.AddBody(support)

    reference = chrono.ChBodyEasyBox(3.0, 0.018, 0.018, 1000, True, False)
    reference.SetName("background reference line")
    reference.SetFixed(True)
    reference.SetPos(chrono.ChVector3d(-1.0, -0.36, 0))
    reference.GetVisualShape(0).SetColor(color(0.45, 0.45, 0.45))
    sys.AddBody(reference)

    pendulum = make_pendulum()
    sys.AddBody(pendulum)

    point_mass = chrono.ChBodyEasySphere(0.075, 1000, True, False)
    point_mass.SetName("released point mass")
    point_mass.SetMass(MASS_POINT)
    point_mass.SetInertiaXX(chrono.ChVector3d(0.004, 0.004, 0.004))
    point_mass.SetPos(PENDULUM_CENTER)
    point_mass.SetPosDt(chrono.ChVector3d(0, OMEGA0 * A, 0))
    point_mass.GetVisualShape(0).SetColor(color(0.94, 0.64, 0.08))
    sys.AddBody(point_mass)

    support_joint = chrono.ChLinkLockRevolute()
    support_joint.Initialize(pendulum, support, chrono.ChFramed(PIVOT))
    sys.AddLink(support_joint)

    mass_coupling = chrono.ChLinkLockSpherical()
    mass_coupling.Initialize(point_mass, pendulum, chrono.ChFramed(PENDULUM_CENTER))
    sys.AddLink(mass_coupling)

    sys._switching_constraint_items = {
        "support_joint": support_joint,
        "mass_coupling": mass_coupling,
        "support_released": False,
        "mass_released": False,
    }
    return sys, pendulum, point_mass, support_joint, mass_coupling


def apply_switches(sys):
    items = getattr(sys, "_switching_constraint_items", None)
    if not items:
        return
    t = sys.GetChTime()
    if t > MASS_RELEASE_TIME and not items["mass_released"]:
        items["mass_coupling"].SetDisabled(True)
        items["mass_released"] = True
    if t > SUPPORT_RELEASE_TIME and not items["support_released"]:
        items["support_joint"].SetDisabled(True)
        items["support_released"] = True


def update_visuals(sys):
    apply_switches(sys)


def simulate(duration, step):
    sys, pendulum, point_mass, support_joint, mass_coupling = build_system()
    while sys.GetChTime() < duration:
        apply_switches(sys)
        sys.DoStepDynamics(step)
    return sys, pendulum, point_mass, support_joint, mass_coupling


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    sys, pendulum, point_mass, support_joint, mass_coupling = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: switchingConstraintsPendulum.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(-0.7, 0.45, 2.4), chrono.ChVector3d(-0.7, -0.25, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and sys.GetChTime() < duration:
        t = sys.GetChTime()
        update_visuals(sys)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(step)
        if t >= next_log:
            print_state(sys, pendulum, point_mass, support_joint, mass_coupling)
            next_log += 0.25


def print_state(sys, pendulum, point_mass, support_joint, mass_coupling):
    angle_z = pendulum.GetRot().GetCardanAnglesXYZ().z
    print(
        f"t={sys.GetChTime():6.3f}  "
        f"pendulum=({pendulum.GetPos().x:+.4f}, {pendulum.GetPos().y:+.4f})  "
        f"mass=({point_mass.GetPos().x:+.4f}, {point_mass.GetPos().y:+.4f})  "
        f"angle_z={angle_z:+.4f}  "
        f"mass_coupled={not mass_coupling.IsDisabled()}  support_active={not support_joint.IsDisabled()}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: switchingConstraintsPendulum.py -> PyChrono switching constraints")
    if args.no_vis:
        sys, pendulum, point_mass, support_joint, mass_coupling = simulate(args.duration, args.step)
        print_state(sys, pendulum, point_mass, support_joint, mass_coupling)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

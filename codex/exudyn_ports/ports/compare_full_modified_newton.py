import argparse

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/compareFullModifiedNewton.py:
# a rigid rectangular pendulum is solved with two Newton/Jacobian strategies.
# Chrono exposes different timesteppers rather than EXUDYN's exact full vs.
# modified Newton switch, so this port compares two implicit Euler variants and
# renders the pendulum body with a visible revolute support.

HALF_LENGTH = 0.5
WIDTH = 0.05
MASS = 12.0
GRAVITY = 9.81
STEP = 2e-2


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def add_pendulum(system, ground, load_container, y_offset, tint, name):
    body = chrono.ChBodyEasyBox(2.0 * HALF_LENGTH, 2.0 * WIDTH, 0.08, 1000, True, False)
    body.SetName(name)
    body.SetMass(MASS)
    inertia_zz = MASS / 12.0 * (2.0 * HALF_LENGTH) ** 2
    body.SetInertiaXX(chrono.ChVector3d(0.02, inertia_zz, inertia_zz))
    body.SetPos(chrono.ChVector3d(HALF_LENGTH, y_offset, 0))
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)

    joint = chrono.ChLinkLockRevolute()
    joint.Initialize(body, ground, chrono.ChFramed(chrono.ChVector3d(0, y_offset, 0)))
    system.AddLink(joint)

    pivot = chrono.ChBodyEasySphere(0.055, 1000, True, False)
    pivot.SetName(f"{name} pivot")
    pivot.SetFixed(True)
    pivot.SetPos(chrono.ChVector3d(0, y_offset, 0))
    pivot.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.08))
    system.AddBody(pivot)

    tip = chrono.ChVisualShapeSphere(0.05)
    tip.SetColor(color(0.92, 0.18, 0.10))
    body.AddVisualShape(tip, chrono.ChFramed(chrono.ChVector3d(HALF_LENGTH, 0, 0)))

    load = chrono.ChLoadBodyForce(
        body,
        chrono.ChVector3d(0, -MASS * GRAVITY, 0),
        False,
        chrono.ChVector3d(HALF_LENGTH, 0, 0),
        True,
    )
    load_container.Add(load)
    return body, joint


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
    ground = chrono.ChBody()
    ground.SetName("Newton-comparison ground")
    ground.SetFixed(True)
    system.AddBody(ground)
    load_container = chrono.ChLoadContainer()
    system.Add(load_container)

    modified, joint_m = add_pendulum(system, ground, load_container, 0.35, color(0.12, 0.42, 0.86), "modified-Newton analogue pendulum")
    full, joint_f = add_pendulum(system, ground, load_container, -0.35, color(0.86, 0.20, 0.12), "full-Newton analogue pendulum")

    frame = chrono.ChBodyEasyBox(1.25, 1.05, 0.012, 1000, True, False)
    frame.SetName("solver comparison reference frame")
    frame.SetFixed(True)
    frame.SetPos(chrono.ChVector3d(0.55, 0.0, -0.06))
    frame.GetVisualShape(0).SetColor(color(0.72, 0.72, 0.72))
    frame.GetVisualShape(0).SetOpacity(0.25)
    system.AddBody(frame)

    system._newton_compare = {"modified": modified, "full": full}
    return system, modified, full, (joint_m, joint_f)


def make_single_system(timestepper_type):
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
    system.SetTimestepperType(timestepper_type)
    ground = chrono.ChBody()
    ground.SetFixed(True)
    system.AddBody(ground)
    load_container = chrono.ChLoadContainer()
    system.Add(load_container)
    body, joint = add_pendulum(system, ground, load_container, 0.0, color(0.25, 0.45, 0.90), "solver comparison pendulum")
    return system, body


def simulate_single(duration, step, timestepper_type):
    system, body = make_single_system(timestepper_type)
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, body


def simulate(duration, step):
    system_m, body_m = simulate_single(duration, step, chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
    system_f, body_f = simulate_single(duration, step, chrono.ChTimestepper.Type_EULER_IMPLICIT)
    return system_m, body_m, system_f, body_f


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, modified, full, joints = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: compareFullModifiedNewton.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.55, 0.25, 2.2), chrono.ChVector3d(0.45, 0.0, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_visual_state(system, modified, full)
            next_log += 0.5


def angle(body):
    return body.GetRot().GetCardanAnglesXYZ().z


def print_visual_state(system, modified, full):
    print(
        f"t={system.GetChTime():6.3f}  "
        f"angles=({angle(modified):+.8f}, {angle(full):+.8f})"
    )


def print_state(system_m, body_m, system_f, body_f):
    diff = angle(body_f) - angle(body_m)
    print(
        f"t={system_f.GetChTime():6.3f}  "
        f"modified_like={angle(body_m):+.8f}  full_like={angle(body_f):+.8f}  diff={diff:+.3e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: compareFullModifiedNewton.py -> PyChrono timestepper comparison")
    if args.no_vis:
        system_m, body_m, system_f, body_f = simulate(args.duration, args.step)
        print_state(system_m, body_m, system_f, body_f)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

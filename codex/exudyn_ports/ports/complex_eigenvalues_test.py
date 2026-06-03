import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN TestModels/complexEigenvaluesTest.py:
# three damped oscillator systems used for complex eigenvalue checks:
# 1) a constrained mass point with a coordinate spring-damper,
# 2) a small rotating beam with a tip spring-damper, and
# 3) a two-link mechanism with a Cartesian spring-damper. Chrono does not expose
# EXUDYN's ODE2 eigenvalue routine here, so this port keeps the same mechanical
# scenes and prints the corresponding analytic/scene metrics. All spring
# connectors are real ChLinkTSDA objects with native coil visuals.

STEP = 5.0e-4
END_TIME = 0.25

MASS = 1.6
MSD_L = 0.5
MSD_K = 4000.0
MSD_C = 8.0
MSD_U0 = -0.08
MSD_V0 = 1.0
MSD_FORCE = 80.0

BEAM_L = 0.1
BEAM_W = 0.01
BEAM_H = 0.001
BEAM_RHO = 5000.0
BEAM_SPRING_L = 0.02
BEAM_SPRING_K = 10.0
BEAM_DREL = 1.0e-4
MECH_SPRING_K = 1.0e3

SCENE_OFFSETS = (
    chrono.ChVector3d(-0.75, 0.0, 0.0),
    chrono.ChVector3d(0.0, 0.0, 0.0),
    chrono.ChVector3d(0.75, 0.0, 0.0),
)


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def add(a, b):
    return chrono.ChVector3d(a.x + b.x, a.y + b.y, a.z + b.z)


def scale(v, s):
    return chrono.ChVector3d(v.x * s, v.y * s, v.z * s)


def rotate_z_y(point, rz, ry):
    cy = math.cos(ry)
    sy = math.sin(ry)
    cz = math.cos(rz)
    sz = math.sin(rz)
    x1 = cy * point.x + sy * point.z
    y1 = point.y
    z1 = -sy * point.x + cy * point.z
    return chrono.ChVector3d(cz * x1 - sz * y1, sz * x1 + cz * y1, z1)


def beam_mass():
    return BEAM_RHO * BEAM_L * BEAM_H * BEAM_W


def beam_inertia_center():
    mass = beam_mass()
    return chrono.ChVector3d(
        mass / 12.0 * (BEAM_H**2 + BEAM_W**2),
        mass / 12.0 * (BEAM_L**2 + BEAM_W**2),
        mass / 12.0 * (BEAM_L**2 + BEAM_H**2),
    )


def beam_theta_zz_about_left():
    mass = beam_mass()
    return mass / 12.0 * (BEAM_L**2 + BEAM_H**2) + mass * (0.5 * BEAM_L) ** 2


def beam_spring_damping():
    return BEAM_DREL * (2.0 * math.sqrt(BEAM_SPRING_K * BEAM_L**2 / beam_theta_zz_about_left()))


def add_anchor(system, name, position, radius=0.018, tint=None):
    anchor = chrono.ChBodyEasySphere(radius, 1000.0, True, False)
    anchor.SetName(name)
    anchor.SetFixed(True)
    anchor.EnableCollision(False)
    anchor.SetPos(position)
    anchor.GetVisualShape(0).SetColor(tint or color(0.05, 0.05, 0.06))
    system.AddBody(anchor)
    return anchor


def add_rail(system, name, position, size, tint):
    rail = chrono.ChBodyEasyBox(size.x, size.y, size.z, 1000.0, True, False)
    rail.SetName(name)
    rail.SetFixed(True)
    rail.EnableCollision(False)
    rail.SetPos(position)
    rail.GetVisualShape(0).SetColor(tint)
    rail.GetVisualShape(0).SetOpacity(0.55)
    system.AddBody(rail)
    return rail


def add_native_spring(system, body_a, body_b, local_a, local_b, rest_length, stiffness, damping, radius, turns, name):
    spring = chrono.ChLinkTSDA()
    spring.SetName(name)
    spring.Initialize(body_a, body_b, True, local_a, local_b)
    spring.SetRestLength(rest_length)
    spring.SetSpringCoefficient(stiffness)
    spring.SetDampingCoefficient(damping)
    system.AddLink(spring)

    spring_shape = chrono.ChVisualShapeSpring(radius, 96, turns)
    spring_shape.SetColor(color(0.86, 0.12, 0.08))
    spring.AddVisualShape(spring_shape)
    fallback = attach_spring_visual(system, spring, radius, 96, turns, color(0.86, 0.12, 0.08))
    fallback.shape.SetThickness(3)
    return spring


def make_ground(system):
    ground = chrono.ChBody()
    ground.SetName("complex eigenvalue shared ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)
    add_rail(system, "complex eigenvalue reference floor", chrono.ChVector3d(0, 0, -0.045), chrono.ChVector3d(1.65, 0.36, 0.018), color(0.72, 0.74, 0.76))
    return ground


def make_mass_spring_scene(system, ground, offset):
    mass = chrono.ChBodyEasySphere(0.045, 1000.0, True, False)
    mass.SetName("complex eigen MSD mass")
    mass.SetMass(MASS)
    mass.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
    mass.SetPos(add(offset, chrono.ChVector3d(MSD_L + MSD_U0, 0, 0)))
    mass.SetPosDt(chrono.ChVector3d(MSD_V0, 0, 0))
    mass.GetVisualShape(0).SetColor(color(0.96, 0.48, 0.10))
    system.AddBody(mass)

    guide = chrono.ChLinkLockPrismatic()
    guide.Initialize(mass, ground, chrono.ChFramed(offset, chrono.Q_ROTATE_Z_TO_X))
    system.AddLink(guide)

    spring = add_native_spring(
        system,
        mass,
        ground,
        chrono.ChVector3d(0, 0, 0),
        offset,
        0.0,
        MSD_K,
        MSD_C,
        0.026,
        16,
        "complex eigen MSD coordinate spring-damper",
    )

    force = chrono.ChForce()
    force.SetF_x(chrono.ChFunctionConst(MSD_FORCE))
    mass.AddForce(force)

    add_anchor(system, "MSD fixed coordinate marker", offset, 0.020)
    add_rail(system, "MSD x-coordinate guide", add(offset, chrono.ChVector3d(0.25, -0.075, 0)), chrono.ChVector3d(0.62, 0.012, 0.012), color(0.35, 0.35, 0.36))
    return {"mass": mass, "spring": spring}


def make_beam_body(name, position, rotation, tint):
    body = chrono.ChBodyEasyBox(BEAM_L, BEAM_H, BEAM_W, 1000.0, True, False)
    body.SetName(name)
    body.SetMass(beam_mass())
    body.SetInertiaXX(beam_inertia_center())
    body.SetPos(position)
    body.SetRot(rotation)
    body.GetVisualShape(0).SetColor(tint)
    body.EnableCollision(False)
    return body


def make_tip_spring_beam_scene(system, ground, offset):
    beam = make_beam_body(
        "complex eigen rotating beam",
        add(offset, chrono.ChVector3d(0.5 * BEAM_L, 0, 0)),
        chrono.QUNIT,
        color(0.96, 0.56, 0.10),
    )
    system.AddBody(beam)

    pivot = chrono.ChLinkLockRevolute()
    pivot.Initialize(beam, ground, chrono.ChFramed(offset, chrono.QUNIT))
    system.AddLink(pivot)

    anchor_pos = add(offset, chrono.ChVector3d(BEAM_L, -BEAM_SPRING_L, 0))
    spring = add_native_spring(
        system,
        beam,
        ground,
        chrono.ChVector3d(0.5 * BEAM_L, 0, 0),
        anchor_pos,
        0.9 * BEAM_SPRING_L,
        BEAM_SPRING_K,
        beam_spring_damping(),
        0.014,
        7,
        "complex eigen rotating beam tip spring-damper",
    )
    add_anchor(system, "rotating beam spring anchor", anchor_pos, 0.010)
    add_anchor(system, "rotating beam revolute marker", offset, 0.012, color(0.02, 0.02, 0.03))
    return {"beam": beam, "spring": spring}


def make_two_link_scene(system, ground, offset):
    p0 = chrono.ChVector3d(0.5 * BEAM_L, 0, 0)
    rz = -0.25 * math.pi
    ry = 0.25 * math.pi
    q1 = chrono.QuatFromAngleZ(rz) * chrono.QuatFromAngleY(ry)

    beam0 = make_beam_body("complex eigen mechanism orange beam", add(offset, p0), chrono.QUNIT, color(0.96, 0.56, 0.10))
    p1 = add(scale(p0, 2.0), rotate_z_y(p0, rz, ry))
    beam1 = make_beam_body("complex eigen mechanism blue beam", add(offset, p1), q1, color(0.08, 0.42, 0.90))
    system.AddBody(beam0)
    system.AddBody(beam1)

    pivot0 = chrono.ChLinkLockRevolute()
    pivot0.Initialize(beam0, ground, chrono.ChFramed(offset, chrono.QUNIT))
    system.AddLink(pivot0)

    joint = chrono.ChLinkLockSpherical()
    joint.Initialize(beam1, beam0, chrono.ChFramed(add(offset, chrono.ChVector3d(BEAM_L, 0, 0)), chrono.QUNIT))
    system.AddLink(joint)

    anchor_pos = add(offset, add(scale(p0, 2.0), rotate_z_y(scale(p0, 2.0), rz, ry)))
    spring = add_native_spring(
        system,
        beam1,
        ground,
        chrono.ChVector3d(0.5 * BEAM_L, 0, 0),
        anchor_pos,
        (anchor_pos - add(offset, p1)).Length(),
        MECH_SPRING_K,
        MECH_SPRING_K * 1.0e-4,
        0.010,
        10,
        "complex eigen two-link Cartesian spring-damper",
    )

    add_anchor(system, "two-link pivot marker", offset, 0.012, color(0.02, 0.02, 0.03))
    add_anchor(system, "two-link spring anchor", anchor_pos, 0.012)
    return {"beam0": beam0, "beam1": beam1, "spring": spring}


def analytic_summary():
    omega0 = math.sqrt(MSD_K / MASS)
    d_rel = MSD_C / (2.0 * math.sqrt(MSD_K * MASS))
    omega = omega0 * math.sqrt(max(0.0, 1.0 - d_rel**2))
    msd_complex = complex(-d_rel * omega0, omega)
    beam_freq = math.sqrt(BEAM_SPRING_K * BEAM_L**2 / beam_theta_zz_about_left()) / (2.0 * math.pi)
    return msd_complex, beam_freq


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
    ground = make_ground(system)

    scenes = {
        "msd": make_mass_spring_scene(system, ground, SCENE_OFFSETS[0]),
        "beam": make_tip_spring_beam_scene(system, ground, SCENE_OFFSETS[1]),
        "mechanism": make_two_link_scene(system, ground, SCENE_OFFSETS[2]),
    }
    update_visuals(system)
    return system, scenes


def update_visuals(system):
    update_system_visuals(system)


def simulate(duration, step):
    system, scenes = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, scenes


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, scenes = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: complexEigenvaluesTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.20, -0.95, 0.55), chrono.ChVector3d(0.0, 0.0, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, scenes)
            next_log += 0.05


def print_state(system, scenes):
    msd_pos = scenes["msd"]["mass"].GetPos()
    beam_tip = scenes["beam"]["spring"].GetPoint1Abs()
    mech_tip = scenes["mechanism"]["spring"].GetPoint1Abs()
    msd_complex, beam_freq = analytic_summary()
    print(
        f"t={system.GetChTime():6.3f}  "
        f"msd_x={msd_pos.x:+.5f}  beam_tip_y={beam_tip.y:+.5f}  mech_tip_norm={mech_tip.Length():.5f}  "
        f"msd_eig={msd_complex.real:+.3f}{msd_complex.imag:+.3f}j  beam_hz={beam_freq:.3f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: complexEigenvaluesTest.py -> PyChrono damped oscillator scenes")
    if args.no_vis:
        system, scenes = simulate(args.duration, args.step)
        print_state(system, scenes)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

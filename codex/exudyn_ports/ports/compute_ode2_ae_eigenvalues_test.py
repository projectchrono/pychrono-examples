import argparse
import math
import sys
from pathlib import Path

import numpy as np
import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN TestModels/computeODE2AEeigenvaluesTest.py:
# compute ODE2 eigenvalues for a hinged beam with algebraic joint constraints,
# then for a two-body constrained mechanism with a Cartesian spring. Chrono does
# not expose the same constrained-ODE2 eigenvalue utility, so this port computes
# the same small-angle hinged-beam analytical value and a reduced two-coordinate
# constrained-mechanism eigenproblem, while rendering the rigid bodies, joints,
# and spring connectors explicitly with coil visuals.

BEAM_L = 0.1
BEAM_W = 0.01
BEAM_H = 0.001
RHO = 5000.0
SPRING_L = 0.02
SPRING_K_SINGLE = 10.0
SPRING_K_MECH = 1.0e3
STEP = 1.0e-3
END_TIME = 0.5


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def v(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def vadd(a, b):
    return chrono.ChVector3d(a.x + b.x, a.y + b.y, a.z + b.z)


def vsub(a, b):
    return chrono.ChVector3d(a.x - b.x, a.y - b.y, a.z - b.z)


def vscale(a, scale):
    return chrono.ChVector3d(a.x * scale, a.y * scale, a.z * scale)


def npv(vec):
    return np.array([vec.x, vec.y, vec.z], dtype=float)


def chv(arr):
    return chrono.ChVector3d(float(arr[0]), float(arr[1]), float(arr[2]))


def rot_y(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)


def rot_z(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def q_mechanism_base():
    return chrono.QuatFromAngleZ(-0.25 * math.pi) * chrono.QuatFromAngleY(0.25 * math.pi)


def add_local_cylinder(body, point_a, point_b, radius, tint):
    segment = chrono.ChLineSegment(point_a, point_b)
    if segment.GetLength() <= 1.0e-12:
        return None
    shape = chrono.ChVisualShapeCylinder(radius, segment.GetLength())
    shape.SetColor(tint)
    body.AddVisualShape(shape, segment.GetFrame())
    return shape


def add_body_axes(body, length=0.035):
    add_local_cylinder(body, v(0, 0, 0), v(length, 0, 0), 0.0012, color(0.92, 0.10, 0.06))
    add_local_cylinder(body, v(0, 0, 0), v(0, length, 0), 0.0012, color(0.08, 0.62, 0.16))
    add_local_cylinder(body, v(0, 0, 0), v(0, 0, length), 0.0012, color(0.08, 0.18, 0.90))


def make_body(system, name, pos, rot, tint):
    body = chrono.ChBodyEasyBox(BEAM_L, BEAM_H, BEAM_W, RHO, True, False)
    body.SetName(name)
    body.EnableCollision(False)
    body.SetPos(pos)
    body.SetRot(rot)
    body.GetVisualShape(0).SetColor(tint)
    body.GetVisualShape(0).SetOpacity(0.78)
    add_body_axes(body)
    system.AddBody(body)
    return body


def make_fixed_sphere(system, name, position, radius, tint):
    sphere = chrono.ChBodyEasySphere(radius, 1000, True, False)
    sphere.SetName(name)
    sphere.SetFixed(True)
    sphere.EnableCollision(False)
    sphere.SetPos(position)
    sphere.GetVisualShape(0).SetColor(tint)
    system.AddBody(sphere)
    return sphere


def make_joint_axis(system, name, position, axis, tint):
    axis_v = chrono.ChVector3d(axis.x, axis.y, axis.z)
    axis_v.Normalize()
    half = vscale(axis_v, 0.018)
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    cyl = chrono.ChVisualShapeCylinder(0.0025, 2.0 * half.Length())
    cyl.SetColor(tint)
    segment = chrono.ChLineSegment(vsub(position, half), vadd(position, half))
    body.AddVisualShape(cyl, segment.GetFrame())
    body.AddVisualShape(chrono.ChVisualShapeSphere(0.006), chrono.ChFramed(position))
    system.AddBody(body)
    return body


def add_visual_tsda(system, name, body_a, body_b, local_a, local_b, radius, turns, tint, stiffness=0.0, damping=0.0):
    spring = chrono.ChLinkTSDA()
    spring.SetName(name)
    spring.Initialize(body_a, body_b, True, local_a, local_b)
    spring.SetSpringCoefficient(stiffness)
    spring.SetDampingCoefficient(damping)
    spring.SetRestLength(max(1.0e-5, spring.GetLength()))
    system.AddLink(spring)
    shape = chrono.ChVisualShapeSpring(radius, 120, turns)
    shape.SetColor(tint)
    spring.AddVisualShape(shape)
    attach_spring_visual(system, spring, radius, 120, turns, tint)
    return spring


def add_fixed_visual_spring(system, name, start, end, tint):
    body_a = chrono.ChBody()
    body_a.SetName(name + " endpoint A")
    body_a.SetFixed(True)
    body_a.EnableCollision(False)
    body_a.SetPos(start)
    marker_a = chrono.ChVisualShapeSphere(0.0035)
    marker_a.SetColor(tint)
    body_a.AddVisualShape(marker_a)
    system.AddBody(body_a)

    body_b = chrono.ChBody()
    body_b.SetName(name + " endpoint B")
    body_b.SetFixed(True)
    body_b.EnableCollision(False)
    body_b.SetPos(end)
    marker_b = chrono.ChVisualShapeSphere(0.0035)
    marker_b.SetColor(tint)
    body_b.AddVisualShape(marker_b)
    system.AddBody(body_b)
    return add_visual_tsda(system, name, body_a, body_b, v(0, 0, 0), v(0, 0, 0), 0.003, 7, tint)


def beam_mass():
    return RHO * BEAM_L * BEAM_H * BEAM_W


def inertia_diag_com():
    mass = beam_mass()
    ixx = mass * (BEAM_H**2 + BEAM_W**2) / 12.0
    iyy = mass * (BEAM_L**2 + BEAM_W**2) / 12.0
    izz = mass * (BEAM_L**2 + BEAM_H**2) / 12.0
    return np.array([ixx, iyy, izz], dtype=float)


def single_hinge_frequency():
    mass = beam_mass()
    theta_zz = inertia_diag_com()[2] + mass * (0.5 * BEAM_L) ** 2
    return math.sqrt(SPRING_K_SINGLE * BEAM_L**2 / theta_zz) / (2.0 * math.pi)


def mechanism_kinematics(q):
    q0, q1 = q
    half = np.array([0.5 * BEAM_L, 0.0, 0.0], dtype=float)
    full = np.array([BEAM_L, 0.0, 0.0], dtype=float)
    r0 = rot_z(q0)
    r1_init = rot_z(-0.25 * math.pi) @ rot_y(0.25 * math.pi)
    r1 = r0 @ r1_init @ rot_y(q1)
    center0 = r0 @ half
    joint = r0 @ full
    center1 = joint + r1 @ half
    spring_point = joint + r1 @ full
    anchor = full + r1_init @ full
    return center0, r0, center1, r1, spring_point, anchor


def mechanism_frequency():
    eps = 1.0e-6
    q0 = np.zeros(2)
    center0, r0, center1, r1, spring_point, anchor = mechanism_kinematics(q0)
    points = []
    centers0 = []
    centers1 = []
    for i in range(2):
        dq = np.zeros(2)
        dq[i] = eps
        c0p, _r0p, c1p, _r1p, spp, _anchor = mechanism_kinematics(dq)
        centers0.append((c0p - center0) / eps)
        centers1.append((c1p - center1) / eps)
        points.append((spp - spring_point) / eps)
    jac_spring = np.column_stack(points)
    stiffness = SPRING_K_MECH * (jac_spring.T @ jac_spring)

    mass = beam_mass()
    inertia_local = inertia_diag_com()
    r1_init = rot_z(-0.25 * math.pi) @ rot_y(0.25 * math.pi)
    omega_axes = [
        [np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, 1.0])],
        [np.array([0.0, 0.0, 0.0]), r1_init @ np.array([0.0, 1.0, 0.0])],
    ]

    mass_matrix = np.zeros((2, 2))
    r0_global = np.eye(3)
    r1_global = r1_init
    inertia0 = r0_global @ np.diag(inertia_local) @ r0_global.T
    inertia1 = r1_global @ np.diag(inertia_local) @ r1_global.T
    for i in range(2):
        for j in range(2):
            mass_matrix[i, j] += mass * centers0[i].dot(centers0[j])
            mass_matrix[i, j] += mass * centers1[i].dot(centers1[j])
            mass_matrix[i, j] += omega_axes[i][0].dot(inertia0 @ omega_axes[j][0])
            mass_matrix[i, j] += omega_axes[i][1].dot(inertia1 @ omega_axes[j][1])

    eigvals = np.linalg.eigvals(np.linalg.solve(mass_matrix, stiffness))
    freqs = np.sqrt(np.clip(np.real(eigvals), 0.0, None)) / (2.0 * math.pi)
    freqs.sort()
    return freqs, mass_matrix, stiffness


def make_reference_plate(system):
    plate = chrono.ChBodyEasyBox(0.36, 0.19, 0.006, 1000, True, False)
    plate.SetName("compute ODE2 AE eigenvalue background")
    plate.SetFixed(True)
    plate.EnableCollision(False)
    plate.SetPos(v(0.10, -0.025, -0.035))
    plate.GetVisualShape(0).SetColor(color(0.76, 0.77, 0.73))
    plate.GetVisualShape(0).SetOpacity(0.32)
    system.AddBody(plate)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(v(0, 0, 0))
    ground = chrono.ChBody()
    ground.SetName("compute ODE2 AE fixed ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    ground_marker = chrono.ChVisualShapeSphere(0.006)
    ground_marker.SetColor(color(0.04, 0.04, 0.045))
    ground.AddVisualShape(ground_marker)
    system.AddBody(ground)
    make_reference_plate(system)

    single_z = 0.03
    single = make_body(system, "single constrained beam with y-spring", v(0.5 * BEAM_L, 0.0, single_z), chrono.QUNIT, color(0.95, 0.52, 0.08))
    joint = chrono.ChLinkLockRevolute()
    joint.SetName("single beam algebraic revolute constraint")
    joint.Initialize(single, ground, chrono.ChFramed(v(0, 0, single_z), chrono.QUNIT))
    system.AddLink(joint)
    make_joint_axis(system, "single beam visible revolute-z axis", v(0, 0, single_z), v(0, 0, 1), color(0.04, 0.04, 0.045))
    make_fixed_sphere(system, "single beam spring ground anchor", v(BEAM_L, -SPRING_L, single_z), 0.0045, color(0.04, 0.04, 0.045))
    spring_single = add_visual_tsda(
        system,
        "single beam y spring-damper coil",
        single,
        ground,
        v(0.5 * BEAM_L, 0, 0),
        v(BEAM_L, -SPRING_L, single_z),
        0.008,
        8,
        color(0.90, 0.18, 0.10),
        SPRING_K_SINGLE,
        0.0,
    )

    mech_offset = np.array([0.0, -0.11, -0.005], dtype=float)
    half = np.array([0.5 * BEAM_L, 0.0, 0.0], dtype=float)
    full = np.array([BEAM_L, 0.0, 0.0], dtype=float)
    r1_init = rot_z(-0.25 * math.pi) @ rot_y(0.25 * math.pi)
    p0 = half + mech_offset
    p1 = full + r1_init @ half + mech_offset
    anchor = full + r1_init @ full + mech_offset
    mech0 = make_body(system, "mechanism beam 0", chv(p0), chrono.QUNIT, color(0.95, 0.52, 0.08))
    mech1 = make_body(system, "mechanism beam 1", chv(p1), q_mechanism_base(), color(0.10, 0.42, 0.90))
    make_joint_axis(system, "mechanism ground revolute-z axis", chv(mech_offset), v(0, 0, 1), color(0.04, 0.04, 0.045))
    make_joint_axis(system, "mechanism inter-body constrained joint marker", chv(full + mech_offset), v(1, 0, 0), color(0.04, 0.04, 0.045))
    make_fixed_sphere(system, "mechanism Cartesian spring anchor", chv(anchor), 0.005, color(0.04, 0.04, 0.045))

    # EXUDYN's CartesianSpringDamper is not a line spring when all three axes are
    # stiff. Show it as three small coil springs along the local x/y/z axes.
    for axis_name, axis, tint in (
        ("x", np.array([1.0, 0.0, 0.0]), color(0.92, 0.10, 0.06)),
        ("y", np.array([0.0, 1.0, 0.0]), color(0.08, 0.62, 0.16)),
        ("z", np.array([0.0, 0.0, 1.0]), color(0.08, 0.18, 0.90)),
    ):
        start = chv(anchor - 0.025 * axis)
        end = chv(anchor + 0.025 * axis)
        add_fixed_visual_spring(system, f"mechanism Cartesian {axis_name}-spring coil", start, end, tint)

    freqs, mass_matrix, stiffness_matrix = mechanism_frequency()
    single_hz = single_hinge_frequency()
    result = freqs[0] / 100.0
    system._compute_ode2_ae_summary = {
        "single_hz": single_hz,
        "mechanism_freqs": freqs,
        "mass_matrix": mass_matrix,
        "stiffness_matrix": stiffness_matrix,
        "result": result,
    }
    update_system_visuals(system)
    return system, single, mech0, mech1, spring_single


def update_visuals(system):
    update_system_visuals(system)


def simulate(duration, step):
    system, single, mech0, mech1, spring_single = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, single, mech0, mech1, spring_single


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, single, mech0, mech1, spring_single = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: computeODE2AEeigenvaluesTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.20, -0.34, 0.20), chrono.ChVector3d(0.10, -0.06, 0.00))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system)
            next_log += 0.25


def print_state(system):
    summary = system._compute_ode2_ae_summary
    freqs = ", ".join(f"{value:.6f}" for value in summary["mechanism_freqs"])
    print(
        f"t={system.GetChTime():6.3f}  "
        f"single_hinge_hz={summary['single_hz']:.9f}  "
        f"mechanism_hz=[{freqs}]  "
        f"source_style_result={summary['result']:.9f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: computeODE2AEeigenvaluesTest.py -> PyChrono constrained eigenvalue scenes")
    if args.no_vis:
        system, single, mech0, mech1, spring_single = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

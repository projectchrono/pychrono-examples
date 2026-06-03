import argparse
import math

import numpy as np
import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/computeODE2EigenvaluesTest.py:
# build a 32-element cable/beam ODE2 model, compute its eigenvalues, and compare
# the first three bending frequencies with analytical free-free beam roots.
# Chrono does not expose EXUDYN's ANCF Cable2D eigenvalue helper, so this port
# assembles the equivalent Euler-Bernoulli beam FE mass/stiffness matrices and
# renders the first three bending mode shapes.

L = 2.0
E = 2.07e11
RHO = 7800.0
B = 0.01
H = 0.01
A = B * H
I = B * H**3 / 12.0
EI = E * I
EA_REDUCED = E * A * 0.1
RHO_A = RHO * A
N_ELEMENTS = 32
N_EIG = 3
STEP = 1.0e-3
END_TIME = 0.4

BETA_FREE_FREE = np.array(
    [
        4.730040744862704,
        7.853204624095838,
        10.99560783800167,
    ]
)


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_line_body(system, name, points, tint, thickness=3):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    line = chrono.ChLinePoly(len(points))
    for i, point in enumerate(points):
        line.SetPoint(i, point)
    shape = chrono.ChVisualShapeLine()
    shape.SetLineGeometry(line)
    shape.SetThickness(thickness)
    shape.SetColor(tint)
    body.AddVisualShape(shape)
    system.AddBody(body)
    return body


def add_box(system, name, size, pos, tint, opacity=1.0):
    body = chrono.ChBodyEasyBox(size.x, size.y, size.z, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(pos)
    body.GetVisualShape(0).SetColor(tint)
    body.GetVisualShape(0).SetOpacity(opacity)
    system.AddBody(body)
    return body


def add_sphere(system, name, radius, pos, tint):
    body = chrono.ChBodyEasySphere(radius, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(pos)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def assemble_beam_matrices():
    n_nodes = N_ELEMENTS + 1
    n_dof = 3 * n_nodes
    le = L / N_ELEMENTS
    stiffness = np.zeros((n_dof, n_dof))
    mass = np.zeros((n_dof, n_dof))

    k_axial = EA_REDUCED / le * np.array([[1.0, -1.0], [-1.0, 1.0]])
    m_axial = RHO_A * le / 6.0 * np.array([[2.0, 1.0], [1.0, 2.0]])
    k_bending = EI / le**3 * np.array(
        [
            [12.0, 6.0 * le, -12.0, 6.0 * le],
            [6.0 * le, 4.0 * le**2, -6.0 * le, 2.0 * le**2],
            [-12.0, -6.0 * le, 12.0, -6.0 * le],
            [6.0 * le, 2.0 * le**2, -6.0 * le, 4.0 * le**2],
        ]
    )
    m_bending = RHO_A * le / 420.0 * np.array(
        [
            [156.0, 22.0 * le, 54.0, -13.0 * le],
            [22.0 * le, 4.0 * le**2, 13.0 * le, -3.0 * le**2],
            [54.0, 13.0 * le, 156.0, -22.0 * le],
            [-13.0 * le, -3.0 * le**2, -22.0 * le, 4.0 * le**2],
        ]
    )

    for element in range(N_ELEMENTS):
        left = element
        right = element + 1
        axial_dofs = [3 * left, 3 * right]
        bending_dofs = [3 * left + 1, 3 * left + 2, 3 * right + 1, 3 * right + 2]
        for i, gi in enumerate(axial_dofs):
            for j, gj in enumerate(axial_dofs):
                stiffness[gi, gj] += k_axial[i, j]
                mass[gi, gj] += m_axial[i, j]
        for i, gi in enumerate(bending_dofs):
            for j, gj in enumerate(bending_dofs):
                stiffness[gi, gj] += k_bending[i, j]
                mass[gi, gj] += m_bending[i, j]
    return mass, stiffness


def compute_eigenpairs():
    mass, stiffness = assemble_beam_matrices()
    system_matrix = np.linalg.solve(mass, stiffness)
    eigvals, eigvecs = np.linalg.eig(system_matrix)
    eigvals = np.real(eigvals)
    eigvecs = np.real(eigvecs)
    omegas = np.sqrt(np.clip(eigvals, 0.0, None))
    order = np.argsort(omegas)
    omegas = omegas[order]
    eigvecs = eigvecs[:, order]

    numerical = omegas[3 : 3 + N_EIG]
    vectors = eigvecs[:, 3 : 3 + N_EIG]
    analytical = ((BETA_FREE_FREE / L) ** 4 * (EI / RHO_A)) ** 0.5
    difference = analytical - numerical
    return {
        "mass": mass,
        "stiffness": stiffness,
        "omegas": numerical,
        "vectors": vectors,
        "analytical": analytical,
        "difference": difference,
        "rel_error": difference / analytical,
    }


def mode_points(vector, y_offset, scale=0.30):
    n_nodes = N_ELEMENTS + 1
    transverse = np.array([vector[3 * i + 1] for i in range(n_nodes)], dtype=float)
    transverse /= max(float(np.max(np.abs(transverse))), 1.0e-12)
    return [
        chrono.ChVector3d(L * i / (n_nodes - 1), y_offset + scale * transverse[i], 0.0)
        for i in range(n_nodes)
    ]


def build_system():
    eig = compute_eigenpairs()
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    add_box(
        system,
        "compute ODE2 eigenvalue background plate",
        chrono.ChVector3d(2.55, 1.82, 0.025),
        chrono.ChVector3d(1.0, 0.0, -0.05),
        color(0.77, 0.78, 0.74),
        0.35,
    )
    baseline = [chrono.ChVector3d(0, 0, 0.02), chrono.ChVector3d(L, 0, 0.02)]
    make_line_body(system, "undeformed free-free beam reference", baseline, color(0.04, 0.04, 0.045), 4)

    palette = [
        color(0.10, 0.38, 0.88),
        color(0.10, 0.62, 0.28),
        color(0.92, 0.56, 0.08),
    ]
    y_offsets = [0.48, 0.0, -0.48]
    for mode, (omega, analytical, diff, tint, y_offset) in enumerate(
        zip(eig["omegas"], eig["analytical"], eig["difference"], palette, y_offsets),
        start=1,
    ):
        make_line_body(
            system,
            f"compute ODE2 eigenvalue bending mode {mode}",
            mode_points(eig["vectors"][:, mode - 1], y_offset),
            tint,
            5,
        )
        add_sphere(system, f"mode {mode} analytical frequency marker", 0.030, chrono.ChVector3d(2.12, y_offset, 0.04), tint)
        bar_height = min(0.24, max(0.025, abs(float(diff / analytical)) * 7000.0))
        add_box(
            system,
            f"mode {mode} relative-error bar numerical {omega:.6f} analytical {analytical:.6f}",
            chrono.ChVector3d(0.055, 0.055, bar_height),
            chrono.ChVector3d(2.30, y_offset, 0.5 * bar_height),
            color(0.08, 0.68, 0.20) if abs(diff / analytical) < 1.0e-4 else color(0.90, 0.20, 0.08),
        )

    system._compute_ode2_eigen_summary = eig
    return system, eig


def simulate(duration, step):
    system, eig = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, eig


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, eig = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: computeODE2EigenvaluesTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.05, -2.75, 1.85), chrono.ChVector3d(1.0, 0.0, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, eig)
            next_log += 0.2


def print_state(system, eig):
    numerical = ", ".join(f"{value:.6f}" for value in eig["omegas"])
    analytical = ", ".join(f"{value:.6f}" for value in eig["analytical"])
    diff = eig["difference"][0]
    print(
        f"t={system.GetChTime():6.3f}  "
        f"omega_numerical=[{numerical}]  "
        f"omega_analytical=[{analytical}]  "
        f"omega_difference={diff:+.9e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: computeODE2EigenvaluesTest.py -> PyChrono beam eigenvalue check")
    if args.no_vis:
        system, eig = simulate(args.duration, args.step)
        print_state(system, eig)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

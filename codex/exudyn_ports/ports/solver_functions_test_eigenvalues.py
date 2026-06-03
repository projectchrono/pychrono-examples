import argparse
import math

import numpy as np
import pychrono.core as chrono


# Reproduces EXUDYN Examples/solverFunctionsTestEigenvalues.py:
# the source builds a 32-element ANCF cable, extracts ODE2 mass and stiffness
# matrices from EXUDYN's static solver, and compares the first four bending
# eigenfrequencies with analytical free-free beam values.  This PyChrono port
# reproduces that solver-function example by assembling an equivalent
# Euler-Bernoulli beam FE eigenproblem with axial, transverse, and rotation DOFs
# at every node, then visualizes the first four bending mode shapes.

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
N_EIG = 4
STEP = 1.0e-3
END_TIME = 0.6

BETA_FREE_FREE = np.array(
    [
        4.730040744862704,
        7.853204624095838,
        10.99560783800167,
        14.13716549125746,
    ]
)


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def add(a, b):
    return chrono.ChVector3d(a.x + b.x, a.y + b.y, a.z + b.z)


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
    body.SetPos(pos)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(tint)
    body.GetVisualShape(0).SetOpacity(opacity)
    system.AddBody(body)
    return body


def add_sphere(system, name, radius, pos, tint):
    body = chrono.ChBodyEasySphere(radius, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.SetPos(pos)
    body.EnableCollision(False)
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

    bending_omegas = omegas[3 : 3 + N_EIG]
    bending_vecs = eigvecs[:, 3 : 3 + N_EIG]
    analytical = ((BETA_FREE_FREE / L) ** 4 * (EI / RHO_A)) ** 0.5
    rel_error = (bending_omegas - analytical) / analytical
    return {
        "mass": mass,
        "stiffness": stiffness,
        "all_omegas": omegas,
        "omegas": bending_omegas,
        "vectors": bending_vecs,
        "analytical": analytical,
        "rel_error": rel_error,
    }


def mode_points(vector, y_offset, scale=0.33):
    n_nodes = N_ELEMENTS + 1
    transverse = np.array([vector[3 * i + 1] for i in range(n_nodes)], dtype=float)
    max_abs = max(float(np.max(np.abs(transverse))), 1.0e-12)
    transverse = transverse / max_abs
    points = []
    for i in range(n_nodes):
        x = L * i / (n_nodes - 1)
        points.append(chrono.ChVector3d(x, y_offset + scale * transverse[i], 0.0))
    return points


def build_system():
    eig = compute_eigenpairs()
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    add_box(
        system,
        "solver eigenvalue background plate",
        chrono.ChVector3d(2.55, 2.25, 0.025),
        chrono.ChVector3d(1.0, 0.02, -0.045),
        color(0.77, 0.79, 0.76),
        0.38,
    )

    # Source background primitives: a rectangle, a small circle marker, and a
    # text placeholder represented as dark ticks because Chrono core has no text
    # visual shape in this environment.
    rect = [
        chrono.ChVector3d(-0.22, -0.92, 0.01),
        chrono.ChVector3d(2.22, -0.92, 0.01),
        chrono.ChVector3d(2.22, 0.92, 0.01),
        chrono.ChVector3d(-0.22, 0.92, 0.01),
        chrono.ChVector3d(-0.22, -0.92, 0.01),
    ]
    make_line_body(system, "source-style blue rectangle background", rect, color(0.1, 0.1, 0.8), 3)

    circle = []
    for i in range(49):
        angle = 2.0 * math.pi * i / 48.0
        circle.append(chrono.ChVector3d(-0.08 + 0.08 * math.cos(angle), 0.0 + 0.08 * math.sin(angle), 0.02))
    make_line_body(system, "source-style circle background", circle, color(0.08, 0.08, 0.08), 3)
    for i in range(3):
        make_line_body(
            system,
            f"source-style text placeholder line {i + 1}",
            [chrono.ChVector3d(0.20, -0.78 - 0.07 * i, 0.02), chrono.ChVector3d(0.72 - 0.10 * i, -0.78 - 0.07 * i, 0.02)],
            color(0.04, 0.04, 0.045),
            2,
        )

    palette = [
        color(0.12, 0.42, 0.85),
        color(0.10, 0.62, 0.28),
        color(0.95, 0.64, 0.08),
        color(0.76, 0.20, 0.76),
    ]
    y_offsets = [0.58, 0.20, -0.18, -0.56]
    for mode, (omega, analytical, rel_error, tint, y_offset) in enumerate(
        zip(eig["omegas"], eig["analytical"], eig["rel_error"], palette, y_offsets),
        start=1,
    ):
        make_line_body(
            system,
            f"free-free beam eigenmode {mode}",
            mode_points(eig["vectors"][:, mode - 1], y_offset),
            tint,
            5,
        )
        add_sphere(system, f"mode {mode} analytical root marker", 0.030, chrono.ChVector3d(2.12, y_offset, 0.04), tint)
        error_height = min(0.26, max(0.025, abs(float(rel_error)) * 7000.0))
        add_box(
            system,
            f"mode {mode} relative-error bar",
            chrono.ChVector3d(0.055, 0.055, error_height),
            chrono.ChVector3d(2.30, y_offset, 0.5 * error_height),
            color(0.08, 0.68, 0.20) if abs(rel_error) < 1.0e-4 else color(0.90, 0.20, 0.08),
        )
        # Numeric metadata is kept in body names for scene inspection.
        add_box(
            system,
            f"mode {mode} omega numerical {omega:.6f} analytical {analytical:.6f}",
            chrono.ChVector3d(0.03, 0.20, 0.06),
            chrono.ChVector3d(-0.12, y_offset, 0.04),
            tint,
        )

    system._eigen_summary = eig
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
    vis.SetWindowTitle("EXUDYN port: solverFunctionsTestEigenvalues.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.05, -3.10, 2.10), chrono.ChVector3d(1.0, 0.0, 0.0))
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
            next_log += 0.25


def print_state(system, eig):
    values = ", ".join(f"{w:.6f}" for w in eig["omegas"])
    analytical = ", ".join(f"{w:.6f}" for w in eig["analytical"])
    max_rel = float(np.max(np.abs(eig["rel_error"])))
    print(
        f"t={system.GetChTime():6.3f}  omega_numerical=[{values}]  "
        f"omega_analytical=[{analytical}]  max_rel_error={max_rel:.3e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: solverFunctionsTestEigenvalues.py -> PyChrono beam eigenvalue scene")
    if args.no_vis:
        system, eig = simulate(args.duration, args.step)
        print_state(system, eig)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

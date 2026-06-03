import argparse
import math
import sys
from pathlib import Path

import numpy as np
import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces EXUDYN Examples/SpringDamperMassUserFunction.py:
# a 24x3 point-grid whose ObjectConnectorSpringDamper elements use a linear
# springForceUserFunction, optionally converted to a symbolic user function.
# The port solves the static 2D spring equilibrium directly with the same
# distance/reference-length law, then renders every point and spring in
# PyChrono with native coil spring visualization and the capture fallback.

N_BODIES = 24
N_ROWS = 3
MASS = 10.0
STIFFNESS = 4000.0
DAMPING = 10.0
LOAD_Y = -0.025
STEP = 1.0e-3
END_TIME = 1.0
REFERENCE_TIP_Y = -0.44056224799446486


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def point_index(i, j):
    return j * N_BODIES + i


def source_points():
    return np.array([[float(i), float(j)] for j in range(N_ROWS) for i in range(N_BODIES)])


def source_edges():
    sqrt2 = math.sqrt(2.0)
    edges = []
    for j in range(N_ROWS - 1):
        for i in range(N_BODIES - 1):
            edges.append((point_index(i, j), point_index(i + 1, j), 1.0))
            edges.append((point_index(i, j), point_index(i, j + 1), 1.0))
            edges.append((point_index(i, j), point_index(i + 1, j + 1), sqrt2))
    for i in range(N_BODIES - 1):
        j = N_ROWS - 1
        edges.append((point_index(i, j), point_index(i + 1, j), 1.0))
    for j in range(N_ROWS - 1):
        i = N_BODIES - 1
        edges.append((point_index(i, j), point_index(i, j + 1), 1.0))
    return edges


def fixed_mask():
    return np.array([i == 0 for j in range(N_ROWS) for i in range(N_BODIES)])


def residual_and_jacobian(q, reference, free, free_slot, edges):
    positions = reference.copy()
    positions[free] = q.reshape((-1, 2))

    residual = np.zeros((len(free), 2))
    residual[:, 1] = LOAD_Y
    jacobian = np.zeros((2 * len(free), 2 * len(free)))
    identity = np.eye(2)

    for a, b, rest_length in edges:
        pa = positions[a]
        pb = positions[b]
        delta = pb - pa
        length = float(np.linalg.norm(delta))
        if length < 1e-12:
            continue
        direction = delta / length
        force = STIFFNESS * (length - rest_length) * direction
        tangent = STIFFNESS * (
            np.outer(direction, direction)
            + (length - rest_length) / length * (identity - np.outer(direction, direction))
        )

        ia = free_slot[a]
        ib = free_slot[b]
        if ia >= 0:
            residual[ia] += force
            jacobian[2 * ia : 2 * ia + 2, 2 * ia : 2 * ia + 2] += -tangent
            if ib >= 0:
                jacobian[2 * ia : 2 * ia + 2, 2 * ib : 2 * ib + 2] += tangent
        if ib >= 0:
            residual[ib] -= force
            jacobian[2 * ib : 2 * ib + 2, 2 * ib : 2 * ib + 2] += -tangent
            if ia >= 0:
                jacobian[2 * ib : 2 * ib + 2, 2 * ia : 2 * ia + 2] += tangent

    return residual.reshape(-1), jacobian, positions


def residual_norm(q, reference, free, free_slot, edges):
    residual, _, _ = residual_and_jacobian(q, reference, free, free_slot, edges)
    return float(np.linalg.norm(residual, ord=np.inf))


def solve_static_equilibrium(max_iterations=160, tolerance=1.0e-10):
    reference = source_points()
    fixed = fixed_mask()
    free = np.where(~fixed)[0]
    free_slot = -np.ones(len(reference), dtype=int)
    free_slot[free] = np.arange(len(free))
    edges = source_edges()
    q = reference[free].reshape(-1).copy()

    iterations = 0
    residual = float("inf")
    for iterations in range(1, max_iterations + 1):
        values, jacobian, positions = residual_and_jacobian(q, reference, free, free_slot, edges)
        residual = float(np.linalg.norm(values, ord=np.inf))
        if residual < tolerance:
            break
        step = np.linalg.solve(jacobian, -values)
        alpha = 1.0
        while alpha > 1.0e-5:
            candidate = q + alpha * step
            if residual_norm(candidate, reference, free, free_slot, edges) < residual:
                q = candidate
                break
            alpha *= 0.5
        else:
            q = q + 1.0e-5 * step

    _, _, positions = residual_and_jacobian(q, reference, free, free_slot, edges)
    return reference, positions, edges, iterations, residual


STATIC_REFERENCE, STATIC_POSITIONS, SPRING_EDGES, STATIC_ITERATIONS, STATIC_RESIDUAL = solve_static_equilibrium()


def interpolate_positions(time):
    alpha = min(1.0, max(0.0, time / END_TIME))
    smooth = alpha * alpha * (3.0 - 2.0 * alpha)
    return STATIC_REFERENCE + smooth * (STATIC_POSITIONS - STATIC_REFERENCE)


def displacement_tint(displacement_y):
    s = min(1.0, max(0.0, abs(displacement_y) / max(1.0e-12, abs(REFERENCE_TIP_Y))))
    return color(0.10 + 0.75 * s, 0.34 * (1.0 - s) + 0.12 * s, 0.88 * (1.0 - s) + 0.12 * s)


def make_node_body(system, point_id):
    i = point_id % N_BODIES
    fixed = i == 0
    radius = 0.090 if fixed else 0.074
    body = chrono.ChBodyEasySphere(radius, 1000, True, False)
    body.SetName(f"symbolic spring grid point {point_id}")
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(vector_from_point(STATIC_REFERENCE[point_id]))
    if fixed:
        body.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.08))
    else:
        body.GetVisualShape(0).SetColor(displacement_tint(STATIC_POSITIONS[point_id, 1] - STATIC_REFERENCE[point_id, 1]))
    system.AddBody(body)
    return body


def vector_from_point(point):
    return chrono.ChVector3d(float(point[0]), float(point[1]), 0.0)


def add_spring(system, bodies, edge):
    a, b, rest_length = edge
    spring = chrono.ChLinkTSDA()
    spring.SetName("symbolic user-function spring")
    spring.Initialize(bodies[a], bodies[b], True, chrono.ChVector3d(0.0, 0.0, 0.0), chrono.ChVector3d(0.0, 0.0, 0.0))
    spring.SetRestLength(rest_length)
    spring.SetSpringCoefficient(0.0)
    spring.SetDampingCoefficient(0.0)
    system.AddLink(spring)

    shape = chrono.ChVisualShapeSpring(0.030, 48, 7)
    shape.SetColor(color(0.86, 0.18, 0.12))
    spring.AddVisualShape(shape)
    attach_spring_visual(system, spring, 0.030, 48, 7, color(0.86, 0.18, 0.12))
    return spring


def make_ground_guides(system):
    ground = chrono.ChBody()
    ground.SetName("SpringDamperMassUserFunction guides")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    for row in range(N_ROWS):
        base = chrono.ChVisualShapeBox(N_BODIES - 1, 0.018, 0.018)
        base.SetColor(color(0.45, 0.45, 0.45))
        ground.AddVisualShape(base, chrono.ChFramed(chrono.ChVector3d(0.5 * (N_BODIES - 1), float(row), -0.06)))

    clamp = chrono.ChVisualShapeBox(0.12, N_ROWS - 1 + 0.35, 0.035)
    clamp.SetColor(color(0.30, 0.30, 0.30))
    ground.AddVisualShape(clamp, chrono.ChFramed(chrono.ChVector3d(0.0, 0.5 * (N_ROWS - 1), -0.08)))
    system.AddBody(ground)
    return ground


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))

    guides = make_ground_guides(system)
    bodies = [make_node_body(system, point_id) for point_id in range(N_BODIES * N_ROWS)]
    springs = [add_spring(system, bodies, edge) for edge in SPRING_EDGES]

    items = {"guides": guides, "bodies": bodies, "springs": springs}
    system._spring_damper_mass_user_function_items = items
    update_visuals(system)
    return system, items


def set_grid_positions(items, time):
    positions = interpolate_positions(time)
    for point_id, body in enumerate(items["bodies"]):
        body.SetPos(vector_from_point(positions[point_id]))
        body.UpdateVisualModel()


def update_visuals(system):
    items = system._spring_damper_mass_user_function_items
    set_grid_positions(items, system.GetChTime())
    update_system_visuals(system)


def simulate(duration, step):
    system, items = build_system()
    set_grid_positions(items, duration)
    update_system_visuals(system)
    system._spring_damper_mass_user_function_time = duration
    return system, items


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 760)
    vis.SetWindowTitle("EXUDYN port: SpringDamperMassUserFunction.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(12.0, -9.0, 12.0), chrono.ChVector3d(12.0, 0.3, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if system.GetChTime() >= next_log:
            print_state(system)
            next_log += 0.25


def tip_position():
    return STATIC_POSITIONS[point_index(N_BODIES - 1, 0)]


def print_state(system):
    tip = tip_position()
    time_value = getattr(system, "_spring_damper_mass_user_function_time", system.GetChTime() if system is not None else END_TIME)
    print(
        f"t={time_value:6.3f}  "
        f"points={N_BODIES * N_ROWS}  springs={len(SPRING_EDGES)}  "
        f"static_iterations={STATIC_ITERATIONS}  residual={STATIC_RESIDUAL:.3e}  "
        f"tip=({tip[0]:+.9f},{tip[1]:+.9f})  "
        f"tip_y_error={tip[1] - REFERENCE_TIP_Y:+.3e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: SpringDamperMassUserFunction.py -> PyChrono symbolic user-spring grid")
    if args.no_vis:
        system, _ = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

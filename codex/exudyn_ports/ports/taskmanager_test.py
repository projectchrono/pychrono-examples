import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces EXUDYN TestModels/taskmanagerTest.py at the level of its
# systematic case sweep: combinations of thread count, ODE1 coordinates,
# mass-point chains, distance constraints, user functions, static solves, and
# explicit solvers.  The PyChrono scene renders representative case panels and a
# 120-case status board, while the headless path evaluates a deterministic
# mass-spring/constraint analogue for the same case vectors.

A = 1.0
B = 0.5
GRAVITY = 9.81
MASS = 0.2
STIFFNESS = 1000.0
DAMPING = 2.0
END_TIME = 2.0
STEP = 0.01


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def vadd(a, b):
    return chrono.ChVector3d(a.x + b.x, a.y + b.y, a.z + b.z)


def vsub(a, b):
    return chrono.ChVector3d(a.x - b.x, a.y - b.y, a.z - b.z)


def vscale(a, scale):
    return chrono.ChVector3d(a.x * scale, a.y * scale, a.z * scale)


def enumerate_cases():
    cases = []
    for n_masses in [0, 1, 2, 4]:
        for constraints in range(2):
            for static in range(2):
                for user in range(2):
                    for ode1 in range(2):
                        for threads in [1, 2, 5]:
                            if static and ode1:
                                continue
                            if n_masses == 0 and not ode1:
                                continue
                            explicit = int(n_masses == 2 and constraints == 0 and static == 0)
                            cases.append((threads, ode1, n_masses, constraints, user, static, explicit))
    return cases


def ode1_result(duration=END_TIME, step=STEP):
    q = [1.0, 0.0, 0.0]

    def deriv(state):
        return [state[1], -100.0 * state[0] + 1.0, -10.0 * state[2] + 50.0]

    time = 0.0
    while time < duration - 1.0e-12:
        h = min(step, duration - time)

        def add_scaled(base, slope, scale):
            return [base[i] + scale * slope[i] for i in range(3)]

        k1 = deriv(q)
        k2 = deriv(add_scaled(q, k1, 0.5 * h))
        k3 = deriv(add_scaled(q, k2, 0.5 * h))
        k4 = deriv(add_scaled(q, k3, h))
        q = [q[i] + h * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0 for i in range(3)]
        time += h
    return q


def user_load(time):
    factor = math.sin(4.0 * math.pi * time)
    return vec(10.0 * factor, 20.0 * factor, 0.0)


def endpoint_position(endpoint, positions):
    if endpoint == "g0":
        return vec(0.0, 0.0, 0.0)
    if endpoint == "g1":
        return vec(0.0, B, 0.0)
    return positions[endpoint]


def endpoint_velocity(endpoint, velocities):
    if endpoint in ("g0", "g1"):
        return vec(0.0, 0.0, 0.0)
    return velocities[endpoint]


def add_endpoint_force(endpoint, forces, force):
    if endpoint not in ("g0", "g1"):
        forces[endpoint] = vadd(forces[endpoint], force)


def spring_force_between(endpoint_a, endpoint_b, rest_length, positions, velocities, forces, nonlinear=False):
    pa = endpoint_position(endpoint_a, positions)
    pb = endpoint_position(endpoint_b, positions)
    delta = vsub(pb, pa)
    length = delta.Length()
    if length < 1.0e-12:
        return
    direction = vscale(delta, 1.0 / length)
    va = endpoint_velocity(endpoint_a, velocities)
    vb = endpoint_velocity(endpoint_b, velocities)
    extension = length - rest_length
    rel_speed = (vb.x - va.x) * direction.x + (vb.y - va.y) * direction.y
    force_value = STIFFNESS * extension + DAMPING * rel_speed
    if nonlinear:
        force_value += STIFFNESS * extension**3
    force = vscale(direction, force_value)
    add_endpoint_force(endpoint_a, forces, force)
    add_endpoint_force(endpoint_b, forces, vscale(force, -1.0))


def project_distance(endpoint_a, endpoint_b, rest_length, positions):
    pa = endpoint_position(endpoint_a, positions)
    pb = endpoint_position(endpoint_b, positions)
    delta = vsub(pb, pa)
    length = delta.Length()
    if length < 1.0e-12:
        return
    correction = vscale(delta, (length - rest_length) / length)
    a_fixed = endpoint_a in ("g0", "g1")
    b_fixed = endpoint_b in ("g0", "g1")
    if not a_fixed and not b_fixed:
        positions[endpoint_a] = vadd(positions[endpoint_a], vscale(correction, 0.5))
        positions[endpoint_b] = vsub(positions[endpoint_b], vscale(correction, 0.5))
    elif a_fixed and not b_fixed:
        positions[endpoint_b] = vsub(positions[endpoint_b], correction)
    elif b_fixed and not a_fixed:
        positions[endpoint_a] = vadd(positions[endpoint_a], correction)


def build_case_topology(n_masses, constraints, user):
    positions = []
    velocities = []
    refs = []
    springs = []
    distance_constraints = []
    last_bottom = "g0"
    last_top = "g1"
    for i in range(n_masses):
        bottom = 2 * i
        top = 2 * i + 1
        refs.append(vec((i + 1) * A, 0.0, 0.0))
        refs.append(vec((i + 1) * A, B, 0.0))
        positions.extend([refs[-2], refs[-1]])
        velocities.extend([vec(0.0, 0.0, 0.0), vec(0.0, 0.0, 0.0)])
        springs.append((last_top, top, A, bool(user)))
        if constraints:
            distance_constraints.extend(
                [
                    (last_bottom, bottom, A),
                    (last_top, bottom, math.sqrt(A * A + B * B)),
                    (bottom, top, B),
                ]
            )
        else:
            springs.extend(
                [
                    (last_bottom, bottom, A, False),
                    (last_top, bottom, math.sqrt(A * A + B * B), False),
                    (bottom, top, B, False),
                ]
            )
        last_bottom = bottom
        last_top = top
    return positions, velocities, refs, springs, distance_constraints, last_bottom, last_top


def simulate_mass_case(case):
    _threads, _ode1, n_masses, constraints, user, static, explicit = case
    if n_masses == 0:
        return vec(0.0, 0.0, 0.0)

    positions, velocities, refs, springs, distance_constraints, last_bottom, last_top = build_case_topology(n_masses, constraints, user)
    n_steps = int(END_TIME / STEP)
    damping_factor = 0.90 if static else 1.0
    for step_index in range(n_steps):
        time = step_index * STEP
        forces = [vec(0.0, -MASS * GRAVITY, 0.0) for _ in positions]
        if user:
            add_endpoint_force(last_top, forces, user_load(time))
        for endpoint_a, endpoint_b, rest, nonlinear in springs:
            spring_force_between(endpoint_a, endpoint_b, rest, positions, velocities, forces, nonlinear)
        for i in range(len(positions)):
            velocities[i] = vadd(velocities[i], vscale(forces[i], STEP / MASS))
            velocities[i] = vscale(velocities[i], damping_factor)
            positions[i] = vadd(positions[i], vscale(velocities[i], STEP))
        if constraints:
            for _ in range(5):
                for endpoint_a, endpoint_b, rest in distance_constraints:
                    project_distance(endpoint_a, endpoint_b, rest, positions)
        if explicit:
            # The source uses RK44/VelocityVerlet for this family; slightly
            # damp the semi-implicit replay to separate it in the diagnostics.
            for i in range(len(velocities)):
                velocities[i] = vscale(velocities[i], 0.995)
    return vsub(positions[last_bottom], refs[last_bottom])


def evaluate_case(case):
    displacement = simulate_mass_case(case)
    ode = ode1_result() if case[1] else [0.0, 0.0, 0.0]
    contribution = displacement.x + displacement.y + displacement.z + sum(ode)
    return {
        "case": case,
        "tip_displacement": (displacement.x, displacement.y, displacement.z),
        "ode1": tuple(ode),
        "contribution": contribution,
    }


def evaluate_sweep():
    results = [evaluate_case(case) for case in enumerate_cases()]
    total = 0.001 * sum(item["contribution"] for item in results)
    explicit = sum(1 for item in results if item["case"][6])
    static = sum(1 for item in results if item["case"][5])
    ode1 = sum(1 for item in results if item["case"][1])
    constraints = sum(1 for item in results if item["case"][3])
    return {"results": results, "total": total, "explicit": explicit, "static": static, "ode1": ode1, "constraints": constraints}


def add_box(system, name, size, pos, tint, opacity=1.0):
    body = chrono.ChBodyEasyBox(size.x, size.y, size.z, 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(pos)
    body.GetVisualShape(0).SetColor(tint)
    body.GetVisualShape(0).SetOpacity(opacity)
    system.AddBody(body)
    return body


def add_sphere(system, name, radius, pos, tint):
    body = chrono.ChBodyEasySphere(radius, 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(pos)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def add_segment(system, name, a, b, tint, thickness=3):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    shape = chrono.ChVisualShapeSegment()
    shape.SetColor(tint)
    shape.SetThickness(thickness)
    shape.SetLineGeometry(chrono.ChLineSegment(a, b))
    body.AddVisualShape(shape)
    system.AddBody(body)
    return body


def add_visual_spring(system, name, point_a, point_b, tint):
    body_a = add_sphere(system, name + " endpoint A", 0.020, point_a, color(0.05, 0.05, 0.055))
    body_b = add_sphere(system, name + " endpoint B", 0.020, point_b, color(0.05, 0.05, 0.055))
    spring = chrono.ChLinkTSDA()
    spring.SetName(name)
    spring.Initialize(body_a, body_b, True, vec(0, 0, 0), vec(0, 0, 0))
    spring.SetRestLength(max(1.0e-6, vsub(point_b, point_a).Length()))
    spring.SetSpringCoefficient(STIFFNESS)
    spring.SetDampingCoefficient(DAMPING)
    system.AddLink(spring)
    shape = chrono.ChVisualShapeSpring(0.035, 90, 11)
    shape.SetColor(tint)
    spring.AddVisualShape(shape)
    attach_spring_visual(system, spring, 0.035, 90, 11, tint)
    return spring


def case_color(case):
    _threads, ode1, _n_masses, constraints, user, static, explicit = case
    if explicit:
        return color(0.95, 0.50, 0.08)
    if static:
        return color(0.55, 0.55, 0.58)
    if ode1:
        return color(0.62, 0.28, 0.86)
    if constraints:
        return color(0.10, 0.46, 0.90)
    if user:
        return color(0.10, 0.62, 0.25)
    return color(0.84, 0.22, 0.16)


def add_case_board(system, summary):
    add_box(system, "taskmanagerTest 120-case board", vec(5.6, 1.55, 0.030), vec(1.90, 1.85, -0.02), color(0.72, 0.74, 0.72), 0.30)
    cols = 20
    cell = 0.18
    for index, item in enumerate(summary["results"]):
        col = index % cols
        row = index // cols
        pos = vec(0.18 + col * cell, 2.35 - row * cell, 0.030)
        add_box(system, f"taskmanagerTest case {index:03d} {item['case']}", vec(0.105, 0.105, 0.035), pos, case_color(item["case"]))


def add_sample_spring_panel(system, x0, y0):
    points = [vec(x0, y0, 0.0), vec(x0 + 0.6, y0, 0.0), vec(x0 + 1.2, y0, 0.0), vec(x0, y0 + 0.5, 0.0), vec(x0 + 0.6, y0 + 0.5, 0.0), vec(x0 + 1.2, y0 + 0.5, 0.0)]
    for i, point in enumerate(points):
        add_sphere(system, f"taskmanagerTest spring panel node {i}", 0.045, point, color(0.84, 0.16, 0.12) if i % 3 else color(0.06, 0.06, 0.065))
    spring_pairs = [(0, 1), (3, 4), (1, 2), (4, 5), (1, 4), (3, 1)]
    for i, (a, b) in enumerate(spring_pairs):
        add_visual_spring(system, f"taskmanagerTest native spring sample {i}", points[a], points[b], color(0.88, 0.16, 0.08))


def add_sample_constraint_panel(system, x0, y0):
    points = [vec(x0, y0, 0.0), vec(x0 + 0.58, y0 - 0.10, 0.0), vec(x0 + 0.58, y0 + 0.40, 0.0), vec(x0 + 1.15, y0 + 0.04, 0.0), vec(x0 + 1.15, y0 + 0.54, 0.0)]
    for i, point in enumerate(points):
        add_sphere(system, f"taskmanagerTest constraint panel node {i}", 0.045, point, color(0.10, 0.42, 0.88) if i else color(0.06, 0.06, 0.065))
    for i, (a, b) in enumerate([(0, 1), (0, 2), (1, 2), (1, 3), (2, 3), (3, 4)]):
        add_segment(system, f"taskmanagerTest distance constraint guide {i}", points[a], points[b], color(0.05, 0.05, 0.055), 5)


def add_ode1_trace(system, x0, y0):
    points = []
    q = ode1_result
    for i in range(101):
        t = END_TIME * i / 100.0
        value = ode1_value_at(t)[0]
        points.append(vec(x0 + 1.6 * i / 100.0, y0 + 0.13 * value, 0.04))
    body = chrono.ChBody()
    body.SetName("taskmanagerTest ODE1 trace")
    body.SetFixed(True)
    body.EnableCollision(False)
    line = chrono.ChLinePoly(len(points))
    for i, point in enumerate(points):
        line.SetPoint(i, point)
    shape = chrono.ChVisualShapeLine()
    shape.SetLineGeometry(line)
    shape.SetColor(color(0.62, 0.28, 0.86))
    shape.SetThickness(4)
    body.AddVisualShape(shape)
    system.AddBody(body)


def ode1_value_at(duration):
    q = [1.0, 0.0, 0.0]
    time = 0.0
    while time < duration - 1.0e-12:
        h = min(STEP, duration - time)

        def deriv(state):
            return [state[1], -100.0 * state[0] + 1.0, -10.0 * state[2] + 50.0]

        def add_scaled(base, slope, scale):
            return [base[i] + scale * slope[i] for i in range(3)]

        k1 = deriv(q)
        k2 = deriv(add_scaled(q, k1, 0.5 * h))
        k3 = deriv(add_scaled(q, k2, 0.5 * h))
        k4 = deriv(add_scaled(q, k3, h))
        q = [q[i] + h * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0 for i in range(3)]
        time += h
    return q


def build_system():
    summary = evaluate_sweep()
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_case_board(system, summary)
    add_sample_spring_panel(system, -1.85, -0.95)
    add_sample_constraint_panel(system, 0.10, -1.02)
    add_ode1_trace(system, 1.85, -0.72)
    add_box(system, "taskmanagerTest summary marker total cases", vec(0.25, 0.25, 0.10), vec(-2.05, 1.95, 0.06), color(0.95, 0.50, 0.08))
    add_box(system, "taskmanagerTest summary marker ODE1 cases", vec(0.25, 0.25, 0.10), vec(-1.70, 1.95, 0.06), color(0.62, 0.28, 0.86))
    add_box(system, "taskmanagerTest summary marker static cases", vec(0.25, 0.25, 0.10), vec(-1.35, 1.95, 0.06), color(0.55, 0.55, 0.58))
    system._taskmanager_summary = summary
    update_system_visuals(system)
    return system, summary


def update_visuals(system):
    update_system_visuals(system)


def simulate(duration, step):
    system, summary = build_system()
    while system.GetChTime() < duration - 1.0e-12:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, summary


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _summary = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: taskmanagerTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(vec(1.35, -5.6, 3.15), vec(1.25, 0.45, 0.0))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def print_result():
    summary = evaluate_sweep()
    first = summary["results"][0]
    last = summary["results"][-1]
    print(
        f"taskmanager_test: total_cases={len(summary['results'])}  "
        f"ode1_cases={summary['ode1']}  static_cases={summary['static']}  "
        f"explicit_cases={summary['explicit']}  constraint_cases={summary['constraints']}"
    )
    print(
        f"taskmanager_test: first_case={first['case']} first_contribution={first['contribution']:+.12f}  "
        f"last_case={last['case']} last_contribution={last['contribution']:+.12f}  "
        f"source_style_result={summary['total']:+.12f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0.12)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()
    print("EXUDYN port: taskmanagerTest.py -> PyChrono task-manager case sweep")
    if args.no_vis:
        print_result()
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

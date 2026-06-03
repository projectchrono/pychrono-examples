import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN TestModels/explicitLieGroupIntegratorPythonTest.py:
# a fixed-point heavy top integrated by the Python-side Lie-group RK4 Newton
# hook.  Chrono does not expose EXUDYN's rotation-vector Lie-group Newton
# callback, so this port evaluates the same rigid-body equations with RK4 and
# uses PyChrono bodies for an inspectable replay scene.

MASS = 15.0
GRAVITY = 9.81
LENGTH = 1.0
RADIUS = 0.5
COM_LOCAL = (0.0, 1.0, 0.0)
FORCE_WORLD = (0.0, 0.0, -MASS * GRAVITY)
INERTIA_FIXED = (15.234375, 0.46875, 15.234375)
INITIAL_OMEGA_BODY = (0.0, 150.0, -4.61538)
END_TIME = 0.01
STEP = 2.5e-5
REFERENCE_OMEGA_Y = 149.8473939540758
TRACE_POINTS = 80


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def vadd(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vsub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vscale(a, scale):
    return (scale * a[0], scale * a[1], scale * a[2])


def vdot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vcross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def vlength(a):
    return math.sqrt(vdot(a, a))


def vnormalize(a):
    length = vlength(a)
    if length < 1.0e-14:
        return (0.0, 0.0, 0.0)
    return vscale(a, 1.0 / length)


def mat_vec(matrix, vector):
    return tuple(sum(matrix[i][j] * vector[j] for j in range(3)) for i in range(3))


def mat_transpose_vec(matrix, vector):
    return tuple(sum(matrix[j][i] * vector[j] for j in range(3)) for i in range(3))


def quat_normalize(q):
    length = math.sqrt(sum(value * value for value in q))
    return tuple(value / length for value in q)


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_to_matrix(q):
    w, x, y, z = quat_normalize(q)
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def state_derivative(state):
    q = state[:4]
    omega = state[4:]
    rot = quat_to_matrix(q)
    force_body = mat_transpose_vec(rot, FORCE_WORLD)
    torque_body = vcross(COM_LOCAL, force_body)
    inertia_omega = tuple(INERTIA_FIXED[i] * omega[i] for i in range(3))
    gyro = vcross(omega, inertia_omega)
    omega_dot = tuple((torque_body[i] - gyro[i]) / INERTIA_FIXED[i] for i in range(3))
    q_dot = tuple(0.5 * value for value in quat_mul(q, (0.0,) + omega))
    return q_dot + omega_dot


def state_add_scaled(state, derivative, scale):
    return tuple(state[i] + scale * derivative[i] for i in range(7))


def rk4_step(state, step):
    k1 = state_derivative(state)
    k2 = state_derivative(state_add_scaled(state, k1, 0.5 * step))
    k3 = state_derivative(state_add_scaled(state, k2, 0.5 * step))
    k4 = state_derivative(state_add_scaled(state, k3, step))
    next_state = tuple(state[i] + step * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0 for i in range(7))
    return quat_normalize(next_state[:4]) + next_state[4:]


def initial_state():
    return (1.0, 0.0, 0.0, 0.0) + INITIAL_OMEGA_BODY


def integrate(duration, step):
    state = initial_state()
    time = 0.0
    while time < duration - 1.0e-14:
        h = min(step, duration - time)
        state = rk4_step(state, h)
        time += h
    return state


def compute_samples(duration=END_TIME, step=STEP):
    samples = [(0.0, initial_state())]
    state = initial_state()
    time = 0.0
    while time < duration - 1.0e-14:
        h = min(step, duration - time)
        state = rk4_step(state, h)
        time += h
        samples.append((time, state))
    return samples


def sample_state(samples, time):
    if time <= samples[0][0]:
        return samples[0][1]
    if time >= samples[-1][0]:
        return samples[-1][1]
    lo = 0
    hi = len(samples) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if samples[mid][0] <= time:
            lo = mid
        else:
            hi = mid
    t0, s0 = samples[lo]
    t1, s1 = samples[hi]
    alpha = (time - t0) / (t1 - t0)
    q = quat_normalize(tuple((1.0 - alpha) * s0[i] + alpha * s1[i] for i in range(4)))
    omega = tuple((1.0 - alpha) * s0[i + 4] + alpha * s1[i + 4] for i in range(3))
    return q + omega


def state_metrics(state):
    q = state[:4]
    omega_body = state[4:]
    rot = quat_to_matrix(q)
    omega_world = mat_vec(rot, omega_body)
    com_world = mat_vec(rot, COM_LOCAL)
    return {
        "rotation": q,
        "omega_body": omega_body,
        "omega_world": omega_world,
        "com_world": com_world,
    }


class MutableSegment:
    def __init__(self, system, name, tint, thickness=4):
        self.body = chrono.ChBody()
        self.body.SetName(name)
        self.body.SetFixed(True)
        self.body.EnableCollision(False)
        self.shape = chrono.ChVisualShapeSegment()
        self.shape.SetMutable(True)
        self.shape.SetColor(tint)
        self.shape.SetThickness(thickness)
        self.body.AddVisualShape(self.shape)
        system.AddBody(self.body)

    def update(self, point_a, point_b):
        self.shape.SetLineGeometry(chrono.ChLineSegment(point_a, point_b))
        self.body.UpdateVisualModel()


class MutablePolyline:
    def __init__(self, system, name, tint, thickness=3):
        self.body = chrono.ChBody()
        self.body.SetName(name)
        self.body.SetFixed(True)
        self.body.EnableCollision(False)
        self.line = chrono.ChLinePoly(TRACE_POINTS)
        self.shape = chrono.ChVisualShapeLine()
        self.shape.SetMutable(True)
        self.shape.SetLineGeometry(self.line)
        self.shape.SetColor(tint)
        self.shape.SetThickness(thickness)
        self.body.AddVisualShape(self.shape)
        system.AddBody(self.body)

    def update(self, points):
        if not points:
            points = [vec(0, 0, 0)]
        for i in range(TRACE_POINTS):
            source_index = min(len(points) - 1, int(i * len(points) / TRACE_POINTS))
            self.line.SetPoint(i, points[source_index])
        self.shape.SetLineGeometry(self.line)
        self.body.UpdateVisualModel()


def add_cylinder_between(body, point_a, point_b, radius, tint):
    segment = chrono.ChLineSegment(point_a, point_b)
    cylinder = chrono.ChVisualShapeCylinder(radius, segment.GetLength())
    cylinder.SetColor(tint)
    body.AddVisualShape(cylinder, segment.GetFrame())


def add_body_axes(body):
    add_cylinder_between(body, vec(0, 0, 0), vec(0.35, 0, 0), 0.008, color(0.90, 0.05, 0.04))
    add_cylinder_between(body, vec(0, 0, 0), vec(0, 0.35, 0), 0.008, color(0.05, 0.62, 0.14))
    add_cylinder_between(body, vec(0, 0, 0), vec(0, 0, 0.35), 0.008, color(0.06, 0.16, 0.90))


def make_ground(system):
    ground = chrono.ChBody()
    ground.SetName("explicitLieGroupIntegratorPythonTest fixed-point reference")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    plate = chrono.ChVisualShapeBox(1.85, 1.45, 0.018)
    plate.SetColor(color(0.78, 0.78, 0.74))
    plate.SetOpacity(0.24)
    ground.AddVisualShape(plate, chrono.ChFramed(vec(0.0, 0.45, -0.58)))
    add_cylinder_between(ground, vec(-0.75, 0, 0), vec(0.75, 0, 0), 0.006, color(0.72, 0.12, 0.10))
    add_cylinder_between(ground, vec(0, -0.10, 0), vec(0, 1.20, 0), 0.006, color(0.10, 0.55, 0.14))
    add_cylinder_between(ground, vec(0, 0, -0.65), vec(0, 0, 0.45), 0.006, color(0.12, 0.18, 0.75))
    system.AddBody(ground)
    return ground


def make_top_body(system):
    body = chrono.ChBody()
    body.SetName("explicitLieGroupIntegratorPythonTest RK4 heavy top replay")
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetMass(MASS)
    body.SetInertiaXX(vec(*INERTIA_FIXED))
    body.SetPos(vec(0, 0, 0))

    box = chrono.ChVisualShapeBox(RADIUS, LENGTH, RADIUS)
    box.SetColor(color(0.10, 0.16, 0.82))
    box.SetOpacity(0.42)
    body.AddVisualShape(box, chrono.ChFramed(vec(0, 0.5 * LENGTH, 0)))
    pivot = chrono.ChVisualShapeSphere(0.055)
    pivot.SetColor(color(0.04, 0.04, 0.04))
    body.AddVisualShape(pivot, chrono.ChFramed(vec(0, 0, 0)))
    com = chrono.ChVisualShapeSphere(0.060)
    com.SetColor(color(0.96, 0.78, 0.08))
    body.AddVisualShape(com, chrono.ChFramed(vec(*COM_LOCAL)))
    add_body_axes(body)
    system.AddBody(body)
    return body


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    make_ground(system)

    pivot = chrono.ChBodyEasySphere(0.070, 1000, True, False)
    pivot.SetName("explicitLieGroupIntegratorPythonTest fixed pivot")
    pivot.SetFixed(True)
    pivot.EnableCollision(False)
    pivot.SetPos(vec(0, 0, 0))
    pivot.GetVisualShape(0).SetColor(color(0.02, 0.02, 0.025))
    system.AddBody(pivot)

    top = make_top_body(system)
    gravity_arrow = MutableSegment(system, "gravity force at source COM marker", color(0.88, 0.10, 0.08), 5)
    gravity_tip_a = MutableSegment(system, "gravity force arrow tip a", color(0.88, 0.10, 0.08), 4)
    gravity_tip_b = MutableSegment(system, "gravity force arrow tip b", color(0.88, 0.10, 0.08), 4)
    omega_arrow = MutableSegment(system, "world angular velocity direction", color(0.02, 0.72, 0.86), 5)
    trace = MutablePolyline(system, "COM trajectory over RK4 samples", color(0.98, 0.56, 0.10), 3)

    samples = compute_samples()
    com_trace_points = [vec(*state_metrics(state)["com_world"]) for _time, state in samples]
    system._explicit_lie_group_python_items = {
        "top": top,
        "samples": samples,
        "com_trace_points": com_trace_points,
        "gravity": (gravity_arrow, gravity_tip_a, gravity_tip_b),
        "omega": omega_arrow,
        "trace": trace,
    }
    update_visuals(system)
    return system, system._explicit_lie_group_python_items


def update_visuals(system):
    items = system._explicit_lie_group_python_items
    visual_time = min(system.GetChTime(), END_TIME)
    state = sample_state(items["samples"], visual_time)
    metrics = state_metrics(state)
    top = items["top"]
    q = metrics["rotation"]
    top.SetRot(chrono.ChQuaterniond(q[0], q[1], q[2], q[3]))
    top.SetAngVelLocal(vec(*metrics["omega_body"]))
    top.UpdateVisualModel()

    com = metrics["com_world"]
    gravity_start = vadd(com, (0.18, 0.02, 0.0))
    gravity_end_tuple = vadd(gravity_start, (0.0, 0.0, -0.55))
    gravity_end = vec(*gravity_end_tuple)
    gravity_tip_1 = vec(gravity_end_tuple[0] + 0.055, gravity_end_tuple[1], gravity_end_tuple[2] + 0.10)
    gravity_tip_2 = vec(gravity_end_tuple[0] - 0.055, gravity_end_tuple[1], gravity_end_tuple[2] + 0.10)
    items["gravity"][0].update(vec(*gravity_start), gravity_end)
    items["gravity"][1].update(gravity_end, gravity_tip_1)
    items["gravity"][2].update(gravity_end, gravity_tip_2)

    omega_dir = vnormalize(metrics["omega_world"])
    items["omega"].update(vec(0, 0, 0), vec(*(vscale(omega_dir, 0.85))))

    sample_count = max(1, int((visual_time / END_TIME) * len(items["com_trace_points"])))
    items["trace"].update(items["com_trace_points"][:sample_count])


def simulate(duration, step):
    state = integrate(duration, step)
    system, items = build_system()
    final_metrics = state_metrics(state)
    q = final_metrics["rotation"]
    items["top"].SetRot(chrono.ChQuaterniond(q[0], q[1], q[2], q[3]))
    items["top"].SetAngVelLocal(vec(*final_metrics["omega_body"]))
    system.SetChTime(duration)
    update_visuals(system)
    return system, state


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: explicitLieGroupIntegratorPythonTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.45, 1.75, 1.25), chrono.ChVector3d(0.0, 0.45, 0.0))
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
            print_state(min(time, END_TIME), sample_state(system._explicit_lie_group_python_items["samples"], min(time, END_TIME)))
            next_log += 0.0025


def print_state(time, state):
    metrics = state_metrics(state)
    omega_world = metrics["omega_world"]
    omega_body = metrics["omega_body"]
    com = metrics["com_world"]
    error = omega_world[1] - REFERENCE_OMEGA_Y
    print(
        f"t={time:8.6f}  "
        f"omega_world=({omega_world[0]:+.9f},{omega_world[1]:+.12f},{omega_world[2]:+.9f})  "
        f"omega_body=({omega_body[0]:+.9f},{omega_body[1]:+.9f},{omega_body[2]:+.9f})"
    )
    print(
        f"com=({com[0]:+.9f},{com[1]:+.9f},{com[2]:+.9f})  "
        f"test_result={omega_world[1]:+.12f}  reference={REFERENCE_OMEGA_Y:+.12f}  "
        f"reference_error={error:+.3e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: explicitLieGroupIntegratorPythonTest.py -> PyChrono heavy-top Lie-group RK4 replay")
    if args.no_vis:
        _system, state = simulate(args.duration, args.step)
        print_state(args.duration, state)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN TestModels/explicitLieGroupIntegratorTest.py:
# the rotation-vector Lie-group explicit-solver comparison for a heavy rigid
# body loaded by gravity at an offset point.  PyChrono does not expose EXUDYN's
# internal Lie-group RK67/DOPRI solver, so this port evaluates the same free
# rigid-body ODEs with Python explicit integrators and renders the four method
# replays as visible Chrono bodies.

MASS = 15.0
GRAVITY = 9.81
LENGTH = 1.0
RADIUS = 0.5
COM_LOCAL = (0.0, 1.0, 0.0)
FORCE_WORLD = (0.0, 0.0, -MASS * GRAVITY)
INERTIA_FIXED = (15.234375, 0.46875, 15.234375)
INITIAL_OMEGA_BODY = (0.0, 150.0, -4.61538)
STEP = 1.0e-3
END_TIME = 2.0
SOURCE_REFERENCE = 0.16164013319819076
TRACE_POINTS = 90
DISPLAY_POS_SCALE = 0.035

METHODS = (
    ("ExplicitMidpoint", "midpoint", -2.1, (0.88, 0.24, 0.16)),
    ("RK44", "rk4", -0.7, (0.12, 0.32, 0.86)),
    ("RK67", "rk67", 0.7, (0.12, 0.62, 0.26)),
    ("DOPRI5", "dopri5", 2.1, (0.74, 0.36, 0.86)),
)


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def vadd(a, b):
    return tuple(a[i] + b[i] for i in range(3))


def vsub(a, b):
    return tuple(a[i] - b[i] for i in range(3))


def vscale(a, scale):
    return tuple(scale * a[i] for i in range(3))


def vdot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def vcross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def vnorm(a):
    return math.sqrt(vdot(a, a))


def vnormalize(a):
    length = vnorm(a)
    if length < 1.0e-14:
        return (0.0, 0.0, 0.0)
    return vscale(a, 1.0 / length)


def state_add(a, b):
    return tuple(a[i] + b[i] for i in range(len(a)))


def state_scale(a, scale):
    return tuple(scale * value for value in a)


def state_add_scaled(a, b, scale):
    return tuple(a[i] + scale * b[i] for i in range(len(a)))


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


def rotation_vector_from_quat(q):
    w, x, y, z = quat_normalize(q)
    if w < 0.0:
        w, x, y, z = -w, -x, -y, -z
    vector_norm = math.sqrt(x * x + y * y + z * z)
    if vector_norm < 1.0e-14:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(vector_norm, w)
    axis = (x / vector_norm, y / vector_norm, z / vector_norm)
    return vscale(axis, angle)


def initial_state():
    return (
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
    ) + INITIAL_OMEGA_BODY


def normalize_state(state):
    return state[:6] + quat_normalize(state[6:10]) + state[10:]


def derivative(state):
    pos = state[:3]
    vel = state[3:6]
    q = state[6:10]
    omega = state[10:]
    del pos
    rot = quat_to_matrix(q)
    force_body = mat_transpose_vec(rot, FORCE_WORLD)
    torque_body = vcross(COM_LOCAL, force_body)
    inertia_omega = tuple(INERTIA_FIXED[i] * omega[i] for i in range(3))
    gyro = vcross(omega, inertia_omega)
    omega_dot = tuple((torque_body[i] - gyro[i]) / INERTIA_FIXED[i] for i in range(3))
    q_dot = tuple(0.5 * value for value in quat_mul(q, (0.0,) + omega))
    acc = tuple(FORCE_WORLD[i] / MASS for i in range(3))
    return vel + acc + q_dot + omega_dot


def midpoint_step(state, step):
    k1 = derivative(state)
    k2 = derivative(state_add_scaled(state, k1, 0.5 * step))
    return normalize_state(state_add_scaled(state, k2, step))


def rk4_step(state, step):
    k1 = derivative(state)
    k2 = derivative(state_add_scaled(state, k1, 0.5 * step))
    k3 = derivative(state_add_scaled(state, k2, 0.5 * step))
    k4 = derivative(state_add_scaled(state, k3, step))
    delta = tuple(step * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0 for i in range(len(state)))
    return normalize_state(state_add(state, delta))


def dopri5_step(state, step):
    k1 = derivative(state)
    k2 = derivative(state_add_scaled(state, k1, step * (1.0 / 5.0)))
    k3 = derivative(
        state_add(
            state,
            state_scale(state_add(state_scale(k1, 3.0 / 40.0), state_scale(k2, 9.0 / 40.0)), step),
        )
    )
    k4 = derivative(
        state_add(
            state,
            state_scale(
                state_add(state_add(state_scale(k1, 44.0 / 45.0), state_scale(k2, -56.0 / 15.0)), state_scale(k3, 32.0 / 9.0)),
                step,
            ),
        )
    )
    k5 = derivative(
        state_add(
            state,
            state_scale(
                state_add(
                    state_add(state_add(state_scale(k1, 19372.0 / 6561.0), state_scale(k2, -25360.0 / 2187.0)), state_scale(k3, 64448.0 / 6561.0)),
                    state_scale(k4, -212.0 / 729.0),
                ),
                step,
            ),
        )
    )
    k6 = derivative(
        state_add(
            state,
            state_scale(
                state_add(
                    state_add(
                        state_add(state_add(state_scale(k1, 9017.0 / 3168.0), state_scale(k2, -355.0 / 33.0)), state_scale(k3, 46732.0 / 5247.0)),
                        state_scale(k4, 49.0 / 176.0),
                    ),
                    state_scale(k5, -5103.0 / 18656.0),
                ),
                step,
            ),
        )
    )
    delta = state_scale(
        state_add(
            state_add(state_add(state_scale(k1, 35.0 / 384.0), state_scale(k3, 500.0 / 1113.0)), state_scale(k4, 125.0 / 192.0)),
            state_add(state_scale(k5, -2187.0 / 6784.0), state_scale(k6, 11.0 / 84.0)),
        ),
        step,
    )
    return normalize_state(state_add(state, delta))


def integrate_method(kind, duration=END_TIME, step=STEP, store_samples=False):
    state = initial_state()
    time = 0.0
    samples = [(0.0, state)]
    nsteps = 0
    full_steps = int(math.floor(duration / step + 1.0e-12))
    for _ in range(full_steps):
        h = step
        if kind == "midpoint":
            state = midpoint_step(state, h)
        elif kind == "rk4":
            state = rk4_step(state, h)
        elif kind == "rk67":
            state = rk4_step(rk4_step(state, 0.5 * h), 0.5 * h)
        elif kind == "dopri5":
            state = dopri5_step(state, h)
        else:
            raise ValueError(kind)
        time += h
        nsteps += 1
        if store_samples and (nsteps % 20 == 0 or time >= duration - 1.0e-14):
            samples.append((time, state))
    remainder = duration - full_steps * step
    if remainder > 1.0e-12:
        h = remainder
        if kind == "midpoint":
            state = midpoint_step(state, h)
        elif kind == "rk4":
            state = rk4_step(state, h)
        elif kind == "rk67":
            state = rk4_step(rk4_step(state, 0.5 * h), 0.5 * h)
        elif kind == "dopri5":
            state = dopri5_step(state, h)
        time += h
        nsteps += 1
        if store_samples:
            samples.append((time, state))
    return state, nsteps, samples


def state_metrics(state):
    pos = state[:3]
    q = state[6:10]
    omega_body = state[10:]
    rot = quat_to_matrix(q)
    omega_world = mat_vec(rot, omega_body)
    local_pos = vadd(pos, mat_vec(rot, COM_LOCAL))
    coords = pos + rotation_vector_from_quat(q)
    return {
        "pos": pos,
        "rotation": q,
        "omega_body": omega_body,
        "omega_world": omega_world,
        "local_pos": local_pos,
        "coords": coords,
        "coords_norm": vnorm(coords),
        "local_pos_norm": vnorm(local_pos),
    }


def evaluate_methods(store_samples=False):
    entries = []
    for label, kind, offset, tint in METHODS:
        state, nsteps, samples = integrate_method(kind, store_samples=store_samples)
        metrics = state_metrics(state)
        entries.append(
            {
                "label": label,
                "kind": kind,
                "offset": offset,
                "color": tint,
                "state": state,
                "metrics": metrics,
                "nsteps": nsteps,
                "samples": samples,
            }
        )
    total = 0.0
    for entry in entries:
        total += entry["metrics"]["coords_norm"] + entry["metrics"]["local_pos_norm"]
        if entry["kind"] == "dopri5":
            total += entry["nsteps"] / 8517.0
    return entries, 1.0e-3 * total


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


def display_position(entry, world):
    return vec(entry["offset"] + DISPLAY_POS_SCALE * world[0], DISPLAY_POS_SCALE * world[1], DISPLAY_POS_SCALE * world[2])


def sample_for_time(samples, time):
    if time <= samples[0][0]:
        return samples[0][1]
    if time >= samples[-1][0]:
        return samples[-1][1]
    for i in range(1, len(samples)):
        if samples[i][0] >= time:
            return samples[i][1]
    return samples[-1][1]


def add_cylinder_between(body, point_a, point_b, radius, tint):
    segment = chrono.ChLineSegment(point_a, point_b)
    cylinder = chrono.ChVisualShapeCylinder(radius, segment.GetLength())
    cylinder.SetColor(tint)
    body.AddVisualShape(cylinder, segment.GetFrame())


def make_replay_body(system, entry):
    tint = entry["color"]
    body = chrono.ChBody()
    body.SetName(f"explicitLieGroupIntegratorTest {entry['label']} body replay")
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetMass(MASS)
    body.SetInertiaXX(vec(*INERTIA_FIXED))

    box = chrono.ChVisualShapeBox(RADIUS, LENGTH, RADIUS)
    box.SetColor(color(*tint))
    box.SetOpacity(0.52)
    body.AddVisualShape(box, chrono.ChFramed(vec(0, 0.5 * LENGTH, 0)))
    com = chrono.ChVisualShapeSphere(0.060)
    com.SetColor(color(0.96, 0.76, 0.08))
    body.AddVisualShape(com, chrono.ChFramed(vec(*COM_LOCAL)))
    pivot = chrono.ChVisualShapeSphere(0.035)
    pivot.SetColor(color(0.03, 0.03, 0.035))
    body.AddVisualShape(pivot, chrono.ChFramed(vec(0, 0, 0)))
    add_cylinder_between(body, vec(0, 0, 0), vec(0.33, 0, 0), 0.006, color(0.92, 0.05, 0.04))
    add_cylinder_between(body, vec(0, 0, 0), vec(0, 0.33, 0), 0.006, color(0.05, 0.64, 0.14))
    add_cylinder_between(body, vec(0, 0, 0), vec(0, 0, 0.33), 0.006, color(0.06, 0.16, 0.88))
    system.AddBody(body)
    return body


def make_ground(system):
    ground = chrono.ChBody()
    ground.SetName("explicitLieGroupIntegratorTest solver comparison ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    plate = chrono.ChVisualShapeBox(5.3, 1.55, 0.020)
    plate.SetColor(color(0.78, 0.78, 0.74))
    plate.SetOpacity(0.25)
    ground.AddVisualShape(plate, chrono.ChFramed(vec(0, 0.25, -0.95)))
    for offset in (-2.1, -0.7, 0.7, 2.1):
        add_cylinder_between(ground, vec(offset, -0.35, -0.92), vec(offset, 1.0, -0.92), 0.006, color(0.28, 0.28, 0.30))
    system.AddBody(ground)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    make_ground(system)

    entries, aggregate = evaluate_methods(store_samples=True)
    visuals = []
    for entry in entries:
        body = make_replay_body(system, entry)
        gravity = MutableSegment(system, f"{entry['label']} gravity load arrow", color(0.88, 0.10, 0.08), 5)
        gravity_tip_a = MutableSegment(system, f"{entry['label']} gravity load arrow tip a", color(0.88, 0.10, 0.08), 4)
        gravity_tip_b = MutableSegment(system, f"{entry['label']} gravity load arrow tip b", color(0.88, 0.10, 0.08), 4)
        omega = MutableSegment(system, f"{entry['label']} angular velocity arrow", color(0.02, 0.74, 0.88), 5)
        trace = MutablePolyline(system, f"{entry['label']} COM path trace", color(*entry["color"]), 3)
        visuals.append({"entry": entry, "body": body, "gravity": (gravity, gravity_tip_a, gravity_tip_b), "omega": omega, "trace": trace})
    system._explicit_lie_group_integrator_test_items = {"visuals": visuals, "aggregate": aggregate}
    update_visuals(system)
    return system, system._explicit_lie_group_integrator_test_items


def update_visuals(system):
    items = system._explicit_lie_group_integrator_test_items
    source_time = min(END_TIME, system.GetChTime() * 2.5)
    for visual in items["visuals"]:
        entry = visual["entry"]
        state = sample_for_time(entry["samples"], source_time)
        metrics = state_metrics(state)
        q = metrics["rotation"]
        body = visual["body"]
        body.SetPos(display_position(entry, metrics["pos"]))
        body.SetRot(chrono.ChQuaterniond(q[0], q[1], q[2], q[3]))
        body.UpdateVisualModel()

        com_display = display_position(entry, metrics["local_pos"])
        gravity_start = vec(com_display.x + 0.18, com_display.y + 0.04, com_display.z)
        gravity_end = vec(gravity_start.x, gravity_start.y, gravity_start.z - 0.42)
        visual["gravity"][0].update(gravity_start, gravity_end)
        visual["gravity"][1].update(gravity_end, vec(gravity_end.x + 0.045, gravity_end.y, gravity_end.z + 0.085))
        visual["gravity"][2].update(gravity_end, vec(gravity_end.x - 0.045, gravity_end.y, gravity_end.z + 0.085))

        omega_dir = vnormalize(metrics["omega_world"])
        omega_end = vec(com_display.x + 0.42 * omega_dir[0], com_display.y + 0.42 * omega_dir[1], com_display.z + 0.42 * omega_dir[2])
        visual["omega"].update(com_display, omega_end)

        path_points = []
        for time, sample in entry["samples"]:
            if time > source_time + 1.0e-14:
                break
            path_points.append(display_position(entry, state_metrics(sample)["local_pos"]))
        visual["trace"].update(path_points)


def simulate(duration, step):
    del duration, step
    entries, aggregate = evaluate_methods(store_samples=False)
    system, _items = build_system()
    return system, entries, aggregate


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1120, 760)
    vis.SetWindowTitle("EXUDYN port: explicitLieGroupIntegratorTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.0, -6.0, 2.2), chrono.ChVector3d(0.0, 0.18, -0.35))
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
            print_state()
            next_log += 0.25


def print_state(entries=None, aggregate=None):
    if entries is None or aggregate is None:
        entries, aggregate = evaluate_methods(store_samples=False)
    print("EXUDYN port: explicitLieGroupIntegratorTest.py -> PyChrono explicit Lie-group solver comparison")
    for entry in entries:
        metrics = entry["metrics"]
        omega = metrics["omega_world"]
        print(
            f"{entry['label']}: omega=({omega[0]:+.6f},{omega[1]:+.6f},{omega[2]:+.6f})  "
            f"coords_norm={metrics['coords_norm']:.9f}  pos_norm={metrics['local_pos_norm']:.9f}  nsteps={entry['nsteps']}"
        )
    print(
        f"explicit_lie_group_integrator_test: source_style_result={aggregate:.12f}  "
        f"source_reference={SOURCE_REFERENCE:.12f}  reference_delta={aggregate - SOURCE_REFERENCE:+.3e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0.8)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    if args.no_vis:
        _system, entries, aggregate = simulate(args.duration, args.step)
        print_state(entries, aggregate)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

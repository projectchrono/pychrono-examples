import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import SpringVisual


# Reproduces EXUDYN TestModels/explicitLieGroupMBSTest.py:
# two rotation-vector rigid bodies, gravity loads at offset markers, a first
# body translationally constrained at the origin, and a CartesianSpringDamper
# between body markers. Chrono's API differs from EXUDYN's Lie-group Newton
# hook, so this port integrates the same marker-force equations with RK4 and
# renders a PyChrono replay, including a visible coil for the Cartesian spring.

MASS = 15.0
GRAVITY = 9.81
RADIUS = 0.2
LENGTH = 1.0
INERTIA_FIXED = (15.234375, 0.46875, 15.234375)
FORCE_WORLD = (0.0, 0.0, -MASS * GRAVITY)
SPRING_K = 1.0e4
SPRING_D = 0.0
STEP = 1.0e-4
END_TIME = 1.0
VISUAL_SPRING_SCALE = 12.0
SOURCE_REFERENCE_OMEGA_Y = 3.493435692625912

BODY1_FORCE_LOCAL = (0.5 * RADIUS, 0.5 * LENGTH, 0.0)
BODY2_FORCE_LOCAL = (0.0, LENGTH, 0.0)
SPRING1_LOCAL = (0.5 * RADIUS, 0.5 * LENGTH, 0.0)
SPRING2_LOCAL = (0.5 * RADIUS, -0.5 * LENGTH, 0.0)


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
        return (0.0, 1.0, 0.0)
    return vscale(a, 1.0 / length)


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


def quat_rotate(q, vector):
    qn = quat_normalize(q)
    qc = (qn[0], -qn[1], -qn[2], -qn[3])
    return quat_mul(quat_mul(qn, (0.0,) + vector), qc)[1:]


def quat_rotate_transpose(q, vector):
    qn = quat_normalize(q)
    qc = (qn[0], -qn[1], -qn[2], -qn[3])
    return quat_mul(quat_mul(qc, (0.0,) + vector), qn)[1:]


def marker_velocity(origin_velocity, omega_body, q, local_point):
    world_point = quat_rotate(q, local_point)
    omega_world = quat_rotate(q, omega_body)
    return vadd(origin_velocity, vcross(omega_world, world_point))


def omega_dot(q, omega_body, torque_world):
    torque_body = quat_rotate_transpose(q, torque_world)
    inertia_omega = tuple(INERTIA_FIXED[i] * omega_body[i] for i in range(3))
    gyro = vcross(omega_body, inertia_omega)
    return tuple((torque_body[i] - gyro[i]) / INERTIA_FIXED[i] for i in range(3))


def initial_state():
    q = (1.0, 0.0, 0.0, 0.0)
    return q + (0.0, 0.0, 0.0) + (0.0, LENGTH, 0.0) + (0.0, 0.0, 0.0) + q + (0.0, 0.0, 0.0)


def state_parts(state):
    return {
        "q1": state[0:4],
        "w1": state[4:7],
        "p2": state[7:10],
        "v2": state[10:13],
        "q2": state[13:17],
        "w2": state[17:20],
    }


def spring_marker_data(state):
    parts = state_parts(state)
    p1 = quat_rotate(parts["q1"], SPRING1_LOCAL)
    p2 = vadd(parts["p2"], quat_rotate(parts["q2"], SPRING2_LOCAL))
    v1 = marker_velocity((0.0, 0.0, 0.0), parts["w1"], parts["q1"], SPRING1_LOCAL)
    v2 = marker_velocity(parts["v2"], parts["w2"], parts["q2"], SPRING2_LOCAL)
    rel = vsub(p2, p1)
    rel_v = vsub(v2, v1)
    force_on_1 = vadd(vscale(rel, SPRING_K), vscale(rel_v, SPRING_D))
    return p1, p2, rel, force_on_1


def derivative(state):
    parts = state_parts(state)
    q1 = parts["q1"]
    w1 = parts["w1"]
    p2 = parts["p2"]
    v2 = parts["v2"]
    q2 = parts["q2"]
    w2 = parts["w2"]
    spring1, spring2, _rel, spring_force_on_1 = spring_marker_data(state)

    force1_local = quat_rotate(q1, BODY1_FORCE_LOCAL)
    force2_local = quat_rotate(q2, BODY2_FORCE_LOCAL)
    spring2_local_world = quat_rotate(q2, SPRING2_LOCAL)

    torque1 = vadd(vcross(force1_local, FORCE_WORLD), vcross(spring1, spring_force_on_1))
    torque2 = vadd(vcross(force2_local, FORCE_WORLD), vcross(spring2_local_world, vscale(spring_force_on_1, -1.0)))

    q1_dot = tuple(0.5 * value for value in quat_mul(q1, (0.0,) + w1))
    q2_dot = tuple(0.5 * value for value in quat_mul(q2, (0.0,) + w2))
    w1_dot = omega_dot(q1, w1, torque1)
    w2_dot = omega_dot(q2, w2, torque2)
    force2 = vadd(FORCE_WORLD, vscale(spring_force_on_1, -1.0))
    a2 = vscale(force2, 1.0 / MASS)
    return q1_dot + w1_dot + v2 + a2 + q2_dot + w2_dot


def state_add_scaled(state, slope, scale):
    return tuple(state[i] + scale * slope[i] for i in range(len(state)))


def normalize_state(state):
    return quat_normalize(state[0:4]) + state[4:13] + quat_normalize(state[13:17]) + state[17:20]


def rk4_step(state, step):
    k1 = derivative(state)
    k2 = derivative(state_add_scaled(state, k1, 0.5 * step))
    k3 = derivative(state_add_scaled(state, k2, 0.5 * step))
    k4 = derivative(state_add_scaled(state, k3, step))
    next_state = tuple(state[i] + step * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0 for i in range(len(state)))
    return normalize_state(next_state)


def compute_samples(duration=END_TIME, step=STEP):
    state = initial_state()
    time = 0.0
    samples = [(0.0, state)]
    store_every = max(1, int(0.002 / step))
    counter = 0
    while time < duration - 1.0e-14:
        h = min(step, duration - time)
        state = rk4_step(state, h)
        time += h
        counter += 1
        if counter % store_every == 0 or time >= duration - 1.0e-14:
            samples.append((time, state))
    return samples


SAMPLES = compute_samples()


def sample_state(time):
    if time <= SAMPLES[0][0]:
        return SAMPLES[0][1]
    if time >= SAMPLES[-1][0]:
        return SAMPLES[-1][1]
    for index in range(1, len(SAMPLES)):
        if SAMPLES[index][0] >= time:
            t0, s0 = SAMPLES[index - 1]
            t1, s1 = SAMPLES[index]
            alpha = (time - t0) / (t1 - t0)
            q1 = quat_normalize(tuple((1.0 - alpha) * s0[i] + alpha * s1[i] for i in range(4)))
            middle = tuple((1.0 - alpha) * s0[i] + alpha * s1[i] for i in range(4, 13))
            q2 = quat_normalize(tuple((1.0 - alpha) * s0[i] + alpha * s1[i] for i in range(13, 17)))
            tail = tuple((1.0 - alpha) * s0[i] + alpha * s1[i] for i in range(17, 20))
            return q1 + middle + q2 + tail
    return SAMPLES[-1][1]


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

    def update(self, a, b):
        self.shape.SetLineGeometry(chrono.ChLineSegment(a, b))
        self.body.UpdateVisualModel()


def add_cylinder_between(body, point_a, point_b, radius, tint):
    segment = chrono.ChLineSegment(point_a, point_b)
    cylinder = chrono.ChVisualShapeCylinder(radius, segment.GetLength())
    cylinder.SetColor(tint)
    body.AddVisualShape(cylinder, segment.GetFrame())


def add_body_axes(body):
    add_cylinder_between(body, vec(0, 0, 0), vec(0.34, 0, 0), 0.006, color(0.92, 0.05, 0.04))
    add_cylinder_between(body, vec(0, 0, 0), vec(0, 0.34, 0), 0.006, color(0.05, 0.64, 0.14))
    add_cylinder_between(body, vec(0, 0, 0), vec(0, 0, 0.34), 0.006, color(0.06, 0.16, 0.88))


def make_body(system, name, tint):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetMass(MASS)
    body.SetInertiaXX(vec(*INERTIA_FIXED))
    box = chrono.ChVisualShapeBox(RADIUS, LENGTH, RADIUS)
    box.SetColor(tint)
    box.SetOpacity(0.52)
    body.AddVisualShape(box, chrono.ChFramed(vec(0, 0, 0)))
    force_marker = chrono.ChVisualShapeSphere(0.050)
    force_marker.SetColor(color(0.96, 0.76, 0.08))
    body.AddVisualShape(force_marker, chrono.ChFramed(vec(*BODY1_FORCE_LOCAL if "body 1" in name else BODY2_FORCE_LOCAL)))
    spring_marker = chrono.ChVisualShapeSphere(0.038)
    spring_marker.SetColor(color(0.04, 0.04, 0.045))
    body.AddVisualShape(spring_marker, chrono.ChFramed(vec(*SPRING1_LOCAL if "body 1" in name else SPRING2_LOCAL)))
    add_body_axes(body)
    system.AddBody(body)
    return body


def make_ground(system):
    ground = chrono.ChBody()
    ground.SetName("explicitLieGroupMBSTest ground and translational constraint axes")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    plate = chrono.ChVisualShapeBox(1.75, 2.25, 0.020)
    plate.SetColor(color(0.78, 0.78, 0.74))
    plate.SetOpacity(0.25)
    ground.AddVisualShape(plate, chrono.ChFramed(vec(0.0, 0.20, -1.10)))
    add_cylinder_between(ground, vec(-0.65, 0, 0), vec(0.65, 0, 0), 0.005, color(0.72, 0.12, 0.10))
    add_cylinder_between(ground, vec(0, -0.25, 0), vec(0, 1.30, 0), 0.005, color(0.10, 0.55, 0.14))
    add_cylinder_between(ground, vec(0, 0, -0.75), vec(0, 0, 0.45), 0.005, color(0.12, 0.18, 0.75))
    system.AddBody(ground)
    return ground


def set_body_pose(body, position, quaternion):
    body.SetPos(vec(*position))
    body.SetRot(chrono.ChQuaterniond(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
    body.UpdateVisualModel()


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    make_ground(system)
    body1 = make_body(system, "explicitLieGroupMBSTest body 1 constrained translation", color(0.12, 0.24, 0.86))
    body2 = make_body(system, "explicitLieGroupMBSTest body 2 spring-coupled", color(0.82, 0.12, 0.10))
    gravity1 = MutableSegment(system, "explicitLieGroupMBSTest body 1 gravity arrow", color(0.86, 0.10, 0.08), 5)
    gravity2 = MutableSegment(system, "explicitLieGroupMBSTest body 2 gravity arrow", color(0.86, 0.10, 0.08), 5)
    omega1 = MutableSegment(system, "explicitLieGroupMBSTest body 1 angular velocity arrow", color(0.02, 0.74, 0.88), 5)
    omega2 = MutableSegment(system, "explicitLieGroupMBSTest body 2 angular velocity arrow", color(0.02, 0.74, 0.88), 5)
    spring_visual = SpringVisual(system, 0.035, 96, 10, color(0.86, 0.16, 0.10))
    spring_body1 = chrono.ChBodyEasySphere(0.020, 1000, True, False)
    spring_body1.SetName("explicitLieGroupMBSTest native coil endpoint 1")
    spring_body1.SetFixed(True)
    spring_body1.EnableCollision(False)
    spring_body1.GetVisualShape(0).SetColor(color(0.05, 0.05, 0.055))
    system.AddBody(spring_body1)
    spring_body2 = chrono.ChBodyEasySphere(0.020, 1000, True, False)
    spring_body2.SetName("explicitLieGroupMBSTest native coil endpoint 2")
    spring_body2.SetFixed(True)
    spring_body2.EnableCollision(False)
    spring_body2.GetVisualShape(0).SetColor(color(0.96, 0.76, 0.08))
    system.AddBody(spring_body2)
    native_spring = chrono.ChLinkTSDA()
    native_spring.SetName("explicitLieGroupMBSTest native ChVisualShapeSpring")
    native_spring.Initialize(spring_body1, spring_body2, True, vec(0, 0, 0), vec(0, 0, 0))
    native_spring.SetRestLength(0.0)
    native_spring.SetSpringCoefficient(0.0)
    native_spring.SetDampingCoefficient(0.0)
    system.AddLink(native_spring)
    native_shape = chrono.ChVisualShapeSpring(0.035, 96, 10)
    native_shape.SetColor(color(0.86, 0.16, 0.10))
    native_spring.AddVisualShape(native_shape)
    connector_axis = MutableSegment(system, "explicitLieGroupMBSTest actual Cartesian spring endpoints", color(0.15, 0.15, 0.16), 2)
    system._explicit_lie_group_mbs_items = {
        "body1": body1,
        "body2": body2,
        "gravity1": gravity1,
        "gravity2": gravity2,
        "omega1": omega1,
        "omega2": omega2,
        "spring_visual": spring_visual,
        "spring_body1": spring_body1,
        "spring_body2": spring_body2,
        "connector_axis": connector_axis,
    }
    update_visuals(system)
    return system, system._explicit_lie_group_mbs_items


def update_arrow(segment, start, direction, scale):
    segment.update(vec(*start), vec(*(vadd(start, vscale(vnormalize(direction), scale)))))


def update_visuals(system):
    state = sample_state(min(system.GetChTime(), END_TIME))
    parts = state_parts(state)
    items = system._explicit_lie_group_mbs_items
    set_body_pose(items["body1"], (0.0, 0.0, 0.0), parts["q1"])
    set_body_pose(items["body2"], parts["p2"], parts["q2"])

    body1_force_point = quat_rotate(parts["q1"], BODY1_FORCE_LOCAL)
    body2_force_point = vadd(parts["p2"], quat_rotate(parts["q2"], BODY2_FORCE_LOCAL))
    update_arrow(items["gravity1"], body1_force_point, FORCE_WORLD, 0.42)
    update_arrow(items["gravity2"], body2_force_point, FORCE_WORLD, 0.42)
    update_arrow(items["omega1"], (0.0, 0.0, 0.0), quat_rotate(parts["q1"], parts["w1"]), 0.55)
    update_arrow(items["omega2"], parts["p2"], quat_rotate(parts["q2"], parts["w2"]), 0.55)

    spring1, spring2, rel, _force = spring_marker_data(state)
    items["connector_axis"].update(vec(*spring1), vec(*spring2))
    display_start = vadd(spring1, (0.0, 0.0, 0.35))
    display_end = vadd(display_start, vscale(rel if vnorm(rel) > 1.0e-8 else (0.0, 1.0, 0.0), VISUAL_SPRING_SCALE))
    items["spring_body1"].SetPos(vec(*display_start))
    items["spring_body2"].SetPos(vec(*display_end))
    items["spring_body1"].UpdateVisualModel()
    items["spring_body2"].UpdateVisualModel()
    items["spring_visual"].update(vec(*display_start), vec(*display_end))


def simulate(duration, step):
    del step
    system, items = build_system()
    system.SetChTime(min(duration, END_TIME))
    update_visuals(system)
    return system, items


def print_state(time):
    state = sample_state(min(time, END_TIME))
    parts = state_parts(state)
    spring1, spring2, rel, force = spring_marker_data(state)
    print(
        f"t={min(time, END_TIME):6.3f}  "
        f"omega1=({parts['w1'][0]:+.9f},{parts['w1'][1]:+.9f},{parts['w1'][2]:+.9f})  "
        f"omega2=({parts['w2'][0]:+.9f},{parts['w2'][1]:+.9f},{parts['w2'][2]:+.9f})"
    )
    print(
        f"body2_pos=({parts['p2'][0]:+.9f},{parts['p2'][1]:+.9f},{parts['p2'][2]:+.9f})  "
        f"spring_length={vnorm(rel):.9f}  spring_force_norm={vnorm(force):.6f}  "
        f"source_comment_omegay={SOURCE_REFERENCE_OMEGA_Y:.12f}"
    )
    print(
        f"spring_endpoints=({spring1[0]:+.5f},{spring1[1]:+.5f},{spring1[2]:+.5f})"
        f"->({spring2[0]:+.5f},{spring2[1]:+.5f},{spring2[2]:+.5f})"
    )


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1100, 760)
    vis.SetWindowTitle("EXUDYN port: explicitLieGroupMBSTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.55, -3.40, 1.80), chrono.ChVector3d(0.0, 0.02, -0.35))
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
            print_state(time)
            next_log += 0.25


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()
    print("EXUDYN port: explicitLieGroupMBSTest.py -> PyChrono two-body Lie-group MBS replay")
    if args.no_vis:
        simulate(args.duration, args.step)
        print_state(args.duration)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

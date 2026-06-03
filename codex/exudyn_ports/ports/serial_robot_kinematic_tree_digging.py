import argparse
import math
import random

import pychrono.core as chrono


# Reproduces EXUDYN Examples/serialRobotKinematicTreeDigging.py:
# a PUMA 560 ObjectKinematicTree with a cup attached at the TCP, two particle
# bins, and a cyclic digging trajectory.  The original runs a large EXUDYN
# general-contact particle simulation.  This PyChrono port keeps the source
# PUMA DH model, source joint trajectory, cup dimensions/offset, bin geometry,
# and a bounded visible particle bed with a carried-particle cluster attached
# to the cup for visual inspection.

STEP = 1.0e-3
END_TIME = 3.45
DEFAULT_BED_PARTICLES = 720

CUP_T = 0.005
CUP_R = 0.07
CUP_RI = CUP_R - CUP_T
CUP_H = 0.15
Z_OFF_TOOL = 0.2
X_OFF_TOOL = 0.075

LL = 1.0
FLOOR_T = 0.02 * LL
BASE_A = 0.2 * LL
BASE_B = 0.35 * LL
WALL_H = 2.4 * BASE_A

JOINT_RADIUS = 0.045
JOINT_AXIS_LENGTH = 0.14

LINK_COLORS = (
    (0.78, 0.22, 0.18),
    (0.10, 0.46, 0.84),
    (0.14, 0.62, 0.28),
    (0.86, 0.56, 0.12),
    (0.48, 0.34, 0.78),
    (0.10, 0.60, 0.62),
)

PUMA_LINKS = (
    {"stdDH": [0.0, 0.0, 0.0, 0.5 * math.pi], "mass": 20.0, "COM": [0.0, 0.0, 0.0]},
    {"stdDH": [0.0, 0.0, 0.4318, 0.0], "mass": 17.4, "COM": [-0.3638, 0.006, 0.2275]},
    {"stdDH": [0.0, 0.15005, 0.0203, -0.5 * math.pi], "mass": 4.8, "COM": [-0.0203, -0.0141, 0.07]},
    {"stdDH": [0.0, 0.4318, 0.0, 0.5 * math.pi], "mass": 0.82, "COM": [0.0, 0.019, 0.0]},
    {"stdDH": [0.0, 0.0, 0.0, -0.5 * math.pi], "mass": 0.34, "COM": [0.0, 0.0, 0.0]},
    {"stdDH": [0.0, 0.0, 0.0, 0.0], "mass": 0.09, "COM": [0.0, 0.0, 0.032]},
)

Q0 = [0.0, 0.5 * math.pi, -1.0 * math.pi, 0.0, 0.0, 0.0]
Q1 = [-0.07 * math.pi, 0.20 * math.pi, -0.8 * math.pi, 0.0, 0.0, -0.9 * math.pi]
Q2 = [-0.07 * math.pi, 0.16 * math.pi, -0.9 * math.pi, 0.0, 0.0, -0.6 * math.pi]
Q3 = [0.10 * math.pi, 0.16 * math.pi, -0.9 * math.pi, 0.0, 0.0, -0.4 * math.pi]
Q4 = [0.10 * math.pi, 0.40 * math.pi, -1.0 * math.pi, 0.0, 0.15 * math.pi, -0.15 * math.pi]
Q5 = [0.65 * math.pi, 0.40 * math.pi, -1.0 * math.pi, 0.0, 0.15 * math.pi, 0.15 * math.pi]
Q6 = [0.65 * math.pi, 0.30 * math.pi, -0.9 * math.pi, 0.0, 0.0, -1.0 * math.pi]
Q7 = [0.65 * math.pi, 0.40 * math.pi, -0.9 * math.pi, 0.0, 0.0, -1.0 * math.pi]

TRAJECTORY_SEGMENTS = (
    (Q0, Q1, 0.25),
    (Q1, Q2, 0.50),
    (Q2, Q3, 0.50),
    (Q3, Q4, 0.75),
    (Q4, Q5, 0.75),
    (Q5, Q6, 0.30),
    (Q6, Q7, 0.15),
    (Q7, Q0, 0.25),
)
CYCLE_TIME = sum(segment[2] for segment in TRAJECTORY_SEGMENTS)


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vector(values):
    return chrono.ChVector3d(values[0], values[1], values[2])


def vadd(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def vscale(a, scale):
    return [scale * a[0], scale * a[1], scale * a[2]]


def identity3():
    return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]


def matmul3(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def ht_identity():
    return [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def ht_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def ht_rot_x(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return [[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]]


def ht_rot_z(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def ht_trans_x(value):
    return [[1, 0, 0, value], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def ht_trans_z(value):
    return [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, value], [0, 0, 0, 1]]


def ht_translate(values):
    return [[1, 0, 0, values[0]], [0, 1, 0, values[1]], [0, 0, 1, values[2]], [0, 0, 0, 1]]


def std_dh(theta, d, a, alpha):
    return ht_mul(ht_mul(ht_mul(ht_rot_z(theta), ht_trans_z(d)), ht_trans_x(a)), ht_rot_x(alpha))


def ht_rotation(ht):
    return [[ht[i][j] for j in range(3)] for i in range(3)]


def ht_translation(ht):
    return [ht[0][3], ht[1][3], ht[2][3]]


def transform_point(ht, point):
    return [
        ht[0][0] * point[0] + ht[0][1] * point[1] + ht[0][2] * point[2] + ht[0][3],
        ht[1][0] * point[0] + ht[1][1] * point[1] + ht[1][2] * point[2] + ht[1][3],
        ht[2][0] * point[0] + ht[2][1] * point[1] + ht[2][2] * point[2] + ht[2][3],
    ]


def quat_from_matrix(m):
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    q = chrono.ChQuaterniond(w, x, y, z)
    q.Normalize()
    return q


def profile_constant_acceleration(tau, duration):
    if tau <= 0.0:
        return 0.0, 0.0
    if tau >= duration:
        return 1.0, 0.0
    half = 0.5 * duration
    if tau <= half:
        s = 2.0 * (tau / duration) ** 2
        sd = 4.0 * tau / (duration * duration)
    else:
        remaining = duration - tau
        s = 1.0 - 2.0 * (remaining / duration) ** 2
        sd = 4.0 * remaining / (duration * duration)
    return s, sd


def trajectory(t):
    tau = t % CYCLE_TIME
    elapsed = 0.0
    for start, end, duration in TRAJECTORY_SEGMENTS:
        if tau <= elapsed + duration:
            s, sd = profile_constant_acceleration(tau - elapsed, duration)
            q = []
            qd = []
            for a, b in zip(start, end):
                delta = b - a
                q.append(a + delta * s)
                qd.append(delta * sd)
            return q, qd
        elapsed += duration
    return list(Q0), [0.0] * 6


def robot_poses(q):
    frames = []
    joint_frames = []
    ht = ht_identity()
    for index, link in enumerate(PUMA_LINKS):
        joint_ht = ht_mul(ht, ht_rot_z(q[index]))
        joint_frames.append(joint_ht)
        ht = ht_mul(joint_ht, std_dh(*link["stdDH"]))
        frames.append(ht)
    tool = ht_mul(ht, ht_translate([0.0, 0.0, 0.0]))
    cup = ht_mul(tool, ht_translate([X_OFF_TOOL, 0.0, Z_OFF_TOOL]))
    return joint_frames, frames, tool, cup


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

    def update(self, start, end):
        self.shape.SetLineGeometry(chrono.ChLineSegment(vector(start), vector(end)))
        self.body.UpdateVisualModel()


def set_body_pose(body, position, rotation):
    body.SetPos(vector(position))
    body.SetRot(quat_from_matrix(rotation))


def make_marker(system, name, radius, tint):
    body = chrono.ChBodyEasySphere(radius, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def make_axis(system, name):
    body = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.007, JOINT_AXIS_LENGTH, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(color(0.02, 0.02, 0.025))
    system.AddBody(body)
    return body


def make_box(system, name, center, size, tint, opacity=1.0):
    body = chrono.ChBodyEasyBox(size[0], size[1], size[2], 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(vector(center))
    body.GetVisualShape(0).SetColor(tint)
    body.GetVisualShape(0).SetOpacity(opacity)
    system.AddBody(body)
    return body


def add_static_segment(system, name, start, end, tint, thickness=3):
    segment = MutableSegment(system, name, tint, thickness)
    segment.update(start, end)
    return segment


def make_bins(system):
    p0 = [0.5 * LL + 0.5 * BASE_A, 0.0, -0.5 * FLOOR_T - BASE_B]
    p1 = [-0.5 * LL, 0.5 * LL + 0.5 * BASE_A, -0.5 * FLOOR_T - BASE_B]
    steel = color(0.22, 0.42, 0.62)
    wall = color(0.60, 0.60, 0.60)
    base = color(0.36, 0.22, 0.12)
    make_box(system, "digging robot base block", [0, 0, -0.5 * BASE_B - 0.025], [BASE_A, BASE_A, BASE_B + FLOOR_T - 0.05], base)

    for label, p, open_x_sign in (("digging source bin", p0, -1.0), ("digging target bin", p1, 1.0)):
        make_box(system, f"{label} floor", p, [LL, LL, FLOOR_T], steel, 0.86)
        make_box(system, f"{label} left wall", vadd(p, [-0.5 * LL, 0.0, 0.35 * WALL_H]), [FLOOR_T, LL, 0.70 * WALL_H], wall, 0.42)
        make_box(system, f"{label} right wall", vadd(p, [0.5 * LL, 0.0, 0.50 * WALL_H]), [FLOOR_T, LL, WALL_H], wall, 0.42)
        make_box(system, f"{label} front wall", vadd(p, [0.0, -0.5 * LL, 0.42 * WALL_H]), [LL, FLOOR_T, 0.84 * WALL_H], wall, 0.42)
        make_box(system, f"{label} rear wall", vadd(p, [0.0, 0.5 * LL, 0.50 * WALL_H]), [LL, FLOOR_T, WALL_H], wall, 0.42)
        gate_start = vadd(p, [open_x_sign * 0.48 * LL, -0.48 * LL, 0.03])
        gate_end = vadd(p, [open_x_sign * 0.48 * LL, 0.48 * LL, 0.03])
        add_static_segment(system, f"{label} open-side guide", gate_start, gate_end, color(0.08, 0.08, 0.09), 3)
    return p0, p1


def make_particle(system, name, radius, tint, position):
    body = chrono.ChBodyEasySphere(radius, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(vector(position))
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def make_particle_bed(system, p0, p1, count):
    rng = random.Random(1)
    bodies = []
    per_bin = max(1, count // 2)
    radius = 0.018
    layers = max(2, int(round(per_bin ** (1.0 / 3.0))))
    nx = max(4, 2 * layers)
    ny = max(4, 2 * layers)
    nz = max(2, layers)
    centers = []
    for base in (p0, p1):
        made = 0
        for ix in range(nx):
            for iy in range(ny):
                for iz in range(nz):
                    if made >= per_bin:
                        break
                    jitter = [rng.uniform(-0.004, 0.004), rng.uniform(-0.004, 0.004), rng.uniform(-0.002, 0.002)]
                    pos = [
                        base[0] - 0.42 + ix * (0.84 / max(1, nx - 1)) + jitter[0],
                        base[1] - 0.42 + iy * (0.84 / max(1, ny - 1)) + jitter[1],
                        base[2] + FLOOR_T + radius + iz * 0.032 + jitter[2],
                    ]
                    centers.append(pos)
                    made += 1
                if made >= per_bin:
                    break
            if made >= per_bin:
                break

    for index, pos in enumerate(centers[:count]):
        height = (pos[2] - min(p0[2], p1[2])) / max(0.001, BASE_A + 0.08)
        tint = color(0.10 + 0.55 * min(1.0, height), 0.30 + 0.45 * min(1.0, height), 0.86 - 0.55 * min(1.0, height))
        bodies.append(make_particle(system, f"digging visible particle {index + 1}", radius * rng.uniform(0.78, 1.05), tint, pos))
    return bodies


def make_scoop_particles(system):
    bodies = []
    local_positions = []
    rng = random.Random(5)
    for index in range(36):
        radial = CUP_RI * math.sqrt(rng.random()) * 0.78
        angle = rng.random() * 2.0 * math.pi
        axial = rng.uniform(-0.40 * CUP_H, 0.38 * CUP_H)
        local = [X_OFF_TOOL + axial, radial * math.cos(angle), Z_OFF_TOOL + radial * math.sin(angle)]
        local_positions.append(local)
        bodies.append(make_particle(system, f"cup carried particle {index + 1}", 0.013 * rng.uniform(0.85, 1.15), color(0.88, 0.68, 0.20), [0, 0, -2]))
    return bodies, local_positions


def make_base_axes(system):
    make_box(system, "digging robot base pedestal", [0, 0, -0.04], [0.14, 0.14, 0.08], color(0.46, 0.46, 0.48))
    add_static_segment(system, "digging robot base x axis", [0, 0, 0], [0.5, 0, 0], color(0.92, 0.10, 0.08), 3)
    add_static_segment(system, "digging robot base y axis", [0, 0, 0], [0, 0.5, 0], color(0.12, 0.62, 0.18), 3)
    add_static_segment(system, "digging robot base z axis", [0, 0, 0], [0, 0, 0.5], color(0.10, 0.24, 0.88), 3)


def make_cup(system):
    cup = chrono.ChBody()
    cup.SetName("PUMA digging transparent cup")
    cup.SetFixed(True)
    cup.EnableCollision(False)

    shell = chrono.ChVisualShapeCylinder(CUP_R, CUP_H)
    shell.SetColor(color(0.85, 0.10, 0.08))
    shell.SetOpacity(0.34)
    cup.AddVisualShape(shell, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))

    rim = chrono.ChVisualShapeCylinder(CUP_R * 1.03, 0.008)
    rim.SetColor(color(0.92, 0.16, 0.10))
    cup.AddVisualShape(rim, chrono.ChFramed(chrono.ChVector3d(0, 0, 0.5 * CUP_H)))

    bottom = chrono.ChVisualShapeCylinder(CUP_RI, 0.010)
    bottom.SetColor(color(0.70, 0.05, 0.04))
    cup.AddVisualShape(bottom, chrono.ChFramed(chrono.ChVector3d(0, 0, -0.5 * CUP_H)))

    stem = chrono.ChVisualShapeCylinder(0.020, Z_OFF_TOOL - CUP_RI)
    stem.SetColor(color(0.85, 0.10, 0.08))
    cup.AddVisualShape(stem, chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, -0.5 * CUP_H - 0.5 * (Z_OFF_TOOL - CUP_RI))))

    system.AddBody(cup)
    return cup


def build_system(particle_count=DEFAULT_BED_PARTICLES):
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    p0, p1 = make_bins(system)
    make_base_axes(system)
    make_particle_bed(system, p0, p1, particle_count)
    scoop_particles, scoop_local = make_scoop_particles(system)

    link_segments = []
    com_segments = []
    joint_markers = []
    com_markers = []
    axes = []
    for index, rgb in enumerate(LINK_COLORS):
        tint = color(*rgb)
        link_segments.append(MutableSegment(system, f"PUMA digging link {index + 1} DH span", tint, 7))
        com_segments.append(MutableSegment(system, f"PUMA digging link {index + 1} COM arm", tint, 4))
        joint_markers.append(make_marker(system, f"PUMA digging joint {index + 1}", JOINT_RADIUS, tint))
        com_markers.append(make_marker(system, f"PUMA digging link {index + 1} COM", 0.017 + 0.002 * index, color(0.04, 0.04, 0.045)))
        axes.append(make_axis(system, f"PUMA digging visible joint {index + 1} z axis"))

    tcp = make_marker(system, "PUMA digging TCP marker", 0.022, color(0.96, 0.24, 0.10))
    cup = make_cup(system)
    trace = MutableSegment(system, "PUMA digging TCP travel chord", color(0.05, 0.05, 0.055), 3)

    items = {
        "link_segments": link_segments,
        "com_segments": com_segments,
        "joint_markers": joint_markers,
        "com_markers": com_markers,
        "axes": axes,
        "tcp": tcp,
        "cup": cup,
        "trace": trace,
        "scoop_particles": scoop_particles,
        "scoop_local": scoop_local,
        "p0": p0,
        "p1": p1,
    }
    system._serial_robot_digging_items = items
    update_visuals(system)
    return system, items


def cup_rotation(tool_rot):
    z_to_neg_x = [[0, 0, -1], [0, 1, 0], [1, 0, 0]]
    return matmul3(tool_rot, z_to_neg_x)


def update_visuals(system):
    items = system._serial_robot_digging_items
    q, _ = trajectory(system.GetChTime())
    joint_frames, link_frames, tool_frame, cup_frame = robot_poses(q)
    previous_origin = [0.0, 0.0, 0.0]

    for index, link in enumerate(PUMA_LINKS):
        joint_origin = ht_translation(joint_frames[index])
        next_origin = ht_translation(link_frames[index])
        com = transform_point(link_frames[index], link["COM"])
        joint_rot = ht_rotation(joint_frames[index])
        items["link_segments"][index].update(previous_origin, next_origin)
        items["com_segments"][index].update(joint_origin, com)
        set_body_pose(items["joint_markers"][index], joint_origin, joint_rot)
        set_body_pose(items["com_markers"][index], com, identity3())
        set_body_pose(items["axes"][index], joint_origin, joint_rot)
        previous_origin = next_origin

    tcp = ht_translation(tool_frame)
    tool_rot = ht_rotation(tool_frame)
    cup_pos = ht_translation(cup_frame)
    set_body_pose(items["tcp"], tcp, tool_rot)
    set_body_pose(items["cup"], cup_pos, cup_rotation(tool_rot))

    items["trace"].update(items["p0"], tcp)
    blend = min(1.0, max(0.0, (system.GetChTime() - 0.95) / 0.55))
    for body, local in zip(items["scoop_particles"], items["scoop_local"]):
        carried = transform_point(tool_frame, local)
        source = vadd(items["p0"], [0.0, 0.0, 0.18])
        pos = vadd(vscale(source, 1.0 - blend), vscale(carried, blend))
        body.SetPos(vector(pos))


def simulate(duration, step, particle_count=DEFAULT_BED_PARTICLES):
    system, items = build_system(particle_count)
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, items


def run_visual(duration, step, particle_count=DEFAULT_BED_PARTICLES):
    import pychrono.irrlicht as chronoirr

    system, items = build_system(particle_count)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1200, 760)
    vis.SetWindowTitle("EXUDYN port: serialRobotKinematicTreeDigging.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(2.05, -2.55, 1.25), chrono.ChVector3d(0.15, 0.30, 0.0))
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
            next_log += 0.5


def print_state(system):
    q, qd = trajectory(system.GetChTime())
    _, _, tool, cup = robot_poses(q)
    tcp = ht_translation(tool)
    cup_pos = ht_translation(cup)
    q_text = ",".join(f"{value:+.3f}" for value in q)
    print(
        f"t={system.GetChTime():6.3f}  cycle_time={CYCLE_TIME:.3f}  "
        f"q=[{q_text}]  q_sum={sum(q):+.9f}  qd_norm={math.sqrt(sum(v*v for v in qd)):.6f}  "
        f"tcp=({tcp[0]:+.5f},{tcp[1]:+.5f},{tcp[2]:+.5f})  "
        f"cup=({cup_pos[0]:+.5f},{cup_pos[1]:+.5f},{cup_pos[2]:+.5f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--particles", type=int, default=DEFAULT_BED_PARTICLES)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: serialRobotKinematicTreeDigging.py -> PyChrono PUMA digging replay")
    if args.no_vis:
        system, _ = simulate(args.duration, args.step, args.particles)
        print_state(system)
    else:
        run_visual(args.duration, args.step, args.particles)


if __name__ == "__main__":
    main()

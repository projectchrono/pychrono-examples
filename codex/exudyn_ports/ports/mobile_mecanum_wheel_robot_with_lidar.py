import argparse
import math

import numpy as np
import pychrono.core as chrono


# Reproduces EXUDYN Examples/mobileMecanumWheelRobotWithLidar.py as a PyChrono
# visual/analytic replay. The source is a four-wheel mecanum mobile robot that
# follows a piecewise constant-acceleration [x, y, phi_z] trajectory, controls
# wheel speeds through a torsional spring-damper motor, computes odometry, and
# transforms a 50-ray rotating lidar scan into the global frame. This port keeps
# the source trajectory, mecanum wheel-speed maps, robot dimensions, obstacle
# field, single top lidar, ray angles, visible mecanum rollers, chassis, wheels,
# walls/obstacles, scan rays, hit points, trail, and source-style diagnostics.

R_WHEEL = 0.4
W_WHEEL = 0.2
RHO_WHEEL = 500.0
L_CAR = 2.0
W_CAR = 1.5
H_CAR = R_WHEEL
M_CAR = 500.0
R_LIDAR = 0.5 * R_WHEEL
P_LIDAR = np.array([0.0, 0.0, H_CAR * 0.8], dtype=float)

MAX_DISTANCE = 20.0
LIDAR_COUNT = 50
LIDAR_ANGLE_START = 0.25 * math.pi
LIDAR_ANGLE_END = 0.75 * math.pi
DT_LIDAR_SOURCE = 0.1
P_CONTROL = 100.0

SOURCE_TRAJECTORY = [
    ([0.0, -4.0, 0.0], 3.0),
    ([0.0, -4.0, 2.1 * math.pi], 6.0),
    ([0.0, 0.0, 2.1 * math.pi], 4.0),
    ([10.0, 0.0, 2.1 * math.pi], 4.0),
    ([10.0, 0.0, 0.0], 4.0),
    ([10.0, 7.0, 0.0], 4.0),
    ([10.0, 7.0, 0.5 * math.pi], 4.0),
    ([12.0, 14.0, 0.5 * math.pi], 4.0),
]
SOURCE_TRAJECTORY_END = sum(duration for _target, duration in SOURCE_TRAJECTORY)
SOURCE_END_TIME = SOURCE_TRAJECTORY_END + 2.0
REPLAY_END_TIME = 1.20
STEP = 0.002

BOXES = [
    ("front block", np.array([0.0, 7.0, 0.5]), np.array([2.0, 1.0, 1.0])),
    ("right tower", np.array([6.0, 5.0, 1.5]), np.array([1.0, 2.0, 3.0])),
    ("lower block", np.array([3.0, -2.5, 0.5]), np.array([2.0, 1.0, 1.0])),
    ("south wall", np.array([2.0, -6.0, 0.5]), np.array([16.0, 0.2, 1.0])),
    ("north wall", np.array([2.0, 10.0, 0.5]), np.array([16.0, 0.2, 1.0])),
    ("west wall", np.array([-6.0, 2.0, 0.5]), np.array([0.2, 16.0, 1.0])),
    ("east wall", np.array([10.0, 2.0, 0.5]), np.array([0.2, 16.0, 1.0])),
]
CYLINDERS = [
    ("round column", np.array([-3.0, 0.0, 0.0]), 1.5, 1.0),
]

LIDAR_ANGLES = np.linspace(LIDAR_ANGLE_START, LIDAR_ANGLE_END, LIDAR_COUNT)


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def np_from_vec(v):
    return np.array([v.x, v.y, v.z], dtype=float)


def ch_from_np(v):
    return vec(v[0], v[1], v[2])


def source_time(replay_time):
    return SOURCE_END_TIME * min(max(replay_time / REPLAY_END_TIME, 0.0), 1.0)


def smoothstep(u):
    u = min(max(u, 0.0), 1.0)
    return u * u * (3.0 - 2.0 * u)


def smoothstep_derivative(u):
    u = min(max(u, 0.0), 1.0)
    return 6.0 * u * (1.0 - u)


def rot2(phi):
    c = math.cos(phi)
    s = math.sin(phi)
    return np.array([[c, -s], [s, c]], dtype=float)


def trajectory_state(t_source):
    start = np.array([0.0, 0.0, 0.0], dtype=float)
    elapsed = 0.0
    for target_data, duration in SOURCE_TRAJECTORY:
        target = np.array(target_data, dtype=float)
        if t_source <= elapsed + duration:
            u = (t_source - elapsed) / duration
            q = start + (target - start) * smoothstep(u)
            q_t = (target - start) * smoothstep_derivative(u) / duration
            return q, q_t
        start = target
        elapsed += duration
    return start, np.zeros(3)


def mecanum_wheel_speeds(x_vel, y_vel, yaw_rate):
    lx_ly_half = 0.5 * (W_CAR + L_CAR)
    mat = (1.0 / R_WHEEL) * np.array(
        [
            [1.0, -1.0, lx_ly_half],
            [-1.0, -1.0, -lx_ly_half],
            [-1.0, -1.0, lx_ly_half],
            [1.0, -1.0, -lx_ly_half],
        ],
        dtype=float,
    )
    return mat @ np.array([x_vel, y_vel, yaw_rate], dtype=float)


def wheel_velocities_to_mecanum(wheels):
    lx_ly_half = 0.5 * (W_CAR + L_CAR)
    mat = (1.0 / R_WHEEL) * np.array(
        [
            [1.0, -1.0, lx_ly_half],
            [-1.0, -1.0, -lx_ly_half],
            [-1.0, -1.0, lx_ly_half],
            [1.0, -1.0, -lx_ly_half],
        ],
        dtype=float,
    )
    return np.linalg.pinv(mat) @ np.asarray(wheels, dtype=float)


def wheel_angle(index, t_source):
    if t_source <= 0.0:
        return 0.0
    step = 0.05
    t = 0.0
    angle = 0.0
    previous = mecanum_wheel_speeds(*trajectory_state(0.0)[1])[index]
    while t < t_source - 1.0e-12:
        h = min(step, t_source - t)
        current = mecanum_wheel_speeds(*trajectory_state(t + h)[1])[index]
        angle += 0.5 * (previous + current) * h
        previous = current
        t += h
    return angle


def wheel_offset(index):
    dx = -0.5 * W_CAR
    dy = -0.5 * L_CAR
    if index > 1:
        dy *= -1.0
    if index == 1 or index == 3:
        dx *= -1.0
    return np.array([dx, dy, 0.0], dtype=float)


def friction_angle(index):
    angle = 0.25 * math.pi
    if index == 0 or index == 3:
        angle *= -1.0
    return angle


def pose_to_world(local, pose):
    xy = pose[:2] + rot2(pose[2]) @ np.asarray(local[:2], dtype=float)
    return np.array([xy[0], xy[1], local[2] + R_WHEEL], dtype=float)


def lidar_pose(pose):
    return pose_to_world(P_LIDAR, pose)


def lidar_direction(angle, pose):
    local = np.array([math.cos(angle), math.sin(angle), 0.0], dtype=float)
    xy = rot2(pose[2]) @ local[:2]
    direction = np.array([xy[0], xy[1], local[2]], dtype=float)
    norm = np.linalg.norm(direction)
    return direction / max(norm, 1.0e-12)


def ray_box_distance(start, direction, center, size, max_distance):
    t_min = -1.0e100
    t_max = 1.0e100
    for axis in range(3):
        origin = start[axis]
        ray = direction[axis]
        lo = center[axis] - 0.5 * size[axis]
        hi = center[axis] + 0.5 * size[axis]
        if abs(ray) < 1.0e-12:
            if origin < lo or origin > hi:
                return None
            continue
        inv = 1.0 / ray
        t0 = (lo - origin) * inv
        t1 = (hi - origin) * inv
        if t0 > t1:
            t0, t1 = t1, t0
        t_min = max(t_min, t0)
        t_max = min(t_max, t1)
        if t_max < t_min:
            return None
    distance = t_min if t_min >= 0.0 else t_max
    if 0.0 <= distance <= max_distance:
        return distance
    return None


def ray_cylinder_distance(start, direction, center, radius, height, max_distance):
    dx = start[0] - center[0]
    dy = start[1] - center[1]
    a = direction[0] * direction[0] + direction[1] * direction[1]
    if a < 1.0e-12:
        return None
    b = 2.0 * (dx * direction[0] + dy * direction[1])
    c = dx * dx + dy * dy - radius * radius
    discriminant = b * b - 4.0 * a * c
    if discriminant < 0.0:
        return None
    root = math.sqrt(discriminant)
    for distance in sorted(((-b - root) / (2.0 * a), (-b + root) / (2.0 * a))):
        if distance < 0.0 or distance > max_distance:
            continue
        z = start[2] + direction[2] * distance
        if center[2] <= z <= center[2] + height:
            return distance
    return None


def measure_lidar(pose):
    start = lidar_pose(pose)
    distances = []
    hit_points = []
    for angle in LIDAR_ANGLES:
        direction = lidar_direction(angle, pose)
        best = MAX_DISTANCE
        for _name, center, size in BOXES:
            distance = ray_box_distance(start, direction, center, size, best)
            if distance is not None:
                best = distance
        for _name, center, radius, height in CYLINDERS:
            distance = ray_cylinder_distance(start, direction, center, radius, height, best)
            if distance is not None:
                best = distance
        distances.append(best)
        hit_points.append(start + direction * best)
    return np.asarray(distances), hit_points


def add_local_cylinder(body, point_a, point_b, radius, tint):
    segment = chrono.ChLineSegment(ch_from_np(np.asarray(point_a)), ch_from_np(np.asarray(point_b)))
    shape = chrono.ChVisualShapeCylinder(radius, segment.GetLength())
    shape.SetColor(tint)
    body.AddVisualShape(shape, segment.GetFrame())
    return shape


def add_mecanum_rollers(wheel, angle):
    n_cyl = 12
    r_cyl = 0.1 * R_WHEEL
    for i in range(n_cyl):
        phi = i / n_cyl * 2.0 * math.pi
        p_axis = np.array([0.0, R_WHEEL * math.sin(phi), -R_WHEEL * math.cos(phi)])
        v_axis = np.array([0.5 * W_WHEEL * math.cos(angle), 0.5 * W_WHEEL * math.sin(angle), 0.0])
        cx = math.cos(phi)
        sx = math.sin(phi)
        v_axis2 = np.array([v_axis[0], cx * v_axis[1] - sx * v_axis[2], sx * v_axis[1] + cx * v_axis[2]])
        tint = color(0.58, 0.58, 0.58) if i < 0.5 * n_cyl else color(0.24, 0.24, 0.24)
        add_local_cylinder(wheel, p_axis - v_axis2, p_axis + v_axis2, r_cyl, tint)


def add_wheel_visuals(wheel, index):
    wheel.GetVisualShape(0).SetColor(color(0.90, 0.30, 0.24))
    wheel.GetVisualShape(0).SetOpacity(0.30)
    core = chrono.ChVisualShapeBox(1.1 * W_WHEEL, 0.70 * R_WHEEL, 0.70 * R_WHEEL)
    core.SetColor(color(0.92, 0.22, 0.16))
    wheel.AddVisualShape(core)
    hub = chrono.ChVisualShapeSphere(0.075)
    hub.SetColor(color(0.03, 0.03, 0.035))
    wheel.AddVisualShape(hub)
    add_mecanum_rollers(wheel, friction_angle(index))


def add_chassis_visual_details(chassis):
    for i in range(4):
        local = wheel_offset(i)
        axis = chrono.ChVisualShapeCylinder(0.022, 0.46)
        axis.SetColor(color(0.03, 0.03, 0.035))
        chassis.AddVisualShape(axis, chrono.ChFramed(ch_from_np(local), chrono.Q_ROTATE_Z_TO_X))
        marker = chrono.ChVisualShapeSphere(0.050)
        marker.SetColor(color(0.95, 0.72, 0.08))
        chassis.AddVisualShape(marker, chrono.ChFramed(ch_from_np(local)))

    pod = chrono.ChVisualShapeCylinder(R_LIDAR, 0.5 * R_LIDAR)
    pod.SetColor(color(0.08, 0.08, 0.09))
    chassis.AddVisualShape(pod, chrono.ChFramed(ch_from_np(P_LIDAR + np.array([0.0, 0.0, 0.05]))))

    cap = chrono.ChVisualShapeSphere(0.08)
    cap.SetColor(color(0.02, 0.02, 0.025))
    chassis.AddVisualShape(cap, chrono.ChFramed(ch_from_np(P_LIDAR + np.array([0.0, 0.0, 0.15]))))

    add_local_cylinder(chassis, [0.0, 0.0, 0.30], [0.55, 0.0, 0.30], 0.020, color(0.90, 0.10, 0.08))
    add_local_cylinder(chassis, [0.0, 0.0, 0.34], [0.0, 0.55, 0.34], 0.020, color(0.08, 0.65, 0.16))


class MutableLine:
    def __init__(self, system, name, tint, thickness=4):
        self.body = chrono.ChBody()
        self.body.SetName(name)
        self.body.SetFixed(True)
        self.body.EnableCollision(False)
        self.shape = chrono.ChVisualShapeLine()
        self.shape.SetMutable(True)
        self.shape.SetColor(tint)
        self.shape.SetThickness(thickness)
        self.body.AddVisualShape(self.shape)
        system.AddBody(self.body)

    def update(self, points):
        line = chrono.ChLinePoly(len(points))
        for i, point in enumerate(points):
            line.SetPoint(i, point)
        self.shape.SetLineGeometry(line)
        self.body.UpdateVisualModel()


class MutableSegment:
    def __init__(self, system, name, tint, thickness=3):
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


def make_box(system, name, center, size, tint, opacity=0.84):
    body = chrono.ChBodyEasyBox(size[0], size[1], size[2], 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(ch_from_np(center))
    body.GetVisualShape(0).SetColor(tint)
    body.GetVisualShape(0).SetOpacity(opacity)
    system.AddBody(body)
    return body


def make_cylinder(system, name, center, radius, height, tint):
    body = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, radius, height, 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(ch_from_np(center + np.array([0.0, 0.0, 0.5 * height])))
    body.GetVisualShape(0).SetColor(tint)
    body.GetVisualShape(0).SetOpacity(0.84)
    system.AddBody(body)
    return body


def make_marker(system, name, radius, tint):
    body = chrono.ChBodyEasySphere(radius, 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def add_scene(system):
    make_box(system, "mobile mecanum lidar checkerboard ground", np.array([2.0, 2.0, -0.03]), np.array([16.5, 16.5, 0.04]), color(0.72, 0.74, 0.70), 0.28)
    grid = []
    for value in np.linspace(-6.0, 10.0, 9):
        grid.append(MutableSegment(system, f"mobile mecanum ground x-grid {value:.1f}", color(0.32, 0.34, 0.36), 1))
        grid[-1].update(vec(value, -6.0, -0.005), vec(value, 10.0, -0.005))
        grid.append(MutableSegment(system, f"mobile mecanum ground y-grid {value:.1f}", color(0.32, 0.34, 0.36), 1))
        grid[-1].update(vec(-6.0, value, -0.004), vec(10.0, value, -0.004))

    for name, center, size in BOXES:
        make_box(system, "mobile mecanum lidar obstacle " + name, center, size, color(0.08, 0.42, 0.95))
    for name, center, radius, height in CYLINDERS:
        make_cylinder(system, "mobile mecanum lidar obstacle " + name, center, radius, height, color(0.08, 0.42, 0.95))


def build_robot(system):
    chassis = chrono.ChBodyEasyBox(W_CAR - 1.1 * W_WHEEL, L_CAR + 2.0 * R_WHEEL, H_CAR, M_CAR / ((W_CAR - 1.1 * W_WHEEL) * (L_CAR + 2.0 * R_WHEEL) * H_CAR), True, False)
    chassis.SetName("mobile mecanum lidar steelblue chassis")
    chassis.SetFixed(True)
    chassis.EnableCollision(False)
    chassis.GetVisualShape(0).SetColor(color(0.16, 0.42, 0.82))
    chassis.GetVisualShape(0).SetOpacity(0.62)
    add_chassis_visual_details(chassis)
    system.AddBody(chassis)

    wheels = []
    for i in range(4):
        wheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_X, R_WHEEL, W_WHEEL, RHO_WHEEL, True, False)
        wheel.SetName(f"mobile mecanum lidar wheel {i}")
        wheel.SetFixed(True)
        wheel.EnableCollision(False)
        add_wheel_visuals(wheel, i)
        system.AddBody(wheel)
        wheels.append(wheel)
    return chassis, wheels


def trajectory_points():
    points = []
    for i in range(160):
        t = SOURCE_TRAJECTORY_END * i / 159
        q, _q_t = trajectory_state(t)
        points.append(vec(q[0], q[1], 0.035))
    return points


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, -9.81))
    add_scene(system)
    chassis, wheels = build_robot(system)

    rays = [MutableSegment(system, f"mobile mecanum lidar red ray {i:02d}", color(0.95, 0.10, 0.08), 4) for i in range(LIDAR_COUNT)]
    hits = [make_marker(system, f"mobile mecanum lidar hit point {i:02d}", 0.065, color(0.95, 0.52, 0.06)) for i in range(LIDAR_COUNT)]
    trail = MutableLine(system, "mobile mecanum source trajectory trail", color(0.05, 0.50, 0.95), 4)
    trail.update(trajectory_points())
    pose_marker = make_marker(system, "mobile mecanum current trajectory marker", 0.085, color(0.95, 0.72, 0.10))

    system._mobile_mecanum_lidar = {
        "chassis": chassis,
        "wheels": wheels,
        "rays": rays,
        "hits": hits,
        "pose_marker": pose_marker,
        "last": None,
    }
    update_visuals(system)
    return system, system._mobile_mecanum_lidar


def update_visuals(system):
    items = system._mobile_mecanum_lidar
    t_source = source_time(system.GetChTime())
    pose, pose_t = trajectory_state(t_source)
    q_yaw = chrono.QuatFromAngleZ(pose[2])
    chassis_center = np.array([pose[0], pose[1], R_WHEEL], dtype=float)
    items["chassis"].SetPos(ch_from_np(chassis_center))
    items["chassis"].SetRot(q_yaw)
    items["chassis"].UpdateVisualModel()

    wheel_speeds = mecanum_wheel_speeds(*pose_t)
    for i, wheel in enumerate(items["wheels"]):
        center = pose_to_world(wheel_offset(i), pose)
        spin = wheel_angle(i, t_source)
        wheel.SetPos(ch_from_np(center))
        wheel.SetRot(q_yaw * chrono.QuatFromAngleX(spin))
        wheel.SetAngVelParent(vec(wheel_speeds[i], 0.0, pose_t[2]))
        wheel.UpdateVisualModel()

    distances, hit_points = measure_lidar(pose)
    start = lidar_pose(pose)
    for i, angle in enumerate(LIDAR_ANGLES):
        direction = lidar_direction(angle, pose)
        end = hit_points[i]
        items["rays"][i].update(ch_from_np(start), ch_from_np(end))
        items["hits"][i].SetPos(ch_from_np(end + np.array([0.0, 0.0, 0.045])))
        items["hits"][i].UpdateVisualModel()
    items["pose_marker"].SetPos(ch_from_np(chassis_center + np.array([0.0, 0.0, 0.45])))
    items["pose_marker"].UpdateVisualModel()
    items["last"] = {"source_t": t_source, "pose": pose, "pose_t": pose_t, "wheel_speeds": wheel_speeds, "distances": distances}


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration - 1.0e-14:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def print_state(system):
    data = system._mobile_mecanum_lidar["last"]
    pose = data["pose"]
    pose_t = data["pose_t"]
    wheel_speeds = data["wheel_speeds"]
    distances = data["distances"]
    odom_twist = wheel_velocities_to_mecanum(wheel_speeds)
    hit_count = int(np.sum(distances < MAX_DISTANCE - 1.0e-9))
    print(
        f"t_replay={system.GetChTime():6.3f}  source_t={data['source_t']:7.3f}  "
        f"pose=({pose[0]:+.6f},{pose[1]:+.6f},{pose[2]:+.6f})  "
        f"vel=({pose_t[0]:+.6f},{pose_t[1]:+.6f},{pose_t[2]:+.6f})"
    )
    print(
        f"wheel_omega=({wheel_speeds[0]:+.6f},{wheel_speeds[1]:+.6f},{wheel_speeds[2]:+.6f},{wheel_speeds[3]:+.6f})  "
        f"odom_twist=({odom_twist[0]:+.6f},{odom_twist[1]:+.6f},{odom_twist[2]:+.6f})"
    )
    print(
        f"lidar: rays={LIDAR_COUNT} hits={hit_count} min={float(np.min(distances)):.6f} "
        f"mean={float(np.mean(distances)):.6f} max={float(np.max(distances)):.6f}  "
        f"obstacles={len(BOXES) + len(CYLINDERS)}  dtLidar={DT_LIDAR_SOURCE:.3f}  pControl={P_CONTROL:.1f}"
    )


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1120, 760)
    vis.SetWindowTitle("EXUDYN port: mobileMecanumWheelRobotWithLidar.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(4.6, -8.3, 5.8), chrono.ChVector3d(0.4, -1.7, 0.55))
    vis.AddTypicalLights()

    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
    update_visuals(system)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=REPLAY_END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: mobileMecanumWheelRobotWithLidar.py -> PyChrono mecanum trajectory/lidar replay")
    print(
        f"source parameters: rWheel={R_WHEEL:.3f} wWheel={W_WHEEL:.3f} wCar={W_CAR:.3f} "
        f"lCar={L_CAR:.3f} lidar_rays={LIDAR_COUNT} source_end={SOURCE_END_TIME:.3f}"
    )
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

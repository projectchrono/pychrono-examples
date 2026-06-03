import math

import numpy as np
import pychrono.core as chrono

from robotics_replay_common import (
    MutablePolyline,
    MutableSegment,
    UR5_LINKS,
    add_grid_ground,
    color,
    generic_main,
    ht_rot_z,
    ht_translate,
    make_box,
    make_cylinder,
    make_marker,
    make_serial_arm,
    quat_from_matrix,
    rot2,
    set_body_pose,
    transform_point,
    update_serial_arm,
    vec,
)


ROS_STEP = 0.001
ROS_REALTIME_END = 1.0e5
MASS_POINT_MASS = 6.0
MASS_POINT_RADIUS = 0.03
MASS_POINT_STIFFNESS = 100.0
MASS_POINT_DAMPING = MASS_POINT_STIFFNESS * 5.0e-2

TURTLE_BOX = np.array([0.5, 0.25, 0.1])
TURTLE_DENSITY = 1.0e-5
TURTLE_DAMPING = 1.0e-4

KAIROS_PLATFORM_DIMENSIONS = np.array([0.575, 0.718, 0.2495])
KAIROS_PLATFORM_MASS = 125.0 - 18.4
KAIROS_WHEEL_TRACK = 0.390
KAIROS_WHEEL_BASE = 0.430
KAIROS_WHEEL_RADIUS = 0.254 / 2.0
KAIROS_WHEEL_WIDTH = 0.1
KAIROS_SERIAL_MOUNT = np.array([0.178, 0.0, 0.12])
KAIROS_Q_OFFSET = np.array([-0.25 * math.pi, 0.0, 0.0, 0.0, 0.0, 0.0])
KAIROS_Q0 = np.array(
    [-0.75 * math.pi, math.pi - 1.0e-15, 0.75 * math.pi, -1.0e-15, -1.0e-15, -1.0e-15],
    dtype=float,
)
KAIROS_TARGET = np.array([3.0, 1.0, 0.685])


def clamp(value, low, high):
    return max(low, min(high, float(value)))


def smoothstep(u):
    u = clamp(u, 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


def set_pose(body, position, rotation=None):
    body.SetPos(vec(position))
    if rotation is not None:
        body.SetRot(rotation)
    body.UpdateVisualModel()


def coil_between(start, end, radius=0.025, turns=7.0, count=96):
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    axis = end - start
    length = float(np.linalg.norm(axis))
    if length < 1.0e-9:
        return [start.copy(), end.copy()]
    axis /= length
    ref = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(axis, ref))) > 0.85:
        ref = np.array([0.0, 1.0, 0.0])
    u = np.cross(axis, ref)
    u /= max(float(np.linalg.norm(u)), 1.0e-12)
    v = np.cross(axis, u)
    points = []
    for i in range(count):
        s = i / (count - 1)
        center = start + axis * length * s
        local_radius = radius * math.sin(math.pi * s)
        angle = 2.0 * math.pi * turns * s
        points.append(center + local_radius * (math.cos(angle) * u + math.sin(angle) * v))
    return points


def yaw_spiral(center, yaw, radius0=0.055, radius1=0.20, turns=2.5, count=96):
    points = []
    sign = 1.0 if yaw >= 0.0 else -1.0
    for i in range(count):
        s = i / (count - 1)
        r = radius0 + (radius1 - radius0) * s
        a = yaw + sign * 2.0 * math.pi * turns * s
        points.append(np.asarray(center, dtype=float) + np.array([r * math.cos(a), r * math.sin(a), 0.0]))
    return points


def add_arrow(system, name, tint, thickness=5):
    return {
        "shaft": MutableSegment(system, name + " shaft", tint, thickness),
        "head_a": MutableSegment(system, name + " head a", tint, max(3, thickness - 1)),
        "head_b": MutableSegment(system, name + " head b", tint, max(3, thickness - 1)),
    }


def update_arrow(arrow, start, end, head_length=0.08):
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    arrow["shaft"].update(start, end)
    direction = end - start
    norm = float(np.linalg.norm(direction))
    if norm < 1.0e-12:
        arrow["head_a"].update(end, end)
        arrow["head_b"].update(end, end)
        return
    direction /= norm
    side = np.array([-direction[1], direction[0], 0.0])
    if float(np.linalg.norm(side)) < 1.0e-9:
        side = np.array([0.0, 1.0, 0.0])
    side /= max(float(np.linalg.norm(side)), 1.0e-12)
    arrow["head_a"].update(end, end - head_length * direction + 0.45 * head_length * side)
    arrow["head_b"].update(end, end - head_length * direction - 0.45 * head_length * side)


def mass_point_state(time):
    pos = np.array(
        [
            0.08 * math.sin(1.35 * time),
            0.055 * math.sin(0.85 * time + 0.35),
            0.045 * math.sin(1.10 * time - 0.20),
        ],
        dtype=float,
    )
    vel = np.array(
        [
            0.08 * 1.35 * math.cos(1.35 * time),
            0.055 * 0.85 * math.cos(0.85 * time + 0.35),
            0.045 * 1.10 * math.cos(1.10 * time - 0.20),
        ],
        dtype=float,
    )
    force = np.array([1.8 * math.sin(1.6 * time), 1.1 * math.cos(1.1 * time), 0.8 * math.sin(2.2 * time)])
    torque = np.array([0.12 * math.sin(0.7 * time), 0.10 * math.cos(0.9 * time), 0.18 * math.sin(1.4 * time)])
    spring_force = -MASS_POINT_STIFFNESS * pos - MASS_POINT_DAMPING * vel
    return pos, vel, force, torque, spring_force


def build_ros_mass_point_system(source_label, modern_api=True):
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, source_label + " yz reference plane", (0.0, 0.0), (1.4, 1.4), z=-0.11, tile_count=8)
    make_box(system, source_label + " ROS checkerboard wall", (0.025, 1.20, 1.20), color(0.70, 0.72, 0.70), (-0.50, 0.0, 0.48), 0.24)

    sphere = make_marker(system, source_label + " red 3D mass rigid body", MASS_POINT_RADIUS, color(0.95, 0.05, 0.04))
    ground = make_marker(system, source_label + " ground marker", 0.025, color(0.03, 0.03, 0.035))
    ground.SetPos(vec(0, 0, 0))
    spring = MutablePolyline(system, source_label + " visible 6D spring-damper coil", color(0.04, 0.04, 0.045), 3)
    force_arrow = add_arrow(system, source_label + " /cmd_wrench force", color(0.05, 0.32, 0.95), 5)
    spring_arrow = add_arrow(system, source_label + " spring restoring force", color(0.92, 0.50, 0.06), 4)
    torque_spiral = MutablePolyline(system, source_label + " /cmd_wrench torque spiral", color(0.88, 0.14, 0.08), 3)
    topic_bars = [
        MutableSegment(system, source_label + f" ROS topic activity bar {name}", color(0.10, 0.56, 0.26), 4)
        for name in ("Pose", "Twist", "SystemState", "cmd_wrench", "my_string")
    ]
    system._ros_mass_point = {
        "source": source_label,
        "modern_api": modern_api,
        "sphere": sphere,
        "spring": spring,
        "force_arrow": force_arrow,
        "spring_arrow": spring_arrow,
        "torque_spiral": torque_spiral,
        "topic_bars": topic_bars,
    }
    update_ros_mass_point_visuals(system)
    return system, system._ros_mass_point


def update_ros_mass_point_visuals(system):
    items = system._ros_mass_point
    time = system.GetChTime()
    pos, vel, force, torque, spring_force = mass_point_state(time)
    set_pose(items["sphere"], pos)
    items["spring"].update(coil_between((0, 0, 0), pos, radius=0.018, turns=6.0, count=112))
    update_arrow(items["force_arrow"], pos + np.array([0.0, 0.0, 0.06]), pos + np.array([0.0, 0.0, 0.06]) + 0.055 * force)
    update_arrow(items["spring_arrow"], pos + np.array([0.0, 0.0, -0.06]), pos + np.array([0.0, 0.0, -0.06]) + 0.008 * spring_force)
    items["torque_spiral"].update(yaw_spiral(pos + np.array([0.0, 0.0, 0.10]), torque[2], radius0=0.035, radius1=0.10, turns=2.0, count=72))
    activity = (1.0, 1.0, 1.0, min(1.0, float(np.linalg.norm(force)) / 2.4), 0.45 + 0.45 * math.sin(0.9 * time) ** 2)
    for i, (bar, value) in enumerate(zip(items["topic_bars"], activity)):
        base = np.array([-0.58 + 0.10 * i, -0.66, -0.10])
        bar.update(base, base + np.array([0.0, 0.0, 0.18 * value]))
    items["last"] = {"pos": pos, "vel": vel, "force": force, "torque": torque, "spring_force": spring_force}


def print_ros_mass_point_state(system):
    items = system._ros_mass_point
    data = items["last"]
    api = "CreateRigidBody" if items["modern_api"] else "AddRigidBody"
    print(
        f"t={system.GetChTime():.4f} source={items['source']} node=exuROSexample3Dmass api={api} "
        f"topicsPub=(Pose,Twist,SystemState,Time,String) topicsSub=(cmd_vel,cmd_wrench,my_string,exudyn/SimpleString) "
        f"mass={MASS_POINT_MASS:.1f} radius={MASS_POINT_RADIUS:.3f} k={MASS_POINT_STIFFNESS:.1f} d={MASS_POINT_DAMPING:.1f} "
        f"pos=({data['pos'][0]:+.5f},{data['pos'][1]:+.5f},{data['pos'][2]:+.5f}) "
        f"cmdForce=({data['force'][0]:+.5f},{data['force'][1]:+.5f},{data['force'][2]:+.5f}) "
        f"springNorm={float(np.linalg.norm(data['spring_force'])):.6f}"
    )


def main_ros_mass_point(build_system, title, intro, duration=2.0):
    generic_main(
        build_system,
        update_ros_mass_point_visuals,
        print_ros_mass_point_state,
        title,
        duration,
        ROS_STEP,
        (1.35, -2.0, 1.15),
        (0.0, 0.0, 0.12),
        intro,
    )


def turtle_state(time, external_publisher=False):
    if external_publisher:
        cmd_v = 0.55 + 0.20 * math.sin(0.9 * time)
        cmd_w = 0.65 * math.sin(0.55 * time)
        theta = 0.65 * (1.0 - math.cos(0.55 * time)) / 0.55
        x = 0.55 * time + 0.12 * math.sin(0.9 * time)
        y = 0.18 * math.sin(0.55 * time)
    else:
        cmd_v = 1.0
        cmd_w = 1.0
        theta = time
        x = math.sin(time)
        y = 1.0 - math.cos(time)
    pos = np.array([x, y, 0.075], dtype=float)
    return pos, theta, cmd_v, cmd_w


def turtle_path_points(time, external_publisher=False):
    horizon = max(time, 0.02)
    points = []
    for i in range(96):
        t = horizon * i / 95.0
        pos, _theta, _cmd_v, _cmd_w = turtle_state(t, external_publisher)
        points.append(pos + np.array([0.0, 0.0, 0.07]))
    return points


def build_ros_turtle_system(source_label, stl_name="ROSTurtle.stl", modern_api=True):
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, source_label + " turtle checkerboard", (0.70, 0.70), (3.4, 3.4), z=0.0, tile_count=10)
    body = make_box(system, source_label + " turtle fallback red body", TURTLE_BOX, color(0.92, 0.06, 0.04), (0, 0, 0.075), 0.92)
    nose = make_box(system, source_label + " turtle heading nose", (0.18, 0.08, 0.08), color(0.96, 0.56, 0.06), (0.30, 0, 0.08), 0.92)
    pose_marker = make_marker(system, source_label + " ROS pose marker", 0.025, color(0.05, 0.05, 0.055))
    path = MutablePolyline(system, source_label + " sensor-stored turtle track", color(0.05, 0.36, 0.92), 4)
    cartesian_coil = MutablePolyline(system, source_label + " CartesianSpringDamper velocity coil", color(0.04, 0.04, 0.045), 3)
    torsional_spiral = MutablePolyline(system, source_label + " TorsionalSpringDamper yaw spiral", color(0.92, 0.50, 0.06), 3)
    velocity_arrow = add_arrow(system, source_label + " desired cmd_vel arrow", color(0.05, 0.32, 0.95), 5)
    turn_bar = MutableSegment(system, source_label + " angular cmd_vel bar", color(0.92, 0.50, 0.06), 5)
    topic_bars = [
        MutableSegment(system, source_label + f" ROS topic bar {name}", color(0.10, 0.56, 0.26), 4)
        for name in ("Twist", "Pose", "SystemState", "cmd_vel", "my_string")
    ]
    system._ros_turtle = {
        "source": source_label,
        "stl_name": stl_name,
        "modern_api": modern_api,
        "body": body,
        "nose": nose,
        "pose_marker": pose_marker,
        "path": path,
        "cartesian_coil": cartesian_coil,
        "torsional_spiral": torsional_spiral,
        "velocity_arrow": velocity_arrow,
        "turn_bar": turn_bar,
        "topic_bars": topic_bars,
        "external_publisher": False,
        "save_track": True,
    }
    update_ros_turtle_visuals(system)
    return system, system._ros_turtle


def update_ros_turtle_visuals(system):
    items = system._ros_turtle
    time = system.GetChTime()
    pos, theta, cmd_v, cmd_w = turtle_state(time, items["external_publisher"])
    yaw = chrono.QuatFromAngleZ(theta)
    set_pose(items["body"], pos, yaw)
    nose_offset = rot2(theta) @ np.array([0.34, 0.0])
    set_pose(items["nose"], pos + np.array([nose_offset[0], nose_offset[1], 0.0]), yaw)
    set_pose(items["pose_marker"], pos + np.array([0.0, 0.0, 0.11]))
    items["path"].update(turtle_path_points(time, items["external_publisher"]))
    items["cartesian_coil"].update(coil_between((0, 0, 0.055), pos, radius=0.025, turns=9.0, count=128))
    items["torsional_spiral"].update(yaw_spiral(pos + np.array([0.0, 0.0, 0.12]), theta, radius0=0.045, radius1=0.16, turns=2.5, count=96))
    forward = np.array([math.cos(theta), math.sin(theta), 0.0])
    update_arrow(items["velocity_arrow"], pos + np.array([0.0, 0.0, 0.15]), pos + np.array([0.0, 0.0, 0.15]) + 0.22 * cmd_v * forward)
    bar_base = np.array([-0.85, 1.95, 0.03])
    items["turn_bar"].update(bar_base, bar_base + np.array([0.0, 0.0, 0.25 * clamp(cmd_w, -1.0, 1.0)]))
    activity = (1.0, 1.0, 1.0, 0.35 if not items["external_publisher"] else 1.0, 0.25 + 0.55 * math.sin(0.7 * time) ** 2)
    for i, (bar, value) in enumerate(zip(items["topic_bars"], activity)):
        base = np.array([-1.02 + 0.12 * i, 1.72, 0.03])
        bar.update(base, base + np.array([0.0, 0.0, 0.20 * value]))
    items["last"] = {"pos": pos, "theta": theta, "cmd_v": cmd_v, "cmd_w": cmd_w}


def print_ros_turtle_state(system):
    items = system._ros_turtle
    data = items["last"]
    api = "CreateRigidBody" if items["modern_api"] else "AddRigidBody"
    print(
        f"t={system.GetChTime():.4f} source={items['source']} node=ROSexampleTurtle api={api} stl={items['stl_name']} "
        f"topicsPub=(Twist,Pose,SystemState) topicsSub=(cmd_vel,my_string) hearToPublisher={items['external_publisher']} "
        f"saveTrack={items['save_track']} box=({TURTLE_BOX[0]:.2f},{TURTLE_BOX[1]:.2f},{TURTLE_BOX[2]:.2f}) "
        f"dampingHelper={TURTLE_DAMPING:.1e} pos=({data['pos'][0]:+.5f},{data['pos'][1]:+.5f},{data['pos'][2]:+.5f}) "
        f"theta={data['theta']:+.6f} desiredLinear={data['cmd_v']:+.3f} desiredAngularZ={data['cmd_w']:+.3f}"
    )


def main_ros_turtle(build_system, title, intro, duration=2.2):
    generic_main(
        build_system,
        update_ros_turtle_visuals,
        print_ros_turtle_state,
        title,
        duration,
        ROS_STEP,
        (2.6, -3.4, 2.1),
        (0.60, 0.80, 0.10),
        intro,
    )


def mobile_pose(time):
    u = smoothstep(min(time / 4.0, 1.0))
    x = 1.25 * u
    y = 0.34 * math.sin(0.9 * time) * smoothstep(min(time / 1.0, 1.0))
    theta = 0.30 * math.sin(0.55 * time)
    vx = 0.0 if time > 4.0 else 1.25 * 6.0 * (time / 4.0) * (1.0 - time / 4.0) / 4.0
    vy = 0.34 * 0.9 * math.cos(0.9 * time) * smoothstep(min(time / 1.0, 1.0))
    yaw_rate = 0.30 * 0.55 * math.cos(0.55 * time)
    return np.array([x, y, theta], dtype=float), np.array([vx, vy, yaw_rate], dtype=float)


def kairos_wheel_speeds(vel):
    vx, vy, wz = vel
    lx = KAIROS_WHEEL_TRACK
    ly = KAIROS_WHEEL_BASE
    r = KAIROS_WHEEL_RADIUS
    return np.array(
        [
            (vx - vy - (lx + ly) * 0.5 * wz) / r,
            (vx + vy + (lx + ly) * 0.5 * wz) / r,
            (vx + vy - (lx + ly) * 0.5 * wz) / r,
            (vx - vy + (lx + ly) * 0.5 * wz) / r,
        ],
        dtype=float,
    )


def mobile_arm_state(time):
    u = smoothstep(min(max((time - 0.65) / 2.2, 0.0), 1.0))
    q_target = KAIROS_Q0 + np.array([0.35, -0.62, 0.38, -0.30, 0.44, 0.22])
    q = KAIROS_Q0 + u * (q_target - KAIROS_Q0)
    qd = (q_target - KAIROS_Q0) * (6.0 * u * (1.0 - u) / 2.2 if 0.0 < u < 1.0 else 0.0)
    return q, qd


def mobile_base_ht(pose):
    return ht_translate((pose[0], pose[1], 0.375)) @ ht_rot_z(float(pose[2])) @ ht_translate(KAIROS_SERIAL_MOUNT) @ ht_rot_z(KAIROS_Q_OFFSET[0])


def build_ros_mobile_manipulator_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, "ROSMobileManipulator checkerboard", (1.40, 0.45), (4.8, 3.0), z=0.0, tile_count=12)
    platform = make_box(system, "ROSMobileManipulator KAIROS platform box", KAIROS_PLATFORM_DIMENSIONS, color(0.18, 0.46, 0.74), (0, 0, 0.375), 0.88)
    wheels = []
    for index in range(4):
        wheels.append(
            make_cylinder(
                system,
                f"ROSMobileManipulator mecanum wheel {index + 1}",
                chrono.ChAxis_Y,
                KAIROS_WHEEL_RADIUS,
                KAIROS_WHEEL_WIDTH,
                color(0.05, 0.05, 0.055),
                (0, 0, KAIROS_WHEEL_RADIUS),
                0.94,
            )
        )
    arm = make_serial_arm(system, "ROSMobileManipulator UR5", UR5_LINKS, mobile_base_ht(np.zeros(3)), (0.0, 0.0, 0.155), base_size=(0.18, 0.18, 0.10), link_thickness=7)
    table = make_box(system, "ROSMobileManipulator target table", (0.20, 0.40, 0.645), color(0.32, 0.32, 0.34), (KAIROS_TARGET[0], KAIROS_TARGET[1], 0.3225), 0.80)
    target = make_cylinder(system, "ROSMobileManipulator lightgreen grasp object", chrono.ChAxis_Z, 0.020, 0.080, color(0.42, 0.92, 0.42), KAIROS_TARGET, 0.95)
    platform_path = MutablePolyline(system, "ROSMobileManipulator platform ROS path", color(0.05, 0.36, 0.92), 4)
    grasp_coil = MutablePolyline(system, "ROSMobileManipulator hidden grasp spring-damper coil", color(0.04, 0.04, 0.045), 3)
    wheel_bars = [MutableSegment(system, f"ROSMobileManipulator wheel velocity bar {i + 1}", color(0.92, 0.50, 0.06), 4) for i in range(4)]
    topic_bars = [
        MutableSegment(system, f"ROSMobileManipulator ROS topic bar {name}", color(0.10, 0.56, 0.26), 4)
        for name in ("cmd_vel", "my_string", "my_pose", "Twist", "Pose")
    ]
    system._ros_mobile_manipulator = {
        "platform": platform,
        "wheels": wheels,
        "arm": arm,
        "table": table,
        "target": target,
        "platform_path": platform_path,
        "grasp_coil": grasp_coil,
        "wheel_bars": wheel_bars,
        "topic_bars": topic_bars,
    }
    update_ros_mobile_manipulator_visuals(system)
    return system, system._ros_mobile_manipulator


def update_ros_mobile_manipulator_visuals(system):
    items = system._ros_mobile_manipulator
    time = system.GetChTime()
    pose, vel = mobile_pose(time)
    yaw = chrono.QuatFromAngleZ(float(pose[2]))
    platform_center = np.array([pose[0], pose[1], 0.375])
    set_pose(items["platform"], platform_center, yaw)
    wheel_speeds = kairos_wheel_speeds(vel)
    wheel_local = (
        (0.5 * KAIROS_WHEEL_BASE, 0.5 * KAIROS_WHEEL_TRACK),
        (0.5 * KAIROS_WHEEL_BASE, -0.5 * KAIROS_WHEEL_TRACK),
        (-0.5 * KAIROS_WHEEL_BASE, 0.5 * KAIROS_WHEEL_TRACK),
        (-0.5 * KAIROS_WHEEL_BASE, -0.5 * KAIROS_WHEEL_TRACK),
    )
    for i, (wheel, local) in enumerate(zip(items["wheels"], wheel_local)):
        xy = pose[:2] + rot2(pose[2]) @ np.asarray(local)
        set_pose(wheel, (xy[0], xy[1], KAIROS_WHEEL_RADIUS), yaw * chrono.QuatFromAngleY(float(wheel_speeds[i] * time)))
    q, qd = mobile_arm_state(time)
    base_ht = mobile_base_ht(pose)
    items["arm"]["base_ht"] = base_ht
    set_body_pose(items["arm"]["base"], transform_point(base_ht, (0, 0, -0.05)), base_ht[:3, :3])
    update_serial_arm(items["arm"], q + KAIROS_Q_OFFSET)
    tcp = items["arm"]["last_tcp"]
    items["grasp_coil"].update(coil_between(tcp, KAIROS_TARGET, radius=0.018, turns=10.0, count=128))
    path_points = []
    horizon = max(time, 0.02)
    for i in range(96):
        p, _v = mobile_pose(horizon * i / 95.0)
        path_points.append(np.array([p[0], p[1], 0.055]))
    items["platform_path"].update(path_points)
    for i, (bar, wheel_speed) in enumerate(zip(items["wheel_bars"], wheel_speeds)):
        base = np.array([-0.95 + 0.13 * i, -1.25, 0.03])
        bar.update(base, base + np.array([0.0, 0.0, 0.22 * clamp(wheel_speed / 24.0, -1.0, 1.0)]))
    stage = "ms" if time < 0.65 else ("a" if time < 2.85 else "mk")
    topic_activity = (1.0, 1.0 if stage != "mk" else 0.35, 1.0 if stage == "a" else 0.20, 1.0, 1.0)
    for i, (bar, value) in enumerate(zip(items["topic_bars"], topic_activity)):
        base = np.array([-0.96 + 0.12 * i, -1.42, 0.03])
        bar.update(base, base + np.array([0.0, 0.0, 0.20 * value]))
    items["last"] = {"pose": pose, "vel": vel, "wheel_speeds": wheel_speeds, "q": q, "qd": qd, "tcp": tcp, "stage": stage}


def print_ros_mobile_manipulator_state(system):
    data = system._ros_mobile_manipulator["last"]
    print(
        f"t={system.GetChTime():.4f} source=ROSMobileManipulator.py node=ROSMobileManipulator "
        f"topicsSub=(cmd_vel,my_string,my_pose) topicsPub=(Twist,Pose) state={data['stage']} "
        f"platformDims=({KAIROS_PLATFORM_DIMENSIONS[0]:.3f},{KAIROS_PLATFORM_DIMENSIONS[1]:.3f},{KAIROS_PLATFORM_DIMENSIONS[2]:.4f}) "
        f"wheelR={KAIROS_WHEEL_RADIUS:.3f} wheelTrack={KAIROS_WHEEL_TRACK:.3f} wheelBase={KAIROS_WHEEL_BASE:.3f} "
        f"pose=({data['pose'][0]:+.5f},{data['pose'][1]:+.5f},{data['pose'][2]:+.5f}) "
        f"wheelOmega=({data['wheel_speeds'][0]:+.4f},{data['wheel_speeds'][1]:+.4f},{data['wheel_speeds'][2]:+.4f},{data['wheel_speeds'][3]:+.4f}) "
        f"ur5Qsum={float(np.sum(data['q'])):+.6f} target=({KAIROS_TARGET[0]:+.2f},{KAIROS_TARGET[1]:+.2f},{KAIROS_TARGET[2]:+.3f})"
    )


def main_ros_mobile_manipulator(build_system, title, intro, duration=3.0):
    generic_main(
        build_system,
        update_ros_mobile_manipulator_visuals,
        print_ros_mobile_manipulator_state,
        title,
        duration,
        0.002,
        (3.1, -4.7, 2.8),
        (1.25, 0.10, 0.55),
        intro,
    )

import math
import struct

import numpy as np
import pychrono.core as chrono

from robotics_replay_common import (
    MutablePolyline,
    MutableSegment,
    add_grid_ground,
    color,
    generic_main,
    make_box,
    make_marker,
    vec,
)


TCP_HOST = "127.0.0.1"
TCP_PORT = 65124
PACKER = struct.Struct("I d d d d")
ACK_PACKER = struct.Struct("I")

CLIENT_DT = 0.5
CLIENT_END = 10.0
CLIENT_X = 1.13
CLIENT_Y = 4.0
CLIENT_Z = -3.1415

MATLAB_L = 1.0
MATLAB_FL = 2.5
MATLAB_MASS = 12.0
MATLAB_INERTIA = MATLAB_MASS / 12.0 * (2.0 * MATLAB_L) ** 2
MATLAB_GRAVITY = 9.81
MATLAB_DAMPING = 10.0
MATLAB_SAMPLE_TIME = 0.01
MATLAB_STEP = 0.002
MATLAB_END = 10.0


def lerp(a, b, u):
    return np.asarray(a, dtype=float) + float(u) * (np.asarray(b, dtype=float) - np.asarray(a, dtype=float))


def clamp(value, low, high):
    return max(low, min(high, float(value)))


def smooth(u):
    u = clamp(u, 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


def set_body_color(body, tint):
    body.GetVisualShape(0).SetColor(tint)
    body.UpdateVisualModel()


def set_marker_pos(body, point):
    body.SetPos(vec(point))
    body.UpdateVisualModel()


def tcp_packet_time(time):
    index = min(int(math.floor(max(time, 0.0) / CLIENT_DT + 1.0e-12)), int(CLIENT_END / CLIENT_DT) - 1)
    return index * CLIENT_DT


def tcp_values(time):
    return (4, tcp_packet_time(time), CLIENT_X, CLIENT_Y, CLIENT_Z)


def tcp_packet_info(time):
    values = tcp_values(time)
    packet = PACKER.pack(*values)
    unpacked = PACKER.unpack(packet)
    checksum = sum(packet)
    server_ack = int(checksum + unpacked[1])
    client_expected = checksum
    return {
        "values": values,
        "unpacked": unpacked,
        "packet": packet,
        "checksum": checksum,
        "server_ack": server_ack,
        "client_expected": client_expected,
        "ack_ok": server_ack == client_expected,
        "packet_size": PACKER.size,
        "ack_size": ACK_PACKER.size,
        "phase": (max(time, 0.0) / CLIENT_DT) % 1.0,
    }


def build_tcpip_python_system(source_label, role):
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, f"{role} TCPIP checkerboard", (0.0, 0.0), (4.0, 1.6), z=0.0, tile_count=10)

    client = make_box(system, f"{role} TCPIP Python client process body", (0.46, 0.30, 0.24), color(0.08, 0.36, 0.86), (-1.25, 0.0, 0.16), 0.88)
    server = make_box(system, f"{role} TCPIP Python server process body", (0.46, 0.30, 0.24), color(0.12, 0.62, 0.26), (1.25, 0.0, 0.16), 0.88)
    socket_line = MutableSegment(system, f"{role} TCPIP 127.0.0.1:{TCP_PORT} stream", color(0.06, 0.06, 0.07), 5)
    socket_line.update((-1.02, 0.0, 0.25), (1.02, 0.0, 0.25))

    packet = make_marker(system, f"{role} TCPIP packed Struct('I d d d d') message", 0.060, color(0.95, 0.56, 0.06))
    ack = make_marker(system, f"{role} TCPIP unsigned-int ack packet", 0.047, color(0.05, 0.46, 0.95))
    status = make_marker(system, f"{role} TCPIP checksum status marker", 0.060, color(0.08, 0.70, 0.22))

    value_bars = [
        MutableSegment(system, f"{role} TCPIP value bar {name}", color(0.95, 0.56, 0.06), 4)
        for name in ("count", "t", "x", "y", "z")
    ]
    checksum_bar = MutableSegment(system, f"{role} TCPIP client checksum bar", color(0.94, 0.50, 0.06), 5)
    ack_bar = MutableSegment(system, f"{role} TCPIP server ack bar", color(0.05, 0.42, 0.95), 5)
    mismatch = MutableSegment(system, f"{role} TCPIP checksum mismatch cue", color(0.88, 0.08, 0.08), 4)

    system._tcpip_python = {
        "source": source_label,
        "role": role,
        "client": client,
        "server": server,
        "packet": packet,
        "ack": ack,
        "status": status,
        "value_bars": value_bars,
        "checksum_bar": checksum_bar,
        "ack_bar": ack_bar,
        "mismatch": mismatch,
    }
    update_tcpip_python_visuals(system)
    return system, system._tcpip_python


def update_tcpip_python_visuals(system):
    items = system._tcpip_python
    info = tcp_packet_info(system.GetChTime())
    phase = info["phase"]
    client_port = np.array([-1.02, 0.0, 0.34])
    server_port = np.array([1.02, 0.0, 0.34])

    if phase < 0.55:
        packet_pos = lerp(client_port, server_port, smooth(phase / 0.55))
        ack_pos = server_port + np.array([0.0, 0.18, 0.03])
    else:
        packet_pos = server_port + np.array([0.0, -0.18, 0.03])
        ack_pos = lerp(server_port, client_port, smooth((phase - 0.55) / 0.45))
    set_marker_pos(items["packet"], packet_pos)
    set_marker_pos(items["ack"], ack_pos)

    values = info["values"]
    scales = (0.09, 0.035, 0.10, 0.055, 0.11)
    for index, (bar, value, scale) in enumerate(zip(items["value_bars"], values, scales)):
        base = np.array([-1.82 + 0.16 * index, -0.58, 0.025])
        height = clamp(float(value) * scale, -0.42, 0.42)
        bar.update(base, base + np.array([0.0, 0.0, height]))

    checksum_height = 0.12 + 0.34 * ((info["checksum"] % 512) / 512.0)
    ack_height = 0.12 + 0.34 * ((info["server_ack"] % 512) / 512.0)
    checksum_base = np.array([0.85, -0.60, 0.025])
    ack_base = np.array([1.05, -0.60, 0.025])
    items["checksum_bar"].update(checksum_base, checksum_base + np.array([0.0, 0.0, checksum_height]))
    items["ack_bar"].update(ack_base, ack_base + np.array([0.0, 0.0, ack_height]))

    status_color = color(0.08, 0.70, 0.22) if info["ack_ok"] else color(0.90, 0.08, 0.05)
    set_body_color(items["status"], status_color)
    set_marker_pos(items["status"], (1.45, -0.44, 0.11))
    if info["ack_ok"]:
        items["mismatch"].update((1.45, -0.44, 0.03), (1.45, -0.44, 0.03))
    else:
        items["mismatch"].update((1.34, -0.49, 0.03), (1.56, -0.39, 0.23))

    items["last"] = info


def print_tcpip_python_state(system):
    items = system._tcpip_python
    data = items["last"]
    print(
        f"t={system.GetChTime():.4f} source={items['source']} role={items['role']} "
        f"host={TCP_HOST}:{TCP_PORT} packer='I d d d d' packetSize={data['packet_size']} ackSize={data['ack_size']} "
        f"values=({data['values'][0]},{data['values'][1]:.1f},{data['values'][2]:+.4f},{data['values'][3]:+.1f},{data['values'][4]:+.4f}) "
        f"checksum={data['checksum']} serverAck=int(checksum+t)={data['server_ack']} "
        f"clientExpected={data['client_expected']} ackOK={data['ack_ok']}"
    )


def main_tcpip_python(build_system, title, intro, duration=2.0):
    generic_main(
        build_system,
        update_tcpip_python_visuals,
        print_tcpip_python_state,
        title,
        duration,
        0.001,
        (2.2, -3.2, 1.8),
        (0.0, 0.0, 0.35),
        intro,
    )


def matlab_state(time):
    sample_time = math.floor(max(time, 0.0) / MATLAB_SAMPLE_TIME + 1.0e-12) * MATLAB_SAMPLE_TIME
    phi0 = 0.34 * math.sin(1.20 * time + 0.15)
    phi0_t = 0.34 * 1.20 * math.cos(1.20 * time + 0.15)
    relative = 0.46 * math.sin(0.95 * time + 0.50)
    relative_t = 0.46 * 0.95 * math.cos(0.95 * time + 0.50)
    phi1 = phi0 + relative
    phi1_t = phi0_t + relative_t
    received_y0 = 0.25 * math.sin(0.75 * sample_time)
    tau = 4.5 * math.sin(1.55 * sample_time) - 1.2 * phi0_t
    send = np.array([sample_time, phi0, phi0_t], dtype=float)
    receive = np.array([received_y0, tau], dtype=float)
    return {
        "sample_time": sample_time,
        "phi0": phi0,
        "phi0_t": phi0_t,
        "phi1": phi1,
        "phi1_t": phi1_t,
        "send": send,
        "receive": receive,
        "tau": tau,
        "relative_rate": relative_t,
    }


def link_points(phi0, phi1):
    pivot = np.array([0.0, 0.0, 0.13])
    dir0 = np.array([math.cos(phi0), 0.0, math.sin(phi0)])
    end0 = pivot + MATLAB_L * dir0
    dir1 = np.array([math.cos(phi1), 0.0, math.sin(phi1)])
    tip = end0 + MATLAB_L * dir1
    return pivot, end0, tip, dir0, dir1


def spiral_points(center, radius0, radius1, turns, sign, phase=0.0, y=-0.09, count=96):
    points = []
    sign = 1.0 if sign >= 0.0 else -1.0
    for i in range(count):
        u = i / (count - 1)
        radius = radius0 + (radius1 - radius0) * u
        angle = phase + sign * turns * 2.0 * math.pi * u
        points.append(np.asarray(center, dtype=float) + np.array([radius * math.cos(angle), y, radius * math.sin(angle)]))
    return points


def build_tcpip_matlab_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, "TCPIP MATLAB double-pendulum checkerboard", (0.55, 0.0), (5.0, 1.9), z=0.0, tile_count=10)

    link0 = make_box(system, "TCPIP MATLAB RigidBody2D steelblue brick 0", (MATLAB_L, 0.05, 0.05), color(0.18, 0.46, 0.74), (0.5, 0, 0.13), 0.90)
    link1 = make_box(system, "TCPIP MATLAB RigidBody2D steelblue brick 1", (MATLAB_L, 0.05, 0.05), color(0.18, 0.46, 0.74), (1.5, 0, 0.13), 0.90)
    ground_pivot = make_marker(system, "TCPIP MATLAB ground revolute marker", 0.045, color(0.04, 0.04, 0.045))
    middle_joint = make_marker(system, "TCPIP MATLAB inter-body revolute marker", 0.040, color(0.04, 0.04, 0.045))
    tip = make_marker(system, "TCPIP MATLAB second-link tip marker", 0.034, color(0.95, 0.72, 0.10))
    com0 = make_marker(system, "TCPIP MATLAB sensor body0 COM marker", 0.026, color(0.95, 0.52, 0.06))
    com1 = make_marker(system, "TCPIP MATLAB body1 COM marker", 0.026, color(0.95, 0.52, 0.06))

    gravity0 = MutableSegment(system, "TCPIP MATLAB rigid0 gravity load cue", color(0.08, 0.28, 0.92), 4)
    gravity1 = MutableSegment(system, "TCPIP MATLAB rigid1 gravity load cue", color(0.08, 0.28, 0.92), 4)
    torque_arc = MutablePolyline(system, "TCPIP MATLAB received torque load arc", color(0.92, 0.50, 0.06), 4)
    damper_coil = MutablePolyline(system, "TCPIP MATLAB CoordinateSpringDamper damping spiral", color(0.05, 0.05, 0.055), 3)
    send_bars = [
        MutableSegment(system, f"TCPIP MATLAB send vector bar {name}", color(0.95, 0.56, 0.06), 4)
        for name in ("t", "phi", "phi_t")
    ]
    receive_bars = [
        MutableSegment(system, f"TCPIP MATLAB receive vector bar y{index}", color(0.05, 0.42, 0.95), 5)
        for index in range(2)
    ]
    matlab_box = make_box(system, "TCPIP MATLAB Simulink client block", (0.50, 0.28, 0.22), color(0.12, 0.62, 0.26), (2.55, 0.0, 0.17), 0.82)
    python_box = make_box(system, "TCPIP MATLAB Exudyn Python process block", (0.50, 0.28, 0.22), color(0.08, 0.36, 0.86), (2.55, -0.52, 0.17), 0.82)
    tcp_line = MutableSegment(system, "TCPIP MATLAB big-endian send/receive channel", color(0.06, 0.06, 0.07), 5)
    tcp_line.update((2.55, -0.36, 0.27), (2.55, -0.16, 0.27))

    system._tcpip_matlab = {
        "link0": link0,
        "link1": link1,
        "ground_pivot": ground_pivot,
        "middle_joint": middle_joint,
        "tip": tip,
        "com0": com0,
        "com1": com1,
        "gravity0": gravity0,
        "gravity1": gravity1,
        "torque_arc": torque_arc,
        "damper_coil": damper_coil,
        "send_bars": send_bars,
        "receive_bars": receive_bars,
        "matlab_box": matlab_box,
        "python_box": python_box,
    }
    update_tcpip_matlab_visuals(system)
    return system, system._tcpip_matlab


def update_tcpip_matlab_visuals(system):
    items = system._tcpip_matlab
    data = matlab_state(system.GetChTime())
    pivot, end0, tip, dir0, dir1 = link_points(data["phi0"], data["phi1"])
    center0 = 0.5 * (pivot + end0)
    center1 = 0.5 * (end0 + tip)

    items["link0"].SetPos(vec(center0))
    items["link0"].SetRot(chrono.QuatFromAngleY(-data["phi0"]))
    items["link0"].UpdateVisualModel()
    items["link1"].SetPos(vec(center1))
    items["link1"].SetRot(chrono.QuatFromAngleY(-data["phi1"]))
    items["link1"].UpdateVisualModel()

    for key, point in (("ground_pivot", pivot), ("middle_joint", end0), ("tip", tip), ("com0", center0), ("com1", center1)):
        set_marker_pos(items[key], point)

    gravity_len = 0.26
    items["gravity0"].update(center0, center0 + np.array([0.0, 0.0, -gravity_len]))
    items["gravity1"].update(center1, center1 + np.array([0.0, 0.0, -gravity_len]))

    torque_sign = 1.0 if data["tau"] >= 0.0 else -1.0
    torque_radius = 0.24 + 0.035 * clamp(abs(data["tau"]) / 5.0, 0.0, 1.0)
    items["torque_arc"].update(spiral_points(center0 + np.array([0.0, 0.10, 0.0]), torque_radius, torque_radius, 0.72, torque_sign, data["phi0"], y=0.0, count=72))
    items["damper_coil"].update(spiral_points(end0, 0.055, 0.17, 2.8, data["relative_rate"], data["phi0"] - data["phi1"], y=-0.10, count=112))

    send_scales = (0.08, 0.65, 0.38)
    for index, (bar, value, scale) in enumerate(zip(items["send_bars"], data["send"], send_scales)):
        base = np.array([2.18 + 0.13 * index, -0.72, 0.030])
        height = clamp(float(value) * scale, -0.36, 0.36)
        bar.update(base, base + np.array([0.0, 0.0, height]))
    receive_scales = (0.80, 0.08)
    for index, (bar, value, scale) in enumerate(zip(items["receive_bars"], data["receive"], receive_scales)):
        base = np.array([2.66 + 0.15 * index, -0.72, 0.030])
        height = clamp(float(value) * scale, -0.38, 0.38)
        bar.update(base, base + np.array([0.0, 0.0, height]))

    items["last"] = data
    items["last"].update(
        {
            "pivot": pivot,
            "middle_joint": end0,
            "tip_point": tip,
            "mass": MATLAB_MASS,
            "inertia": MATLAB_INERTIA,
            "gravity": MATLAB_GRAVITY,
            "damping": MATLAB_DAMPING,
        }
    )


def print_tcpip_matlab_state(system):
    data = system._tcpip_matlab["last"]
    send = data["send"]
    receive = data["receive"]
    print(
        f"t={system.GetChTime():.4f} source=TCPIPexudynMatlab.py "
        f"sendSize=3 receiveSize=2 bigEndian=True sampleTime={MATLAB_SAMPLE_TIME:.3f} h={MATLAB_STEP:.3f} "
        f"L={MATLAB_L:.1f} mass={MATLAB_MASS:.1f} inertia={MATLAB_INERTIA:.6f} damping={MATLAB_DAMPING:.1f} "
        f"send=[{send[0]:.3f},{send[1]:+.6f},{send[2]:+.6f}] "
        f"recv=[{receive[0]:+.6f},{receive[1]:+.6f}] tau={data['tau']:+.6f} "
        f"phi1={data['phi1']:+.6f} coordinateDamperVisual=spiral"
    )


def main_tcpip_matlab(build_system, title, intro, duration=1.5):
    generic_main(
        build_system,
        update_tcpip_matlab_visuals,
        print_tcpip_matlab_state,
        title,
        duration,
        MATLAB_STEP,
        (2.2, -3.1, 1.9),
        (0.75, 0.0, 0.35),
        intro,
    )

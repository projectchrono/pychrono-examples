import math

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


def coil_between(start, end, radius=0.025, turns=6.0, count=80):
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


def make_bar_set(system, prefix, count, tint=None):
    tint = tint or color(0.10, 0.56, 0.26)
    return [MutableSegment(system, f"{prefix} result bar {i:03d}", tint, 4) for i in range(count)]


def update_bars(bars, values, origin, spacing=0.10, height=0.36, axis="z"):
    values = np.asarray(values, dtype=float)
    max_abs = max(1.0e-12, float(np.max(np.abs(values))))
    origin = np.asarray(origin, dtype=float)
    for i, (bar, value) in enumerate(zip(bars, values)):
        base = origin + np.array([spacing * i, 0.0, 0.0])
        scale = clamp(value / max_abs, -1.0, 1.0) * height
        delta = np.array([0.0, 0.0, scale]) if axis == "z" else np.array([0.0, scale, 0.0])
        bar.update(base, base + delta)


def make_arrow(system, name, tint, thickness=5):
    return {
        "shaft": MutableSegment(system, name + " shaft", tint, thickness),
        "head_a": MutableSegment(system, name + " head a", tint, max(3, thickness - 1)),
        "head_b": MutableSegment(system, name + " head b", tint, max(3, thickness - 1)),
    }


def update_arrow(arrow, start, end, head_length=0.06):
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


def damped_mass_spring_force(time, mass, stiffness, damping, u0=-0.08, v0=1.0, force=80.0):
    omega0 = math.sqrt(stiffness / mass)
    zeta = damping / (2.0 * math.sqrt(stiffness * mass))
    static = force / stiffness
    if zeta < 1.0:
        omega = omega0 * math.sqrt(1.0 - zeta * zeta)
        c1 = u0 - static
        c2 = (v0 + zeta * omega0 * c1) / omega
        exp_term = math.exp(-zeta * omega0 * time)
        disp = static + exp_term * (c1 * math.cos(omega * time) + c2 * math.sin(omega * time))
        vel = exp_term * (
            (-zeta * omega0 * c1 + omega * c2) * math.cos(omega * time)
            + (-zeta * omega0 * c2 - omega * c1) * math.sin(omega * time)
        )
    else:
        disp = static
        vel = 0.0
    spring_force = stiffness * disp + damping * vel
    return disp, vel, spring_force


def build_dispy_parameter_variation_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, "dispy parameter variation grid", (0.62, 0.0), (2.4, 1.5), z=-0.05, tile_count=8)
    host = make_box(system, "dispy local cluster host", (0.36, 0.20, 0.18), color(0.12, 0.58, 0.28), (-0.35, 0.42, 0.10), 0.88)
    reference = make_box(system, "dispy reference solution mass-spring", (0.28, 0.16, 0.16), color(0.08, 0.34, 0.86), (-0.35, -0.42, 0.10), 0.88)
    task_boxes = []
    springs = []
    bars = make_bar_set(system, "dispy parameter variation", 4, color(0.92, 0.50, 0.06))
    for j, mass in enumerate((1.0, 2.0)):
        for i, stiffness in enumerate((2000.0, 8000.0)):
            index = 2 * j + i
            pos = np.array([0.35 + 0.42 * i, -0.28 + 0.56 * j, 0.09])
            task_boxes.append(make_box(system, f"dispy task mass={mass:.1f} spring={stiffness:.0f}", (0.22, 0.14, 0.14), color(0.18, 0.46, 0.74), pos, 0.86))
            springs.append(MutablePolyline(system, f"dispy visible spring result {index}", color(0.05, 0.05, 0.055), 3))
    system._dispy_parameter_variation = {"host": host, "reference": reference, "task_boxes": task_boxes, "springs": springs, "bars": bars}
    update_dispy_parameter_variation_visuals(system)
    return system, system._dispy_parameter_variation


def update_dispy_parameter_variation_visuals(system):
    items = system._dispy_parameter_variation
    t = system.GetChTime()
    masses = (1.0, 2.0)
    springs_k = (2000.0, 8000.0)
    ref = damped_mass_spring_force(1.0, 1.6, 4000.0, 8.0)[2]
    errors = []
    for j, mass in enumerate(masses):
        for i, stiffness in enumerate(springs_k):
            index = 2 * j + i
            disp, _vel, force = damped_mass_spring_force(1.0, mass, stiffness, 8.0)
            errors.append(abs(ref - force))
            base = np.array([0.12 + 0.42 * i, -0.28 + 0.56 * j, 0.09])
            box_pos = np.array([0.35 + 0.42 * i + 0.04 * math.sin(3.0 * t + index), -0.28 + 0.56 * j, 0.09])
            set_pose(items["task_boxes"][index], box_pos)
            items["springs"][index].update(coil_between(base, box_pos, radius=0.018, turns=4.5, count=64))
    update_bars(items["bars"], errors, (-0.02, -0.72, 0.0), spacing=0.12, height=0.32)
    items["last"] = {"errors": np.asarray(errors), "ref_force": ref}


def print_dispy_parameter_variation_state(system):
    data = system._dispy_parameter_variation["last"]
    print(
        f"t={system.GetChTime():.4f} source=dispyParameterVariationExample.py ParameterVariation=dispy/local-host "
        f"parameters=mass(1,2,2)xspring(2000,8000,2) variations=4 default=(mass=1.6,spring=4000,damper=8,u0=-0.08,v0=1,f=80) "
        f"refForce={data['ref_force']:+.6f} errorNorms=({data['errors'][0]:.6f},{data['errors'][1]:.6f},{data['errors'][2]:.6f},{data['errors'][3]:.6f})"
    )


def main_dispy_parameter_variation(build_system, title, intro, duration=1.2):
    generic_main(build_system, update_dispy_parameter_variation_visuals, print_dispy_parameter_variation_state, title, duration, 0.001, (1.8, -2.6, 1.55), (0.45, 0.0, 0.12), intro)


def build_mpi4py_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, "mpi4py parameter variation grid", (0.8, 0.0), (2.8, 1.6), z=-0.04, tile_count=8)
    main_rank = make_box(system, "mpi4py main rank", (0.28, 0.18, 0.18), color(0.08, 0.34, 0.86), (-0.48, 0.0, 0.10), 0.90)
    workers = [make_box(system, f"mpi4py worker rank {i + 1}", (0.18, 0.14, 0.14), color(0.12, 0.58, 0.28), (0.05 + 0.20 * i, 0.42, 0.09), 0.86) for i in range(8)]
    task_stream = MutablePolyline(system, "mpi4py 640-task parameter stream", color(0.92, 0.50, 0.06), 4)
    result_bars = make_bar_set(system, "mpi4py chunk", 8, color(0.92, 0.50, 0.06))
    system._mpi4py = {"main_rank": main_rank, "workers": workers, "task_stream": task_stream, "result_bars": result_bars}
    update_mpi4py_visuals(system)
    return system, system._mpi4py


def mpi_results():
    stiffness = np.linspace(1000.0, 2000.0, 640)
    result = (1.0 - 0.5) ** 2 + (stiffness - 0.2) ** 2 * 100.0
    return stiffness, result


def update_mpi4py_visuals(system):
    items = system._mpi4py
    t = system.GetChTime()
    points = []
    for i in range(9):
        phase = (t * 0.9 + i / 8.0) % 1.0
        points.append(np.array([-0.30 + phase * 1.65, 0.18 + 0.10 * math.sin(2.0 * math.pi * phase), 0.22]))
    items["task_stream"].update(points)
    _stiffness, result = mpi_results()
    chunks = np.array_split(result, 8)
    sums = np.array([chunk.sum() for chunk in chunks])
    update_bars(items["result_bars"], sums, (0.02, -0.52, 0.0), spacing=0.16, height=0.34)
    items["last"] = {"sum": float(result.sum()), "min": float(result.min()), "max": float(result.max()), "chunk_sums": sums}


def print_mpi4py_state(system):
    data = system._mpi4py["last"]
    print(
        f"t={system.GetChTime():.4f} source=mpi4pyExample.py useMPI=True command='mpiexec -n 9 python3 -m mpi4py.futures mpi4pyExample.py' "
        f"workers=8 variations=640 mass=1 stiffnessRange=(1000,2000) tEnd=100 h=0.001 "
        f"resultMin={data['min']:.6f} resultMax={data['max']:.6f} sumTEnd100={data['sum']:.6f} "
        f"sourceCommentSum=14931163024.242020"
    )


def main_mpi4py(build_system, title, intro, duration=1.4):
    generic_main(build_system, update_mpi4py_visuals, print_mpi4py_state, title, duration, 0.001, (1.9, -2.7, 1.55), (0.60, 0.0, 0.10), intro)


def build_multiprocessing_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, "multiprocessing free-mass grid", (0.85, 0.0), (3.0, 1.4), z=-0.04, tile_count=8)
    workers = [make_box(system, f"multiprocessing Pool worker {i + 1}", (0.14, 0.12, 0.12), color(0.12, 0.58, 0.28), (-0.45 + 0.18 * i, 0.42, 0.08), 0.86) for i in range(20)]
    bodies = [make_marker(system, f"multiprocessing free Mass1D result {i + 1}", 0.040, color(0.18, 0.46, 0.74)) for i in range(20)]
    bars = make_bar_set(system, "multiprocessing x", 20, color(0.92, 0.50, 0.06))
    system._multiprocessing = {"workers": workers, "bodies": bodies, "bars": bars}
    update_multiprocessing_visuals(system)
    return system, system._multiprocessing


def update_multiprocessing_visuals(system):
    items = system._multiprocessing
    t = system.GetChTime()
    inputs = np.linspace(1.0, 10.0, 20)
    results = 10.0 * inputs
    for i, (body, result) in enumerate(zip(items["bodies"], results)):
        x = -0.35 + 1.70 * (result - results.min()) / (results.max() - results.min())
        y = -0.22 + 0.08 * math.sin(2.0 * t + i)
        set_pose(body, (x, y, 0.08))
    update_bars(items["bars"], results, (-0.55, -0.62, 0.0), spacing=0.075, height=0.34)
    items["last"] = {"inputs": inputs, "results": results}


def print_multiprocessing_state(system):
    data = system._multiprocessing["last"]
    print(
        f"t={system.GetChTime():.4f} source=multiprocessingTest.py PoolTasks=20 vInput=linspace(1,10,20) "
        f"freeMass1D tEnd=10 h=1e-6 graphicsDisabled=True resultFirst={data['results'][0]:.6f} "
        f"resultLast={data['results'][-1]:.6f} resultSum={float(data['results'].sum()):.6f}"
    )


def main_multiprocessing(build_system, title, intro, duration=1.2):
    generic_main(build_system, update_multiprocessing_visuals, print_multiprocessing_state, title, duration, 0.001, (1.8, -2.5, 1.45), (0.50, 0.0, 0.08), intro)


def mouse_chain_points(time, active=True):
    sx = 0.25
    n = 16
    delta = 0.05 * math.pi / n
    points = [np.array([0.0, 0.0, 0.0])]
    yaw = 0.0
    for i in range(n):
        yaw = i * delta + 0.13 * math.sin(0.85 * time + 0.35 * i) * (1.0 if active else 0.25)
        z = -0.035 * i * i / n + 0.05 * math.sin(0.7 * time + i)
        prev = points[-1]
        next_point = prev + np.array([sx * math.cos(yaw), sx * math.sin(yaw), z / n])
        points.append(next_point)
    return points


def build_mouse_interaction_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, -9.81))
    add_grid_ground(system, "mouse interaction chain ground", (1.9, 0.0), (4.4, 1.8), z=-0.40, tile_count=10)
    bodies = [make_box(system, f"mouseInteraction red chain body {i + 1}", (0.25, 0.10, 0.10), color(0.92, 0.08, 0.06), (0, 0, 0), 0.92) for i in range(16)]
    joints = [make_marker(system, f"mouseInteraction revolute joint {i}", 0.030, color(0.04, 0.04, 0.045)) for i in range(17)]
    dampers = [MutablePolyline(system, f"mouseInteraction hidden Cartesian damping coil {i + 1}", color(0.05, 0.05, 0.055), 2) for i in range(0, 16, 3)]
    drag_x = MutablePolyline(system, "mouseInteraction coordinate mouse-drag x coil", color(0.92, 0.50, 0.06), 3)
    drag_z = MutablePolyline(system, "mouseInteraction coordinate mouse-drag z coil", color(0.10, 0.36, 0.92), 3)
    key_bar = MutableSegment(system, "mouseInteraction key D activation bar", color(0.10, 0.56, 0.26), 5)
    target = make_marker(system, "mouseInteraction OpenGL mouse target", 0.045, color(0.95, 0.72, 0.08))
    system._mouse_interaction = {"bodies": bodies, "joints": joints, "dampers": dampers, "drag_x": drag_x, "drag_z": drag_z, "key_bar": key_bar, "target": target}
    update_mouse_interaction_visuals(system)
    return system, system._mouse_interaction


def update_mouse_interaction_visuals(system):
    items = system._mouse_interaction
    t = system.GetChTime()
    active = (math.sin(0.65 * t) >= -0.45)
    points = mouse_chain_points(t, active)
    for i, body in enumerate(items["bodies"]):
        center = 0.5 * (points[i] + points[i + 1])
        direction = points[i + 1] - points[i]
        yaw = math.atan2(direction[1], direction[0])
        set_pose(body, center, chrono.QuatFromAngleZ(yaw))
    for joint, point in zip(items["joints"], points):
        set_pose(joint, point)
    for damper_index, coil in enumerate(items["dampers"]):
        point = points[min(16, 1 + 3 * damper_index)]
        coil.update(coil_between((0, 0, 0), point, radius=0.010, turns=4.0, count=56))
    tip = points[-1]
    target = np.array([3.5 + 0.25 * math.sin(0.7 * t), -0.42 + 0.10 * math.cos(0.9 * t), 0.10 + 0.10 * math.sin(0.5 * t)])
    set_pose(items["target"], target)
    items["drag_x"].update(coil_between(tip, np.array([target[0], tip[1], tip[2]]), radius=0.016, turns=5.0, count=72))
    items["drag_z"].update(coil_between(tip, np.array([tip[0], tip[1], target[2]]), radius=0.014, turns=5.0, count=72))
    items["key_bar"].update((-0.55, -0.72, -0.36), (-0.55, -0.72, -0.36 + (0.32 if active else 0.05)))
    items["last"] = {"active": active, "tip": tip, "target": target, "n_bodies": 16}


def print_mouse_interaction_state(system):
    data = system._mouse_interaction["last"]
    print(
        f"t={system.GetChTime():.4f} source=mouseInteractionExample.py nBodies=16 sx=0.25 sy=0.1 sz=0.1 "
        f"activateWithKeyPress=True key=D active={data['active']} mouseTarget=({data['target'][0]:+.5f},{data['target'][1]:+.5f},{data['target'][2]:+.5f}) "
        f"tip=({data['tip'][0]:+.5f},{data['tip'][1]:+.5f},{data['tip'][2]:+.5f}) hiddenCoordinateSprings=visibleCoils"
    )


def main_mouse_interaction(build_system, title, intro, duration=1.6):
    generic_main(build_system, update_mouse_interaction_visuals, print_mouse_interaction_state, title, duration, 0.001, (3.2, -3.2, 1.9), (1.8, 0.0, -0.10), intro)


ENGINE_CRANK = 0.04
ENGINE_CONROD = 0.10
ENGINE_PISTON_LEN = 0.05
ENGINE_PISTON_RADIUS = 0.02
ENGINE_DISTANCE = 0.012 + 2.0 * 0.01 + 0.024
ENGINE_CRANK_DEG = (0.0, 90.0, 270.0, 180.0)
ENGINE_PISTON_DEG = (90.0, 90.0, 90.0, 90.0)
ENGINE_OMEGA = 4.0 * math.pi * 0.5


def slider_crank(angle_crank, angle_piston):
    phi1 = angle_crank - angle_piston
    h = ENGINE_CRANK * math.sin(phi1)
    phi2 = math.asin(clamp(h / ENGINE_CONROD, -1.0, 1.0))
    angle_conrod = angle_piston - phi2
    dp = ENGINE_CRANK * math.cos(phi1) + ENGINE_CONROD * math.cos(phi2)
    crank_pin = np.array([ENGINE_CRANK * math.cos(angle_crank), ENGINE_CRANK * math.sin(angle_crank), 0.0])
    piston = np.array([dp * math.cos(angle_piston), dp * math.sin(angle_piston), 0.0])
    return crank_pin, piston, angle_conrod


def build_openvr_engine_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, "openVR engine floor", (0.0, 0.0), (2.6, 1.6), z=-0.60, tile_count=12)
    make_box(system, "openVR engine left VR wall", (0.025, 1.6, 1.3), color(0.74, 0.74, 0.74), (-1.20, 0.0, 0.02), 0.22)
    make_box(system, "openVR engine right VR wall", (0.025, 1.6, 1.3), color(0.74, 0.74, 0.74), (1.20, 0.0, 0.02), 0.22)
    make_box(system, "openVR engine rear blue wall", (2.4, 0.025, 1.3), color(0.20, 0.45, 0.92), (0.0, -0.80, 0.02), 0.20)
    crank_axis = MutableSegment(system, "openVR engine crank shaft body", color(0.46, 0.46, 0.48), 9)
    crank_arms = [MutableSegment(system, f"openVR engine crank arm {i + 1}", color(0.42, 0.42, 0.44), 6) for i in range(4)]
    conrods = [MutableSegment(system, f"openVR engine dodgerblue conrod {i + 1}", color(0.12, 0.48, 0.92), 7) for i in range(4)]
    pistons = [make_box(system, f"openVR engine piston body {i + 1}", (ENGINE_PISTON_LEN, 0.040, 0.040), color(0.58, 0.34, 0.20), (0, 0, 0), 0.70) for i in range(4)]
    joints = [make_marker(system, f"openVR engine revolute/prismatic marker {i + 1}", 0.018, color(0.04, 0.04, 0.045)) for i in range(8)]
    drive_bar = MutableSegment(system, "openVR engine CoordinateConstraint drive bar", color(0.92, 0.50, 0.06), 5)
    vr_bar = MutableSegment(system, "openVR engine OpenVR enable flag bar", color(0.10, 0.56, 0.26), 5)
    system._openvr_engine = {"crank_axis": crank_axis, "crank_arms": crank_arms, "conrods": conrods, "pistons": pistons, "joints": joints, "drive_bar": drive_bar, "vr_bar": vr_bar}
    update_openvr_engine_visuals(system)
    return system, system._openvr_engine


def update_openvr_engine_visuals(system):
    items = system._openvr_engine
    t = system.GetChTime()
    driven = t < 1.0
    angle_base = ENGINE_OMEGA * min(t, 1.0) + (0.55 * ENGINE_OMEGA * (t - 1.0) if t >= 1.0 else 0.0)
    e_length = ENGINE_DISTANCE * 4 + 0.012
    items["crank_axis"].update((0, 0, -0.5 * e_length), (0, 0, 0.5 * e_length))
    joint_points = []
    for i, (cdeg, pdeg) in enumerate(zip(ENGINE_CRANK_DEG, ENGINE_PISTON_DEG)):
        z = -0.5 * e_length + i * ENGINE_DISTANCE + 0.012
        angle_crank = math.radians(cdeg) + angle_base
        angle_piston = math.radians(pdeg)
        crank_pin, piston_xy, conrod_angle = slider_crank(angle_crank, angle_piston)
        crank_pin = crank_pin + np.array([0.0, 0.0, z])
        piston = piston_xy + np.array([0.0, 0.0, z])
        items["crank_arms"][i].update((0, 0, z), crank_pin)
        items["conrods"][i].update(crank_pin, piston)
        set_pose(items["pistons"][i], piston, chrono.QuatFromAngleZ(angle_piston))
        joint_points.extend([crank_pin, piston])
    for marker, point in zip(items["joints"], joint_points):
        set_pose(marker, point)
    items["drive_bar"].update((-0.45, 0.62, -0.54), (-0.45, 0.62, -0.54 + (0.36 if driven else 0.08)))
    items["vr_bar"].update((-0.32, 0.62, -0.54), (-0.32, 0.62, -0.54 + 0.08))
    items["last"] = {"angle": angle_base, "driven": driven, "omega": ENGINE_OMEGA if driven else 0.55 * ENGINE_OMEGA, "e_length": e_length}


def print_openvr_engine_state(system):
    data = system._openvr_engine["last"]
    print(
        f"t={system.GetChTime():.4f} source=openVRengine.py pistons=4 crankAngles={ENGINE_CRANK_DEG} pistonAngles={ENGINE_PISTON_DEG} "
        f"omegaDrive={ENGINE_OMEGA:.6f} fixedSpeed=False driveActive={data['driven']} useOpenVR=False window=(1176,1320) "
        f"engineLength={data['e_length']:.6f} crankAngle={data['angle']:+.6f}"
    )


def main_openvr_engine(build_system, title, intro, duration=1.4):
    generic_main(build_system, update_openvr_engine_visuals, print_openvr_engine_state, title, duration, 0.002, (0.30, -0.55, 0.35), (0.0, 0.0, -0.02), intro)

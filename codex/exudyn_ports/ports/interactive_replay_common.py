import math

import numpy as np
import pychrono.core as chrono

from robotics_replay_common import (
    MutablePolyline,
    MutableSegment,
    PUMA_LINKS,
    add_grid_ground,
    color,
    generic_main,
    ht_identity,
    make_box,
    make_marker,
    make_serial_arm,
    serial_fk,
    update_serial_arm,
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


def coil_between(start, end, radius=0.025, turns=8.0, count=96):
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


def spiral_in_frame(frame, value, radius0=0.035, radius1=0.13, turns=2.5, count=80):
    origin = frame[:3, 3]
    x_axis = frame[:3, 0]
    y_axis = frame[:3, 1]
    z_axis = frame[:3, 2]
    sign = 1.0 if value >= 0.0 else -1.0
    points = []
    for i in range(count):
        s = i / (count - 1)
        radius = radius0 + (radius1 - radius0) * s
        angle = sign * 2.0 * math.pi * turns * s + float(value)
        points.append(origin + radius * math.cos(angle) * x_axis + radius * math.sin(angle) * y_axis + 0.018 * (s - 0.5) * z_axis)
    return points


def make_dialog_bars(system, prefix, names, base=(-1.0, -0.8, 0.03), spacing=0.12, height=0.24):
    return {
        "names": tuple(names),
        "base": np.asarray(base, dtype=float),
        "spacing": spacing,
        "height": height,
        "bars": [
            MutableSegment(system, f"{prefix} interactive dialog bar {name}", color(0.10, 0.56, 0.26), 4)
            for name in names
        ],
    }


def update_dialog_bars(dialog, values):
    for i, (bar, value) in enumerate(zip(dialog["bars"], values)):
        base = dialog["base"] + np.array([dialog["spacing"] * i, 0.0, 0.0])
        top = base + np.array([0.0, 0.0, dialog["height"] * clamp(value, 0.0, 1.0)])
        bar.update(base, top)


def make_arrow(system, name, tint, thickness=5):
    return {
        "shaft": MutableSegment(system, name + " shaft", tint, thickness),
        "head_a": MutableSegment(system, name + " head a", tint, max(3, thickness - 1)),
        "head_b": MutableSegment(system, name + " head b", tint, max(3, thickness - 1)),
    }


def update_arrow(arrow, start, end, head_length=0.055):
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


def oscillator_displacement(time, frequency, amplitude=0.055, phase=0.0, nonlinear=False):
    value = amplitude * math.sin(2.0 * math.pi * frequency * time + phase)
    if nonlinear:
        value += 0.22 * amplitude * math.sin(6.0 * math.pi * frequency * time + phase)
    return value


def build_interactive_tutorial_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, -1, 0))
    add_grid_ground(system, "interactiveTutorial checkerboard", (0.50, 0.0), (1.6, 1.0), z=-0.05, tile_count=8)
    node0 = make_marker(system, "interactiveTutorial NodePoint 0 visible mass", 0.065, color(0.10, 0.36, 0.90))
    node1 = make_marker(system, "interactiveTutorial NodePoint 1 visible mass", 0.065, color(0.92, 0.50, 0.06))
    force_arrow = make_arrow(system, "interactiveTutorial LoadForceVector", color(0.88, 0.08, 0.08), 5)
    dialog = make_dialog_bars(system, "interactiveTutorial", ("interactiveMode", "nodes", "objects", "markers", "loads"), base=(-0.22, -0.54, -0.04), spacing=0.12)
    system._interactive_tutorial = {"node0": node0, "node1": node1, "force_arrow": force_arrow, "dialog": dialog}
    update_interactive_tutorial_visuals(system)
    return system, system._interactive_tutorial


def update_interactive_tutorial_visuals(system):
    items = system._interactive_tutorial
    t = system.GetChTime()
    p0 = np.array([0.0, -0.5 * t * t, 0.05])
    p1 = np.array([1.0, 0.0, 0.05])
    set_pose(items["node0"], p0)
    set_pose(items["node1"], p1)
    update_arrow(items["force_arrow"], p0 + np.array([0.0, 0.15, 0.0]), p0 + np.array([0.0, -0.15, 0.0]))
    update_dialog_bars(items["dialog"], (1.0, 1.0, 1.0, 0.5, 0.5 + 0.5 * smoothstep(min(t / 0.5, 1.0))))
    items["last"] = {"p0": p0, "p1": p1, "load": np.array([0.0, -1.0, 0.0])}


def print_interactive_tutorial_state(system):
    data = system._interactive_tutorial["last"]
    print(
        f"t={system.GetChTime():.4f} source=interactiveTutorial.py interactiveMode=True "
        f"nodes=2 objects=2 marker=MarkerNodePosition load=LoadForceVector(0,-1,0) "
        f"node0=({data['p0'][0]:+.5f},{data['p0'][1]:+.5f},{data['p0'][2]:+.5f}) nodeSize=0.1"
    )


def main_interactive_tutorial(build_system, title, intro, duration=1.0):
    generic_main(build_system, update_interactive_tutorial_visuals, print_interactive_tutorial_state, title, duration, 0.001, (1.5, -2.0, 1.15), (0.45, 0.0, 0.0), intro)


def build_simulate_interactively_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, "simulateInteractively oscillator ground", (0.50, 0.0), (1.4, 0.8), z=-0.08, tile_count=8)
    make_marker(system, "simulateInteractively ground coordinate marker", 0.035, color(0.03, 0.03, 0.035)).SetPos(vec(0, 0, 0))
    mass = make_box(system, "simulateInteractively steelblue mass point", (0.10, 0.10, 0.10), color(0.18, 0.46, 0.74), (0.5, 0, 0), 0.92)
    spring = MutablePolyline(system, "simulateInteractively CoordinateSpringDamper coil", color(0.86, 0.16, 0.10), 3)
    load_arrow = make_arrow(system, "simulateInteractively harmonic LoadCoordinate", color(0.92, 0.50, 0.06), 5)
    dialog = make_dialog_bars(system, "simulateInteractively", ("mode", "frequency", "damping", "stiffness"), base=(0.05, -0.45, -0.06), spacing=0.14)
    system._simulate_interactively = {"mass": mass, "spring": spring, "load_arrow": load_arrow, "dialog": dialog}
    update_simulate_interactively_visuals(system)
    return system, system._simulate_interactively


def update_simulate_interactively_visuals(system):
    items = system._simulate_interactively
    t = system.GetChTime()
    mass = 1.6
    stiffness = 4000.0 * (0.75 + 0.25 * math.sin(0.35 * t) ** 2)
    damping = 20.0 + 16.0 * smoothstep(min(t / 2.5, 1.0))
    frequency = math.sqrt(4000.0 / mass) * 0.5 / (2.0 * math.pi) * (0.75 + 0.25 * math.sin(0.45 * t) ** 2)
    nonlinear = math.sin(0.30 * t) > 0.0
    displacement = oscillator_displacement(t, frequency, amplitude=0.075, nonlinear=nonlinear)
    pos = np.array([0.5 + displacement, 0.0, 0.02])
    set_pose(items["mass"], pos)
    items["spring"].update(coil_between((0, 0, 0.02), pos, radius=0.035, turns=8.0, count=112))
    load = 80.0 * math.sin(2.0 * math.pi * frequency * t)
    update_arrow(items["load_arrow"], pos + np.array([0.0, 0.16, 0.0]), pos + np.array([0.22 * load / 80.0, 0.16, 0.0]))
    update_dialog_bars(items["dialog"], (1.0 if nonlinear else 0.30, frequency / 17.5, damping / 40.0, stiffness / 10000.0))
    items["last"] = {"u": displacement, "load": load, "frequency": frequency, "damping": damping, "stiffness": stiffness, "nonlinear": nonlinear}


def print_simulate_interactively_state(system):
    data = system._simulate_interactively["last"]
    print(
        f"t={system.GetChTime():.4f} source=simulateInteractively.py InteractiveDialog=True "
        f"mode={'nonlinear' if data['nonlinear'] else 'linear'} mass=1.6 spring={data['stiffness']:.3f} damping={data['damping']:.3f} "
        f"frequencyHz={data['frequency']:.6f} load0=80 u={data['u']:+.6f} load={data['load']:+.6f}"
    )


def main_simulate_interactively(build_system, title, intro, duration=2.0):
    generic_main(build_system, update_simulate_interactively_visuals, print_simulate_interactively_state, title, duration, 0.001, (1.05, -1.65, 1.05), (0.48, 0.0, 0.0), intro)


def build_mass_spring_friction_interactive_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, "massSpringFrictionInteractive moving band", (0.30, 0.0), (1.7, 0.70), z=-0.08, tile_count=10)
    band = make_box(system, "massSpringFrictionInteractive moving band visual", (1.35, 0.18, 0.025), color(0.72, 0.72, 0.74), (0.15, 0, -0.02), 0.60)
    mass = make_box(system, "massSpringFrictionInteractive steelblue mass", (0.10, 0.10, 0.10), color(0.18, 0.46, 0.74), (0.5, 0, 0.035), 0.94)
    spring = MutablePolyline(system, "massSpringFrictionInteractive CoordinateSpringDamper coil", color(0.86, 0.16, 0.10), 3)
    friction = make_arrow(system, "massSpringFrictionInteractive Stribeck friction", color(0.88, 0.08, 0.08), 5)
    load_arrow = make_arrow(system, "massSpringFrictionInteractive optional load", color(0.92, 0.50, 0.06), 5)
    dialog = make_dialog_bars(system, "massSpringFrictionInteractive", ("bandVelocity", "dynamicFriction", "staticOffset", "stiffness", "relDamping", "addForce"), base=(-0.18, -0.48, -0.06), spacing=0.105)
    system._mass_spring_friction_interactive = {"band": band, "mass": mass, "spring": spring, "friction": friction, "load_arrow": load_arrow, "dialog": dialog}
    update_mass_spring_friction_interactive_visuals(system)
    return system, system._mass_spring_friction_interactive


def update_mass_spring_friction_interactive_visuals(system):
    items = system._mass_spring_friction_interactive
    t = system.GetChTime()
    v_band = 4.0 + 3.0 * math.sin(0.35 * t) ** 2
    f_dynamic = 40.0
    f_static_offset = 3.0 + 7.0 * smoothstep(min(t / 2.0, 1.0))
    stiffness = 400.0 + 220.0 * math.sin(0.55 * t) ** 2
    rel_damping = 0.002 + 0.018 * math.sin(0.40 * t) ** 2
    displacement = 0.065 * math.sin(10.0 * t) * math.exp(-0.03 * t)
    velocity = 0.065 * 10.0 * math.cos(10.0 * t) * math.exp(-0.03 * t)
    pos = np.array([0.5 + displacement, 0.0, 0.035])
    set_pose(items["band"], (0.15 + 0.08 * math.sin(v_band * t), 0.0, -0.02))
    set_pose(items["mass"], pos)
    items["spring"].update(coil_between((0, 0, 0.035), pos, radius=0.028, turns=7.0, count=112))
    friction_force = (f_dynamic + f_static_offset * math.exp(-abs(velocity) / 0.1)) * (1.0 if velocity >= 0.0 else -1.0)
    update_arrow(items["friction"], pos + np.array([0.0, -0.15, 0.0]), pos + np.array([-0.22 * friction_force / 50.0, -0.15, 0.0]))
    add_force = 1.0 if math.sin(0.60 * t) > 0 else 0.0
    update_arrow(items["load_arrow"], pos + np.array([0.0, 0.15, 0.0]), pos + np.array([0.20 * add_force, 0.15, 0.0]))
    update_dialog_bars(items["dialog"], (v_band / 20.0, f_dynamic / 100.0, f_static_offset / 100.0, stiffness / 2000.0, rel_damping / 0.4, add_force))
    items["last"] = {"u": displacement, "v": velocity, "v_band": v_band, "friction": friction_force, "stiffness": stiffness, "rel_damping": rel_damping, "add_force": add_force}


def print_mass_spring_friction_interactive_state(system):
    data = system._mass_spring_friction_interactive["last"]
    print(
        f"t={system.GetChTime():.4f} source=massSpringFrictionInteractive.py InteractiveDialog=True "
        f"vBand={data['v_band']:.6f} dynamicFriction=40.0 staticOffset~3..10 stiffness={data['stiffness']:.3f} "
        f"relDamping={data['rel_damping']:.6f} u={data['u']:+.6f} v={data['v']:+.6f} friction={data['friction']:+.6f} addForce={data['add_force']:.0f}"
    )


def main_mass_spring_friction_interactive(build_system, title, intro, duration=2.0):
    generic_main(build_system, update_mass_spring_friction_interactive_visuals, print_mass_spring_friction_interactive_state, title, duration, 0.001, (1.15, -1.75, 1.10), (0.45, 0.0, 0.0), intro)


def n_mass_state(time):
    n = 12
    omega = 3.55 + 12.0 * math.sin(0.2 * time) ** 2
    force_amp = 1.0 + 3.0 * smoothstep(min(time / 2.0, 1.0))
    damping = 2.0 + 4.0 * math.sin(0.3 * time) ** 2
    values = []
    for i in range(n):
        mode_shape = math.sin((i + 1) * math.pi / (n + 1))
        values.append(0.045 * force_amp * mode_shape * math.sin(omega * time - 0.18 * i) / max(math.sqrt(damping), 1.0))
    return np.asarray(values), omega, force_amp, damping


def build_n_mass_oscillator_interactive_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, "nMassOscillatorInteractive background", (1.35, 0.0), (3.3, 1.2), z=-0.08, tile_count=12)
    masses = []
    springs = []
    for i in range(12):
        tint = color(0.10, 0.62, 0.22) if i == 0 else (color(0.86, 0.16, 0.10) if i == 11 else color(0.18, 0.46, 0.74))
        masses.append(make_marker(system, f"nMassOscillatorInteractive mass {i + 1}", 0.050, tint))
        springs.append(MutablePolyline(system, f"nMassOscillatorInteractive spring-damper coil {i + 1}", color(0.42, 0.42, 0.44), 3))
    load_arrow = make_arrow(system, "nMassOscillatorInteractive harmonic load on last mass", color(0.92, 0.50, 0.06), 5)
    dialog = make_dialog_bars(system, "nMassOscillatorInteractive", ("loadCase", "omega", "reset", "forceAmplitude", "damping", "period"), base=(0.55, -0.68, -0.06), spacing=0.12)
    system._n_mass_oscillator_interactive = {"masses": masses, "springs": springs, "load_arrow": load_arrow, "dialog": dialog}
    update_n_mass_oscillator_interactive_visuals(system)
    return system, system._n_mass_oscillator_interactive


def update_n_mass_oscillator_interactive_visuals(system):
    items = system._n_mass_oscillator_interactive
    t = system.GetChTime()
    displacements, omega, force_amp, damping = n_mass_state(t)
    rest = 0.2
    previous = np.array([0.0, 0.0, 0.02])
    points = []
    for i, displacement in enumerate(displacements):
        point = np.array([(i + 1) * rest + displacement, 0.0, 0.02])
        set_pose(items["masses"][i], point)
        items["springs"][i].update(coil_between(previous, point, radius=0.025, turns=5.0, count=72))
        points.append(point)
        previous = point
    update_arrow(items["load_arrow"], points[-1] + np.array([0.0, 0.12, 0.0]), points[-1] + np.array([0.18 * math.sin(omega * t), 0.12, 0.0]))
    update_dialog_bars(items["dialog"], (0.50 + 0.50 * math.sin(0.5 * t) ** 2, omega / 60.0, 0.0, force_amp / 100.0, damping / 40.0, 0.01 / 0.1))
    items["last"] = {"u0": displacements[0], "uN": displacements[-1], "omega": omega, "force_amp": force_amp, "damping": damping}


def print_n_mass_oscillator_interactive_state(system):
    data = system._n_mass_oscillator_interactive["last"]
    print(
        f"t={system.GetChTime():.4f} source=nMassOscillatorInteractive.py InteractiveDialog=True N=12 spring=800 mass=1 "
        f"omega={data['omega']:.6f} forceAmplitude={data['force_amp']:.6f} damping={data['damping']:.6f} "
        f"u0={data['u0']:+.6f} uN={data['uN']:+.6f} period=0.01"
    )


def main_n_mass_oscillator_interactive(build_system, title, intro, duration=2.0):
    generic_main(build_system, update_n_mass_oscillator_interactive_visuals, print_n_mass_oscillator_interactive_state, title, duration, 0.001, (1.35, -2.35, 1.35), (1.25, 0.0, 0.0), intro)


PUMA_LIMITS = np.array(
    [
        [-0.75 * math.pi, 0.75 * math.pi],
        [0.0, math.pi],
        [-math.pi, 0.4 * math.pi],
        [-0.5 * math.pi, 0.5 * math.pi],
        [-0.5 * math.pi, 0.5 * math.pi],
        [-0.5 * math.pi, 0.5 * math.pi],
    ],
    dtype=float,
)


def serial_interactive_q(time):
    q = []
    qd = []
    for i, (low, high) in enumerate(PUMA_LIMITS):
        center = 0.5 * (low + high)
        amp = 0.43 * (high - low)
        freq = 0.45 + 0.08 * i
        phase = 0.30 * i
        q.append(center + amp * math.sin(freq * time + phase))
        qd.append(amp * freq * math.cos(freq * time + phase))
    control = 1 if time < 1.2 else (0 if time < 2.2 else 2)
    if control == 2:
        q = [value * (1.0 - smoothstep(min((time - 2.2) / 0.5, 1.0))) for value in q]
    return np.asarray(q), np.asarray(qd), control


def build_serial_robot_interactive_limits_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, "serialRobotInteractiveLimits checkerboard", (-0.15, 0.0), (2.4, 2.0), z=-0.70, tile_count=8)
    arm = make_serial_arm(system, "serialRobotInteractiveLimits PUMA", PUMA_LINKS, ht_identity(), (0.0, 0.0, 0.10), base_size=(0.20, 0.20, 0.18), link_thickness=7)
    springs = [MutablePolyline(system, f"serialRobotInteractiveLimits joint {i + 1} torsional spring spiral", color(0.05, 0.05, 0.055), 3) for i in range(6)]
    limit_bars = [MutableSegment(system, f"serialRobotInteractiveLimits joint {i + 1} limit slider", color(0.92, 0.50, 0.06), 4) for i in range(6)]
    drag_coil = MutablePolyline(system, "serialRobotInteractiveLimits hidden mouse-drag CartesianSpringDamper coil", color(0.04, 0.04, 0.045), 3)
    dialog = make_dialog_bars(system, "serialRobotInteractiveLimits", ("q1", "q2", "q3", "q4", "q5", "q6", "control"), base=(-1.15, -0.92, -0.64), spacing=0.105)
    system._serial_robot_interactive_limits = {"arm": arm, "springs": springs, "limit_bars": limit_bars, "drag_coil": drag_coil, "dialog": dialog}
    update_serial_robot_interactive_limits_visuals(system)
    return system, system._serial_robot_interactive_limits


def update_serial_robot_interactive_limits_visuals(system):
    items = system._serial_robot_interactive_limits
    q, qd, control = serial_interactive_q(system.GetChTime())
    update_serial_arm(items["arm"], q)
    joint_frames, _link_frames, _tool = serial_fk(PUMA_LINKS, q, ht_identity(), (0.0, 0.0, 0.10))
    for i, frame in enumerate(joint_frames):
        items["springs"][i].update(spiral_in_frame(frame, qd[i], radius0=0.035, radius1=0.10, turns=2.4, count=76))
        low, high = PUMA_LIMITS[i]
        ratio = (q[i] - low) / (high - low)
        base = np.array([-1.15 + 0.105 * i, -0.72, -0.64])
        items["limit_bars"][i].update(base, base + np.array([0.0, 0.0, 0.26 * clamp(ratio, 0.0, 1.0)]))
    tcp = items["arm"]["last_tcp"]
    drag_target = tcp + np.array([0.15 * math.sin(0.8 * system.GetChTime()), -0.12, 0.08])
    items["drag_coil"].update(coil_between(tcp, drag_target, radius=0.017, turns=6.0, count=84))
    slider_values = [(q[i] - PUMA_LIMITS[i, 0]) / (PUMA_LIMITS[i, 1] - PUMA_LIMITS[i, 0]) for i in range(6)] + [control / 2.0]
    update_dialog_bars(items["dialog"], slider_values)
    items["last"] = {"q": q, "qd": qd, "control": control, "tcp": tcp}


def print_serial_robot_interactive_limits_state(system):
    data = system._serial_robot_interactive_limits["last"]
    control = {0: "Mouse drag", 1: "Control on", 2: "Reset"}[int(data["control"])]
    q = data["q"]
    print(
        f"t={system.GetChTime():.4f} source=serialRobotInteractiveLimits.py InteractiveDialog=True "
        f"PUMA=6 controlActive={data['control']}({control}) q=({q[0]:+.4f},{q[1]:+.4f},{q[2]:+.4f},{q[3]:+.4f},{q[4]:+.4f},{q[5]:+.4f}) "
        f"limits=enabled torsionalSpringVisuals=6 cartesianDragCoil=visible"
    )


def main_serial_robot_interactive_limits(build_system, title, intro, duration=2.4):
    generic_main(build_system, update_serial_robot_interactive_limits_visuals, print_serial_robot_interactive_limits_state, title, duration, 0.001, (1.8, -3.0, 1.7), (-0.20, 0.0, 0.10), intro)

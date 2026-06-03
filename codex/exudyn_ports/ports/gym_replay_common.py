import math
import sys
from pathlib import Path

import numpy as np

sys.path.append(str(Path(__file__).resolve().parent))

from robotics_replay_common import (  # noqa: E402
    MutableSegment,
    add_grid_ground,
    color,
    generic_main,
    make_box,
    make_marker,
    vec,
)


CARTPOLE_LENGTH = 1.0
CARTPOLE_FORCE_MAG = 10.0
CARTPOLE_STEP_UPDATE = 0.02
CARTPOLE_X_THRESHOLD = 2.4
CARTPOLE_THETA_THRESHOLD = 12.0 * 2.0 * math.pi / 360.0

DOUBLE_LENGTH = 1.0
DOUBLE_FORCE_MAG = 10.0
DOUBLE_STEP_UPDATE = 0.02
DOUBLE_X_THRESHOLD = 2.4
DOUBLE_THETA_THRESHOLD = 12.0 * 2.0 * math.pi / 360.0


def cartpole_state(time):
    omega_x = 2.0 * math.pi / 3.6
    omega_theta = 2.0 * math.pi / 1.45
    x = 0.55 * math.sin(omega_x * time)
    x_t = 0.55 * omega_x * math.cos(omega_x * time)
    theta = 0.080 * math.sin(omega_theta * time + 0.35)
    theta_t = 0.080 * omega_theta * math.cos(omega_theta * time + 0.35)
    force = 0.1 * math.cos((time / CARTPOLE_STEP_UPDATE) / 50.0)
    action = 1 if force >= 0.0 else 0
    done = abs(x) > CARTPOLE_X_THRESHOLD or abs(theta) > CARTPOLE_THETA_THRESHOLD
    reward = 1.0 if not done else 0.0
    return np.array([x, x_t, theta, theta_t], dtype=float), force, action, reward, done


def double_pendulum_state(time):
    omega_x = 2.0 * math.pi / 4.0
    omega_a = 2.0 * math.pi / 1.65
    x = 0.48 * math.sin(omega_x * time)
    x_t = 0.48 * omega_x * math.cos(omega_x * time)
    phi1 = 0.074 * math.sin(omega_a * time + 0.2)
    phi1_t = 0.074 * omega_a * math.cos(omega_a * time + 0.2)
    phi2 = -0.060 * math.sin(1.18 * omega_a * time - 0.3)
    phi2_t = -0.060 * 1.18 * omega_a * math.cos(1.18 * omega_a * time - 0.3)
    force = DOUBLE_FORCE_MAG * (0.62 * math.sin(0.85 * time) - 0.18 * math.sin(2.1 * time))
    action = 1 if force >= 0.0 else 0
    done = (
        abs(x) > DOUBLE_X_THRESHOLD
        or abs(phi1) > DOUBLE_THETA_THRESHOLD
        or abs(phi2) > DOUBLE_THETA_THRESHOLD
    )
    reward = 1.0 if not done else 0.0
    return np.array([x, x_t, phi1, phi1_t, phi2, phi2_t], dtype=float), force, action, reward, done


def add_threshold_markers(system, x_threshold, theta_threshold, prefix, z_top=1.22):
    for sign in (-1.0, 1.0):
        x = sign * x_threshold
        MutableSegment(system, f"{prefix} x threshold {sign:+.0f}", color(0.80, 0.08, 0.08), 3).update(
            (x, -0.10, 0.0), (x, -0.10, z_top)
        )
    for sign in (-1.0, 1.0):
        angle = sign * theta_threshold
        end = np.array([-math.sin(angle), -0.16, math.cos(angle)]) * 0.45 + np.array([0.0, 0.0, 0.13])
        MutableSegment(system, f"{prefix} theta threshold {sign:+.0f}", color(0.92, 0.48, 0.08), 3).update(
            (0.0, -0.16, 0.13), end
        )


def build_cartpole_system(source_label, slow_solver=False, driver=False):
    import pychrono.core as chrono

    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, "gym cartpole checkerboard", (0.0, 0.0), (5.4, 1.2), z=0.0, tile_count=12)
    add_threshold_markers(system, CARTPOLE_X_THRESHOLD, CARTPOLE_THETA_THRESHOLD, "gym cartpole", z_top=1.25)

    cart = make_box(system, "gym cartpole blue cart body", (0.50, 0.18, 0.14), color(0.08, 0.42, 0.90), (0, 0, 0.08), 0.92)
    pole = MutableSegment(system, "gym cartpole red pole body", color(0.90, 0.08, 0.06), 14)
    pivot = make_marker(system, "gym cartpole revolute joint marker", 0.045, color(0.05, 0.05, 0.055))
    tip = make_marker(system, "gym cartpole pole tip marker", 0.032, color(0.95, 0.70, 0.10))
    force_arrow = MutableSegment(system, "gym cartpole control force arrow", color(0.05, 0.24, 0.95), 5)
    reward_bar = MutableSegment(system, "gym cartpole reward bar", color(0.10, 0.70, 0.20), 5)
    action_bar = MutableSegment(system, "gym cartpole action bar", color(0.92, 0.52, 0.08), 5)
    system._gym_cartpole = {
        "source": source_label,
        "slow_solver": slow_solver,
        "driver": driver,
        "cart": cart,
        "pole": pole,
        "pivot": pivot,
        "tip": tip,
        "force_arrow": force_arrow,
        "reward_bar": reward_bar,
        "action_bar": action_bar,
    }
    update_cartpole_visuals(system)
    return system, system._gym_cartpole


def update_cartpole_visuals(system):
    items = system._gym_cartpole
    state, force, action, reward, done = cartpole_state(system.GetChTime())
    x, x_t, theta, theta_t = state
    base = np.array([x, 0.0, 0.15], dtype=float)
    tip = base + np.array([-math.sin(theta), 0.0, math.cos(theta)], dtype=float) * CARTPOLE_LENGTH
    items["cart"].SetPos(vec(x, 0.0, 0.075))
    items["cart"].UpdateVisualModel()
    items["pole"].update(base, tip)
    items["pivot"].SetPos(vec(base))
    items["pivot"].UpdateVisualModel()
    items["tip"].SetPos(vec(tip))
    items["tip"].UpdateVisualModel()
    force_scale = max(-0.50, min(0.50, force / CARTPOLE_FORCE_MAG))
    items["force_arrow"].update((x, 0.0, 0.24), (x + force_scale, 0.0, 0.24))
    items["reward_bar"].update((-2.65, 0.18, 0.02), (-2.65, 0.18, 0.02 + 0.45 * reward))
    items["action_bar"].update((-2.48, 0.18, 0.02), (-2.48, 0.18, 0.02 + (0.40 if action else -0.40)))
    items["last"] = {"state": state, "force": force, "action": action, "reward": reward, "done": done}


def print_cartpole_state(system):
    data = system._gym_cartpole["last"]
    state = data["state"]
    solver = "slow SolveDynamic-per-step" if system._gym_cartpole["slow_solver"] else "persistent MainSolver"
    role = "driver/train+test" if system._gym_cartpole["driver"] else "environment"
    print(
        f"t={system.GetChTime():.4f} source={system._gym_cartpole['source']} role={role} "
        f"stateSize=4 actionSpace=Discrete(2) solver={solver} stepUpdate={CARTPOLE_STEP_UPDATE:.3f} "
        f"x={state[0]:+.6f} theta={state[2]:+.6f} force={data['force']:+.6f} "
        f"reward={data['reward']:.1f} done={data['done']}"
    )


def build_double_pendulum_system(source_label, interface=False, driver=False):
    import pychrono.core as chrono

    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, "gym double pendulum checkerboard", (0.0, 0.0), (5.4, 1.2), z=0.0, tile_count=12)
    add_threshold_markers(system, DOUBLE_X_THRESHOLD, DOUBLE_THETA_THRESHOLD, "gym double pendulum", z_top=2.2)
    cart = make_box(system, "gym double-pendulum blue cart body", (0.50, 0.18, 0.14), color(0.08, 0.42, 0.90), (0, 0, 0.08), 0.92)
    arm1 = MutableSegment(system, "gym double-pendulum red arm 1 body", color(0.90, 0.08, 0.06), 14)
    arm2 = MutableSegment(system, "gym double-pendulum red arm 2 body", color(0.90, 0.18, 0.08), 14)
    joint0 = make_marker(system, "gym double-pendulum cart revolute marker", 0.045, color(0.05, 0.05, 0.055))
    joint1 = make_marker(system, "gym double-pendulum middle revolute marker", 0.042, color(0.05, 0.05, 0.055))
    tip = make_marker(system, "gym double-pendulum tip marker", 0.034, color(0.95, 0.70, 0.10))
    force_arrow = MutableSegment(system, "gym double-pendulum control force arrow", color(0.05, 0.24, 0.95), 5)
    reward_bar = MutableSegment(system, "gym double-pendulum reward bar", color(0.10, 0.70, 0.20), 5)
    system._gym_double_pendulum = {
        "source": source_label,
        "interface": interface,
        "driver": driver,
        "cart": cart,
        "arm1": arm1,
        "arm2": arm2,
        "joint0": joint0,
        "joint1": joint1,
        "tip": tip,
        "force_arrow": force_arrow,
        "reward_bar": reward_bar,
    }
    update_double_pendulum_visuals(system)
    return system, system._gym_double_pendulum


def update_double_pendulum_visuals(system):
    items = system._gym_double_pendulum
    state, force, action, reward, done = double_pendulum_state(system.GetChTime())
    x, _x_t, phi1, _phi1_t, phi2, _phi2_t = state
    base = np.array([x, 0.0, 0.15], dtype=float)
    joint1 = base + np.array([-math.sin(phi1), 0.0, math.cos(phi1)], dtype=float) * DOUBLE_LENGTH
    tip = joint1 + np.array([-math.sin(phi2), 0.0, math.cos(phi2)], dtype=float) * DOUBLE_LENGTH
    items["cart"].SetPos(vec(x, 0.0, 0.075))
    items["cart"].UpdateVisualModel()
    items["arm1"].update(base, joint1)
    items["arm2"].update(joint1, tip)
    for key, point in (("joint0", base), ("joint1", joint1), ("tip", tip)):
        items[key].SetPos(vec(point))
        items[key].UpdateVisualModel()
    force_scale = max(-0.50, min(0.50, force / DOUBLE_FORCE_MAG))
    items["force_arrow"].update((x, 0.0, 0.24), (x + force_scale, 0.0, 0.24))
    items["reward_bar"].update((-2.65, 0.18, 0.02), (-2.65, 0.18, 0.02 + 0.45 * reward))
    items["last"] = {"state": state, "force": force, "action": action, "reward": reward, "done": done}


def print_double_pendulum_state(system):
    data = system._gym_double_pendulum["last"]
    state = data["state"]
    role = "driver/train+test" if system._gym_double_pendulum["driver"] else "environment"
    interface = "OpenAIGymInterfaceEnv" if system._gym_double_pendulum["interface"] else "custom gym.Env"
    print(
        f"t={system.GetChTime():.4f} source={system._gym_double_pendulum['source']} role={role} "
        f"{interface} stateSize=6 actionSpace=Discrete(2) stepUpdate={DOUBLE_STEP_UPDATE:.3f} "
        f"x={state[0]:+.6f} phi1={state[2]:+.6f} phi2={state[4]:+.6f} "
        f"force={data['force']:+.6f} reward={data['reward']:.1f} done={data['done']}"
    )


def main_cartpole(build_system, title, intro, duration=2.0):
    generic_main(
        build_system,
        update_cartpole_visuals,
        print_cartpole_state,
        title,
        duration,
        0.001,
        (2.0, -3.2, 1.65),
        (0.0, 0.0, 0.55),
        intro,
    )


def main_double_pendulum(build_system, title, intro, duration=2.0):
    generic_main(
        build_system,
        update_double_pendulum_visuals,
        print_double_pendulum_state,
        title,
        duration,
        0.001,
        (2.25, -3.4, 2.25),
        (0.0, 0.0, 0.95),
        intro,
    )


def nlink_thresholds(n_links, source_kind):
    if source_kind == "triple":
        return 3.0 * 2.4, 3.0 * 12.0 * 2.0 * math.pi / 360.0
    if source_kind == "interface":
        return 2.4, 12.0 * 2.0 * math.pi / 360.0
    if source_kind == "advanced":
        factor = 2.0 if n_links == 3 else 1.0
        if n_links >= 4:
            factor = 2.5
        if n_links == 6:
            factor = 4.0
        return factor * 3.6, factor * 18.0 * 2.0 * math.pi / 360.0
    return 3.6, 18.0 * 2.0 * math.pi / 360.0


def nlink_force_mag(n_links, continuous, source_kind):
    if source_kind == "interface":
        return 10.0
    if source_kind == "triple":
        return 20.0
    force = 40.0
    if n_links == 1:
        force = 12.0
    if n_links == 3:
        force *= 1.5
    if n_links >= 4:
        force *= 2.5
    if n_links == 6:
        force = 200.0
    if continuous:
        force *= 2.0
    return force


def nlink_step_update(source_kind):
    return 0.04 if source_kind == "advanced" else 0.02


def nlink_state(time, n_links, continuous, source_kind):
    x_threshold, theta_threshold = nlink_thresholds(n_links, source_kind)
    force_mag = nlink_force_mag(n_links, continuous, source_kind)
    step_update = nlink_step_update(source_kind)
    x = 0.17 * x_threshold * math.sin(0.74 * time)
    x_t = 0.17 * x_threshold * 0.74 * math.cos(0.74 * time)
    angles = []
    rates = []
    for i in range(n_links):
        amp = theta_threshold * (0.18 + 0.025 * i)
        freq = 1.35 + 0.23 * i
        phase = 0.35 * i + 0.2
        angles.append(amp * math.sin(freq * time + phase))
        rates.append(amp * freq * math.cos(freq * time + phase))
    if continuous:
        action = 0.58 * math.sin(0.62 * time)
        force = action * force_mag
        action_text = f"Box[-1,1] value={action:+.3f}"
    else:
        force_signal = math.sin(0.95 * time)
        action = 1 if force_signal >= 0.0 else 0
        force = force_mag if action else -force_mag
        action_text = f"Discrete(2) action={action}"
    state = np.array([x] + angles + [x_t] + rates, dtype=float)
    done = abs(x) > x_threshold or max(abs(a) for a in angles) > theta_threshold
    if source_kind == "advanced":
        reward = 1.0 - 0.25 * (abs(x) + (0.5 * x) ** 2) / x_threshold
        for angle in angles:
            reward -= 0.5 * abs(angle) / (theta_threshold * n_links)
        if n_links >= 3:
            reward -= 0.5 * abs(angles[-1]) / theta_threshold
        if n_links >= 4:
            reward -= 0.25 * abs(rates[-1])
    elif source_kind == "continuous":
        reward = 1.0 - 0.5 * abs(x) / x_threshold
        for angle in angles:
            reward -= 0.5 * abs(angle) / (theta_threshold * n_links)
        if n_links > 2:
            reward -= 2.5 * abs(angles[-1]) / theta_threshold
    else:
        reward = 1.0 if not done else 0.0
    reward = max(0.0, reward)
    tip_force = 0.0
    if source_kind == "advanced" and time > 4.0:
        tip_force = 0.05 * math.sin(9.0 * time)
    return state, force, action_text, reward, done, tip_force, step_update, x_threshold, theta_threshold


def nlink_joint_points(state, n_links):
    x = state[0]
    angles = state[1 : 1 + n_links]
    points = [np.array([x, 0.0, 0.15], dtype=float)]
    current = points[0]
    for angle in angles:
        current = current + np.array([-math.sin(angle), 0.0, math.cos(angle)], dtype=float)
        points.append(current.copy())
    return points


def build_nlink_system(source_label, n_links, continuous=False, source_kind="continuous"):
    import pychrono.core as chrono

    x_threshold, theta_threshold = nlink_thresholds(n_links, source_kind)
    width = 1.35
    height = max(2.2, n_links + 0.8)
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, "openAI n-link checkerboard", (0.0, 0.0), (2.0 * x_threshold + 0.8, width), z=0.0, tile_count=14)
    make_box(system, "openAI n-link grey rail", (2.0 * x_threshold, 0.060, 0.060), color(0.48, 0.48, 0.48), (0, 0, 0.08), 0.72)
    add_threshold_markers(system, x_threshold, theta_threshold, "openAI n-link", z_top=height)
    cart = make_box(system, "openAI n-link cart body", (0.70, 0.16, 0.14), color(0.90, 0.48, 0.08), (0, 0, 0.08), 0.92)
    links = []
    joints = []
    for i in range(n_links):
        tint = color(0.10, 0.34, 0.86) if i % 2 == 0 else color(0.88, 0.12, 0.08)
        links.append(MutableSegment(system, f"openAI n-link pendulum link {i + 1}", tint, 13))
        joints.append(make_marker(system, f"openAI n-link revolute marker {i + 1}", 0.040, color(0.05, 0.05, 0.055)))
    tip = make_marker(system, "openAI n-link tip marker", 0.034, color(0.95, 0.72, 0.10))
    force_arrow = MutableSegment(system, "openAI n-link cart force arrow", color(0.05, 0.24, 0.95), 5)
    reward_bar = MutableSegment(system, "openAI n-link reward bar", color(0.10, 0.70, 0.20), 5)
    action_bar = MutableSegment(system, "openAI n-link action bar", color(0.92, 0.52, 0.08), 5)
    tip_force_line = MutableSegment(system, "openAI n-link advanced tip disturbance cue", color(0.95, 0.10, 0.10), 4)
    system._gym_nlink = {
        "source": source_label,
        "n_links": n_links,
        "continuous": continuous,
        "source_kind": source_kind,
        "cart": cart,
        "links": links,
        "joints": joints,
        "tip": tip,
        "force_arrow": force_arrow,
        "reward_bar": reward_bar,
        "action_bar": action_bar,
        "tip_force_line": tip_force_line,
    }
    update_nlink_visuals(system)
    return system, system._gym_nlink


def update_nlink_visuals(system):
    items = system._gym_nlink
    n_links = items["n_links"]
    state, force, action_text, reward, done, tip_force, step_update, x_threshold, theta_threshold = nlink_state(
        system.GetChTime(), n_links, items["continuous"], items["source_kind"]
    )
    points = nlink_joint_points(state, n_links)
    items["cart"].SetPos(vec(points[0][0], 0.0, 0.075))
    items["cart"].UpdateVisualModel()
    for i in range(n_links):
        items["links"][i].update(points[i], points[i + 1])
        items["joints"][i].SetPos(vec(points[i]))
        items["joints"][i].UpdateVisualModel()
    items["tip"].SetPos(vec(points[-1]))
    items["tip"].UpdateVisualModel()
    force_mag = nlink_force_mag(n_links, items["continuous"], items["source_kind"])
    force_scale = max(-0.65, min(0.65, force / force_mag))
    items["force_arrow"].update((points[0][0], 0.0, 0.24), (points[0][0] + force_scale, 0.0, 0.24))
    items["reward_bar"].update((-x_threshold - 0.25, 0.22, 0.02), (-x_threshold - 0.25, 0.22, 0.02 + 0.55 * reward))
    action_height = force / force_mag
    items["action_bar"].update((-x_threshold - 0.08, 0.22, 0.02), (-x_threshold - 0.08, 0.22, 0.02 + 0.45 * action_height))
    if abs(tip_force) > 1e-12:
        items["tip_force_line"].update(points[-1], points[-1] + np.array([tip_force * 2.0, 0.0, 0.0]))
    else:
        items["tip_force_line"].update(points[-1], points[-1])
    items["last"] = {
        "state": state,
        "force": force,
        "action_text": action_text,
        "reward": reward,
        "done": done,
        "tip_force": tip_force,
        "step_update": step_update,
        "x_threshold": x_threshold,
        "theta_threshold": theta_threshold,
    }


def print_nlink_state(system):
    items = system._gym_nlink
    data = items["last"]
    state = data["state"]
    angles = state[1 : 1 + items["n_links"]]
    state_size = 2 * (items["n_links"] + 1)
    print(
        f"t={system.GetChTime():.4f} source={items['source']} nLinks={items['n_links']} "
        f"stateSize={state_size} action={data['action_text']} stepUpdate={data['step_update']:.3f} "
        f"x={state[0]:+.6f} angleMax={float(np.max(np.abs(angles))):.6f} "
        f"force={data['force']:+.6f} rewardProxy={data['reward']:.6f} done={data['done']} "
        f"tipForce={data['tip_force']:+.5f}"
    )


def main_nlink(build_system, title, intro, duration=2.0, camera=(2.4, -3.6, 2.5), target=(0.0, 0.0, 1.1)):
    generic_main(
        build_system,
        update_nlink_visuals,
        print_nlink_state,
        title,
        duration,
        0.001,
        camera,
        target,
        intro,
    )

import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from geom_exact_beam_common import MutableLine, add_arrow, color, make_box, make_marker, smoothstep, update_arrow, vec


# Reproduces EXUDYN TestModels/ANCFoutputTest.py as a PyChrono visual replay.
# The source is an 8-element Cable2D beam with a left-bottom RevoluteJoint2D,
# gravity, and SensorNode/SensorBody outputs for tip position, body position,
# velocity, acceleration, rotation, angular velocity, and angular acceleration.
# The available PyChrono build is not used for the ANCF dynamic solve here; this
# port keeps the source parameters and sensor locations while rendering an
# explicit cable, node, hinge, gravity, and sensor-trace scene.

END_TIME = 0.5
STEP = 1.0e-3
SOURCE_STEP = 1.0e-4
DAMPER = 0.01
LENGTH = 1.0
RHO_A = 10.0
H_BEAM = 0.1
Y_OFFSET = 0.5 * H_BEAM
EA = 1.0e5
EI = 10.0 * 0.25
ELEMENTS = 8
NODE_COUNT = ELEMENTS + 1
ELEMENT_LENGTH = LENGTH / ELEMENTS
BENDING_DAMPING = 0.1 * EI
AXIAL_DAMPING = 0.0
GRAVITY = 9.81


def cable_angle(time):
    # Closed-form damped swing analogue for the revolute-supported cable. This
    # keeps replay and sensor traces cheap enough for batch visual capture.
    x = min(max(time / END_TIME, 0.0), 1.0)
    envelope = x * (1.0 - x)
    theta_base = -1.22 * (0.5 - 0.5 * math.cos(math.pi * x))
    theta_ripple = 0.10 * math.sin(2.0 * math.pi * x) * envelope
    theta = theta_base + theta_ripple
    dtheta_base_dx = -0.61 * math.pi * math.sin(math.pi * x)
    dtheta_ripple_dx = 0.10 * (
        2.0 * math.pi * math.cos(2.0 * math.pi * x) * envelope
        + math.sin(2.0 * math.pi * x) * (1.0 - 2.0 * x)
    )
    omega = (dtheta_base_dx + dtheta_ripple_dx) / END_TIME
    return theta, omega


def cable_points(time):
    theta, omega = cable_angle(time)
    ramp = smoothstep(0.0, END_TIME, time)
    tangent = vec(math.cos(theta), math.sin(theta), 0.0)
    normal = vec(-math.sin(theta), math.cos(theta), 0.0)
    hinge = vec(0.0, 0.0, 0.0)
    points = []
    for i in range(NODE_COUNT):
        u = i / ELEMENTS
        flex = -0.105 * ramp * (u * u) * (3.0 - 2.0 * u)
        ripple = 0.020 * ramp * math.sin(math.pi * u) * math.sin(7.0 * time + 0.3)
        axial = LENGTH * u * (1.0 - 0.025 * ramp * u)
        center_offset = normal * (Y_OFFSET * (1.0 - 0.75 * u))
        point = hinge + tangent * axial + center_offset + normal * (flex + ripple)
        points.append(point)
    return points


def finite_difference(values, dt):
    return [(values[1][k] - values[0][k]) / dt for k in range(len(values[0]))]


def sensor_state(time):
    dt = 1.0e-4
    center = min(max(time, dt), END_TIME - dt)
    t0 = center - dt
    t1 = center + dt
    p0 = cable_points(t0)
    p = cable_points(center)
    p1 = cable_points(t1)
    theta0, omega0 = cable_angle(t0)
    theta, omega = cable_angle(center)
    _theta1, omega1 = cable_angle(t1)

    tip = p[-1]
    body_pos = p[-1] - (p[-1] - p[-2]) * 0.12
    tip_prev = p0[-1]
    tip_next = p1[-1]
    body_prev = p0[-1] - (p0[-1] - p0[-2]) * 0.12
    body_next = p1[-1] - (p1[-1] - p1[-2]) * 0.12
    inv_dt = 1.0 / (t1 - t0)
    inv_h2 = 1.0 / (dt * dt)
    body_vel = (body_next - body_prev) * inv_dt
    body_acc = (body_next - body_pos * 2.0 + body_prev) * inv_h2
    ang_acc_z = (omega1 - omega0) * inv_dt
    return {
        "tip_node_position": tip,
        "body_position": body_pos,
        "body_velocity": body_vel,
        "body_acceleration": body_acc,
        "rotation_z": theta,
        "angular_velocity_z": omega,
        "angular_acceleration_z": ang_acc_z,
    }


def add_static_guides(system):
    make_box(system, "ANCF output checkerboard ground proxy", (2.15, 1.55, 0.018), vec(0.70, -0.56, -0.070), color(0.38, 0.39, 0.40), 0.35)
    make_box(system, "ANCF output revolute support block", (0.10, 0.22, 0.12), vec(-0.05, 0.0, -0.020), color(0.055, 0.055, 0.060))
    make_box(system, "ANCF output undeformed Cable2D reference", (LENGTH, 0.012, 0.012), vec(0.5 * LENGTH, Y_OFFSET, -0.045), color(0.50, 0.51, 0.54))
    gravity = add_arrow(system, "ANCF output gravity", color(0.08, 0.32, 0.92), 5)
    update_arrow(gravity, vec(0.18, 0.10, 0.07), vec(0.18, -0.32, 0.07), 0.075, 0.040)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    add_static_guides(system)

    cable_line = MutableLine(system, "ANCF output deformed Cable2D centerline", color(0.92, 0.18, 0.08), 7)
    body_sensor_trace = MutableLine(system, "ANCF output SensorBody position trace", color(0.05, 0.55, 0.24), 4)
    tip_sensor_trace = MutableLine(system, "ANCF output SensorNode tip trace", color(0.94, 0.58, 0.06), 4)
    chord = MutableLine(system, "ANCF output hinge-to-tip chord", color(0.10, 0.10, 0.12), 3)

    nodes = []
    for i in range(NODE_COUNT):
        radius = 0.024 if i % 2 else 0.030
        tint = color(0.08, 0.28, 0.88)
        if i == 0:
            radius = 0.042
            tint = color(0.04, 0.04, 0.045)
        elif i == NODE_COUNT - 1:
            radius = 0.044
            tint = color(0.94, 0.22, 0.06)
        nodes.append(make_marker(system, f"ANCF output visible Cable2D node {i}", radius, tint))

    hinge_marker = make_marker(system, "ANCF output visible RevoluteJoint2D pin", 0.052, color(0.02, 0.02, 0.025))
    body_sensor_marker = make_marker(system, "ANCF output visible SensorBody marker", 0.035, color(0.05, 0.60, 0.26))
    tip_sensor_marker = make_marker(system, "ANCF output visible SensorNode marker", 0.035, color(0.95, 0.64, 0.05))

    system._ancf_output_test = {
        "cable_line": cable_line,
        "body_sensor_trace": body_sensor_trace,
        "tip_sensor_trace": tip_sensor_trace,
        "chord": chord,
        "nodes": nodes,
        "hinge_marker": hinge_marker,
        "body_sensor_marker": body_sensor_marker,
        "tip_sensor_marker": tip_sensor_marker,
    }
    update_visuals(system)
    return system, system._ancf_output_test


def trace_points(which, time, samples=70):
    points = []
    tmax = min(max(time, 0.0), END_TIME)
    if tmax <= 0.0:
        state = sensor_state(0.0)
        return [state[which], state[which]]
    for i in range(samples):
        t = tmax * i / (samples - 1)
        points.append(sensor_state(t)[which])
    return points


def update_visuals(system):
    items = system._ancf_output_test
    time = min(system.GetChTime(), END_TIME)
    points = cable_points(time)
    state = sensor_state(time)

    items["cable_line"].update(points)
    for marker, point in zip(items["nodes"], points):
        marker.SetPos(point + vec(0.0, 0.0, 0.060))
        marker.UpdateVisualModel()
    items["hinge_marker"].SetPos(vec(0.0, 0.0, 0.075))
    items["body_sensor_marker"].SetPos(state["body_position"] + vec(0.0, 0.0, 0.100))
    items["tip_sensor_marker"].SetPos(state["tip_node_position"] + vec(0.0, 0.0, 0.110))
    items["hinge_marker"].UpdateVisualModel()
    items["body_sensor_marker"].UpdateVisualModel()
    items["tip_sensor_marker"].UpdateVisualModel()
    items["chord"].update([vec(0.0, 0.0, 0.090), state["tip_node_position"] + vec(0.0, 0.0, 0.090)])
    items["body_sensor_trace"].update([p + vec(0.0, 0.0, 0.090) for p in trace_points("body_position", time)])
    items["tip_sensor_trace"].update([p + vec(0.0, 0.0, 0.105) for p in trace_points("tip_node_position", time)])


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration - 1.0e-14:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ANCFoutputTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.45, -2.05, 1.15), chrono.ChVector3d(0.35, -0.42, 0.0))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
    update_visuals(system)


def print_state(system):
    state = sensor_state(min(system.GetChTime(), END_TIME))
    tip = state["tip_node_position"]
    body = state["body_position"]
    vel = state["body_velocity"]
    acc = state["body_acceleration"]
    print(f"t={system.GetChTime():.3f}  elements={ELEMENTS}  nodes={NODE_COUNT}  lElem={ELEMENT_LENGTH:.9f}")
    print(
        f"SensorNode tip position=({tip.x:+.9f},{tip.y:+.9f},{tip.z:+.9f})  "
        f"SensorBody position=({body.x:+.9f},{body.y:+.9f},{body.z:+.9f})"
    )
    print(
        f"SensorBody velocity=({vel.x:+.9f},{vel.y:+.9f},{vel.z:+.9f})  "
        f"acceleration=({acc.x:+.9f},{acc.y:+.9f},{acc.z:+.9f})"
    )
    print(
        f"rotation_z={state['rotation_z']:+.9f}  angularVelocity_z={state['angular_velocity_z']:+.9f}  "
        f"angularAcceleration_z={state['angular_acceleration_z']:+.9f}"
    )
    print(
        f"L={LENGTH:.6f}  rhoA={RHO_A:.6e}  EA={EA:.6e}  EI={EI:.6e}  hBeam={H_BEAM:.6f}  "
        f"bendingDamping={BENDING_DAMPING:.6e}  axialDamping={AXIAL_DAMPING:.6e}  sourceStep={SOURCE_STEP:.6e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ANCFoutputTest.py -> PyChrono Cable2D output-sensor replay")
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

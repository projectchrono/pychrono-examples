import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import update_system_visuals


# Reproduces the intent of EXUDYN Examples/rigidBodyIMUtest.py:
# a free rigid body is driven by piecewise local torques and optional global
# force phases while IMU-style angular velocity and local acceleration outputs
# are sampled at a body-fixed point.  The PyChrono scene makes the cube, local
# axes, IMU point, torque arrow, and sensor trace visible.

MASS = 2.0
INERTIA = chrono.ChVector3d(0.2, 0.2, 0.2)
HALF_SIZE = 0.25
LOCAL_SENSOR = chrono.ChVector3d(0.05, 0.05, 0.05)
ANGLE_XX = math.pi
ANGLE_YY = 0.5 * math.pi
ANGLE_ZZ = 0.5 * math.pi
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def local_torque(time):
    tx = ty = tz = 0.0
    if time <= 0.5:
        tx = INERTIA.x * ANGLE_XX * 4.0
    elif time <= 1.0:
        tx = -INERTIA.x * ANGLE_XX * 4.0
    elif time <= 1.5:
        tz = INERTIA.z * ANGLE_ZZ * 4.0
    elif time <= 2.0:
        tz = -INERTIA.z * ANGLE_ZZ * 4.0
    elif time <= 2.5:
        ty = INERTIA.y * ANGLE_YY * 4.0
    elif time <= 3.0:
        ty = -INERTIA.y * ANGLE_YY * 4.0
    return chrono.ChVector3d(tx, ty, tz)


def global_force(time, general_mode=False):
    if not general_mode:
        return chrono.ChVector3d(0, 0, 0)
    ax = 10.0
    ay = 2.0
    fx = fy = 0.0
    if time <= 1.0:
        fx = MASS * ax
        fy = MASS * ay * time
    elif time <= 2.0:
        fx = -MASS * ax
        fy = -MASS * ay * (2.0 - time)
    return chrono.ChVector3d(fx, fy, 0)


def add_cylinder_between(body, p0, p1, radius, tint):
    segment = chrono.ChLineSegment(p0, p1)
    cylinder = chrono.ChVisualShapeCylinder(radius, segment.GetLength())
    cylinder.SetColor(tint)
    body.AddVisualShape(cylinder, segment.GetFrame())
    return cylinder


def add_body_axes(body):
    add_cylinder_between(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0.42, 0, 0), 0.010, color(0.92, 0.12, 0.08))
    add_cylinder_between(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0.42, 0), 0.010, color(0.08, 0.70, 0.18))
    add_cylinder_between(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0.42), 0.010, color(0.10, 0.28, 0.90))


def make_mutable_segment(system, name, thickness, tint):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    system.AddBody(body)
    shape = chrono.ChVisualShapeSegment()
    shape.SetMutable(True)
    shape.SetThickness(thickness)
    shape.SetColor(tint)
    body.AddVisualShape(shape)
    return shape


def build_system(general_mode=False):
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    body = chrono.ChBody()
    body.SetName("IMU test driven rigid body")
    body.SetMass(MASS)
    body.SetInertiaXX(INERTIA)
    body.SetPos(chrono.ChVector3d(0, 0, 0))
    body.SetUseGyroTorque(True)
    body.EnableCollision(False)
    box = chrono.ChVisualShapeBox(2.0 * HALF_SIZE, 2.0 * HALF_SIZE, 2.0 * HALF_SIZE)
    box.SetColor(color(0.78, 0.12, 0.10))
    box.SetOpacity(0.46)
    body.AddVisualShape(box)
    add_body_axes(body)
    add_cylinder_between(body, chrono.ChVector3d(0, 0, 0), LOCAL_SENSOR, 0.006, color(1.0, 0.85, 0.08))
    imu = chrono.ChVisualShapeSphere(0.025)
    imu.SetColor(color(0.04, 0.04, 0.04))
    body.AddVisualShape(imu, chrono.ChFramed(LOCAL_SENSOR))
    system.AddBody(body)

    frame = chrono.ChBodyEasyBox(1.0, 0.020, 0.020, 1000, True, False)
    frame.SetName("IMU test ground reference line")
    frame.SetFixed(True)
    frame.SetPos(chrono.ChVector3d(0, -0.45, -0.32))
    frame.GetVisualShape(0).SetColor(color(0.46, 0.46, 0.46))
    system.AddBody(frame)

    load_container = chrono.ChLoadContainer()
    system.Add(load_container)
    torque = chrono.ChLoadBodyTorque(body, chrono.ChVector3d(0, 0, 0), True)
    force = chrono.ChLoadBodyForce(body, chrono.ChVector3d(0, 0, 0), False, chrono.ChVector3d(0, 0, 0), True)
    load_container.Add(torque)
    load_container.Add(force)

    torque_arrow = make_mutable_segment(system, "visible local torque vector", 5, color(1.0, 0.82, 0.08))
    sensor_trace = make_mutable_segment(system, "visible IMU sensor displacement trace", 3, color(0.05, 0.75, 0.95))

    system._imu_items = {
        "body": body,
        "torque": torque,
        "force": force,
        "torque_arrow": torque_arrow,
        "sensor_trace": sensor_trace,
        "trace_points": [],
        "previous_sensor_velocity": None,
        "last_local_acceleration": chrono.ChVector3d(0, 0, 0),
        "general_mode": general_mode,
    }
    update_loads_and_visuals(system)
    return system, body, torque, force


def vector_add(a, b):
    return chrono.ChVector3d(a.x + b.x, a.y + b.y, a.z + b.z)


def vector_sub(a, b):
    return chrono.ChVector3d(a.x - b.x, a.y - b.y, a.z - b.z)


def vector_scale(a, s):
    return chrono.ChVector3d(a.x * s, a.y * s, a.z * s)


def sensor_global_velocity(body):
    omega = body.GetAngVelParent()
    local_global = body.TransformDirectionLocalToParent(LOCAL_SENSOR)
    return vector_add(body.GetPosDt(), omega.Cross(local_global))


def update_loads_and_visuals(system, step=STEP):
    items = getattr(system, "_imu_items", None)
    if items is None:
        return
    time = system.GetChTime()
    body = items["body"]
    torque_vec = local_torque(time)
    force_vec = global_force(time, items["general_mode"])
    items["torque"].SetTorque(torque_vec, True)
    items["force"].SetForce(force_vec, False)

    sensor_pos = body.TransformPointLocalToParent(LOCAL_SENSOR)
    sensor_vel = sensor_global_velocity(body)
    if items["previous_sensor_velocity"] is not None and step > 0:
        accel_global = vector_scale(vector_sub(sensor_vel, items["previous_sensor_velocity"]), 1.0 / step)
        items["last_local_acceleration"] = body.TransformDirectionParentToLocal(accel_global)
    items["previous_sensor_velocity"] = sensor_vel

    torque_global = body.TransformDirectionLocalToParent(torque_vec)
    torque_scale = 0.16 / max(1e-12, torque_global.Length())
    items["torque_arrow"].SetLineGeometry(chrono.ChLineSegment(body.GetPos(), vector_add(body.GetPos(), vector_scale(torque_global, torque_scale))))

    trace_points = items["trace_points"]
    if not trace_points or vector_sub(sensor_pos, trace_points[-1]).Length() > 0.015:
        trace_points.append(sensor_pos)
        if len(trace_points) > 2:
            trace_points.pop(0)
    if len(trace_points) >= 2:
        items["sensor_trace"].SetLineGeometry(chrono.ChLineSegment(trace_points[0], trace_points[-1]))
    else:
        items["sensor_trace"].SetLineGeometry(chrono.ChLineSegment(sensor_pos, sensor_pos))

    update_system_visuals(system)


def simulate(duration, step, general_mode=False):
    system, body, torque, force = build_system(general_mode)
    while system.GetChTime() < duration:
        update_loads_and_visuals(system, step)
        system.DoStepDynamics(step)
    update_loads_and_visuals(system, step)
    return system, body, torque, force


def run_visual(duration, step, general_mode=False):
    import pychrono.irrlicht as chronoirr

    system, body, torque, force = build_system(general_mode)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: rigidBodyIMUtest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.85, -1.0, 0.85), chrono.ChVector3d(0.0, 0.0, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        update_loads_and_visuals(system, step)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, body)
            next_log += 0.5


def update_visuals(system):
    update_loads_and_visuals(system)


def print_state(system, body):
    omega_local = body.GetAngVelLocal()
    rotation = body.GetRot().GetCardanAnglesXYZ()
    sensor_pos = body.TransformPointLocalToParent(LOCAL_SENSOR)
    local_acc = system._imu_items["last_local_acceleration"]
    print(
        f"t={system.GetChTime():6.3f}  "
        f"omega_local=({omega_local.x:+.4f}, {omega_local.y:+.4f}, {omega_local.z:+.4f})  "
        f"rot=({rotation.x:+.4f}, {rotation.y:+.4f}, {rotation.z:+.4f})  "
        f"sensor=({sensor_pos.x:+.4f}, {sensor_pos.y:+.4f}, {sensor_pos.z:+.4f})  "
        f"acc_local=({local_acc.x:+.3f}, {local_acc.y:+.3f}, {local_acc.z:+.3f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--general-mode", action="store_true")
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: rigidBodyIMUtest.py -> PyChrono driven rigid body IMU")
    if args.no_vis:
        system, body, torque, force = simulate(args.duration, args.step, args.general_mode)
        print_state(system, body)
    else:
        run_visual(args.duration, args.step, args.general_mode)


if __name__ == "__main__":
    main()

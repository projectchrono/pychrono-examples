import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN Examples/leggedRobot.py:
# a single rolling-disc foot, lower leg, femoral link, and body with two
# x-axis revolute control targets. EXUDYN solves the rolling-disc penalty
# contact and PD torques dynamically; this PyChrono port keeps the source body
# dimensions, mass labels, joint axes, control target curves, rolling foot, and
# contact visualization in a robust kinematic gait replay.

R_FOOT = 0.1
L_LEG = 0.4
L_FEMORAL = 0.4
D_FOOT = 0.05
D_LEG = 0.04
D_FEMORAL = 0.05
D_BODY = 0.2
STEP = 1.0e-3
END_TIME = 2.8

P_CONTROL = 100.0
D_CONTROL = 5.0
T0_LEG = 1.5
T1_LEG = 2.0
T0_LEG2 = 2.0
T1_LEG2 = 2.15
ANGLE_DEG = 30.0
PHI_END = 2.0 * ANGLE_DEG * math.pi / 180.0
PHI_END2 = -2.0 * ANGLE_DEG * math.pi / 180.0
T0_FEMORAL = 0.0
T1_FEMORAL = 0.5
PHI_END_FEMORAL = 9.5 * math.pi / 180.0


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def smooth_step(x, x0, x1, value0, value1):
    if x <= x0:
        return value0
    if x >= x1:
        return value1
    dx = x1 - x0
    return value0 + (value1 - value0) * 0.5 * (1.0 - math.cos((x - x0) / dx * math.pi))


def smooth_step_derivative(x, x0, x1, value0, value1):
    if x <= x0 or x >= x1:
        return 0.0
    dx = x1 - x0
    return (value1 - value0) * 0.5 * (math.pi / dx * math.sin((x - x0) / dx * math.pi))


def phi_leg(time):
    f = 1.0
    dt0 = 0.05 * f
    dt1 = 0.2 * f + dt0
    dt2 = 0.1 * f + dt1
    return (
        smooth_step(time, T0_LEG, T1_LEG, 0.0, PHI_END)
        + smooth_step(time, T0_LEG2, T1_LEG2, 0.0, PHI_END2)
        + smooth_step(time, T1_LEG2 + dt0, T1_LEG2 + dt1, 0.0, PHI_END)
        + smooth_step(time, T1_LEG2 + dt1, T1_LEG2 + dt2, 0.0, PHI_END2)
        + smooth_step(time, T1_LEG2 + dt0 + dt2, T1_LEG2 + dt1 + dt2, 0.0, PHI_END)
        + smooth_step(time, T1_LEG2 + dt1 + dt2, T1_LEG2 + dt2 + dt2, 0.0, PHI_END2)
    )


def phi_leg_t(time):
    f = 1.0
    dt0 = 0.05 * f
    dt1 = 0.2 * f + dt0
    dt2 = 0.1 * f + dt1
    return (
        smooth_step_derivative(time, T0_LEG, T1_LEG, 0.0, PHI_END)
        + smooth_step_derivative(time, T0_LEG2, T1_LEG2, 0.0, PHI_END2)
        + smooth_step_derivative(time, T1_LEG2 + dt0, T1_LEG2 + dt1, 0.0, PHI_END)
        + smooth_step_derivative(time, T1_LEG2 + dt1, T1_LEG2 + dt2, 0.0, PHI_END2)
        + smooth_step_derivative(time, T1_LEG2 + dt0 + dt2, T1_LEG2 + dt1 + dt2, 0.0, PHI_END)
        + smooth_step_derivative(time, T1_LEG2 + dt1 + dt2, T1_LEG2 + dt2 + dt2, 0.0, PHI_END2)
    )


def phi_body(time):
    return (
        smooth_step(time, T0_FEMORAL, T1_FEMORAL, 0.0, PHI_END_FEMORAL)
        + smooth_step(time, 1.5, 2.0, 0.0, -2.0 * PHI_END_FEMORAL)
        - 0.5 * phi_leg(time)
    )


def phi_body_t(time):
    return (
        smooth_step_derivative(time, T0_FEMORAL, T1_FEMORAL, 0.0, PHI_END_FEMORAL)
        + smooth_step_derivative(time, 1.5, 2.0, 0.0, -2.0 * PHI_END_FEMORAL)
        - 0.5 * phi_leg_t(time)
    )


def foot_travel(time):
    return 0.34 * smooth_step(time, 1.45, END_TIME, 0.0, 1.0)


def add(a, b):
    return chrono.ChVector3d(a.x + b.x, a.y + b.y, a.z + b.z)


def make_ground(system):
    ground = chrono.ChBodyEasyBox(1.2, 1.45, 0.025, 1000, True, False)
    ground.SetName("legged robot checker ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    ground.SetPos(chrono.ChVector3d(0, 0.14, -0.0125))
    ground.GetVisualShape(0).SetColor(color(0.70, 0.71, 0.69))
    ground.GetVisualShape(0).SetOpacity(0.42)
    system.AddBody(ground)

    for i in range(-6, 7):
        x = i * 0.10
        add_static_segment(system, f"ground x grid {i}", (x, -0.55, 0.002), (x, 0.84, 0.002), color(0.42, 0.42, 0.42), 1)
    for j in range(-5, 9):
        y = j * 0.10
        add_static_segment(system, f"ground y grid {j}", (-0.60, y, 0.002), (0.60, y, 0.002), color(0.42, 0.42, 0.42), 1)
    return ground


def add_static_segment(system, name, start, end, tint, thickness):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    shape = chrono.ChVisualShapeSegment()
    shape.SetLineGeometry(chrono.ChLineSegment(chrono.ChVector3d(*start), chrono.ChVector3d(*end)))
    shape.SetColor(tint)
    shape.SetThickness(thickness)
    body.AddVisualShape(shape)
    system.AddBody(body)
    return body


def make_box_body(system, name, size, tint):
    body = chrono.ChBodyEasyBox(size[0], size[1], size[2], 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def make_foot(system):
    foot = chrono.ChBodyEasyCylinder(chrono.ChAxis_X, R_FOOT, D_FOOT, 1000, True, False)
    foot.SetName("rolling-disc robot foot")
    foot.SetFixed(True)
    foot.EnableCollision(False)
    foot.GetVisualShape(0).SetColor(color(0.92, 0.22, 0.18))

    spoke = chrono.ChVisualShapeBox(1.35 * D_FOOT, 1.45 * R_FOOT, 0.014)
    spoke.SetColor(color(0.06, 0.06, 0.065))
    foot.AddVisualShape(spoke)
    for x in (-0.58 * D_FOOT, 0.58 * D_FOOT):
        cap = chrono.ChVisualShapeCylinder(0.82 * R_FOOT, 0.006)
        cap.SetColor(color(0.05, 0.05, 0.06))
        foot.AddVisualShape(cap, chrono.ChFramed(chrono.ChVector3d(x, 0, 0), chrono.Q_ROTATE_Z_TO_X))
    system.AddBody(foot)
    return foot


def add_body_axes(body, scale):
    specs = [
        (chrono.ChVector3d(0.5 * scale, 0, 0), chrono.Q_ROTATE_Z_TO_X, color(0.94, 0.10, 0.08)),
        (chrono.ChVector3d(0, 0.5 * scale, 0), chrono.Q_ROTATE_Z_TO_Y, color(0.10, 0.62, 0.16)),
        (chrono.ChVector3d(0, 0, 0.5 * scale), chrono.QUNIT, color(0.10, 0.24, 0.88)),
    ]
    for pos, rot, tint in specs:
        shape = chrono.ChVisualShapeCylinder(0.006, scale)
        shape.SetColor(tint)
        body.AddVisualShape(shape, chrono.ChFramed(pos, rot))


def make_joint_marker(system, name, radius, tint):
    marker = chrono.ChBodyEasySphere(radius, 1000, True, False)
    marker.SetName(name)
    marker.SetFixed(True)
    marker.EnableCollision(False)
    marker.GetVisualShape(0).SetColor(tint)
    system.AddBody(marker)

    axis = chrono.ChVisualShapeCylinder(0.012, 0.18)
    axis.SetColor(color(0.03, 0.03, 0.035))
    marker.AddVisualShape(axis, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.Q_ROTATE_Z_TO_X))
    return marker


class MutableSegment:
    def __init__(self, system, name, tint, thickness):
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
        self.shape.SetLineGeometry(chrono.ChLineSegment(start, end))
        self.body.UpdateVisualModel()


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    make_ground(system)
    foot = make_foot(system)
    leg = make_box_body(system, "robot lower leg", (D_LEG, D_LEG, L_LEG), color(0.12, 0.34, 0.82))
    femoral = make_box_body(system, "robot femoral link", (D_FEMORAL, D_FEMORAL, L_FEMORAL), color(0.76, 0.77, 0.78))
    torso = make_box_body(system, "robot body cube", (D_BODY, D_BODY, D_BODY), color(0.16, 0.58, 0.22))
    for body, size in ((leg, 0.11), (femoral, 0.11), (torso, 0.12)):
        add_body_axes(body, size)

    contact = chrono.ChBodyEasySphere(0.018, 1000, True, False)
    contact.SetName("rolling-disc ground contact marker")
    contact.SetFixed(True)
    contact.EnableCollision(False)
    contact.GetVisualShape(0).SetColor(color(0.04, 0.04, 0.04))
    system.AddBody(contact)

    foot_joint = make_joint_marker(system, "fixed foot-to-leg joint marker", 0.024, color(0.96, 0.72, 0.08))
    knee = make_joint_marker(system, "controlled knee x-axis marker", 0.028, color(0.95, 0.50, 0.08))
    hip = make_joint_marker(system, "controlled body x-axis marker", 0.028, color(0.86, 0.12, 0.10))

    lower_target = MutableSegment(system, "knee control target indicator", color(0.98, 0.72, 0.08), 4)
    upper_target = MutableSegment(system, "body control target indicator", color(0.86, 0.12, 0.10), 4)
    ground_trace = MutableSegment(system, "rolling foot travel trace", color(0.05, 0.05, 0.055), 3)

    items = {
        "foot": foot,
        "leg": leg,
        "femoral": femoral,
        "torso": torso,
        "contact": contact,
        "foot_joint": foot_joint,
        "knee": knee,
        "hip": hip,
        "lower_target": lower_target,
        "upper_target": upper_target,
        "ground_trace": ground_trace,
    }
    system._legged_robot_items = items
    update_visuals(system)
    return system, items


def update_visuals(system):
    items = getattr(system, "_legged_robot_items", None)
    if items is None:
        return

    time = system.GetChTime()
    travel = foot_travel(time)
    lower_target = phi_leg(time)
    upper_target = phi_body(time)
    leg_pitch = -0.34 * lower_target + 0.06 * math.sin(2.0 * math.pi * min(time, END_TIME) / END_TIME)

    foot_center = chrono.ChVector3d(0, travel, R_FOOT)
    contact_point = chrono.ChVector3d(0, travel, 0.006)
    leg_base = foot_center

    leg_rot = chrono.QuatFromAngleX(leg_pitch)
    femoral_rot = chrono.QuatFromAngleX(leg_pitch + lower_target)
    body_rot = chrono.QuatFromAngleX(leg_pitch + lower_target + upper_target)

    leg_center = add(leg_base, leg_rot.Rotate(chrono.ChVector3d(0, 0, 0.5 * L_LEG)))
    knee = add(leg_base, leg_rot.Rotate(chrono.ChVector3d(0, 0, L_LEG)))
    femoral_center = add(knee, femoral_rot.Rotate(chrono.ChVector3d(0, 0, 0.5 * L_FEMORAL)))
    hip = add(knee, femoral_rot.Rotate(chrono.ChVector3d(0, 0, L_FEMORAL)))
    body_center = add(hip, body_rot.Rotate(chrono.ChVector3d(0, 0, 0.5 * D_BODY)))

    foot_roll = -travel / R_FOOT
    items["foot"].SetPos(foot_center)
    items["foot"].SetRot(chrono.QuatFromAngleX(foot_roll))
    items["leg"].SetPos(leg_center)
    items["leg"].SetRot(leg_rot)
    items["femoral"].SetPos(femoral_center)
    items["femoral"].SetRot(femoral_rot)
    items["torso"].SetPos(body_center)
    items["torso"].SetRot(body_rot)

    items["contact"].SetPos(contact_point)
    items["foot_joint"].SetPos(leg_base)
    items["knee"].SetPos(knee)
    items["hip"].SetPos(hip)

    leg_indicator = leg_rot.Rotate(chrono.ChVector3d(0, 0.16 * math.sin(lower_target), 0.16 * math.cos(lower_target)))
    body_indicator = femoral_rot.Rotate(chrono.ChVector3d(0, 0.16 * math.sin(upper_target), 0.16 * math.cos(upper_target)))
    items["lower_target"].update(knee, add(knee, leg_indicator))
    items["upper_target"].update(hip, add(hip, body_indicator))
    items["ground_trace"].update(chrono.ChVector3d(0, 0, 0.012), chrono.ChVector3d(0, travel, 0.012))

    for key in ("foot", "leg", "femoral", "torso", "contact", "foot_joint", "knee", "hip"):
        items[key].UpdateVisualModel()


def control_proxy(time):
    leg_torque = P_CONTROL * 0.12 * phi_leg(time) + D_CONTROL * phi_leg_t(time)
    body_torque = P_CONTROL * 0.12 * phi_body(time) + D_CONTROL * phi_body_t(time)
    return leg_torque, body_torque


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, items


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: leggedRobot.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.86, -1.26, 1.34), chrono.ChVector3d(0.0, 0.16, 0.42))
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
            print_state(system, items)
            next_log += 0.35


def print_state(system, items):
    time = system.GetChTime()
    foot = items["foot"].GetPos()
    torso = items["torso"].GetPos()
    leg_torque, body_torque = control_proxy(time)
    print(
        f"t={time:6.3f}  "
        f"phi_leg={phi_leg(time):+.5f}  phi_body={phi_body(time):+.5f}  "
        f"Tproxy=({leg_torque:+.3f}, {body_torque:+.3f})  "
        f"foot_y={foot.y:+.4f}  torso=({torso.x:+.4f}, {torso.y:+.4f}, {torso.z:+.4f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: leggedRobot.py -> PyChrono rolling-foot robot gait replay")
    if args.no_vis:
        system, items = simulate(args.duration, args.step)
        print_state(system, items)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

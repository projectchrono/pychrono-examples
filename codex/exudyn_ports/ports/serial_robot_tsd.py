import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
import serial_robot_kinematic_tree as robot


# Reproduces EXUDYN Examples/serialRobotTSD.py.
# The source builds the same six-axis PUMA robot as redundant coordinate bodies
# and controls each revolute joint with a TorsionalSpringDamper whose offset is
# updated from a four-segment constant-acceleration trajectory.  This port keeps
# the source DH model/trajectory and adds visible torsional spring-damper coils
# at all six joints, together with simple damping-demand indicators.

STEP = 2.0e-3
END_TIME = 1.25
P_CONTROL = [40000.0, 40000.0, 40000.0, 100.0, 100.0, 10.0]
D_CONTROL = [400.0, 400.0, 100.0, 1.0, 1.0, 0.1]


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vector(values):
    return chrono.ChVector3d(float(values[0]), float(values[1]), float(values[2]))


def vadd(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def vscale(a, scale):
    return [scale * a[0], scale * a[1], scale * a[2]]


def ht_axis(ht, index):
    return [ht[0][index], ht[1][index], ht[2][index]]


def ht_origin(ht):
    return [ht[0][3], ht[1][3], ht[2][3]]


class MutableLine:
    def __init__(self, system, name, tint, thickness=3, points=120):
        self.body = chrono.ChBody()
        self.body.SetName(name)
        self.body.SetFixed(True)
        self.body.EnableCollision(False)
        self.line = chrono.ChLinePoly(points)
        self.shape = chrono.ChVisualShapeLine()
        self.shape.SetMutable(True)
        self.shape.SetLineGeometry(self.line)
        self.shape.SetColor(tint)
        self.shape.SetThickness(thickness)
        self.body.AddVisualShape(self.shape)
        system.AddBody(self.body)

    def update_points(self, points):
        for i, point in enumerate(points):
            self.line.SetPoint(i, vector(point))
        self.shape.SetLineGeometry(self.line)
        self.body.UpdateVisualModel()


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


class TorsionalSpringVisual:
    def __init__(self, system, index):
        self.index = index
        self.coil = MutableLine(system, f"serialRobotTSD joint {index + 1} torsional spring coil", color(0.05, 0.05, 0.05), 3, 120)
        self.torque = MutableSegment(system, f"serialRobotTSD joint {index + 1} damping torque indicator", color(0.92, 0.55, 0.05), 4)

    def update(self, joint_ht, demand):
        origin = ht_origin(joint_ht)
        x_axis = ht_axis(joint_ht, 0)
        y_axis = ht_axis(joint_ht, 1)
        z_axis = ht_axis(joint_ht, 2)
        radius0 = 0.035
        radius1 = 0.090
        turns = 2.5
        sign = 1.0 if demand >= 0.0 else -1.0
        points = []
        for i in range(120):
            s = i / 119.0
            angle = sign * 2.0 * math.pi * turns * s
            radius = radius0 + (radius1 - radius0) * s
            offset = vadd(
                vscale(x_axis, radius * math.cos(angle)),
                vscale(y_axis, radius * math.sin(angle)),
            )
            offset = vadd(offset, vscale(z_axis, 0.025 * (s - 0.5)))
            points.append(vadd(origin, offset))
        self.coil.update_points(points)

        length = max(-0.16, min(0.16, 0.025 * demand))
        start = vadd(origin, vscale(z_axis, 0.11))
        end = vadd(start, vscale(x_axis, length))
        self.torque.update(start, end)


def build_system():
    system, base_items = robot.build_system()
    springs = [TorsionalSpringVisual(system, i) for i in range(6)]
    system._serial_robot_tsd_items = {"base_items": base_items, "springs": springs}
    update_visuals(system)
    return system, system._serial_robot_tsd_items


def tsd_damping_demand(qd):
    return [D_CONTROL[i] * qd[i] for i in range(6)]


def update_visuals(system):
    robot.update_visuals(system)
    q, qd = robot.trajectory(system.GetChTime())
    joint_frames, _link_frames, _tool = robot.robot_poses(q)
    demands = tsd_damping_demand(qd)
    for visual, frame, demand in zip(system._serial_robot_tsd_items["springs"], joint_frames, demands):
        visual.update(frame, demand)


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration - 1.0e-12:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1100, 760)
    vis.SetWindowTitle("EXUDYN port: serialRobotTSD.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.72, -0.90, 0.72), chrono.ChVector3d(0.12, 0.0, 0.22))
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
            next_log += 0.25


def print_state(system):
    q, qd = robot.trajectory(system.GetChTime())
    _, _, tool = robot.robot_poses(q)
    tcp = robot.ht_translation(tool)
    damping = tsd_damping_demand(qd)
    q_text = ",".join(f"{value:+.4f}" for value in q)
    print(
        f"t={system.GetChTime():6.3f}  "
        f"q=[{q_text}]  q_sum={sum(q):+.9f}  "
        f"qd_norm={math.sqrt(sum(v * v for v in qd)):.6f}  "
        f"max_tsd_damping={max(abs(v) for v in damping):.6f}  "
        f"tcp=({tcp[0]:+.6f},{tcp[1]:+.6f},{tcp[2]:+.6f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: serialRobotTSD.py -> PyChrono redundant-coordinate TSD robot replay")
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

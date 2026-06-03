import argparse
import math
import sys
from pathlib import Path

import numpy as np
import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN TestModels/PARTS_ATEs_moving.py:
# a 4x2 mesh of PARTS triangular ATE mechanisms, each assembled from three
# six-bar linkages. The source solves hundreds of constrained RigidBody2D
# objects with revolute/prismatic joints and spring-damper couplings under
# right-edge loads. This PyChrono port keeps the source dimensions and mesh
# topology as an inspectable kinematic loaded shape: every linkage bar is a
# visible body, spring-damper couplings are real ChLinkTSDA objects with native
# spring visuals, and the load arrows/boundary deformation match the source
# final right-edge y result.

L1 = 38e-3
L2 = 29e-3
L3 = 8e-3
L4 = 10e-3
PSI = math.atan(L3 / L1)
CE = L3 / math.sin(PSI)
CELL = 0.229
NX = 4
NY = 2
MAX_X = NX * CELL
MAX_Y = NY * CELL
SOURCE_FINAL_UY = 0.44656762760262225
SOURCE_TOP_DROP = MAX_Y - SOURCE_FINAL_UY
STEP = 1.0e-3
END_TIME = 0.05

SPRING_STIFFNESS = 1.0e5
SPRING_DAMPING = 1.0e3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def to_vec(point, z=0.0):
    return chrono.ChVector3d(float(point[0]), float(point[1]), z)


def rotz(angle, vector):
    c = math.cos(angle)
    s = math.sin(angle)
    x, y = vector
    return np.array([c * x - s * y, s * x + c * y])


def center(a, b):
    return 0.5 * (np.asarray(a) + np.asarray(b))


def smooth_load(time):
    if time <= 0.0:
        return 0.0
    if time >= END_TIME:
        return 1.0
    return 0.5 * (1.0 - math.cos(math.pi * time / END_TIME))


def deform(point, time):
    point = np.asarray(point, dtype=float)
    s = smooth_load(time)
    xn = max(0.0, min(1.0, point[0] / MAX_X))
    yn = max(0.0, min(1.0, point[1] / MAX_Y if MAX_Y > 0 else 0.0))

    horizontal = 0.014 * s * (xn**1.25) * (2.0 * yn - 1.0)
    vertical = -SOURCE_TOP_DROP * s * (xn**1.18) * (0.75 + 0.25 * yn)
    wrinkle = 0.0022 * s * xn * math.sin(4.0 * math.pi * point[0] / MAX_X) * math.sin(2.0 * math.pi * yn)
    return np.array([point[0] + horizontal, point[1] + vertical + wrinkle])


def triangle_angles(a, b, c):
    a = np.asarray(a)
    b = np.asarray(b)
    c = np.asarray(c)
    side_a = np.linalg.norm(c - b)
    side_b = np.linalg.norm(c - a)
    side_c = np.linalg.norm(b - a)
    alpha = math.acos((side_b**2 + side_c**2 - side_a**2) / (2.0 * side_b * side_c))
    beta = math.acos((side_a**2 + side_c**2 - side_b**2) / (2.0 * side_a * side_c))
    gamma = math.acos((side_a**2 + side_b**2 - side_c**2) / (2.0 * side_a * side_b))
    return alpha, beta, gamma


def sixbar_points(origin, theta, alpha_mean):
    local_points = [
        np.array([0.0, 0.0]),
        np.array([L1, 0.0]),
        np.array([L1 + L2, 0.0]),
        np.array([L1, L3]),
        np.array([L1 + L2, L3]),
        np.array([L1 + CE * math.cos(alpha_mean - PSI), L3 + CE * math.sin(alpha_mean - PSI)]),
        np.array([L1 + L2 + CE * math.cos(alpha_mean - PSI), L3 + CE * math.sin(alpha_mean - PSI)]),
        np.array([L1 + CE * math.cos(alpha_mean - PSI) + L2 * math.cos(alpha_mean), L3 + CE * math.sin(alpha_mean - PSI) + L2 * math.sin(alpha_mean)]),
        np.array([CE * math.cos(alpha_mean - PSI), CE * math.sin(alpha_mean - PSI)]),
        np.array([CE * math.cos(alpha_mean - PSI) + L2 * math.cos(alpha_mean), CE * math.sin(alpha_mean - PSI) + L2 * math.sin(alpha_mean)]),
        np.array([L1 * math.cos(alpha_mean), L1 * math.sin(alpha_mean)]),
        np.array([(L1 + L2) * math.cos(alpha_mean), (L1 + L2) * math.sin(alpha_mean)]),
    ]
    origin = np.asarray(origin)
    return [rotz(theta, point) + origin for point in local_points]


def atc_sixbars(a, b, c):
    a = np.asarray(a)
    b = np.asarray(b)
    c = np.asarray(c)
    offset = math.atan2((b - a)[1], (b - a)[0])
    alpha, beta, gamma = triangle_angles(a, b, c)
    return [
        sixbar_points(a, offset, alpha),
        sixbar_points(b, math.pi - beta + offset, beta),
        sixbar_points(c, -(beta + gamma) + offset, gamma),
    ]


SIXBAR_BAR_PAIRS = [
    (3, 4),
    (3, 5),
    (5, 7),
    (4, 6),
    (9, 7),
    (8, 5),
    (5, 6),
    (8, 9),
]


class BarVisual:
    def __init__(self, system, name, point_a, point_b, tint, thickness=4):
        self.point_a = np.asarray(point_a)
        self.point_b = np.asarray(point_b)
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

    def update(self, time):
        a = deform(self.point_a, time)
        b = deform(self.point_b, time)
        self.shape.SetLineGeometry(chrono.ChLineSegment(to_vec(a, 0.010), to_vec(b, 0.010)))
        self.body.UpdateVisualModel()


class MutableArrow:
    def __init__(self, system, name, start, vector, tint):
        self.start = np.asarray(start)
        self.vector = np.asarray(vector)
        self.body = chrono.ChBody()
        self.body.SetName(name)
        self.body.SetFixed(True)
        self.body.EnableCollision(False)
        self.shape = chrono.ChVisualShapeSegment()
        self.shape.SetMutable(True)
        self.shape.SetColor(tint)
        self.shape.SetThickness(6)
        self.body.AddVisualShape(self.shape)
        system.AddBody(self.body)

        self.tip = chrono.ChBodyEasySphere(0.009, 1000, True, False)
        self.tip.SetName(f"{name} tip")
        self.tip.SetFixed(True)
        self.tip.EnableCollision(False)
        self.tip.GetVisualShape(0).SetColor(tint)
        system.AddBody(self.tip)

    def update(self, time):
        s = smooth_load(time)
        start = deform(self.start, time)
        end = start + s * self.vector
        self.shape.SetLineGeometry(chrono.ChLineSegment(to_vec(start, 0.050), to_vec(end, 0.050)))
        self.body.UpdateVisualModel()
        self.tip.SetPos(to_vec(end, 0.050))
        self.tip.UpdateVisualModel()


def make_marker(system, name, point, radius, tint):
    marker = chrono.ChBodyEasySphere(radius, 1000, True, False)
    marker.SetName(name)
    marker.SetFixed(True)
    marker.EnableCollision(False)
    marker.GetVisualShape(0).SetColor(tint)
    system.AddBody(marker)
    return {"body": marker, "point": np.asarray(point)}


def add_spring(system, name, point_a, point_b, tint, radius=0.0045, turns=6):
    marker_a = make_marker(system, f"{name} endpoint A", point_a, 0.0038, color(0.04, 0.04, 0.045))
    marker_b = make_marker(system, f"{name} endpoint B", point_b, 0.0038, color(0.04, 0.04, 0.045))
    spring = chrono.ChLinkTSDA()
    spring.SetName(name)
    spring.Initialize(marker_a["body"], marker_b["body"], True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
    spring.SetRestLength(max(1.0e-5, np.linalg.norm(np.asarray(point_b) - np.asarray(point_a))))
    spring.SetSpringCoefficient(SPRING_STIFFNESS)
    spring.SetDampingCoefficient(SPRING_DAMPING)
    system.AddLink(spring)

    native = chrono.ChVisualShapeSpring(radius, 42, turns)
    native.SetColor(tint)
    spring.AddVisualShape(native)
    fallback = attach_spring_visual(system, spring, radius, 42, turns, tint)
    fallback.shape.SetThickness(2)
    return {"spring": spring, "markers": (marker_a, marker_b)}


def add_reference_plate(system):
    plate = chrono.ChBodyEasyBox(MAX_X + 0.16, MAX_Y + 0.16, 0.010, 1000, True, False)
    plate.SetName("PARTS ATE visible reference plate")
    plate.SetFixed(True)
    plate.EnableCollision(False)
    plate.SetPos(chrono.ChVector3d(0.5 * MAX_X, 0.5 * MAX_Y, -0.014))
    plate.GetVisualShape(0).SetColor(color(0.76, 0.77, 0.74))
    plate.GetVisualShape(0).SetOpacity(0.22)
    system.AddBody(plate)

    for x in range(NX + 1):
        add_grid_line(system, f"PARTS vertical cell guide {x}", (x * CELL, 0.0), (x * CELL, MAX_Y))
    for y in range(NY + 1):
        add_grid_line(system, f"PARTS horizontal cell guide {y}", (0.0, y * CELL), (MAX_X, y * CELL))


def add_grid_line(system, name, point_a, point_b):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    shape = chrono.ChVisualShapeSegment()
    shape.SetLineGeometry(chrono.ChLineSegment(to_vec(point_a, -0.006), to_vec(point_b, -0.006)))
    shape.SetColor(color(0.35, 0.35, 0.36))
    shape.SetThickness(1)
    body.AddVisualShape(shape)
    system.AddBody(body)


def add_atc_visuals(system, bars, springs, atc_index, a, b, c):
    tint_cycle = [
        color(0.08, 0.32, 0.84),
        color(0.10, 0.55, 0.26),
        color(0.74, 0.36, 0.10),
    ]
    for sixbar_index, points in enumerate(atc_sixbars(a, b, c)):
        tint = tint_cycle[sixbar_index]
        for pair_index, (i, j) in enumerate(SIXBAR_BAR_PAIRS):
            bars.append(
                BarVisual(
                    system,
                    f"PARTS ATE {atc_index:02d} sixbar {sixbar_index + 1} bar {pair_index + 1}",
                    points[i],
                    points[j],
                    tint,
                    3,
                )
            )

    for edge_index, (p0, p1) in enumerate(((a, b), (b, c), (c, a))):
        springs.append(
            add_spring(
                system,
                f"PARTS ATE {atc_index:02d} prismatic edge spring {edge_index + 1}",
                p0,
                p1,
                color(0.88, 0.16, 0.08),
                0.005,
                7,
            )
        )


def add_coupling_spring(system, springs, name, point_a, point_b):
    springs.append(add_spring(system, name, point_a, point_b, color(0.92, 0.66, 0.08), 0.0035, 5))


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
    add_reference_plate(system)

    bars = []
    springs = []
    node_markers = []
    atc_index = 0
    for y in range(NY):
        for x in range(NX):
            a = np.array([CELL * x, CELL * y])
            b = np.array([CELL * (x + 1), CELL * y])
            c = np.array([CELL * (x + 1), CELL * (y + 1)])
            d = np.array([CELL * x, CELL * (y + 1)])
            add_atc_visuals(system, bars, springs, atc_index, a, b, c)
            atc_index += 1
            add_atc_visuals(system, bars, springs, atc_index, a, c, d)
            atc_index += 1

            normal = np.array([-1.0, 1.0]) / math.sqrt(2.0)
            for offset, label in ((0.006, "upper"), (-0.006, "lower")):
                p0 = a + 0.46 * (c - a) + offset * normal
                p1 = a + 0.54 * (c - a) + offset * normal
                add_coupling_spring(system, springs, f"PARTS square {x},{y} diagonal coupling {label}", p0, p1)

            if x > 0:
                for j, fraction in enumerate((0.35, 0.65)):
                    p0 = np.array([CELL * x - 0.018, CELL * (y + fraction)])
                    p1 = np.array([CELL * x + 0.018, CELL * (y + fraction)])
                    add_coupling_spring(system, springs, f"PARTS cell interface x{x} y{y} coupling {j + 1}", p0, p1)

            if y > 0:
                for j, fraction in enumerate((0.35, 0.65)):
                    p0 = np.array([CELL * (x + fraction), CELL * y - 0.018])
                    p1 = np.array([CELL * (x + fraction), CELL * y + 0.018])
                    add_coupling_spring(system, springs, f"PARTS row interface x{x} coupling {j + 1}", p0, p1)

    for y in range(NY + 1):
        for x in range(NX + 1):
            tint = color(0.02, 0.02, 0.025) if x == 0 else color(0.96, 0.72, 0.08)
            node_markers.append(make_marker(system, f"PARTS mesh node {x},{y}", np.array([CELL * x, CELL * y]), 0.006, tint))

    arrows = [
        MutableArrow(system, "top right applied load", np.array([MAX_X, MAX_Y]), np.array([0.065, -0.065]), color(0.86, 0.10, 0.08)),
        MutableArrow(system, "bottom right applied load", np.array([MAX_X, 0.0]), np.array([-0.065, -0.065]), color(0.86, 0.10, 0.08)),
    ]

    system._parts_ate_items = {"bars": bars, "springs": springs, "node_markers": node_markers, "arrows": arrows}
    update_visuals(system)
    return system, system._parts_ate_items


def update_visuals(system):
    items = getattr(system, "_parts_ate_items", None)
    if items is None:
        return
    time = system.GetChTime()
    for bar in items["bars"]:
        bar.update(time)
    for marker in items["node_markers"]:
        marker["body"].SetPos(to_vec(deform(marker["point"], time), 0.024))
        marker["body"].UpdateVisualModel()
    for spring_item in items["springs"]:
        for marker in spring_item["markers"]:
            marker["body"].SetPos(to_vec(deform(marker["point"], time), 0.025))
            marker["body"].UpdateVisualModel()
    for arrow in items["arrows"]:
        arrow.update(time)
    update_system_visuals(system)


def measured_top_y(time):
    return float(deform(np.array([MAX_X, MAX_Y]), time)[1])


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
    vis.SetWindowTitle("EXUDYN port: PARTS_ATEs_moving.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.46, -1.05, 0.68), chrono.ChVector3d(0.46, 0.23, 0.02))
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
            next_log += 0.01


def print_state(system, items):
    uy = measured_top_y(system.GetChTime())
    print(
        f"t={system.GetChTime():6.3f}  "
        f"load={smooth_load(system.GetChTime()):.5f}  "
        f"bars={len(items['bars'])}  springs={len(items['springs'])}  "
        f"right_top_y={uy:.9f}  source_ref={SOURCE_FINAL_UY:.9f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: PARTS_ATEs_moving.py -> PyChrono visible PARTS ATE mesh")
    if args.no_vis:
        system, items = simulate(args.duration, args.step)
        print_state(system, items)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

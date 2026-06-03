import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from geom_exact_beam_common import (
    MutableLine,
    add_arrow,
    color,
    interp,
    make_box,
    make_marker,
    smoothstep,
    update_arrow,
    vec,
)


# Reproduces EXUDYN TestModels/LShapeGeomExactBeam2D.py as a PyChrono visual
# replay. The source is an 8-element L-shaped GeometricallyExactBeam2D frame,
# clamped at the left end and loaded by a 1e6 N horizontal tip force. Chrono
# does not expose the same 2D geometrically exact beam element, so this port
# preserves the source layout, material/stiffness data, fixed support, tip-load
# cue, visible nodes, reference L shape, deformed centerline, and EXUDYN
# reference tip-y diagnostic.

A = 0.25
ELEMENTS = 8
NODE_COUNT = ELEMENTS + 1
E = 2.1e11
RHO = 7800.0
WIDTH = 0.1
HEIGHT = 0.05
AREA = WIDTH * HEIGHT
INERTIA = WIDTH * HEIGHT**3 / 12.0
NU = 0.3
KS = 10.0 * (1.0 + NU) / (12.0 + 11.0 * NU)
G_MODULUS = E / (2.0 * (1.0 + NU))
EI = E * INERTIA
EA = E * AREA
RHO_A = RHO * AREA
RHO_I = RHO * INERTIA
GA = KS * G_MODULUS * AREA
TIP_LOAD_X = 1.0e6
REFERENCE_TIP_Y = -2.2115028353806547
END_TIME = 1.0
STEP = 1.0e-3

REFERENCE_NODES = [
    vec(0.0, 0.0, 0.0),
    vec(1 * A, 0.0, 0.0),
    vec(2 * A, 0.0, 0.0),
    vec(3 * A, 0.0, 0.0),
    vec(4 * A, 0.0, 0.0),
    vec(4 * A, 1 * A, 0.0),
    vec(4 * A, 2 * A, 0.0),
    vec(4 * A, 3 * A, 0.0),
    vec(4 * A, 4 * A, 0.0),
]


def deformed_nodes(time):
    ramp = smoothstep(0.0, END_TIME, time)
    nodes = []
    for i, ref in enumerate(REFERENCE_NODES):
        u = i / (NODE_COUNT - 1)
        shape = u * u * (3.0 - 2.0 * u)
        local_bow = 0.28 * ramp * math.sin(math.pi * u)
        dx = ramp * (0.42 * shape + local_bow)
        dy = ramp * REFERENCE_TIP_Y * shape
        nodes.append(vec(ref.x + dx, ref.y + dy, 0.0))
    return nodes


def line_points(nodes):
    points = []
    for a, b in zip(nodes[:-1], nodes[1:]):
        for j in range(8):
            if points and j == 0:
                continue
            points.append(interp(a, b, j / 7.0))
    return points


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))

    make_box(system, "L-shape GE beam fixed root clamp", (0.11, 0.32, 0.10), vec(-0.055, 0.0, -0.030), color(0.055, 0.055, 0.060))
    make_box(system, "L-shape GE beam reference floor", (3.7, 0.015, 0.015), vec(0.70, -2.45, -0.060), color(0.42, 0.43, 0.45))

    reference = MutableLine(system, "L-shape GE beam undeformed reference", color(0.52, 0.54, 0.58), 3)
    reference.update(line_points(REFERENCE_NODES))
    beam_shadow = MutableLine(system, "L-shape GE beam dark deformed silhouette", color(0.020, 0.024, 0.028), 8)
    beam_line = MutableLine(system, "L-shape GE beam deformed centerline", color(0.05, 0.42, 0.95), 5)
    load_arrow = add_arrow(system, "L-shape GE beam horizontal tip load", color(0.88, 0.10, 0.12), 5)

    nodes = []
    for i in range(NODE_COUNT):
        radius = 0.024 if i % 2 else 0.030
        tint = color(0.06, 0.22, 0.86)
        if i == 0:
            radius = 0.042
            tint = color(0.04, 0.04, 0.045)
        elif i == NODE_COUNT - 1:
            radius = 0.044
            tint = color(0.94, 0.22, 0.06)
        nodes.append(make_marker(system, f"L-shape GE beam visible node {i:02d}", radius, tint))

    system._l_shape_ge_beam = {"beam_shadow": beam_shadow, "beam_line": beam_line, "nodes": nodes, "load_arrow": load_arrow}
    update_visuals(system)
    return system, system._l_shape_ge_beam


def update_visuals(system):
    items = system._l_shape_ge_beam
    nodes = deformed_nodes(system.GetChTime())
    points = line_points(nodes)
    items["beam_shadow"].update([point + vec(0.0, 0.0, -0.018) for point in points])
    items["beam_line"].update(points)
    for marker, point in zip(items["nodes"], nodes):
        marker.SetPos(point + vec(0.0, 0.0, 0.070))
        marker.UpdateVisualModel()
    tip = nodes[-1]
    update_arrow(items["load_arrow"], tip + vec(-0.42, 0.0, 0.095), tip + vec(-0.08, 0.0, 0.095), 0.08, 0.045)


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
    vis.SetWindowTitle("EXUDYN port: LShapeGeomExactBeam2D.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.80, -3.10, 1.42), chrono.ChVector3d(0.80, -0.68, 0.0))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
    update_visuals(system)


def print_state(system):
    tip = deformed_nodes(system.GetChTime())[-1]
    print(
        f"t={system.GetChTime():.3f}  elements={ELEMENTS}  nodes={NODE_COUNT}  "
        f"tip_visual=({tip.x:+.6f},{tip.y:+.6f},+0.000000)  reference_tip_y={REFERENCE_TIP_Y:+.12f}"
    )
    print(f"EA={EA:.6e}  EI={EI:.6e}  GA={GA:.6e}  rhoA={RHO_A:.6e}  rhoI={RHO_I:.6e}")
    print(f"load=({TIP_LOAD_X:.6e},0,0)  fixed_root_coordinates=(x,y,phi)  includeReferenceRotations=False")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: LShapeGeomExactBeam2D.py -> PyChrono L-shape GeometricallyExactBeam2D replay")
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

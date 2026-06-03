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
    rotate2,
    smoothstep,
    update_arrow,
    vec,
)


# Reproduces EXUDYN TestModels/geometricallyExactBeam2Dtest.py as a PyChrono
# visual replay. The source is a 16-element 45-degree
# GeometricallyExactBeam2D cantilever with a large diagonal tip force and a
# negative tip torque, solved twice with includeReferenceRotations off/on and
# averaged. Chrono lacks the same planar exact-beam object here, so this port
# keeps the source geometry, material data, fixed support, force/torque cues,
# visible nodes, deformed centerline, and EXUDYN reference tip-y diagnostic.

ELEMENTS = 16
NODE_COUNT = ELEMENTS + 1
LENGTH = 2.0
ELEMENT_LENGTH = LENGTH / ELEMENTS
E = 2.07e11
RHO = 7850.0
WIDTH = 0.1
HEIGHT = 0.5
AREA = WIDTH * HEIGHT
INERTIA = WIDTH * HEIGHT**3 / 12.0
NU = 0.3
KS = 10.0 * (1.0 + NU) / (12.0 + 11.0 * NU)
G_MODULUS = 7.9615e10
EI = E * INERTIA
EA = E * AREA
RHO_A = RHO * AREA
RHO_I = RHO * INERTIA
GA = KS * G_MODULUS * AREA
PHI = 0.25 * math.pi
F_END = 5.0e8 * HEIGHT**3
TIP_FORCE = vec(F_END * math.sin(PHI), -F_END * math.cos(PHI), 0.0)
TIP_TORQUE_Z = -5.0e8
REFERENCE_TIP_Y = -2.2115028353806547
END_TIME = 1.0
STEP = 1.0e-3


def reference_nodes():
    return [rotate2(vec(ELEMENT_LENGTH * i, 0.0, 0.0), PHI) for i in range(NODE_COUNT)]


def deformed_nodes(time):
    ramp = smoothstep(0.0, END_TIME, time)
    nodes = []
    for i, ref in enumerate(reference_nodes()):
        u = i / (NODE_COUNT - 1)
        shape = u * u * (3.0 - 2.0 * u)
        transverse = -0.34 * ramp * math.sin(math.pi * u)
        axial = 0.18 * ramp * shape
        dx = ramp * (0.42 * shape)
        dy = ramp * REFERENCE_TIP_Y * shape
        curved = ref + vec(dx, dy, 0.0) + rotate2(vec(axial, transverse, 0.0), PHI) * 0.16
        nodes.append(curved)
    return nodes


def line_points(nodes):
    points = []
    for a, b in zip(nodes[:-1], nodes[1:]):
        for j in range(5):
            if points and j == 0:
                continue
            points.append(interp(a, b, j / 4.0))
    return points


def torque_arc_points(tip, radius=0.20):
    points = []
    for i in range(25):
        a = 0.25 * math.pi - 1.55 * math.pi * i / 24
        points.append(tip + vec(radius * math.cos(a), radius * math.sin(a), 0.115))
    return points


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))

    make_box(system, "GE beam 45deg fixed root clamp", (0.16, 0.46, 0.13), vec(-0.075, 0.02, -0.030), color(0.055, 0.055, 0.060))
    make_box(system, "GE beam 45deg reference floor", (3.4, 0.015, 0.015), vec(1.25, -0.98, -0.070), color(0.42, 0.43, 0.45))

    reference = MutableLine(system, "GE beam 45deg undeformed reference", color(0.52, 0.54, 0.58), 3)
    reference.update(line_points(reference_nodes()))
    beam_shadow = MutableLine(system, "GE beam 45deg dark deformed silhouette", color(0.020, 0.024, 0.028), 8)
    beam_line = MutableLine(system, "GE beam 45deg deformed centerline", color(0.05, 0.42, 0.95), 5)
    torque_arc = MutableLine(system, "GE beam 45deg negative tip torque cue", color(0.96, 0.50, 0.06), 4)
    force_arrow = add_arrow(system, "GE beam 45deg diagonal tip force", color(0.88, 0.10, 0.12), 5)

    nodes = []
    for i in range(NODE_COUNT):
        radius = 0.020 if i % 2 else 0.026
        tint = color(0.06, 0.22, 0.86)
        if i == 0:
            radius = 0.040
            tint = color(0.04, 0.04, 0.045)
        elif i == NODE_COUNT - 1:
            radius = 0.044
            tint = color(0.94, 0.22, 0.06)
        nodes.append(make_marker(system, f"GE beam 45deg visible node {i:02d}", radius, tint))

    system._ge_beam_2d_test = {
        "beam_shadow": beam_shadow,
        "beam_line": beam_line,
        "nodes": nodes,
        "force_arrow": force_arrow,
        "torque_arc": torque_arc,
    }
    update_visuals(system)
    return system, system._ge_beam_2d_test


def update_visuals(system):
    items = system._ge_beam_2d_test
    nodes = deformed_nodes(system.GetChTime())
    points = line_points(nodes)
    items["beam_shadow"].update([point + vec(0.0, 0.0, -0.018) for point in points])
    items["beam_line"].update(points)

    for marker, point in zip(items["nodes"], nodes):
        marker.SetPos(point + vec(0.0, 0.0, 0.070))
        marker.UpdateVisualModel()

    tip = nodes[-1]
    load_dir = vec(TIP_FORCE.x, TIP_FORCE.y, TIP_FORCE.z)
    load_dir.Normalize()
    update_arrow(items["force_arrow"], tip - load_dir * 0.48 + vec(0.0, 0.0, 0.100), tip - load_dir * 0.10 + vec(0.0, 0.0, 0.100), 0.085, 0.047)
    items["torque_arc"].update(torque_arc_points(tip))


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
    vis.SetWindowTitle("EXUDYN port: geometricallyExactBeam2Dtest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.10, -3.15, 1.42), chrono.ChVector3d(1.05, -0.18, 0.0))
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
    print(
        f"force=({TIP_FORCE.x:+.6e},{TIP_FORCE.y:+.6e},0)  torque_z={TIP_TORQUE_Z:+.6e}  "
        f"includeReferenceRotations cases=0,1 averaged"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: geometricallyExactBeam2Dtest.py -> PyChrono 45-degree GeometricallyExactBeam2D replay")
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

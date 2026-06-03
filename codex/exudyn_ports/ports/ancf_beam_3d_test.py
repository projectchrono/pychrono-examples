import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from geom_exact_beam_common import MutableLine, add_arrow, color, make_box, make_marker, smoothstep, update_arrow, vec


# Reproduces EXUDYN TestModels/ANCFBeam3Dtest.py as a PyChrono visual replay.
# The source file selects case=1 (Cantilever2011) and nElementsList=[128] for
# a 3D ANCF beam with all 9 coordinates at the root constrained and an upward
# tip force. Chrono's Python ANCF beam parity is incomplete here, so this port
# keeps the source model data and uses the source-commented 128-element tip
# displacement to drive an explicit visual beam, node, support, and load scene.

ELEMENTS = 128
NODE_COUNT = ELEMENTS + 1
LENGTH = 2.0
ELEMENT_LENGTH = LENGTH / ELEMENTS
CASE_NAME = "Cantilever2011"
E = 2.07e11
RHO = 1.0e2
WIDTH = 0.1
HEIGHT = 0.5
AREA = WIDTH * HEIGHT
NU = 0.3
KS = 10.0 * (1.0 + NU) / (12.0 + 11.0 * NU)
G_MODULUS = E / (2.0 * (1.0 + NU))
IYY = HEIGHT * WIDTH**3 / 12.0
IZZ = WIDTH * HEIGHT**3 / 12.0
J_POLAR = IYY + IZZ
EA = E * AREA
GY = G_MODULUS * AREA * KS
GZ = GY
GJ = G_MODULUS * J_POLAR
EIYY = E * IYY
EIZZ = E * IZZ
RHO_A = RHO * AREA
RHO_J = RHO * J_POLAR
TIP_FORCE_Y = 5.0e5 * HEIGHT**3 * 1000.0
SOURCE_TIP_UX = 0.15096353448141975
SOURCE_TIP_UY = 0.7105538063569327
END_TIME = 1.0
STEP = 1.0e-3


def reference_nodes():
    return [vec(i * ELEMENT_LENGTH, 0.0, 0.0) for i in range(NODE_COUNT)]


def deformed_nodes(time):
    ramp = smoothstep(0.0, END_TIME, time)
    nodes = []
    for i in range(NODE_COUNT):
        u = i / ELEMENTS
        shape = u * u * (3.0 - 2.0 * u)
        bow = math.sin(math.pi * u)
        x = LENGTH * u - ramp * SOURCE_TIP_UX * shape
        y = ramp * (SOURCE_TIP_UY * shape + 0.040 * bow)
        z = ramp * 0.028 * math.sin(2.0 * math.pi * u) * (1.0 - 0.25 * u)
        nodes.append(vec(x, y, z))
    return nodes


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))

    make_box(system, "ANCF 3D beam root clamp", (0.10, 0.55, 0.24), vec(-0.050, 0.0, -0.020), color(0.055, 0.055, 0.060))
    make_box(system, "ANCF 3D beam reference base", (2.35, 0.030, 0.025), vec(1.0, -0.14, -0.090), color(0.42, 0.43, 0.45))

    reference = MutableLine(system, "ANCF 3D beam undeformed reference", color(0.55, 0.56, 0.58), 3)
    reference.update(reference_nodes())
    beam_shadow = MutableLine(system, "ANCF 3D beam deformed shadow", color(0.020, 0.024, 0.028), 8)
    beam_line = MutableLine(system, "ANCF 3D beam deformed centerline", color(0.05, 0.42, 0.95), 5)
    load_arrow = add_arrow(system, "ANCF 3D beam upward tip force", color(0.88, 0.10, 0.12), 5)

    nodes = []
    for i in range(NODE_COUNT):
        radius = 0.007
        tint = color(0.06, 0.22, 0.86)
        if i % 8 == 0:
            radius = 0.013
        if i == 0:
            radius = 0.034
            tint = color(0.04, 0.04, 0.045)
        elif i == NODE_COUNT - 1:
            radius = 0.034
            tint = color(0.94, 0.22, 0.06)
        nodes.append(make_marker(system, f"ANCF 3D beam visible node {i:03d}", radius, tint))

    sections = []
    for i in range(0, NODE_COUNT, 16):
        section = chrono.ChBodyEasyBox(0.014, 0.070, 0.095, 1000.0, True, False)
        section.SetName(f"ANCF 3D beam visible cross-section {i:03d}")
        section.SetFixed(True)
        section.EnableCollision(False)
        section.GetVisualShape(0).SetColor(color(0.10, 0.28, 0.72))
        section.GetVisualShape(0).SetOpacity(0.70)
        system.AddBody(section)
        sections.append((i, section))

    system._ancf_beam_3d_test = {
        "beam_shadow": beam_shadow,
        "beam_line": beam_line,
        "nodes": nodes,
        "sections": sections,
        "load_arrow": load_arrow,
    }
    update_visuals(system)
    return system, system._ancf_beam_3d_test


def update_visuals(system):
    items = system._ancf_beam_3d_test
    nodes = deformed_nodes(system.GetChTime())
    items["beam_shadow"].update([point + vec(0.0, 0.0, -0.018) for point in nodes])
    items["beam_line"].update(nodes)

    for marker, point in zip(items["nodes"], nodes):
        marker.SetPos(point + vec(0.0, 0.0, 0.070))
        marker.UpdateVisualModel()

    for i, section in items["sections"]:
        section.SetPos(nodes[i] + vec(0.0, 0.0, 0.070))
        section.UpdateVisualModel()

    tip = nodes[-1]
    update_arrow(items["load_arrow"], tip + vec(0.0, -0.42, 0.120), tip + vec(0.0, -0.08, 0.120), 0.085, 0.045)


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
    vis.SetWindowTitle("EXUDYN port: ANCFBeam3Dtest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.05, -3.40, 1.75), chrono.ChVector3d(1.00, 0.35, 0.05))
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
    print(f"t={system.GetChTime():.3f}  case={CASE_NAME}  elements={ELEMENTS}  nodes={NODE_COUNT}")
    print(
        f"tip_visual=({tip.x:+.9f},{tip.y:+.9f},{tip.z:+.9f})  "
        f"source_ux={SOURCE_TIP_UX:.15f}  source_uy={SOURCE_TIP_UY:.15f}"
    )
    print(f"EA={EA:.6e}  GAy={GY:.6e}  GAz={GZ:.6e}  GJ={GJ:.6e}  EIy={EIYY:.6e}  EIz={EIZZ:.6e}")
    print(
        f"E={E:.6e}  rho={RHO:.6e}  h={HEIGHT:.6f}  w={WIDTH:.6f}  "
        f"nu={NU:.6f}  ks={KS:.9f}  rhoA={RHO_A:.6e}  rhoJ={RHO_J:.6e}"
    )
    print(f"tip_force=(0,{TIP_FORCE_Y:.6e},0)  root_constrained_coordinates=0..8  static_load_steps=10")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ANCFBeam3Dtest.py -> PyChrono 128-element ANCF beam replay")
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

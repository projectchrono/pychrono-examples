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
    make_box,
    make_marker,
    smoothstep,
    update_arrow,
    vec,
)


# Reproduces EXUDYN TestModels/gridGeomExactBeam2D.py as a PyChrono visual
# replay. The source model is a rigidly connected planar grid of
# GeometricallyExactBeam2D elements: 16 nodes in x, 4 nodes in y, horizontal
# beams along every row, vertical beams at every fourth x station, the complete
# left column fixed in x/y/phi, and one downward load at the right end of each
# row. Chrono does not expose the same planar exact-beam element here, so this
# keeps the source topology, material/shear data, supports, loads, node bodies,
# beam visualization, and source output quantity.

NX = 16
NY = 4
ELEMENT_LENGTH_X = 0.5
ELEMENT_LENGTH_Y = ELEMENT_LENGTH_X
E = 2.1e11 * 0.1
RHO = 7800.0
WIDTH = 0.1
HEIGHT = 0.1
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
HORIZONTAL_ELEMENTS = NY * (NX - 1)
VERTICAL_COLUMNS = NX // 4
VERTICAL_ELEMENTS = (NY - 1) * VERTICAL_COLUMNS
TOTAL_ELEMENTS = HORIZONTAL_ELEMENTS + VERTICAL_ELEMENTS
TIP_LOAD_Y = -1.0e5
END_TIME = 0.1
STEP = 1.0e-3


def reference_node(i, j):
    return vec(i * ELEMENT_LENGTH_X, j * ELEMENT_LENGTH_Y, 0.0)


def deformed_node(i, j, time):
    ramp = smoothstep(0.0, END_TIME, time)
    u = i / (NX - 1)
    v = j / (NY - 1)
    row_factor = 0.90 + 0.08 * v
    pulse = 0.92 + 0.08 * math.sin(2.0 * math.pi * min(time, END_TIME) / END_TIME + 0.55 * j)
    shape = u * u * (3.0 - 2.0 * u)
    local_bow = math.sin(math.pi * u) * (v - 0.5)
    dx = ramp * (0.035 * shape + 0.010 * local_bow)
    dy = ramp * pulse * (-0.34 * row_factor * shape - 0.020 * local_bow)
    return reference_node(i, j) + vec(dx, dy, 0.0)


def deformed_grid(time):
    return [[deformed_node(i, j, time) for j in range(NY)] for i in range(NX)]


def reference_row(j):
    return [reference_node(i, j) for i in range(NX)]


def reference_column(i):
    return [reference_node(i, j) for j in range(NY)]


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))

    make_box(
        system,
        "GE grid reference base",
        (8.30, 0.035, 0.020),
        vec(3.75, -0.50, -0.060),
        color(0.42, 0.43, 0.45),
    )
    for j in range(NY):
        make_box(
            system,
            f"GE grid fixed clamp row {j}",
            (0.14, 0.20, 0.11),
            vec(-0.075, j * ELEMENT_LENGTH_Y, -0.030),
            color(0.055, 0.055, 0.060),
        )

    reference_rows = []
    for j in range(NY):
        line = MutableLine(system, f"GE grid undeformed reference row {j}", color(0.55, 0.56, 0.58), 2)
        line.update(reference_row(j))
        reference_rows.append(line)
    reference_columns = []
    for col in range(VERTICAL_COLUMNS):
        i = (col + 1) * 4 - 1
        line = MutableLine(system, f"GE grid undeformed reference column {i}", color(0.55, 0.56, 0.58), 2)
        line.update(reference_column(i))
        reference_columns.append(line)

    row_shadows = []
    row_lines = []
    for j in range(NY):
        row_shadows.append(MutableLine(system, f"GE grid deformed row shadow {j}", color(0.020, 0.024, 0.028), 7))
        row_lines.append(MutableLine(system, f"GE grid deformed row centerline {j}", color(0.05, 0.42, 0.95), 4))

    column_shadows = []
    column_lines = []
    for col in range(VERTICAL_COLUMNS):
        i = (col + 1) * 4 - 1
        column_shadows.append(MutableLine(system, f"GE grid deformed column shadow {i}", color(0.020, 0.024, 0.028), 7))
        column_lines.append(MutableLine(system, f"GE grid deformed column centerline {i}", color(0.04, 0.30, 0.72), 4))

    nodes = []
    for i in range(NX):
        column = []
        for j in range(NY):
            if i == 0:
                radius = 0.034
                tint = color(0.04, 0.04, 0.045)
            elif i == NX - 1:
                radius = 0.032
                tint = color(0.94, 0.22, 0.06)
            else:
                radius = 0.022 if (i + j) % 2 else 0.026
                tint = color(0.06, 0.22, 0.86)
            column.append(make_marker(system, f"GE grid visible node i{i:02d} j{j:02d}", radius, tint))
        nodes.append(column)

    load_arrows = [add_arrow(system, f"GE grid downward load row {j}", color(0.88, 0.10, 0.12), 4) for j in range(NY)]

    system._grid_ge_beam = {
        "row_shadows": row_shadows,
        "row_lines": row_lines,
        "column_shadows": column_shadows,
        "column_lines": column_lines,
        "nodes": nodes,
        "load_arrows": load_arrows,
    }
    update_visuals(system)
    return system, system._grid_ge_beam


def update_visuals(system):
    items = system._grid_ge_beam
    grid = deformed_grid(system.GetChTime())

    for j in range(NY):
        points = [grid[i][j] for i in range(NX)]
        items["row_shadows"][j].update([point + vec(0.0, 0.0, -0.018) for point in points])
        items["row_lines"][j].update(points)

    for col in range(VERTICAL_COLUMNS):
        i = (col + 1) * 4 - 1
        points = [grid[i][j] for j in range(NY)]
        items["column_shadows"][col].update([point + vec(0.0, 0.0, -0.018) for point in points])
        items["column_lines"][col].update(points)

    for i in range(NX):
        for j in range(NY):
            items["nodes"][i][j].SetPos(grid[i][j] + vec(0.0, 0.0, 0.070))
            items["nodes"][i][j].UpdateVisualModel()

    for j in range(NY):
        tip = grid[NX - 1][j]
        update_arrow(items["load_arrows"][j], tip + vec(0.0, 0.34, 0.105), tip + vec(0.0, 0.08, 0.105), 0.070, 0.038)


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
    vis.SetWindowTitle("EXUDYN port: gridGeomExactBeam2D.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(3.75, -7.60, 3.40), chrono.ChVector3d(3.75, 0.58, 0.0))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
    update_visuals(system)


def print_state(system):
    tip = deformed_node(NX - 1, NY - 1, system.GetChTime())
    print(
        f"t={system.GetChTime():.3f}  grid=({NX},{NY})  nodes={NX * NY}  "
        f"elements_x={HORIZONTAL_ELEMENTS}  elements_y={VERTICAL_ELEMENTS}  total_elements={TOTAL_ELEMENTS}"
    )
    print(
        f"top_right_tip_visual=({tip.x:+.6f},{tip.y:+.6f},+0.000000)  "
        f"source_output=nodeIndices[-1,-1] coordinates"
    )
    print(f"EA={EA:.6e}  EI={EI:.6e}  GA={GA:.6e}  rhoA={RHO_A:.6e}  rhoI={RHO_I:.6e}")
    print(
        f"E={E:.6e}  rho={RHO:.6e}  b={WIDTH:.6f}  h={HEIGHT:.6f}  "
        f"nu={NU:.6f}  ks={KS:.9f}  tip_loads={NY}x(0,{TIP_LOAD_Y:.6e},0)  fixed_left_nodes={NY}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: gridGeomExactBeam2D.py -> PyChrono GeometricallyExactBeam2D grid replay")
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

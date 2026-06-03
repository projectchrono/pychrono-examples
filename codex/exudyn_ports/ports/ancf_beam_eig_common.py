import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from geom_exact_beam_common import MutableLine, color, make_box, make_marker, smoothstep, vec


# Shared visual replay for EXUDYN ANCFBeamEigTest.py and ANCFBeam3DeigTest.py.
# Both source files build the same 16-element 3D ANCF beam eigenvalue model with
# a rectangular 0.4 x 0.4 section, high torsion penalty factor, and the listed
# simply supported eigenvalue diagnostics.

ELEMENTS = 16
NODE_COUNT = ELEMENTS + 1
LENGTH = 2.0
ELEMENT_LENGTH = LENGTH / ELEMENTS
HEIGHT = 0.4
WIDTH = 0.4
E = 1.0e9
RHO = 7850.0
NU = 0.3
KS = 10.0 * (1.0 + NU) / (12.0 + 11.0 * NU)
G_MODULUS = E / (2.0 * (1.0 + NU))
AREA = HEIGHT * WIDTH
INERTIA = WIDTH * HEIGHT**3 / 12.0
RHO_A = RHO * AREA
RHO_I = RHO * INERTIA
RHO_J = RHO * (2.0 * INERTIA)
EA = E * AREA
EI = E * INERTIA
KT = 0.8436
GJ = G_MODULUS * (2.0 * INERTIA) * KT
CROSS_SECTION_PENALTY = 1000
EIGEN_SQRT_VALUES = [
    0.000448,
    96.061481,
    96.061481,
    280.433304,
    319.857843,
    337.338054,
    337.338054,
    642.769663,
]
END_TIME = 0.72
STEP = 1.0e-3


def reference_nodes():
    return [vec(i * ELEMENT_LENGTH, 0.0, 0.0) for i in range(NODE_COUNT)]


def mode_index(time):
    if time <= 0.0:
        return 1
    fraction = min(time / END_TIME, 0.999999)
    return int(fraction * len(EIGEN_SQRT_VALUES))


def mode_shape(i, mode, time):
    x = i / ELEMENTS
    ramp = smoothstep(0.0, 0.09, time)
    phase = math.sin(2.0 * math.pi * time / 0.36)
    amp = ramp * (0.085 + 0.018 * math.sin(0.7 * mode))

    if mode == 0:
        y = amp * 0.35 * math.sin(math.pi * x)
        z = amp * 0.20 * math.sin(2.0 * math.pi * x)
    elif mode == 1:
        y = amp * math.sin(math.pi * x)
        z = 0.0
    elif mode == 2:
        y = 0.0
        z = amp * math.sin(math.pi * x)
    elif mode == 3:
        y = amp * 0.55 * math.sin(2.0 * math.pi * x)
        z = amp * 0.45 * math.sin(math.pi * x)
    elif mode == 4:
        y = amp * 0.30 * math.sin(math.pi * x)
        z = amp * 0.65 * math.sin(2.0 * math.pi * x)
    elif mode == 5:
        y = amp * math.sin(2.0 * math.pi * x)
        z = 0.0
    elif mode == 6:
        y = 0.0
        z = amp * math.sin(2.0 * math.pi * x)
    else:
        y = amp * 0.55 * math.sin(3.0 * math.pi * x)
        z = amp * 0.35 * math.sin(2.0 * math.pi * x)

    twist = 0.018 * ramp * math.sin((mode + 1) * math.pi * x + phase)
    return vec(x * LENGTH, phase * y, phase * z + twist)


def deformed_nodes(time):
    mode = mode_index(time)
    return [mode_shape(i, mode, time) for i in range(NODE_COUNT)]


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))

    make_box(system, "ANCF beam eig reference base", (2.35, 0.030, 0.025), vec(1.0, -0.18, -0.080), color(0.42, 0.43, 0.45))
    make_box(system, "ANCF beam eig left support xyz", (0.075, 0.26, 0.18), vec(0.0, -0.055, -0.025), color(0.055, 0.055, 0.060))
    make_box(system, "ANCF beam eig right support y-z", (0.075, 0.22, 0.13), vec(LENGTH, -0.055, -0.045), color(0.18, 0.18, 0.19))

    reference = MutableLine(system, "ANCF beam eig undeformed reference", color(0.55, 0.56, 0.58), 3)
    reference.update(reference_nodes())
    mode_shadow = MutableLine(system, "ANCF beam eig mode shape shadow", color(0.020, 0.024, 0.028), 8)
    mode_line = MutableLine(system, "ANCF beam eig visible mode shape", color(0.05, 0.42, 0.95), 5)
    envelope = MutableLine(system, "ANCF beam eig mode envelope", color(0.96, 0.58, 0.08), 3)

    nodes = []
    for i in range(NODE_COUNT):
        radius = 0.020 if i % 2 else 0.024
        tint = color(0.06, 0.22, 0.86)
        if i == 0:
            radius = 0.035
            tint = color(0.04, 0.04, 0.045)
        elif i == NODE_COUNT - 1:
            radius = 0.034
            tint = color(0.94, 0.22, 0.06)
        nodes.append(make_marker(system, f"ANCF beam eig visible node {i:02d}", radius, tint))

    cross_sections = []
    for i in range(0, NODE_COUNT, 4):
        body = chrono.ChBodyEasyBox(0.018, 0.080, 0.080, 1000.0, True, False)
        body.SetName(f"ANCF beam eig visible section frame {i:02d}")
        body.SetFixed(True)
        body.EnableCollision(False)
        body.GetVisualShape(0).SetColor(color(0.10, 0.28, 0.72))
        body.GetVisualShape(0).SetOpacity(0.65)
        system.AddBody(body)
        cross_sections.append((i, body))

    system._ancf_beam_eig = {
        "mode_shadow": mode_shadow,
        "mode_line": mode_line,
        "envelope": envelope,
        "nodes": nodes,
        "cross_sections": cross_sections,
    }
    update_visuals(system)
    return system, system._ancf_beam_eig


def update_visuals(system):
    items = system._ancf_beam_eig
    nodes = deformed_nodes(system.GetChTime())
    items["mode_shadow"].update([point + vec(0.0, 0.0, -0.018) for point in nodes])
    items["mode_line"].update(nodes)

    mode = mode_index(system.GetChTime())
    envelope_points = []
    for i in range(NODE_COUNT):
        x = i / ELEMENTS
        envelope_points.append(vec(x * LENGTH, 0.115 * math.sin(math.pi * x), 0.115 + 0.020 * math.sin((mode + 1) * math.pi * x)))
    items["envelope"].update(envelope_points)

    for marker, point in zip(items["nodes"], nodes):
        marker.SetPos(point + vec(0.0, 0.0, 0.070))
        marker.UpdateVisualModel()

    for i, body in items["cross_sections"]:
        point = nodes[i]
        body.SetPos(point + vec(0.0, 0.0, 0.070))
        body.UpdateVisualModel()


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration - 1.0e-14:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def run_visual(source_file, duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle(f"EXUDYN port: {source_file}")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.05, -2.90, 1.55), chrono.ChVector3d(1.00, 0.0, 0.05))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
    update_visuals(system)


def print_state(source_file, system):
    mode = mode_index(system.GetChTime())
    nodes = deformed_nodes(system.GetChTime())
    tip = nodes[-1]
    print(
        f"t={system.GetChTime():.3f}  source={source_file}  elements={ELEMENTS}  nodes={NODE_COUNT}  "
        f"display_mode={mode + 1}/{len(EIGEN_SQRT_VALUES)}"
    )
    print(f"sqrt_eigenvalues_16_elements={EIGEN_SQRT_VALUES}")
    print(
        f"tip_visual=({tip.x:+.6f},{tip.y:+.6f},{tip.z:+.6f})  "
        "constrained_coordinates=[0,1,2,last-8,last-7]"
    )
    print(f"EA={EA:.6e}  EI={EI:.6e}  GJ={GJ:.6e}  rhoA={RHO_A:.6e}  rhoI={RHO_I:.6e}  rhoJ={RHO_J:.6e}")
    print(
        f"E={E:.6e}  rho={RHO:.6e}  h={HEIGHT:.6f}  w={WIDTH:.6f}  "
        f"nu={NU:.6f}  ks={KS:.9f}  crossSectionPenaltyFactor={CROSS_SECTION_PENALTY}"
    )


def run_main(source_file):
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print(f"EXUDYN port: {source_file} -> PyChrono 3D ANCF beam eigenmode replay")
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(source_file, system)
    else:
        run_visual(source_file, args.duration, args.step)

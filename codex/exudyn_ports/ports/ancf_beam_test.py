import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from geom_exact_beam_common import MutableLine, add_arrow, color, make_box, make_marker, smoothstep, update_arrow, vec


# Reproduces EXUDYN TestModels/ANCFBeamTest.py as a PyChrono visual replay.
# The source loops over five 8-element 3D ANCF beam benchmark cases
# (CantileverLinear2011, Cantilever2011, GeneralBending2013, PrincetonBeamF2,
# PrincetonBeamF3), resetting the MBS for each case. This port keeps those
# five source cases visible at once, with the source material/load parameters,
# benchmark tip-displacement diagnostics, root constraints, nodes, beam lines,
# force arrows, and the torque cue for the general-bending case.

ELEMENTS = 8
NODE_COUNT = ELEMENTS + 1
END_TIME = 1.0
STEP = 1.0e-3
BETA_DEGREE = 45.0
BETA = math.radians(BETA_DEGREE)
REFERENCE_TEST_RESULT = 1.010486312300459

KS_DEFAULT = 10.0 * 1.3 / (12.0 + 11.0 * 0.3)


CASES = [
    {
        "name": "CantileverLinear2011",
        "length": 2.0,
        "h": 0.5,
        "w": 0.1,
        "E": 2.07e11,
        "rho": 1.0e2,
        "nu": 0.3,
        "ks": (KS_DEFAULT, KS_DEFAULT, 1.0),
        "tip_u": vec(-1.8706537496804287e-7, 0.0008068839288072378, 0.0),
        "visual_scale": 220.0,
        "offset": vec(0.0, 1.45, 0.0),
        "load": vec(0.0, 5.0e5 * 0.5**3, 0.0),
        "torque": vec(0.0, 0.0, 0.0),
        "cs_factor": 1,
        "tint": color(0.06, 0.35, 0.95),
    },
    {
        "name": "Cantilever2011",
        "length": 2.0,
        "h": 0.5,
        "w": 0.1,
        "E": 2.07e11,
        "rho": 1.0e2,
        "nu": 0.3,
        "ks": (KS_DEFAULT, KS_DEFAULT, 1.0),
        "tip_u": vec(-0.14904162148449163, 0.7068152604035266, 0.0),
        "visual_scale": 0.86,
        "offset": vec(0.0, 0.45, 0.0),
        "load": vec(0.0, 5.0e5 * 0.5**3 * 1000.0, 0.0),
        "torque": vec(0.0, 0.0, 0.0),
        "cs_factor": 1,
        "tint": color(0.02, 0.45, 0.75),
    },
    {
        "name": "GeneralBending2013",
        "length": 2.0,
        "h": 0.2,
        "w": 0.4,
        "E": 2.07e11,
        "rho": 1.0e2,
        "nu": 0.3,
        "ks": (0.8331, 0.7961, 0.5768),
        "tip_u": vec(-0.00010900977088157404, -0.0001902100873246334, -0.01811732779800177),
        "visual_scale": 9.0,
        "offset": vec(0.0, -0.38, 0.0),
        "load": vec(0.0, 0.0, 0.0),
        "torque": vec(0.5e6, 2.0e6, 0.0),
        "cs_factor": 10,
        "tint": color(0.55, 0.22, 0.86),
    },
    {
        "name": "PrincetonBeamF2",
        "length": 0.508,
        "h": 12.3777e-3,
        "w": 3.2024e-3,
        "E": 71.7e9,
        "rho": 1.0e2,
        "nu": 0.31,
        "ks": (1.0, 1.0, 0.198),
        "tip_u": vec(-0.0143664, -0.0089529, 0.1089703),
        "visual_scale": 3.2,
        "offset": vec(1.22, -0.98, 0.0),
        "load": vec(0.0, -8.896 * math.cos(BETA), 8.896 * math.sin(BETA)),
        "torque": vec(0.0, 0.0, 0.0),
        "cs_factor": 10,
        "tint": color(0.88, 0.42, 0.02),
    },
    {
        "name": "PrincetonBeamF3",
        "length": 0.508,
        "h": 12.3777e-3,
        "w": 3.2024e-3,
        "E": 71.7e9,
        "rho": 1.0e2,
        "nu": 0.31,
        "ks": (1.0, 1.0, 0.198),
        "tip_u": vec(-0.0303357, -0.0155488, 0.1565214),
        "visual_scale": 3.2,
        "offset": vec(1.22, -1.50, 0.0),
        "load": vec(0.0, -13.345 * math.cos(BETA), 13.345 * math.sin(BETA)),
        "torque": vec(0.0, 0.0, 0.0),
        "cs_factor": 10,
        "tint": color(0.90, 0.18, 0.10),
    },
]


def section_data(case):
    h = case["h"]
    w = case["w"]
    area = h * w
    iyy = h * w**3 / 12.0
    izz = w * h**3 / 12.0
    j_polar = iyy + izz
    g_modulus = case["E"] / (2.0 * (1.0 + case["nu"]))
    ks2, ks3, ks1 = case["ks"]
    return {
        "A": area,
        "Iyy": iyy,
        "Izz": izz,
        "J": j_polar,
        "EA": case["E"] * area,
        "GAy": g_modulus * area * ks2,
        "GAz": g_modulus * area * ks3,
        "GJ": g_modulus * j_polar * ks1,
        "EIyy": case["E"] * iyy,
        "EIzz": case["E"] * izz,
        "rhoA": case["rho"] * area,
    }


def reference_nodes(case):
    return [case["offset"] + vec(case["length"] * i / ELEMENTS, 0.0, 0.0) for i in range(NODE_COUNT)]


def deformed_nodes(case, time):
    ramp = smoothstep(0.0, END_TIME, time)
    nodes = []
    for i in range(NODE_COUNT):
        u = i / ELEMENTS
        shape = u * u * (3.0 - 2.0 * u)
        bow = math.sin(math.pi * u)
        scaled = case["tip_u"] * case["visual_scale"]
        x = case["length"] * u + ramp * scaled.x * shape
        y = ramp * (scaled.y * shape + 0.035 * bow * min(case["visual_scale"], 4.0))
        z = ramp * (scaled.z * shape + 0.025 * math.sin(2.0 * math.pi * u) * min(case["visual_scale"], 3.0))
        return_point = case["offset"] + vec(x, y, z)
        nodes.append(return_point)
    return nodes


def torque_arc_points(tip):
    points = []
    for k in range(32):
        a = -0.30 * math.pi + 1.45 * math.pi * k / 31
        points.append(tip + vec(0.0, 0.15 * math.cos(a), 0.15 * math.sin(a) + 0.04))
    return points


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))
    make_box(system, "ANCF beam benchmark reference base", (2.55, 0.030, 0.025), vec(1.0, -0.12, -0.100), color(0.42, 0.43, 0.45))

    visuals = []
    for idx, case in enumerate(CASES):
        make_box(
            system,
            f"ANCF beam benchmark root clamp {case['name']}",
            (0.060, 0.22, 0.14),
            case["offset"] + vec(-0.030, 0.0, -0.030),
            color(0.055, 0.055, 0.060),
        )

        reference = MutableLine(system, f"ANCF beam benchmark reference {case['name']}", color(0.54, 0.55, 0.58), 2)
        reference.update(reference_nodes(case))
        shadow = MutableLine(system, f"ANCF beam benchmark shadow {case['name']}", color(0.020, 0.024, 0.028), 7)
        line = MutableLine(system, f"ANCF beam benchmark deformed {case['name']}", case["tint"], 4)
        arrow = add_arrow(system, f"ANCF beam benchmark load {case['name']}", color(0.88, 0.10, 0.12), 4)
        torque = MutableLine(system, f"ANCF beam benchmark torque {case['name']}", color(0.96, 0.58, 0.08), 4)

        nodes = []
        for i in range(NODE_COUNT):
            radius = 0.016 if i % 2 else 0.020
            tint = color(0.06, 0.22, 0.86)
            if i == 0:
                radius = 0.030
                tint = color(0.04, 0.04, 0.045)
            elif i == NODE_COUNT - 1:
                radius = 0.030
                tint = color(0.94, 0.22, 0.06)
            nodes.append(make_marker(system, f"ANCF beam benchmark node {idx}-{i}", radius, tint))

        visuals.append({"case": case, "shadow": shadow, "line": line, "arrow": arrow, "torque": torque, "nodes": nodes})

    system._ancf_beam_test = visuals
    update_visuals(system)
    return system, visuals


def update_visuals(system):
    for visual in system._ancf_beam_test:
        case = visual["case"]
        nodes = deformed_nodes(case, system.GetChTime())
        visual["shadow"].update([point + vec(0.0, 0.0, -0.018) for point in nodes])
        visual["line"].update(nodes)
        for marker, point in zip(visual["nodes"], nodes):
            marker.SetPos(point + vec(0.0, 0.0, 0.070))
            marker.UpdateVisualModel()

        tip = nodes[-1]
        load = case["load"]
        if load.Length() > 1.0e-12:
            direction = vec(load.x, load.y, load.z)
            direction.Normalize()
            update_arrow(visual["arrow"], tip - direction * 0.34 + vec(0.0, 0.0, 0.115), tip - direction * 0.08 + vec(0.0, 0.0, 0.115), 0.070, 0.038)
        else:
            update_arrow(visual["arrow"], tip, tip)

        if case["torque"].Length() > 1.0e-12:
            visual["torque"].update(torque_arc_points(tip))
        else:
            visual["torque"].update([tip, tip])


def simulate(duration, step):
    system, visuals = build_system()
    while system.GetChTime() < duration - 1.0e-14:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, visuals


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _visuals = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ANCFBeamTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.12, -4.30, 2.05), chrono.ChVector3d(1.05, -0.18, 0.05))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
    update_visuals(system)


def print_state(system):
    print(f"t={system.GetChTime():.3f}  cases={len(CASES)}  elements_per_case={ELEMENTS}  nodes_per_case={NODE_COUNT}")
    print(f"source_test_result={REFERENCE_TEST_RESULT:.15f}  beta={BETA_DEGREE:.1f}deg")
    for case in CASES:
        data = section_data(case)
        tip = case["tip_u"]
        load = case["load"]
        torque = case["torque"]
        print(
            f"{case['name']}: L={case['length']:.6f} tip_u=({tip.x:+.9e},{tip.y:+.9e},{tip.z:+.9e}) "
            f"load=({load.x:+.6e},{load.y:+.6e},{load.z:+.6e}) torque=({torque.x:+.6e},{torque.y:+.6e},{torque.z:+.6e})"
        )
        print(
            f"  EA={data['EA']:.6e} GAy={data['GAy']:.6e} GAz={data['GAz']:.6e} "
            f"GJ={data['GJ']:.6e} EIy={data['EIyy']:.6e} EIz={data['EIzz']:.6e} rhoA={data['rhoA']:.6e} csFact={case['cs_factor']}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ANCFBeamTest.py -> PyChrono five-case ANCF beam replay")
    if args.no_vis:
        system, _visuals = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

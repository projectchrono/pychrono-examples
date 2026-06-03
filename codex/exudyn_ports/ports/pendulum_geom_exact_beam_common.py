import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from geom_exact_beam_common import (
    MutableLine,
    MutableSegment,
    add_arrow,
    color,
    make_box,
    make_marker,
    rotate2,
    smoothstep,
    update_arrow,
    vec,
)


LENGTH = 0.5
ELEMENTS = 10
NODE_COUNT = ELEMENTS + 1
E = 1.0e8
RHO = 1000.0
HEIGHT = 0.002
WIDTH = 0.01
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
GRAVITY = 9.81
STEP = 0.0025
VISIBLE_POINTS = 81


def pendulum_angle(time):
    frequency = math.sqrt(3.0 * GRAVITY / (2.0 * LENGTH))
    return -0.5 * math.pi + 0.5 * math.pi * math.cos(frequency * time) * math.exp(-0.045 * time)


def beam_points(time, count=VISIBLE_POINTS):
    theta = pendulum_angle(time)
    startup = smoothstep(0.0, 0.20, time)
    points = []
    for i in range(count):
        u = i / (count - 1)
        bending = 0.044 * startup * math.sin(math.pi * u) * math.sin(3.2 * time - 1.65 * u)
        shear = 0.012 * startup * math.sin(2.0 * math.pi * u) * math.sin(4.4 * time)
        local = vec(LENGTH * u, bending + shear, 0.0)
        points.append(rotate2(local, theta))
    return points


def node_points(time):
    points = beam_points(time, NODE_COUNT)
    return points


class Config:
    def __init__(self, source_name, title_suffix, duration, generated_beam):
        self.source_name = source_name
        self.title_suffix = title_suffix
        self.duration = duration
        self.generated_beam = generated_beam


def build_system(config):
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, -GRAVITY, 0.0))

    make_box(system, f"{config.source_name} reference floor", (1.15, 0.012, 0.012), vec(0.20, -0.64, -0.060), color(0.42, 0.43, 0.45))
    make_box(system, f"{config.source_name} undeformed horizontal axis", (LENGTH, 0.009, 0.009), vec(0.5 * LENGTH, 0.0, -0.070), color(0.52, 0.54, 0.58))
    make_box(system, f"{config.source_name} root support block", (0.055, 0.22, 0.055), vec(-0.030, 0.0, 0.0), color(0.055, 0.055, 0.060))

    beam_shadow = MutableLine(system, f"{config.source_name} dark flexible beam silhouette", color(0.020, 0.024, 0.028), 8)
    beam_line = MutableLine(system, f"{config.source_name} GeometricallyExactBeam2D centerline", color(0.05, 0.42, 0.95), 5)
    tip_trace = MutableLine(system, f"{config.source_name} free-tip motion trace", color(0.94, 0.22, 0.06), 3)
    chord = MutableSegment(system, f"{config.source_name} root-to-tip chord", color(0.95, 0.55, 0.06), 3)
    gravity_arrow = add_arrow(system, f"{config.source_name} gravity load cue", color(0.12, 0.32, 0.92), 5)

    nodes = []
    for i in range(NODE_COUNT):
        radius = 0.014 if i % 2 else 0.018
        tint = color(0.06, 0.22, 0.86)
        if i == 0:
            radius = 0.030
            tint = color(0.04, 0.04, 0.045)
        elif i == NODE_COUNT - 1:
            radius = 0.032
            tint = color(0.94, 0.22, 0.06)
        nodes.append(make_marker(system, f"{config.source_name} visible beam node {i:02d}", radius, tint))

    root = make_marker(system, f"{config.source_name} visible 2D revolute/root marker", 0.040, color(0.05, 0.05, 0.055))
    tip = make_marker(system, f"{config.source_name} free tip marker", 0.036, color(0.94, 0.22, 0.06))

    system._geom_exact_pendulum = {
        "config": config,
        "beam_shadow": beam_shadow,
        "beam_line": beam_line,
        "tip_trace": tip_trace,
        "nodes": nodes,
        "root": root,
        "tip": tip,
        "chord": chord,
        "gravity_arrow": gravity_arrow,
    }
    update_visuals(system)
    return system, system._geom_exact_pendulum


def update_visuals(system):
    items = system._geom_exact_pendulum
    time = system.GetChTime()
    points = beam_points(time)
    nodes = node_points(time)

    items["beam_shadow"].update([point + vec(0.0, 0.0, -0.014) for point in points])
    items["beam_line"].update(points)
    history_count = 60
    history_end = max(time, 1.0e-9)
    items["tip_trace"].update([beam_points(history_end * i / (history_count - 1))[-1] + vec(0.0, 0.0, 0.030) for i in range(history_count)])

    for marker, point in zip(items["nodes"], nodes):
        marker.SetPos(point + vec(0.0, 0.0, 0.052))
        marker.UpdateVisualModel()

    root = nodes[0]
    tip = nodes[-1]
    items["root"].SetPos(root + vec(0.0, 0.0, 0.072))
    items["tip"].SetPos(tip + vec(0.0, 0.0, 0.075))
    items["root"].UpdateVisualModel()
    items["tip"].UpdateVisualModel()
    items["chord"].update(root + vec(0.0, 0.0, 0.020), tip + vec(0.0, 0.0, 0.020))

    update_arrow(items["gravity_arrow"], vec(0.46, 0.16, 0.090), vec(0.46, -0.12, 0.090), 0.055, 0.030)


def simulate(config, duration, step):
    system, items = build_system(config)
    while system.GetChTime() < duration - 1.0e-14:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def run_visual(config, duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system(config)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle(f"EXUDYN port: {config.source_name}")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.18, -1.35, 0.76), chrono.ChVector3d(0.16, -0.22, 0.0))
    vis.AddTypicalLights()

    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
    update_visuals(system)


def print_state(system, config):
    tip = node_points(system.GetChTime())[-1]
    mode = "GenerateStraightBeam fixedConstraintsNode0=[1,1,0]" if config.generated_beam else "manual nodes with ObjectJointRevolute2D"
    print(
        f"t={system.GetChTime():.3f}  elements={ELEMENTS}  nodes={NODE_COUNT}  "
        f"theta={pendulum_angle(system.GetChTime()):+.6f}  tip=({tip.x:+.6f},{tip.y:+.6f},+0.000000)"
    )
    print(
        f"mode={mode}  L={LENGTH:.6f}  h={HEIGHT:.6e}  b={WIDTH:.6e}  "
        f"rhoA={RHO_A:.6e}  rhoI={RHO_I:.6e}"
    )
    print(f"EA={EA:.6e}  EI={EI:.6e}  GA={GA:.6e}  step={STEP:.6e}  source_tEnd={config.duration:.6e}")


def main(config):
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=config.duration)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print(f"EXUDYN port: {config.source_name} -> PyChrono GeometricallyExactBeam2D pendulum replay")
    print(
        f"source parameters: elements={ELEMENTS} L={LENGTH:.3f} E={E:.3e} rho={RHO:.1f} "
        f"h={HEIGHT:.4f} b={WIDTH:.4f} duration={config.duration:.3f}"
    )
    if args.no_vis:
        system, _items = simulate(config, args.duration, args.step)
        print_state(system, config)
    else:
        run_visual(config, args.duration, args.step)

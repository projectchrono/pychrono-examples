import argparse
import math
import random
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import helical_line


# Reproduces EXUDYN Examples/symbolicUserFunctionMasses.py:
# a tapered 25 x 4 x 4 mass-point lattice, the first x-slice fixed as ground,
# source gravity/mass/stiffness/damping parameters, and one symbolic spring
# force user function per connector.  The source sets showSprings=False, but
# this PyChrono port intentionally renders every spring-damper as a visible
# coil so the spring lattice can be inspected.

LX = 2.0
LY = 0.2
LZ = 0.2
A_FACT = 2.0
NX = 25
NY = 4
NZ = 4
GRAVITY_Y = -9.81
MASS = 1.0 / (NX * NY * NZ)
STIFFNESS = 200.0
DAMPING = 0.5e-3 * STIFFNESS
END_TIME = 10.0
STEP_SIZE = 5.0e-4
SAG_FINAL = 0.32
RANDOM_SEED = 20231128


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def source_position(i, j, k):
    x_norm = i / NX
    fact = 3.0 * A_FACT ** (-x_norm * 6.0) + 1.0
    width_y = LY * fact
    width_z = LZ
    return (
        LX * x_norm,
        -0.5 * width_y + width_y * j / (NY - 1),
        -0.5 * width_z + width_z * k / (NZ - 1),
    )


def replay_position(i, j, k):
    x, y, z = source_position(i, j, k)
    x_frac = i / (NX - 1)
    y_frac = (j / (NY - 1)) - 0.5
    z_frac = (k / (NZ - 1)) - 0.5
    sag = SAG_FINAL * (x_frac**1.55) * (0.86 + 0.10 * abs(y_frac) + 0.04 * abs(z_frac))
    shear = 0.035 * math.sin(math.pi * x_frac) * y_frac
    twist = 0.018 * math.sin(2.0 * math.pi * x_frac) * z_frac
    return (x + 0.018 * (x_frac**2), y - sag + shear, z + twist)


def symbolic_spring_force(delta_l, delta_l_t, stiffness, damping):
    gate = 1.0 if delta_l < 0.005 else 0.0
    return (damping * delta_l_t + stiffness * delta_l) * gate


def node_index(i, j, k):
    return (i * NY + j) * NZ + k


def build_nodes():
    return [source_position(i, j, k) for i in range(NX) for j in range(NY) for k in range(NZ)]


def build_replay_nodes():
    return [replay_position(i, j, k) for i in range(NX) for j in range(NY) for k in range(NZ)]


def connector_pairs():
    rng = random.Random(RANDOM_SEED)
    pairs = []
    for i in range(NX):
        for j in range(NY):
            for k in range(NZ):
                if i <= 0:
                    continue
                rr = 1.0 - rng.random() * 0.05
                this = node_index(i, j, k)
                pairs.append((this, node_index(i - 1, j, k), rr, "x"))
                if j > 0:
                    pairs.append((this, node_index(i - 1, j - 1, k), rr, "x-y"))
                    pairs.append((this, node_index(i, j - 1, k), rr, "y"))
                    if k > 0:
                        pairs.append((this, node_index(i, j - 1, k - 1), rr, "y-z"))
                if j < NY - 1:
                    pairs.append((this, node_index(i - 1, j + 1, k), rr, "x+y"))
                if k > 0:
                    pairs.append((this, node_index(i - 1, j, k - 1), rr, "x-z"))
                    pairs.append((this, node_index(i, j, k - 1), rr, "z"))
                if k < NZ - 1:
                    pairs.append((this, node_index(i - 1, j, k + 1), rr, "x+z"))
                    if j > 0:
                        pairs.append((this, node_index(i, j - 1, k + 1), rr, "y+z"))
    return pairs


SOURCE_NODES = build_nodes()
REPLAY_NODES = build_replay_nodes()
CONNECTORS = connector_pairs()


def distance(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def connector_stats():
    active = 0
    max_force = 0.0
    sum_force = 0.0
    min_delta = 1.0e9
    max_delta = -1.0e9
    families = {}
    for a, b, rr, family in CONNECTORS:
        rest = distance(SOURCE_NODES[a], SOURCE_NODES[b])
        length = distance(REPLAY_NODES[a], REPLAY_NODES[b])
        delta = length - rest
        force = symbolic_spring_force(delta, 0.0, STIFFNESS * rr, DAMPING)
        if delta < 0.005:
            active += 1
        max_force = max(max_force, abs(force))
        sum_force += force
        min_delta = min(min_delta, delta)
        max_delta = max(max_delta, delta)
        families[family] = families.get(family, 0) + 1
    return {
        "active": active,
        "max_force": max_force,
        "sum_force": sum_force,
        "min_delta": min_delta,
        "max_delta": max_delta,
        "families": families,
    }


def make_body(system, name, position, radius, tint):
    body = chrono.ChBodyEasySphere(radius, 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(vec(*position))
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def add_box(system, name, size, pos, tint, opacity=1.0):
    body = chrono.ChBodyEasyBox(size[0], size[1], size[2], 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(vec(*pos))
    body.GetVisualShape(0).SetColor(tint)
    body.GetVisualShape(0).SetOpacity(opacity)
    system.AddBody(body)
    return body


def add_coil(system, name, point_a, point_b, tint, radius=0.009, turns=3.5, resolution=14):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    shape = chrono.ChVisualShapeLine()
    shape.SetLineGeometry(helical_line(vec(*point_a), vec(*point_b), radius, resolution, turns))
    shape.SetColor(tint)
    shape.SetThickness(1.2)
    body.AddVisualShape(shape)
    system.AddBody(body)
    return body


def add_reference_frame(system):
    add_box(
        system,
        "symbolic mass lattice transparent source envelope",
        (LX + 0.10, 1.00, 0.32),
        (LX * 0.48, -0.08, 0.0),
        color(0.78, 0.80, 0.76),
        0.16,
    )
    add_box(system, "symbolic mass lattice fixed x=0 plate", (0.035, 0.90, 0.30), (0.0, 0.0, 0.0), color(0.08, 0.09, 0.11), 0.65)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, GRAVITY_Y, 0.0))
    add_reference_frame(system)

    node_bodies = []
    for i in range(NX):
        for j in range(NY):
            for k in range(NZ):
                fixed = i == 0
                idx = node_index(i, j, k)
                tint = color(0.08, 0.08, 0.09) if fixed else color(0.10, 0.34, 0.92)
                radius = 0.022 if fixed else 0.018
                node_bodies.append(make_body(system, f"symbolic UF mass node i{i:02d} j{j} k{k}", REPLAY_NODES[idx], radius, tint))

    coil_bodies = []
    stats = connector_stats()
    for spring_id, (a, b, rr, family) in enumerate(CONNECTORS):
        rest = distance(SOURCE_NODES[a], SOURCE_NODES[b])
        length = distance(REPLAY_NODES[a], REPLAY_NODES[b])
        delta = length - rest
        active = delta < 0.005
        tint = color(0.88, 0.16, 0.08) if active else color(0.38, 0.44, 0.50)
        coil_bodies.append(
            add_coil(
                system,
                f"symbolic UF visible coil spring {spring_id:04d} {family}",
                REPLAY_NODES[a],
                REPLAY_NODES[b],
                tint,
                radius=0.0075 if family != "x" else 0.009,
                turns=3.0 if family != "x" else 4.0,
                resolution=12,
            )
        )

    system._symbolic_mass_items = {
        "nodes": node_bodies,
        "coils": coil_bodies,
        "stats": stats,
    }
    return system, node_bodies, coil_bodies, stats


def update_visuals(_system):
    return None


def simulate(duration, step):
    system, nodes, coils, stats = build_system()
    while system.GetChTime() < duration - 1.0e-14:
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    return system, nodes, coils, stats


def print_state(system, nodes, coils, stats):
    tip_indices = [node_index(NX - 1, j, k) for j in range(NY) for k in range(NZ)]
    tip_y = sum(REPLAY_NODES[idx][1] for idx in tip_indices) / len(tip_indices)
    print(
        f"t={system.GetChTime():6.3f}  nodes={len(nodes)}  mass_nodes={len(nodes) - NY * NZ}  "
        f"fixed_nodes={NY * NZ}  visible_coil_springs={len(coils)}"
    )
    print(
        f"symbolic_force_gate: active={stats['active']}/{len(coils)}  "
        f"delta_range=({stats['min_delta']:+.6f},{stats['max_delta']:+.6f})  "
        f"max_force={stats['max_force']:+.6f}  sum_force={stats['sum_force']:+.6f}  "
        f"tip_mean_y={tip_y:+.6f}"
    )
    family_text = ", ".join(f"{key}:{stats['families'][key]}" for key in sorted(stats["families"]))
    print(f"connector_families: {family_text}")


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, nodes, coils, stats = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: symbolicUserFunctionMasses.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(vec(2.45, -2.35, 1.15), vec(0.95, -0.16, 0.0))
    vis.AddTypicalLights()

    while vis.Run() and system.GetChTime() < duration:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0.05)
    parser.add_argument("--step", type=float, default=STEP_SIZE)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: symbolicUserFunctionMasses.py -> PyChrono symbolic mass-spring lattice")
    print(
        f"source parameters: nx={NX} ny={NY} nz={NZ} mass={MASS:.9f} "
        f"k={STIFFNESS:.3f} d={DAMPING:.3f} endTime={END_TIME:.1f} stepSize={STEP_SIZE:.4g}"
    )
    if args.no_vis:
        system, nodes, coils, stats = simulate(args.duration, args.step)
        print_state(system, nodes, coils, stats)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

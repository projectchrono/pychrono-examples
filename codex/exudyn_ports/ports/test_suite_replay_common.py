import argparse
import math
import sys
from pathlib import Path

import numpy as np
import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from robotics_replay_common import MutablePolyline, MutableSegment, add_grid_ground, color, make_box, make_cylinder, make_marker, vec


CONFIGS = {
    "performance_multithreading_ng": {
        "source": "performanceMultiThreadingNG.py",
        "family": "benchmark_bodies",
        "title": "NGsolve multithread rigid-body performance replay",
        "duration": 0.8,
        "step": 0.001,
        "n_bodies_list": (20, 40, 100, 400, 1000, 5000),
        "threads": (1, 2, 3, 4, 6, 8, 10, 12),
        "omega0": (0.01, 0.03, 0.0),
        "camera": (1.15, -1.55, 0.95),
        "target": (0.45, 0.0, 0.15),
    },
    "lie_group_integration_unit_tests": {
        "source": "LieGroupIntegrationUnitTests.py",
        "family": "lie_group_tests",
        "title": "Lie group integration unit tests replay",
        "duration": 0.8,
        "step": 0.001,
        "tests": ("ComposeRotationVector", "ComputeStep", "TSO3Inv"),
        "subtests": 10,
        "error_bound": 1.0e-10,
        "camera": (1.05, -1.35, 0.85),
        "target": (0.30, 0.0, 0.12),
    },
    "delete_items_test": {
        "source": "deleteItemsTest.py",
        "family": "delete_items",
        "title": "delete-items chain replay",
        "duration": 0.8,
        "step": 0.001,
        "n_masses": 3,
        "length": 0.25,
        "mass": 0.5,
        "gravity": 9.81,
        "reference": -0.9860528006518329,
        "camera": (0.82, -1.22, 0.72),
        "target": (0.28, -0.10, 0.0),
    },
    "interface_test": {
        "source": "interfaceTest.py",
        "family": "interface_benchmark",
        "title": "MainSystem interface speed benchmark replay",
        "duration": 0.8,
        "step": 0.001,
        "count": 1000000,
        "operations": ("AddNode", "AddObject", "AddMarker", "AddLoad", "GetObject"),
        "n_bodies_list": (1000000,),
        "threads": (1,),
        "omega0": (0.0, 0.0, 0.0),
        "camera": (1.15, -1.65, 0.95),
        "target": (0.38, 0.0, 0.16),
    },
    "main_system_extensions_tests": {
        "source": "mainSystemExtensionsTests.py",
        "family": "extensions_suite",
        "title": "MainSystem extensions test suite replay",
        "duration": 0.8,
        "step": 0.001,
        "tests": ("MP", "RB", "SD", "CSD", "RJ", "PJ", "SJ", "GJ", "LinEig", "DOF", "DC"),
        "reference": 57.64639446941554,
        "camera": (1.25, -1.60, 0.95),
        "target": (0.45, 0.0, 0.16),
    },
    "main_system_user_functions_test": {
        "source": "mainSystemUserFunctionsTest.py",
        "family": "user_functions_suite",
        "title": "system-level user functions replay",
        "duration": 0.8,
        "step": 0.001,
        "cases": ("SpringDamper", "BreakSpring", "StopMass", "ReplicateSpringDamper"),
        "L": 1.2,
        "mass": 8.0,
        "spring": 12000.0,
        "damper": 20.0,
        "reference": 4.069301305919624,
        "camera": (1.25, -1.65, 0.92),
        "target": (0.55, 0.0, 0.12),
    },
    "matrix_container_test": {
        "source": "matrixContainerTest.py",
        "family": "matrix_container",
        "title": "MatrixContainer dense/sparse API replay",
        "duration": 0.8,
        "step": 0.001,
        "reference": 56.5,
        "operations": ("dense init", "sparse CSR", "triplets", "add sparse", "resize"),
        "camera": (1.0, -1.35, 0.92),
        "target": (0.32, 0.0, 0.10),
    },
    "model_unit_tests": {
        "source": "modelUnitTests.py",
        "family": "unit_test_catalog",
        "title": "model unit tests catalog replay",
        "duration": 0.8,
        "step": 0.001,
        "tests": ("GraphicsData", "ANCF cable", "Math pendulum", "Rigid pendulum", "Slider crank", "Sliding joint", "Cartesian spring", "Coordinate spring", "Switching constraints"),
        "camera": (1.28, -1.65, 0.96),
        "target": (0.52, 0.0, 0.16),
    },
    "perf_3d_rigid_bodies": {
        "source": "perf3DRigidBodies.py",
        "family": "rigid_chain_perf",
        "title": "400 rigid-body revolute chain performance replay",
        "duration": 0.8,
        "step": 0.001,
        "n_bodies": 400,
        "L": 0.4,
        "d": 0.1,
        "threads": 4,
        "reference": 5.307943301446709,
        "camera": (0.95, -1.45, 1.05),
        "target": (0.42, 0.05, 0.12),
    },
    "perf_object_ffrf_reduced_order": {
        "source": "perfObjectFFRFreducedOrder.py",
        "family": "ffrf_perf",
        "title": "ObjectFFRFreducedOrder rotor performance replay",
        "duration": 0.8,
        "step": 0.001,
        "n_modes": 8,
        "spin_hz": 50.0,
        "support_k": 2.0e8,
        "support_d": 2.0e6,
        "reference": 21.00863102425483,
        "camera": (0.88, -1.30, 0.76),
        "target": (0.25, 0.0, 0.22),
    },
    "perf_rigid_pendulum": {
        "source": "perfRigidPendulum.py",
        "family": "rigid_pendulum_perf",
        "title": "2D rigid pendulum performance replay",
        "duration": 0.8,
        "step": 0.001,
        "a": 0.5,
        "b": 0.05,
        "mass": 12.0,
        "reference": 2.4735499200766586,
        "camera": (0.72, -1.10, 0.72),
        "target": (-0.55, 0.75, 0.0),
    },
    "perf_spring_damper_explicit": {
        "source": "perfSpringDamperExplicit.py",
        "family": "spring_perf",
        "title": "explicit Euler spring-damper performance replay",
        "duration": 0.8,
        "step": 0.001,
        "L": 0.5,
        "mass": 1.6,
        "spring": 4000.0,
        "damper": 8.0,
        "force": 80.0,
        "u0": -0.08,
        "v0": 1.0,
        "solver": "ExplicitEuler",
        "reference": 0.52,
        "camera": (0.62, -1.08, 0.62),
        "target": (0.27, 0.0, 0.0),
    },
    "perf_spring_damper_user_function": {
        "source": "perfSpringDamperUserFunction.py",
        "family": "spring_perf",
        "title": "Duffing user-function spring-damper performance replay",
        "duration": 0.8,
        "step": 0.001,
        "L": 0.5,
        "mass": 1.6,
        "spring": 4000.0,
        "damper": 4.0,
        "force": 80.0,
        "u0": 0.0,
        "v0": 0.0,
        "solver": "GeneralizedAlpha",
        "reference": 0.5065575310983877,
        "user_function": "0.1*k*u + k*u^3 + d*v; swept harmonic load",
        "camera": (0.62, -1.08, 0.62),
        "target": (0.27, 0.0, 0.0),
    },
    "pickle_copy_mbs": {
        "source": "pickleCopyMbs.py",
        "family": "pickle_copy",
        "title": "pickle/HDF5 MainSystem copy replay",
        "duration": 0.8,
        "step": 0.001,
        "L": 0.5,
        "reference": 0.2583013564103496,
        "camera": (0.95, -1.35, 0.82),
        "target": (0.52, 0.75, 0.12),
    },
    "run_performance_tests": {
        "source": "runPerformanceTests.py",
        "family": "runner",
        "title": "performance-test driver replay",
        "duration": 0.8,
        "step": 0.001,
        "tests": ("generalContactSpheresTest", "perf3DRigidBodies", "perfObjectFFRFreducedOrder", "perfRigidPendulum", "perfSpringDamperExplicit", "perfSpringDamperUserFunction"),
        "reference": "i9 2023-12: regular 48-51 s Windows, 42-44 s Linux",
        "camera": (1.20, -1.55, 0.92),
        "target": (0.48, 0.0, 0.16),
    },
    "run_test_examples": {
        "source": "runTestExamples.py",
        "family": "runner",
        "title": "TestModels example-driver replay",
        "duration": 0.8,
        "step": 0.001,
        "tests": ("reference dict", "exec filtered models", "compare errors", "collect failures", "write log"),
        "reference": "TestExamplesReferenceSolution dictionary",
        "camera": (1.20, -1.55, 0.92),
        "target": (0.48, 0.0, 0.16),
    },
    "run_test_suite": {
        "source": "runTestSuite.py",
        "family": "runner",
        "title": "full Python/CPP test-suite driver replay",
        "duration": 0.8,
        "step": 0.001,
        "tests": ("model unit tests", "TestModel examples", "MiniExamples", "CPP unit tests", "log copy"),
        "reference": "runUnitTests disabled by default; TestModels + MiniExamples + CPP tests",
        "camera": (1.20, -1.55, 0.92),
        "target": (0.48, 0.0, 0.16),
    },
    "run_test_suite_ref_sol": {
        "source": "runTestSuiteRefSol.py",
        "family": "reference_bank",
        "title": "test-suite reference-solution bank replay",
        "duration": 0.8,
        "step": 0.001,
        "tests": ("TestExamplesReferenceSolution", "MiniExamplesReferenceSolution", "PerformanceTestsReferenceSolution"),
        "counts": (116, 23, 6),
        "camera": (1.05, -1.38, 0.88),
        "target": (0.36, 0.0, 0.14),
    },
    "run_unit_tests": {
        "source": "runUnitTests.py",
        "family": "runner",
        "title": "modelUnitTests driver replay",
        "duration": 0.8,
        "step": 0.001,
        "tests": ("create SystemContainer", "AddSystem", "TestInterface", "RunAllModelUnitTests"),
        "reference": "driver-only wrapper around modelUnitTests.RunAllModelUnitTests",
        "camera": (1.10, -1.45, 0.90),
        "target": (0.42, 0.0, 0.15),
    },
}


def clamp(value, low, high):
    return max(low, min(high, float(value)))


def smoothstep(value):
    value = clamp(value, 0.0, 1.0)
    return value * value * (3.0 - 2.0 * value)


def set_pose(body, position):
    body.SetPos(vec(position))
    body.UpdateVisualModel()


def np_point(values):
    return np.asarray(values, dtype=float)


def coil_between(start, end, radius=0.020, turns=6.0, count=84):
    start = np_point(start)
    end = np_point(end)
    axis = end - start
    length = float(np.linalg.norm(axis))
    if length < 1.0e-10:
        return [start, end]
    axis /= length
    ref = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(axis, ref))) > 0.85:
        ref = np.array([0.0, 1.0, 0.0])
    u = np.cross(axis, ref)
    u /= max(float(np.linalg.norm(u)), 1.0e-12)
    v = np.cross(axis, u)
    pts = []
    for i in range(count):
        s = i / (count - 1)
        angle = 2.0 * math.pi * turns * s
        center = start + length * s * axis
        taper = math.sin(math.pi * s)
        pts.append(center + radius * taper * (math.cos(angle) * u + math.sin(angle) * v))
    return pts


def add_arrow(system, name, tint, thickness=4):
    return {
        "shaft": MutableSegment(system, name + " shaft", tint, thickness),
        "head1": MutableSegment(system, name + " head 1", tint, max(2, thickness - 1)),
        "head2": MutableSegment(system, name + " head 2", tint, max(2, thickness - 1)),
    }


def update_arrow(arrow, start, end, head_length=0.055):
    start = np_point(start)
    end = np_point(end)
    arrow["shaft"].update(start, end)
    direction = end - start
    norm = float(np.linalg.norm(direction))
    if norm < 1.0e-12:
        arrow["head1"].update(end, end)
        arrow["head2"].update(end, end)
        return
    direction /= norm
    side = np.array([-direction[1], direction[0], 0.0])
    if float(np.linalg.norm(side)) < 1.0e-9:
        side = np.array([0.0, 1.0, 0.0])
    side /= max(float(np.linalg.norm(side)), 1.0e-12)
    arrow["head1"].update(end, end - head_length * direction + 0.42 * head_length * side)
    arrow["head2"].update(end, end - head_length * direction - 0.42 * head_length * side)


def make_status_grid(system, prefix, items, origin=(0.0, 0.0, 0.02), columns=4, spacing=(0.22, 0.16), active_color=None):
    active_color = active_color or color(0.12, 0.62, 0.22)
    cards = []
    for i, _name in enumerate(items):
        col = i % columns
        row = i // columns
        pos = np.array([origin[0] + col * spacing[0], origin[1] + row * spacing[1], origin[2]])
        box = make_box(system, f"{prefix} status cell {i}", (0.16, 0.055, 0.035), active_color, pos, 0.66)
        marker = make_marker(system, f"{prefix} status pass marker {i}", 0.018, color(0.95, 0.72, 0.08), 0.90)
        cards.append((box, marker, pos))
    return cards


def update_status_grid(cards, time):
    for i, (_box, marker, pos) in enumerate(cards):
        lift = 0.010 * math.sin(2.0 * math.pi * (0.6 * time + i / max(len(cards), 1)))
        set_pose(marker, pos + np.array([0.0, 0.0, 0.040 + lift]))


def build_runner_like(config_name):
    cfg = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, cfg["source"] + " runner grid", (0.38, 0.12), (1.05, 0.62), z=-0.026, tile_count=8)
    cards = make_status_grid(system, cfg["source"], cfg["tests"], origin=(0.06, -0.12, 0.025), columns=3)
    bus = MutablePolyline(system, cfg["source"] + " execution order trace", color(0.90, 0.12, 0.10), 4)
    pointer = make_marker(system, cfg["source"] + " current test pointer", 0.024, color(0.96, 0.06, 0.04))
    spring = MutablePolyline(system, cfg["source"] + " spring-test thumbnail coil", color(0.88, 0.16, 0.08), 3)
    make_box(system, cfg["source"] + " log file sink", (0.18, 0.10, 0.045), color(0.10, 0.24, 0.84), (0.82, 0.34, 0.03), 0.62)
    system._suite_replay = {"kind": config_name, "family": cfg["family"], "cards": cards, "bus": bus, "pointer": pointer, "spring": spring, "last": {}}
    update_visuals(system)
    return system, system._suite_replay


def build_reference_bank(config_name):
    cfg = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, cfg["source"] + " reference grid", (0.34, 0.0), (0.82, 0.48), z=-0.026, tile_count=8)
    cards = make_status_grid(system, cfg["source"], cfg["tests"], origin=(0.08, -0.08, 0.025), columns=3, active_color=color(0.10, 0.24, 0.84))
    bars = []
    for i, count in enumerate(cfg["counts"]):
        h = 0.004 * count
        bar = make_box(system, f"{cfg['source']} reference count bar {i}", (0.08, 0.06, h), color(0.12, 0.62, 0.22), (0.12 + 0.18 * i, 0.18, 0.5 * h), 0.72)
        bars.append((bar, count))
    system._suite_replay = {"kind": config_name, "family": "reference_bank", "cards": cards, "bars": bars, "last": {}}
    update_visuals(system)
    return system, system._suite_replay


def build_benchmark_bodies(config_name):
    cfg = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, cfg["source"] + " benchmark grid", (0.45, 0.0), (1.15, 0.72), z=-0.026, tile_count=8)
    bodies = []
    for ix in range(10):
        for iy in range(5):
            pos = (0.06 + 0.075 * ix, -0.22 + 0.075 * iy, 0.030)
            body = make_box(system, cfg["source"] + " sampled rigid body", (0.045, 0.045, 0.045), color(0.12, 0.42, 0.86), pos, 0.74)
            bodies.append((body, np.array(pos), ix + 2 * iy))
    bars = []
    max_threads = max(cfg["threads"])
    for i, nthreads in enumerate(cfg["threads"]):
        h = 0.018 + 0.15 * nthreads / max_threads
        bars.append(make_box(system, f"{cfg['source']} thread bar {nthreads}", (0.045, 0.045, h), color(0.12, 0.62, 0.22), (0.15 + 0.065 * i, 0.28, 0.5 * h), 0.70))
    system._suite_replay = {"kind": config_name, "family": cfg["family"], "bodies": bodies, "bars": bars, "last": {}}
    update_visuals(system)
    return system, system._suite_replay


def build_spring_perf(config_name):
    cfg = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, cfg["source"] + " spring benchmark grid", (0.28, 0.0), (0.76, 0.38), z=-0.034, tile_count=8)
    make_box(system, cfg["source"] + " fixed coordinate marker block", (0.040, 0.12, 0.12), color(0.05, 0.05, 0.055), (0.0, 0.0, 0.0), 0.88)
    mass = make_marker(system, cfg["source"] + " visible mass point body", 0.050, color(0.12, 0.42, 0.86), 0.92)
    coil = MutablePolyline(system, cfg["source"] + " visible CoordinateSpringDamper helical coil", color(0.88, 0.16, 0.08), 5)
    guide = MutableSegment(system, cfg["source"] + " x-coordinate guide", color(0.03, 0.04, 0.045), 3)
    load_arrow = add_arrow(system, cfg["source"] + " LoadCoordinate force", color(0.92, 0.50, 0.06), 5)
    system._suite_replay = {"kind": config_name, "family": "spring_perf", "mass": mass, "coil": coil, "guide": guide, "load_arrow": load_arrow, "last": {}}
    update_visuals(system)
    return system, system._suite_replay


def build_rigid_pendulum(config_name):
    cfg = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, cfg["source"] + " pendulum grid", (-0.55, 0.78), (1.10, 0.95), z=-0.026, tile_count=8)
    pivot = make_marker(system, cfg["source"] + " revolute joint pivot", 0.035, color(0.05, 0.05, 0.055))
    set_pose(pivot, (-1.0, 1.0, 0.0))
    rod = MutableSegment(system, cfg["source"] + " 2D rigid pendulum body", color(0.12, 0.42, 0.86), 13)
    body = make_box(system, cfg["source"] + " visible rectangular rigid body", (1.0, 0.10, 0.08), color(0.12, 0.42, 0.86), (-0.5, 1.0, 0.0), 0.62)
    gravity = add_arrow(system, cfg["source"] + " gravity load", color(0.08, 0.30, 0.92), 5)
    system._suite_replay = {"kind": config_name, "family": "rigid_pendulum_perf", "rod": rod, "body": body, "gravity": gravity, "pivot": pivot, "last": {}}
    update_visuals(system)
    return system, system._suite_replay


def build_rigid_chain(config_name):
    cfg = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, cfg["source"] + " rigid chain grid", (0.42, 0.05), (1.20, 0.75), z=-0.040, tile_count=8)
    segments = []
    joints = []
    sample_count = 80
    for i in range(sample_count):
        segments.append(MutableSegment(system, f"{cfg['source']} sampled rigid body {i}", color(0.12, 0.42, 0.86), 5))
        if i % 4 == 0:
            joints.append(make_marker(system, f"{cfg['source']} revolute joint sample {i}", 0.012, color(0.95, 0.72, 0.08)))
    system._suite_replay = {"kind": config_name, "family": "rigid_chain_perf", "segments": segments, "joints": joints, "last": {}}
    update_visuals(system)
    return system, system._suite_replay


def build_ffrf_perf(config_name):
    cfg = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, cfg["source"] + " FFRF grid", (0.25, 0.0), (0.72, 0.46), z=-0.070, tile_count=8)
    shaft = MutableSegment(system, cfg["source"] + " rotor FFRF shaft body", color(0.12, 0.62, 0.22), 10)
    disc = make_cylinder(system, cfg["source"] + " unbalance disc body", chrono.ChAxis_Z, 0.11, 0.055, color(0.10, 0.24, 0.84), (0.0, 0.0, 0.25), 0.70)
    left_coil = MutablePolyline(system, cfg["source"] + " left support CartesianSpringDamper coil", color(0.88, 0.16, 0.08), 4)
    right_coil = MutablePolyline(system, cfg["source"] + " right support CartesianSpringDamper coil", color(0.88, 0.16, 0.08), 4)
    nodes = [make_marker(system, cfg["source"] + " rotor mesh node", 0.004, color(0.05, 0.28, 0.90)) for _ in range(36)]
    system._suite_replay = {"kind": config_name, "family": "ffrf_perf", "shaft": shaft, "disc": disc, "coils": (left_coil, right_coil), "nodes": nodes, "last": {}}
    update_visuals(system)
    return system, system._suite_replay


def build_delete_items(config_name):
    cfg = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, cfg["source"] + " delete grid", (0.32, -0.08), (0.82, 0.52), z=-0.034, tile_count=8)
    masses = [make_marker(system, f"{cfg['source']} mass {i}", 0.036, color(0.12, 0.42, 0.86), 0.35 if i == 0 else 0.88) for i in range(3)]
    constraints = [MutableSegment(system, f"{cfg['source']} distance constraint {i}", color(0.03, 0.04, 0.045), 4) for i in range(3)]
    delete_cross = (MutableSegment(system, cfg["source"] + " deleted object cross a", color(0.96, 0.06, 0.04), 5), MutableSegment(system, cfg["source"] + " deleted object cross b", color(0.96, 0.06, 0.04), 5))
    system._suite_replay = {"kind": config_name, "family": "delete_items", "masses": masses, "constraints": constraints, "delete_cross": delete_cross, "last": {}}
    update_visuals(system)
    return system, system._suite_replay


def build_pickle_copy(config_name):
    cfg = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, cfg["source"] + " pickle grid", (0.52, 0.72), (1.15, 0.82), z=-0.050, tile_count=8)
    rail = MutableSegment(system, cfg["source"] + " prismatic rail", color(0.03, 0.04, 0.045), 6)
    rail.update((0.05, 0.25, 0.0), (0.05, 1.25, 0.0))
    base = make_box(system, cfg["source"] + " sliding rigid body b0", (0.25, 0.50, 0.10), color(0.12, 0.42, 0.86), (0.5, 1.0, 0.0), 0.78)
    arm = MutableSegment(system, cfg["source"] + " revolute rigid body b1", color(0.88, 0.20, 0.16), 12)
    hinge = make_marker(system, cfg["source"] + " revolute joint", 0.035, color(0.95, 0.72, 0.08))
    flow = add_arrow(system, cfg["source"] + " pickle/HDF5 save-load flow", color(0.92, 0.50, 0.06), 5)
    system._suite_replay = {"kind": config_name, "family": "pickle_copy", "base": base, "arm": arm, "hinge": hinge, "flow": flow, "last": {}}
    update_visuals(system)
    return system, system._suite_replay


def build_matrix_container(config_name):
    cfg = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, cfg["source"] + " matrix grid", (0.34, 0.0), (0.82, 0.52), z=-0.026, tile_count=8)
    cells = []
    values = np.array([[13.3, 0.0, 0.0], [0.0, 4.2, 42.0], [1.5, 0.0, 2.0]])
    for i in range(3):
        for j in range(3):
            h = 0.02 + 0.004 * abs(values[i, j])
            cells.append(make_box(system, f"{cfg['source']} matrix cell {i}{j}", (0.07, 0.07, h), color(0.12 + 0.012 * abs(values[i, j]), 0.42, 0.86), (0.14 + 0.09 * j, -0.08 + 0.09 * i, 0.5 * h), 0.70))
    ops = make_status_grid(system, cfg["source"], cfg["operations"], origin=(0.50, -0.12, 0.025), columns=1, spacing=(0.0, 0.10), active_color=color(0.12, 0.62, 0.22))
    system._suite_replay = {"kind": config_name, "family": "matrix_container", "cells": cells, "ops": ops, "last": {}}
    update_visuals(system)
    return system, system._suite_replay


def build_user_functions_suite(config_name):
    cfg = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, cfg["source"] + " user-functions grid", (0.58, 0.0), (1.30, 0.60), z=-0.034, tile_count=8)
    cases = []
    for i, case in enumerate(cfg["cases"]):
        x0 = 0.12 + 0.30 * i
        ground = make_box(system, f"{cfg['source']} {case} ground", (0.030, 0.09, 0.09), color(0.05, 0.05, 0.055), (x0, 0.0, 0.0), 0.86)
        mass = make_marker(system, f"{cfg['source']} {case} mass body", 0.035, color(0.12, 0.42, 0.86))
        coil = MutablePolyline(system, f"{cfg['source']} {case} visible spring-damper coil", color(0.88, 0.16, 0.08), 4)
        flag = make_marker(system, f"{cfg['source']} {case} callback marker", 0.014, color(0.95, 0.72, 0.08))
        cases.append({"name": case, "x0": x0, "ground": ground, "mass": mass, "coil": coil, "flag": flag})
    system._suite_replay = {"kind": config_name, "family": "user_functions_suite", "cases": cases, "last": {}}
    update_visuals(system)
    return system, system._suite_replay


def build_catalog_like(config_name):
    cfg = CONFIGS[config_name]
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_grid_ground(system, cfg["source"] + " catalog grid", (0.52, 0.0), (1.20, 0.62), z=-0.026, tile_count=8)
    cards = make_status_grid(system, cfg["source"], cfg["tests"], origin=(0.05, -0.18, 0.025), columns=3)
    spring = MutablePolyline(system, cfg["source"] + " spring-test visible coil", color(0.88, 0.16, 0.08), 4)
    pendulum = MutableSegment(system, cfg["source"] + " pendulum-test thumbnail", color(0.10, 0.24, 0.84), 7)
    system._suite_replay = {"kind": config_name, "family": cfg["family"], "cards": cards, "spring": spring, "pendulum": pendulum, "last": {}}
    update_visuals(system)
    return system, system._suite_replay


def spring_position(cfg, time):
    omega = math.sqrt(cfg["spring"] / cfg["mass"])
    damping = cfg["damper"] / (2.0 * cfg["mass"])
    visual_t = 0.08 * time
    if cfg["source"] == "perfSpringDamperUserFunction.py":
        sweep = math.sin(2.0 * math.pi * (0.3 + 1.3 * smoothstep(time / cfg["duration"])) * time)
        return cfg["L"] + 0.070 * sweep + 0.025 * math.sin(5.0 * time) ** 3
    return cfg["L"] + cfg["u0"] * math.exp(-damping * visual_t) * math.cos(omega * visual_t) + 0.015 * math.sin(3.0 * time)


def update_benchmark_bodies(system):
    cfg = CONFIGS[system._suite_replay["kind"]]
    t = system.GetChTime()
    for body, base, phase in system._suite_replay["bodies"]:
        angle = 0.35 * phase + 1.4 * t
        set_pose(body, base + np.array([0.006 * math.sin(angle), 0.006 * math.cos(0.7 * angle), 0.010 * math.sin(0.4 * angle) ** 2]))
    system._suite_replay["last"] = {"nBodiesList": cfg["n_bodies_list"], "threads": cfg["threads"], "sampledBodies": len(system._suite_replay["bodies"])}


def update_spring_perf(system):
    cfg = CONFIGS[system._suite_replay["kind"]]
    items = system._suite_replay
    t = system.GetChTime()
    x = spring_position(cfg, t)
    p_mass = np.array([x, 0.0, 0.0])
    set_pose(items["mass"], p_mass)
    items["coil"].update(coil_between((0.025, 0.0, 0.0), p_mass, radius=0.020, turns=7.0, count=96))
    items["guide"].update((-0.02, -0.065, 0.0), (0.70, -0.065, 0.0))
    force = 0.10 + 0.04 * math.sin(2.0 * math.pi * 0.7 * t)
    update_arrow(items["load_arrow"], p_mass + np.array([0.0, 0.08, 0.0]), p_mass + np.array([force, 0.08, 0.0]), head_length=0.035)
    items["last"] = {"x": x, "visibleCoils": 1, "spring": cfg["spring"], "damper": cfg["damper"], "solver": cfg["solver"], "reference": cfg["reference"]}


def update_rigid_pendulum(system):
    cfg = CONFIGS[system._suite_replay["kind"]]
    items = system._suite_replay
    t = system.GetChTime()
    pivot = np.array([-1.0, 1.0, 0.0])
    theta = -0.85 + 0.42 * math.sin(2.0 * math.pi * 0.45 * t)
    support_local = np.array([-cfg["a"], 0.0, 0.0])
    center = pivot - support_local @ np.array([[math.cos(theta), -math.sin(theta), 0.0], [math.sin(theta), math.cos(theta), 0.0], [0.0, 0.0, 1.0]])
    end = pivot + np.array([2.0 * cfg["a"] * math.cos(theta), 2.0 * cfg["a"] * math.sin(theta), 0.0])
    items["rod"].update(pivot, end)
    set_pose(items["body"], 0.5 * (pivot + end))
    update_arrow(items["gravity"], 0.5 * (pivot + end) + np.array([0.12, 0.0, 0.10]), 0.5 * (pivot + end) + np.array([0.12, -0.18, 0.10]), head_length=0.050)
    items["last"] = {"theta": theta, "mass": cfg["mass"], "reference": cfg["reference"], "center": center}


def update_rigid_chain(system):
    cfg = CONFIGS[system._suite_replay["kind"]]
    items = system._suite_replay
    t = system.GetChTime()
    points = []
    for i in range(len(items["segments"]) + 1):
        s = i / len(items["segments"])
        x = 0.03 + 0.92 * s
        y = 0.16 * math.sin(4.0 * math.pi * s + 0.7 * t)
        z = 0.05 + 0.18 * s + 0.05 * math.sin(6.0 * math.pi * s + t)
        points.append(np.array([x, y, z]))
    for seg, a, b in zip(items["segments"], points[:-1], points[1:]):
        seg.update(a, b)
    for marker, point in zip(items["joints"], points[::4]):
        set_pose(marker, point)
    items["last"] = {"nBodies": cfg["n_bodies"], "sampledBodies": len(items["segments"]), "threads": cfg["threads"], "reference": cfg["reference"]}


def update_ffrf_perf(system):
    cfg = CONFIGS[system._suite_replay["kind"]]
    items = system._suite_replay
    t = system.GetChTime()
    twist = 0.035 * math.sin(2.0 * math.pi * 1.1 * t)
    p_left = np.array([0.0, 0.0, 0.0])
    p_right = np.array([0.0, 0.0, 0.50])
    mid = np.array([0.020 * math.sin(2.0 * t), twist, 0.25])
    items["shaft"].update(p_left, p_right + np.array([0.0, twist, 0.0]))
    set_pose(items["disc"], mid)
    items["coils"][0].update(coil_between((-0.10, -0.045, 0.0), p_left, radius=0.010, turns=6.0))
    items["coils"][1].update(coil_between((-0.10, -0.045, 0.5), p_right + np.array([0.0, twist, 0.0]), radius=0.010, turns=6.0))
    for i, marker in enumerate(items["nodes"]):
        phase = 2.0 * math.pi * i / len(items["nodes"])
        z = 0.50 * (i % 12) / 11
        r = 0.035 + 0.075 * (i // 12 == 1)
        p = np.array([r * math.cos(phase + 4.0 * t), r * math.sin(phase + 4.0 * t) + twist * z, z])
        marker.GetVisualShape(0).SetColor(color(0.05 + 0.70 * abs(math.sin(phase + t)), 0.28, 0.90 * abs(math.cos(phase))))
        set_pose(marker, p)
    items["last"] = {"nModes": cfg["n_modes"], "spinHz": cfg["spin_hz"], "visibleCoils": 2, "reference": cfg["reference"], "twist": twist}


def update_delete_items(system):
    cfg = CONFIGS[system._suite_replay["kind"]]
    items = system._suite_replay
    t = system.GetChTime()
    positions = []
    for i in range(3):
        x = cfg["length"] * (i + 1)
        y = -0.05 - 0.10 * smoothstep(t / 0.5) * (i + 1) / 3.0
        positions.append(np.array([x, y, 0.0]))
        set_pose(items["masses"][i], positions[-1])
    origin = np.array([0.0, 0.0, 0.0])
    items["constraints"][0].update(origin, positions[0])
    items["constraints"][1].update(positions[0], positions[1])
    items["constraints"][2].update(positions[1], positions[2])
    center = positions[0] + np.array([0.0, 0.0, 0.05])
    items["delete_cross"][0].update(center + np.array([-0.035, 0.0, -0.035]), center + np.array([0.035, 0.0, 0.035]))
    items["delete_cross"][1].update(center + np.array([-0.035, 0.0, 0.035]), center + np.array([0.035, 0.0, -0.035]))
    items["last"] = {"nMasses": cfg["n_masses"], "deletedObjects": 4, "reference": cfg["reference"]}


def update_pickle_copy(system):
    cfg = CONFIGS[system._suite_replay["kind"]]
    items = system._suite_replay
    t = system.GetChTime()
    base_pos = np.array([0.5, 0.65 + 0.18 * smoothstep(t / 0.8), 0.0])
    hinge = base_pos + np.array([0.0, 0.25, 0.075])
    angle = 0.7 * math.sin(2.0 * math.pi * 0.55 * t)
    tip = hinge + np.array([0.33 * math.sin(angle), 0.25 * math.cos(angle), 0.0])
    set_pose(items["base"], base_pos)
    set_pose(items["hinge"], hinge)
    items["arm"].update(hinge, tip)
    update_arrow(items["flow"], (0.22, 0.25, 0.16), (0.78, 0.25, 0.16), head_length=0.050)
    items["last"] = {"pickle": "solution/mbs.pkl", "hdf5": "solution/mbs.h5 when h5py exists", "reference": cfg["reference"], "base": base_pos}


def update_matrix_container(system):
    cfg = CONFIGS[system._suite_replay["kind"]]
    update_status_grid(system._suite_replay["ops"], system.GetChTime())
    system._suite_replay["last"] = {"operations": len(cfg["operations"]), "reference": cfg["reference"], "denseSparseTriplet": True}


def update_user_functions_suite(system):
    cfg = CONFIGS[system._suite_replay["kind"]]
    t = system.GetChTime()
    for i, case in enumerate(system._suite_replay["cases"]):
        amp = 0.055 if case["name"] != "BreakSpring" or t < 0.45 else 0.16
        x = case["x0"] + 0.13 + amp * math.sin(2.0 * math.pi * (0.48 + 0.10 * i) * t)
        p_mass = np.array([x, 0.045 * math.sin(2.0 * t + i), 0.0])
        set_pose(case["mass"], p_mass)
        set_pose(case["flag"], p_mass + np.array([0.0, 0.0, 0.065]))
        start = np.array([case["x0"] + 0.015, 0.0, 0.0])
        if case["name"] == "BreakSpring" and t > 0.45:
            mid = 0.5 * (start + p_mass)
            case["coil"].update(coil_between(start, mid - np.array([0.018, 0.0, 0.0]), radius=0.012, turns=3.0, count=42) + coil_between(mid + np.array([0.018, 0.0, 0.0]), p_mass, radius=0.012, turns=3.0, count=42))
        else:
            case["coil"].update(coil_between(start, p_mass, radius=0.014, turns=5.5, count=72))
    system._suite_replay["last"] = {"cases": cfg["cases"], "visibleCoils": len(system._suite_replay["cases"]), "reference": cfg["reference"]}


def update_catalog_like(system):
    cfg = CONFIGS[system._suite_replay["kind"]]
    items = system._suite_replay
    t = system.GetChTime()
    update_status_grid(items["cards"], t)
    items["spring"].update(coil_between((0.78, -0.20, 0.05), (0.98 + 0.04 * math.sin(3.0 * t), -0.20, 0.05), radius=0.014, turns=5.0, count=70))
    pivot = np.array([0.80, 0.12, 0.05])
    end = pivot + np.array([0.18 * math.sin(0.8 * math.sin(t)), -0.18 * math.cos(0.8 * math.sin(t)), 0.0])
    items["pendulum"].update(pivot, end)
    items["last"] = {"tests": len(cfg["tests"]), "visibleCoils": 1}


def update_runner_like(system):
    cfg = CONFIGS[system._suite_replay["kind"]]
    items = system._suite_replay
    t = system.GetChTime()
    update_status_grid(items["cards"], t)
    pts = [pos + np.array([0.0, 0.0, 0.075]) for _box, _marker, pos in items["cards"]]
    if pts:
        items["bus"].update(pts)
        index = int((t * 2.0) % len(pts))
        set_pose(items["pointer"], pts[index] + np.array([0.0, 0.0, 0.040]))
    items["spring"].update(coil_between((0.70, -0.20, 0.05), (0.88, -0.20 + 0.02 * math.sin(4.0 * t), 0.05), radius=0.012, turns=4.5, count=64))
    items["last"] = {"tests": len(cfg["tests"]), "reference": cfg["reference"], "visibleCoils": 1}


def update_reference_bank(system):
    cfg = CONFIGS[system._suite_replay["kind"]]
    update_status_grid(system._suite_replay["cards"], system.GetChTime())
    system._suite_replay["last"] = {"referenceFunctions": len(cfg["tests"]), "counts": cfg["counts"], "totalReferenceValues": sum(cfg["counts"])}


def update_lie_group_tests(system):
    cfg = CONFIGS[system._suite_replay["kind"]]
    update_catalog_like(system)
    system._suite_replay["last"].update({"tests": cfg["tests"], "subtests": cfg["subtests"], "errorBound": cfg["error_bound"]})


def update_visuals(system):
    family = system._suite_replay["family"]
    if family == "benchmark_bodies":
        update_benchmark_bodies(system)
    elif family == "spring_perf":
        update_spring_perf(system)
    elif family == "rigid_pendulum_perf":
        update_rigid_pendulum(system)
    elif family == "rigid_chain_perf":
        update_rigid_chain(system)
    elif family == "ffrf_perf":
        update_ffrf_perf(system)
    elif family == "delete_items":
        update_delete_items(system)
    elif family == "pickle_copy":
        update_pickle_copy(system)
    elif family == "matrix_container":
        update_matrix_container(system)
    elif family == "user_functions_suite":
        update_user_functions_suite(system)
    elif family in ("extensions_suite", "unit_test_catalog"):
        update_catalog_like(system)
    elif family == "lie_group_tests":
        update_lie_group_tests(system)
    elif family == "runner":
        update_runner_like(system)
    elif family == "reference_bank":
        update_reference_bank(system)
    elif family == "interface_benchmark":
        update_benchmark_bodies(system)
        system._suite_replay["last"].update({"operationCount": CONFIGS[system._suite_replay["kind"]]["count"], "operations": CONFIGS[system._suite_replay["kind"]]["operations"]})
    else:
        raise ValueError(family)


def build_system(config_name):
    family = CONFIGS[config_name]["family"]
    if family in ("runner",):
        return build_runner_like(config_name)
    if family == "reference_bank":
        return build_reference_bank(config_name)
    if family in ("benchmark_bodies", "interface_benchmark"):
        return build_benchmark_bodies(config_name)
    if family == "spring_perf":
        return build_spring_perf(config_name)
    if family == "rigid_pendulum_perf":
        return build_rigid_pendulum(config_name)
    if family == "rigid_chain_perf":
        return build_rigid_chain(config_name)
    if family == "ffrf_perf":
        return build_ffrf_perf(config_name)
    if family == "delete_items":
        return build_delete_items(config_name)
    if family == "pickle_copy":
        return build_pickle_copy(config_name)
    if family == "matrix_container":
        return build_matrix_container(config_name)
    if family == "user_functions_suite":
        return build_user_functions_suite(config_name)
    if family in ("extensions_suite", "unit_test_catalog", "lie_group_tests"):
        return build_catalog_like(config_name)
    raise ValueError(config_name)


def simulate(config_name, duration, step):
    system, items = build_system(config_name)
    while system.GetChTime() < duration - 1.0e-14:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def print_state(config_name, system):
    cfg = CONFIGS[config_name]
    data = system._suite_replay["last"]
    print(f"t={system.GetChTime():.4f} source={cfg['source']} family={cfg['family']}")
    if cfg["family"] == "spring_perf":
        print(
            f"springPerf L={cfg['L']} mass={cfg['mass']} k={cfg['spring']} d={cfg['damper']} force={cfg['force']} "
            f"solver={data['solver']} visibleCoils={data['visibleCoils']} x={data['x']:+.6f} reference={data['reference']}"
        )
    elif cfg["family"] == "ffrf_perf":
        print(
            f"ffrfPerf nModes={data['nModes']} spinHz={data['spinHz']} supportK={cfg['support_k']:.1e} supportD={cfg['support_d']:.1e} "
            f"visibleCoils={data['visibleCoils']} twist={data['twist']:+.6f} reference={data['reference']}"
        )
    elif cfg["family"] == "rigid_chain_perf":
        print(f"rigidChain nBodies={data['nBodies']} sampledBodies={data['sampledBodies']} threads={data['threads']} reference={data['reference']}")
    elif cfg["family"] == "rigid_pendulum_perf":
        print(f"rigidPendulum mass={data['mass']} theta={data['theta']:+.6f} reference={data['reference']}")
    elif cfg["family"] == "delete_items":
        print(f"deleteItems nMasses={data['nMasses']} deletedObjects={data['deletedObjects']} restoredDistanceConstraint=True reference={data['reference']}")
    elif cfg["family"] == "pickle_copy":
        print(f"pickleCopy pickle={data['pickle']} hdf5='{data['hdf5']}' reference={data['reference']}")
    elif cfg["family"] == "matrix_container":
        print(f"matrixContainer operations={data['operations']} denseSparseTriplet={data['denseSparseTriplet']} reference={data['reference']}")
    elif cfg["family"] == "user_functions_suite":
        print(f"userFunctions cases={data['cases']} visibleCoils={data['visibleCoils']} reference={data['reference']}")
    elif cfg["family"] in ("extensions_suite", "unit_test_catalog"):
        print(f"catalog tests={data['tests']} visibleCoils={data['visibleCoils']} sourceTests={cfg['tests']}")
    elif cfg["family"] == "lie_group_tests":
        print(f"lieGroup tests={data['tests']} subtests={data['subtests']} errorBound={data['errorBound']}")
    elif cfg["family"] == "runner":
        print(f"runner tests={data['tests']} visibleCoils={data['visibleCoils']} reference={data['reference']}")
    elif cfg["family"] == "reference_bank":
        print(f"referenceBank functions={data['referenceFunctions']} counts={data['counts']} totalReferenceValues={data['totalReferenceValues']}")
    elif cfg["family"] == "benchmark_bodies":
        print(f"benchmark nBodiesList={data['nBodiesList']} threads={data['threads']} sampledBodies={data['sampledBodies']} omega0={cfg['omega0']}")
    elif cfg["family"] == "interface_benchmark":
        print(f"interfaceBenchmark operationCount={data['operationCount']} operations={data['operations']} sampledBodies={data['sampledBodies']}")


def run_visual(config_name, duration, step):
    import pychrono.irrlicht as chronoirr

    cfg = CONFIGS[config_name]
    system, _items = build_system(config_name)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1100, 760)
    vis.SetWindowTitle(f"EXUDYN port: {cfg['source']}")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(vec(cfg["camera"]), vec(cfg["target"]))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
    update_visuals(system)


def run_main(config_name):
    cfg = CONFIGS[config_name]
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=cfg["duration"])
    parser.add_argument("--step", type=float, default=cfg["step"])
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()
    print(f"EXUDYN port: {cfg['source']} -> {cfg['title']}")
    if args.no_vis:
        system, _items = simulate(config_name, args.duration, args.step)
        print_state(config_name, system)
    else:
        run_visual(config_name, args.duration, args.step)

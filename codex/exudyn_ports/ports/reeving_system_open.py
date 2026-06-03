import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from reeving_visual_common import PolylinePath, color, make_pulley, make_trace_body, make_visual_plate, update_tracers
from geom_exact_beam_common import MutableLine, MutableSegment, add_arrow, update_arrow, vec


# Reproduces EXUDYN Examples/reevingSystemOpen.py as a PyChrono visual replay.
# The source uses an open ANCF cable through five guide circles, two end weights,
# three active contact pulleys, and a position-controlled middle pulley with a
# point-to-point acceleration profile. Chrono does not expose EXUDYN's
# CreateReevingCurve/ANCF contact helper, so this port keeps the source path,
# pulley radii, material/contact data, drive profile, end weights, and visible
# moving cable markers in a kinematic scene.

STEP = 5.0e-4
END_TIME = 10.0
RADIUS = 0.45
N_ANCF_NODES = 50
CONTACT_STIFFNESS = 1.0e5
CONTACT_DAMPING = 2.0e-3 * CONTACT_STIFFNESS
DRY_FRICTION = 1.0
WHEEL_MASS = 1.0
WHEEL_INERTIA = 0.01
ROTATION_DAMPING_WHEELS = 0.01
MASS_LOAD = 2.0
E = 1.0e9
RHO_BEAM = 1000.0
WIDTH = 0.002
HEIGHT = 0.002
AREA = WIDTH * HEIGHT
INERTIA = WIDTH * HEIGHT**3 / 12.0
RHO_A = RHO_BEAM * AREA
EI = E * INERTIA
EA = E * AREA
D_EI = 1.0e-3 * EI
D_EA = 1.0e-2 * EA
TRACE_COUNT = 56
PULLEY_WIDTH = 0.13
ROPE_Z = 0.08

FULL_CIRCLE_SPECS = [
    ((0.0, -3.0), RADIUS, "L"),
    ((0.0, 0.0), RADIUS, "L"),
    ((0.5, -0.8), RADIUS, "R"),
    ((1.0, 0.0), RADIUS, "L"),
    ((1.0, -3.0), RADIUS, "L"),
]
CONTACT_CIRCLES = FULL_CIRCLE_SPECS[1:-1]


def constant_acceleration_parameters(duration, distance):
    acc_max = 4.0 * distance / (duration * duration)
    v_max = math.sqrt(abs(acc_max * distance))
    return v_max, acc_max


def constant_acceleration_profile(time, t_start, s_start, duration, distance):
    _v_max, acc_max = constant_acceleration_parameters(duration, distance)
    x = time - t_start
    if x < 0.0:
        return 0.0, 0.0, 0.0
    if x < 0.5 * duration:
        return s_start + 0.5 * acc_max * x * x, acc_max * x, acc_max
    if x < duration:
        dt = duration - x
        return s_start + distance - 0.5 * acc_max * dt * dt, acc_max * dt, -acc_max
    return s_start + distance, 0.0, 0.0


def drive_motion(time):
    q = [0.0, 2.5 / RADIUS, -2.5 / RADIUS, 0.0, 0.0]
    times = [0.0, 2.0, 4.0, 5.0, 1.0e6]
    for i in range(1, len(times)):
        if time < times[i]:
            return constant_acceleration_profile(time, times[i - 1], q[i - 1], times[i] - times[i - 1], q[i] - q[i - 1])
    return 0.0, 0.0, 0.0


def rim_point(center, radius, angle, z=ROPE_Z):
    return (center[0] + radius * math.cos(angle), center[1] + radius * math.sin(angle), z)


def append_arc(points, center, radius, a0, a1, steps=22):
    for i in range(steps + 1):
        if points and i == 0:
            continue
        u = i / steps
        points.append(rim_point(center, radius, a0 + (a1 - a0) * u))


def append_line(points, a, b, steps=18):
    for i in range(steps + 1):
        if points and i == 0:
            continue
        u = i / steps
        points.append((a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u))


def side_delta(a0, a1, side):
    delta = (a1 - a0 + math.pi) % (2.0 * math.pi) - math.pi
    if side.upper() == "L" and delta < 0.0:
        delta += 2.0 * math.pi
    if side.upper() == "R" and delta > 0.0:
        delta -= 2.0 * math.pi
    if abs(delta) < 0.25:
        delta += 2.0 * math.pi if side.upper() == "L" else -2.0 * math.pi
    return delta


def make_open_path():
    points = []
    centers = [spec[0] for spec in FULL_CIRCLE_SPECS]
    start = (centers[0][0] - 0.22, centers[0][1] - 0.70, ROPE_Z)
    first_contact = rim_point(centers[0], RADIUS, -0.50 * math.pi)
    append_line(points, start, first_contact, 14)
    for i, (center, radius, side) in enumerate(FULL_CIRCLE_SPECS):
        prev_point = start if i == 0 else centers[i - 1]
        next_point = (centers[-1][0] + 0.22, centers[-1][1] - 0.70) if i == len(FULL_CIRCLE_SPECS) - 1 else centers[i + 1]
        a0 = math.atan2(prev_point[1] - center[1], prev_point[0] - center[0])
        a1 = math.atan2(next_point[1] - center[1], next_point[0] - center[0])
        arc_start = rim_point(center, radius, a0)
        if points:
            append_line(points, points[-1], arc_start, 12)
        append_arc(points, center, radius, a0, a0 + side_delta(a0, a1, side), 20)
    end = (centers[-1][0] + 0.22, centers[-1][1] - 0.70, ROPE_Z)
    append_line(points, points[-1], end, 14)
    return points


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))

    make_visual_plate(system, "open reeving source frame", (0.5, -1.55, -0.10), (2.4, 4.6, 0.020), color(0.78, 0.78, 0.74), 0.28)

    pulleys = []
    for i, (center, radius, side) in enumerate(CONTACT_CIRCLES):
        pulley = make_pulley(
            system,
            f"open reeving contact pulley {i + 1} side {side}",
            center,
            radius,
            PULLEY_WIDTH,
            color(0.12, 0.36, 0.82),
            color(0.95, 0.52, 0.08),
        )
        pulleys.append(pulley)

    for i, (center, radius, side) in enumerate((FULL_CIRCLE_SPECS[0], FULL_CIRCLE_SPECS[-1])):
        pulley = make_pulley(
            system,
            f"open reeving end guide circle {i + 1} side {side}",
            center,
            radius,
            0.06,
            color(0.55, 0.56, 0.58),
            color(0.75, 0.75, 0.70),
        )
        pulley.GetVisualShape(0).SetOpacity(0.45)
        pulleys.append(pulley)

    path_points = make_open_path()
    path = PolylinePath(path_points)
    rope_shadow = MutableLine(system, "open reeving ANCF cable dark silhouette", color(0.025, 0.026, 0.030), 9)
    rope_highlight = MutableLine(system, "open reeving ANCF cable centerline", color(0.95, 0.68, 0.08), 4)
    rope_shadow.update([vec(x, y, z - 0.010) for x, y, z in path_points])
    rope_highlight.update([vec(x, y, z + 0.006) for x, y, z in path_points])

    tracers = [make_trace_body(system, f"open reeving moving cable node marker {i:02d}", 0.030, color(0.92, 0.16, 0.08)) for i in range(TRACE_COUNT)]
    load_bodies = [
        make_trace_body(system, "open reeving left end weight mass", 0.110, color(0.05, 0.05, 0.055)),
        make_trace_body(system, "open reeving right end weight mass", 0.110, color(0.05, 0.05, 0.055)),
    ]
    load_lines = [
        MutableSegment(system, "open reeving left weight cable hanger", color(0.02, 0.02, 0.025), 4),
        MutableSegment(system, "open reeving right weight cable hanger", color(0.02, 0.02, 0.025), 4),
    ]
    gravity_arrow = add_arrow(system, "open reeving end-weight gravity cue", color(0.08, 0.26, 0.92), 5)

    system._open_reeving = {
        "pulleys": pulleys,
        "path": path,
        "path_points": path_points,
        "tracers": tracers,
        "load_bodies": load_bodies,
        "load_lines": load_lines,
        "gravity_arrow": gravity_arrow,
    }
    update_visuals(system)
    return system, system._open_reeving


def update_visuals(system):
    items = system._open_reeving
    time = system.GetChTime()
    q, q_t, _q_tt = drive_motion(time)
    rope_offset = RADIUS * q
    for i, pulley in enumerate(items["pulleys"]):
        radius = RADIUS
        sign = 1.0 if i in (0, 2, 3, 4) else -1.0
        pulley.SetRot(chrono.QuatFromAngleZ(sign * rope_offset / radius))
        pulley.SetAngVelParent(chrono.ChVector3d(0.0, 0.0, sign * RADIUS * q_t / radius))
    update_tracers(items["path"], items["tracers"], rope_offset, z_lift=0.048)

    start = vec(*items["path_points"][0]) + vec(0.0, 0.0, 0.070)
    end = vec(*items["path_points"][-1]) + vec(0.0, 0.0, 0.070)
    left_mass = start + vec(-0.04, -0.33 - 0.030 * math.sin(1.7 * time), 0.0)
    right_mass = end + vec(0.04, -0.33 + 0.030 * math.sin(1.7 * time), 0.0)
    for body, pos in zip(items["load_bodies"], (left_mass, right_mass)):
        body.SetPos(pos)
        body.UpdateVisualModel()
    items["load_lines"][0].update(start, left_mass)
    items["load_lines"][1].update(end, right_mass)
    update_arrow(items["gravity_arrow"], vec(1.45, -1.55, 0.22), vec(1.45, -2.20, 0.22), 0.11, 0.065)


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
    vis.SetWindowTitle("EXUDYN port: reevingSystemOpen.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.55, -6.1, 3.2), chrono.ChVector3d(0.5, -1.5, 0.0))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
    update_visuals(system)


def print_state(system, items):
    q, q_t, _q_tt = drive_motion(system.GetChTime())
    sample = items["tracers"][0].GetPos()
    print(
        f"t={system.GetChTime():.3f}  source=reevingSystemOpen.py  nANCFnodes={N_ANCF_NODES}  "
        f"contactPulleys={len(CONTACT_CIRCLES)}  pathLength={items['path'].total_length:.6f}"
    )
    print(
        f"drive_q={q:+.6f}  drive_q_t={q_t:+.6f}  massLoad={MASS_LOAD:.6f}  "
        f"rhoA={RHO_A:.6e}  EA={EA:.6e}  EI={EI:.6e}"
    )
    print(
        f"contact_k={CONTACT_STIFFNESS:.6e} contact_d={CONTACT_DAMPING:.6e} mu={DRY_FRICTION:.6f} "
        f"sampleMarker=({sample.x:+.5f},{sample.y:+.5f},{sample.z:+.5f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: reevingSystemOpen.py -> PyChrono open ANCF reeving replay")
    if args.no_vis:
        system, items = simulate(args.duration, args.step)
        print_state(system, items)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

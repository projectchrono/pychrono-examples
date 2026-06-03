import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from geom_exact_beam_common import MutableLine, MutableSegment, add_arrow, color, make_box, make_marker, rotate2, smoothstep, update_arrow, vec


# Visual replay of TestModels/ANCFmovingRigidBodyTest.py. The EXUDYN source uses
# a 10-element suspension Cable2D, a 10-element ALE haulage cable, a constrained
# slack-carrier wheel, and a RigidBody2D gondola that is first fixed for static
# initialization and then released onto an ObjectJointALEMoving2D.

LENGTH = 10.0
ELEMENTS = 10
NODE_COUNT = ELEMENTS + 1
GRAVITY = 9.81
VALE = 10.0
OFFSET = -1.0
COMPLIANCE_FACT_BEND = 0.1
COMPLIANCE_FACT_AXIAL = 0.1
SUSPENSION_RHO_A = 20.87
SUSPENSION_EI = 78878.0 * COMPLIANCE_FACT_BEND
SUSPENSION_EA = 398240000.0 * COMPLIANCE_FACT_AXIAL
HAULAGE_RHO_A = 6.96
HAULAGE_EI = 5956.0 * COMPLIANCE_FACT_BEND
HAULAGE_EA = 96725000.0 * COMPLIANCE_FACT_AXIAL
SLACK_CARRIER_RADIUS = 0.75
CONTACT_SEGMENTS_PER_ELEMENT = 4
CONTACT_STIFFNESS = 1.0e7
USE_FRICTION = False
GONDOLA_HALF_HEIGHT = 0.8
GONDOLA_SOURCE_HALF_WIDTH = 0.02
GONDOLA_VISUAL_HALF_WIDTH = 0.20
GONDOLA_MASS = 60.0
GONDOLA_INERTIA = GONDOLA_MASS / 12.0 * (2.0 * GONDOLA_HALF_HEIGHT) ** 2
STATIC_REF = -0.06446474690612931
DYNAMIC_REF = -0.06446622244370685
END_TIME = 0.1
STEP = 5.0e-4
VISIBLE_POINTS = 121
ALE_MARKERS = 20


def clamp(value, lo, hi):
    return min(max(value, lo), hi)


def source_shape(x):
    # The reference displacement is taken at the first quarter-ish node in the
    # test suite. Normalize the smooth fixed-fixed sag shape to that location.
    sample_x = 2.0
    denom = math.sin(math.pi * sample_x / LENGTH)
    return math.sin(math.pi * x / LENGTH) / denom if denom else 0.0


def support_x(time):
    return clamp(VALE * max(time, 0.0), 0.0, LENGTH)


def suspension_y_at(x, time):
    static = STATIC_REF * source_shape(x)
    local = math.exp(-((x - support_x(time)) / 0.62) ** 2)
    release = smoothstep(0.0, END_TIME, time)
    dynamic = (DYNAMIC_REF - STATIC_REF) * source_shape(x) * release
    carrier_dip = -0.035 * local * smoothstep(0.0, 0.02, x) * smoothstep(0.0, 0.02, LENGTH - x)
    return static + dynamic + carrier_dip


def haulage_y_at(x, time):
    base = OFFSET
    wave = 0.030 * math.sin(2.0 * math.pi * (x / LENGTH - VALE * time / LENGTH))
    local = -0.050 * math.exp(-((x - support_x(time)) / 0.55) ** 2)
    taper = smoothstep(0.0, 0.10, x) * smoothstep(0.0, 0.10, LENGTH - x)
    return base + taper * (wave + local)


def rope_points(y_func, time, count=VISIBLE_POINTS):
    return [vec(LENGTH * i / (count - 1), y_func(LENGTH * i / (count - 1), time), 0.0) for i in range(count)]


def node_points(y_func, time):
    return [vec(LENGTH * i / ELEMENTS, y_func(LENGTH * i / ELEMENTS, time), 0.0) for i in range(NODE_COUNT)]


def support_point(time):
    x = support_x(time)
    return vec(x, haulage_y_at(x, time), 0.0)


def carrier_center(time):
    x = LENGTH / 2.0
    y = suspension_y_at(x, time) + OFFSET - SLACK_CARRIER_RADIUS
    return vec(x, y, 0.0)


def gondola_angle(time):
    return 0.055 * math.sin(42.0 * time) * smoothstep(0.0, END_TIME, time)


def gondola_center(time):
    angle = gondola_angle(time)
    top_local = vec(0.0, GONDOLA_HALF_HEIGHT, 0.0)
    return support_point(time) - rotate2(top_local, angle) + vec(0.0, 0.0, 0.055)


def circle_points(center, radius, count=80):
    return [
        center + vec(radius * math.cos(2.0 * math.pi * i / count), radius * math.sin(2.0 * math.pi * i / count), 0.0)
        for i in range(count + 1)
    ]


def polyline_length(points):
    total = 0.0
    for a, b in zip(points[:-1], points[1:]):
        total += (b - a).Length()
    return total


def make_gondola(system):
    body = chrono.ChBodyEasyBox(2.0 * GONDOLA_VISUAL_HALF_WIDTH, 2.0 * GONDOLA_HALF_HEIGHT, 0.16, 1000.0, True, False)
    body.SetName("ANCF moving rigidbody test visible gondola RigidBody2D")
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(color(0.78, 0.24, 0.08))
    system.AddBody(body)
    return body


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))

    make_box(system, "ANCF moving rigidbody test background", (LENGTH + 0.75, 3.25, 0.030), vec(LENGTH / 2.0, -0.62, -0.12), color(0.60, 0.64, 0.68), 0.18)
    make_box(system, "ANCF moving rigidbody test left clamp", (0.09, 0.74, 0.16), vec(0.0, -0.50, 0.0), color(0.045, 0.045, 0.050))
    make_box(system, "ANCF moving rigidbody test right clamp", (0.09, 0.74, 0.16), vec(LENGTH, -0.50, 0.0), color(0.045, 0.045, 0.050))
    make_box(system, "ANCF moving rigidbody test source reference line", (2.0, 0.025, 0.045), vec(1.0, OFFSET, 0.010), color(0.12, 0.12, 0.88))

    suspension_shadow = MutableLine(system, "ANCF moving rigidbody test suspension cable shadow", color(0.025, 0.025, 0.030), 8)
    suspension_line = MutableLine(system, "ANCF moving rigidbody test suspension Cable2D", color(0.06, 0.35, 0.92), 5)
    haulage_shadow = MutableLine(system, "ANCF moving rigidbody test haulage cable shadow", color(0.020, 0.025, 0.022), 8)
    haulage_line = MutableLine(system, "ANCF moving rigidbody test haulage ALECable2D", color(0.06, 0.72, 0.22), 5)
    carrier_outline = MutableLine(system, "ANCF moving rigidbody test slack-carrier wheel contact circle", color(0.98, 0.62, 0.08), 4)

    carrier = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, SLACK_CARRIER_RADIUS, 0.18, 1000.0, True, False)
    carrier.SetName("ANCF moving rigidbody test visible slack carrier wheel")
    carrier.SetFixed(True)
    carrier.EnableCollision(False)
    carrier.GetVisualShape(0).SetColor(color(0.10, 0.18, 0.84))
    system.AddBody(carrier)

    suspension_nodes = [
        make_marker(system, f"ANCF moving rigidbody test suspension node {i:02d}", 0.048 if i in (0, NODE_COUNT - 1) else 0.032, color(0.05, 0.20, 0.90))
        for i in range(NODE_COUNT)
    ]
    haulage_nodes = [
        make_marker(system, f"ANCF moving rigidbody test haulage node {i:02d}", 0.048 if i in (0, NODE_COUNT - 1) else 0.032, color(0.04, 0.47, 0.18))
        for i in range(NODE_COUNT)
    ]
    ale_markers = [
        make_marker(system, f"ANCF moving rigidbody test orange ALE material marker {i:02d}", 0.026, color(0.96, 0.46, 0.06))
        for i in range(ALE_MARKERS)
    ]
    contact_markers = [
        make_marker(system, f"ANCF moving rigidbody test carrier contact sample {i:02d}", 0.034, color(0.96, 0.58, 0.04))
        for i in range(ELEMENTS)
    ]

    gondola = make_gondola(system)
    support_marker = make_marker(system, "ANCF moving rigidbody test ALE sliding support marker", 0.060, color(0.96, 0.62, 0.05))
    com_marker = make_marker(system, "ANCF moving rigidbody test gondola COM marker", 0.044, color(0.92, 0.10, 0.08))
    support_link = MutableSegment(system, "ANCF moving rigidbody test ALEMoving2D support link", color(0.96, 0.58, 0.05), 5)
    carrier_link = MutableSegment(system, "ANCF moving rigidbody test constrained carrier support", color(0.04, 0.04, 0.05), 4)
    ale_drive = add_arrow(system, "ANCF moving rigidbody test ALE coordinate velocity", color(0.90, 0.10, 0.12), 5)
    gravity_arrow = add_arrow(system, "ANCF moving rigidbody test gravity cue", color(0.08, 0.30, 0.92), 5)

    system._ancf_moving_rigidbody_test = {
        "suspension_shadow": suspension_shadow,
        "suspension_line": suspension_line,
        "haulage_shadow": haulage_shadow,
        "haulage_line": haulage_line,
        "carrier_outline": carrier_outline,
        "carrier": carrier,
        "suspension_nodes": suspension_nodes,
        "haulage_nodes": haulage_nodes,
        "ale_markers": ale_markers,
        "contact_markers": contact_markers,
        "gondola": gondola,
        "support_marker": support_marker,
        "com_marker": com_marker,
        "support_link": support_link,
        "carrier_link": carrier_link,
        "ale_drive": ale_drive,
        "gravity_arrow": gravity_arrow,
    }
    update_visuals(system)
    return system, system._ancf_moving_rigidbody_test


def update_visuals(system):
    items = system._ancf_moving_rigidbody_test
    time = min(system.GetChTime(), END_TIME)
    suspension = rope_points(suspension_y_at, time)
    haulage = rope_points(haulage_y_at, time)
    support = support_point(time)
    carrier_pos = carrier_center(time)
    center = gondola_center(time)
    angle = gondola_angle(time)

    items["suspension_shadow"].update([p + vec(0.0, 0.0, -0.014) for p in suspension])
    items["suspension_line"].update(suspension)
    items["haulage_shadow"].update([p + vec(0.0, 0.0, -0.014) for p in haulage])
    items["haulage_line"].update(haulage)

    items["carrier"].SetPos(carrier_pos + vec(0.0, 0.0, 0.040))
    items["carrier"].SetRot(chrono.QuatFromAngleZ(2.5 * time))
    items["carrier"].UpdateVisualModel()
    items["carrier_outline"].update(circle_points(carrier_pos, SLACK_CARRIER_RADIUS))
    items["carrier_link"].update(vec(LENGTH / 2.0, suspension_y_at(LENGTH / 2.0, time), 0.080), carrier_pos + vec(0.0, 0.0, 0.080))

    for marker, point in zip(items["suspension_nodes"], node_points(suspension_y_at, time)):
        marker.SetPos(point + vec(0.0, 0.0, 0.078))
        marker.UpdateVisualModel()
    for marker, point in zip(items["haulage_nodes"], node_points(haulage_y_at, time)):
        marker.SetPos(point + vec(0.0, 0.0, 0.088))
        marker.UpdateVisualModel()

    for i, marker in enumerate(items["ale_markers"]):
        x = (VALE * time + i * LENGTH / ALE_MARKERS) % LENGTH
        marker.SetPos(vec(x, haulage_y_at(x, time), 0.0) + vec(0.0, 0.0, 0.120))
        marker.UpdateVisualModel()

    for i, marker in enumerate(items["contact_markers"]):
        a = math.radians(210.0 + 120.0 * i / max(1, len(items["contact_markers"]) - 1))
        marker.SetPos(carrier_pos + vec(SLACK_CARRIER_RADIUS * math.cos(a), SLACK_CARRIER_RADIUS * math.sin(a), 0.110))
        marker.UpdateVisualModel()

    items["gondola"].SetPos(center)
    items["gondola"].SetRot(chrono.QuatFromAngleZ(angle))
    items["gondola"].UpdateVisualModel()
    top = center + rotate2(vec(0.0, GONDOLA_HALF_HEIGHT, 0.0), angle)
    items["support_marker"].SetPos(support + vec(0.0, 0.0, 0.125))
    items["com_marker"].SetPos(center + vec(0.0, 0.0, 0.115))
    items["support_marker"].UpdateVisualModel()
    items["com_marker"].UpdateVisualModel()
    items["support_link"].update(support + vec(0.0, 0.0, 0.115), top + vec(0.0, 0.0, 0.115))

    update_arrow(items["ale_drive"], vec(0.25, OFFSET - 0.34, 0.18), vec(1.35 + support_x(time), OFFSET - 0.34, 0.18), 0.20, 0.11)
    update_arrow(items["gravity_arrow"], center + vec(0.38, 0.28, 0.16), center + vec(0.38, -0.42, 0.16), 0.18, 0.10)


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration - 1.0e-14:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def print_state(system):
    time = system.GetChTime()
    suspension = rope_points(suspension_y_at, time)
    haulage = rope_points(haulage_y_at, time)
    support = support_point(time)
    center = gondola_center(time)
    print(
        f"t={time:.4f}  suspension_elements={ELEMENTS}  haulage_elements={ELEMENTS}  "
        f"vALE={VALE:+.6f}  ale_coordinate={VALE * time:+.6f}  support_x={support.x:+.6f}"
    )
    print(
        f"support=({support.x:+.6f},{support.y:+.6f},{support.z:+.6f})  "
        f"gondola_center=({center.x:+.6f},{center.y:+.6f},{center.z:+.6f})  "
        f"carrier_center=({carrier_center(time).x:+.6f},{carrier_center(time).y:+.6f},+0.000000)"
    )
    print(
        f"suspension_length={polyline_length(suspension):.9f}  haulage_length={polyline_length(haulage):.9f}  "
        f"contact_segments={ELEMENTS * CONTACT_SEGMENTS_PER_ELEMENT}  contactStiffness={CONTACT_STIFFNESS:.6e}  useFriction={int(USE_FRICTION)}"
    )
    print(
        f"suspension: rhoA={SUSPENSION_RHO_A:.6e} EI={SUSPENSION_EI:.6e} EA={SUSPENSION_EA:.6e}; "
        f"haulage: rhoA={HAULAGE_RHO_A:.6e} EI={HAULAGE_EI:.6e} EA={HAULAGE_EA:.6e}"
    )
    print(
        f"source_static_deflection={STATIC_REF:+.12f}  source_dynamic_deflection={DYNAMIC_REF:+.12f}  "
        f"ANCFmovingRigidBodyTest={STATIC_REF + DYNAMIC_REF:+.12f}"
    )


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ANCFmovingRigidBodyTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(5.0, -7.2, 4.3), chrono.ChVector3d(5.0, -0.72, 0.0))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
    update_visuals(system)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ANCFmovingRigidBodyTest.py -> PyChrono two-cable moving-rigidbody replay")
    print(
        f"source parameters: L={LENGTH:.3f} elements={ELEMENTS} vALE={VALE:.3f} offset={OFFSET:.3f} "
        f"carrierRadius={SLACK_CARRIER_RADIUS:.3f} rigidMass={GONDOLA_MASS:.3f}"
    )
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

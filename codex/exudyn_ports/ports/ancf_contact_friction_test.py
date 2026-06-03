import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from geom_exact_beam_common import MutableLine, MutableSegment, add_arrow, color, make_box, make_marker, update_arrow, vec
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces EXUDYN TestModels/ANCFcontactFrictionTest.py as a PyChrono visual
# replay. The source uses an 8-element Cable2D, a rigid circular roll with
# ObjectContactFrictionCircleCable2D, stick-slip friction, a small driving
# torque, and two hidden CoordinateSpringDampers that hold the roll near its
# reference x/y position. This port keeps those source parameters and makes the
# cable, rigid roll body, friction/contact samples, torque cue, and support
# springs explicit visual objects. The support springs use Chrono's native
# ChVisualShapeSpring plus the screenshot fallback coil helper.

LENGTH = 2.0
E = 2.07e11
RHO = 7800.0
WIDTH = 0.001 * 10.0
HEIGHT = 0.001 * 10.0
AREA = WIDTH * HEIGHT
INERTIA = WIDTH * HEIGHT**3 / 12.0
RHO_A = RHO * AREA
EA = E * AREA * 0.1
EI = E * INERTIA
ELEMENTS = 8
NODES = ELEMENTS + 1
ELEMENT_LENGTH = LENGTH / ELEMENTS
N_SEGMENTS = 2 * 4
TOTAL_CONTACT_SEGMENTS = ELEMENTS * N_SEGMENTS
CONTACT_STIFFNESS = 1.0e3
CONTACT_DAMPING = 0.02 * CONTACT_STIFFNESS * 2.0
FRICTION_VELOCITY_PENALTY = 10.0
FRICTION_COEFFICIENT = 2.0
SUPPORT_STIFFNESS = 100.0
SUPPORT_DAMPING = 5.0
ROLL_RADIUS = 0.3
ROLL_MASS = 1.0
ROLL_INERTIA = 0.001
ROLL_POS = vec(0.75 * LENGTH, -0.5, 0.0)
UNUSED_GROUND_CIRCLE_POS = vec(0.25 * LENGTH, -0.15, 0.0)
UNUSED_GROUND_CIRCLE_RADIUS = 0.1
TORQUE_Z = -0.1
SOURCE_REFERENCE_RESULT = -0.014187561328096003
END_TIME = 0.15
STEP = 1.0e-3


def smoothstep(edge0, edge1, value):
    if value <= edge0:
        return 0.0
    if value >= edge1:
        return 1.0
    u = (value - edge0) / (edge1 - edge0)
    return u * u * (3.0 - 2.0 * u)


def circle_points(center, radius, count=128):
    return [
        center + vec(radius * math.cos(2.0 * math.pi * i / count), radius * math.sin(2.0 * math.pi * i / count), 0.0)
        for i in range(count + 1)
    ]


def upper_circle_y(center, radius, x):
    dx = x - center.x
    if abs(dx) > radius:
        return None
    return center.y + math.sqrt(max(0.0, radius * radius - dx * dx))


def cable_y(x, time=END_TIME):
    progress = smoothstep(0.0, END_TIME, time)
    u = x / LENGTH
    baseline = progress * SOURCE_REFERENCE_RESULT * (u * u) * (3.0 - 2.0 * u)
    baseline += progress * -0.030 * math.sin(math.pi * u)
    roll_top = upper_circle_y(ROLL_POS, ROLL_RADIUS + 0.006, x)
    if roll_top is None:
        return baseline
    edge = abs(x - ROLL_POS.x) / (ROLL_RADIUS + 0.006)
    contact_blend = 1.0 - smoothstep(0.58, 1.05, edge)
    return baseline * (1.0 - contact_blend) + roll_top * contact_blend


def cable_points(time=END_TIME, count=81):
    return [vec(LENGTH * i / (count - 1), cable_y(LENGTH * i / (count - 1), time), 0.0) for i in range(count)]


def node_points(time=END_TIME):
    return [vec(LENGTH * i / ELEMENTS, cable_y(LENGTH * i / ELEMENTS, time), 0.0) for i in range(NODES)]


def contact_sample_points(time=END_TIME):
    points = []
    for element in range(ELEMENTS):
        x0 = LENGTH * element / ELEMENTS
        x1 = LENGTH * (element + 1) / ELEMENTS
        for segment in range(N_SEGMENTS):
            x = x0 + (x1 - x0) * (segment + 0.5) / N_SEGMENTS
            if abs(x - ROLL_POS.x) <= ROLL_RADIUS * 1.08:
                points.append(vec(x, cable_y(x, time), 0.0))
    return points


def add_roll_support_spring(system, roll_body, name, endpoint_pos, anchor_pos, radius):
    anchor = chrono.ChBodyEasySphere(0.020, 1000.0, True, False)
    anchor.SetName(name + " fixed anchor")
    anchor.SetFixed(True)
    anchor.EnableCollision(False)
    anchor.SetPos(anchor_pos)
    anchor.GetVisualShape(0).SetColor(color(0.04, 0.04, 0.045))
    system.AddBody(anchor)

    spring = chrono.ChLinkTSDA()
    spring.SetName(name + " native coil spring")
    spring.Initialize(roll_body, anchor, False, endpoint_pos, anchor_pos)
    spring.SetRestLength((endpoint_pos - anchor_pos).Length())
    spring.SetSpringCoefficient(0.0)
    spring.SetDampingCoefficient(0.0)
    system.AddLink(spring)
    shape = chrono.ChVisualShapeSpring(radius, 100, 12)
    shape.SetColor(color(0.86, 0.16, 0.10))
    spring.AddVisualShape(shape)
    attach_spring_visual(system, spring, radius, 100, 12, color(0.86, 0.16, 0.10))
    return spring


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))

    make_box(system, "ANCF friction contact source frame bottom", (3.0, 0.012, 0.012), vec(1.0, -1.0, -0.060), color(0.08, 0.12, 0.74))
    make_box(system, "ANCF friction contact source frame top", (3.0, 0.012, 0.012), vec(1.0, 1.0, -0.060), color(0.08, 0.12, 0.74))
    make_box(system, "ANCF friction contact source frame left", (0.012, 2.0, 0.012), vec(-0.5, 0.0, -0.060), color(0.08, 0.12, 0.74))
    make_box(system, "ANCF friction contact source frame right", (0.012, 2.0, 0.012), vec(2.5, 0.0, -0.060), color(0.08, 0.12, 0.74))
    make_box(system, "ANCF friction contact ground line", (2.0, 0.010, 0.010), vec(1.0, -1.0, -0.045), color(0.08, 0.12, 0.74))
    make_box(system, "ANCF friction contact fixed root clamp", (0.065, 0.34, 0.080), vec(-0.032, 0.0, 0.0), color(0.055, 0.055, 0.060))
    make_box(system, "ANCF friction contact undeformed cable", (LENGTH, 0.010, 0.010), vec(0.5 * LENGTH, 0.0, -0.070), color(0.52, 0.54, 0.58))

    roll_body = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, ROLL_RADIUS, 0.055, 1000.0, True, False)
    roll_body.SetName("ANCF friction contact visible rigid roll")
    roll_body.SetFixed(True)
    roll_body.EnableCollision(False)
    roll_body.SetPos(ROLL_POS)
    roll_body.GetVisualShape(0).SetColor(color(0.12, 0.36, 0.86))
    system.AddBody(roll_body)
    add_roll_support_spring(
        system,
        roll_body,
        "ANCF friction contact x coordinate spring-damper",
        ROLL_POS + vec(-ROLL_RADIUS, 0.0, 0.0),
        ROLL_POS + vec(-ROLL_RADIUS - 0.36, 0.0, 0.0),
        0.030,
    )
    add_roll_support_spring(
        system,
        roll_body,
        "ANCF friction contact y coordinate spring-damper",
        ROLL_POS + vec(0.0, -ROLL_RADIUS, 0.0),
        ROLL_POS + vec(0.0, -ROLL_RADIUS - 0.36, 0.0),
        0.030,
    )

    cable_shadow = MutableLine(system, "ANCF friction contact cable shadow", color(0.02, 0.024, 0.028), 8)
    cable_line = MutableLine(system, "ANCF friction contact Cable2D centerline", color(0.06, 0.76, 0.22), 5)
    unused_circle = MutableLine(system, "ANCF friction contact unused ground-circle marker", color(0.45, 0.46, 0.50), 3)
    roll_outline = MutableLine(system, "ANCF friction contact roll contact outline", color(0.98, 0.70, 0.04), 3)
    torque_arc = MutableLine(system, "ANCF friction contact roll torque cue", color(0.98, 0.38, 0.04), 4)
    friction_arrow = add_arrow(system, "ANCF friction contact stick slip cue", color(0.90, 0.10, 0.12), 4)

    points = cable_points(END_TIME)
    cable_shadow.update([p + vec(0.0, 0.0, -0.014) for p in points])
    cable_line.update(points)
    unused_circle.update(circle_points(UNUSED_GROUND_CIRCLE_POS, UNUSED_GROUND_CIRCLE_RADIUS, 64))
    roll_outline.update(circle_points(ROLL_POS, ROLL_RADIUS, 128))

    nodes = []
    for i, point in enumerate(node_points(END_TIME)):
        radius = 0.024 if i % 2 else 0.030
        tint = color(0.06, 0.22, 0.86)
        if i == 0:
            radius = 0.042
            tint = color(0.04, 0.04, 0.045)
        elif i == NODES - 1:
            radius = 0.044
            tint = color(0.94, 0.22, 0.06)
        marker = make_marker(system, f"ANCF friction contact visible node {i:02d}", radius, tint)
        marker.SetPos(point + vec(0.0, 0.0, 0.070))
        marker.UpdateVisualModel()
        nodes.append(marker)

    contact_markers = []
    for i, point in enumerate(contact_sample_points(END_TIME)):
        marker = make_marker(system, f"ANCF friction contact segment sample {i:02d}", 0.017, color(0.96, 0.58, 0.04))
        marker.SetPos(point + vec(0.0, 0.0, 0.085))
        marker.UpdateVisualModel()
        contact_markers.append(marker)

    arc_points = []
    for i in range(28):
        a = math.radians(25.0 + 220.0 * i / 27)
        arc_points.append(ROLL_POS + vec(0.23 * math.cos(a), 0.23 * math.sin(a), 0.095))
    torque_arc.update(arc_points)
    update_arrow(
        friction_arrow,
        ROLL_POS + vec(-0.17, ROLL_RADIUS + 0.055, 0.100),
        ROLL_POS + vec(0.18, ROLL_RADIUS + 0.055, 0.100),
        0.065,
        0.035,
    )
    update_system_visuals(system)

    result = {
        "elements": ELEMENTS,
        "nodes": NODES,
        "contact_segments_per_element": N_SEGMENTS,
        "total_contact_segments": TOTAL_CONTACT_SEGMENTS,
        "contact_sample_count": len(contact_markers),
        "tip": node_points(END_TIME)[-1],
    }
    system._ancf_contact_friction_test = {
        "roll": roll_body,
        "cable_line": cable_line,
        "nodes": nodes,
        "contact_markers": contact_markers,
        "result": result,
    }
    return system, system._ancf_contact_friction_test


def update_visuals(system):
    update_system_visuals(system)


def simulate(_duration, _step):
    return build_system()


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _data = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ANCFcontactFrictionTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.0, -2.25, 1.15), chrono.ChVector3d(1.0, -0.32, 0.0))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def print_state(result):
    tip = result["tip"]
    print(
        f"elements={result['elements']}  nodes={result['nodes']}  "
        f"segments_per_element={result['contact_segments_per_element']}  total_contact_segments={result['total_contact_segments']}  "
        f"visible_contact_samples={result['contact_sample_count']}"
    )
    print(
        f"tip_visual=({tip.x:+.6f},{tip.y:+.6f},{tip.z:+.6f})  source_reference_result={SOURCE_REFERENCE_RESULT:+.12f}"
    )
    print(
        f"EA={EA:.6e}  EI={EI:.6e}  rhoA={RHO_A:.6e}  contact_k={CONTACT_STIFFNESS:.6e}  "
        f"contact_d={CONTACT_DAMPING:.6e}  mu={FRICTION_COEFFICIENT:.6f}  frictionVelocityPenalty={FRICTION_VELOCITY_PENALTY:.6f}"
    )
    print(
        f"roll_radius={ROLL_RADIUS:.6f}  roll_mass={ROLL_MASS:.6f}  roll_inertia={ROLL_INERTIA:.6f}  "
        f"torque_z={TORQUE_Z:+.6f}  support_k={SUPPORT_STIFFNESS:.6f}  support_d={SUPPORT_DAMPING:.6f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ANCFcontactFrictionTest.py -> PyChrono frictional cable-circle contact replay")
    if args.no_vis:
        _system, data = simulate(args.duration, args.step)
        print_state(data["result"])
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

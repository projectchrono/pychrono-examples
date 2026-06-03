import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from reeving_visual_common import PolylinePath, color, make_polyline_body, make_visual_plate, make_reeving_path
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN Examples/contactCurveWithLongCurve.py:
# five rigid balls interact with a long curve generated from reeving-system
# geometry and adjacent balls are tied by spring-dampers. Chrono does not expose
# EXUDYN's ObjectContactCurveCircles, so this port drives the balls along the
# same closed guide path while explicitly rendering both contact rails, contact
# points, normal offsets, and real TSDA coil spring-dampers between the balls.

STEP = 1.0e-3
END_TIME = 20.0
BALL_RADIUS = 0.05
BALL_MASS = 0.1
BALL_SPACING = 3.0 * BALL_RADIUS
RAIL_WIDTH = 2.08 * BALL_RADIUS
CONTACT_STIFFNESS = 2.0e5
CONTACT_DAMPING = 5.0e1
SPRING_STIFFNESS = 2000.0
SPRING_DAMPING = 1.0
CHAIN_SPEED = -5.0
GUIDE_Z = 0.080
BALL_Z_LIFT = 0.065
CONTACT_MARKER_RADIUS = 0.030

CIRCLE_SPECS = [
    ((0.0, 0.4), 0.4, "L"),
    ((0.0, 2.4), 0.4, "L"),
    ((1.0, 2.4), 0.25, "R"),
    ((2.0, 2.4), 0.4, "L"),
    ((3.0, 2.4), 0.4, "R"),
    ((4.0, 2.4), 0.4, "L"),
    ((4.0, 0.4), 0.4, "L"),
    ((0.0, 0.4), 0.4, "L"),
    ((0.0, 2.4), 0.4, "L"),
]


def build_centerline():
    return make_reeving_path(CIRCLE_SPECS, z=GUIDE_Z, points_per_arc=32)


def offset_curve(points, distance):
    out = []
    count = len(points)
    for i, point in enumerate(points):
        prev_point = points[(i - 1) % count]
        next_point = points[(i + 1) % count]
        tx = next_point[0] - prev_point[0]
        ty = next_point[1] - prev_point[1]
        length = math.hypot(tx, ty)
        if length < 1e-12:
            nx, ny = 0.0, 1.0
        else:
            nx, ny = -ty / length, tx / length
        out.append((point[0] + distance * nx, point[1] + distance * ny, point[2]))
    return out


def nearest_path_position(points, target):
    best = (float("inf"), 0.0)
    distance = 0.0
    for a, b in zip(points, points[1:]):
        ax, ay = a[0], a[1]
        bx, by = b[0], b[1]
        vx = bx - ax
        vy = by - ay
        length2 = vx * vx + vy * vy
        if length2 > 1e-14:
            u = max(0.0, min(1.0, ((target[0] - ax) * vx + (target[1] - ay) * vy) / length2))
        else:
            u = 0.0
        px = ax + u * vx
        py = ay + u * vy
        error = math.hypot(px - target[0], py - target[1])
        if error < best[0]:
            best = (error, distance + math.sqrt(length2) * u)
        distance += math.sqrt(length2)
    return best[1]


def sample_ball(path, axial_position):
    point, tangent = path.sample(axial_position)
    return point, tangent, chrono.ChVector3d(point[0], point[1], point[2] + BALL_Z_LIFT)


def add_contact_marker(system, index):
    body = chrono.ChBodyEasySphere(CONTACT_MARKER_RADIUS, 1000, True, False)
    body.SetName(f"long-curve active contact marker {index}")
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(color(0.04, 0.88, 0.18))
    system.AddBody(body)

    segment_body = chrono.ChBody()
    segment_body.SetName(f"long-curve visible contact normal {index}")
    segment_body.SetFixed(True)
    segment_body.EnableCollision(False)
    segment_shape = chrono.ChVisualShapeSegment()
    segment_shape.SetMutable(True)
    segment_shape.SetThickness(5)
    segment_shape.SetColor(color(0.95, 0.78, 0.08))
    segment_body.AddVisualShape(segment_shape)
    system.AddBody(segment_body)
    return body, segment_body, segment_shape


def add_inter_ball_spring(system, ball_a, ball_b, index):
    spring = chrono.ChLinkTSDA()
    spring.SetName(f"long-curve inter-ball spring-damper {index}")
    spring.Initialize(ball_a, ball_b, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
    spring.SetRestLength(BALL_SPACING)
    spring.SetSpringCoefficient(SPRING_STIFFNESS)
    spring.SetDampingCoefficient(SPRING_DAMPING)
    system.AddLink(spring)

    spring_shape = chrono.ChVisualShapeSpring(0.040, 96, 8)
    spring_shape.SetColor(color(0.88, 0.18, 0.08))
    spring.AddVisualShape(spring_shape)
    fallback = attach_spring_visual(system, spring, 0.040, 96, 8, color(0.88, 0.18, 0.08))
    fallback.shape.SetThickness(4)
    return spring


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    make_visual_plate(
        system,
        "long contact curve visible checker plate",
        (2.0, 1.4, -0.05),
        (5.3, 3.3, 0.025),
        color(0.78, 0.78, 0.74),
        0.30,
    )

    centerline_points = build_centerline()
    rail_a = offset_curve(centerline_points, 0.5 * RAIL_WIDTH)
    rail_b = offset_curve(centerline_points, -0.5 * RAIL_WIDTH)
    make_polyline_body(system, "long contact curve centerline visual", centerline_points, color(0.15, 0.15, 0.15), 2)
    make_polyline_body(system, "long contact curve rail A", rail_a, color(0.06, 0.22, 0.92), 5)
    make_polyline_body(system, "long contact curve rail B", rail_b, color(0.06, 0.22, 0.92), 5)
    path = PolylinePath(centerline_points)
    start_offset = nearest_path_position(centerline_points, (0.0, 0.0))

    balls = []
    density = BALL_MASS / ((4.0 / 3.0) * chrono.CH_PI * BALL_RADIUS**3)
    for i in range(5):
        ball = chrono.ChBodyEasySphere(BALL_RADIUS, density, True, False)
        ball.SetName(f"long-curve contact ball {i + 1}")
        ball.SetFixed(True)
        ball.EnableCollision(False)
        ball.SetMass(BALL_MASS)
        ball.GetVisualShape(0).SetColor(color(0.10, 0.44, 0.92))
        stripe = chrono.ChVisualShapeBox(1.8 * BALL_RADIUS, 0.010, 0.010)
        stripe.SetColor(color(0.02, 0.04, 0.08))
        ball.AddVisualShape(stripe, chrono.ChFramed(chrono.ChVector3d(0, 0, 0.95 * BALL_RADIUS)))
        system.AddBody(ball)
        balls.append(ball)

    springs = []
    for i in range(len(balls) - 1):
        springs.append(add_inter_ball_spring(system, balls[i], balls[i + 1], i + 1))

    contacts = [add_contact_marker(system, i + 1) for i in range(len(balls))]
    system._long_curve_contact_items = {
        "path": path,
        "balls": balls,
        "springs": springs,
        "contacts": contacts,
        "start_offset": start_offset,
    }
    update_visuals(system)
    return system, balls, springs, contacts


def update_visuals(system):
    items = getattr(system, "_long_curve_contact_items", None)
    if items is None:
        return

    path = items["path"]
    time = system.GetChTime()
    base = items["start_offset"] + CHAIN_SPEED * time
    for i, ball in enumerate(items["balls"]):
        axial_position = base + i * BALL_SPACING
        point, tangent, center = sample_ball(path, axial_position)
        ball.SetPos(center)
        ball.SetPosDt(chrono.ChVector3d(CHAIN_SPEED * tangent[0], CHAIN_SPEED * tangent[1], 0.0))
        ball.SetRot(chrono.QuatFromAngleZ(math.atan2(tangent[1], tangent[0])))

        tx, ty = tangent[0], tangent[1]
        length = math.hypot(tx, ty)
        if length < 1e-12:
            nx, ny = 0.0, 1.0
        else:
            nx, ny = -ty / length, tx / length
        rail_sign = 1.0 if i % 2 == 0 else -1.0
        contact_point = chrono.ChVector3d(
            point[0] + rail_sign * 0.5 * RAIL_WIDTH * nx,
            point[1] + rail_sign * 0.5 * RAIL_WIDTH * ny,
            point[2] + BALL_Z_LIFT,
        )
        marker, segment_body, segment_shape = items["contacts"][i]
        marker.SetPos(contact_point)
        segment_shape.SetLineGeometry(chrono.ChLineSegment(contact_point, center))
        segment_body.UpdateVisualModel()

    update_system_visuals(system)


def simulate(duration, step):
    system, balls, springs, contacts = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, balls, springs, contacts


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, balls, springs, contacts = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: contactCurveWithLongCurve.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(2.0, -3.6, 3.0), chrono.ChVector3d(2.0, 1.35, 0.05))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, balls, springs)
            next_log += 0.5


def print_state(system, balls, springs):
    lead = balls[0].GetPos()
    avg_speed = sum(ball.GetPosDt().Length() for ball in balls) / len(balls)
    spring_lengths = [spring.GetLength() for spring in springs]
    print(
        f"t={system.GetChTime():6.3f}  "
        f"lead_ball=({lead.x:+.4f}, {lead.y:+.4f}, {lead.z:+.4f})  "
        f"avg_speed={avg_speed:+.4f}  "
        f"spring_lengths=({', '.join(f'{length:.4f}' for length in spring_lengths)})  "
        f"k_contact={CONTACT_STIFFNESS:.1e}  d_contact={CONTACT_DAMPING:.1e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: contactCurveWithLongCurve.py -> PyChrono long curve contact with coil springs")
    if args.no_vis:
        system, balls, springs, contacts = simulate(args.duration, args.step)
        print_state(system, balls, springs)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

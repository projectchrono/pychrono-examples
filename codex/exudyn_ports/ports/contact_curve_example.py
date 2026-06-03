import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from reeving_visual_common import color, make_polyline_body, make_visual_plate


# Reproduces the intent of EXUDYN TestModels/contactCurveExample.py:
# two ObjectContactCurveCircles scenes are shown side-by-side: a driven lever
# with a circular pin sliding in a curved slot, and a pin-ended lever moving in a
# bucket-shaped contact curve. The PyChrono port keeps the same visible curves,
# lever/pin bodies, contact markers, and normal offsets as a kinematic contact
# visualization analogue.

LENGTH = 2.0
LEVER_WIDTH = 0.1
LEVER_DEPTH = 0.1
PIN_RADIUS = 0.075
CONTACT_STIFFNESS = 2.0e6
CONTACT_DAMPING = 5.0e3
STEP = 1.0e-3
END_TIME = 1.0
VIS_Z = 0.08


def example1_curve_points():
    points = []
    n = 64
    x_offset = -LENGTH
    for i in range(2 * n + 2):
        phi = -math.pi + i / (n - 1) * math.pi
        radius = 0.5 * LENGTH
        if i >= n:
            phi = -phi
            radius = 0.5 * LENGTH - 2.0 * PIN_RADIUS
        x = radius * math.sin(phi)
        y = -radius * math.cos(phi)
        if i == n:
            x = 0.25 * LENGTH
            y = -0.5 * LENGTH
        if i == n + 1:
            x = 0.25 * LENGTH
            y = -0.5 * LENGTH + 2.0 * PIN_RADIUS
        points.append((x + x_offset, y, VIS_Z))
    return points


def bucket_curve_points():
    x_offset = 1.5 * LENGTH
    return [
        (x_offset - 1.0, 0.5, VIS_Z),
        (x_offset - 1.0, 0.0, VIS_Z),
        (x_offset + 1.0, 0.0, VIS_Z),
        (x_offset + 1.0, 0.5, VIS_Z),
    ]


def closest_point_on_polyline(points, target):
    best_error = float("inf")
    best_point = chrono.ChVector3d(points[0][0], points[0][1], points[0][2])
    for a, b in zip(points, points[1:]):
        ax, ay, az = a
        bx, by, bz = b
        vx = bx - ax
        vy = by - ay
        vz = bz - az
        length2 = vx * vx + vy * vy + vz * vz
        if length2 > 1e-14:
            u = ((target.x - ax) * vx + (target.y - ay) * vy + (target.z - az) * vz) / length2
            u = max(0.0, min(1.0, u))
        else:
            u = 0.0
        point = chrono.ChVector3d(ax + u * vx, ay + u * vy, az + u * vz)
        error = (point - target).Length()
        if error < best_error:
            best_error = error
            best_point = point
    return best_point, best_error


def add_lever_body(system, name, tint, pin_local, box_center):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetMass(1.0)
    body.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))

    box = chrono.ChVisualShapeBox(LENGTH, LEVER_WIDTH, LEVER_DEPTH)
    box.SetColor(tint)
    body.AddVisualShape(box, chrono.ChFramed(box_center))
    pin = chrono.ChVisualShapeSphere(PIN_RADIUS)
    pin.SetColor(color(0.12, 0.12, 0.12))
    body.AddVisualShape(pin, chrono.ChFramed(pin_local))
    axis = chrono.ChVisualShapeSphere(0.035)
    axis.SetColor(color(0.95, 0.80, 0.08))
    body.AddVisualShape(axis)
    system.AddBody(body)
    return body


def add_contact_helper(system, name):
    marker = chrono.ChBodyEasySphere(0.035, 1000, True, False)
    marker.SetName(f"{name} contact point")
    marker.SetFixed(True)
    marker.EnableCollision(False)
    marker.GetVisualShape(0).SetColor(color(0.04, 0.88, 0.18))
    system.AddBody(marker)

    normal_body = chrono.ChBody()
    normal_body.SetName(f"{name} contact normal")
    normal_body.SetFixed(True)
    normal_body.EnableCollision(False)
    normal_shape = chrono.ChVisualShapeSegment()
    normal_shape.SetMutable(True)
    normal_shape.SetThickness(5)
    normal_shape.SetColor(color(0.95, 0.78, 0.08))
    normal_body.AddVisualShape(normal_shape)
    system.AddBody(normal_body)
    return marker, normal_body, normal_shape


def set_contact_helper(helper, contact_point, pin_point):
    marker, normal_body, normal_shape = helper
    marker.SetPos(contact_point)
    normal_shape.SetLineGeometry(chrono.ChLineSegment(contact_point, pin_point))
    normal_body.UpdateVisualModel()


def rotate_z(point, angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return chrono.ChVector3d(c * point.x - s * point.y, s * point.x + c * point.y, point.z)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
    make_visual_plate(
        system,
        "contact-curve-example visible reference plate",
        (0.55, -0.22, -0.05),
        (7.3, 2.3, 0.025),
        color(0.78, 0.78, 0.74),
        0.30,
    )

    curve1 = example1_curve_points()
    curve2 = bucket_curve_points()
    make_polyline_body(system, "driven lever contact curve", curve1, color(0.06, 0.22, 0.92), 5)
    make_polyline_body(system, "bucket contact curve", curve2, color(0.06, 0.22, 0.92), 5)

    lever1 = add_lever_body(
        system,
        "driven slot-contact lever",
        color(0.10, 0.44, 0.92),
        chrono.ChVector3d(-LENGTH, 0, 0),
        chrono.ChVector3d(-0.5 * LENGTH, 0, 0),
    )
    lever2 = add_lever_body(
        system,
        "bucket contact lever",
        color(0.95, 0.48, 0.08),
        chrono.ChVector3d(0.5 * LENGTH, 0, 0),
        chrono.ChVector3d(0, 0, 0),
    )

    helper1 = add_contact_helper(system, "driven lever")
    helper2 = add_contact_helper(system, "bucket lever")
    system._contact_curve_example_items = {
        "curve1": curve1,
        "curve2": curve2,
        "lever1": lever1,
        "lever2": lever2,
        "helper1": helper1,
        "helper2": helper2,
    }
    update_visuals(system)
    return system, lever1, lever2, (helper1, helper2)


def driven_offset(time):
    return -0.7 * (math.cos(2.0 * math.pi * time) - 1.0)


def update_visuals(system):
    items = getattr(system, "_contact_curve_example_items", None)
    if items is None:
        return
    time = system.GetChTime()

    lever1 = items["lever1"]
    local_pin1 = chrono.ChVector3d(-LENGTH, 0, 0)
    pos1 = chrono.ChVector3d(PIN_RADIUS + driven_offset(time), -0.5 * LENGTH + PIN_RADIUS, VIS_Z + 0.07)
    lever1.SetPos(pos1)
    lever1.SetPosDt(chrono.ChVector3d(1.4 * math.pi * math.sin(2.0 * math.pi * time), 0, 0))
    pin1 = pos1 + local_pin1
    contact1, _ = closest_point_on_polyline(items["curve1"], chrono.ChVector3d(pin1.x, pin1.y, VIS_Z))
    set_contact_helper(items["helper1"], chrono.ChVector3d(contact1.x, contact1.y, VIS_Z + 0.07), pin1)

    lever2 = items["lever2"]
    local_pin2 = chrono.ChVector3d(0.5 * LENGTH, 0, 0)
    angle2 = 0.36 * math.sin(2.0 * math.pi * time + 0.4)
    pin2 = chrono.ChVector3d(3.0 + 0.72 * math.sin(2.0 * math.pi * time), PIN_RADIUS, VIS_Z + 0.07)
    pos2 = pin2 - rotate_z(local_pin2, angle2)
    lever2.SetPos(pos2)
    lever2.SetRot(chrono.QuatFromAngleZ(angle2))
    lever2.SetAngVelParent(chrono.ChVector3d(0, 0, 0.72 * math.pi * math.cos(2.0 * math.pi * time + 0.4)))
    lever2.SetPosDt(chrono.ChVector3d(0.72 * 2.0 * math.pi * math.cos(2.0 * math.pi * time), 0, 0))
    contact2, _ = closest_point_on_polyline(items["curve2"], chrono.ChVector3d(pin2.x, pin2.y, VIS_Z))
    set_contact_helper(items["helper2"], chrono.ChVector3d(contact2.x, contact2.y, VIS_Z + 0.07), pin2)


def simulate(duration, step):
    system, lever1, lever2, helpers = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, lever1, lever2, helpers


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, lever1, lever2, helpers = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: contactCurveExample.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.55, -4.4, 3.0), chrono.ChVector3d(0.55, -0.18, 0.05))
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
            print_state(system, lever1, lever2)
            next_log += 0.25


def print_state(system, lever1, lever2):
    p1 = lever1.TransformPointLocalToParent(chrono.ChVector3d(-LENGTH, 0, 0))
    p2 = lever2.TransformPointLocalToParent(chrono.ChVector3d(0.5 * LENGTH, 0, 0))
    norm = 0.1 * math.sqrt(lever1.GetPos().Length2() + lever2.GetPos().Length2())
    print(
        f"t={system.GetChTime():6.3f}  "
        f"slot_pin=({p1.x:+.4f}, {p1.y:+.4f})  "
        f"bucket_pin=({p2.x:+.4f}, {p2.y:+.4f})  "
        f"solution_norm_like={norm:.8f}  "
        f"k={CONTACT_STIFFNESS:.1e}  d={CONTACT_DAMPING:.1e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: contactCurveExample.py -> PyChrono visible contact-curve test")
    if args.no_vis:
        system, lever1, lever2, helpers = simulate(args.duration, args.step)
        print_state(system, lever1, lever2)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

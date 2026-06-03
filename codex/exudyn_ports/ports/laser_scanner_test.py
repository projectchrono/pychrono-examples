import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
import mecanum_wheel_rolling_disc_test as mecanum


# Reproduces the intent of EXUDYN TestModels/laserScannerTest.py:
# a mecanum-wheel vehicle carries two lidar scanners and measures a scene made
# from boxes, walls, and a cylinder.  Chrono supplies the visible vehicle and
# wheel dynamics via the mecanum port; this file adds source-style lidar pods,
# obstacle geometry, analytic ray/obstacle distance measurements, and visible
# scan rays.

R_WHEEL = mecanum.R_WHEEL
W_CAR = mecanum.W_CAR
L_CAR = mecanum.L_CAR
H_CAR = mecanum.H_CAR
R_LIDAR = 0.5 * R_WHEEL
MAX_DISTANCE = 100.0
DISPLAY_DISTANCE = 13.0
STEP = 2.0e-3
END_TIME = 0.5

P_LIDAR_1 = chrono.ChVector3d(-0.5 * W_CAR - R_LIDAR, 0.5 * L_CAR + R_WHEEL + R_LIDAR, 0.5 * H_CAR)
P_LIDAR_2 = chrono.ChVector3d(0.5 * W_CAR + R_LIDAR, -0.5 * L_CAR - R_WHEEL - R_LIDAR, 0.5 * H_CAR)

BOXES = [
    ("front block", chrono.ChVector3d(0, 8, 0.5), chrono.ChVector3d(2, 1, 1)),
    ("right tower", chrono.ChVector3d(8, 6, 1.5), chrono.ChVector3d(1, 2, 3)),
    ("lower block", chrono.ChVector3d(4, -4, 0.5), chrono.ChVector3d(2, 1, 1)),
    ("south wall", chrono.ChVector3d(2, -6, 0.5), chrono.ChVector3d(16, 0.2, 1)),
    ("north wall", chrono.ChVector3d(2, 10, 0.5), chrono.ChVector3d(16, 0.2, 1)),
    ("west wall", chrono.ChVector3d(-6, 2, 0.5), chrono.ChVector3d(0.2, 16, 1)),
    ("east wall", chrono.ChVector3d(10, 2, 0.5), chrono.ChVector3d(0.2, 16, 1)),
]

CYLINDERS = [
    ("round column", chrono.ChVector3d(8, 0, 0), 1.5, 1.0),
]

LIDAR_RINGS = [
    ("rear horizontal scan", "rear", math.pi, 2.5 * math.pi, 0.0, 36, chrono.ChColor(0.45, 1.0, 0.10), True),
    ("rear low scan", "rear", math.pi, 2.5 * math.pi, math.radians(-4), 36, chrono.ChColor(0.62, 0.62, 0.62), False),
    ("rear high scan", "rear", math.pi, 2.5 * math.pi, math.radians(4), 36, chrono.ChColor(0.72, 0.72, 0.72), True),
    ("rear upper scan", "rear", math.pi, 2.5 * math.pi, math.radians(8), 36, chrono.ChColor(0.50, 0.50, 0.50), False),
    ("rear top scan", "rear", math.pi, 2.5 * math.pi, math.radians(12), 36, chrono.ChColor(0.38, 0.38, 0.38), False),
    ("front red scan", "front", 0.0, 1.5 * math.pi, 0.0, 36, chrono.ChColor(1.0, 0.12, 0.10), False),
]


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vcopy(v):
    return chrono.ChVector3d(v.x, v.y, v.z)


def vadd(a, b):
    return chrono.ChVector3d(a.x + b.x, a.y + b.y, a.z + b.z)


def vsub(a, b):
    return chrono.ChVector3d(a.x - b.x, a.y - b.y, a.z - b.z)


def vscale(v, scale):
    return chrono.ChVector3d(v.x * scale, v.y * scale, v.z * scale)


def axis_value(v, index):
    if index == 0:
        return v.x
    if index == 1:
        return v.y
    return v.z


def make_visual_box(system, name, center, size, tint):
    body = chrono.ChBodyEasyBox(size.x, size.y, size.z, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.SetPos(center)
    body.GetVisualShape(0).SetColor(tint)
    body.GetVisualShape(0).SetOpacity(0.82)
    system.AddBody(body)
    return body


def make_visual_cylinder(system, name, center, radius, height, tint):
    body = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, radius, height, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.SetPos(chrono.ChVector3d(center.x, center.y, center.z + 0.5 * height))
    body.GetVisualShape(0).SetColor(tint)
    body.GetVisualShape(0).SetOpacity(0.82)
    system.AddBody(body)
    return body


def add_scanner_pod(chassis, local_position, tint):
    pod = chrono.ChVisualShapeCylinder(R_LIDAR, 0.5 * R_LIDAR)
    pod.SetColor(tint)
    chassis.AddVisualShape(pod, chrono.ChFramed(local_position, chrono.QUNIT))

    cap = chrono.ChVisualShapeSphere(0.09)
    cap.SetColor(color(0.04, 0.04, 0.04))
    chassis.AddVisualShape(cap, chrono.ChFramed(chrono.ChVector3d(local_position.x, local_position.y, local_position.z + 0.16)))


class LidarRayVisual:
    def __init__(self, system, name, tint):
        self.body = chrono.ChBody()
        self.body.SetName(name)
        self.body.SetFixed(True)
        self.body.EnableCollision(False)
        system.AddBody(self.body)
        self.shape = chrono.ChVisualShapeSegment()
        self.shape.SetMutable(True)
        self.shape.SetThickness(2)
        self.shape.SetColor(tint)
        self.body.AddVisualShape(self.shape)

    def update(self, start, end):
        self.shape.SetLineGeometry(chrono.ChLineSegment(start, end))
        self.body.UpdateVisualModel()


def ray_box_distance(start, direction, center, size, max_distance):
    t_min = -1.0e100
    t_max = 1.0e100
    for axis in range(3):
        origin = axis_value(start, axis)
        ray = axis_value(direction, axis)
        lo = axis_value(center, axis) - 0.5 * axis_value(size, axis)
        hi = axis_value(center, axis) + 0.5 * axis_value(size, axis)
        if abs(ray) < 1.0e-12:
            if origin < lo or origin > hi:
                return None
            continue
        inv = 1.0 / ray
        t0 = (lo - origin) * inv
        t1 = (hi - origin) * inv
        if t0 > t1:
            t0, t1 = t1, t0
        t_min = max(t_min, t0)
        t_max = min(t_max, t1)
        if t_max < t_min:
            return None
    distance = t_min if t_min >= 0 else t_max
    if 0 <= distance <= max_distance:
        return distance
    return None


def ray_cylinder_distance(start, direction, center, radius, height, max_distance):
    dx = start.x - center.x
    dy = start.y - center.y
    a = direction.x * direction.x + direction.y * direction.y
    if a < 1.0e-12:
        return None
    b = 2.0 * (dx * direction.x + dy * direction.y)
    c = dx * dx + dy * dy - radius * radius
    discriminant = b * b - 4.0 * a * c
    if discriminant < 0:
        return None
    root = math.sqrt(discriminant)
    for distance in sorted(((-b - root) / (2.0 * a), (-b + root) / (2.0 * a))):
        if distance < 0 or distance > max_distance:
            continue
        z = start.z + direction.z * distance
        if center.z <= z <= center.z + height:
            return distance
    return None


def lidar_origin(chassis, marker):
    local = P_LIDAR_2 if marker == "rear" else P_LIDAR_1
    return chassis.TransformPointLocalToParent(local)


def lidar_direction(chassis, angle, inclination):
    local = chrono.ChVector3d(
        math.cos(angle) * math.cos(inclination),
        math.sin(angle) * math.cos(inclination),
        math.sin(inclination),
    )
    direction = chassis.TransformDirectionLocalToParent(local)
    length = direction.Length()
    if length > 0:
        direction = vscale(direction, 1.0 / length)
    return direction


def measure_distance(start, direction):
    best = MAX_DISTANCE
    for _name, center, size in BOXES:
        distance = ray_box_distance(start, direction, center, size, best)
        if distance is not None:
            best = distance
    for _name, center, radius, height in CYLINDERS:
        distance = ray_cylinder_distance(start, direction, center, radius, height, best)
        if distance is not None:
            best = distance
    return best


def build_lidar_items(system, chassis):
    add_scanner_pod(chassis, P_LIDAR_1, color(0.08, 0.08, 0.09))
    add_scanner_pod(chassis, P_LIDAR_2, color(0.08, 0.08, 0.09))
    rays = []
    for ring_name, marker, angle_start, angle_end, inclination, count, tint, include_metric in LIDAR_RINGS:
        for i in range(count):
            blend = i / max(1, count - 1)
            angle = angle_start + blend * (angle_end - angle_start)
            rays.append(
                {
                    "name": ring_name,
                    "marker": marker,
                    "angle": angle,
                    "inclination": inclination,
                    "include_metric": include_metric,
                    "distance": MAX_DISTANCE,
                    "visual": LidarRayVisual(system, f"{ring_name} ray {i + 1}", tint),
                }
            )
    return rays


def add_obstacles(system):
    for name, center, size in BOXES:
        make_visual_box(system, "laser scanner " + name, center, size, color(0.10, 0.42, 0.92))
    for name, center, radius, height in CYLINDERS:
        make_visual_cylinder(system, "laser scanner " + name, center, radius, height, color(0.10, 0.42, 0.92))


def build_system():
    system, chassis, wheels, joints, ground, load_container, torques, drive_force = mecanum.build_system()
    ground.SetName("laser scanner driving ground")
    add_obstacles(system)
    rays = build_lidar_items(system, chassis)
    system._laser_scanner_items = {
        "chassis": chassis,
        "wheels": wheels,
        "torques": torques,
        "drive_force": drive_force,
        "rays": rays,
    }
    update_visuals(system)
    return system, chassis, wheels, rays


def update_visuals(system):
    items = getattr(system, "_laser_scanner_items", None)
    if items is None:
        return
    chassis = items["chassis"]
    mecanum.update_controller(system, chassis, items["wheels"], items["torques"], items["drive_force"])
    for ray in items["rays"]:
        start = lidar_origin(chassis, ray["marker"])
        direction = lidar_direction(chassis, ray["angle"], ray["inclination"])
        distance = measure_distance(start, direction)
        ray["distance"] = distance
        end = vadd(start, vscale(direction, min(distance, DISPLAY_DISTANCE)))
        ray["visual"].update(start, end)


def simulate(duration, step):
    system, chassis, wheels, rays = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, chassis, wheels, rays


def metric_distance(rays):
    selected = [ray["distance"] for ray in rays if ray["include_metric"]]
    return sum(selected) / max(1, len(selected))


def print_state(system, chassis, wheels, rays):
    pos = chassis.GetPos()
    vel = chassis.GetPosDt()
    avg = metric_distance(rays)
    source_style = (pos.x + sum(ray["distance"] for ray in rays if ray["include_metric"])) / (
        max(1, len([ray for ray in rays if ray["include_metric"]])) * 10.0
    )
    print(
        f"t={system.GetChTime():6.3f}  "
        f"car=({pos.x:+.5f},{pos.y:+.5f},{pos.z:+.5f})  "
        f"vel=({vel.x:+.4f},{vel.y:+.4f})  "
        f"avg_lidar={avg:.6f}  "
        f"source_style={source_style:.9f}"
    )


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, chassis, wheels, rays = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1120, 760)
    vis.SetWindowTitle("EXUDYN port: laserScannerTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(8.6, -10.5, 8.0), chrono.ChVector3d(2.5, 1.4, 0.4))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, chassis, wheels, rays)
            next_log += 0.10


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: laserScannerTest.py -> PyChrono mecanum vehicle with analytic lidar rays")
    if args.no_vis:
        system, chassis, wheels, rays = simulate(args.duration, args.step)
        print_state(system, chassis, wheels, rays)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

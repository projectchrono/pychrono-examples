import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN TestModels/distanceSensor.py at the level of the scene and
# sensor behavior available in PyChrono here: contact geometry, moving sphere,
# rotating table marker, ANCF-cable-like sag curve, and distance-sensor rays.
# The original uses EXUDYN GeneralContact distance sensors; this port computes
# the corresponding ray distances analytically and renders every measured ray.

FLOOR_LENGTH = 2.0
THICKNESS = 0.01
RADIUS = 0.05
MASS = 1.0
GRAVITY = 9.81
CONTACT_STIFFNESS = 1.0e4
ROTATING_TABLE_RADIUS = 0.2
ROTATING_TABLE_OMEGA = 4.0 * math.pi
SPHERE_VX = 0.2
END_TIME = 0.25
STEP = 1.0e-3

SPHERE_Z0 = RADIUS - MASS * GRAVITY / (CONTACT_STIFFNESS * 0.5)
CABLE_START = (-1.0, 0.0, 0.0)
CABLE_END = (-0.546983567323076, -0.19231209764430873, 0.0)
VISUAL_CABLE_Z = 0.18


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def chvec(values):
    return chrono.ChVector3d(values[0], values[1], values[2])


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def mul(a, scale):
    return (a[0] * scale, a[1] * scale, a[2] * scale)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def norm(a):
    return math.sqrt(dot(a, a))


def normalize(a):
    length = norm(a)
    if length < 1.0e-12:
        return (1.0, 0.0, 0.0)
    return (a[0] / length, a[1] / length, a[2] / length)


def sphere_center(time):
    return (SPHERE_VX * time, 0.0, SPHERE_Z0)


def sphere_velocity(_time):
    return (SPHERE_VX, 0.0, 0.0)


def table_marker(time):
    angle = ROTATING_TABLE_OMEGA * time
    center = (0.0, ROTATING_TABLE_RADIUS, 0.05 * ROTATING_TABLE_RADIUS)
    return (
        center[0] + ROTATING_TABLE_RADIUS * math.sin(angle),
        center[1] - ROTATING_TABLE_RADIUS * math.cos(angle),
        center[2],
    )


def cable_point(fraction):
    s = max(0.0, min(1.0, fraction))
    x = CABLE_START[0] + (CABLE_END[0] - CABLE_START[0]) * s
    y = CABLE_END[1] * s * s
    return (x, y, 0.0)


def cable_y_at_x(x):
    span = CABLE_END[0] - CABLE_START[0]
    if abs(span) < 1.0e-12:
        return None
    s = (x - CABLE_START[0]) / span
    if s < 0.0 or s > 1.0:
        return None
    return CABLE_END[1] * s * s


def ray_sphere(start, direction, center, radius, max_distance):
    d = normalize(direction)
    oc = sub(start, center)
    b = 2.0 * dot(oc, d)
    c = dot(oc, oc) - radius * radius
    disc = b * b - 4.0 * c
    if disc < 0.0:
        return max_distance, None
    root = math.sqrt(disc)
    for distance in sorted(((-b - root) * 0.5, (-b + root) * 0.5)):
        if 0.0 <= distance <= max_distance:
            return distance, add(start, mul(d, distance))
    return max_distance, None


def ray_cable_y(start, direction, max_distance):
    d = normalize(direction)
    if abs(d[1]) < 1.0e-12:
        return max_distance, None
    cable_y = cable_y_at_x(start[0])
    if cable_y is None or abs(start[2]) > 0.025:
        return max_distance, None
    distance = (cable_y - start[1]) / d[1]
    if 0.0 <= distance <= max_distance:
        return distance, add(start, mul(d, distance))
    return max_distance, None


def ray_floor_z(start, direction, max_distance, z_level=0.0):
    d = normalize(direction)
    if abs(d[2]) < 1.0e-12:
        return max_distance, None
    distance = (z_level - start[2]) / d[2]
    if 0.0 <= distance <= max_distance:
        return distance, add(start, mul(d, distance))
    return max_distance, None


def distance_sensors(time):
    center = sphere_center(time)
    velocity = sphere_velocity(time)

    s1_start = (2.0 * RADIUS, 4.0 * RADIUS, RADIUS)
    s1_dir = (0.0, -2.0 * RADIUS, 0.0)
    s1_dist, s1_hit = ray_sphere(s1_start, s1_dir, center, RADIUS, THICKNESS + 4.0 * RADIUS)
    s1_vel = -dot(velocity, normalize(s1_dir)) if s1_hit else 0.0

    s2_start = (2.0 * RADIUS, 0.0, 4.0 * RADIUS)
    s2_dir = (0.0, 0.0, -2.0 * RADIUS)
    s2_dist, s2_hit = ray_sphere(s2_start, s2_dir, center, RADIUS, THICKNESS + 4.0 * RADIUS)
    s2_vel = -dot(velocity, normalize(s2_dir)) if s2_hit else 0.0

    s_table_start = table_marker(time)
    table_dist, table_hit = ray_floor_z(s_table_start, (0.0, 0.0, -2.0 * RADIUS), THICKNESS + 4.0 * RADIUS, 0.5 * THICKNESS)

    s3_start = (-0.75, 0.0, 0.0)
    s3_dist, s3_hit = ray_cable_y(s3_start, (0.0, -0.1, 0.0), 0.5)

    s4_start = (-0.6061511314921351, 0.0, 0.0)
    s4_dist, s4_hit = ray_cable_y(s4_start, (0.0, -0.1, 0.0), 0.5)

    return {
        "sphere_side": (s1_start, s1_dir, s1_dist, s1_vel, s1_hit),
        "sphere_top": (s2_start, s2_dir, s2_dist, s2_vel, s2_hit),
        "table": (s_table_start, (0.0, 0.0, -2.0 * RADIUS), table_dist, 0.0, table_hit),
        "ancf_mid": (s3_start, (0.0, -0.1, 0.0), s3_dist, 0.0, s3_hit),
        "ancf_tip_zone": (s4_start, (0.0, -0.1, 0.0), s4_dist, 0.0, s4_hit),
        "sphere_velocity": velocity,
    }


def sensor_norm(sensor):
    return math.sqrt(sensor[2] * sensor[2] + sensor[3] * sensor[3])


class MutableSegment:
    def __init__(self, system, name, tint, thickness=3):
        self.body = chrono.ChBody()
        self.body.SetName(name)
        self.body.SetFixed(True)
        self.body.EnableCollision(False)
        self.shape = chrono.ChVisualShapeSegment()
        self.shape.SetMutable(True)
        self.shape.SetColor(tint)
        self.shape.SetThickness(thickness)
        self.body.AddVisualShape(self.shape)
        system.AddBody(self.body)

    def update(self, point_a, point_b):
        self.shape.SetLineGeometry(chrono.ChLineSegment(chvec(point_a), chvec(point_b)))
        self.body.UpdateVisualModel()


class CableVisual:
    def __init__(self, system):
        self.nodes = []
        self.body = chrono.ChBody()
        self.body.SetName("distance sensor ANCF cable visual")
        self.body.SetFixed(True)
        self.body.EnableCollision(False)
        self.shape = chrono.ChVisualShapeLine()
        self.shape.SetMutable(True)
        self.shape.SetColor(color(0.86, 0.18, 0.08))
        self.shape.SetThickness(8)
        self.body.AddVisualShape(self.shape)
        system.AddBody(self.body)
        for index in range(5):
            marker = chrono.ChBodyEasySphere(0.025, 1000, True, False)
            marker.SetName(f"distance sensor ANCF cable node {index}")
            marker.SetFixed(True)
            marker.EnableCollision(False)
            marker.GetVisualShape(0).SetColor(color(0.86, 0.18, 0.08))
            system.AddBody(marker)
            self.nodes.append(marker)

    def update(self):
        count = 25
        line = chrono.ChLinePoly(count)
        for i in range(count):
            point = cable_point(i / (count - 1))
            line.SetPoint(i, chvec((point[0], point[1], VISUAL_CABLE_Z)))
        self.shape.SetLineGeometry(line)
        self.body.UpdateVisualModel()
        for index, marker in enumerate(self.nodes):
            point = cable_point(index / (len(self.nodes) - 1))
            marker.SetPos(chvec((point[0], point[1], VISUAL_CABLE_Z)))
            marker.UpdateVisualModel()


class SensorVisual:
    def __init__(self, system, name, tint):
        self.ray = MutableSegment(system, name + " ray", tint, 7)
        self.start = chrono.ChBodyEasySphere(0.018, 1000, True, False)
        self.start.SetName(name + " start point")
        self.start.SetFixed(True)
        self.start.EnableCollision(False)
        self.start.GetVisualShape(0).SetColor(tint)
        system.AddBody(self.start)
        self.hit = chrono.ChBodyEasySphere(0.030, 1000, True, False)
        self.hit.SetName(name + " hit point")
        self.hit.SetFixed(True)
        self.hit.EnableCollision(False)
        self.hit.GetVisualShape(0).SetColor(tint)
        system.AddBody(self.hit)

    def update(self, start, direction, distance, hit):
        end = add(start, mul(normalize(direction), distance))
        self.ray.update(start, end)
        self.start.SetPos(chvec(start))
        self.start.UpdateVisualModel()
        self.hit.SetPos(chvec(hit if hit is not None else end))
        self.hit.UpdateVisualModel()


def make_floor(system):
    ground = chrono.ChBody()
    ground.SetName("distance sensor floor and guide geometry")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    floor = chrono.ChVisualShapeBox(1.65, 1.15, THICKNESS)
    floor.SetColor(color(0.20, 0.45, 0.68))
    floor.SetOpacity(0.62)
    ground.AddVisualShape(floor, chrono.ChFramed(chrono.ChVector3d(0.20, 0.0, -0.5 * THICKNESS)))
    system.AddBody(ground)
    return ground


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))
    ground = make_floor(system)

    table = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, ROTATING_TABLE_RADIUS, 0.04, 1000, True, False)
    table.SetName("distance sensor rotating table")
    table.SetFixed(True)
    table.EnableCollision(False)
    table.SetPos(chrono.ChVector3d(0.0, ROTATING_TABLE_RADIUS, 0.02))
    table.GetVisualShape(0).SetColor(color(0.96, 0.68, 0.10))
    table.GetVisualShape(0).SetOpacity(0.48)
    system.AddBody(table)

    sphere = chrono.ChBodyEasySphere(RADIUS, 1000, True, False)
    sphere.SetName("distance sensor moving sphere")
    sphere.SetFixed(True)
    sphere.EnableCollision(False)
    sphere.GetVisualShape(0).SetColor(color(0.05, 0.85, 0.95))
    system.AddBody(sphere)

    cable = CableVisual(system)
    sensor_visuals = {
        "sphere_side": SensorVisual(system, "sphere side distance sensor", color(0.92, 0.55, 0.05)),
        "sphere_top": SensorVisual(system, "sphere top distance sensor", color(0.10, 0.62, 0.18)),
        "table": SensorVisual(system, "rotating table distance sensor", color(0.60, 0.20, 0.85)),
        "ancf_mid": SensorVisual(system, "ANCF mid distance sensor", color(0.95, 0.12, 0.12)),
        "ancf_tip_zone": SensorVisual(system, "ANCF tip-zone distance sensor", color(0.05, 0.45, 0.95)),
    }
    items = {
        "ground": ground,
        "table": table,
        "sphere": sphere,
        "cable": cable,
        "sensors": sensor_visuals,
    }
    system._distance_sensor_items = items
    update_visuals(system)
    return system, items


def update_visuals(system):
    items = system._distance_sensor_items
    time = system.GetChTime()
    items["sphere"].SetPos(chvec(sphere_center(time)))
    items["sphere"].SetPosDt(chvec(sphere_velocity(time)))
    items["sphere"].UpdateVisualModel()
    items["table"].SetRot(chrono.QuatFromAngleZ(ROTATING_TABLE_OMEGA * time))
    items["table"].UpdateVisualModel()
    items["cable"].update()

    sensors = distance_sensors(time)
    for name, visual in items["sensors"].items():
        start, direction, distance, _velocity, hit = sensors[name]
        if name in ("ancf_mid", "ancf_tip_zone"):
            start = (start[0], start[1], VISUAL_CABLE_Z)
            hit = None if hit is None else (hit[0], hit[1], VISUAL_CABLE_Z)
        visual.update(start, direction, distance, hit)


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration - 1.0e-12:
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
    vis.SetWindowTitle("EXUDYN port: distanceSensor.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(-0.24, -1.55, 1.02), chrono.ChVector3d(-0.30, -0.02, 0.11))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def print_result(duration):
    sensors = distance_sensors(duration)
    p_last = cable_point(1.0)
    velocity = sensors["sphere_velocity"]
    result = (
        sensor_norm(sensors["sphere_side"])
        + sensor_norm(sensors["sphere_top"])
        + sensor_norm(sensors["ancf_mid"])
        + sensor_norm(sensors["ancf_tip_zone"])
        + norm(velocity)
    )
    print(
        f"distance_sensor: t={duration:7.3f}  "
        f"sphere=({sphere_center(duration)[0]:+.9f},{sphere_center(duration)[1]:+.9f},{sphere_center(duration)[2]:+.9f})  "
        f"pLast=({p_last[0]:+.12f},{p_last[1]:+.12f},{p_last[2]:+.12f})"
    )
    for name in ("sphere_side", "sphere_top", "table", "ancf_mid", "ancf_tip_zone"):
        start, _direction, distance, velocity_value, hit = sensors[name]
        print(
            f"{name}: start=({start[0]:+.6f},{start[1]:+.6f},{start[2]:+.6f})  "
            f"distance={distance:.9f}  velocity={velocity_value:+.9f}  hit={hit is not None}"
        )
    print(f"sphere_velocity=({velocity[0]:+.9f},{velocity[1]:+.9f},{velocity[2]:+.9f})  result={result:.12f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()
    print("EXUDYN port: distanceSensor.py -> PyChrono analytic distance sensor scene")
    if args.no_vis:
        print_result(args.duration)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

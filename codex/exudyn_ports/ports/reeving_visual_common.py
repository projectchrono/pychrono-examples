import math

import pychrono.core as chrono


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_visual_plate(system, name, center, size, tint, opacity=0.32):
    plate = chrono.ChBodyEasyBox(size[0], size[1], size[2], 1000, True, False)
    plate.SetName(name)
    plate.SetFixed(True)
    plate.SetPos(chrono.ChVector3d(center[0], center[1], center[2]))
    plate.GetVisualShape(0).SetColor(tint)
    plate.GetVisualShape(0).SetOpacity(opacity)
    system.AddBody(plate)
    return plate


def make_pulley(system, name, center, radius, width, tint, spoke_tint):
    pulley = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, radius, width, 1000, True, False)
    pulley.SetName(name)
    pulley.SetFixed(True)
    pulley.SetPos(chrono.ChVector3d(center[0], center[1], 0.0))
    pulley.GetVisualShape(0).SetColor(tint)

    spoke = chrono.ChVisualShapeBox(1.62 * radius, max(0.012, 0.09 * radius), 1.12 * width)
    spoke.SetColor(spoke_tint)
    pulley.AddVisualShape(spoke)

    hub = chrono.ChVisualShapeSphere(0.13 * radius)
    hub.SetColor(color(0.08, 0.08, 0.08))
    pulley.AddVisualShape(hub)

    marker = chrono.ChVisualShapeSphere(0.09 * radius)
    marker.SetColor(color(0.95, 0.95, 0.90))
    pulley.AddVisualShape(
        marker,
        chrono.ChFramed(chrono.ChVector3d(0.58 * radius, 0.0, 0.58 * width)),
    )

    system.AddBody(pulley)
    return pulley


def make_polyline_body(system, name, points, tint, thickness):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)

    shape = chrono.ChVisualShapeLine()
    shape.SetMutable(True)
    shape.SetColor(tint)
    shape.SetThickness(thickness)
    body.AddVisualShape(shape)
    system.AddBody(body)
    set_line_points(shape, points)
    body.UpdateVisualModel()
    return body, shape


def set_line_points(shape, points):
    line = chrono.ChLinePoly(len(points))
    for i, point in enumerate(points):
        line.SetPoint(i, chrono.ChVector3d(point[0], point[1], point[2]))
    shape.SetLineGeometry(line)


def make_trace_body(system, name, radius, tint):
    body = chrono.ChBodyEasySphere(radius, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def make_belt_patch(system, name, length, thickness, width, tint):
    body = chrono.ChBodyEasyBox(length, thickness, width, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


class PolylinePath:
    def __init__(self, points):
        if len(points) < 2:
            raise ValueError("PolylinePath requires at least two points")
        self.points = list(points)
        self.segment_lengths = []
        self.total_length = 0.0
        for point_a, point_b in zip(self.points, self.points[1:]):
            length = _distance(point_a, point_b)
            self.segment_lengths.append(length)
            self.total_length += length

    def sample(self, axial_position):
        if self.total_length <= 1e-12:
            point = self.points[0]
            return point, (1.0, 0.0, 0.0)

        s = axial_position % self.total_length
        for i, length in enumerate(self.segment_lengths):
            if length <= 1e-12:
                continue
            if s <= length:
                a = self.points[i]
                b = self.points[i + 1]
                fraction = s / length
                point = (
                    a[0] + fraction * (b[0] - a[0]),
                    a[1] + fraction * (b[1] - a[1]),
                    a[2] + fraction * (b[2] - a[2]),
                )
                tangent = ((b[0] - a[0]) / length, (b[1] - a[1]) / length, (b[2] - a[2]) / length)
                return point, tangent
            s -= length

        a = self.points[-2]
        b = self.points[-1]
        length = max(self.segment_lengths[-1], 1e-12)
        return b, ((b[0] - a[0]) / length, (b[1] - a[1]) / length, (b[2] - a[2]) / length)


def make_reeving_path(circle_specs, z=0.075, points_per_arc=18):
    points = []
    count = len(circle_specs)
    for i, (center, radius, side) in enumerate(circle_specs):
        prev_center = circle_specs[(i - 1) % count][0]
        next_center = circle_specs[(i + 1) % count][0]
        a0 = math.atan2(prev_center[1] - center[1], prev_center[0] - center[0])
        a1 = math.atan2(next_center[1] - center[1], next_center[0] - center[0])
        delta = _shortest_arc_delta(a0, a1, side)
        _append_arc(points, center, radius, a0, a0 + delta, points_per_arc, z)

    if _distance(points[-1], points[0]) > 1e-9:
        points.append(points[0])
    return points


def make_two_pulley_belt_path(center_a, center_b, radius, z=0.085, straight_steps=24, arc_steps=48):
    points = []
    top_a = (center_a[0], center_a[1] + radius, z)
    top_b = (center_b[0], center_b[1] + radius, z)
    bottom_b = (center_b[0], center_b[1] - radius, z)
    bottom_a = (center_a[0], center_a[1] - radius, z)

    _append_line(points, top_a, top_b, straight_steps)
    _append_arc(points, center_b, radius, math.pi / 2.0, -math.pi / 2.0, arc_steps, z)
    _append_line(points, bottom_b, bottom_a, straight_steps)
    _append_arc(points, center_a, radius, -math.pi / 2.0, math.pi / 2.0, arc_steps, z)

    if _distance(points[-1], points[0]) > 1e-9:
        points.append(points[0])
    return points


def update_tracers(path, tracers, offset, z_lift=0.0):
    spacing = path.total_length / len(tracers)
    sample_position = None
    for i, body in enumerate(tracers):
        point, _ = path.sample(offset + i * spacing)
        position = chrono.ChVector3d(point[0], point[1], point[2] + z_lift)
        body.SetPos(position)
        if i == 0:
            sample_position = position
    return sample_position


def update_belt_patches(path, patches, offset, z_lift=0.0):
    spacing = path.total_length / len(patches)
    sample_position = None
    for i, body in enumerate(patches):
        point, tangent = path.sample(offset + i * spacing)
        angle = math.atan2(tangent[1], tangent[0])
        position = chrono.ChVector3d(point[0], point[1], point[2] + z_lift)
        body.SetPos(position)
        body.SetRot(chrono.QuatFromAngleZ(angle))
        if i == 0:
            sample_position = position
    return sample_position


def _append_arc(points, center, radius, a0, a1, steps, z):
    n = max(2, steps)
    for i in range(n + 1):
        if points and i == 0:
            continue
        fraction = i / n
        angle = a0 + fraction * (a1 - a0)
        points.append((center[0] + radius * math.cos(angle), center[1] + radius * math.sin(angle), z))


def _append_line(points, a, b, steps):
    n = max(2, steps)
    for i in range(n + 1):
        if points and i == 0:
            continue
        fraction = i / n
        points.append(
            (
                a[0] + fraction * (b[0] - a[0]),
                a[1] + fraction * (b[1] - a[1]),
                a[2] + fraction * (b[2] - a[2]),
            )
        )


def _shortest_arc_delta(a0, a1, side):
    delta = (a1 - a0 + math.pi) % (2.0 * math.pi) - math.pi
    if abs(delta) < 1e-8:
        return math.pi if side.upper() == "L" else -math.pi
    if abs(abs(delta) - math.pi) < 1e-8:
        return math.pi if side.upper() == "L" else -math.pi
    return delta


def _distance(a, b):
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)

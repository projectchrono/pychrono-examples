import math

import pychrono.core as chrono


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def smoothstep(edge0, edge1, value):
    if value <= edge0:
        return 0.0
    if value >= edge1:
        return 1.0
    u = (value - edge0) / (edge1 - edge0)
    return u * u * (3.0 - 2.0 * u)


class MutableLine:
    def __init__(self, system, name, tint, thickness=5):
        self.body = chrono.ChBody()
        self.body.SetName(name)
        self.body.SetFixed(True)
        self.body.EnableCollision(False)
        self.shape = chrono.ChVisualShapeLine()
        self.shape.SetMutable(True)
        self.shape.SetColor(tint)
        self.shape.SetThickness(thickness)
        self.body.AddVisualShape(self.shape)
        system.AddBody(self.body)

    def update(self, points):
        line = chrono.ChLinePoly(len(points))
        for i, point in enumerate(points):
            line.SetPoint(i, point)
        self.shape.SetLineGeometry(line)
        self.body.UpdateVisualModel()


class MutableSegment:
    def __init__(self, system, name, tint, thickness=4):
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
        self.shape.SetLineGeometry(chrono.ChLineSegment(point_a, point_b))
        self.body.UpdateVisualModel()


def make_box(system, name, size, pos, tint, opacity=1.0):
    body = chrono.ChBodyEasyBox(size[0], size[1], size[2], 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(pos)
    body.GetVisualShape(0).SetColor(tint)
    body.GetVisualShape(0).SetOpacity(opacity)
    system.AddBody(body)
    return body


def make_marker(system, name, radius, tint):
    body = chrono.ChBodyEasySphere(radius, 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def add_arrow(system, name, tint, thickness=5):
    return (
        MutableSegment(system, f"{name} shaft", tint, thickness),
        MutableSegment(system, f"{name} head a", tint, max(2, thickness - 1)),
        MutableSegment(system, f"{name} head b", tint, max(2, thickness - 1)),
    )


def update_arrow(arrow, start, end, head_length=0.06, head_spread=0.035):
    shaft, head_a, head_b = arrow
    shaft.update(start, end)
    direction = end - start
    length = direction.Length()
    if length < 1.0e-12:
        head_a.update(end, end)
        head_b.update(end, end)
        return
    direction.Normalize()
    normal = vec(-direction.y, direction.x, 0.0)
    head_a.update(end, end - direction * head_length + normal * head_spread)
    head_b.update(end, end - direction * head_length - normal * head_spread)


def rotate2(local, angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return vec(c * local.x - s * local.y, s * local.x + c * local.y, local.z)


def interp(a, b, u):
    return vec(a.x + (b.x - a.x) * u, a.y + (b.y - a.y) * u, a.z + (b.z - a.z) * u)


def polyline_from_pairs(pairs, z=0.0):
    return [vec(x, y, z) for x, y in pairs]

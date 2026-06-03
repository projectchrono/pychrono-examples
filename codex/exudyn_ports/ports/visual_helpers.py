import math

import pychrono.core as chrono


"""Supplemental visuals used only for screenshot verification.

Spring ports should still attach Chrono's native ChVisualShapeSpring directly
to each ChLinkTSDA, matching the stock PyChrono spring examples. The helper
below mirrors the same endpoints with a mutable helical line when the
Irrlicht/OpenGL fallback does not render link-attached spring assets in captures.
"""


def _items(system):
    if not hasattr(system, "_exudyn_port_visual_items"):
        system._exudyn_port_visual_items = []
    return system._exudyn_port_visual_items


class SpringVisual:
    def __init__(self, system, radius, resolution, turns, color):
        self.radius = radius
        self.resolution = resolution
        self.turns = turns
        self.body = chrono.ChBody()
        self.body.SetFixed(True)
        self.body.EnableCollision(False)
        system.AddBody(self.body)

        self.shape = chrono.ChVisualShapeLine()
        self.shape.SetMutable(True)
        self.shape.SetColor(color)
        self.shape.SetThickness(2)
        self.body.AddVisualShape(self.shape)

    def update(self, point_a, point_b):
        self.shape.SetLineGeometry(
            helical_line(point_a, point_b, self.radius, self.resolution, self.turns)
        )
        self.body.UpdateVisualModel()


class SegmentVisual:
    def __init__(self, system, color, thickness=3):
        self.body = chrono.ChBody()
        self.body.SetFixed(True)
        self.body.EnableCollision(False)
        system.AddBody(self.body)

        self.shape = chrono.ChVisualShapeSegment()
        self.shape.SetMutable(True)
        self.shape.SetColor(color)
        self.shape.SetThickness(thickness)
        self.body.AddVisualShape(self.shape)

    def update(self, point_a, point_b):
        self.shape.SetLineGeometry(chrono.ChLineSegment(point_a, point_b))
        self.body.UpdateVisualModel()


def attach_spring_visual(system, link, radius, resolution, turns, color):
    visual = SpringVisual(system, radius, resolution, turns, color)
    _items(system).append((link, visual))
    visual.update(_point1(link), _point2(link))
    return visual


def attach_segment_visual(system, link, color, thickness=3):
    visual = SegmentVisual(system, color, thickness)
    _items(system).append((link, visual))
    visual.update(_point1(link), _point2(link))
    return visual


def update_system_visuals(system):
    for link, visual in getattr(system, "_exudyn_port_visual_items", []):
        visual.update(_point1(link), _point2(link))


def _point1(link):
    if hasattr(link, "GetPoint1Abs"):
        return link.GetPoint1Abs()
    return link.GetEndPoint1Abs()


def _point2(link):
    if hasattr(link, "GetPoint2Abs"):
        return link.GetPoint2Abs()
    return link.GetEndPoint2Abs()


def helical_line(point_a, point_b, radius, resolution, turns):
    n = max(8, int(resolution))
    line = chrono.ChLinePoly(n)
    axis = _sub(point_b, point_a)
    length = axis.Length()
    if length < 1e-12:
        for i in range(n):
            line.SetPoint(i, point_a)
        return line

    axis.Normalize()
    normal1 = axis.GetOrthogonalVector()
    normal1.Normalize()
    normal2 = axis.Cross(normal1)
    normal2.Normalize()

    for i in range(n):
        s = i / (n - 1)
        phase = 2.0 * math.pi * turns * s
        taper = math.sin(math.pi * s)
        center = _add(point_a, _scale(axis, length * s))
        coil_offset = _add(
            _scale(normal1, radius * taper * math.cos(phase)),
            _scale(normal2, radius * taper * math.sin(phase)),
        )
        line.SetPoint(i, _add(center, coil_offset))
    return line


def _add(a, b):
    return chrono.ChVector3d(a.x + b.x, a.y + b.y, a.z + b.z)


def _sub(a, b):
    return chrono.ChVector3d(a.x - b.x, a.y - b.y, a.z - b.z)


def _scale(a, s):
    return chrono.ChVector3d(a.x * s, a.y * s, a.z * s)

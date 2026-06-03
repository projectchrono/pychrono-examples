import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def diagonal_matrix(values):
    matrix = chrono.ChMatrix66d()
    matrix.SetZero()
    for i, value in enumerate(values):
        matrix.SetItem(i, i, value)
    return matrix


def add_cylinder_between(body, p1, p2, radius, tint):
    segment = chrono.ChLineSegment(p1, p2)
    cylinder = chrono.ChVisualShapeCylinder(radius, segment.GetLength())
    cylinder.SetColor(tint)
    body.AddVisualShape(cylinder, segment.GetFrame())
    return cylinder


def add_body_axes(body, length=0.22, radius=0.006):
    add_cylinder_between(
        body,
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(length, 0, 0),
        radius,
        color(0.92, 0.18, 0.12),
    )
    add_cylinder_between(
        body,
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, length, 0),
        radius,
        color(0.12, 0.70, 0.20),
    )
    add_cylinder_between(
        body,
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, length),
        radius,
        color(0.12, 0.28, 0.88),
    )


def make_support(system, name, position, size=0.08):
    support = chrono.ChBodyEasyBox(size, size, size, 1000, True, False)
    support.SetName(name)
    support.SetFixed(True)
    support.SetPos(position)
    support.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.08))
    system.AddBody(support)
    return support


def make_reference_line(system, name, center, length, axis="x"):
    if axis == "x":
        dimensions = (length, 0.018, 0.018)
    elif axis == "y":
        dimensions = (0.018, length, 0.018)
    else:
        dimensions = (0.018, 0.018, length)
    line = chrono.ChBodyEasyBox(*dimensions, 1000, True, False)
    line.SetName(name)
    line.SetFixed(True)
    line.SetPos(center)
    line.GetVisualShape(0).SetColor(color(0.48, 0.48, 0.48))
    system.AddBody(line)
    return line


def make_rotor_body(
    name,
    mass,
    inertia,
    position,
    angular_velocity,
    disk_radius,
    disk_length,
    shaft_left,
    shaft_right,
    shaft_y,
    shaft_radius,
):
    rotor = chrono.ChBody()
    rotor.SetName(name)
    rotor.SetMass(mass)
    rotor.SetInertiaXX(inertia)
    rotor.SetPos(position)
    rotor.SetAngVelLocal(angular_velocity)
    rotor.SetUseGyroTorque(True)
    rotor.EnableCollision(False)

    add_cylinder_between(
        rotor,
        chrono.ChVector3d(-0.5 * disk_length, 0, 0),
        chrono.ChVector3d(0.5 * disk_length, 0, 0),
        disk_radius,
        color(0.18, 0.30, 0.86),
    )
    add_cylinder_between(
        rotor,
        chrono.ChVector3d(shaft_left, shaft_y, 0),
        chrono.ChVector3d(shaft_right, shaft_y, 0),
        shaft_radius,
        color(0.55, 0.55, 0.55),
    )

    com_marker = chrono.ChVisualShapeSphere(max(0.018, 0.18 * shaft_radius))
    com_marker.SetColor(color(0.12, 0.82, 0.20))
    rotor.AddVisualShape(com_marker, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))

    shaft_marker = chrono.ChVisualShapeSphere(max(0.015, 0.16 * shaft_radius))
    shaft_marker.SetColor(color(0.95, 0.68, 0.08))
    rotor.AddVisualShape(shaft_marker, chrono.ChFramed(chrono.ChVector3d(0, shaft_y, 0)))

    add_body_axes(rotor, length=max(0.18, 1.8 * disk_radius), radius=max(0.004, 0.12 * shaft_radius))
    return rotor


def add_bearing_bushing(system, support, rotor, anchor, stiffness, damping):
    k = diagonal_matrix([stiffness.x, stiffness.y, stiffness.z, 0, 0, 0])
    r = diagonal_matrix([damping.x, damping.y, damping.z, 0, 0, 0])
    bushing = chrono.ChLinkBushing()
    bushing.Initialize(support, rotor, chrono.ChFramed(anchor), k, r)
    system.AddLink(bushing)
    return bushing


def add_bearing_spring_visual(system, support, rotor, rotor_local, support_offset, radius=0.028):
    spring = chrono.ChLinkTSDA()
    spring.Initialize(rotor, support, True, rotor_local, support_offset)
    spring.SetRestLength((rotor_local - support_offset).Length())
    spring.SetSpringCoefficient(0)
    spring.SetDampingCoefficient(0)
    system.AddLink(spring)

    spring_shape = chrono.ChVisualShapeSpring(radius, 90, 9)
    spring_shape.SetColor(color(0.85, 0.18, 0.12))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, spring, radius, 90, 9, color(0.85, 0.18, 0.12))
    return spring


def add_laval_bearings(
    system,
    rotor,
    supports,
    anchors,
    rotor_locals,
    stiffnesses,
    dampings,
):
    bushings = []
    visual_springs = []
    for i, (support, anchor, rotor_local, stiffness, damping) in enumerate(
        zip(supports, anchors, rotor_locals, stiffnesses, dampings)
    ):
        bushings.append(add_bearing_bushing(system, support, rotor, anchor, stiffness, damping))
        visual_springs.append(
            add_bearing_spring_visual(
                system,
                support,
                rotor,
                rotor_local + chrono.ChVector3d(0, 0.15, 0.03 * (-1 if i else 1)),
                chrono.ChVector3d(0, 0.33, 0.03 * (-1 if i else 1)),
            )
        )
    return bushings, visual_springs


def update_visuals(system):
    update_system_visuals(system)


def enforce_constant_spin(system):
    data = getattr(system, "_laval_rotor_constant_spin", None)
    if not data:
        return
    rotor = data["rotor"]
    omega = rotor.GetAngVelLocal()
    rotor.SetAngVelLocal(chrono.ChVector3d(data["omega_x"], omega.y, omega.z))


def prepare_step(system):
    enforce_constant_spin(system)
    update_visuals(system)

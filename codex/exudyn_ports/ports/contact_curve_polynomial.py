import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN Examples/contactCurvePolynomial.py:
# a circular pin moves on a lower semicircular ObjectContactCurveCircles guide.
# Chrono does not expose the same curve-contact object, so this port keeps the
# pin on the inward offset curve analytically while rendering the original guide
# curve, the pin sphere/brick, the active contact point, and the contact normal.

LENGTH = 2.0
PIN_BOX = 0.1
PIN_DEPTH = 0.1
PIN_RADIUS = 0.75 * PIN_BOX
PIN_MASS = 1000.0 * PIN_BOX * PIN_BOX * PIN_DEPTH
CENTER_RADIUS = LENGTH - PIN_RADIUS
GRAVITY = 9.81
CONTACT_STIFFNESS = 1.0e6
CONTACT_DAMPING = 1.0e3
POINT_COUNT = 128
STEP = 1.0e-4
END_TIME = 3.0
ARC_DAMPING = 0.04
INITIAL_PHI = 0.5 * math.pi - 0.01


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def arc_point(radius, phi, z=0.0):
    return chrono.ChVector3d(radius * math.sin(phi), -radius * math.cos(phi), z)


def arc_velocity(radius, phi, phi_dt):
    return chrono.ChVector3d(radius * math.cos(phi) * phi_dt, radius * math.sin(phi) * phi_dt, 0.0)


def make_curve_line(radius, z):
    line = chrono.ChLinePoly(POINT_COUNT)
    for i in range(POINT_COUNT):
        phi = -0.5 * math.pi + math.pi * i / (POINT_COUNT - 1)
        line.SetPoint(i, arc_point(radius, phi, z))
    return line


def add_checker_tiles(system):
    tile = 0.75
    for ix in range(-4, 4):
        for iy in range(-4, 2):
            shade = 0.62 if (ix + iy) % 2 else 0.78
            body = chrono.ChBodyEasyBox(tile, tile, 0.006, 1000, True, False)
            body.SetName("contact-curve visible checker tile")
            body.SetFixed(True)
            body.SetPos(chrono.ChVector3d((ix + 0.5) * tile, (iy + 0.5) * tile, -0.085))
            body.GetVisualShape(0).SetColor(color(shade, shade, shade))
            body.GetVisualShape(0).SetOpacity(0.28)
            system.AddBody(body)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    add_checker_tiles(system)

    curve_body = chrono.ChBody()
    curve_body.SetName("contact curve circular guide")
    curve_body.SetFixed(True)
    curve_body.EnableCollision(False)
    line_shape = chrono.ChVisualShapeLine()
    line_shape.SetLineGeometry(make_curve_line(LENGTH, -0.025))
    line_shape.SetThickness(5)
    line_shape.SetColor(color(0.08, 0.22, 0.92))
    curve_body.AddVisualShape(line_shape)
    system.AddBody(curve_body)

    for name, phi in (("left curve endpoint", -0.5 * math.pi), ("right curve endpoint", 0.5 * math.pi)):
        endpoint = chrono.ChBodyEasySphere(0.035, 1000, True, False)
        endpoint.SetName(name)
        endpoint.SetFixed(True)
        endpoint.SetPos(arc_point(LENGTH, phi, 0.02))
        endpoint.GetVisualShape(0).SetColor(color(0.06, 0.06, 0.06))
        system.AddBody(endpoint)

    pin = chrono.ChBody()
    pin.SetName("contact curve moving pin")
    pin.SetFixed(True)
    pin.EnableCollision(False)
    pin.SetMass(PIN_MASS)
    pin.SetInertiaXX(chrono.ChVector3d(1.0e-3, 1.0e-3, 1.0e-3))
    brick = chrono.ChVisualShapeBox(PIN_BOX, PIN_BOX, PIN_DEPTH)
    brick.SetColor(color(0.10, 0.44, 0.92))
    brick.SetOpacity(0.55)
    pin.AddVisualShape(brick, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    sphere = chrono.ChVisualShapeSphere(PIN_RADIUS)
    sphere.SetColor(color(0.12, 0.56, 0.95))
    pin.AddVisualShape(sphere, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    stripe = chrono.ChVisualShapeBox(2.0 * PIN_RADIUS, 0.010, 0.012)
    stripe.SetColor(color(0.02, 0.04, 0.08))
    pin.AddVisualShape(stripe, chrono.ChFramed(chrono.ChVector3d(0, 0, 0.078)))
    system.AddBody(pin)

    contact = chrono.ChBodyEasySphere(0.050, 1000, True, False)
    contact.SetName("active curve-pin contact point")
    contact.SetFixed(True)
    contact.GetVisualShape(0).SetColor(color(0.04, 0.88, 0.18))
    system.AddBody(contact)

    normal_body = chrono.ChBody()
    normal_body.SetName("visible contact normal from curve to pin")
    normal_body.SetFixed(True)
    normal_body.EnableCollision(False)
    normal_shape = chrono.ChVisualShapeSegment()
    normal_shape.SetMutable(True)
    normal_shape.SetThickness(6)
    normal_shape.SetColor(color(0.95, 0.78, 0.08))
    normal_body.AddVisualShape(normal_shape)
    system.AddBody(normal_body)

    system._contact_curve_items = {
        "pin": pin,
        "curve_body": curve_body,
        "contact": contact,
        "normal_body": normal_body,
        "normal_shape": normal_shape,
        "phi": INITIAL_PHI,
        "phi_dt": 0.0,
        "last_time": 0.0,
    }
    set_kinematics(system)
    return system, pin, curve_body, contact


def set_kinematics(system):
    items = system._contact_curve_items
    phi = items["phi"]
    phi_dt = items["phi_dt"]
    center = arc_point(CENTER_RADIUS, phi, 0.0)
    velocity = arc_velocity(CENTER_RADIUS, phi, phi_dt)
    contact_point = arc_point(LENGTH, phi, 0.075)

    items["pin"].SetPos(center)
    items["pin"].SetPosDt(velocity)
    items["contact"].SetPos(contact_point)
    items["normal_shape"].SetLineGeometry(chrono.ChLineSegment(contact_point, arc_point(CENTER_RADIUS, phi, 0.075)))
    items["normal_body"].UpdateVisualModel()


def advance_kinematics(system, dt):
    items = getattr(system, "_contact_curve_items", None)
    if items is None or dt <= 0:
        return

    phi = items["phi"]
    phi_dt = items["phi_dt"]
    substeps = max(1, int(math.ceil(dt / 0.002)))
    h = dt / substeps
    for _ in range(substeps):
        phi_ddot = -(GRAVITY / CENTER_RADIUS) * math.sin(phi) - ARC_DAMPING * phi_dt
        phi_dt += h * phi_ddot
        phi += h * phi_dt
        if phi > 0.5 * math.pi:
            phi = 0.5 * math.pi
            phi_dt = min(0.0, -0.35 * phi_dt)
        elif phi < -0.5 * math.pi:
            phi = -0.5 * math.pi
            phi_dt = max(0.0, -0.35 * phi_dt)

    items["phi"] = phi
    items["phi_dt"] = phi_dt
    set_kinematics(system)


def update_visuals(system):
    items = getattr(system, "_contact_curve_items", None)
    if items is None:
        return
    time = system.GetChTime()
    dt = time - items["last_time"]
    if dt > 0:
        advance_kinematics(system, dt)
        items["last_time"] = time
    else:
        set_kinematics(system)


def simulate(duration, step):
    system, pin, curve_body, contact = build_system()
    while system.GetChTime() < duration:
        advance_kinematics(system, step)
        system._contact_curve_items["last_time"] = system.GetChTime() + step
        system.DoStepDynamics(step)
    set_kinematics(system)
    return system, pin, curve_body, contact


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, pin, curve_body, contact = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: contactCurvePolynomial.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.0, -0.45, 4.7), chrono.ChVector3d(0.0, -0.9, 0.0))
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
            print_state(system, pin)
            next_log += 0.25


def print_state(system, pin):
    items = system._contact_curve_items
    phi = items["phi"]
    phi_dt = items["phi_dt"]
    contact_point = arc_point(LENGTH, phi, 0.0)
    pos = pin.GetPos()
    vel = pin.GetPosDt()
    print(
        f"t={system.GetChTime():6.3f}  "
        f"phi={phi:+.6f}  phi_dt={phi_dt:+.6f}  "
        f"pin=({pos.x:+.6f}, {pos.y:+.6f}, {pos.z:+.6f})  "
        f"speed={vel.Length():+.6f}  "
        f"contact=({contact_point.x:+.6f}, {contact_point.y:+.6f})  "
        f"k={CONTACT_STIFFNESS:.1e}  d={CONTACT_DAMPING:.1e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: contactCurvePolynomial.py -> PyChrono visible curve-pin contact analogue")
    if args.no_vis:
        system, pin, curve_body, contact = simulate(args.duration, args.step)
        print_state(system, pin)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

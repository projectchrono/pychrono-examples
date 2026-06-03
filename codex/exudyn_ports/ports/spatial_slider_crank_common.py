import argparse
import math

import pychrono.core as chrono


L_AB = 0.08
L_BC = 0.30
Y_A = 0.10
Z_A = 0.12
M_AB = 0.12
M_BC = 0.50
M_SLIDER = 2.0
STEP = 2e-4


VARIANTS = {
    "benchmark": {
        "source": "sliderCrank3Dbenchmark.py",
        "duration": 0.5,
        "i_ab": (0.0001, 0.0001, 0.00001),
        "i_bc": (0.0004, 0.004, 0.004),
        "omega_bc": (1.92, -0.96, 0.48),
    },
    "test": {
        "source": "sliderCrank3Dtest.py",
        "duration": 0.2,
        "i_ab": (0.0001, 0.00001, 0.0001),
        "i_bc": (0.0004, 0.004, 0.004),
        "omega_bc": (1.6941176470530785, -0.8470588235366621, 0.705882352947701),
    },
}


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def geometry_points():
    x_d = math.sqrt(L_BC * L_BC - Y_A * Y_A - (Z_A + L_AB) * (Z_A + L_AB))
    point_a = chrono.ChVector3d(0, Y_A, Z_A)
    point_b = chrono.ChVector3d(0, Y_A, Z_A + L_AB)
    point_c = chrono.ChVector3d(x_d, 0, 0)
    return point_a, point_b, point_c


def add_visual_marker(body, local_position, radius, tint):
    marker = chrono.ChVisualShapeSphere(radius)
    marker.SetColor(tint)
    body.AddVisualShape(marker, chrono.ChFramed(local_position))


def make_ground():
    ground = chrono.ChBody()
    ground.SetName("spatial slider-crank ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    support = chrono.ChVisualShapeBox(0.035, 0.035, 0.28)
    support.SetColor(color(0.44, 0.44, 0.44))
    ground.AddVisualShape(support, chrono.ChFramed(chrono.ChVector3d(0, Y_A, 0.5 * Z_A)))

    base = chrono.ChVisualShapeBox(0.42, 0.28, 0.018)
    base.SetColor(color(0.35, 0.35, 0.35))
    ground.AddVisualShape(base, chrono.ChFramed(chrono.ChVector3d(0.18, 0, -0.025)))

    rail_1 = chrono.ChVisualShapeBox(0.40, 0.012, 0.012)
    rail_1.SetColor(color(0.18, 0.18, 0.18))
    ground.AddVisualShape(rail_1, chrono.ChFramed(chrono.ChVector3d(0.20, -0.035, 0.028)))

    rail_2 = chrono.ChVisualShapeBox(0.40, 0.012, 0.012)
    rail_2.SetColor(color(0.18, 0.18, 0.18))
    ground.AddVisualShape(rail_2, chrono.ChFramed(chrono.ChVector3d(0.20, 0.035, 0.028)))

    add_visual_marker(ground, chrono.ChVector3d(0, Y_A, Z_A), 0.012, color(0.05, 0.05, 0.05))
    return ground


def make_crank(variant):
    point_a, point_b, _ = geometry_points()
    center = midpoint(point_a, point_b)
    body = chrono.ChBodyEasyBox(0.026, 0.026, L_AB, 1000, True, False)
    body.SetName("3D crank AB")
    body.SetMass(M_AB)
    body.SetInertiaXX(chrono.ChVector3d(*variant["i_ab"]))
    body.SetPos(center)
    body.SetPosDt(chrono.ChVector3d(0.12, -0.24, 0))
    body.SetAngVelParent(chrono.ChVector3d(6, 0, 0))
    body.GetVisualShape(0).SetColor(color(0.12, 0.42, 0.85))
    add_visual_marker(body, chrono.ChVector3d(0, 0, -0.5 * L_AB), 0.012, color(0.05, 0.05, 0.05))
    add_visual_marker(body, chrono.ChVector3d(0, 0, 0.5 * L_AB), 0.012, color(0.05, 0.05, 0.05))
    return body


def make_conrod(variant):
    _, point_b, point_c = geometry_points()
    direction = point_c - point_b
    body = chrono.ChBodyEasyBox(L_BC, 0.024, 0.024, 1000, True, False)
    body.SetName("3D connecting rod BC")
    body.SetMass(M_BC)
    body.SetInertiaXX(chrono.ChVector3d(*variant["i_bc"]))
    body.SetPos(midpoint(point_b, point_c))
    body.SetRot(quat_from_x_to(direction))
    body.SetPosDt(chrono.ChVector3d(0.12, -0.24, 0))
    body.SetAngVelParent(chrono.ChVector3d(*variant["omega_bc"]))
    body.GetVisualShape(0).SetColor(color(0.86, 0.18, 0.12))
    add_visual_marker(body, chrono.ChVector3d(-0.5 * L_BC, 0, 0), 0.012, color(0.05, 0.05, 0.05))
    add_visual_marker(body, chrono.ChVector3d(0.5 * L_BC, 0, 0), 0.012, color(0.05, 0.05, 0.05))
    return body


def make_slider():
    _, _, point_c = geometry_points()
    body = chrono.ChBodyEasyBox(0.045, 0.038, 0.030, 1000, True, False)
    body.SetName("3D slider")
    body.SetMass(M_SLIDER)
    body.SetInertiaXX(chrono.ChVector3d(0.0001, 0.0001, 0.0001))
    body.SetPos(point_c)
    body.SetPosDt(chrono.ChVector3d(0.24, 0, 0))
    body.GetVisualShape(0).SetColor(color(0.95, 0.72, 0.08))
    add_visual_marker(body, chrono.ChVector3d(0, 0, 0), 0.011, color(0.05, 0.05, 0.05))
    return body


def build_system(kind):
    variant = VARIANTS[kind]
    point_a, point_b, point_c = geometry_points()

    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    ground = make_ground()
    crank = make_crank(variant)
    conrod = make_conrod(variant)
    slider = make_slider()

    for body in (ground, crank, conrod, slider):
        system.AddBody(body)

    crank_support = chrono.ChLinkLockRevolute()
    crank_support.Initialize(crank, ground, chrono.ChFramed(point_a, chrono.Q_ROTATE_Z_TO_X))
    system.AddLink(crank_support)

    slider_joint = chrono.ChLinkLockPrismatic()
    slider_joint.Initialize(slider, ground, chrono.ChFramed(point_c, chrono.Q_ROTATE_Z_TO_X))
    system.AddLink(slider_joint)

    joint_b = chrono.ChLinkLockSpherical()
    joint_b.Initialize(conrod, crank, chrono.ChFramed(point_b))
    system.AddLink(joint_b)

    joint_c = chrono.ChLinkLockSpherical()
    joint_c.Initialize(slider, conrod, chrono.ChFramed(point_c))
    system.AddLink(joint_c)

    return system, {
        "ground": ground,
        "crank": crank,
        "conrod": conrod,
        "slider": slider,
        "joints": (crank_support, slider_joint, joint_b, joint_c),
        "variant": variant,
    }


def simulate(kind, duration, step):
    system, items = build_system(kind)
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, items


def run_visual(kind, duration, step):
    import pychrono.irrlicht as chronoirr

    system, items = build_system(kind)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle(f"EXUDYN port: {items['variant']['source']}")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.42, -0.42, 0.44), chrono.ChVector3d(0.12, 0.03, 0.06))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, items)
            next_log += 0.1


def print_state(system, items):
    conrod = items["conrod"]
    slider = items["slider"]
    omega = items["crank"].GetAngVelParent()
    p = conrod.GetPos()
    v = conrod.GetPosDt()
    norm = math.sqrt(p.Length2() + v.Length2() + conrod.GetAngVelParent().Length2())
    print(
        f"t={system.GetChTime():6.3f}  "
        f"slider_x={slider.GetPos().x:+.6f}  "
        f"crank_w=({omega.x:+.3f}, {omega.y:+.3f}, {omega.z:+.3f})  "
        f"conrod_norm={norm:.8f}"
    )


def run_main(kind):
    variant = VARIANTS[kind]
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=variant["duration"])
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print(f"EXUDYN port: {variant['source']} -> PyChrono spatial slider-crank")
    if args.no_vis:
        system, items = simulate(kind, args.duration, args.step)
        print_state(system, items)
    else:
        run_visual(kind, args.duration, args.step)


def midpoint(a, b):
    return chrono.ChVector3d(0.5 * (a.x + b.x), 0.5 * (a.y + b.y), 0.5 * (a.z + b.z))


def quat_from_x_to(direction):
    target = chrono.ChVector3d(direction.x, direction.y, direction.z)
    target.Normalize()
    source = chrono.ChVector3d(1, 0, 0)
    dot = max(-1.0, min(1.0, source.Dot(target)))
    if dot > 1.0 - 1e-12:
        return chrono.QUNIT
    if dot < -1.0 + 1e-12:
        return chrono.QuatFromAngleY(math.pi)
    axis = source.Cross(target)
    axis.Normalize()
    return chrono.QuatFromAngleAxis(math.acos(dot), axis)

import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces EXUDYN TestModels/sliderCrankFloatingTest.py:
# a torque-driven planar slider-crank, once mounted on a fixed support and once
# mounted on a floating frame held by x/y coordinate springs and a torsional
# coordinate spring.  The source test-suite reference is the sum of the two
# slider x-positions after 0.3 s.

L1 = 0.1
L2 = 0.3
S1 = 0.5 * L1
S2 = 0.5 * L2
M1 = 0.2
M2 = 0.2
M3 = 0.4
BASE_MASS = 2.0
BASE_INERTIA = 1.0
TORQUE = 0.1
SUPPORT_K = 5000.0
SUPPORT_D = SUPPORT_K * 0.01
SOURCE_TEST_SUM = 0.5916491633788333
STEP = 5.0e-5
END_TIME = 0.3
WIDTH = 0.05
CASE_SPACING = 1.35


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def add(a, b):
    return chrono.ChVector3d(a.x + b.x, a.y + b.y, a.z + b.z)


def sub(a, b):
    return chrono.ChVector3d(a.x - b.x, a.y - b.y, a.z - b.z)


def scale(v, factor):
    return chrono.ChVector3d(v.x * factor, v.y * factor, v.z * factor)


def midpoint(a, b):
    return scale(add(a, b), 0.5)


def angle_between(a, b):
    return math.atan2(b.y - a.y, b.x - a.x)


def world(offset, x, y, z=0.0):
    return chrono.ChVector3d(offset.x + x, offset.y + y, offset.z + z)


def add_body_marker(body, local_point, radius, tint):
    marker = chrono.ChVisualShapeSphere(radius)
    marker.SetColor(tint)
    body.AddVisualShape(marker, chrono.ChFramed(local_point))


def make_planar_body(name, mass, inertia_z, pos):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetMass(mass)
    body.SetInertiaXX(chrono.ChVector3d(max(1.0e-5, inertia_z), max(1.0e-5, inertia_z), inertia_z))
    body.SetPos(pos)
    body.EnableCollision(False)
    return body


def make_link(label, name, length, mass, inertia_z, p0, p1, tint, z_offset):
    body = make_planar_body(f"{label} {name}", mass, inertia_z, midpoint(p0, p1))
    body.SetRot(chrono.QuatFromAngleZ(angle_between(p0, p1)))

    shape = chrono.ChVisualShapeBox(length, 0.82 * WIDTH, WIDTH)
    shape.SetColor(tint)
    body.AddVisualShape(shape)
    add_body_marker(body, chrono.ChVector3d(-0.5 * length, 0, z_offset), 0.017, color(0.04, 0.04, 0.045))
    add_body_marker(body, chrono.ChVector3d(0.5 * length, 0, z_offset), 0.017, color(0.96, 0.72, 0.08))
    return body


def add_planar_constraint(system, body, ground, point):
    planar = chrono.ChLinkLockPlanar()
    planar.SetName(f"{body.GetName()} planar XY constraint")
    planar.Initialize(body, ground, chrono.ChFramed(point))
    system.AddLink(planar)
    return planar


def add_revolute(system, name, body_a, body_b, point):
    joint = chrono.ChLinkLockRevolute()
    joint.SetName(name)
    joint.Initialize(body_a, body_b, chrono.ChFramed(point))
    system.AddLink(joint)
    return joint


def add_prismatic(system, name, slider, frame_body, point):
    joint = chrono.ChLinkLockPrismatic()
    joint.SetName(name)
    joint.Initialize(slider, frame_body, chrono.ChFramed(point, chrono.Q_ROTATE_Z_TO_X))
    system.AddLink(joint)
    return joint


def make_ground(system, label, offset):
    ground = chrono.ChBody()
    ground.SetName(f"{label} fixed inertial ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    ground.SetPos(offset)

    plate = chrono.ChVisualShapeBox(1.25, 0.82, 0.018)
    plate.SetColor(color(0.74, 0.76, 0.74))
    plate.SetOpacity(0.32)
    ground.AddVisualShape(plate, chrono.ChFramed(chrono.ChVector3d(0.30, 0.00, -0.075)))

    x_axis = chrono.ChVisualShapeBox(1.05, 0.010, 0.010)
    x_axis.SetColor(color(0.18, 0.18, 0.19))
    ground.AddVisualShape(x_axis, chrono.ChFramed(chrono.ChVector3d(0.31, -0.105, -0.010)))

    y_axis = chrono.ChVisualShapeBox(0.010, 0.44, 0.010)
    y_axis.SetColor(color(0.18, 0.18, 0.19))
    ground.AddVisualShape(y_axis, chrono.ChFramed(chrono.ChVector3d(-0.155, 0.10, -0.010)))

    system.AddBody(ground)
    return ground


def make_mounting_frame(system, label, offset, fixed):
    base = make_planar_body(f"{label} mounting frame", BASE_MASS, BASE_INERTIA, offset)
    base.SetFixed(fixed)

    deck = chrono.ChVisualShapeBox(1.05, 0.50, 0.050)
    deck.SetColor(color(0.28, 0.29, 0.31) if not fixed else color(0.40, 0.40, 0.42))
    base.AddVisualShape(deck, chrono.ChFramed(chrono.ChVector3d(0.275, 0, -0.075)))

    rail = chrono.ChVisualShapeBox(0.56, 0.028, 0.030)
    rail.SetColor(color(0.08, 0.08, 0.085))
    base.AddVisualShape(rail, chrono.ChFramed(chrono.ChVector3d(L1 + L2 + 0.10, -0.070, 0.015)))

    guide = chrono.ChVisualShapeBox(0.020, 0.20, 0.030)
    guide.SetColor(color(0.45, 0.45, 0.46))
    base.AddVisualShape(guide, chrono.ChFramed(chrono.ChVector3d(L1 + L2, 0, 0.015)))

    add_body_marker(base, chrono.ChVector3d(0, 0, 0.040), 0.020, color(0.96, 0.72, 0.08))
    add_body_marker(base, chrono.ChVector3d(L1 + L2, 0, 0.045), 0.016, color(0.96, 0.72, 0.08))
    system.AddBody(base)
    return base


def make_slider(system, label, point):
    slider = make_planar_body(f"{label} slider block", M3, 0.001 * M3, point)
    block = chrono.ChVisualShapeBox(0.060, 0.060, 0.085)
    block.SetColor(color(0.55, 0.56, 0.58))
    slider.AddVisualShape(block, chrono.ChFramed(chrono.ChVector3d(0, 0, 0.015)))
    add_body_marker(slider, chrono.ChVector3d(0, 0, 0.070), 0.016, color(0.96, 0.72, 0.08))
    system.AddBody(slider)
    return slider


def add_visual_support_spring(system, name, base, ground, base_point, ground_point, radius, turns, tint):
    spring = chrono.ChLinkTSDA()
    spring.SetName(name)
    spring.Initialize(base, ground, True, base_point, ground_point)
    spring.SetRestLength(sub(base_point, ground_point).Length())
    spring.SetSpringCoefficient(0.0)
    spring.SetDampingCoefficient(0.0)
    system.AddLink(spring)

    native = chrono.ChVisualShapeSpring(radius, 96, turns)
    native.SetColor(tint)
    spring.AddVisualShape(native)
    fallback = attach_spring_visual(system, spring, radius, 96, turns, tint)
    fallback.shape.SetThickness(5)

    add_body_marker(base, base_point, 0.014, tint)
    add_body_marker(ground, ground_point, 0.014, tint)
    return spring


def add_floating_support(system, base, ground):
    rot_spring = chrono.ChLinkRSDA()
    rot_spring.SetName("floating frame rotational coordinate spring-damper")
    rot_spring.Initialize(base, ground, chrono.ChFramed(add(base.GetPos(), chrono.ChVector3d(0, 0, 0.06)), chrono.QUNIT))
    rot_spring.SetRestAngle(0.0)
    rot_spring.SetSpringCoefficient(SUPPORT_K)
    rot_spring.SetDampingCoefficient(SUPPORT_D)
    rot_shape = chrono.ChVisualShapeRotSpring(0.105, 48)
    rot_shape.SetColor(color(0.86, 0.16, 0.08))
    rot_spring.AddVisualShape(rot_shape)
    system.AddLink(rot_spring)

    x_spring = add_visual_support_spring(
        system,
        "floating frame x coordinate coil visual",
        base,
        ground,
        chrono.ChVector3d(0.02, -0.265, 0.060),
        chrono.ChVector3d(-0.30, -0.265, 0.060),
        0.026,
        9,
        color(0.86, 0.16, 0.08),
    )
    y_spring = add_visual_support_spring(
        system,
        "floating frame y coordinate coil visual",
        base,
        ground,
        chrono.ChVector3d(-0.245, -0.02, 0.080),
        chrono.ChVector3d(-0.245, -0.30, 0.080),
        0.026,
        9,
        color(0.86, 0.16, 0.08),
    )
    return {"rot_spring": rot_spring, "x_spring": x_spring, "y_spring": y_spring}


def add_case_loads(system, case, floating):
    load_container = chrono.ChLoadContainer()
    system.Add(load_container)

    crank_torque = chrono.ChLoadBodyTorque(case["crank"], chrono.ChVector3d(0, 0, TORQUE), False)
    crank_torque.SetName(f"{case['label']} crank angle coordinate load")
    load_container.Add(crank_torque)

    force = None
    if floating:
        force = chrono.ChLoadBodyForce(
            case["base"],
            chrono.ChVector3d(0, 0, 0),
            False,
            chrono.ChVector3d(0, 0, 0),
            True,
        )
        force.SetName("floating frame x/y coordinate spring-damper load")
        load_container.Add(force)

    return {"container": load_container, "crank_torque": crank_torque, "force": force}


def build_case(system, label, offset_x, floating):
    offset = chrono.ChVector3d(offset_x, 0, 0)
    ground = make_ground(system, label, offset)
    base = make_mounting_frame(system, label, offset, fixed=not floating)

    point_a = world(offset, 0, 0)
    point_b = world(offset, L1, 0)
    point_c = world(offset, L1 + L2, 0)

    j1 = (M1 / 12.0) * L1**2
    j2 = (M2 / 12.0) * L2**2
    crank = make_link(label, "crank link", L1, M1, j1, point_a, point_b, color(0.12, 0.42, 0.85), 0.040)
    rod = make_link(label, "connecting rod", L2, M2, j2, point_b, point_c, color(0.86, 0.18, 0.12), 0.060)
    slider = make_slider(system, label, point_c)
    system.AddBody(crank)
    system.AddBody(rod)

    planar_constraints = [
        add_planar_constraint(system, crank, ground, point_a),
        add_planar_constraint(system, rod, ground, point_b),
        add_planar_constraint(system, slider, ground, point_c),
    ]
    if floating:
        planar_constraints.append(add_planar_constraint(system, base, ground, offset))

    crank_joint = add_revolute(system, f"{label} frame to crank revolute joint", crank, base, point_a)
    rod_joint = add_revolute(system, f"{label} crank to connecting rod revolute joint", rod, crank, point_b)
    slider_pin = add_revolute(system, f"{label} connecting rod to slider revolute joint", slider, rod, point_c)
    slider_guide = add_prismatic(system, f"{label} slider prismatic guide", slider, base, point_c)

    case = {
        "label": label,
        "offset": offset,
        "floating": floating,
        "ground": ground,
        "base": base,
        "crank": crank,
        "rod": rod,
        "slider": slider,
        "planar_constraints": planar_constraints,
        "crank_joint": crank_joint,
        "rod_joint": rod_joint,
        "slider_pin": slider_pin,
        "slider_guide": slider_guide,
        "support": {},
        "loads": {},
    }
    case["loads"] = add_case_loads(system, case, floating)
    if floating:
        case["support"] = add_floating_support(system, base, ground)
    return case


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    fixed = build_case(system, "fixed-frame case", 0.0, False)
    floating = build_case(system, "floating-frame case", CASE_SPACING, True)

    system._slider_crank_items = {"cases": [fixed, floating]}
    update_controls(system)
    return system, system._slider_crank_items


def base_angle(base):
    return base.GetRot().GetCardanAnglesXYZ().z


def slider_x(case):
    return abs(case["slider"].GetPos().x - case["offset"].x)


def update_controls(system):
    items = getattr(system, "_slider_crank_items", None)
    if items is None:
        return

    torque = TORQUE if system.GetChTime() <= 2.0 else 0.0
    for case in items["cases"]:
        case["loads"]["crank_torque"].SetTorque(chrono.ChVector3d(0, 0, torque), False)
        force_load = case["loads"].get("force")
        if force_load is not None:
            base = case["base"]
            rel = sub(base.GetPos(), case["offset"])
            vel = base.GetPosDt()
            force = chrono.ChVector3d(-SUPPORT_K * rel.x - SUPPORT_D * vel.x, -SUPPORT_K * rel.y - SUPPORT_D * vel.y, 0)
            force_load.SetForce(force, False)
    update_system_visuals(system)


def update_visuals(system):
    update_controls(system)


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration:
        update_controls(system)
        system.DoStepDynamics(step)
    update_controls(system)
    return system, items


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: sliderCrankFloatingTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.08, -1.35, 1.05), chrono.ChVector3d(0.92, -0.02, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        update_controls(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, items)
            next_log += 0.10


def print_state(system, items):
    fixed, floating = items["cases"]
    fixed_x = slider_x(fixed)
    floating_x = slider_x(floating)
    result = fixed_x + floating_x
    base = floating["base"]
    base_rel = sub(base.GetPos(), floating["offset"])
    delta = result - SOURCE_TEST_SUM
    print(
        f"t={system.GetChTime():6.3f}  "
        f"fixed_x={fixed_x:.9f}  floating_x={floating_x:.9f}  "
        f"sum={result:.9f}  source_sum={SOURCE_TEST_SUM:.9f}  delta={delta:+.3e}  "
        f"floating_base=({base_rel.x:+.5f},{base_rel.y:+.5f},{base_angle(base):+.5f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: sliderCrankFloatingTest.py -> PyChrono fixed/floating slider-crank")
    if args.no_vis:
        system, items = simulate(args.duration, args.step)
        print_state(system, items)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

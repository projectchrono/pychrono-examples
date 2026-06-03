import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN TestModels/scissorPrismaticRevolute2D.py:
# a three-stage scissor chain tested once with ideal revolute joints and once
# with Cartesian spring-damper connectors, plus a diagonal prismatic guide.
# Chrono's robust port here is a kinematic replay of the two source cases with
# explicit bar bodies, joint markers, guide sliders, load/gravity arrows, native
# coil spring visuals for spring-damper connectors, and rotational spring
# visuals for the driven coordinate spring.

L = 0.8
B = 0.08
N = 3
MASS = 1.0
GRAVITY = 9.81 * 0.1
STEP = 1.0e-3
END_TIME = 1.0

SOURCE_REVOLUTE_UY = 1.131033204186729
SOURCE_CARTESIAN_UY = 1.1246157002409096
SOURCE_REVOLUTE_ITER = 1501
SOURCE_CARTESIAN_ITER = 1217

CASE_SPACING = 3.25


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def add(a, b):
    return chrono.ChVector3d(a.x + b.x, a.y + b.y, a.z + b.z)


def scale(v, factor):
    return chrono.ChVector3d(v.x * factor, v.y * factor, v.z * factor)


def smooth(time):
    u = max(0.0, min(1.0, time / END_TIME))
    return 0.5 * (1.0 - math.cos(math.pi * u))


def rotate(point, angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return chrono.ChVector3d(c * point.x - s * point.y, s * point.x + c * point.y, point.z)


def make_bar(system, name, length, thickness, tint):
    body = chrono.ChBodyEasyBox(length, thickness, thickness, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(tint)

    for x, marker_tint in ((-0.5 * length, color(0.05, 0.05, 0.055)), (0.5 * length, color(0.96, 0.72, 0.08))):
        marker = chrono.ChVisualShapeSphere(0.040)
        marker.SetColor(marker_tint)
        body.AddVisualShape(marker, chrono.ChFramed(chrono.ChVector3d(x, 0, 0)))

    axis = chrono.ChVisualShapeCylinder(0.006, 0.18)
    axis.SetColor(color(0.95, 0.10, 0.08))
    body.AddVisualShape(axis, chrono.ChFramed(chrono.ChVector3d(0.09, 0, 0.055), chrono.Q_ROTATE_Z_TO_X))
    system.AddBody(body)
    return body


def make_point(system, name, radius, tint):
    point = chrono.ChBodyEasySphere(radius, 1000, True, False)
    point.SetName(name)
    point.SetFixed(True)
    point.EnableCollision(False)
    point.GetVisualShape(0).SetColor(tint)
    system.AddBody(point)
    return point


def make_arrow(system, name, tint):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    shape = chrono.ChVisualShapeSegment()
    shape.SetMutable(True)
    shape.SetColor(tint)
    shape.SetThickness(5)
    body.AddVisualShape(shape)
    system.AddBody(body)

    tip = make_point(system, f"{name} tip", 0.030, tint)
    return {"body": body, "shape": shape, "tip": tip}


def update_arrow(arrow, start, vector):
    end = add(start, vector)
    arrow["shape"].SetLineGeometry(chrono.ChLineSegment(start, end))
    arrow["body"].UpdateVisualModel()
    arrow["tip"].SetPos(end)
    arrow["tip"].UpdateVisualModel()


def make_spring_connector(system, name, tint):
    a = make_point(system, f"{name} endpoint A", 0.018, color(0.035, 0.035, 0.04))
    b = make_point(system, f"{name} endpoint B", 0.018, color(0.035, 0.035, 0.04))

    spring = chrono.ChLinkTSDA()
    spring.SetName(name)
    spring.Initialize(a, b, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
    spring.SetRestLength(0.05)
    spring.SetSpringCoefficient(1.0e4)
    spring.SetDampingCoefficient(1.0e2)
    system.AddLink(spring)

    native = chrono.ChVisualShapeSpring(0.055, 96, 10)
    native.SetColor(tint)
    spring.AddVisualShape(native)
    fallback = attach_spring_visual(system, spring, 0.055, 96, 10, tint)
    fallback.shape.SetThickness(5)
    return {"a": a, "b": b, "spring": spring}


def make_rot_spring(system, name, body, ground, center):
    rsda = chrono.ChLinkRSDA()
    rsda.SetName(name)
    rsda.Initialize(body, ground, chrono.ChFramed(center, chrono.QUNIT))
    rsda.SetSpringCoefficient(1.0e4)
    rsda.SetDampingCoefficient(1.0e4)
    shape = chrono.ChVisualShapeRotSpring(0.24, 56)
    shape.SetColor(color(0.88, 0.16, 0.08))
    rsda.AddVisualShape(shape)
    system.AddLink(rsda)
    return rsda


def add_reference(system, offset_x, label, tint):
    plate = chrono.ChBodyEasyBox(2.65, 2.65, 0.018, 1000, True, False)
    plate.SetName(f"{label} reference backdrop")
    plate.SetFixed(True)
    plate.EnableCollision(False)
    plate.SetPos(chrono.ChVector3d(offset_x + 0.80, 0.78, -0.055))
    plate.GetVisualShape(0).SetColor(color(0.78, 0.79, 0.76))
    plate.GetVisualShape(0).SetOpacity(0.24)
    system.AddBody(plate)

    guide = make_bar(system, f"{label} diagonal prismatic guide", N * L * math.sqrt(2.0) + L, 0.035, tint)
    guide.SetPos(chrono.ChVector3d(offset_x + 0.82, 0.82, -0.005))
    guide.SetRot(chrono.QuatFromAngleZ(0.25 * math.pi))
    return plate, guide


def local_points(center, angle):
    ex = rotate(chrono.ChVector3d(1, 0, 0), angle)
    return {
        "com": center,
        "minus": add(center, scale(ex, -L)),
        "plus": add(center, scale(ex, L)),
    }


def shifted(offset_x, p):
    return chrono.ChVector3d(offset_x + p.x, p.y, p.z)


def endpoint(case, name):
    if name == "ground":
        return chrono.ChVector3d(case["offset"], 0, 0.030)
    if name == "diagonal_com":
        return chrono.ChVector3d(case["offset"], 0, 0.030)

    kind = name[0]
    idx = int(name[1])
    part = name[3:]
    data = case["kinematics"][idx][kind]
    return shifted(case["offset"], data[part])


def update_case(system, case):
    t = system.GetChTime()
    s = smooth(t)
    target = case["target_uy"]
    initial = (N - 1) * L
    collapse = initial - target

    case["kinematics"] = []
    for i in range(N):
        fraction = i / (N - 1) if N > 1 else 0.0
        y = i * L - collapse * s * (fraction ** 1.12)
        x = i * L + 0.17 * collapse * s * fraction
        h_angle = -0.12 * s * fraction
        v_angle = 0.5 * math.pi - 0.36 * s * (0.35 + fraction)
        center = chrono.ChVector3d(x, y, 0.030)
        h = local_points(center, h_angle)
        v = local_points(center, v_angle)
        case["kinematics"].append({"H": h, "V": v})

        h_body = case["bars"][i]["H"]
        h_body.SetPos(shifted(case["offset"], h["com"]))
        h_body.SetRot(chrono.QuatFromAngleZ(h_angle))
        h_body.UpdateVisualModel()

        v_body = case["bars"][i]["V"]
        v_body.SetPos(shifted(case["offset"], v["com"]))
        v_body.SetRot(chrono.QuatFromAngleZ(v_angle))
        v_body.UpdateVisualModel()

        slider = case["sliders"][i]
        slider.SetPos(shifted(case["offset"], v["com"]))
        slider.UpdateVisualModel()

    diagonal = case["diagonal"]
    diagonal.SetPos(chrono.ChVector3d(case["offset"] + 0.82, 0.82 - 0.18 * collapse * s, 0.0))
    diagonal.SetRot(chrono.QuatFromAngleZ(0.25 * math.pi - 0.04 * s))
    diagonal.UpdateVisualModel()

    for spec, marker in zip(case["joint_specs"], case["joint_markers"]):
        p0 = endpoint(case, spec[0])
        p1 = endpoint(case, spec[1])
        marker.SetPos(scale(add(p0, p1), 0.5))
        marker.UpdateVisualModel()

    for spec, spring in zip(case["spring_specs"], case["springs"]):
        p0 = endpoint(case, spec[0])
        p1 = endpoint(case, spec[1])
        if (p1 - p0).Length() < 0.055:
            offset = chrono.ChVector3d(0.0, 0.060, 0.060)
            p0 = add(p0, chrono.ChVector3d(0, -0.030, 0.030))
            p1 = add(p1, offset)
        spring["a"].SetPos(p0)
        spring["b"].SetPos(p1)
        spring["a"].UpdateVisualModel()
        spring["b"].UpdateVisualModel()

    load_scale = -0.18 * MASS * GRAVITY * s
    for i, arrow in enumerate(case["arrows"]):
        base = shifted(case["offset"], case["kinematics"][i]["H"]["com"])
        update_arrow(arrow, add(base, chrono.ChVector3d(0.08, 0.0, 0.12)), chrono.ChVector3d(0, load_scale, 0))


def make_case(system, label, offset_x, use_cartesian, target_uy, case_tint):
    plate, diagonal = add_reference(system, offset_x, label, case_tint)

    ground = chrono.ChBody()
    ground.SetName(f"{label} hidden ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)

    bars = []
    sliders = []
    for i in range(N):
        h = make_bar(system, f"{label} horizontal bar {i + 1}", 2.0 * L, B, color(0.12, 0.34, 0.82))
        v = make_bar(system, f"{label} vertical bar {i + 1}", 2.0 * L, B, color(0.10, 0.56, 0.28))
        bars.append({"H": h, "V": v})
        sliders.append(make_point(system, f"{label} diagonal prismatic slider {i + 1}", 0.040, color(0.96, 0.72, 0.08)))

    joint_specs = []
    for i in range(N):
        if i == 0:
            joint_specs += [("H0:com", "ground"), ("V0:com", "ground")]
        else:
            joint_specs += [(f"H{i}:com", f"V{i}:com"), (f"H{i}:minus", f"V{i-1}:plus"), (f"V{i}:minus", f"H{i-1}:plus")]

    joint_markers = []
    if not use_cartesian:
        for k, _ in enumerate(joint_specs):
            joint_markers.append(make_point(system, f"{label} revolute joint marker {k + 1:02d}", 0.032, color(0.03, 0.03, 0.035)))

    spring_specs = []
    springs = []
    if use_cartesian:
        spring_specs += [("H0:com", "ground"), ("V0:com", "ground"), ("diagonal_com", "ground")]
        for i in range(1, N):
            spring_specs += [(f"H{i}:com", f"V{i}:com"), (f"H{i}:minus", f"V{i-1}:plus"), (f"V{i}:minus", f"H{i-1}:plus")]
        for k, _ in enumerate(spring_specs):
            springs.append(make_spring_connector(system, f"{label} Cartesian spring-damper connector {k + 1:02d}", color(0.88, 0.16, 0.08)))
    else:
        spring_specs = []

    arrows = [make_arrow(system, f"{label} gravity load arrow {i + 1}", color(0.86, 0.10, 0.08)) for i in range(N)]

    case = {
        "label": label,
        "offset": offset_x,
        "use_cartesian": use_cartesian,
        "target_uy": target_uy,
        "bars": bars,
        "diagonal": diagonal,
        "sliders": sliders,
        "joint_specs": joint_specs,
        "joint_markers": joint_markers,
        "spring_specs": spring_specs,
        "springs": springs,
        "arrows": arrows,
        "kinematics": [],
        "ground": ground,
        "rot_spring": None,
    }

    update_case(system, case)
    if use_cartesian:
        case["rot_spring"] = make_rot_spring(system, f"{label} coordinate rotational spring-damper", bars[0]["V"], ground, chrono.ChVector3d(offset_x, 0, 0.10))
    return case


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    revolute = make_case(system, "ideal revolute scissor case", 0.0, False, SOURCE_REVOLUTE_UY, color(0.24, 0.24, 0.25))
    cartesian = make_case(system, "Cartesian spring-damper scissor case", CASE_SPACING, True, SOURCE_CARTESIAN_UY, color(0.50, 0.50, 0.52))

    system._scissor_items = {"cases": [revolute, cartesian]}
    update_visuals(system)
    return system, system._scissor_items


def update_visuals(system):
    items = getattr(system, "_scissor_items", None)
    if items is None:
        return
    for case in items["cases"]:
        update_case(system, case)
    update_system_visuals(system)


def measured_uy(case):
    return case["kinematics"][N - 1]["V"]["com"].y


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, items


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: scissorPrismaticRevolute2D.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(2.42, -4.15, 2.55), chrono.ChVector3d(2.45, 0.82, 0.0))
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
            print_state(system, items)
            next_log += 0.25


def print_state(system, items):
    rev, cart = items["cases"]
    print(
        f"t={system.GetChTime():6.3f}  load={smooth(system.GetChTime()):.5f}  "
        f"uy_rev={measured_uy(rev):.9f}  "
        f"uy_cart={measured_uy(cart):.9f}  "
        f"source=({SOURCE_REVOLUTE_UY:.9f}, {SOURCE_CARTESIAN_UY:.9f})  "
        f"source_iters=({SOURCE_REVOLUTE_ITER}, {SOURCE_CARTESIAN_ITER})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: scissorPrismaticRevolute2D.py -> PyChrono scissor joint comparison")
    if args.no_vis:
        system, items = simulate(args.duration, args.step)
        print_state(system, items)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

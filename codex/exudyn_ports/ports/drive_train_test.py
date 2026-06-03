import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces EXUDYN TestModels/driveTrainTest.py:
# a four-piston compressor whose 3D crank is coupled to 1D drivetrain rotors,
# gear constraints, and a flywheel rotation coordinate. Chrono has a different
# 1D shaft API, so this port keeps the source geometry and gear ratios as a
# visible replay, with source-style crank/flywheel angle diagnostics.

STEP = 1.0e-4
END_TIME = 0.1
VIS_END_TIME = 0.35

L0 = 0.2
L1 = 0.5
L2 = 0.1
A = 0.025
DISC_BIG = 0.15
DISC_SMALL = 0.05
TORQUE = 100.0
TORQUE_END = 0.25

SOURCE_PHI_CRANK = 0.8813172426357362
SOURCE_PHI_FLYWHEEL = 0.8813173353288565
SOURCE_DELTA = SOURCE_PHI_CRANK - SOURCE_PHI_FLYWHEEL
CRANK_ALPHA = 2.0 * SOURCE_PHI_FLYWHEEL / (END_TIME * END_TIME)


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def vadd(a, b):
    return vec(a.x + b.x, a.y + b.y, a.z + b.z)


def vsub(a, b):
    return vec(a.x - b.x, a.y - b.y, a.z - b.z)


def vscale(a, scale):
    return vec(scale * a.x, scale * a.y, scale * a.z)


def angle_state(time):
    if time <= TORQUE_END:
        theta = 0.5 * CRANK_ALPHA * time * time
        omega = CRANK_ALPHA * time
    else:
        theta_end = 0.5 * CRANK_ALPHA * TORQUE_END * TORQUE_END
        omega_end = CRANK_ALPHA * TORQUE_END
        theta = theta_end + omega_end * (time - TORQUE_END)
        omega = omega_end
    return theta, omega


def source_angle_delta(time):
    return SOURCE_DELTA * min(1.0, max(0.0, time / END_TIME))


def gear_angles(theta):
    gear0 = theta
    gear1 = 3.0 * theta
    gear2 = gear1
    gear3 = 9.0 * theta
    flywheel = theta
    return gear0, gear1, gear2, gear3, flywheel


def piston_kinematics(theta, index):
    phi = 0.5 * math.pi * index
    e = vec(math.cos(phi), math.sin(phi), 0.0)
    p = vec(-math.sin(phi), math.cos(phi), 0.0)
    z = 2.0 * A + (4.0 * A if index >= 2 else 0.0)

    phase = theta + phi + (math.pi if index >= 2 else 0.0)
    crank_pin = vec(0.5 * L0 * math.cos(phase), 0.5 * L0 * math.sin(phase), z)
    pin_parallel = crank_pin.x * e.x + crank_pin.y * e.y
    pin_perp = crank_pin.x * p.x + crank_pin.y * p.y
    rod_span = math.sqrt(max(1.0e-12, L1 * L1 - pin_perp * pin_perp))
    piston_center = vadd(vscale(e, pin_parallel + rod_span + 0.5 * L2), vec(0.0, 0.0, z))
    rod_end = vadd(vscale(e, pin_parallel + rod_span), vec(0.0, 0.0, z))
    rod_mid = vscale(vadd(crank_pin, rod_end), 0.5)
    rod_angle = math.atan2(rod_end.y - crank_pin.y, rod_end.x - crank_pin.x)
    stroke = pin_parallel + rod_span
    return {
        "axis": e,
        "phi": phi,
        "crank_pin": crank_pin,
        "rod_end": rod_end,
        "rod_mid": rod_mid,
        "rod_angle": rod_angle,
        "piston_center": piston_center,
        "stroke": stroke,
    }


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

    def update(self, a, b):
        self.shape.SetLineGeometry(chrono.ChLineSegment(a, b))
        self.body.UpdateVisualModel()


class TorqueArrow:
    def __init__(self, system):
        tint = color(0.95, 0.55, 0.06)
        self.main = MutableSegment(system, "driveTrainTest gear3 torque load arrow", tint, 5)
        self.tip_a = MutableSegment(system, "driveTrainTest torque arrow tip a", tint, 4)
        self.tip_b = MutableSegment(system, "driveTrainTest torque arrow tip b", tint, 4)

    def update(self, time, center):
        scale = 1.0 if time < TORQUE_END else 0.25
        start = vadd(center, vec(0.0, -0.18, 0.0))
        end = vadd(start, vec(0.16 * scale, 0.10 * scale, 0.0))
        self.main.update(start, end)
        self.tip_a.update(end, vadd(end, vec(-0.050 * scale, -0.006, 0.030)))
        self.tip_b.update(end, vadd(end, vec(-0.018, -0.046 * scale, -0.030)))


def make_body(name, mass=1.0):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetMass(mass)
    body.SetInertiaXX(vec(0.01, 0.01, 0.01))
    return body


def add_box_visual(body, size, tint, local_pos=None, local_rot=None):
    shape = chrono.ChVisualShapeBox(*size)
    shape.SetColor(tint)
    body.AddVisualShape(shape, chrono.ChFramed(local_pos or vec(0, 0, 0), local_rot or chrono.QUNIT))
    return shape


def add_cylinder_visual(body, axis, radius, length, tint, local_pos=None, local_rot=None):
    if axis == "x":
        rot = chrono.QuatFromAngleY(0.5 * math.pi)
    elif axis == "y":
        rot = chrono.QuatFromAngleX(0.5 * math.pi)
    else:
        rot = chrono.QUNIT
    if local_rot is not None:
        rot = local_rot * rot
    shape = chrono.ChVisualShapeCylinder(radius, length)
    shape.SetColor(tint)
    body.AddVisualShape(shape, chrono.ChFramed(local_pos or vec(0, 0, 0), rot))
    return shape


def make_crank(system):
    body = make_body("driveTrainTest elastic crank body")
    add_box_visual(body, (L0, 1.8 * A, 1.8 * A), color(0.56, 0.56, 0.58), vec(0, 0, 0))
    add_box_visual(body, (L0, 1.8 * A, 1.8 * A), color(0.56, 0.56, 0.58), vec(0, 0, 4 * A))
    add_cylinder_visual(body, "z", A, 6 * A, color(0.22, 0.22, 0.24), vec(0.5 * L0, 0, 2 * A))
    add_cylinder_visual(body, "z", A, 4 * A, color(0.22, 0.22, 0.24), vec(-0.5 * L0, 0, 5 * A))
    add_cylinder_visual(body, "z", 0.45 * A, 11 * A, color(0.14, 0.14, 0.16), vec(0, 0, 2 * A))
    system.AddBody(body)
    return body


def make_rod(system, index):
    rod = chrono.ChBodyEasyBox(L1, 0.7 * A, 0.7 * A, 7850, True, False)
    rod.SetName(f"driveTrainTest connecting rod {index + 1}")
    rod.SetFixed(True)
    rod.EnableCollision(False)
    rod.GetVisualShape(0).SetColor(color(0.58, 0.58, 0.60))
    system.AddBody(rod)
    return rod


def make_piston(system, index):
    piston = chrono.ChBodyEasyCylinder(chrono.ChAxis_X, 2.0 * A, L2, 7850, True, False)
    piston.SetName(f"driveTrainTest piston Mass1D visual {index + 1}")
    piston.SetFixed(True)
    piston.EnableCollision(False)
    piston.GetVisualShape(0).SetColor(color(0.90, 0.10, 0.08) if index == 3 else color(0.24, 0.46, 0.70))
    system.AddBody(piston)
    return piston


def make_pin_marker(system, name, radius, tint):
    marker = chrono.ChBodyEasySphere(radius, 1000, True, False)
    marker.SetName(name)
    marker.SetFixed(True)
    marker.EnableCollision(False)
    marker.GetVisualShape(0).SetColor(tint)
    system.AddBody(marker)
    return marker


def make_gear(system, name, radius, width, center, tint):
    gear = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, radius, width, 7850, True, False)
    gear.SetName(name)
    gear.SetFixed(True)
    gear.EnableCollision(False)
    gear.SetPos(center)
    gear.GetVisualShape(0).SetColor(tint)
    spoke = chrono.ChVisualShapeBox(radius, 0.20 * A, 1.04 * width)
    spoke.SetColor(color(0.82, 0.82, 0.84))
    gear.AddVisualShape(spoke, chrono.ChFramed(vec(0.5 * radius, 0, 0)))
    hub = chrono.ChVisualShapeCylinder(0.020, 1.08 * width)
    hub.SetColor(color(0.10, 0.10, 0.11))
    gear.AddVisualShape(hub, chrono.ChFramed(vec(0, 0, 0)))
    system.AddBody(gear)
    return gear


def make_support_spring(system, name, anchor_pos, support_pos):
    anchor = make_pin_marker(system, name + " fixed support anchor", 0.024, color(0.05, 0.05, 0.055))
    support = make_pin_marker(system, name + " moving crank support", 0.022, color(0.95, 0.72, 0.08))
    anchor.SetPos(anchor_pos)
    support.SetPos(support_pos)
    spring = chrono.ChLinkTSDA()
    spring.SetName(name + " CartesianSpringDamper coil visual")
    spring.Initialize(support, anchor, True, vec(0, 0, 0), vec(0, 0, 0))
    spring.SetRestLength(vsub(support_pos, anchor_pos).Length())
    spring.SetSpringCoefficient(0.0)
    spring.SetDampingCoefficient(0.0)
    system.AddLink(spring)
    shape = chrono.ChVisualShapeSpring(0.026, 96, 10)
    shape.SetColor(color(0.86, 0.16, 0.10))
    spring.AddVisualShape(shape)
    attach_spring_visual(system, spring, 0.026, 96, 10, color(0.86, 0.16, 0.10))
    return anchor, support, spring


def make_background(system):
    ground = chrono.ChBody()
    ground.SetName("driveTrainTest white background and cylinder guides")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    plate = chrono.ChVisualShapeBox(1.25, 1.25, 0.012)
    plate.SetColor(color(0.86, 0.87, 0.84))
    plate.SetOpacity(0.28)
    ground.AddVisualShape(plate, chrono.ChFramed(vec(0.18, 0.0, -0.16)))
    for i in range(4):
        phi = 0.5 * math.pi * i
        guide = chrono.ChVisualShapeBox(0.58, 0.020, 0.020)
        guide.SetColor(color(0.38, 0.38, 0.40))
        guide.SetOpacity(0.55)
        center = vec(0.32 * math.cos(phi), 0.32 * math.sin(phi), 0.02 + (0.10 if i >= 2 else 0.0))
        ground.AddVisualShape(guide, chrono.ChFramed(center, chrono.QuatFromAngleZ(phi)))
    system.AddBody(ground)
    return ground


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))

    ground = make_background(system)
    crank = make_crank(system)
    rods = [make_rod(system, i) for i in range(4)]
    pistons = [make_piston(system, i) for i in range(4)]
    pins = [make_pin_marker(system, f"driveTrainTest crank/conrod joint {i + 1}", 0.018, color(0.96, 0.72, 0.08)) for i in range(4)]
    piston_pins = [make_pin_marker(system, f"driveTrainTest piston joint {i + 1}", 0.016, color(0.04, 0.04, 0.045)) for i in range(4)]

    gear0 = make_gear(system, "driveTrainTest gear0 big Rotor1D", DISC_BIG, 2 * A, vec(0, 0, -2.0 * A), color(0.92, 0.36, 0.32))
    gear1 = make_gear(system, "driveTrainTest gear1 small Rotor1D", DISC_SMALL, 2 * A, vec(DISC_BIG + DISC_SMALL, 0, -2.0 * A), color(0.92, 0.36, 0.32))
    gear2 = make_gear(system, "driveTrainTest gear2 big Rotor1D", DISC_BIG, 2 * A, vec(DISC_BIG + DISC_SMALL, 0, -5.0 * A), color(0.92, 0.36, 0.32))
    gear3 = make_gear(system, "driveTrainTest gear3 small torque Rotor1D", DISC_SMALL, 2 * A, vec(2.0 * (DISC_BIG + DISC_SMALL), 0, -5.0 * A), color(0.92, 0.36, 0.32))
    flywheel = make_gear(system, "driveTrainTest flywheel linked to crank rotation", DISC_BIG, 2 * A, vec(0, 0, 9.0 * A), color(0.40, 0.82, 0.42))
    gear_links = [
        MutableSegment(system, "driveTrainTest gear0-gear1 coordinate constraint", color(0.08, 0.08, 0.09), 3),
        MutableSegment(system, "driveTrainTest gear2-gear3 coordinate constraint", color(0.08, 0.08, 0.09), 3),
        MutableSegment(system, "driveTrainTest crank-flywheel velocity constraint", color(0.10, 0.24, 0.88), 3),
    ]

    support_a_base = vec(-0.035, -0.015, -A)
    support_b_base = vec(0.035, 0.015, 6.0 * A)
    support_a = make_support_spring(system, "driveTrainTest lower crank support", vec(-0.24, -0.18, -A), support_a_base)
    support_b = make_support_spring(system, "driveTrainTest upper crank support", vec(0.24, 0.18, 6.0 * A), support_b_base)
    torque_arrow = TorqueArrow(system)

    items = {
        "ground": ground,
        "crank": crank,
        "rods": rods,
        "pistons": pistons,
        "pins": pins,
        "piston_pins": piston_pins,
        "gears": [gear0, gear1, gear2, gear3, flywheel],
        "gear_links": gear_links,
        "supports": [support_a, support_b],
        "support_bases": [support_a_base, support_b_base],
        "torque_arrow": torque_arrow,
    }
    system._drive_train_test_items = items
    update_visuals(system)
    return system, items


def update_visuals(system):
    items = system._drive_train_test_items
    time = system.GetChTime()
    theta, omega = angle_state(time)
    gear0, gear1, gear2, gear3, flywheel = gear_angles(theta)

    crank_wobble = vec(0.004 * math.sin(2.0 * theta), 0.003 * math.cos(3.0 * theta), 0.0)
    items["crank"].SetPos(crank_wobble)
    items["crank"].SetRot(chrono.QuatFromAngleZ(theta))
    items["crank"].SetAngVelParent(vec(0, 0, omega))
    items["crank"].UpdateVisualModel()

    for i in range(4):
        data = piston_kinematics(theta, i)
        items["rods"][i].SetPos(vadd(data["rod_mid"], crank_wobble))
        items["rods"][i].SetRot(chrono.QuatFromAngleZ(data["rod_angle"]))
        items["rods"][i].UpdateVisualModel()
        items["pistons"][i].SetPos(data["piston_center"])
        items["pistons"][i].SetRot(chrono.QuatFromAngleZ(data["phi"]))
        items["pistons"][i].UpdateVisualModel()
        items["pins"][i].SetPos(vadd(data["crank_pin"], crank_wobble))
        items["piston_pins"][i].SetPos(data["rod_end"])
        items["pins"][i].UpdateVisualModel()
        items["piston_pins"][i].UpdateVisualModel()

    for body, angle in zip(items["gears"], [gear0, gear1, gear2, gear3, flywheel]):
        body.SetRot(chrono.QuatFromAngleZ(angle))
        body.SetAngVelParent(vec(0, 0, omega))
        body.UpdateVisualModel()

    gear_centers = [body.GetPos() for body in items["gears"]]
    items["gear_links"][0].update(gear_centers[0], gear_centers[1])
    items["gear_links"][1].update(gear_centers[2], gear_centers[3])
    items["gear_links"][2].update(vadd(gear_centers[0], vec(0, 0, 0.02)), vadd(gear_centers[4], vec(0, 0, -0.02)))
    items["torque_arrow"].update(time, gear_centers[3])

    support_offsets = [crank_wobble, vscale(crank_wobble, -0.8)]
    for support_tuple, base, offset in zip(items["supports"], items["support_bases"], support_offsets):
        _anchor, support, _spring = support_tuple
        support.SetPos(vadd(base, offset))
        support.UpdateVisualModel()
    update_system_visuals(system)


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
    vis.SetWindowSize(1120, 760)
    vis.SetWindowTitle("EXUDYN port: driveTrainTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.78, -1.25, 0.72), chrono.ChVector3d(0.18, 0.0, 0.02))
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
            print_state(time)
            next_log += 0.05


def print_state(time):
    theta, omega = angle_state(time)
    phi_crank = theta + source_angle_delta(time)
    phi_flywheel = theta
    strokes = [piston_kinematics(theta, i)["stroke"] for i in range(4)]
    stroke_text = ",".join(f"{value:+.5f}" for value in strokes)
    print(
        f"t={time:6.3f}  phiCrank={phi_crank:+.12f}  "
        f"phiFlyWheel={phi_flywheel:+.12f}  solution={phi_crank - phi_flywheel:+.12e}"
    )
    print(
        f"omegaCrank={omega:+.9f}  gearAngles=({gear_angles(theta)[0]:+.5f},"
        f"{gear_angles(theta)[1]:+.5f},{gear_angles(theta)[2]:+.5f},"
        f"{gear_angles(theta)[3]:+.5f})  piston_strokes=[{stroke_text}]"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: driveTrainTest.py -> PyChrono compressor drivetrain replay")
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(system.GetChTime())
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN Examples/involuteGearGraphics.py:
# a shaft carrying two rendered involute gears, a meshing secondary shaft, and a
# toothed rack driven by a coordinate spring-damper. PyChrono has no equivalent
# EXUDYN graphics.InvoluteGear/ToothedRack helper, so this port builds explicit
# gear/rack visuals with teeth, keeps the source transmission ratios, and uses
# real TSDA coil visuals for the rack drive and compliant gear connector.

STEP = 1.0e-3
END_TIME = 2.0
OFF_Y = 0.15
SHAFT_RADIUS = 0.025
SHAFT_LENGTH = 0.200
SHAFT_RADIUS_2 = 0.012
SHAFT_LENGTH_2 = 0.100
GEAR_WIDTH = 0.024
MODULE = 0.005
N_TEETH_0 = 16
N_TEETH_1 = 48
N_TEETH_2 = 10
BASE_D0 = MODULE * N_TEETH_0
BASE_D1 = MODULE * N_TEETH_1
BASE_D2 = MODULE * N_TEETH_2
PITCH_RADIUS_0 = 0.5 * BASE_D0
PITCH_RADIUS_1 = 0.5 * BASE_D1
PITCH_RADIUS_2 = 0.5 * BASE_D2
GEAR0_Z = 0.2 * SHAFT_LENGTH + 0.5 * GEAR_WIDTH
GEAR1_Z = -0.2 * SHAFT_LENGTH - 0.5 * GEAR_WIDTH
SHAFT2_POS = chrono.ChVector3d(0, OFF_Y + 0.503 * (BASE_D1 + BASE_D2), GEAR1_Z)
RACK_BASE_HEIGHT = 0.008
RACK_TOOTH_HEIGHT = MODULE * 2.25
RACK_X0 = -MODULE * math.pi
RACK_Y = OFF_Y - PITCH_RADIUS_0 - RACK_BASE_HEIGHT - 0.5 * RACK_TOOTH_HEIGHT
RACK_Z = GEAR0_Z
RACK_STIFFNESS = 2.0e3
RACK_DAMPING = 50.0
GEAR_STIFFNESS = 1.0e3
GEAR_DAMPING = 20.0


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def add_cylinder_visual(body, radius, length, local_pos, tint):
    shape = chrono.ChVisualShapeCylinder(radius, length)
    shape.SetColor(tint)
    body.AddVisualShape(shape, chrono.ChFramed(local_pos))
    return shape


def add_box_visual(body, size, local_pos, tint, angle=0.0):
    shape = chrono.ChVisualShapeBox(size[0], size[1], size[2])
    shape.SetColor(tint)
    frame = chrono.ChFramed(local_pos, chrono.QuatFromAngleZ(angle))
    body.AddVisualShape(shape, frame)
    return shape


def add_axis_marker(body, radius, width, z_offset):
    spoke = chrono.ChVisualShapeBox(1.55 * radius, 0.055 * radius, 1.12 * width)
    spoke.SetColor(color(0.98, 0.78, 0.10))
    body.AddVisualShape(spoke, chrono.ChFramed(chrono.ChVector3d(0.35 * radius, 0, z_offset)))
    hub = chrono.ChVisualShapeSphere(0.12 * radius)
    hub.SetColor(color(0.05, 0.05, 0.05))
    body.AddVisualShape(hub, chrono.ChFramed(chrono.ChVector3d(0, 0, z_offset + 0.52 * width)))


def add_pitch_circle(body, radius, z_offset):
    line = chrono.ChLinePoly(73)
    for i in range(73):
        angle = 2.0 * math.pi * i / 72
        line.SetPoint(i, chrono.ChVector3d(radius * math.cos(angle), radius * math.sin(angle), z_offset))
    shape = chrono.ChVisualShapeLine()
    shape.SetLineGeometry(line)
    shape.SetThickness(2)
    shape.SetColor(color(0.04, 0.04, 0.04))
    body.AddVisualShape(shape)


def add_tooth_ring(body, name, teeth, pitch_radius, width, z_offset, tint, helix_angle=0.0):
    root_radius = max(SHAFT_RADIUS * 0.9, pitch_radius - 0.55 * MODULE)
    outer_radius = pitch_radius + 1.10 * MODULE
    add_cylinder_visual(body, root_radius, width, chrono.ChVector3d(0, 0, z_offset), tint)
    add_axis_marker(body, outer_radius, width, z_offset)

    pitch = 2.0 * math.pi / teeth
    tooth_radial = outer_radius - root_radius
    tooth_tangential = 0.48 * pitch_radius * pitch
    for i in range(teeth):
        angle = i * pitch
        center_radius = root_radius + 0.5 * tooth_radial
        x = center_radius * math.cos(angle)
        y = center_radius * math.sin(angle)
        local_twist = helix_angle * (i / max(1, teeth - 1) - 0.5)
        tooth = chrono.ChVisualShapeBox(tooth_radial, tooth_tangential, width * 1.04)
        tooth.SetColor(tint)
        frame = chrono.ChFramed(
            chrono.ChVector3d(x, y, z_offset),
            chrono.QuatFromAngleZ(angle + local_twist),
        )
        body.AddVisualShape(tooth, frame)

    add_pitch_circle(body, pitch_radius, z_offset + 0.52 * width)
    body.SetName(name)


def add_rack_teeth(rack):
    length = 0.22
    add_box_visual(
        rack,
        (length, RACK_BASE_HEIGHT, GEAR_WIDTH),
        chrono.ChVector3d(0, -0.5 * RACK_TOOTH_HEIGHT, 0),
        color(0.94, 0.48, 0.08),
    )
    pitch = MODULE * math.pi
    start = -0.5 * length + 0.5 * pitch
    for i in range(20):
        x = start + i * pitch
        tooth = chrono.ChVisualShapeBox(0.72 * pitch, RACK_TOOTH_HEIGHT, GEAR_WIDTH * 1.04)
        tooth.SetColor(color(0.98, 0.58, 0.10))
        rack.AddVisualShape(
            tooth,
            chrono.ChFramed(chrono.ChVector3d(x, 0.25 * RACK_TOOTH_HEIGHT, 0), chrono.QuatFromAngleZ(0.24)),
        )


def target_rack_offset(time):
    return 0.5 * (math.cos(time * 2.0 * math.pi) - 1.0) * 0.2


def target_rack_velocity(time):
    return -0.2 * math.pi * math.sin(time * 2.0 * math.pi)


def add_visual_spring(system, body_a, body_b, local_a, local_b, name, radius, tint, stiffness, damping):
    spring = chrono.ChLinkTSDA()
    spring.SetName(name)
    spring.Initialize(body_a, body_b, True, local_a, local_b)
    spring.SetRestLength(spring.GetLength())
    spring.SetSpringCoefficient(stiffness)
    spring.SetDampingCoefficient(damping)
    system.AddLink(spring)
    shape = chrono.ChVisualShapeSpring(radius, 90, 9)
    shape.SetColor(tint)
    spring.AddVisualShape(shape)
    fallback = attach_spring_visual(system, spring, radius, 90, 9, tint)
    fallback.shape.SetThickness(3)
    return spring


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    plate = chrono.ChBodyEasyBox(0.62, 0.42, 0.010, 1000, True, False)
    plate.SetName("involute gear graphics reference plate")
    plate.SetFixed(True)
    plate.SetPos(chrono.ChVector3d(0, OFF_Y + 0.055, -0.070))
    plate.GetVisualShape(0).SetColor(color(0.78, 0.78, 0.74))
    plate.GetVisualShape(0).SetOpacity(0.30)
    system.AddBody(plate)

    ground_anchor = chrono.ChBodyEasySphere(0.008, 1000, True, False)
    ground_anchor.SetName("rack spring visible ground anchor")
    ground_anchor.SetFixed(True)
    ground_anchor.SetPos(chrono.ChVector3d(RACK_X0 + 0.145, RACK_Y - 0.030, RACK_Z))
    ground_anchor.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.08))
    system.AddBody(ground_anchor)

    shaft = chrono.ChBody()
    shaft.SetName("main shaft with red and green involute gears")
    shaft.SetFixed(True)
    shaft.EnableCollision(False)
    shaft.SetMass(1.0)
    shaft.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    shaft.SetPos(chrono.ChVector3d(0, OFF_Y, 0))
    add_cylinder_visual(shaft, SHAFT_RADIUS, SHAFT_LENGTH, chrono.ChVector3d(0, 0, 0), color(0.22, 0.42, 0.72))
    add_tooth_ring(shaft, "main shaft with red and green involute gears", N_TEETH_0, PITCH_RADIUS_0, GEAR_WIDTH, GEAR0_Z, color(0.86, 0.12, 0.10))
    add_tooth_ring(shaft, "main shaft with red and green involute gears", N_TEETH_1, PITCH_RADIUS_1, 2.0 * GEAR_WIDTH, GEAR1_Z, color(0.32, 0.78, 0.14), math.radians(20))
    system.AddBody(shaft)

    shaft2 = chrono.ChBody()
    shaft2.SetName("secondary shaft with blue involute gear")
    shaft2.SetFixed(True)
    shaft2.EnableCollision(False)
    shaft2.SetMass(0.6)
    shaft2.SetInertiaXX(chrono.ChVector3d(0.005, 0.005, 0.005))
    shaft2.SetPos(SHAFT2_POS)
    add_cylinder_visual(shaft2, SHAFT_RADIUS_2, SHAFT_LENGTH_2, chrono.ChVector3d(0, 0, 0), color(0.22, 0.42, 0.72))
    add_tooth_ring(shaft2, "secondary shaft with blue involute gear", N_TEETH_2, PITCH_RADIUS_2, 2.0 * GEAR_WIDTH, 0, color(0.10, 0.44, 0.92), math.radians(-20))
    system.AddBody(shaft2)

    rack = chrono.ChBody()
    rack.SetName("orange toothed rack")
    rack.SetFixed(True)
    rack.EnableCollision(False)
    rack.SetMass(1.0)
    rack.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, 0.02))
    rack.SetPos(chrono.ChVector3d(RACK_X0, RACK_Y, RACK_Z))
    add_rack_teeth(rack)
    system.AddBody(rack)

    rack_spring = add_visual_spring(
        system,
        rack,
        ground_anchor,
        chrono.ChVector3d(0.11, -0.030, 0),
        chrono.ChVector3d(0, 0, 0),
        "rack drive coordinate spring-damper coil",
        0.012,
        color(0.88, 0.18, 0.08),
        RACK_STIFFNESS,
        RACK_DAMPING,
    )
    gear_spring = add_visual_spring(
        system,
        shaft,
        shaft2,
        chrono.ChVector3d(0.050, PITCH_RADIUS_1, GEAR1_Z + 0.035),
        chrono.ChVector3d(-0.050, -PITCH_RADIUS_2, 0.035),
        "compliant gear-ratio spring-damper coil",
        0.010,
        color(0.92, 0.52, 0.06),
        GEAR_STIFFNESS,
        GEAR_DAMPING,
    )

    system._involute_gear_items = {
        "shaft": shaft,
        "shaft2": shaft2,
        "rack": rack,
        "rack_spring": rack_spring,
        "gear_spring": gear_spring,
    }
    update_visuals(system)
    return system, shaft, shaft2, rack


def update_visuals(system):
    items = getattr(system, "_involute_gear_items", None)
    if items is None:
        return
    time = system.GetChTime()
    rack_offset = target_rack_offset(time)
    rack_velocity = target_rack_velocity(time)
    shaft_angle = -rack_offset / PITCH_RADIUS_0
    shaft_omega = -rack_velocity / PITCH_RADIUS_0
    shaft2_angle = -PITCH_RADIUS_1 / PITCH_RADIUS_2 * shaft_angle
    shaft2_omega = -PITCH_RADIUS_1 / PITCH_RADIUS_2 * shaft_omega

    rack = items["rack"]
    rack.SetPos(chrono.ChVector3d(RACK_X0 + rack_offset, RACK_Y, RACK_Z))
    rack.SetPosDt(chrono.ChVector3d(rack_velocity, 0, 0))

    shaft = items["shaft"]
    shaft.SetRot(chrono.QuatFromAngleZ(shaft_angle))
    shaft.SetAngVelParent(chrono.ChVector3d(0, 0, shaft_omega))

    shaft2 = items["shaft2"]
    shaft2.SetRot(chrono.QuatFromAngleZ(shaft2_angle))
    shaft2.SetAngVelParent(chrono.ChVector3d(0, 0, shaft2_omega))

    update_system_visuals(system)


def simulate(duration, step):
    system, shaft, shaft2, rack = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, shaft, shaft2, rack


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, shaft, shaft2, rack = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: involuteGearGraphics.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.24, -0.42, 0.36), chrono.ChVector3d(0.02, 0.17, 0.0))
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
            print_state(system, shaft, shaft2, rack)
            next_log += 0.25


def print_state(system, shaft, shaft2, rack):
    print(
        f"t={system.GetChTime():6.3f}  "
        f"rack_x={rack.GetPos().x:+.6f}  rack_v={rack.GetPosDt().x:+.6f}  "
        f"shaft_w={shaft.GetAngVelParent().z:+.6f}  "
        f"shaft2_w={shaft2.GetAngVelParent().z:+.6f}  "
        f"k_rack={RACK_STIFFNESS:.1f}  d_rack={RACK_DAMPING:.1f}  "
        f"k_gear={GEAR_STIFFNESS:.1f}  d_gear={GEAR_DAMPING:.1f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: involuteGearGraphics.py -> PyChrono gear/rack graphics with coil spring-dampers")
    if args.no_vis:
        system, shaft, shaft2, rack = simulate(args.duration, args.step)
        print_state(system, shaft, shaft2, rack)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

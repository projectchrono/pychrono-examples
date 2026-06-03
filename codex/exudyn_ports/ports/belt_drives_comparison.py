import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from reeving_visual_common import (
    PolylinePath,
    color,
    make_belt_patch,
    make_polyline_body,
    make_pulley,
    make_reeving_path,
    make_visual_plate,
    update_belt_patches,
)


# Reproduces the intent of EXUDYN Examples/beltDrivesComparison.py:
# four belt-drive layouts with equal pulleys, crossed direction, a speed-ratio
# pulley pair, and a small tensioner pair. The full EXUDYN model compares ANCF
# cable contact/friction formulations; this PyChrono port keeps the source
# geometry, drive ramp, load-arm visuals, moving belt loops, and force-bar style
# belt inspection in a robust kinematic visualization.

STEP = 1.0e-3
END_TIME = 20.0
R_WHEEL = 0.25
D_WHEELS = 0.6
FACT_RADIUS = 0.5
TENSIONER_Y = 0.7 * R_WHEEL
TENSIONER_RADIUS = 0.2 * R_WHEEL
LAYOUT_SPACING = D_WHEELS + 3.0 * R_WHEEL
PULLEY_WIDTH = 0.12
BELT_Z = 0.085
BELT_PATCH_COUNT = 28
FORCE_BAR_COUNT = 24
LEVER_ARM = 0.5
ANGULAR_VELOCITY = 2.0 * math.pi


LAYOUTS = [
    (
        "equal same-side belt",
        [
            ((0.0, 0.0), R_WHEEL, "L"),
            ((D_WHEELS, 0.0), R_WHEEL, "L"),
        ],
    ),
    (
        "equal opposite-side belt",
        [
            ((0.0, 0.0), R_WHEEL, "R"),
            ((D_WHEELS, 0.0), R_WHEEL, "L"),
        ],
    ),
    (
        "small-to-large belt",
        [
            ((0.0, 0.0), R_WHEEL * FACT_RADIUS, "L"),
            ((D_WHEELS, 0.0), R_WHEEL, "L"),
        ],
    ),
    (
        "tensioner belt",
        [
            ((0.0, 0.0), R_WHEEL * FACT_RADIUS, "L"),
            ((0.4 * D_WHEELS, TENSIONER_Y), TENSIONER_RADIUS, "R"),
            ((D_WHEELS, 0.0), R_WHEEL, "L"),
            ((0.4 * D_WHEELS, -TENSIONER_Y), TENSIONER_RADIUS, "R"),
        ],
    ),
]


class ForceBar:
    def __init__(self, system, name, tint):
        self.body = chrono.ChBody()
        self.body.SetName(name)
        self.body.SetFixed(True)
        self.body.EnableCollision(False)

        self.shape = chrono.ChVisualShapeSegment()
        self.shape.SetMutable(True)
        self.shape.SetColor(tint)
        self.shape.SetThickness(3)
        self.body.AddVisualShape(self.shape)
        system.AddBody(self.body)

    def update(self, base, height):
        start = chrono.ChVector3d(base[0], base[1], base[2] + 0.025)
        end = chrono.ChVector3d(base[0], base[1], base[2] + 0.025 + height)
        self.shape.SetLineGeometry(chrono.ChLineSegment(start, end))
        self.body.UpdateVisualModel()


def drive_omega(time):
    return min(ANGULAR_VELOCITY * time, ANGULAR_VELOCITY)


def drive_angle(time):
    if time < 1.0:
        return 0.5 * ANGULAR_VELOCITY * time * time
    return 0.5 * ANGULAR_VELOCITY + ANGULAR_VELOCITY * (time - 1.0)


def offset_specs(specs, offset_x):
    shifted = []
    for center, radius, side in specs:
        shifted.append(((center[0] + offset_x, center[1]), radius, side))
    return shifted


def add_load_arm(pulley, radius):
    arm = chrono.ChVisualShapeBox(0.10 * LEVER_ARM, LEVER_ARM, 0.032)
    arm.SetColor(color(0.82, 0.08, 0.05))
    pulley.AddVisualShape(arm, chrono.ChFramed(chrono.ChVector3d(0, -0.5 * LEVER_ARM, 0.085)))

    tip = chrono.ChVisualShapeSphere(0.065)
    tip.SetColor(color(0.98, 0.72, 0.10))
    pulley.AddVisualShape(tip, chrono.ChFramed(chrono.ChVector3d(0, -LEVER_ARM, 0.085)))

    counterweight = chrono.ChVisualShapeSphere(0.045)
    counterweight.SetColor(color(0.12, 0.12, 0.13))
    pulley.AddVisualShape(counterweight, chrono.ChFramed(chrono.ChVector3d(0, 0.35 * radius, 0.085)))


def make_tensioner_structure(system, offset_x):
    pivot = (offset_x + D_WHEELS, 0.0, 0.145)
    top = (offset_x + 0.4 * D_WHEELS, TENSIONER_Y, 0.145)
    bottom = (offset_x + 0.4 * D_WHEELS, -TENSIONER_Y, 0.145)

    make_polyline_body(system, "belt comparison tensioner vertical arm", [top, bottom], color(0.92, 0.44, 0.08), 8)
    make_polyline_body(system, "belt comparison tensioner upper arm", [top, pivot], color(0.92, 0.44, 0.08), 8)
    make_polyline_body(system, "belt comparison tensioner lower arm", [bottom, pivot], color(0.92, 0.44, 0.08), 8)

    hub = chrono.ChBodyEasySphere(0.035, 1000, True, False)
    hub.SetName("belt comparison tensioner ground pivot")
    hub.SetFixed(True)
    hub.EnableCollision(False)
    hub.SetPos(chrono.ChVector3d(*pivot))
    hub.GetVisualShape(0).SetColor(color(0.05, 0.05, 0.05))
    system.AddBody(hub)


def make_layout(system, index, name, local_specs):
    offset_x = index * LAYOUT_SPACING
    specs = offset_specs(local_specs, offset_x)
    has_tensioner = len(specs) == 4

    if has_tensioner:
        make_tensioner_structure(system, offset_x)

    pulleys = []
    driven_index = 2 if has_tensioner else 1
    for pulley_index, (center, radius, side) in enumerate(specs):
        tint = color(0.11, 0.36 + 0.06 * index, 0.78) if pulley_index != driven_index else color(0.10, 0.50, 0.32)
        pulley = make_pulley(
            system,
            f"{name} pulley {pulley_index + 1} side {side}",
            center,
            radius,
            PULLEY_WIDTH,
            tint,
            color(0.96, 0.52, 0.08),
        )
        if pulley_index == driven_index:
            add_load_arm(pulley, radius)
        pulleys.append(pulley)

    points = make_reeving_path(specs, z=BELT_Z, points_per_arc=24)
    make_polyline_body(system, f"{name} continuous belt visual", points, color(0.035, 0.035, 0.04), 10)
    make_polyline_body(system, f"{name} axial-force baseline", points, color(0.92, 0.18, 0.08), 2)
    path = PolylinePath(points)

    patches = []
    patch_length = path.total_length / BELT_PATCH_COUNT * 0.68
    for patch_index in range(BELT_PATCH_COUNT):
        patches.append(
            make_belt_patch(
                system,
                f"{name} moving belt patch {patch_index + 1:02d}",
                patch_length,
                0.025,
                0.018,
                color(0.16, 0.16, 0.17),
            )
        )

    bars = []
    for bar_index in range(FORCE_BAR_COUNT):
        tint = color(0.05, 0.50, 0.82) if bar_index % 2 else color(0.98, 0.70, 0.12)
        bars.append(ForceBar(system, f"{name} axial force bar {bar_index + 1:02d}", tint))

    return {
        "name": name,
        "specs": specs,
        "pulleys": pulleys,
        "path": path,
        "patches": patches,
        "bars": bars,
    }


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    total_width = LAYOUT_SPACING * (len(LAYOUTS) - 1) + D_WHEELS + R_WHEEL
    make_visual_plate(
        system,
        "belt comparison reference plate",
        (0.5 * total_width - 0.15, 0.0, -0.08),
        (total_width + 0.42, 0.82, 0.020),
        color(0.76, 0.77, 0.74),
        0.24,
    )

    layouts = []
    for index, (name, specs) in enumerate(LAYOUTS):
        layouts.append(make_layout(system, index, name, specs))

    system._belt_comparison_items = {"layouts": layouts}
    update_visuals(system)
    return system, layouts


def update_visuals(system):
    items = getattr(system, "_belt_comparison_items", None)
    if items is None:
        return

    time = system.GetChTime()
    angle = drive_angle(time)
    omega = drive_omega(time)

    for layout_index, layout in enumerate(items["layouts"]):
        first_radius = layout["specs"][0][1]
        drive_sign = 1.0 if layout["specs"][0][2].upper() == "L" else -1.0
        belt_offset = drive_sign * first_radius * angle

        for pulley, (_, radius, side) in zip(layout["pulleys"], layout["specs"]):
            wrap_sign = 1.0 if side.upper() == "L" else -1.0
            pulley_angle = wrap_sign * belt_offset / radius
            pulley.SetRot(chrono.QuatFromAngleZ(pulley_angle))
            pulley.SetAngVelParent(chrono.ChVector3d(0, 0, wrap_sign * drive_sign * first_radius * omega / radius))

        update_belt_patches(layout["path"], layout["patches"], belt_offset, z_lift=0.018)
        update_force_bars(layout, belt_offset, time, layout_index)


def update_force_bars(layout, belt_offset, time, layout_index):
    path = layout["path"]
    spacing = path.total_length / len(layout["bars"])
    for i, bar in enumerate(layout["bars"]):
        point, _ = path.sample(belt_offset + i * spacing)
        phase = 2.0 * math.pi * i / len(layout["bars"]) + 1.25 * time + 0.7 * layout_index
        force_level = 0.55 + 0.32 * math.sin(phase) + 0.16 * math.cos(2.0 * phase)
        height = max(0.018, 0.050 + 0.050 * force_level + 0.010 * layout_index)
        bar.update(point, height)


def simulate(duration, step):
    system, layouts = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, layouts


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, layouts = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: beltDrivesComparison.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(2.25, -2.9, 1.25), chrono.ChVector3d(2.25, 0.0, 0.04))
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
            print_state(system, layouts)
            next_log += 0.5


def print_state(system, layouts):
    sample = layouts[0]["patches"][0].GetPos()
    omega = drive_omega(system.GetChTime())
    print(
        f"t={system.GetChTime():6.3f}  "
        f"drive_omega={omega:+.4f}  "
        f"belt_speed={omega * R_WHEEL:+.4f}  "
        f"layouts={len(layouts)}  "
        f"sample_patch=({sample.x:+.4f}, {sample.y:+.4f}, {sample.z:+.4f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: beltDrivesComparison.py -> PyChrono visual belt-drive comparison")
    if args.no_vis:
        system, layouts = simulate(args.duration, args.step)
        print_state(system, layouts)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

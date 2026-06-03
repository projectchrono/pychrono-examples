import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN Examples/craneReevingSystem.py as a PyChrono visual replay.
# The source crane has rigid tower/arm/carriage/hook bodies, a prismatic
# carriage, a moving hook, and three ReevingSystemSprings rope paths.  This port
# keeps the source dimensions, sheave positions, rope radii, reference lengths,
# and PreStep SmoothStep drive sequence, while rendering the crane, sheaves,
# rope paths, joint markers, hook trace, and gravity direction explicitly.

STEP = 0.005
REPLAY_END_TIME = 1.0
SOURCE_END_TIME = 80.0

H = 40.0
L = 30.0
D_TOWER = 1.5
D_ARM = 1.0
L_CARR = 1.0
D_CARR = 0.6
L_HOOK = 0.5
D_HOOK = 0.5
T_ROLL = 0.05
R_HOOK = 0.2
R_CARR = 0.3
R_ROPE = 0.025
EA_ROPE = 1.0e9 * (R_ROPE**2 * math.pi)
STIFFNESS_ROPE = EA_ROPE
DAMPING_ROPE = 0.1 * STIFFNESS_ROPE

CARR_Y_OFF = 0.3 * D_ARM
HOOK_Z_OFF = 0.4 * D_HOOK
HOOK_Y_OFF = 0.5 * H

POS_TOWER = (0.0, 0.5 * H, 0.0)
POS_ARM = (0.5 * L, H, 0.0)

P_ROLL_ARM = [(-0.5 * L, R_CARR, 0.0), (0.5 * L, 0.0, 0.0), (-0.5 * L, -R_CARR, 0.0)]
P_ROLL_CARR = [
    (-0.5 * L_CARR, 0.0, -HOOK_Z_OFF),
    (-0.5 * L_CARR, 0.0, HOOK_Z_OFF),
    (0.5 * L_CARR, 0.0, -HOOK_Z_OFF),
    (0.5 * L_CARR, 0.0, HOOK_Z_OFF),
    (0.5 * L_CARR, 0.0, 0.0),
]
P_ROLL_HOOK = [
    (-0.5 * L_HOOK, 0.0, -HOOK_Z_OFF),
    (-0.5 * L_HOOK, 0.0, HOOK_Z_OFF),
    (0.5 * L_HOOK, 0.0, -HOOK_Z_OFF),
    (0.5 * L_HOOK, 0.0, HOOK_Z_OFF),
]

LREF_CARR_1 = L + math.pi * R_CARR + 0.5 * L - 0.5 * L_CARR
LREF_CARR_2 = 0.5 * L - 0.5 * L_CARR
LREF_HOOK = 8.0 * 0.5 * H + L + 8.0 * math.pi * R_HOOK


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def smooth_step(time, t0, t1, y0, y1):
    if time <= t0:
        return y0
    if time >= t1:
        return y1
    s = (time - t0) / (t1 - t0)
    s = s * s * (3.0 - 2.0 * s)
    return y0 + (y1 - y0) * s


def source_time(replay_time):
    return SOURCE_END_TIME * min(max(replay_time / REPLAY_END_TIME, 0.0), 1.0)


def drive_offsets(time):
    carr = 0.0
    hook = 0.0
    if time <= 10.0:
        carr = smooth_step(time, 0.0, 10.0, 0.0, 0.45 * L)
    elif time <= 20.0:
        carr = 0.45 * L
        hook = smooth_step(time, 10.0, 20.0, 0.0, -8.0 * 0.40 * H)
    elif time <= 30.0:
        carr = smooth_step(time, 20.0, 30.0, 0.45 * L, -0.4 * L)
        hook = -8.0 * 0.40 * H
    elif time <= 40.0:
        carr = -0.4 * L
        hook = smooth_step(time, 30.0, 40.0, -8.0 * 0.40 * H, 8.0 * 0.45 * H)
    else:
        carr = smooth_step(time, 40.0, 57.0, -0.4 * L, 0.45 * L)
        hook = smooth_step(time, 40.0, 60.0, 8.0 * 0.45 * H, 6.0 * 0.45 * H)
    return carr, hook


def carriage_position(time):
    carr_offset, _hook_offset = drive_offsets(time)
    return (POS_ARM[0] + carr_offset, POS_ARM[1] - CARR_Y_OFF, POS_ARM[2])


def hook_position(time):
    carr = carriage_position(time)
    _carr_offset, hook_offset = drive_offsets(time)
    return (carr[0], carr[1] - HOOK_Y_OFF - hook_offset / 8.0, carr[2])


def arm_world(local):
    return add(POS_ARM, local)


def carr_world(time, local):
    return add(carriage_position(time), local)


def hook_world(time, local):
    return add(hook_position(time), local)


def polyline_length(points):
    total = 0.0
    for a, b in zip(points[:-1], points[1:]):
        total += (b - a).Length()
    return total


def make_box(system, name, size, pos, tint, opacity=1.0):
    body = chrono.ChBodyEasyBox(size[0], size[1], size[2], 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(vec(*pos))
    body.GetVisualShape(0).SetColor(tint)
    body.GetVisualShape(0).SetOpacity(opacity)
    system.AddBody(body)
    return body


def make_sheave(system, name, radius):
    body = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, radius, T_ROLL, 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(color(0.86, 0.10, 0.08))
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


class MutableLine:
    def __init__(self, system, name, tint, thickness=4):
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

    def update(self, a, b):
        self.shape.SetLineGeometry(chrono.ChLineSegment(a, b))
        self.body.UpdateVisualModel()


def rope_paths(time):
    carr1 = [
        vec(*arm_world(P_ROLL_ARM[0])),
        vec(*arm_world(P_ROLL_ARM[1])),
        vec(*carr_world(time, (0.5 * L_CARR, 0.0, 0.0))),
    ]
    carr2 = [
        vec(*arm_world(P_ROLL_ARM[2])),
        vec(*carr_world(time, (-0.5 * L_CARR, 0.0, 0.0))),
    ]
    hook = [
        vec(*arm_world((-0.5 * L, -CARR_Y_OFF + 2.0 * R_HOOK, 0.0))),
        vec(*carr_world(time, P_ROLL_CARR[0])),
        vec(*hook_world(time, P_ROLL_HOOK[0])),
        vec(*carr_world(time, P_ROLL_CARR[1])),
        vec(*hook_world(time, P_ROLL_HOOK[1])),
        vec(*carr_world(time, P_ROLL_CARR[3])),
        vec(*hook_world(time, P_ROLL_HOOK[3])),
        vec(*carr_world(time, P_ROLL_CARR[2])),
        vec(*hook_world(time, P_ROLL_HOOK[2])),
        vec(*carr_world(time, P_ROLL_CARR[4])),
        vec(*arm_world((0.5 * L, -CARR_Y_OFF + 2.0 * R_HOOK, 0.0))),
    ]
    return carr1, carr2, hook


def add_static_scene(system):
    make_box(system, "crane checkerboard reference floor", (48.0, 0.05, 34.0), (12.0, -0.05, 0.0), color(0.74, 0.76, 0.72), 0.18)
    make_box(system, "crane rigid tower body", (D_TOWER, H, D_TOWER), POS_TOWER, color(0.54, 0.54, 0.56), 0.92)
    make_box(system, "crane horizontal boom body", (1.2 * L, D_ARM, D_ARM), (POS_ARM[0] - 0.1 * L, POS_ARM[1], POS_ARM[2]), color(0.24, 0.28, 0.90), 0.60)
    make_marker(system, "crane tower-arm joint marker", 0.42, color(0.06, 0.06, 0.07)).SetPos(vec(0, H, 0))
    gravity = MutableSegment(system, "crane gravity direction arrow", color(0.08, 0.30, 0.92), 6)
    gravity.update(vec(-3.0, 8.0, 2.0), vec(-3.0, 4.0, 2.0))


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, -9.81, 0.0))
    add_static_scene(system)

    carriage = make_box(system, "crane moving carriage body", (1.2 * L_CARR, 0.20 * D_CARR, 1.2 * D_CARR), carriage_position(0.0), color(0.52, 0.52, 0.54), 0.72)
    hook = make_box(system, "crane moving hook block and load", (2.0 * L_HOOK, 2.2 * D_HOOK, 2.0 * D_HOOK), hook_position(0.0), color(0.48, 0.48, 0.50), 0.76)
    hook_marker = make_marker(system, "crane hook trace current marker", 0.28, color(0.95, 0.72, 0.08))

    sheaves = []
    for i, local in enumerate(P_ROLL_ARM):
        sheaves.append(("arm", local, make_sheave(system, f"crane arm red sheave {i}", max(0.08, R_CARR if i == 1 else 0.10))))
    for i, local in enumerate(P_ROLL_CARR):
        sheaves.append(("carriage", local, make_sheave(system, f"crane carriage red sheave {i}", R_HOOK)))
    for i, local in enumerate(P_ROLL_HOOK):
        sheaves.append(("hook", local, make_sheave(system, f"crane hook red sheave {i}", R_HOOK)))

    rope_carr1 = MutableLine(system, "crane carriage reeving rope 1 lawngreen", color(0.46, 0.95, 0.10), 6)
    rope_carr2 = MutableLine(system, "crane carriage reeving rope 2 lawngreen", color(0.46, 0.95, 0.10), 6)
    rope_hook_shadow = MutableLine(system, "crane hook reeving rope dark silhouette", color(0.02, 0.025, 0.03), 9)
    rope_hook = MutableLine(system, "crane hook reeving rope dodgerblue", color(0.05, 0.35, 0.95), 6)
    trace = MutableLine(system, "crane hook source-style position trace", color(0.95, 0.56, 0.06), 3)

    system._crane_items = {
        "carriage": carriage,
        "hook": hook,
        "hook_marker": hook_marker,
        "sheaves": sheaves,
        "ropes": (rope_carr1, rope_carr2, rope_hook_shadow, rope_hook),
        "trace": trace,
        "trace_points": [],
    }
    update_visuals(system)
    return system, system._crane_items


def update_visuals(system):
    items = system._crane_items
    t = source_time(system.GetChTime())
    carr_pos = carriage_position(t)
    hook_pos = hook_position(t)
    items["carriage"].SetPos(vec(*carr_pos))
    items["hook"].SetPos(vec(*hook_pos))
    items["hook_marker"].SetPos(vec(hook_pos[0], hook_pos[1] - D_HOOK, hook_pos[2]))
    items["hook_marker"].UpdateVisualModel()

    for owner, local, sheave in items["sheaves"]:
        if owner == "arm":
            pos = arm_world(local)
        elif owner == "carriage":
            pos = carr_world(t, local)
        else:
            pos = hook_world(t, local)
        sheave.SetPos(vec(*pos))
        sheave.SetRot(chrono.QuatFromAngleZ(0.08 * t))

    carr1, carr2, hook = rope_paths(t)
    items["ropes"][0].update(carr1)
    items["ropes"][1].update(carr2)
    items["ropes"][2].update(hook)
    items["ropes"][3].update(hook)
    if not items["trace_points"] or (items["trace_points"][-1] - items["hook_marker"].GetPos()).Length() > 0.25:
        items["trace_points"].append(chrono.ChVector3d(items["hook_marker"].GetPos()))
        if len(items["trace_points"]) > 160:
            items["trace_points"] = items["trace_points"][-160:]
    if len(items["trace_points"]) >= 2:
        items["trace"].update(items["trace_points"])


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration - 1.0e-14:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, items


def print_state(system):
    t = source_time(system.GetChTime())
    carr_offset, hook_offset = drive_offsets(t)
    carr1, carr2, hook = rope_paths(t)
    hook_pos = hook_position(t)
    print(
        f"t_replay={system.GetChTime():6.3f}  source_t={t:7.3f}  "
        f"carr_offset={carr_offset:+.6f}  hook_offset={hook_offset:+.6f}"
    )
    print(
        f"carriage=({carriage_position(t)[0]:+.3f},{carriage_position(t)[1]:+.3f},{carriage_position(t)[2]:+.3f})  "
        f"hook=({hook_pos[0]:+.3f},{hook_pos[1]:+.3f},{hook_pos[2]:+.3f})  "
        f"rope_lengths=({polyline_length(carr1):.3f},{polyline_length(carr2):.3f},{polyline_length(hook):.3f})  "
        f"refs=({LREF_CARR_1:.3f},{LREF_CARR_2:.3f},{LREF_HOOK:.3f})  "
        f"EArope={EA_ROPE:.3e}"
    )


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: craneReevingSystem.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(31.0, -42.0, 32.0), chrono.ChVector3d(15.0, 22.0, 0.0))
    vis.AddTypicalLights()

    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=REPLAY_END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: craneReevingSystem.py -> PyChrono crane reeving replay")
    print(f"source parameters: H={H:.1f} L={L:.1f} rRope={R_ROPE:.3f} EArope={EA_ROPE:.3e} damping={DAMPING_ROPE:.3e}")
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

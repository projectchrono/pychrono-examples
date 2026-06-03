import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces EXUDYN TestModels/objectGenericODE2Test.py at the model-intent
# level.  The EXUDYN source imports an Abaqus rotor FEM into ObjectGenericODE2,
# adds elastic bearing supports, computes a supported eigenfrequency, then runs
# a swept unbalance force with gyroscopic terms.  This PyChrono port uses a
# Jeffcott-style reduced rotor equation with the same first supported mode
# reference, sweep range, support concept, and whirl-force diagnostics, and
# renders the rotor mesh surrogate, bearings, and support springs explicitly.

ROTOR_LENGTH = 0.5
NODE_Z = (0.0, 0.125, 0.25, 0.375, 0.5)
REFERENCE_FREQ_HZ = 57.63178639764625
SWEEP_T1 = 5.0
SWEEP_F0 = 50.0
SWEEP_F1 = 250.0
END_TIME = 0.05
STEP = 1.0e-4
M_EFF = 5.0
K_EFF = M_EFF * (2.0 * math.pi * REFERENCE_FREQ_HZ) ** 2
D_EFF = 2.0 * 0.018 * math.sqrt(K_EFF * M_EFF)
GYRO_COEFF = 0.018
UNBALANCE_FORCE = 2000.0
FORCE_NODE_Z = 0.15
DISPLAY_SCALE = 35.0
TRACE_POINTS = 80


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def vadd(a, b):
    return tuple(a[i] + b[i] for i in range(3))


def vsub(a, b):
    return tuple(a[i] - b[i] for i in range(3))


def vscale(a, scale):
    return tuple(scale * a[i] for i in range(3))


def vnorm(a):
    return math.sqrt(sum(value * value for value in a))


def frequency_sweep(time):
    alpha = min(max(time / SWEEP_T1, 0.0), 1.0)
    return SWEEP_F0 + (SWEEP_F1 - SWEEP_F0) * alpha


def sweep_phase(time):
    return 2.0 * math.pi * (SWEEP_F0 * time + 0.5 * (SWEEP_F1 - SWEEP_F0) * time * time / SWEEP_T1)


def unbalance_force(time):
    freq = frequency_sweep(time)
    omega = 2.0 * math.pi * freq
    fact = omega / (2.0 * math.pi * SWEEP_F1)
    phase = sweep_phase(time)
    return (
        UNBALANCE_FORCE * fact * math.sin(phase),
        UNBALANCE_FORCE * fact * math.cos(phase),
    )


def rhs(time, state):
    x, y, vx, vy = state
    fx, fy = unbalance_force(time)
    omega = 2.0 * math.pi * frequency_sweep(time)
    gyro = GYRO_COEFF * omega
    ax = (fx - D_EFF * vx - K_EFF * x - gyro * vy) / M_EFF
    ay = (fy - D_EFF * vy - K_EFF * y + gyro * vx) / M_EFF
    return (vx, vy, ax, ay)


def add_scaled(state, slope, scale):
    return tuple(state[i] + scale * slope[i] for i in range(4))


def rk4_step(time, state, step):
    k1 = rhs(time, state)
    k2 = rhs(time + 0.5 * step, add_scaled(state, k1, 0.5 * step))
    k3 = rhs(time + 0.5 * step, add_scaled(state, k2, 0.5 * step))
    k4 = rhs(time + step, add_scaled(state, k3, step))
    return tuple(state[i] + step * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0 for i in range(4))


def compute_samples(duration=END_TIME, step=STEP):
    time = 0.0
    state = (0.0, 0.0, 0.0, 0.0)
    samples = [(time, state)]
    while time < duration - 1.0e-14:
        h = min(step, duration - time)
        state = rk4_step(time, state, h)
        time += h
        samples.append((time, state))
    return samples


SAMPLES = compute_samples()


def sample_state(time):
    if time <= SAMPLES[0][0]:
        return SAMPLES[0][1]
    if time >= SAMPLES[-1][0]:
        return SAMPLES[-1][1]
    index = min(int(time / STEP), len(SAMPLES) - 2)
    while SAMPLES[index + 1][0] < time and index < len(SAMPLES) - 2:
        index += 1
    t0, s0 = SAMPLES[index]
    t1, s1 = SAMPLES[index + 1]
    alpha = (time - t0) / (t1 - t0)
    return tuple((1.0 - alpha) * s0[i] + alpha * s1[i] for i in range(4))


def mode_shape(z):
    return 0.20 + 0.80 * math.sin(math.pi * z / ROTOR_LENGTH)


def node_position(z, state):
    x, y, _vx, _vy = state
    shape = mode_shape(z)
    return (DISPLAY_SCALE * shape * x, DISPLAY_SCALE * shape * y, z)


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


class MutablePolyline:
    def __init__(self, system, name, tint, thickness=3):
        self.body = chrono.ChBody()
        self.body.SetName(name)
        self.body.SetFixed(True)
        self.body.EnableCollision(False)
        self.line = chrono.ChLinePoly(TRACE_POINTS)
        self.shape = chrono.ChVisualShapeLine()
        self.shape.SetMutable(True)
        self.shape.SetLineGeometry(self.line)
        self.shape.SetColor(tint)
        self.shape.SetThickness(thickness)
        self.body.AddVisualShape(self.shape)
        system.AddBody(self.body)

    def update(self, points):
        if not points:
            points = [vec(0, 0, 0)]
        for i in range(TRACE_POINTS):
            index = min(len(points) - 1, int(i * len(points) / TRACE_POINTS))
            self.line.SetPoint(i, points[index])
        self.shape.SetLineGeometry(self.line)
        self.body.UpdateVisualModel()


class ForceArrow:
    def __init__(self, system):
        self.main = MutableSegment(system, "ObjectGenericODE2Test swept unbalance force", color(0.92, 0.55, 0.05), 5)
        self.tip_a = MutableSegment(system, "ObjectGenericODE2Test force arrow tip a", color(0.92, 0.55, 0.05), 4)
        self.tip_b = MutableSegment(system, "ObjectGenericODE2Test force arrow tip b", color(0.92, 0.55, 0.05), 4)

    def update(self, start, force_xy):
        scale = 0.0010
        force = (force_xy[0] * scale, force_xy[1] * scale, 0.0)
        end = vadd(start, force)
        self.main.update(vec(*start), vec(*end))
        direction = vsub(end, start)
        length = max(vnorm(direction), 1.0e-12)
        unit = vscale(direction, 1.0 / length)
        side = (-unit[1], unit[0], 0.0)
        self.tip_a.update(vec(*end), vec(*vadd(vadd(end, vscale(unit, -0.055)), vscale(side, 0.035))))
        self.tip_b.update(vec(*end), vec(*vadd(vadd(end, vscale(unit, -0.055)), vscale(side, -0.035))))


def make_body(system, name, radius, tint):
    body = chrono.ChBodyEasySphere(radius, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def make_support_spring(system, name, anchor_pos, rotor_body, rotor_base_pos, rotor_local):
    anchor = make_body(system, name + " anchor", 0.018, color(0.05, 0.05, 0.055))
    anchor.SetPos(vec(*anchor_pos))
    spring = chrono.ChLinkTSDA()
    spring.SetName(name + " native ChVisualShapeSpring")
    spring.Initialize(rotor_body, anchor, True, vec(*rotor_local), vec(0, 0, 0))
    spring.SetRestLength(vnorm(vsub(vadd(rotor_base_pos, rotor_local), anchor_pos)))
    spring.SetSpringCoefficient(0.0)
    spring.SetDampingCoefficient(0.0)
    system.AddLink(spring)
    shape = chrono.ChVisualShapeSpring(0.025, 90, 10)
    shape.SetColor(color(0.86, 0.16, 0.10))
    spring.AddVisualShape(shape)
    attach_spring_visual(system, spring, 0.025, 90, 10, color(0.86, 0.16, 0.10))
    return {"anchor": anchor, "spring": spring, "local": rotor_local}


def make_ground(system):
    ground = chrono.ChBody()
    ground.SetName("ObjectGenericODE2Test ground and reference axes")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    plate = chrono.ChVisualShapeBox(1.20, 1.10, 0.020)
    plate.SetColor(color(0.78, 0.78, 0.74))
    plate.SetOpacity(0.22)
    ground.AddVisualShape(plate, chrono.ChFramed(vec(0.0, 0.0, 0.25)))
    axis = chrono.ChVisualShapeCylinder(0.008, ROTOR_LENGTH + 0.08)
    axis.SetColor(color(0.12, 0.12, 0.14))
    ground.AddVisualShape(axis, chrono.ChFramed(vec(0, 0, 0.25)))
    system.AddBody(ground)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    make_ground(system)

    nodes = [make_body(system, f"ObjectGenericODE2Test rotor node {i}", 0.026, color(0.15, 0.34, 0.86)) for i in range(len(NODE_Z))]
    for body, z in zip(nodes, NODE_Z):
        body.SetPos(vec(0, 0, z))
    disc = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.12, 0.035, 1000, True, False)
    disc.SetName("ObjectGenericODE2Test unbalance disc at z=0.15")
    disc.SetFixed(True)
    disc.EnableCollision(False)
    disc.GetVisualShape(0).SetColor(color(0.86, 0.28, 0.12))
    system.AddBody(disc)

    shaft_segments = [
        MutableSegment(system, f"ObjectGenericODE2Test rotor shaft segment {i}", color(0.12, 0.12, 0.14), 6)
        for i in range(len(NODE_Z) - 1)
    ]

    support_springs = []
    for z in (NODE_Z[0], NODE_Z[-1]):
        node_index = 0 if z == NODE_Z[0] else -1
        base_pos = (0.0, 0.0, z)
        support_springs.append(make_support_spring(system, f"ObjectGenericODE2Test z={z:.3f} x-support", (-0.18, 0.0, z), nodes[node_index], base_pos, (0, 0, 0)))
        support_springs.append(make_support_spring(system, f"ObjectGenericODE2Test z={z:.3f} y-support", (0.0, -0.18, z), nodes[node_index], base_pos, (0, 0, 0)))

    force_arrow = ForceArrow(system)
    trace = MutablePolyline(system, "ObjectGenericODE2Test mid-node displacement trace", color(0.92, 0.48, 0.08), 3)
    items = {
        "nodes": nodes,
        "disc": disc,
        "shaft_segments": shaft_segments,
        "support_springs": support_springs,
        "force_arrow": force_arrow,
        "trace": trace,
    }
    system._object_generic_ode2_test_items = items
    update_visuals(system)
    return system, items


def update_visuals(system):
    time = min(system.GetChTime(), END_TIME)
    state = sample_state(time)
    items = system._object_generic_ode2_test_items
    positions = [node_position(z, state) for z in NODE_Z]
    for body, pos in zip(items["nodes"], positions):
        body.SetPos(vec(*pos))
        body.UpdateVisualModel()
    force_pos = node_position(FORCE_NODE_Z, state)
    items["disc"].SetPos(vec(*force_pos))
    items["disc"].UpdateVisualModel()
    for segment, a, b in zip(items["shaft_segments"], positions[:-1], positions[1:]):
        segment.update(vec(*a), vec(*b))
    items["force_arrow"].update(force_pos, unbalance_force(time))
    trace_points = [vec(*node_position(0.25, sample_state(t))) for t, _state in SAMPLES if t <= time + 1.0e-14]
    items["trace"].update(trace_points)
    update_system_visuals(system)


def simulate(duration, step):
    del step
    system, _items = build_system()
    system.SetChTime(min(duration, END_TIME))
    update_visuals(system)
    return system


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: objectGenericODE2Test.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.85, -1.45, 0.85), chrono.ChVector3d(0.0, 0.0, 0.25))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def print_state(duration):
    state = sample_state(min(duration, END_TIME))
    x, y, vx, vy = state
    freq_error = REFERENCE_FREQ_HZ / 57.63178639764625 - 1.0
    source_style = 1.0e-2 * freq_error + x
    fx, fy = unbalance_force(min(duration, END_TIME))
    print("EXUDYN port: objectGenericODE2Test.py -> PyChrono reduced GenericODE2 rotor replay")
    print(
        f"t={min(duration, END_TIME):6.3f}  first_mode_hz={REFERENCE_FREQ_HZ:.9f}  "
        f"mid_disp=({x:+.12e},{y:+.12e})  mid_vel=({vx:+.12e},{vy:+.12e})"
    )
    print(
        f"sweep_frequency={frequency_sweep(min(duration, END_TIME)):.6f}  "
        f"unbalance_force=({fx:+.6f},{fy:+.6f})  source_style_result={source_style:+.12e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()
    if args.no_vis:
        simulate(args.duration, args.step)
        print_state(args.duration)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

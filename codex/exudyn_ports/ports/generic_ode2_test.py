import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces EXUDYN TestModels/genericODE2test.py.
# The source compares a regular CartesianSpringDamper mass point with an
# ObjectGenericODE2 node using diagonal mass/stiffness/damping matrices and a
# swept z-direction force user function.  This port uses the same trapezoidal
# Newmark update (spectralRadius=1) for the scalar equations and renders both
# systems with explicit mass bodies and coil spring visuals.

LENGTH = 0.5
MASS = 1.6
STIFFNESS = 4000.0
DAMPING = 0.05 * 2.0 * 80.0
U0 = -0.08
V0 = 1.0
FORCE = 80.0
FDYN_Z = 10.0
END_TIME = 1.0
SOURCE_STEPS = 2000
STEP = END_TIME / SOURCE_STEPS
REFERENCE_RESULT = 0.03604546349898683
GENERIC_VIS_SCALE = 8.0


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def sweep(time, duration=10.0, freq0=1.0, freq1=100.0):
    slope = (freq1 - freq0) / duration
    return math.sin(2.0 * math.pi * (freq0 + 0.5 * slope * time) * time)


def newmark_scalar(q, v, a, load_next, damping, step):
    beta = 0.25
    gamma = 0.5
    q_predict = q + step * v + step * step * (0.5 - beta) * a
    v_predict = v + step * (1.0 - gamma) * a
    denominator = MASS + damping * gamma * step + STIFFNESS * beta * step * step
    a_next = (load_next - damping * v_predict - STIFFNESS * q_predict) / denominator
    q_next = q_predict + beta * step * step * a_next
    v_next = v_predict + gamma * step * a_next
    return q_next, v_next, a_next


def initial_acceleration(q, v, load, damping):
    return (load - damping * v - STIFFNESS * q) / MASS


def initial_state():
    return {
        "cart": [U0, V0, initial_acceleration(U0, V0, FORCE, DAMPING)],
        "gen_x": [U0, V0, initial_acceleration(U0, V0, FORCE, DAMPING)],
        "gen_y": [FORCE / STIFFNESS * 0.999, 0.0, initial_acceleration(FORCE / STIFFNESS * 0.999, 0.0, FORCE, 0.0)],
        "gen_z": [0.0, 0.0, initial_acceleration(0.0, 0.0, 0.0, DAMPING)],
        "time": 0.0,
    }


def advance_state(state, target_time, step=STEP):
    while state["time"] < target_time - 1.0e-12:
        dt = min(step, target_time - state["time"])
        next_time = state["time"] + dt
        state["cart"][:] = newmark_scalar(*state["cart"], FORCE, DAMPING, dt)
        state["gen_x"][:] = newmark_scalar(*state["gen_x"], FORCE, DAMPING, dt)
        state["gen_y"][:] = newmark_scalar(*state["gen_y"], FORCE, 0.0, dt)
        state["gen_z"][:] = newmark_scalar(*state["gen_z"], FDYN_Z * sweep(next_time), DAMPING, dt)
        state["time"] = next_time
    return state


def solve(duration=END_TIME, step=STEP):
    return advance_state(initial_state(), duration, step)


def source_norm(state):
    u1 = abs(state["cart"][0])
    qx = state["gen_x"][0]
    qy = state["gen_y"][0]
    qz = state["gen_z"][0]
    u2 = math.sqrt(qx * qx + qy * qy + qz * qz)
    return u1 + u2


def cart_position(state):
    return chrono.ChVector3d(LENGTH + state["cart"][0], -0.38, 0.0)


def generic_reference():
    return chrono.ChVector3d(2.0 * LENGTH, 0.42, 0.0)


def generic_position(state):
    qx = state["gen_x"][0]
    qy = state["gen_y"][0]
    qz = state["gen_z"][0]
    ref = generic_reference()
    return chrono.ChVector3d(ref.x + GENERIC_VIS_SCALE * qx, ref.y + GENERIC_VIS_SCALE * qy, GENERIC_VIS_SCALE * qz)


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

    def update(self, point_a, point_b):
        self.shape.SetLineGeometry(chrono.ChLineSegment(point_a, point_b))
        self.body.UpdateVisualModel()


class Arrow:
    def __init__(self, system, name, tint):
        self.main = MutableSegment(system, name, tint, 5)
        self.tip_a = MutableSegment(system, name + " tip a", tint, 4)
        self.tip_b = MutableSegment(system, name + " tip b", tint, 4)

    def update_x(self, start, span):
        end = start + chrono.ChVector3d(span, 0.0, 0.0)
        direction = 1.0 if span >= 0.0 else -1.0
        self.main.update(start, end)
        self.tip_a.update(end, end + chrono.ChVector3d(-0.055 * direction, 0.030, 0.0))
        self.tip_b.update(end, end + chrono.ChVector3d(-0.055 * direction, -0.030, 0.0))

    def update_z(self, start, span):
        end = start + chrono.ChVector3d(0.0, 0.0, span)
        direction = 1.0 if span >= 0.0 else -1.0
        self.main.update(start, end)
        self.tip_a.update(end, end + chrono.ChVector3d(0.030, 0.0, -0.055 * direction))
        self.tip_b.update(end, end + chrono.ChVector3d(-0.030, 0.0, -0.055 * direction))


def make_body(system, name, radius, tint, pos):
    body = chrono.ChBodyEasySphere(radius, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetMass(MASS)
    body.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    body.SetPos(pos)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def add_visual_spring(system, name, body_a, body_b, rest_length, radius=0.035):
    spring = chrono.ChLinkTSDA()
    spring.SetName(name)
    spring.Initialize(body_b, body_a, True, chrono.ChVector3d(0.0, 0.0, 0.0), chrono.ChVector3d(0.0, 0.0, 0.0))
    spring.SetRestLength(rest_length)
    spring.SetSpringCoefficient(0.0)
    spring.SetDampingCoefficient(0.0)
    system.AddLink(spring)
    spring_shape = chrono.ChVisualShapeSpring(radius, 100, 13)
    spring_shape.SetColor(color(0.86, 0.16, 0.10))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, spring, radius, 100, 13, color(0.86, 0.16, 0.10))
    return spring


def make_ground(system):
    ground = chrono.ChBody()
    ground.SetName("genericODE2test ground and guides")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    for row_y in (-0.38, 0.42):
        rail = chrono.ChVisualShapeBox(1.95, 0.018, 0.018)
        rail.SetColor(color(0.42, 0.42, 0.42))
        ground.AddVisualShape(rail, chrono.ChFramed(chrono.ChVector3d(0.55, row_y - 0.15, 0.0)))

    static_marker = chrono.ChVisualShapeBox(0.014, 0.20, 0.018)
    static_marker.SetColor(color(0.08, 0.08, 0.08))
    ground.AddVisualShape(static_marker, chrono.ChFramed(chrono.ChVector3d(LENGTH + FORCE / STIFFNESS, -0.53, 0.0)))

    generic_static = chrono.ChVisualShapeBox(0.014, 0.20, 0.018)
    generic_static.SetColor(color(0.08, 0.08, 0.08))
    ground.AddVisualShape(
        generic_static,
        chrono.ChFramed(chrono.ChVector3d(2.0 * LENGTH + GENERIC_VIS_SCALE * FORCE / STIFFNESS, 0.27, 0.0)),
    )

    system.AddBody(ground)
    return ground


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))
    make_ground(system)

    state = initial_state()

    cart_anchor = make_body(system, "genericODE2test CartesianSpringDamper anchor", 0.035, color(0.08, 0.08, 0.08), chrono.ChVector3d(0.0, -0.38, 0.0))
    cart_mass = make_body(system, "genericODE2test CartesianSpringDamper mass", 0.065, color(0.12, 0.38, 0.88), cart_position(state))
    cart_spring = add_visual_spring(system, "genericODE2test CartesianSpringDamper coil", cart_anchor, cart_mass, LENGTH)

    generic_anchor = make_body(system, "genericODE2test GenericODE2 reference node", 0.035, color(0.08, 0.08, 0.08), generic_reference())
    generic_mass = make_body(system, "genericODE2test GenericODE2 visual node", 0.070, color(0.12, 0.58, 0.28), generic_position(state))
    generic_spring = add_visual_spring(system, "genericODE2test GenericODE2 stiffness coil", generic_anchor, generic_mass, 0.0)

    cart_load = Arrow(system, "genericODE2test Cartesian constant load", color(0.92, 0.55, 0.05))
    generic_load = Arrow(system, "genericODE2test GenericODE2 swept z-load", color(0.92, 0.55, 0.05))

    items = {
        "state": state,
        "cart_anchor": cart_anchor,
        "cart_mass": cart_mass,
        "cart_spring": cart_spring,
        "generic_anchor": generic_anchor,
        "generic_mass": generic_mass,
        "generic_spring": generic_spring,
        "cart_load": cart_load,
        "generic_load": generic_load,
    }
    system._generic_ode2_test_items = items
    update_visuals(system)
    return system, items


def set_visual_state(items, state):
    items["cart_mass"].SetPos(cart_position(state))
    items["cart_mass"].SetPosDt(chrono.ChVector3d(state["cart"][1], 0.0, 0.0))
    items["cart_mass"].UpdateVisualModel()

    items["generic_mass"].SetPos(generic_position(state))
    items["generic_mass"].SetPosDt(
        chrono.ChVector3d(GENERIC_VIS_SCALE * state["gen_x"][1], GENERIC_VIS_SCALE * state["gen_y"][1], GENERIC_VIS_SCALE * state["gen_z"][1])
    )
    items["generic_mass"].UpdateVisualModel()

    items["cart_load"].update_x(items["cart_mass"].GetPos() + chrono.ChVector3d(0.0, 0.14, 0.0), 0.28)
    z_load = FDYN_Z * sweep(state["time"])
    items["generic_load"].update_z(items["generic_mass"].GetPos() + chrono.ChVector3d(0.0, 0.10, 0.0), 0.025 * z_load)


def update_visuals(system):
    items = system._generic_ode2_test_items
    advance_state(items["state"], system.GetChTime(), STEP)
    set_visual_state(items, items["state"])
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
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: genericODE2test.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.70, -2.30, 1.40), chrono.ChVector3d(0.65, 0.02, 0.0))
    vis.AddTypicalLights()

    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def print_result(duration, step):
    state = solve(duration, step)
    result = source_norm(state)
    print(
        f"generic_ode2_test: t={duration:7.3f}  "
        f"u1=({state['cart'][0]:+.12f},0,0)  "
        f"u2=({state['gen_x'][0]:+.12f},{state['gen_y'][0]:+.12f},{state['gen_z'][0]:+.12f})  "
        f"v2=({state['gen_x'][1]:+.12f},{state['gen_y'][1]:+.12f},{state['gen_z'][1]:+.12f})  "
        f"result={result:.15f}  reference_error={result - REFERENCE_RESULT:+.3e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: genericODE2test.py -> PyChrono GenericODE2 matrix/user-force test")
    if args.no_vis:
        print_result(args.duration, args.step)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

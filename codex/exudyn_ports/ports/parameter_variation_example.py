import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN Examples/parameterVariationExample.py:
# a 1D mass-spring-damper is solved for a reference parameter set, then mass
# and stiffness are varied over coarse and refined grids and each run is scored
# by its force-history error against the reference.  The PyChrono scene shows
# the nominal oscillator with a real coil spring and a fixed 3D error surface.

LENGTH = 0.5
MASS = 1.6
STIFFNESS = 4000.0
DAMPING = 8.0
INITIAL_DISPLACEMENT = -0.08
INITIAL_VELOCITY = 1.0
FORCE = 80.0
T_END = 1.0
SENSOR_STEP = 5e-3
STEPS = 1000
GRID_SIZE = 16
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def lerp(a, b, t):
    return a + (b - a) * t


def parameter_values(start, stop, count):
    if count == 1:
        return [start]
    return [start + (stop - start) * i / (count - 1) for i in range(count)]


def exact_displacement_velocity(t, mass, stiffness, damping, force, u0, v0):
    omega0 = math.sqrt(stiffness / mass)
    zeta = damping / (2.0 * math.sqrt(stiffness * mass))
    static = force / stiffness
    c1 = u0 - static
    if zeta < 1.0:
        omega = omega0 * math.sqrt(1.0 - zeta * zeta)
        c2 = (v0 + omega0 * zeta * c1) / omega
        exp_term = math.exp(-omega0 * zeta * t)
        cos_term = math.cos(omega * t)
        sin_term = math.sin(omega * t)
        u = exp_term * (c1 * cos_term + c2 * sin_term) + static
        v = exp_term * (
            -omega0 * zeta * (c1 * cos_term + c2 * sin_term)
            + (-c1 * omega * sin_term + c2 * omega * cos_term)
        )
        return u, v

    # This parameter range is underdamped, but keep a stable fallback.
    exp_term = math.exp(-omega0 * t)
    c2 = v0 + omega0 * c1
    u = exp_term * (c1 + c2 * t) + static
    v = exp_term * (c2 - omega0 * (c1 + c2 * t))
    return u, v


def connector_force(t, mass, stiffness, damping, force=FORCE, u0=INITIAL_DISPLACEMENT, v0=INITIAL_VELOCITY):
    u, v = exact_displacement_velocity(t, mass, stiffness, damping, force, u0, v0)
    return stiffness * u + damping * v


def force_history(mass, stiffness, damping=DAMPING):
    count = int(T_END / SENSOR_STEP) + 1
    return [connector_force(i * SENSOR_STEP, mass, stiffness, damping) for i in range(count)]


def error_against_reference(mass, stiffness, reference):
    values = force_history(mass, stiffness)
    diff2 = sum((value - ref) ** 2 for value, ref in zip(values, reference))
    return math.sqrt(diff2) / STEPS * T_END


def parameter_variation(grid_size=GRID_SIZE):
    reference = force_history(MASS, STIFFNESS)

    coarse_masses = parameter_values(1.0, 2.0, grid_size)
    coarse_springs = parameter_values(2000.0, 8000.0, grid_size)
    coarse = []
    for mass in coarse_masses:
        for stiffness in coarse_springs:
            coarse.append((mass, stiffness, error_against_reference(mass, stiffness, reference)))

    refined_masses = parameter_values(1.5, 1.7, grid_size)
    refined_springs = parameter_values(3000.0, 5000.0, grid_size)
    refined = []
    for mass in refined_masses:
        for stiffness in refined_springs:
            refined.append((mass, stiffness, error_against_reference(mass, stiffness, reference)))

    return coarse, refined


def tint_from_value(value, max_value):
    s = 0.0 if max_value <= 0 else min(1.0, value / max_value)
    return color(lerp(0.08, 0.90, s), lerp(0.35, 0.08, s), lerp(0.85, 0.12, s))


def add_error_surface(system, coarse, refined):
    all_values = [row[2] for row in coarse + refined]
    max_value = max(all_values) if all_values else 1.0
    scale_z = 0.70 / max_value if max_value > 0 else 1.0

    base = chrono.ChBodyEasyBox(1.45, 0.95, 0.025, 1000, True, False)
    base.SetName("parameter variation error surface base")
    base.SetFixed(True)
    base.SetPos(chrono.ChVector3d(0.78, 0.92, -0.18))
    base.GetVisualShape(0).SetColor(color(0.20, 0.22, 0.24))
    base.GetVisualShape(0).SetOpacity(0.28)
    system.AddBody(base)

    dots = []
    for mass, stiffness, error in coarse:
        x = 0.20 + 1.15 * (mass - 1.0) / (2.0 - 1.0)
        y = 0.55 + 0.58 * (stiffness - 2000.0) / (8000.0 - 2000.0)
        z = -0.14 + error * scale_z
        dot = chrono.ChBodyEasySphere(0.018, 1000, True, False)
        dot.SetName("coarse parameter-error sample")
        dot.SetFixed(True)
        dot.SetPos(chrono.ChVector3d(x, y, z))
        dot.GetVisualShape(0).SetColor(tint_from_value(error, max_value))
        system.AddBody(dot)
        dots.append(dot)

    for mass, stiffness, error in refined:
        x = 0.20 + 1.15 * (mass - 1.0) / (2.0 - 1.0)
        y = 0.55 + 0.58 * (stiffness - 2000.0) / (8000.0 - 2000.0)
        z = -0.08 + error * scale_z
        dot = chrono.ChBodyEasySphere(0.011, 1000, True, False)
        dot.SetName("refined parameter-error sample")
        dot.SetFixed(True)
        dot.SetPos(chrono.ChVector3d(x, y, z))
        dot.GetVisualShape(0).SetColor(color(0.96, 0.78, 0.08))
        system.AddBody(dot)
        dots.append(dot)

    return base, dots


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    ground = chrono.ChBody()
    ground.SetName("parameter-variation hidden ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)

    anchor = chrono.ChBodyEasySphere(0.045, 1000, True, False)
    anchor.SetName("parameter-variation spring anchor")
    anchor.SetFixed(True)
    anchor.SetPos(chrono.ChVector3d(0, 0, 0))
    anchor.GetVisualShape(0).SetColor(color(0.10, 0.10, 0.10))
    system.AddBody(anchor)

    mass = chrono.ChBodyEasySphere(0.070, 1000, True, False)
    mass.SetName("parameter-variation oscillator mass")
    mass.SetMass(MASS)
    mass.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    mass.SetPos(chrono.ChVector3d(LENGTH + INITIAL_DISPLACEMENT, 0, 0))
    mass.SetPosDt(chrono.ChVector3d(INITIAL_VELOCITY, 0, 0))
    mass.GetVisualShape(0).SetColor(color(0.08, 0.38, 0.86))
    system.AddBody(mass)

    slider = chrono.ChLinkLockPrismatic()
    slider.SetName("parameter-variation x guide")
    slider.Initialize(mass, ground, chrono.ChFramed(chrono.ChVector3d(LENGTH, 0, 0), chrono.Q_ROTATE_Z_TO_X))
    system.AddLink(slider)

    spring = chrono.ChLinkTSDA()
    spring.SetName("parameter-variation nominal spring-damper")
    spring.Initialize(mass, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
    spring.SetRestLength(LENGTH)
    spring.SetSpringCoefficient(STIFFNESS)
    spring.SetDampingCoefficient(DAMPING)
    system.AddLink(spring)

    spring_shape = chrono.ChVisualShapeSpring(0.055, 100, 15)
    spring_shape.SetColor(color(0.88, 0.18, 0.08))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, spring, 0.055, 100, 15, color(0.88, 0.18, 0.08))

    rail = chrono.ChBodyEasyBox(1.15, 0.018, 0.018, 1000, True, False)
    rail.SetName("parameter-variation visible guide rail")
    rail.SetFixed(True)
    rail.SetPos(chrono.ChVector3d(0.55, -0.16, 0))
    rail.GetVisualShape(0).SetColor(color(0.45, 0.45, 0.45))
    system.AddBody(rail)

    force = chrono.ChForce()
    force.SetF_x(chrono.ChFunctionConst(FORCE))
    mass.AddForce(force)

    coarse, refined = parameter_variation(GRID_SIZE)
    base, dots = add_error_surface(system, coarse, refined)

    system._parameter_variation_items = {
        "mass": mass,
        "spring": spring,
        "coarse": coarse,
        "refined": refined,
        "surface_base": base,
        "surface_dots": dots,
    }
    return system, mass, spring, coarse, refined


def update_visuals(system):
    update_system_visuals(system)


def simulate(duration, step):
    system, mass, spring, coarse, refined = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    return system, mass, spring, coarse, refined


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, mass, spring, coarse, refined = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: parameterVariationExample.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.15, 1.55, 1.95), chrono.ChVector3d(0.65, 0.45, 0.02))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, mass, spring, coarse, refined)
            next_log += 0.25


def print_state(system, mass, spring, coarse, refined):
    displacement = mass.GetPos().x - LENGTH
    reference, _ = exact_displacement_velocity(system.GetChTime(), MASS, STIFFNESS, DAMPING, FORCE, INITIAL_DISPLACEMENT, INITIAL_VELOCITY)
    print(
        f"t={system.GetChTime():6.3f}  "
        f"u={displacement:+.8f}  exact={reference:+.8f}  "
        f"coarse_last={coarse[-1][2]:+.6f}  refined_last={refined[-1][2]:+.6f}  "
        f"samples={len(coarse) + len(refined)}  spring_force={spring.GetForce():+.3f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=T_END)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: parameterVariationExample.py -> PyChrono mass-spring-damper parameter sweep")
    if args.no_vis:
        system, mass, spring, coarse, refined = simulate(args.duration, args.step)
        print_state(system, mass, spring, coarse, refined)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

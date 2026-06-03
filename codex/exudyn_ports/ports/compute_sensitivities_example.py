import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN Examples/ComputeSensitivitiesExample.py:
# a 1D mass-spring-damper is evaluated at a reference mass/stiffness and finite
# differences are used to estimate sensitivities of the average absolute spring
# force and the static displacement.  The PyChrono scene shows the reference
# oscillator with a real coil spring and fixed bars for the sensitivity matrix.

LENGTH = 0.5
DEFAULT_MASS = 1.6
REFERENCE_MASS = 1.5
REFERENCE_STIFFNESS = 4000.0
DAMPING = 8.0
INITIAL_DISPLACEMENT = -0.08
INITIAL_VELOCITY = 1.0
FORCE = 80.0
T_END = 1.0
SENSOR_STEP = 5e-3
MASS_EPS = 1e-3
STIFFNESS_EPS = 1.5e-3
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


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

    exp_term = math.exp(-omega0 * t)
    c2 = v0 + omega0 * c1
    u = exp_term * (c1 + c2 * t) + static
    v = exp_term * (c2 - omega0 * (c1 + c2 * t))
    return u, v


def spring_force(t, mass, stiffness):
    u, v = exact_displacement_velocity(
        t,
        mass,
        stiffness,
        DAMPING,
        FORCE,
        INITIAL_DISPLACEMENT,
        INITIAL_VELOCITY,
    )
    return stiffness * u + DAMPING * v


def outputs(mass, stiffness):
    samples = int(T_END / SENSOR_STEP) + 1
    avg_abs_force = sum(abs(spring_force(i * SENSOR_STEP, mass, stiffness)) for i in range(samples)) / samples
    static_displacement = FORCE / stiffness
    return avg_abs_force, static_displacement


def central_difference(parameter, delta):
    if parameter == "mass":
        high = outputs(REFERENCE_MASS + delta, REFERENCE_STIFFNESS)
        low = outputs(REFERENCE_MASS - delta, REFERENCE_STIFFNESS)
    elif parameter == "spring":
        high = outputs(REFERENCE_MASS, REFERENCE_STIFFNESS + delta)
        low = outputs(REFERENCE_MASS, REFERENCE_STIFFNESS - delta)
    else:
        raise ValueError(parameter)
    return tuple((h - l) / (2.0 * delta) for h, l in zip(high, low))


def compute_sensitivities():
    value_reference = outputs(REFERENCE_MASS, REFERENCE_STIFFNESS)
    sensitivity = {
        "mass": central_difference("mass", MASS_EPS),
        "spring": central_difference("spring", STIFFNESS_EPS),
    }
    return value_reference, sensitivity


def make_bar(system, name, origin, value, scale, tint):
    height = max(0.012, abs(value) * scale)
    center_z = origin.z + 0.5 * height * (1.0 if value >= 0 else -1.0)
    bar = chrono.ChBodyEasyBox(0.10, 0.10, height, 1000, True, False)
    bar.SetName(name)
    bar.SetFixed(True)
    bar.SetPos(chrono.ChVector3d(origin.x, origin.y, center_z))
    bar.GetVisualShape(0).SetColor(tint)
    system.AddBody(bar)
    return bar


def add_sensitivity_visuals(system, sensitivity):
    plate = chrono.ChBodyEasyBox(1.35, 0.75, 0.025, 1000, True, False)
    plate.SetName("sensitivity matrix reference plate")
    plate.SetFixed(True)
    plate.SetPos(chrono.ChVector3d(0.85, 0.83, -0.18))
    plate.GetVisualShape(0).SetColor(color(0.20, 0.22, 0.24))
    plate.GetVisualShape(0).SetOpacity(0.30)
    system.AddBody(plate)

    values = [
        sensitivity["mass"][0],
        sensitivity["spring"][0],
        sensitivity["mass"][1],
        sensitivity["spring"][1],
    ]
    max_value = max(abs(v) for v in values) or 1.0
    scale = 0.62 / max_value
    origins = [
        chrono.ChVector3d(0.45, 0.68, -0.16),
        chrono.ChVector3d(0.72, 0.68, -0.16),
        chrono.ChVector3d(0.98, 0.68, -0.16),
        chrono.ChVector3d(1.25, 0.68, -0.16),
    ]
    bars = []
    for i, (origin, value) in enumerate(zip(origins, values)):
        tint = color(0.90, 0.18, 0.10) if value >= 0 else color(0.10, 0.35, 0.88)
        bars.append(make_bar(system, f"sensitivity bar {i}", origin, value, scale, tint))
    return plate, bars


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    ground = chrono.ChBody()
    ground.SetName("sensitivity hidden ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)

    anchor = chrono.ChBodyEasySphere(0.045, 1000, True, False)
    anchor.SetName("sensitivity spring anchor")
    anchor.SetFixed(True)
    anchor.SetPos(chrono.ChVector3d(0, 0, 0))
    anchor.GetVisualShape(0).SetColor(color(0.10, 0.10, 0.10))
    system.AddBody(anchor)

    mass = chrono.ChBodyEasySphere(0.070, 1000, True, False)
    mass.SetName("sensitivity reference mass")
    mass.SetMass(REFERENCE_MASS)
    mass.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    mass.SetPos(chrono.ChVector3d(LENGTH + INITIAL_DISPLACEMENT, 0, 0))
    mass.SetPosDt(chrono.ChVector3d(INITIAL_VELOCITY, 0, 0))
    mass.GetVisualShape(0).SetColor(color(0.08, 0.38, 0.86))
    system.AddBody(mass)

    slider = chrono.ChLinkLockPrismatic()
    slider.SetName("sensitivity x guide")
    slider.Initialize(mass, ground, chrono.ChFramed(chrono.ChVector3d(LENGTH, 0, 0), chrono.Q_ROTATE_Z_TO_X))
    system.AddLink(slider)

    spring = chrono.ChLinkTSDA()
    spring.SetName("sensitivity nominal spring-damper")
    spring.Initialize(mass, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
    spring.SetRestLength(LENGTH)
    spring.SetSpringCoefficient(REFERENCE_STIFFNESS)
    spring.SetDampingCoefficient(DAMPING)
    system.AddLink(spring)
    spring_shape = chrono.ChVisualShapeSpring(0.055, 100, 15)
    spring_shape.SetColor(color(0.88, 0.18, 0.08))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, spring, 0.055, 100, 15, color(0.88, 0.18, 0.08))

    rail = chrono.ChBodyEasyBox(1.15, 0.018, 0.018, 1000, True, False)
    rail.SetName("sensitivity visible guide rail")
    rail.SetFixed(True)
    rail.SetPos(chrono.ChVector3d(0.55, -0.16, 0))
    rail.GetVisualShape(0).SetColor(color(0.45, 0.45, 0.45))
    system.AddBody(rail)

    force = chrono.ChForce()
    force.SetF_x(chrono.ChFunctionConst(FORCE))
    mass.AddForce(force)

    values, sensitivity = compute_sensitivities()
    plate, bars = add_sensitivity_visuals(system, sensitivity)
    system._sensitivity_items = {
        "values": values,
        "sensitivity": sensitivity,
        "plate": plate,
        "bars": bars,
    }
    return system, mass, spring, values, sensitivity


def update_visuals(system):
    update_system_visuals(system)


def simulate(duration, step):
    system, mass, spring, values, sensitivity = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    return system, mass, spring, values, sensitivity


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, mass, spring, values, sensitivity = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ComputeSensitivitiesExample.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.15, 1.45, 1.85), chrono.ChVector3d(0.65, 0.42, 0.02))
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
            print_state(system, mass, spring, values, sensitivity)
            next_log += 0.25


def print_state(system, mass, spring, values, sensitivity):
    displacement = mass.GetPos().x - LENGTH
    reference, _ = exact_displacement_velocity(
        system.GetChTime(),
        REFERENCE_MASS,
        REFERENCE_STIFFNESS,
        DAMPING,
        FORCE,
        INITIAL_DISPLACEMENT,
        INITIAL_VELOCITY,
    )
    avg_sensitivity = sum(abs(v) for row in sensitivity.values() for v in row) / 4.0
    print(
        f"t={system.GetChTime():6.3f}  "
        f"u={displacement:+.8f}  exact={reference:+.8f}  "
        f"val_ref=({values[0]:+.6f}, {values[1]:+.6f})  "
        f"d_avgF_dm={sensitivity['mass'][0]:+.6f}  d_avgF_dk={sensitivity['spring'][0]:+.6f}  "
        f"d_x0_dm={sensitivity['mass'][1]:+.6f}  d_x0_dk={sensitivity['spring'][1]:+.6f}  "
        f"avg_abs_sensitivity={avg_sensitivity:+.6f}  spring_force={spring.GetForce():+.3f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=T_END)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ComputeSensitivitiesExample.py -> PyChrono mass-spring-damper sensitivities")
    if args.no_vis:
        system, mass, spring, values, sensitivity = simulate(args.duration, args.step)
        print_state(system, mass, spring, values, sensitivity)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/connectorGravityTest.py:
# four light satellites orbiting a massive central body under Newtonian gravity.

G = 6.6743e-11
MASS_STAR = 1e20
MASS_SATELLITE = 1e3
ORBIT_RADII = [2e5, 4e5, 8e5, 10e5]
VIS_SCALE = 1e-6
STEP = 1000.0
END_TIME = 1e6


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def scaled(pos):
    return chrono.ChVector3d(pos.x * VIS_SCALE, pos.y * VIS_SCALE, pos.z * VIS_SCALE)


def circular_position(radius, time):
    omega = math.sqrt(G * MASS_STAR / radius**3)
    return chrono.ChVector3d(radius * math.cos(omega * time), radius * math.sin(omega * time), 0)


def acceleration(pos):
    radius = max(pos.Length(), 1e-12)
    factor = -G * MASS_STAR / radius**3
    return chrono.ChVector3d(factor * pos.x, factor * pos.y, factor * pos.z)


def rk4_step(pos, vel, dt):
    def rhs(p, v):
        return v, acceleration(p)

    k1_p, k1_v = rhs(pos, vel)
    k2_p, k2_v = rhs(
        chrono.ChVector3d(pos.x + 0.5 * dt * k1_p.x, pos.y + 0.5 * dt * k1_p.y, pos.z + 0.5 * dt * k1_p.z),
        chrono.ChVector3d(vel.x + 0.5 * dt * k1_v.x, vel.y + 0.5 * dt * k1_v.y, vel.z + 0.5 * dt * k1_v.z),
    )
    k3_p, k3_v = rhs(
        chrono.ChVector3d(pos.x + 0.5 * dt * k2_p.x, pos.y + 0.5 * dt * k2_p.y, pos.z + 0.5 * dt * k2_p.z),
        chrono.ChVector3d(vel.x + 0.5 * dt * k2_v.x, vel.y + 0.5 * dt * k2_v.y, vel.z + 0.5 * dt * k2_v.z),
    )
    k4_p, k4_v = rhs(
        chrono.ChVector3d(pos.x + dt * k3_p.x, pos.y + dt * k3_p.y, pos.z + dt * k3_p.z),
        chrono.ChVector3d(vel.x + dt * k3_v.x, vel.y + dt * k3_v.y, vel.z + dt * k3_v.z),
    )

    new_pos = chrono.ChVector3d(
        pos.x + dt * (k1_p.x + 2 * k2_p.x + 2 * k3_p.x + k4_p.x) / 6.0,
        pos.y + dt * (k1_p.y + 2 * k2_p.y + 2 * k3_p.y + k4_p.y) / 6.0,
        pos.z + dt * (k1_p.z + 2 * k2_p.z + 2 * k3_p.z + k4_p.z) / 6.0,
    )
    new_vel = chrono.ChVector3d(
        vel.x + dt * (k1_v.x + 2 * k2_v.x + 2 * k3_v.x + k4_v.x) / 6.0,
        vel.y + dt * (k1_v.y + 2 * k2_v.y + 2 * k3_v.y + k4_v.y) / 6.0,
        vel.z + dt * (k1_v.z + 2 * k2_v.z + 2 * k3_v.z + k4_v.z) / 6.0,
    )
    return new_pos, new_vel


def make_orbit_line(radius):
    line = chrono.ChLinePoly(180)
    scaled_radius = radius * VIS_SCALE
    for i in range(180):
        angle = 2.0 * math.pi * i / 179
        line.SetPoint(i, chrono.ChVector3d(scaled_radius * math.cos(angle), scaled_radius * math.sin(angle), 0))
    return line


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    star = chrono.ChBodyEasySphere(0.10, 1000, True, False)
    star.SetFixed(True)
    star.SetPos(chrono.ChVector3d(0, 0, 0))
    star.GetVisualShape(0).SetColor(color(0.12, 0.35, 0.95))
    system.AddBody(star)

    orbit_body = chrono.ChBody()
    orbit_body.SetFixed(True)
    orbit_body.EnableCollision(False)
    for radius in ORBIT_RADII:
        orbit_shape = chrono.ChVisualShapeLine()
        orbit_shape.SetLineGeometry(make_orbit_line(radius))
        orbit_shape.SetColor(color(0.25, 0.25, 0.25))
        orbit_body.AddVisualShape(orbit_shape)
    system.AddBody(orbit_body)

    satellite_colors = [
        color(0.95, 0.20, 0.20),
        color(0.10, 0.65, 0.25),
        color(0.95, 0.72, 0.05),
        color(0.70, 0.25, 0.95),
    ]
    satellites = []
    for i, radius in enumerate(ORBIT_RADII):
        satellite = chrono.ChBodyEasySphere(0.028 + 0.004 * i, 1000, True, False)
        satellite.SetFixed(True)
        satellite.SetPos(scaled(circular_position(radius, 0)))
        satellite.GetVisualShape(0).SetColor(satellite_colors[i])
        system.AddBody(satellite)
        satellites.append(satellite)

    system._exudyn_port_gravity_satellites = satellites
    return system, star, satellites


def update_visuals(system):
    satellites = getattr(system, "_exudyn_port_gravity_satellites", [])
    for satellite, radius in zip(satellites, ORBIT_RADII):
        satellite.SetPos(scaled(circular_position(radius, system.GetChTime())))


def initial_states():
    states = []
    for radius in ORBIT_RADII:
        speed = math.sqrt(G * MASS_STAR / radius)
        states.append(
            {
                "pos": chrono.ChVector3d(radius, 0, 0),
                "vel": chrono.ChVector3d(0, speed, 0),
            }
        )
    return states


def simulate(duration, step):
    system, star, satellites = build_system()
    states = initial_states()
    while system.GetChTime() < duration:
        dt = min(step, duration - system.GetChTime())
        for state in states:
            state["pos"], state["vel"] = rk4_step(state["pos"], state["vel"], dt)
        system.SetChTime(system.GetChTime() + dt)
    return system, states


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, star, satellites = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: connectorGravityTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.4, 1.8, 1.4), chrono.ChVector3d(0.25, 0.25, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        update_visuals(system)
        vis.Render()
        vis.EndScene()
        system.SetChTime(t + step)
        if t >= next_log:
            print_visual_state(system)
            next_log += duration / 4.0


def print_visual_state(system):
    pos = circular_position(ORBIT_RADII[-1], system.GetChTime())
    print(
        f"t={system.GetChTime():9.1f}  "
        f"outer_exact=({pos.x:+.2f}, {pos.y:+.2f}, {pos.z:+.2f})  "
        f"sum={pos.x + pos.y + pos.z:+.6f}"
    )


def print_state(system, states):
    state = states[-1]
    exact = circular_position(ORBIT_RADII[-1], system.GetChTime())
    pos = state["pos"]
    error = math.sqrt((pos.x - exact.x) ** 2 + (pos.y - exact.y) ** 2 + (pos.z - exact.z) ** 2)
    print(
        f"t={system.GetChTime():9.1f}  "
        f"outer=({pos.x:+.2f}, {pos.y:+.2f}, {pos.z:+.2f})  "
        f"sum={pos.x + pos.y + pos.z:+.6f}  exact_error={error:.6e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: connectorGravityTest.py -> PyChrono visual Newtonian satellite system")
    if args.no_vis:
        system, states = simulate(args.duration, args.step)
        print_state(system, states)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

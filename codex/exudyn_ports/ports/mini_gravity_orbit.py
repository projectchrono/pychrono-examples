import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN
# TestModels/MiniExamples/ObjectConnectorGravity.py:
# a light satellite orbiting a massive body under Newtonian gravity. The massive
# body is effectively fixed at this scale, as in the EXUDYN test result.

MASS0 = 1e25
MASS1 = 1e3
RADIUS = 1e5
G = 6.6743e-11
V_INIT = math.sqrt(G * MASS0 / RADIUS)
T_END = 0.5 * math.pi * RADIUS / V_INIT
STEP = T_END / 5000.0
VIS_SCALE = 1.0 / RADIUS


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def acceleration(pos):
    r = max(pos.Length(), 1e-12)
    return chrono.ChVector3d(
        -G * MASS0 * pos.x / r**3,
        -G * MASS0 * pos.y / r**3,
        -G * MASS0 * pos.z / r**3,
    )


def scaled(pos):
    return chrono.ChVector3d(pos.x * VIS_SCALE, pos.y * VIS_SCALE, pos.z * VIS_SCALE)


def update_visuals(sys):
    satellite = getattr(sys, "_exudyn_port_satellite", None)
    if satellite is None:
        return
    angle = (V_INIT / RADIUS) * sys.GetChTime()
    pos = chrono.ChVector3d(RADIUS * math.cos(angle), RADIUS * math.sin(angle), 0)
    satellite.SetPos(scaled(pos))


def build_system():
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    sun = chrono.ChBodyEasySphere(0.08, 1000, True, False)
    sun.SetFixed(True)
    sun.SetPos(chrono.ChVector3d(0, 0, 0))
    sun.GetVisualShape(0).SetColor(color(0.95, 0.65, 0.08))
    sys.AddBody(sun)

    satellite = chrono.ChBodyEasySphere(0.035, 1000, True, False)
    satellite.SetPos(scaled(chrono.ChVector3d(RADIUS, 0, 0)))
    satellite.GetVisualShape(0).SetColor(color(0.12, 0.35, 0.90))
    sys.AddBody(satellite)

    orbit = chrono.ChBody()
    orbit.SetFixed(True)
    orbit.EnableCollision(False)
    orbit_line = chrono.ChLinePoly(160)
    for i in range(160):
        angle = 2.0 * math.pi * i / 159
        orbit_line.SetPoint(i, chrono.ChVector3d(math.cos(angle), math.sin(angle), 0))
    orbit_shape = chrono.ChVisualShapeLine()
    orbit_shape.SetLineGeometry(orbit_line)
    orbit_shape.SetColor(color(0.25, 0.25, 0.25))
    orbit.AddVisualShape(orbit_shape)
    sys.AddBody(orbit)

    state = {
        "pos": chrono.ChVector3d(RADIUS, 0, 0),
        "vel": chrono.ChVector3d(0, V_INIT, 0),
    }
    sys._exudyn_port_satellite = satellite
    return sys, sun, satellite, state


def step_orbit(state, dt):
    pos = state["pos"]
    vel = state["vel"]
    acc = acceleration(pos)
    vel = chrono.ChVector3d(vel.x + acc.x * dt, vel.y + acc.y * dt, vel.z + acc.z * dt)
    pos = chrono.ChVector3d(pos.x + vel.x * dt, pos.y + vel.y * dt, pos.z + vel.z * dt)
    state["pos"] = pos
    state["vel"] = vel


def simulate(duration, step):
    sys, sun, satellite, state = build_system()
    while sys.GetChTime() < duration:
        step_orbit(state, step)
        satellite.SetPos(scaled(state["pos"]))
        sys.SetChTime(sys.GetChTime() + step)
    return sys, sun, satellite, state


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    sys, sun, satellite, state = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ObjectConnectorGravity.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.4, 1.8, 1.4), chrono.ChVector3d(0.25, 0.25, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and sys.GetChTime() < duration:
        t = sys.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        step_orbit(state, step)
        satellite.SetPos(scaled(state["pos"]))
        sys.SetChTime(sys.GetChTime() + step)
        if t >= next_log:
            print_state(sys, state)
            next_log += duration / 4.0


def print_state(sys, state):
    pos = state["pos"]
    print(
        f"t={sys.GetChTime():9.3f}  "
        f"pos=({pos.x:+.2f},{pos.y:+.2f})  y/r={pos.y / RADIUS:+.8f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=T_END)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ObjectConnectorGravity.py -> PyChrono visual Newtonian orbit")
    if args.no_vis:
        sys, sun, satellite, state = simulate(args.duration, args.step)
        print_state(sys, state)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

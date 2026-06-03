import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


AMP = 0.35
FREQ = 1.15
OMEGA = 2.0 * math.pi * FREQ
ANCHOR_LENGTH = 0.75
STEP = 1e-3


def make_color(r, g, b):
    return chrono.ChColor(r, g, b)


def add_visual_box(body, size, local_pos, color):
    shape = chrono.ChVisualShapeBox(size[0], size[1], size[2])
    shape.SetColor(color)
    body.AddVisualShape(shape, chrono.ChFramed(local_pos))
    return shape


def make_shaking_base(sys, z, color):
    base = chrono.ChBody()
    base.SetFixed(True)
    base.EnableCollision(False)
    sys.AddBody(base)

    add_visual_box(base, (2.0, 0.12, 0.42), chrono.ChVector3d(0, 0, 0), color)
    add_visual_box(
        base,
        (0.10, 1.10, 0.10),
        chrono.ChVector3d(-ANCHOR_LENGTH, 0.55, 0),
        color,
    )
    base.SetPos(chrono.ChVector3d(0, 0, z))
    return base


def make_structure(sys, z, color, mass):
    body = chrono.ChBodyEasyBox(0.55, 1.55, 0.42, 1000, True, False)
    body.SetMass(mass)
    body.SetInertiaXX(chrono.ChVector3d(1.0, 1.0, 1.0))
    body.SetPos(chrono.ChVector3d(0, 1.0, z))
    body.GetVisualShape(0).SetColor(color)
    sys.AddBody(body)
    return body


def connect_isolator(sys, structure, base, stiffness, damping, color):
    spring = chrono.ChLinkTSDA()
    spring.Initialize(
        structure,
        base,
        True,
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(-ANCHOR_LENGTH, 1.0, 0),
    )
    spring.SetRestLength(ANCHOR_LENGTH)
    spring.SetSpringCoefficient(stiffness)
    spring.SetDampingCoefficient(damping)
    sys.AddLink(spring)

    visual = chrono.ChVisualShapeSpring(0.035, 80, 12)
    visual.SetColor(color)
    spring.AddVisualShape(visual)
    return spring


def update_base_motion(base, t, z):
    x = AMP * math.sin(OMEGA * t)
    v = AMP * OMEGA * math.cos(OMEGA * t)
    base.SetPos(chrono.ChVector3d(x, 0, z))
    base.SetPosDt(chrono.ChVector3d(v, 0, 0))
    return x


print("Seismic isolation demo")
print("Red: stiff structure. Blue: isolated structure with lower stiffness/damping.")

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

stiff_z = -1.2
isolated_z = 1.2

stiff_base = make_shaking_base(sys, stiff_z, make_color(0.45, 0.45, 0.45))
isolated_base = make_shaking_base(sys, isolated_z, make_color(0.45, 0.45, 0.45))

stiff = make_structure(sys, stiff_z, make_color(0.85, 0.12, 0.08), 10.0)
isolated = make_structure(sys, isolated_z, make_color(0.08, 0.32, 0.85), 10.0)

connect_isolator(sys, stiff, stiff_base, 700.0, 7.0, make_color(0.85, 0.12, 0.08))
connect_isolator(sys, isolated, isolated_base, 75.0, 18.0, make_color(0.08, 0.32, 0.85))

floor = chrono.ChBodyEasyBox(4.6, 0.05, 3.4, 1000, True, False)
floor.SetFixed(True)
floor.SetPos(chrono.ChVector3d(0, -0.09, 0))
floor.GetVisualShape(0).SetColor(make_color(0.30, 0.34, 0.36))
sys.AddBody(floor)

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1180, 760)
vis.SetWindowTitle("Meaningful demo: seismic isolation")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4.0, 3.0, 6.5))
vis.AddTypicalLights()

max_stiff_drift = 0.0
max_isolated_drift = 0.0
next_log = 0.0

while vis.Run():
    t = sys.GetChTime()
    base_x = update_base_motion(stiff_base, t, stiff_z)
    update_base_motion(isolated_base, t, isolated_z)

    stiff_drift = stiff.GetPos().x - base_x
    isolated_drift = isolated.GetPos().x - base_x
    max_stiff_drift = max(max_stiff_drift, abs(stiff_drift))
    max_isolated_drift = max(max_isolated_drift, abs(isolated_drift))

    vis.BeginScene()
    vis.Render()
    chronoirr.drawLabel3D(
        vis,
        "stiff: high force transfer",
        chrono.ChVector3d(-1.6, 2.0, stiff_z),
        make_color(0.85, 0.12, 0.08),
    )
    chronoirr.drawLabel3D(
        vis,
        "isolated: lower acceleration",
        chrono.ChVector3d(-1.8, 2.0, isolated_z),
        make_color(0.08, 0.32, 0.85),
    )
    chronoirr.drawLabel3D(
        vis,
        f"base input = {base_x:+.2f} m",
        chrono.ChVector3d(-2.0, 0.25, 0),
        make_color(0.05, 0.05, 0.05),
    )
    vis.EndScene()

    sys.DoStepDynamics(STEP)

    if t >= next_log:
        print(
            f"t={t:5.2f}  base={base_x:+.3f}  "
            f"stiff_drift={stiff_drift:+.3f}  isolated_drift={isolated_drift:+.3f}  "
            f"peak=({max_stiff_drift:.3f}, {max_isolated_drift:.3f})"
        )
        next_log += 0.5

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


STEP = 1e-3
ANCHOR_X = -0.9
REST_TO_ANCHOR = 0.9
INITIAL_OFFSET = 0.65


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_anchor(sys, z):
    anchor = chrono.ChBodyEasyBox(0.16, 0.16, 0.16, 1000, True, False)
    anchor.SetFixed(True)
    anchor.SetPos(chrono.ChVector3d(ANCHOR_X, 1.0, z))
    anchor.GetVisualShape(0).SetColor(color(0.35, 0.35, 0.35))
    sys.AddBody(anchor)
    return anchor


def make_tower(sys, z, col):
    tower = chrono.ChBodyEasyBox(0.45, 1.55, 0.42, 1000, True, False)
    tower.SetMass(20.0)
    tower.SetInertiaXX(chrono.ChVector3d(1.2, 1.2, 1.2))
    tower.SetPos(chrono.ChVector3d(INITIAL_OFFSET, 1.0, z))
    tower.GetVisualShape(0).SetColor(col)
    sys.AddBody(tower)
    return tower


def connect_to_anchor(sys, body, anchor, col):
    spring = chrono.ChLinkTSDA()
    spring.Initialize(
        body,
        anchor,
        True,
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
    )
    spring.SetRestLength(REST_TO_ANCHOR)
    spring.SetSpringCoefficient(85.0)
    spring.SetDampingCoefficient(0.9)
    sys.AddLink(spring)
    visual = chrono.ChVisualShapeSpring(0.035, 80, 10)
    visual.SetColor(col)
    spring.AddVisualShape(visual)
    return spring


def add_tmd(sys, tower, z):
    absorber = chrono.ChBodyEasyBox(0.28, 0.28, 0.34, 1000, True, False)
    absorber.SetMass(2.0)
    absorber.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
    absorber.SetPos(chrono.ChVector3d(INITIAL_OFFSET, 2.0, z))
    absorber.GetVisualShape(0).SetColor(color(0.95, 0.72, 0.05))
    sys.AddBody(absorber)

    spring = chrono.ChLinkTSDA()
    spring.Initialize(
        absorber,
        tower,
        True,
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(-0.40, 1.0, 0),
    )
    spring.SetRestLength(0.40)
    spring.SetSpringCoefficient(8.5)
    spring.SetDampingCoefficient(1.6)
    sys.AddLink(spring)
    visual = chrono.ChVisualShapeSpring(0.025, 70, 8)
    visual.SetColor(color(0.95, 0.72, 0.05))
    spring.AddVisualShape(visual)
    return absorber


print("Tuned mass damper demo")
print("Both towers start displaced. The yellow top mass absorbs energy.")

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

plain_z = -1.35
tmd_z = 1.35

plain_anchor = make_anchor(sys, plain_z)
tmd_anchor = make_anchor(sys, tmd_z)

plain = make_tower(sys, plain_z, color(0.85, 0.12, 0.08))
with_tmd = make_tower(sys, tmd_z, color(0.08, 0.32, 0.85))
absorber = add_tmd(sys, with_tmd, tmd_z)

connect_to_anchor(sys, plain, plain_anchor, color(0.85, 0.12, 0.08))
connect_to_anchor(sys, with_tmd, tmd_anchor, color(0.08, 0.32, 0.85))

floor = chrono.ChBodyEasyBox(4.3, 0.05, 3.5, 1000, True, False)
floor.SetFixed(True)
floor.SetPos(chrono.ChVector3d(0, 0.15, 0))
floor.GetVisualShape(0).SetColor(color(0.30, 0.34, 0.36))
sys.AddBody(floor)

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1180, 760)
vis.SetWindowTitle("Meaningful demo: tuned mass damper")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4.0, 3.0, 6.5))
vis.AddTypicalLights()

plain_peak = 0.0
tmd_peak = 0.0
next_log = 0.0

while vis.Run():
    t = sys.GetChTime()

    plain_x = plain.GetPos().x
    tmd_x = with_tmd.GetPos().x
    plain_peak = max(plain_peak, abs(plain_x))
    tmd_peak = max(tmd_peak, abs(tmd_x))

    vis.BeginScene()
    vis.Render()
    chronoirr.drawLabel3D(
        vis,
        "no damper",
        chrono.ChVector3d(-1.3, 2.05, plain_z),
        color(0.85, 0.12, 0.08),
    )
    chronoirr.drawLabel3D(
        vis,
        "with tuned mass damper",
        chrono.ChVector3d(-1.7, 2.35, tmd_z),
        color(0.08, 0.32, 0.85),
    )
    chronoirr.drawLabel3D(
        vis,
        "yellow mass moves out of phase",
        absorber.GetPos() + chrono.ChVector3d(-0.8, 0.35, 0),
        color(0.08, 0.08, 0.08),
    )
    vis.EndScene()

    sys.DoStepDynamics(STEP)

    if t >= next_log:
        print(
            f"t={t:5.2f}  no_damper_x={plain_x:+.3f}  "
            f"tmd_tower_x={tmd_x:+.3f}  absorber_x={absorber.GetPos().x:+.3f}  "
            f"peak=({plain_peak:.3f}, {tmd_peak:.3f})"
        )
        next_log += 0.5

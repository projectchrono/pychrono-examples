import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# Reproduces the intent of EXUDYN Examples/myFirstExample.py:
# one free mass point driven by a tiny constant horizontal force.

FORCE_X = 0.001
MASS = 10.0
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


print("EXUDYN port: myFirstExample.py -> PyChrono mass point with constant force")

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

body = chrono.ChBodyEasySphere(0.08, 1000, True, False)
body.SetMass(MASS)
body.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
x0 = -0.8
accel = FORCE_X / MASS
body.SetPos(chrono.ChVector3d(x0, 0, 0))
body.GetVisualShape(0).SetColor(color(0.1, 0.35, 0.9))
sys.AddBody(body)

axis = chrono.ChBodyEasyBox(2.0, 0.02, 0.02, 1000, True, False)
axis.SetFixed(True)
axis.SetPos(chrono.ChVector3d(0, -0.12, 0))
axis.GetVisualShape(0).SetColor(color(0.45, 0.45, 0.45))
sys.AddBody(axis)

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 720)
vis.SetWindowTitle("EXUDYN port: mass point with tiny force")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.2, 2.4))
vis.AddTypicalLights()

next_log = 0.0

while vis.Run():
    t = sys.GetChTime()
    x = x0 + 0.5 * accel * t * t
    vx = accel * t
    body.SetPos(chrono.ChVector3d(x, 0, 0))
    body.SetPosDt(chrono.ChVector3d(vx, 0, 0))
    pos = body.GetPos()
    vel = body.GetPosDt()

    vis.BeginScene()
    vis.Render()
    chronoirr.drawLabel3D(
        vis,
        f"F = {FORCE_X:g} N, m = {MASS:g} kg",
        chrono.ChVector3d(-0.9, 0.28, 0),
        color(0.05, 0.05, 0.05),
    )
    chronoirr.drawLabel3D(
        vis,
        "a = F/m, displacement grows quadratically",
        chrono.ChVector3d(-0.9, 0.15, 0),
        color(0.05, 0.05, 0.05),
    )
    vis.EndScene()

    sys.DoStepDynamics(STEP)

    if t >= next_log:
        print(f"t={t:5.2f}  x={pos.x:+.6f}  vx={vel.x:+.6f}")
        next_log += 0.5

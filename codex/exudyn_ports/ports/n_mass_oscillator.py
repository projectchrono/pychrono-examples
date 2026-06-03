import math
import sys
from pathlib import Path

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN Examples/nMassOscillator.py:
# a chain of masses joined by spring-dampers, base-driven at the right end.

N = 12
MASS = 1.0
K = 800.0
D = 2.0
REST = 0.22
DRIVE_AMPLITUDE = 0.12
DRIVE_FREQ = 3.55
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def add_sphere(sys, x, col):
    body = chrono.ChBodyEasySphere(0.055, 1000, True, False)
    body.SetMass(MASS)
    body.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, 0.02))
    body.SetPos(chrono.ChVector3d(x, 0, 0))
    body.GetVisualShape(0).SetColor(col)
    sys.AddBody(body)
    return body


print("EXUDYN port: nMassOscillator.py -> PyChrono chain oscillator")

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

ground = chrono.ChBody()
ground.SetFixed(True)
sys.AddBody(ground)

anchor = chrono.ChBodyEasySphere(0.045, 1000, True, False)
anchor.SetFixed(True)
anchor.SetPos(chrono.ChVector3d(0, 0, 0))
anchor.GetVisualShape(0).SetColor(color(0.1, 0.1, 0.1))
sys.AddBody(anchor)

masses = []
prev = ground
prev_anchor = chrono.ChVector3d(0, 0, 0)

for i in range(N):
    col = color(0.12, 0.42, 0.85)
    if i == 0:
        col = color(0.10, 0.65, 0.25)
    elif i == N - 1:
        col = color(0.85, 0.12, 0.08)
    body = add_sphere(sys, (i + 1) * REST, col)
    masses.append(body)

    spring = chrono.ChLinkTSDA()
    p0 = chrono.ChVector3d(0, 0, 0)
    p1 = prev_anchor if i == 0 else chrono.ChVector3d(0, 0, 0)
    spring.Initialize(body, prev, True, p0, p1)
    spring.SetRestLength(REST)
    spring.SetSpringCoefficient(K)
    spring.SetDampingCoefficient(D)
    sys.AddLink(spring)
    visual = chrono.ChVisualShapeSpring(0.025, 50, 8)
    visual.SetColor(color(0.45, 0.45, 0.45))
    spring.AddVisualShape(visual)
    attach_spring_visual(sys, spring, 0.025, 50, 8, color(0.45, 0.45, 0.45))

    prev = body

driver_x0 = (N + 1) * REST
driver = chrono.ChBodyEasyBox(0.08, 0.16, 0.12, 1000, True, False)
driver.SetFixed(True)
driver.SetPos(chrono.ChVector3d(driver_x0, 0, 0))
driver.GetVisualShape(0).SetColor(color(0.95, 0.72, 0.05))
sys.AddBody(driver)

driver_spring = chrono.ChLinkTSDA()
driver_spring.Initialize(
    masses[-1],
    driver,
    True,
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(0, 0, 0),
)
driver_spring.SetRestLength(REST)
driver_spring.SetSpringCoefficient(K)
driver_spring.SetDampingCoefficient(D)
sys.AddLink(driver_spring)
driver_visual = chrono.ChVisualShapeSpring(0.025, 50, 8)
driver_visual.SetColor(color(0.95, 0.72, 0.05))
driver_spring.AddVisualShape(driver_visual)
attach_spring_visual(sys, driver_spring, 0.025, 50, 8, color(0.95, 0.72, 0.05))

rail = chrono.ChBodyEasyBox((N + 2) * REST, 0.025, 0.025, 1000, True, False)
rail.SetFixed(True)
rail.SetPos(chrono.ChVector3d((N + 1) * REST / 2, -0.13, 0))
rail.GetVisualShape(0).SetColor(color(0.35, 0.35, 0.35))
sys.AddBody(rail)

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1180, 720)
vis.SetWindowTitle("EXUDYN port: n-mass oscillator")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.45, 1.2, 2.8))
vis.AddTypicalLights()

next_log = 0.0

while vis.Run():
    t = sys.GetChTime()
    drive = DRIVE_AMPLITUDE * math.sin(DRIVE_FREQ * t)
    driver.SetPos(chrono.ChVector3d(driver_x0 + drive, 0, 0))
    driver.SetPosDt(chrono.ChVector3d(DRIVE_AMPLITUDE * DRIVE_FREQ * math.cos(DRIVE_FREQ * t), 0, 0))

    vis.BeginScene()
    update_system_visuals(sys)
    vis.Render()
    chronoirr.drawLabel3D(
        vis,
        "harmonic base motion drives final spring",
        chrono.ChVector3d(1.0, 0.35, 0),
        color(0.05, 0.05, 0.05),
    )
    vis.EndScene()

    sys.DoStepDynamics(STEP)

    if t >= next_log:
        xs = [m.GetPos().x - (i + 1) * REST for i, m in enumerate(masses)]
        print(f"t={t:5.2f}  driver={drive:+.3f}  first={xs[0]:+.3f}  last={xs[-1]:+.3f}")
        next_log += 0.5

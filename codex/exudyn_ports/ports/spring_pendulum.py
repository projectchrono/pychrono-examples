import sys
from pathlib import Path

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN Examples/pendulum.py:
# a point-mass pendulum modeled as a mass attached to ground through a
# stiff spring-damper, with gravity.

LENGTH = 0.8
MASS = 2.5
SPRING = 4000.0
DAMPING = 200.0
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


print("EXUDYN port: pendulum.py -> PyChrono stiff spring pendulum")

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

ground = chrono.ChBody()
ground.SetFixed(True)
sys.AddBody(ground)

body = chrono.ChBodyEasySphere(0.06, 1000, True, False)
body.SetMass(MASS)
body.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, 0.02))
body.SetPos(chrono.ChVector3d(LENGTH, 0, 0))
body.GetVisualShape(0).SetColor(color(0.1, 0.3, 0.9))
sys.AddBody(body)

spring = chrono.ChLinkTSDA()
spring.Initialize(
    body,
    ground,
    True,
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(0, 0, 0),
)
spring.SetRestLength(LENGTH)
spring.SetSpringCoefficient(SPRING)
spring.SetDampingCoefficient(DAMPING)
sys.AddLink(spring)
visual = chrono.ChVisualShapeSpring(0.025, 90, 12)
visual.SetColor(color(0.9, 0.2, 0.1))
spring.AddVisualShape(visual)
attach_spring_visual(sys, spring, 0.035, 90, 12, color(0.9, 0.2, 0.1))

anchor = chrono.ChBodyEasySphere(0.04, 1000, True, False)
anchor.SetFixed(True)
anchor.SetPos(chrono.ChVector3d(0, 0, 0))
anchor.GetVisualShape(0).SetColor(color(0.15, 0.15, 0.15))
sys.AddBody(anchor)

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 720)
vis.SetWindowTitle("EXUDYN port: stiff spring pendulum")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.75, 2.4))
vis.AddTypicalLights()

next_log = 0.0

while vis.Run():
    t = sys.GetChTime()
    pos = body.GetPos()

    vis.BeginScene()
    update_system_visuals(sys)
    vis.Render()
    chronoirr.drawLabel3D(
        vis,
        "stiff spring approximates distance constraint",
        chrono.ChVector3d(-0.5, 0.35, 0),
        color(0.05, 0.05, 0.05),
    )
    vis.EndScene()

    sys.DoStepDynamics(STEP)

    if t >= next_log:
        print(
            f"t={t:5.2f}  pos=({pos.x:+.3f}, {pos.y:+.3f})  "
            f"length={spring.GetLength():.4f}  force={spring.GetForce():+.2f}"
        )
        next_log += 0.5

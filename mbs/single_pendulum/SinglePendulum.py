import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Create the ground body with two visualization cylinders
ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

cyl_1 = chrono.ChVisualShapeCylinder(0.2, 0.4)
ground.AddVisualShape(cyl_1, chrono.ChFramed(chrono.ChVector3d(0, 0, +1)))
# Create a pendulum modeled using ChBody
pend_1 =  chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)
pend_1.EnableCollision(False)
pend_1.SetMass(1)
pend_1.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))
# Attach a visualization asset. Note that the cylinder is defined with
# respect to the centroidal reference frame (which is the body reference
# frame for a ChBody)
cyl_1 = chrono.ChVisualShapeCylinder(0.2, 2)
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))
pend_1.AddVisualShape(cyl_1, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Specify the intial position of the pendulum (horizontal, pointing towards
# positive X). In this case, we set the absolute position of its center of
# mass
pend_1.SetPos(chrono.ChVector3d(1, 0, 1))
# Create a revolute joint to connect pendulum to ground. We specify the link
# coordinate frame in the absolute frame.
rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(rev_1)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('ChBodyAuxRef demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()
# Simulation Loop
log_info = True
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)

    if log_info and sys.GetChTime() > 1:
        pos_1 = pend_1.GetPos()
        print("t = ", sys.GetChTime())
        print("     ", pos_1.x, "  ", pos_1.y)
        lin_vel_1 = pend_1.GetPosDt()
        print("     ", lin_vel_1.x, "  ", lin_vel_1.y)
        log_info = False

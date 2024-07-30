import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Create the system
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Ground body
ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)
ground.EnableCollision(False)



# First pendulum
pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)
pend_1.EnableCollision(False)
pend_1.SetMass(1)
pend_1.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))
pend_1.SetPos(chrono.ChVector3d(1, 0, 0))
pend_1.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))

# Visualization for the first pendulum
cyl_1 = chrono.ChVisualShapeCylinder(0.1, 2)
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))
#pend_1.AddVisualShape(cyl_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Initial position of the first pendulum (horizontal)
pend_1.SetPos(chrono.ChVector3d(1, 0, 0))

# Revolute joint for the first pendulum (connected to ground)
rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(rev_1)

# Second pendulum
pend_2 = chrono.ChBody()
sys.AddBody(pend_2)
pend_2.SetFixed(False)
pend_2.EnableCollision(False)
pend_2.SetMass(1)
pend_2.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))

# Visualization for the second pendulum
cyl_2 = chrono.ChVisualShapeCylinder(0.1, 2)
cyl_2.SetColor(chrono.ChColor(0, 0, 0.6))
#pend_2.AddVisualShape(cyl_2, chrono.ChFramed(chrono.ChVector3d(3/2, 0, 0), chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Initial position of the second pendulum
pend_2.SetPos(chrono.ChVector3d(3/2, 0, 0))

# Revolute joint for the second pendulum (connected to the first pendulum)
rev_2 = chrono.ChLinkLockSpherical()
rev_2.Initialize(pend_1, pend_2, chrono.ChFramed(chrono.ChVector3d(3/2, 0, 0), chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.AddLink(rev_2)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Double Pendulum Simulation')
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
    sys.DoStepDynamics(1e-2)

    if log_info and sys.GetChTime() > 1:
        # Get positions and velocities for debugging
        pos_1 = pend_1.GetPos()
        pos_2 = pend_2.GetPos()
        lin_vel_1 = pend_1.GetPosDt()
        lin_vel_2 = pend_2.GetPosDt()

        print(f"t = {sys.GetChTime()}")
        print(f"Pendulum 1 position: {pos_1.x}, {pos_1.y}")
        print(f"Pendulum 1 velocity: {lin_vel_1.x}, {lin_vel_1.y}")
        print(f"Pendulum 2 position: {pos_2.x}, {pos_2.y}")
        print(f"Pendulum 2 velocity: {lin_vel_2.x}, {lin_vel_2.y}")

        log_info = False

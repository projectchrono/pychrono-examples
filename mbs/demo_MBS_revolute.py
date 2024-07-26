import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys      = chrono.ChSystemNSC()

# Create a fixed rigid body

mbody1 = chrono.ChBody()
mbody1.SetFixed(True)
mbody1.SetPos( chrono.ChVector3d(0,0,-0.2))
sys.Add(mbody1)

mboxasset = chrono.ChVisualShapeBox(0.4, 1.0, 0.2)
mbody1.AddVisualShape(mboxasset)



# Create a swinging rigid body

mbody2 = chrono.ChBody()
mbody2.SetFixed(False)
sys.Add(mbody2)

mboxasset = chrono.ChVisualShapeBox(0.4, 1.0, 0.2)
mboxasset.SetTexture(chrono.GetChronoDataFile('textures/concrete.jpg'))
mbody2.AddVisualShape(mboxasset)


# Create a revolute constraint

mlink = chrono.ChLinkRevolute()

    # the coordinate sys of the constraint reference in abs. space:
mframe = chrono.ChFramed(chrono.ChVector3d(0.1,0.5,0))

    # initialize the constraint telling which part must be connected, and where:
mlink.Initialize(mbody1,mbody2, mframe)

sys.Add(mlink)

# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the sys
#

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Revolute joint demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.6,0.6,0.8))
vis.AddTypicalLights()


# ---------------------------------------------------------------------
#
#  Run the simulation
#


while vis.Run():
    vis.BeginScene() 
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(5e-3)





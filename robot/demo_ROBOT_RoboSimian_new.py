import os
import errno
import math
import pychrono as chrono
import pychrono.robot as robosimian
import pychrono.irrlicht as chronoirr

time_step = 1e-3


def SetContactProperties(robot):
    friction = 0.8
    Y = 1e7
    cr = 0.0

    robot.GetSledContactMaterial().SetFriction(friction)
    robot.GetSledContactMaterial().SetRestitution(cr)

    robot.GetWheelContactMaterial().SetFriction(friction)
    robot.GetWheelContactMaterial().SetRestitution(cr)

    if robot.GetSystem().GetContactMethod() == chrono.ChContactMethod_SMC:
        sled_matSMC = chrono.CastToChContactMaterialSMC(robot.GetSledContactMaterial())
        sled_matSMC.SetYoungModulus(Y)
        wheel_matSMC = chrono.CastToChContactMaterialSMC(robot.GetWheelContactMaterial())
        wheel_matSMC.SetYoungModulus(Y)
    # Drop the robot on rigid terrain
drop = True

# Robot locomotion mode
mode = robosimian.LocomotionMode_WALK

# Contact method (system type)
contact_method = chrono.ChContactMethod_SMC

# Phase durations
duration_pose = 1.0
duration_settle_robot = 0.5
duration_sim = 10

# Output frequencies
output_fps = 100
render_fps = 60

# Output directories
out_dir = "./ROBOSIMIAN_RIGID"
pov_dir = out_dir + "/POVRAY"
img_dir = out_dir + "/IMG"

# POV-Ray and/or IMG output
data_output = True
povray_output = False
image_output = False

# Timed events
time_create_terrain = duration_pose
time_start = time_create_terrain + duration_settle_robot
time_end = time_start + duration_sim

# Create system
if contact_method == chrono.ChContactMethod_NSC:
    sys = chrono.ChSystemNSC()
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)
else:
    sys = chrono.ChSystemSMC()

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.GetSolver().AsIterative().SetMaxIterations(200)
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.8))

# Create RoboSimian robot
robot = robosimian.RoboSimian(sys, True, True)
robot.SetOutputDirectory(out_dir)
robot.Initialize(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))

# Set contact properties
SetContactProperties(robot)

# Set driver
if mode == robosimian.LocomotionMode_WALK:
    driver = robosimian.RS_Driver(
        "",
        chrono.GetChronoDataFile("robot/robosimian/actuation/walking_cycle.txt"),
        "",
        True)
elif mode == robosimian.LocomotionMode_SCULL:
    driver = robosimian.RS_Driver(
        chrono.GetChronoDataFile("robot/robosimian/actuation/sculling_start.txt"),
        chrono.GetChronoDataFile("robot/robosimian/actuation/sculling_cycle2.txt"),
        chrono.GetChronoDataFile("robot/robosimian/actuation/sculling_stop.txt"),
        True)
elif mode == robosimian.LocomotionMode_INCHWORM:
    driver = robosimian.RS_Driver(
        chrono.GetChronoDataFile("robot/robosimian/actuation/inchworming_start.txt"),
        chrono.GetChronoDataFile("robot/robosimian/actuation/inchworming_cycle.txt"),
        chrono.GetChronoDataFile("robot/robosimian/actuation/inchworming_stop.txt"),
        True)
elif mode == robosimian.LocomotionMode_DRIVE:
    driver = robosimian.RS_Driver(
        chrono.GetChronoDataFile("robot/robosimian/actuation/driving_start.txt"),
        chrono.GetChronoDataFile("robot/robosimian/actuation/driving_cycle.txt"),
        chrono.GetChronoDataFile("robot/robosimian/actuation/driving_stop.txt"),
        True)
else:
    raise ValueError('Invalid locomotion mode')

cbk = robosimian.RS_DriverCallback(robot)
driver.RegisterPhaseChangeCallback(cbk)

driver.SetTimeOffsets(duration_pose, duration_settle_robot)
robot.SetDriver(driver)

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('RoboSimian - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, -2.75, 0.2), chrono.ChVector3d(1, 0, 0))
vis.AddLight(chrono.ChVector3d(100, +100, 100), 290)
vis.AddLight(chrono.ChVector3d(100, -100, 80), 190)

# Output directories
try:
    os.mkdir(out_dir)
except OSError as exc:
    if exc.errno != errno.EEXIST:
       print("Error creating output directory")
if povray_output:
    os.mkdir(pov_dir)
if image_output:
    os.mkdir(img_dir)

# Simulation loop
output_steps = math.ceil((1.0 / output_fps) / time_step)
render_steps = math.ceil((1.0 / render_fps) / time_step)
sim_frame = 0
render_frame = 0
terrain_created = False

while vis.Run():
    if drop and not terrain_created and sys.GetChTime() > time_create_terrain:
        z = robot.GetWheelPos(robosimian.FR).z - 0.15
        length = 8
        width = 2
        ground = CreateTerrain(sys, length, width, z, length / 4)
        SetContactProperties(robot)
        vis.BindItem(ground)
        robot.GetChassisBody().SetFixed(False)
        terrain_created = True

    vis.BeginScene()
    vis.Render()
    if data_output and sim_frame % output_steps == 0:
        robot.Output()
    if povray_output and sim_frame % render_steps == 0:
        filename = pov_dir + '/data_' + str(render_frame + 1).zfill(4) + '.dat'
        chrono.WriteVisualizationAssets(sys, filename)
    if image_output and sim_frame % render_steps == 0:
        filename = img_dir + '/img_' + str(render_frame + 1).zfill(4) + '.jpg'
        image = vis.GetVideoDriver().createScreenShot()
        if image:
            vis.GetVideoDriver().writeImageToFile(image, filename)
            image.drop()
        render_frame += 1
    robot.DoStepDynamics(time_step)
    sim_frame += 1
    vis.EndScene()

print("avg. speed: "  + str(cbk.GetAvgSpeed()) + '\n')

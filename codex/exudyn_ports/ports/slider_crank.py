import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# Reproduces the intent of EXUDYN Examples/SliderCrank.py:
# a motor-driven crank, connecting rod, and prismatic piston.

STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


print("EXUDYN port: SliderCrank.py -> PyChrono motorized slider crank")

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

crank_center = chrono.ChVector3d(-1, 0.5, 0)
crank_rad = 0.4
crank_thick = 0.1
rod_length = 1.5

floor = chrono.ChBodyEasyBox(3, 0.16, 1.0, 1000, True, False)
floor.SetPos(chrono.ChVector3d(0, 0, 0))
floor.SetFixed(True)
floor.GetVisualShape(0).SetColor(color(0.35, 0.35, 0.35))
sys.Add(floor)

crank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)
crank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))
crank.SetRot(chrono.Q_ROTATE_Y_TO_Z)
crank.GetVisualShape(0).SetColor(color(0.85, 0.12, 0.08))
sys.Add(crank)

rod = chrono.ChBodyEasyBox(rod_length, 0.08, 0.08, 1000)
rod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))
rod.GetVisualShape(0).SetColor(color(0.12, 0.42, 0.85))
sys.Add(rod)

piston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.20, 0.30, 1000)
piston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))
piston.SetRot(chrono.Q_ROTATE_Y_TO_X)
piston.GetVisualShape(0).SetColor(color(0.95, 0.72, 0.05))
sys.Add(piston)

motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, floor, chrono.ChFramed(crank_center))
motor.SetMotorFunction(chrono.ChFunctionConst(chrono.CH_PI))
sys.Add(motor)

joint_a = chrono.ChLinkLockRevolute()
joint_a.Initialize(rod, crank, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))
sys.Add(joint_a)

joint_b = chrono.ChLinkLockRevolute()
joint_b.Initialize(
    piston,
    rod,
    chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)),
)
sys.Add(joint_b)

joint_c = chrono.ChLinkLockPrismatic()
joint_c.Initialize(
    piston,
    floor,
    chrono.ChFramed(
        crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0),
        chrono.Q_ROTATE_Z_TO_X,
    ),
)
sys.Add(joint_c)

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 720)
vis.SetWindowTitle("EXUDYN port: slider crank")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 1.2, 3.0), chrono.ChVector3d(0, 0.5, 0))
vis.AddTypicalLights()

next_log = 0.0

while vis.Run():
    t = sys.GetChTime()
    vis.BeginScene()
    vis.Render()
    chronoirr.drawLabel3D(
        vis,
        "motor -> crank -> rod -> prismatic piston",
        chrono.ChVector3d(-1.4, 1.25, 0),
        color(0.05, 0.05, 0.05),
    )
    vis.EndScene()
    sys.DoStepDynamics(STEP)

    if t >= next_log:
        print(
            f"t={t:5.2f}  angle={motor.GetMotorAngle():+.3f}  "
            f"piston_x={piston.GetPos().x:+.3f}  piston_vx={piston.GetPosDt().x:+.3f}"
        )
        next_log += 0.5

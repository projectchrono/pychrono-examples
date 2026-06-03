import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


RADIUS = 0.35
SPACING = 2.02 * RADIUS
STEP = 8e-4


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_sphere(sys, name, x, initial_v, mat, col):
    body = chrono.ChBodyEasySphere(RADIUS, 1000, True, True, mat)
    body.SetName(name)
    body.SetPos(chrono.ChVector3d(x, 0.75, 0))
    body.SetPosDt(chrono.ChVector3d(initial_v, 0, 0))
    body.GetVisualShape(0).SetColor(col)
    sys.AddBody(body)
    return body


print("Momentum transfer demo")
print("A moving sphere transfers momentum through a nearly elastic collision row.")

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(60)

mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.0)
mat.SetRestitution(0.92)

spheres = []
spheres.append(make_sphere(sys, "incoming", -2.4, 4.0, mat, color(0.95, 0.72, 0.08)))
for i in range(6):
    col = color(0.08, 0.32, 0.85)
    if i == 5:
        col = color(0.10, 0.65, 0.25)
    spheres.append(make_sphere(sys, f"target_{i + 1}", i * SPACING, 0.0, mat, col))

rail = chrono.ChBodyEasyBox(6.2, 0.08, 0.12, 1000, True, False)
rail.SetFixed(True)
rail.SetPos(chrono.ChVector3d(0.15, 0.18, 0))
rail.GetVisualShape(0).SetColor(color(0.38, 0.38, 0.38))
sys.AddBody(rail)

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1180, 760)
vis.SetWindowTitle("Meaningful demo: momentum transfer")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.0, 2.7, 6.2))
vis.AddTypicalLights()

next_log = 0.0

while vis.Run():
    t = sys.GetChTime()

    vis.BeginScene()
    vis.Render()
    chronoirr.drawLabel3D(
        vis,
        "nearly elastic momentum transfer",
        chrono.ChVector3d(-2.4, 1.55, 0),
        color(0.05, 0.05, 0.05),
    )
    chronoirr.drawLabel3D(
        vis,
        "watch the last sphere leave the row",
        chrono.ChVector3d(1.5, 1.35, 0),
        color(0.05, 0.05, 0.05),
    )
    vis.EndScene()

    sys.DoStepDynamics(STEP)

    if t >= next_log:
        velocities = " ".join(f"{s.GetPosDt().x:+.2f}" for s in spheres)
        print(f"t={t:5.2f}  vx=[{velocities}]")
        next_log += 0.35

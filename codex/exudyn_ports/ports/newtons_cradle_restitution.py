import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# Reproduces the intent of EXUDYN Examples/newtonsCradle.py:
# several Newton-cradle rows with different restitution coefficients.
# PyChrono uses direct rigid sphere contact for the impact model.

RADIUS = 0.14
SPACING = 2.05 * RADIUS
STEP = 7.5e-4
ROWS = [
    ("e=1.00", 1.00, 0.75),
    ("e=0.95", 0.95, 0.25),
    ("e=0.80", 0.80, -0.25),
    ("e=0.10", 0.10, -0.75),
]


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_row(sys, label, restitution, z):
    mat = chrono.ChContactMaterialNSC()
    mat.SetFriction(0.0)
    mat.SetRestitution(restitution)

    spheres = []
    for i in range(5):
        x = i * SPACING
        vx = 0.0
        col = color(0.15, 0.38, 0.85)
        if i == 4:
            x += 0.55
            vx = -1.55
            col = color(0.95, 0.72, 0.05)

        body = chrono.ChBodyEasySphere(RADIUS, 7800, True, True, mat)
        body.SetPos(chrono.ChVector3d(x, 0.5, z))
        body.SetPosDt(chrono.ChVector3d(vx, 0, 0))
        body.GetVisualShape(0).SetColor(col)
        sys.AddBody(body)
        spheres.append(body)

    support = chrono.ChBodyEasyBox(1.65, 0.04, 0.04, 1000, True, False)
    support.SetFixed(True)
    support.SetPos(chrono.ChVector3d(0.3, 0.77, z))
    support.GetVisualShape(0).SetColor(color(0.4, 0.4, 0.4))
    sys.AddBody(support)
    return label, spheres


print("EXUDYN port: newtonsCradle.py -> PyChrono restitution comparison")

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(80)

rows = [make_row(sys, *row) for row in ROWS]

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1180, 760)
vis.SetWindowTitle("EXUDYN port: Newton cradle restitution comparison")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.7, 1.6, 3.4))
vis.AddTypicalLights()

next_log = 0.0

while vis.Run():
    t = sys.GetChTime()

    vis.BeginScene()
    vis.Render()
    for label, zinfo in zip([r[0] for r in ROWS], [r[2] for r in ROWS]):
        chronoirr.drawLabel3D(
            vis,
            label,
            chrono.ChVector3d(-0.45, 0.9, zinfo),
            color(0.05, 0.05, 0.05),
        )
    vis.EndScene()

    sys.DoStepDynamics(STEP)

    if t >= next_log:
        report = []
        for label, spheres in rows:
            report.append(f"{label}: outgoing vx={spheres[0].GetPosDt().x:+.2f}")
        print(f"t={t:5.2f}  " + " | ".join(report))
        next_log += 0.35

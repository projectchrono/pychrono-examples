import argparse

import pychrono.core as chrono


# Reproduces EXUDYN TestModels/MiniExamples/miniExamplesFileList.py, which is
# a data-only module containing the list of mini example file names.

MINI_EXAMPLES = [
    "ObjectMassPoint.py",
    "ObjectMassPoint2D.py",
    "ObjectMass1D.py",
    "ObjectRotationalMass1D.py",
    "ObjectRigidBody2D.py",
    "ObjectGenericODE2.py",
    "ObjectGenericODE1.py",
    "ObjectKinematicTree.py",
    "ObjectANCFCable.py",
    "ObjectANCFCable2D.py",
    "ObjectANCFThinPlate.py",
    "ObjectConnectorSpringDamper.py",
    "ObjectConnectorCartesianSpringDamper.py",
    "ObjectConnectorRigidBodySpringDamper.py",
    "ObjectConnectorLinearSpringDamper.py",
    "ObjectConnectorTorsionalSpringDamper.py",
    "ObjectConnectorCoordinateSpringDamper.py",
    "ObjectConnectorGravity.py",
    "ObjectConnectorDistance.py",
    "ObjectConnectorCoordinate.py",
    "ObjectJointRevoluteZ.py",
    "MarkerSuperElementPosition.py",
    "LoadMassProportional.py",
]

STEP = 1.0e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(x, y, z)


def marker_color(index):
    palette = [
        color(0.12, 0.38, 0.88),
        color(0.10, 0.58, 0.32),
        color(0.90, 0.42, 0.08),
        color(0.62, 0.26, 0.82),
    ]
    return palette[index % len(palette)]


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))

    board = chrono.ChBody()
    board.SetName("miniExamplesFileList marker board")
    board.SetFixed(True)
    board.EnableCollision(False)
    back = chrono.ChVisualShapeBox(1.65, 1.10, 0.030)
    back.SetColor(color(0.20, 0.22, 0.25))
    back.SetOpacity(0.30)
    board.AddVisualShape(back, chrono.ChFramed(vec(0.0, 0.0, -0.020)))
    system.AddBody(board)

    columns = 6
    spacing_x = 0.25
    spacing_y = 0.22
    start_x = -0.5 * spacing_x * (columns - 1)
    rows = (len(MINI_EXAMPLES) + columns - 1) // columns
    start_y = 0.5 * spacing_y * (rows - 1)
    markers = []
    for index, name in enumerate(MINI_EXAMPLES):
        col = index % columns
        row = index // columns
        marker = chrono.ChBodyEasyBox(0.105, 0.075, 0.030, 1000.0, True, False)
        marker.SetName("miniExamplesFileList item " + name)
        marker.SetFixed(True)
        marker.EnableCollision(False)
        marker.SetPos(vec(start_x + col * spacing_x, start_y - row * spacing_y, 0.030))
        marker.GetVisualShape(0).SetColor(marker_color(index))
        system.AddBody(marker)
        markers.append(marker)

    result = {
        "count": len(MINI_EXAMPLES),
        "first": MINI_EXAMPLES[0],
        "last": MINI_EXAMPLES[-1],
        "markers": len(markers),
    }
    system._mini_examples_file_list_result = result
    return system, result


def update_visuals(_system):
    return None


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _result = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: miniExamplesFileList.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(vec(0.0, -1.80, 1.30), vec(0.0, 0.0, 0.0))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def print_result():
    joined = ",".join(MINI_EXAMPLES)
    print(
        f"mini_examples_file_list: count={len(MINI_EXAMPLES)}  "
        f"first={MINI_EXAMPLES[0]}  last={MINI_EXAMPLES[-1]}"
    )
    print(f"mini_examples_file_list: files={joined}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0.10)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: miniExamplesFileList.py -> PyChrono static list scene")
    if args.no_vis:
        print_result()
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

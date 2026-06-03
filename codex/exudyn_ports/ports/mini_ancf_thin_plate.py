import argparse

import pychrono.core as chrono


# Reproduces EXUDYN TestModels/MiniExamples/ObjectANCFThinPlate.py.
# The upstream mini example is an explicit placeholder: after creating ground
# and a ground node, it only contains "#to be done" and sets testResult = 0.
# This port mirrors that behavior and provides a static placeholder visual so
# the example is still inspectable in the PyChrono capture workflow.

TEST_RESULT = 0.0
STEP = 1.0e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(x, y, z)


def add_segment(system, name, a, b, tint, thickness=3):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    shape = chrono.ChVisualShapeSegment()
    shape.SetColor(tint)
    shape.SetThickness(thickness)
    shape.SetLineGeometry(chrono.ChLineSegment(a, b))
    body.AddVisualShape(shape)
    system.AddBody(body)
    return body


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0.0, 0.0, 0.0))

    ground = chrono.ChBody()
    ground.SetName("ObjectANCFThinPlate placeholder ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    base = chrono.ChVisualShapeBox(1.35, 0.78, 0.018)
    base.SetColor(color(0.22, 0.45, 0.62))
    base.SetOpacity(0.22)
    ground.AddVisualShape(base, chrono.ChFramed(vec(0.0, 0.0, -0.012)))
    system.AddBody(ground)

    plate = chrono.ChBody()
    plate.SetName("ObjectANCFThinPlate to-be-done plate outline")
    plate.SetFixed(True)
    plate.EnableCollision(False)
    shape = chrono.ChVisualShapeBox(0.90, 0.45, 0.010)
    shape.SetColor(color(0.86, 0.38, 0.08))
    shape.SetOpacity(0.36)
    plate.AddVisualShape(shape, chrono.ChFramed(vec(0.0, 0.0, 0.035)))
    system.AddBody(plate)

    corners = [(-0.45, -0.225), (0.45, -0.225), (0.45, 0.225), (-0.45, 0.225)]
    for index, (x, y) in enumerate(corners):
        node = chrono.ChBodyEasySphere(0.035, 1000.0, True, False)
        node.SetName(f"ObjectANCFThinPlate placeholder corner node {index}")
        node.SetFixed(True)
        node.EnableCollision(False)
        node.SetPos(vec(x, y, 0.055))
        node.GetVisualShape(0).SetColor(color(0.08, 0.10, 0.12))
        system.AddBody(node)

    add_segment(system, "ObjectANCFThinPlate x reference axis", vec(-0.60, 0.0, 0.075), vec(0.62, 0.0, 0.075), color(0.12, 0.32, 0.95), 3)
    add_segment(system, "ObjectANCFThinPlate y reference axis", vec(0.0, -0.34, 0.075), vec(0.0, 0.34, 0.075), color(0.10, 0.62, 0.20), 3)

    result = {"test_result": TEST_RESULT, "implemented_in_source": False, "corner_nodes": len(corners)}
    system._mini_ancf_thin_plate_result = result
    return system, result


def update_visuals(_system):
    return None


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _result = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ObjectANCFThinPlate.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(vec(0.70, -1.45, 0.95), vec(0.0, 0.0, 0.04))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def print_result():
    print(
        "mini_ancf_thin_plate: source_status=to_be_done  "
        "implemented_in_source=False  test_result=+0.000000000000"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0.10)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ObjectANCFThinPlate.py -> PyChrono source-placeholder scene")
    if args.no_vis:
        print_result()
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

import argparse

import pychrono.core as chrono
import pychrono.fea as fea


RHO_A = 78.0
EA = 1.0e6
EI = 833.3333333333333
LENGTH = 2.0
ELEMENTS = 32
GRAVITY = 9.81
SECTION_AREA = 1.0e-2
DRAW_RADIUS = 0.025
REFERENCE_UX = -0.5013058140308901
STEP = 1.0e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(x, y, z)


def make_section():
    section = fea.ChBeamSectionCable()
    young_modulus = EA / SECTION_AREA
    section.SetArea(SECTION_AREA)
    section.SetYoungModulus(young_modulus)
    section.SetInertia(EI / young_modulus)
    section.SetDensity(RHO_A / SECTION_AREA)
    section.SetRayleighDamping(0.0)
    section.SetDrawCircularRadius(DRAW_RADIUS)
    return section


def add_mesh_visuals(mesh):
    beam = chrono.ChVisualShapeFEA()
    beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
    beam.SetColormapRange(-2800.0, 2800.0)
    beam.SetSmoothFaces(True)
    beam.SetWireframe(False)
    mesh.AddVisualShapeFEA(beam)

    nodes = chrono.ChVisualShapeFEA()
    nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
    nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
    nodes.SetSymbolsThickness(0.018)
    nodes.SetSymbolsScale(0.028)
    nodes.SetZbufferHide(False)
    mesh.AddVisualShapeFEA(nodes)


def make_segment_body(system, name, a, b, tint, thickness=3):
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


def add_scene_visuals(system):
    anchor = chrono.ChBodyEasySphere(0.065, 1000.0, True, False)
    anchor.SetName("ANCF cable fixed left node")
    anchor.SetFixed(True)
    anchor.EnableCollision(False)
    anchor.GetVisualShape(0).SetColor(color(0.05, 0.09, 0.12))
    system.AddBody(anchor)

    make_segment_body(
        system,
        "ANCF cable undeformed reference line",
        vec(0.0, 0.0, -0.015),
        vec(LENGTH, 0.0, -0.015),
        color(0.52, 0.56, 0.60),
        2,
    )
    make_segment_body(
        system,
        "ANCF cable gravity load direction",
        vec(0.22, 0.10, 0.06),
        vec(0.22, -0.35, 0.06),
        color(0.10, 0.35, 0.95),
        5,
    )
    return anchor


def add_end_marker(system, end_pos):
    marker = chrono.ChBodyEasySphere(0.075, 1000.0, True, False)
    marker.SetName("ANCF cable deformed free-end marker")
    marker.SetFixed(True)
    marker.EnableCollision(False)
    marker.SetPos(end_pos)
    marker.GetVisualShape(0).SetColor(color(0.95, 0.24, 0.08))
    system.AddBody(marker)
    return marker


def make_solver(system):
    solver = chrono.ChSolverSparseQR()
    system.SetSolver(solver)
    solver.UseSparsityPatternLearner(True)
    solver.LockSparsityPattern(True)
    solver.SetVerbose(False)
    return solver


def build_cable_system(title):
    system = chrono.ChSystemSMC()
    system.SetGravitationalAcceleration(vec(0.0, -GRAVITY, 0.0))

    mesh = fea.ChMesh()
    mesh.SetAutomaticGravity(True)

    builder = fea.ChBuilderCableANCF()
    builder.BuildBeam(mesh, make_section(), ELEMENTS, vec(0.0, 0.0, 0.0), vec(LENGTH, 0.0, 0.0))
    nodes = builder.GetLastBeamNodes()

    anchor = add_scene_visuals(system)
    constraint_pos = fea.ChLinkNodeFrame()
    constraint_pos.Initialize(nodes.front(), anchor)
    system.Add(constraint_pos)

    constraint_slope = fea.ChLinkNodeSlopeFrame()
    constraint_slope.Initialize(nodes.front(), anchor)
    constraint_slope.SetDirectionInAbsoluteCoords(vec(1.0, 0.0, 0.0))
    system.Add(constraint_slope)

    add_mesh_visuals(mesh)
    system.Add(mesh)
    make_solver(system)
    system.DoStaticNonlinear(20)

    end_node = nodes.back()
    end_pos = chrono.ChVector3d(end_node.GetPos())
    add_end_marker(system, end_pos)

    for i in range(len(nodes)):
        nodes[i].SetFixed(True)

    ux = end_pos.x - LENGTH
    result = {
        "title": title,
        "node_count": len(nodes),
        "element_count": mesh.GetNumElements(),
        "end_pos": (end_pos.x, end_pos.y, end_pos.z),
        "ux": ux,
        "uy": end_pos.y,
        "reference_error": ux - REFERENCE_UX,
    }
    system._mini_ancf_result = result
    return system, result


def update_visuals(_system):
    return None


def print_result(result, label):
    x, y, z = result["end_pos"]
    print(
        f"{label}: nodes={result['node_count']}  elements={result['element_count']}  "
        f"end_pos=({x:+.12f},{y:+.12f},{z:+.12f})"
    )
    print(
        f"{label}: displacement=({result['ux']:+.12f},{result['uy']:+.12f},+0.000000000000)  "
        f"reference_ux={REFERENCE_UX:+.12f}  reference_error={result['reference_error']:+.3e}"
    )


def run_visual(title, duration, step):
    import pychrono.irrlicht as chronoirr

    system, _result = build_cable_system(title)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle(title)
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(vec(1.00, -2.65, 0.95), vec(1.00, -0.62, 0.0))
    vis.AddTypicalLights()
    while vis.Run() and system.GetChTime() < duration:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def run_main(label, source_name, build_system_func):
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0.10)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()
    title = f"EXUDYN port: {source_name}"
    print(f"{title} -> PyChrono ChElementCableANCF static solve")
    if args.no_vis:
        _system, result = build_system_func()
        print_result(result, label)
    else:
        run_visual(title, args.duration, args.step)

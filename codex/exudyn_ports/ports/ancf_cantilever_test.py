import argparse

import pychrono.core as chrono
import pychrono.fea as fea


# Reproduces EXUDYN Examples/ANCFcantileverTest.py:
# a clamped ANCF cable cantilever with the source steel-like cross section and
# the benchmark end load f = 3*E*I/L^2. Chrono uses ChElementCableANCF for the
# static solve and renders the deformed beam, nodes, clamp, load arrow, and
# free-end marker explicitly.

LENGTH = 2.0
YOUNG_MODULUS = 2.07e11
DENSITY = 7800.0
WIDTH = 0.1
HEIGHT = 0.1
AREA = WIDTH * HEIGHT
INERTIA = WIDTH * HEIGHT**3 / 12.0
TIP_LOAD = 3.0 * YOUNG_MODULUS * INERTIA / LENGTH**2
ELEMENTS = 32
STEP = 1.0e-3
END_TIME = 0.08
REFERENCE_UX_32 = -0.508537277761696
REFERENCE_UY_32 = -1.2072398264650905


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def make_section():
    section = fea.ChBeamSectionCable()
    section.SetArea(AREA)
    section.SetYoungModulus(YOUNG_MODULUS)
    section.SetInertia(INERTIA)
    section.SetDensity(DENSITY)
    section.SetRayleighDamping(0.0)
    section.SetDrawCircularRadius(0.025)
    return section


def add_mesh_visuals(mesh):
    beam = chrono.ChVisualShapeFEA()
    beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
    beam.SetColormapRange(-TIP_LOAD * LENGTH, TIP_LOAD * LENGTH)
    beam.SetSmoothFaces(True)
    beam.SetWireframe(False)
    mesh.AddVisualShapeFEA(beam)

    nodes = chrono.ChVisualShapeFEA()
    nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
    nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
    nodes.SetSymbolsThickness(0.020)
    nodes.SetSymbolsScale(0.040)
    nodes.SetZbufferHide(False)
    mesh.AddVisualShapeFEA(nodes)


def add_segment_body(system, name, point_a, point_b, tint, thickness=3):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    shape = chrono.ChVisualShapeSegment()
    shape.SetColor(tint)
    shape.SetThickness(thickness)
    shape.SetLineGeometry(chrono.ChLineSegment(point_a, point_b))
    body.AddVisualShape(shape)
    system.AddBody(body)
    return body


def add_static_box(system, name, size, pos, tint):
    body = chrono.ChBodyEasyBox(size[0], size[1], size[2], 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(pos)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def add_scene_guides(system):
    add_static_box(
        system,
        "ANCF cantilever fixed clamp",
        (0.08, 0.52, 0.12),
        vec(-0.04, 0.0, 0.0),
        color(0.08, 0.08, 0.09),
    )
    add_static_box(
        system,
        "ANCF cantilever source plot background",
        (2.95, 0.018, 0.018),
        vec(0.95, -1.95, -0.035),
        color(0.12, 0.12, 0.45),
    )
    add_static_box(
        system,
        "ANCF cantilever right plot boundary",
        (0.018, 2.48, 0.018),
        vec(2.50, -0.75, -0.035),
        color(0.12, 0.12, 0.45),
    )
    add_static_box(
        system,
        "ANCF cantilever undeformed reference beam",
        (LENGTH, 0.018, 0.018),
        vec(0.5 * LENGTH, 0.0, -0.050),
        color(0.54, 0.56, 0.60),
    )


def add_tip_visuals(system, tip_pos):
    marker = chrono.ChBodyEasySphere(0.065, 1000.0, True, False)
    marker.SetName("ANCF cantilever loaded free-end marker")
    marker.SetFixed(True)
    marker.EnableCollision(False)
    marker.SetPos(tip_pos)
    marker.GetVisualShape(0).SetColor(color(0.95, 0.22, 0.05))
    system.AddBody(marker)

    start = tip_pos + vec(0.20, 0.25, 0.0)
    end = tip_pos + vec(0.20, -0.35, 0.0)
    tint = color(0.95, 0.12, 0.08)
    add_segment_body(system, "ANCF cantilever tip load arrow", start, end, tint, 5)
    add_segment_body(system, "ANCF cantilever tip load arrow head a", end, end + vec(-0.055, 0.090, 0.0), tint, 4)
    add_segment_body(system, "ANCF cantilever tip load arrow head b", end, end + vec(0.055, 0.090, 0.0), tint, 4)
    return marker


def add_node_markers(system, nodes):
    markers = []
    for i in range(len(nodes)):
        pos = chrono.ChVector3d(nodes[i].GetPos())
        pos.z += 0.055
        marker = chrono.ChBodyEasySphere(0.038 if i % 4 == 0 else 0.026, 1000.0, True, False)
        marker.SetName(f"ANCF cantilever visible node {i:02d}")
        marker.SetFixed(True)
        marker.EnableCollision(False)
        marker.SetPos(pos)
        tint = color(0.05, 0.05, 0.055) if i == 0 else color(0.08, 0.22, 0.78)
        if i == len(nodes) - 1:
            tint = color(0.95, 0.22, 0.05)
        marker.GetVisualShape(0).SetColor(tint)
        system.AddBody(marker)
        markers.append(marker)
    return markers


def make_solver(system):
    solver = chrono.ChSolverSparseQR()
    solver.UseSparsityPatternLearner(True)
    solver.LockSparsityPattern(True)
    solver.SetVerbose(False)
    system.SetSolver(solver)
    return solver


def build_system():
    system = chrono.ChSystemSMC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))

    mesh = fea.ChMesh()
    mesh.SetAutomaticGravity(False)
    builder = fea.ChBuilderCableANCF()
    builder.BuildBeam(mesh, make_section(), ELEMENTS, vec(0, 0, 0), vec(LENGTH, 0, 0))
    nodes = builder.GetLastBeamNodes()
    nodes.front().SetFixed(True)
    nodes.back().SetForce(vec(0, -TIP_LOAD, 0))

    add_mesh_visuals(mesh)
    system.Add(mesh)
    add_scene_guides(system)
    make_solver(system)
    system.DoStaticNonlinear(40)

    tip_pos = chrono.ChVector3d(nodes.back().GetPos())
    node_markers = add_node_markers(system, nodes)
    add_tip_visuals(system, tip_pos)

    for i in range(len(nodes)):
        nodes[i].SetFixed(True)

    result = {
        "nodes": len(nodes),
        "elements": mesh.GetNumElements(),
        "tip_pos": (tip_pos.x, tip_pos.y, tip_pos.z),
        "ux": tip_pos.x - LENGTH,
        "uy": tip_pos.y,
        "reference_error_x": tip_pos.x - LENGTH - REFERENCE_UX_32,
        "reference_error_y": tip_pos.y - REFERENCE_UY_32,
        "tip_load": TIP_LOAD,
    }
    system._ancf_cantilever_result = result
    system._ancf_cantilever_node_markers = node_markers
    return system, result


def update_visuals(_system):
    return None


def simulate(_duration, _step):
    return build_system()


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _result = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: ANCFcantileverTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.20, -4.30, 1.15), chrono.ChVector3d(1.00, -0.72, 0.0))
    vis.AddTypicalLights()

    while vis.Run() and system.GetChTime() < duration:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)


def print_state(result):
    x, y, z = result["tip_pos"]
    print(
        f"nodes={result['nodes']}  elements={result['elements']}  tip_load={result['tip_load']:.6g}  "
        f"tip=({x:+.12f},{y:+.12f},{z:+.12f})"
    )
    print(
        f"displacement=({result['ux']:+.12f},{result['uy']:+.12f},+0.000000000000)  "
        f"reference32=({REFERENCE_UX_32:+.12f},{REFERENCE_UY_32:+.12f})  "
        f"error=({result['reference_error_x']:+.3e},{result['reference_error_y']:+.3e})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: ANCFcantileverTest.py -> PyChrono ChElementCableANCF loaded cantilever")
    if args.no_vis:
        _system, result = simulate(args.duration, args.step)
        print_state(result)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

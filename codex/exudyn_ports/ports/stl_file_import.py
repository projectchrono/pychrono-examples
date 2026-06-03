import argparse
import struct
from pathlib import Path

import pychrono.core as chrono


# Reproduces the intent of EXUDYN Examples/stlFileImport.py:
# create a simple STL brick, import it back as a graphics mesh, attach it to a
# pendulum body, and show the revolute joint and body basis.  The physical model
# is a one-link pendulum; the important parity point is that the visible body is
# loaded through Chrono's STL triangle-mesh import path, not drawn as a plain box.

LENGTH = 1.0
WIDTH = 0.1
DENSITY = 5000.0
MASS = DENSITY * LENGTH * WIDTH * WIDTH
STEP = 1.0e-3
END_TIME = 4.0
STL_PATH = Path("/tmp/pychrono_exudyn_stl_import_box.stl")


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def write_box_stl(path):
    hx = 0.5 * LENGTH
    hy = 0.5 * WIDTH
    hz = 0.5 * WIDTH
    vertices = {
        "000": (-hx, -hy, -hz),
        "001": (-hx, -hy, hz),
        "010": (-hx, hy, -hz),
        "011": (-hx, hy, hz),
        "100": (hx, -hy, -hz),
        "101": (hx, -hy, hz),
        "110": (hx, hy, -hz),
        "111": (hx, hy, hz),
    }
    faces = [
        ("-x", (vertices["000"], vertices["001"], vertices["011"], vertices["010"]), (-1, 0, 0)),
        ("+x", (vertices["100"], vertices["110"], vertices["111"], vertices["101"]), (1, 0, 0)),
        ("-y", (vertices["000"], vertices["100"], vertices["101"], vertices["001"]), (0, -1, 0)),
        ("+y", (vertices["010"], vertices["011"], vertices["111"], vertices["110"]), (0, 1, 0)),
        ("-z", (vertices["000"], vertices["010"], vertices["110"], vertices["100"]), (0, 0, -1)),
        ("+z", (vertices["001"], vertices["101"], vertices["111"], vertices["011"]), (0, 0, 1)),
    ]
    triangles = []
    for _, quad, normal in faces:
        triangles.append((normal, quad[0], quad[1], quad[2]))
        triangles.append((normal, quad[0], quad[2], quad[3]))

    with path.open("wb") as out:
        header = b"PyChrono EXUDYN stlFileImport box"
        out.write(header + b" " * (80 - len(header)))
        out.write(struct.pack("<I", len(triangles)))
        for normal, a, b, c in triangles:
            out.write(struct.pack("<3f", *normal))
            for p in (a, b, c):
                out.write(struct.pack("<3f", *p))
            out.write(struct.pack("<H", 0))


def add_cylinder_between(body, p0, p1, radius, tint):
    segment = chrono.ChLineSegment(p0, p1)
    cylinder = chrono.ChVisualShapeCylinder(radius, segment.GetLength())
    cylinder.SetColor(tint)
    body.AddVisualShape(cylinder, segment.GetFrame())
    return cylinder


def add_body_axes(body, length=0.20, radius=0.006):
    add_cylinder_between(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(length, 0, 0), radius, color(0.92, 0.12, 0.08))
    add_cylinder_between(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, length, 0), radius, color(0.08, 0.70, 0.18))
    add_cylinder_between(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, length), radius, color(0.10, 0.28, 0.90))


def add_box_edges(body):
    hx = 0.5 * LENGTH
    hy = 0.5 * WIDTH
    hz = 0.5 * WIDTH
    corners = [
        chrono.ChVector3d(x, y, z)
        for x in (-hx, hx)
        for y in (-hy, hy)
        for z in (-hz, hz)
    ]
    for a, b in (
        (0, 1), (0, 2), (0, 4), (3, 1), (3, 2), (3, 7),
        (5, 1), (5, 4), (5, 7), (6, 2), (6, 4), (6, 7),
    ):
        edge = chrono.ChVisualShapeSegment()
        edge.SetLineGeometry(chrono.ChLineSegment(corners[a], corners[b]))
        edge.SetThickness(3)
        edge.SetColor(color(0.03, 0.03, 0.04))
        body.AddVisualShape(edge)


def build_system():
    write_box_stl(STL_PATH)
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    ground = chrono.ChBodyEasyBox(1.35, 0.16, 0.012, 1000, True, False)
    ground.SetName("stl import visible ground reference")
    ground.SetFixed(True)
    ground.SetPos(chrono.ChVector3d(0.45, -0.18, -0.075))
    ground.GetVisualShape(0).SetColor(color(0.78, 0.78, 0.74))
    ground.GetVisualShape(0).SetOpacity(0.32)
    pivot = chrono.ChVisualShapeSphere(0.035)
    pivot.SetColor(color(0.05, 0.05, 0.05))
    ground.AddVisualShape(pivot, chrono.ChFramed(chrono.ChVector3d(-0.45, 0.18, 0.075)))
    system.AddBody(ground)

    body = chrono.ChBody()
    body.SetName("pendulum body using imported STL mesh")
    body.EnableCollision(False)
    body.SetMass(MASS)
    body.SetInertiaXX(chrono.ChVector3d(
        MASS * (WIDTH * WIDTH + WIDTH * WIDTH) / 12.0,
        MASS * (LENGTH * LENGTH + WIDTH * WIDTH) / 12.0,
        MASS * (LENGTH * LENGTH + WIDTH * WIDTH) / 12.0,
    ))
    body.SetPos(chrono.ChVector3d(0.5 * LENGTH, 0, 0))

    mesh = chrono.ChTriangleMeshConnected()
    if not mesh.LoadSTLMesh(str(STL_PATH), True):
        raise RuntimeError(f"could not load generated STL file: {STL_PATH}")
    shape = chrono.ChVisualShapeTriangleMesh()
    shape.SetMesh(mesh)
    shape.SetColor(color(0.10, 0.44, 0.92))
    shape.SetBackfaceCull(False)
    body.AddVisualShape(shape)
    add_box_edges(body)
    add_body_axes(body, length=0.22, radius=0.005)
    com = chrono.ChVisualShapeSphere(0.025)
    com.SetColor(color(0.95, 0.78, 0.08))
    body.AddVisualShape(com)
    system.AddBody(body)

    joint = chrono.ChLinkLockRevolute()
    joint.SetName("STL pendulum revolute joint")
    joint.Initialize(body, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    system.AddLink(joint)

    system._stl_import_items = {"body": body, "ground": ground, "joint": joint}
    return system, body, ground


def simulate(duration, step):
    system, body, ground = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, body, ground


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, body, ground = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: stlFileImport.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.42, -1.18, 0.72), chrono.ChVector3d(0.32, -0.12, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, body)
            next_log += 0.5


def print_state(system, body):
    pos = body.GetPos()
    omega = body.GetAngVelParent()
    print(
        f"t={system.GetChTime():6.3f}  "
        f"body=({pos.x:+.5f}, {pos.y:+.5f}, {pos.z:+.5f})  "
        f"omega=({omega.x:+.5f}, {omega.y:+.5f}, {omega.z:+.5f})  "
        f"stl='{STL_PATH}'"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: stlFileImport.py -> PyChrono generated STL pendulum import")
    if args.no_vis:
        system, body, ground = simulate(args.duration, args.step)
        print_state(system, body)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

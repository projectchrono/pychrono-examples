import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import helical_line


# Reproduces the intent of EXUDYN TestModels/sparseMatrixSpringDamperTest.py:
# a 72 x 6 marker lattice with mass points, horizontal/vertical/diagonal
# spring-dampers, a small downward load on every free point, and one extra
# coordinate constraint near the fixed edge.  EXUDYN uses this as a sparse
# static-solver test; this PyChrono version keeps the full topology and renders
# every connector as a real spring coil.

N_COLS = 72
N_ROWS = 6
MASS = 10.0
STIFFNESS = 4000.0
DAMPING = 10.0
LOAD_Y = -0.025
REST = 1.0
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def _visual_items(system):
    if not hasattr(system, "_sparse_lattice_offset_springs"):
        system._sparse_lattice_offset_springs = []
    return system._sparse_lattice_offset_springs


class OffsetSpringVisual:
    def __init__(self, system, radius, resolution, turns, tint, offset_z):
        self.radius = radius
        self.resolution = resolution
        self.turns = turns
        self.offset_z = offset_z
        self.body = chrono.ChBody()
        self.body.SetFixed(True)
        self.body.EnableCollision(False)
        system.AddBody(self.body)

        self.shape = chrono.ChVisualShapeLine()
        self.shape.SetMutable(True)
        self.shape.SetColor(tint)
        self.shape.SetThickness(1)
        self.body.AddVisualShape(self.shape)

    def update(self, point_a, point_b):
        self.shape.SetLineGeometry(
            helical_line(
                chrono.ChVector3d(point_a.x, point_a.y, point_a.z + self.offset_z),
                chrono.ChVector3d(point_b.x, point_b.y, point_b.z + self.offset_z),
                self.radius,
                self.resolution,
                self.turns,
            )
        )
        self.body.UpdateVisualModel()


def attach_offset_spring_visual(system, spring):
    visual = OffsetSpringVisual(system, 0.055, 28, 5, color(0.88, 0.18, 0.08), -0.08)
    _visual_items(system).append((spring, visual))
    visual.update(spring.GetPoint1Abs(), spring.GetPoint2Abs())
    return visual


def marker_index(row, col):
    return row * N_COLS + col


def make_marker_body(row, col):
    fixed = col == 0
    radius = 0.090 if fixed else 0.075
    body = chrono.ChBodyEasySphere(radius, 1000, True, False)
    body.SetName(f"sparse lattice {'support' if fixed else 'mass'} r{row:02d} c{col:02d}")
    body.SetMass(MASS if not fixed else 1.0)
    body.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    body.SetFixed(fixed)
    body.SetPos(chrono.ChVector3d(col * REST, row * REST, 0))
    body.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.08) if fixed else color(0.05, 0.65, 0.22))
    return body


def add_load(body):
    force = chrono.ChForce()
    force.SetF_y(chrono.ChFunctionConst(LOAD_Y))
    body.AddForce(force)
    return force


def add_spring(system, bodies, a, b, rest_length, name):
    spring = chrono.ChLinkTSDA()
    spring.SetName(name)
    spring.Initialize(bodies[a], bodies[b], True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
    spring.SetRestLength(rest_length)
    spring.SetSpringCoefficient(STIFFNESS)
    spring.SetDampingCoefficient(DAMPING)
    system.AddLink(spring)

    spring_shape = chrono.ChVisualShapeSpring(0.055, 36, 5)
    spring_shape.SetColor(color(0.88, 0.18, 0.08))
    spring.AddVisualShape(spring_shape)
    attach_offset_spring_visual(system, spring)
    return spring


def add_coordinate_constraint(system, ground, body):
    # EXUDYN constrains the y-coordinate of node 1 while leaving x free.
    # A Chrono prismatic-x guide gives the same visual/mechanical role.
    frame = chrono.ChFramed(body.GetPos(), chrono.QUNIT)
    guide = chrono.ChLinkLockPrismatic()
    guide.SetName("sparse lattice y-coordinate constraint at node 1")
    guide.Initialize(body, ground, frame)
    system.AddLink(guide)

    rail = chrono.ChBodyEasyBox(1.0, 0.025, 0.025, 1000, True, False)
    rail.SetName("visible coordinate-constraint guide")
    rail.SetFixed(True)
    rail.SetPos(body.GetPos() + chrono.ChVector3d(0.40, -0.10, 0))
    rail.GetVisualShape(0).SetColor(color(0.45, 0.45, 0.45))
    system.AddBody(rail)
    return guide, rail


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    ground = chrono.ChBody()
    ground.SetName("sparse lattice hidden ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)

    bodies = []
    loads = []
    for row in range(N_ROWS):
        for col in range(N_COLS):
            body = make_marker_body(row, col)
            system.AddBody(body)
            bodies.append(body)
            if col > 0:
                loads.append(add_load(body))

    springs = []
    for row in range(N_ROWS - 1):
        for col in range(N_COLS - 1):
            springs.append(
                add_spring(
                    system,
                    bodies,
                    marker_index(row, col),
                    marker_index(row, col + 1),
                    REST,
                    f"horizontal spring r{row:02d} c{col:02d}",
                )
            )
            springs.append(
                add_spring(
                    system,
                    bodies,
                    marker_index(row, col),
                    marker_index(row + 1, col),
                    REST,
                    f"vertical spring r{row:02d} c{col:02d}",
                )
            )
            springs.append(
                add_spring(
                    system,
                    bodies,
                    marker_index(row, col),
                    marker_index(row + 1, col + 1),
                    math.sqrt(2.0) * REST,
                    f"diagonal spring r{row:02d} c{col:02d}",
                )
            )

    top_row = N_ROWS - 1
    for col in range(N_COLS - 1):
        springs.append(
            add_spring(
                system,
                bodies,
                marker_index(top_row, col),
                marker_index(top_row, col + 1),
                REST,
                f"top-row spring c{col:02d}",
            )
        )

    right_col = N_COLS - 1
    for row in range(N_ROWS - 1):
        springs.append(
            add_spring(
                system,
                bodies,
                marker_index(row, right_col),
                marker_index(row + 1, right_col),
                REST,
                f"right-column spring r{row:02d}",
            )
        )

    guide, rail = add_coordinate_constraint(system, ground, bodies[marker_index(0, 1)])
    system._sparse_matrix_items = {
        "bodies": bodies,
        "loads": loads,
        "springs": springs,
        "guide": guide,
        "rail": rail,
    }
    return system, bodies, springs, guide


def update_visuals(system):
    for spring, visual in getattr(system, "_sparse_lattice_offset_springs", []):
        visual.update(spring.GetPoint1Abs(), spring.GetPoint2Abs())


def simulate(duration, step):
    system, bodies, springs, guide = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    return system, bodies, springs, guide


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, bodies, springs, guide = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1180, 720)
    vis.SetWindowTitle("EXUDYN port: sparseMatrixSpringDamperTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(10.0, -12.0, 14.0), chrono.ChVector3d(10.0, 2.5, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, bodies, springs)
            next_log += 0.05


def print_state(system, bodies, springs):
    tip = bodies[marker_index(0, N_COLS - 2)]
    center = bodies[marker_index(N_ROWS // 2, N_COLS // 2)]
    print(
        f"t={system.GetChTime():6.3f}  "
        f"tip=({tip.GetPos().x:+.4f}, {tip.GetPos().y:+.4f}, {tip.GetPos().z:+.4f})  "
        f"center_y={center.GetPos().y:+.5f}  "
        f"markers={len(bodies)}  springs={len(springs)}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0.20)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: sparseMatrixSpringDamperTest.py -> PyChrono sparse spring lattice")
    if args.no_vis:
        system, bodies, springs, guide = simulate(args.duration, args.step)
        print_state(system, bodies, springs)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

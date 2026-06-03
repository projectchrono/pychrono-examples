import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN TestModels/rigidBody2Dtest.py:
# compare rigid pendulum formulations with different reference/COM conventions.
# Chrono represents rigid bodies at their COM, so this port keeps three visible
# pendulum rows with equivalent COM physics, bushing support compliance, and
# explicit support-spring visuals.

LENGTH = 1.0
WIDTH = 0.10
MASS = 12.0
GRAVITY = 9.81
STIFFNESS = 5000.0 * 100.0
DAMPING = 50.0 * 100.0
STEP = 1e-3
END_TIME = 0.4

ROWS = [
    ("COM-at-body-origin formulation", 0.42, 0.00, (0.10, 0.42, 0.88)),
    ("offset-COM RigidBody2D formulation", 0.00, 0.22, (0.88, 0.14, 0.10)),
    ("3D rigid-body COM formulation", -0.42, 0.44, (0.14, 0.62, 0.22)),
]


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def diagonal_matrix(values):
    matrix = chrono.ChMatrix66d()
    matrix.SetZero()
    for i, value in enumerate(values):
        matrix.SetItem(i, i, value)
    return matrix


def make_anchor(system, y_offset, z_offset):
    anchor = chrono.ChBodyEasySphere(0.055, 1000, True, False)
    anchor.SetName(f"rigidBody2Dtest support anchor z{z_offset:.2f}")
    anchor.SetFixed(True)
    anchor.SetPos(chrono.ChVector3d(-LENGTH, LENGTH + y_offset, z_offset))
    anchor.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.08))
    system.AddBody(anchor)
    return anchor


def make_pendulum(system, label, y_offset, z_offset, tint):
    body = chrono.ChBodyEasyBox(LENGTH, WIDTH, WIDTH, 1000, True, False)
    body.SetName(label)
    body.SetMass(MASS)
    body.SetInertiaXX(chrono.ChVector3d(0.03, MASS / 12.0 * LENGTH * LENGTH, MASS / 12.0 * LENGTH * LENGTH))
    body.SetPos(chrono.ChVector3d(-0.5 * LENGTH, LENGTH + y_offset, z_offset))
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(color(*tint))

    joint_marker = chrono.ChVisualShapeSphere(0.055)
    joint_marker.SetColor(color(0.08, 0.08, 0.08))
    body.AddVisualShape(joint_marker, chrono.ChFramed(chrono.ChVector3d(-0.5 * LENGTH, 0, 0)))

    com_marker = chrono.ChVisualShapeSphere(0.040)
    com_marker.SetColor(color(0.96, 0.84, 0.10))
    body.AddVisualShape(com_marker)

    system.AddBody(body)
    return body


class BodySegmentVisual:
    def __init__(self, system, body, local_a, local_b, tint):
        self.source_body = body
        self.local_a = local_a
        self.local_b = local_b
        self.body = chrono.ChBody()
        self.body.SetName(f"{body.GetName()} explicit rod visual")
        self.body.SetFixed(True)
        self.body.EnableCollision(False)
        self.shape = chrono.ChVisualShapeSegment()
        self.shape.SetMutable(True)
        self.shape.SetColor(tint)
        self.shape.SetThickness(8)
        self.body.AddVisualShape(self.shape)
        system.AddBody(self.body)
        self.update()

    def update(self):
        point_a = self.source_body.TransformPointLocalToParent(self.local_a)
        point_b = self.source_body.TransformPointLocalToParent(self.local_b)
        self.shape.SetLineGeometry(chrono.ChLineSegment(point_a, point_b))
        self.body.UpdateVisualModel()


class BodyBeadVisual:
    def __init__(self, system, body, tint):
        self.source_body = body
        self.local_points = [
            chrono.ChVector3d(-0.45 * LENGTH + i * 0.18 * LENGTH, 0, 0)
            for i in range(6)
        ]
        self.beads = []
        for i, _ in enumerate(self.local_points):
            bead = chrono.ChBodyEasySphere(0.038, 1000, True, False)
            bead.SetName(f"{body.GetName()} colored body bead {i + 1}")
            bead.SetFixed(True)
            bead.GetVisualShape(0).SetColor(tint)
            system.AddBody(bead)
            self.beads.append(bead)
        self.update()

    def update(self):
        for bead, local_point in zip(self.beads, self.local_points):
            bead.SetPos(self.source_body.TransformPointLocalToParent(local_point))


def add_body_segment_visual(system, body, tint):
    if not hasattr(system, "_rigid_body_2d_segment_visuals"):
        system._rigid_body_2d_segment_visuals = []
    visual = BodySegmentVisual(
        system,
        body,
        chrono.ChVector3d(-0.5 * LENGTH, 0, 0),
        chrono.ChVector3d(0.5 * LENGTH, 0, 0),
        tint,
    )
    system._rigid_body_2d_segment_visuals.append(visual)
    system._rigid_body_2d_segment_visuals.append(BodyBeadVisual(system, body, tint))
    return visual


def add_support_bushing(system, anchor_body, pendulum, y_offset, z_offset):
    frame = chrono.ChFramed(chrono.ChVector3d(-LENGTH, LENGTH + y_offset, z_offset), chrono.QUNIT)
    k = diagonal_matrix([STIFFNESS, STIFFNESS, 0, 0, 0, 0])
    r = diagonal_matrix([DAMPING, DAMPING, 0, 0, 0, 0])
    bushing = chrono.ChLinkBushing()
    bushing.SetName(f"rigidBody2Dtest xy support bushing z{z_offset:.2f}")
    bushing.Initialize(anchor_body, pendulum, frame, k, r)
    system.AddLink(bushing)

    visual_spring = chrono.ChLinkTSDA()
    visual_spring.SetName(f"rigidBody2Dtest visible support coil z{z_offset:.2f}")
    visual_spring.Initialize(
        pendulum,
        anchor_body,
        True,
        chrono.ChVector3d(-0.5 * LENGTH, 0, 0),
        chrono.ChVector3d(0, 0, 0),
    )
    visual_spring.SetSpringCoefficient(0)
    visual_spring.SetDampingCoefficient(0)
    system.AddLink(visual_spring)
    spring_shape = chrono.ChVisualShapeSpring(0.045, 70, 8)
    spring_shape.SetColor(color(0.90, 0.20, 0.08))
    visual_spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, visual_spring, 0.045, 70, 8, color(0.90, 0.20, 0.08))
    return bushing, visual_spring


def add_gravity_load(body):
    force = chrono.ChForce()
    force.SetF_y(chrono.ChFunctionConst(-MASS * GRAVITY))
    body.AddForce(force)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    plate = chrono.ChBodyEasyBox(1.55, 1.45, 0.035, 1000, True, False)
    plate.SetName("visible rigidBody2Dtest reference plate")
    plate.SetFixed(True)
    plate.SetPos(chrono.ChVector3d(-0.45, 0.55, -2.0))
    plate.GetVisualShape(0).SetColor(color(0.80, 0.80, 0.76))
    plate.GetVisualShape(0).SetOpacity(0.16)
    system.AddBody(plate)

    bodies = []
    bushings = []
    for label, y_offset, z_offset, tint in ROWS:
        anchor = make_anchor(system, y_offset, z_offset)
        body = make_pendulum(system, label, y_offset, z_offset, tint)
        add_body_segment_visual(system, body, color(*tint))
        add_gravity_load(body)
        bushing, visual_spring = add_support_bushing(system, anchor, body, y_offset, z_offset)
        bodies.append(body)
        bushings.append(bushing)

    update_visuals(system)
    return system, bodies, bushings


def update_visuals(system):
    update_system_visuals(system)
    for visual in getattr(system, "_rigid_body_2d_segment_visuals", []):
        visual.update()


def simulate(duration, step):
    system, bodies, bushings = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, bodies, bushings


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, bodies, bushings = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: rigidBody2Dtest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(-0.35, 1.0, 4.0), chrono.ChVector3d(-0.45, 1.0, 0.18))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, bodies)
            next_log += 0.1


def yaw_z(body):
    q = body.GetRot()
    return math.atan2(2.0 * (q.e0 * q.e3 + q.e1 * q.e2), 1.0 - 2.0 * (q.e2 * q.e2 + q.e3 * q.e3))


def print_state(system, bodies):
    phis = [yaw_z(body) for body in bodies]
    coms = [body.GetPos() for body in bodies]
    metric = sum(phis) + sum(math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z) for p in coms)
    print(
        f"t={system.GetChTime():6.3f}  "
        f"phis=({phis[0]:+.5f}, {phis[1]:+.5f}, {phis[2]:+.5f})  "
        f"com0=({coms[0].x:+.4f}, {coms[0].y:+.4f}, {coms[0].z:+.4f})  "
        f"metric={metric:.6f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: rigidBody2Dtest.py -> PyChrono COM pendulum comparison")
    if args.no_vis:
        system, bodies, bushings = simulate(args.duration, args.step)
        print_state(system, bodies)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

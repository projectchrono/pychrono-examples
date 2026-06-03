import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN TestModels/rigidBodySpringDamperIntrinsic.py:
# three rigid-body spring-damper connector cases with different marker/body
# orderings. Chrono bushings carry the six-DOF translational/rotational
# compliance, and zero-force TSDA links provide explicit coil visuals between
# the connector endpoints.

LENGTH = 1.0
WIDTH = 0.10
HEIGHT = 0.20
DENSITY = 1000.0
STEP = 0.002
END_TIME = 1.0

K = 10000.0
KR = 250.0
STIFFNESS = [0.4 * K, 0.5 * K, 0.7 * K, KR, 2.0 * KR, 1.3 * KR]
DAMPING = [0, 0, 0, 0, 0, 0]
INITIAL_VELOCITY = chrono.ChVector3d(0.0, 1.0, 2.0)
INITIAL_OMEGA = chrono.ChVector3d(8.5, 10.5, 16.0)

ROW_BODY_GROUND = -0.60
ROW_REVERSED_MARKERS = 0.15
ROW_FREE_PAIR = 1.10


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def diagonal_matrix(values):
    matrix = chrono.ChMatrix66d()
    matrix.SetZero()
    for i, value in enumerate(values):
        matrix.SetItem(i, i, value)
    return matrix


def make_body(name, position, tint, initial_velocity=None, initial_omega=None):
    body = chrono.ChBodyEasyBox(LENGTH, WIDTH, HEIGHT, DENSITY, True, False)
    body.SetName(name)
    body.SetPos(position)
    body.GetVisualShape(0).SetColor(tint)
    if initial_velocity is not None:
        body.SetPosDt(initial_velocity)
    if initial_omega is not None:
        body.SetAngVelParent(initial_omega)
    return body


def make_ground(name, y_position):
    ground = chrono.ChBodyEasyBox(LENGTH, WIDTH, 0.25, DENSITY, True, False)
    ground.SetName(name)
    ground.SetFixed(True)
    ground.SetPos(chrono.ChVector3d(0.0, y_position, 0.0))
    ground.GetVisualShape(0).SetColor(color(0.12, 0.25, 0.82))
    return ground


def add_endpoint_marker(body, local_position, tint):
    marker = chrono.ChVisualShapeSphere(0.055)
    marker.SetColor(tint)
    body.AddVisualShape(marker, chrono.ChFramed(local_position))


def add_bushing(system, body_a, body_b, frame_position, name):
    bushing = chrono.ChLinkBushing()
    bushing.SetName(name)
    bushing.Initialize(
        body_a,
        body_b,
        chrono.ChFramed(frame_position),
        diagonal_matrix(STIFFNESS),
        diagonal_matrix(DAMPING),
    )
    system.AddLink(bushing)
    return bushing


def add_visual_connector(system, body_a, body_b, local_a, local_b, name, tint):
    spring = chrono.ChLinkTSDA()
    spring.SetName(name)
    spring.Initialize(body_a, body_b, True, local_a, local_b)
    spring.SetSpringCoefficient(0.0)
    spring.SetDampingCoefficient(0.0)
    system.AddLink(spring)

    add_endpoint_marker(body_a, local_a, color(0.96, 0.86, 0.10))
    add_endpoint_marker(body_b, local_b, color(0.96, 0.86, 0.10))

    spring_shape = chrono.ChVisualShapeSpring(0.075, 90, 10)
    spring_shape.SetColor(tint)
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, spring, 0.075, 90, 10, tint)
    return spring


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    plate = chrono.ChBodyEasyBox(2.15, 2.25, 0.025, DENSITY, True, False)
    plate.SetName("visible rigid-body spring-damper reference plate")
    plate.SetFixed(True)
    plate.SetPos(chrono.ChVector3d(0.5, 0.28, -0.22))
    plate.GetVisualShape(0).SetColor(color(0.80, 0.80, 0.76))
    plate.GetVisualShape(0).SetOpacity(0.28)
    system.AddBody(plate)

    ground_1 = make_ground("body-ground connector fixed body", ROW_BODY_GROUND)
    ground_2 = make_ground("reversed-marker connector fixed body", ROW_REVERSED_MARKERS)
    system.AddBody(ground_1)
    system.AddBody(ground_2)

    body_1 = make_body(
        "rigid body spring-damper body 1",
        chrono.ChVector3d(LENGTH, ROW_BODY_GROUND, 0.0),
        color(0.88, 0.16, 0.12),
        INITIAL_VELOCITY,
        INITIAL_OMEGA,
    )
    body_2 = make_body(
        "rigid body spring-damper body 2 reversed order",
        chrono.ChVector3d(LENGTH, ROW_REVERSED_MARKERS, 0.0),
        color(0.12, 0.62, 0.24),
        INITIAL_VELOCITY,
        INITIAL_OMEGA,
    )
    body_3a = make_body(
        "free pair spring-damper body A",
        chrono.ChVector3d(0.0, ROW_FREE_PAIR, 0.0),
        color(0.88, 0.16, 0.12),
        None,
        INITIAL_OMEGA,
    )
    body_3b = make_body(
        "free pair spring-damper body B",
        chrono.ChVector3d(LENGTH, ROW_FREE_PAIR, 0.0),
        color(0.12, 0.62, 0.24),
        None,
        chrono.ChVector3d(-INITIAL_OMEGA.x, -INITIAL_OMEGA.y, -INITIAL_OMEGA.z),
    )

    for body in (body_1, body_2, body_3a, body_3b):
        system.AddBody(body)

    bushings = [
        add_bushing(system, ground_1, body_1, chrono.ChVector3d(0.5, ROW_BODY_GROUND, 0.0), "ground-to-body bushing"),
        add_bushing(system, body_2, ground_2, chrono.ChVector3d(0.5, ROW_REVERSED_MARKERS, 0.0), "reversed-order bushing"),
        add_bushing(system, body_3a, body_3b, chrono.ChVector3d(0.5, ROW_FREE_PAIR, 0.0), "free-pair bushing"),
    ]

    visual_springs = [
        add_visual_connector(
            system,
            body_1,
            ground_1,
            chrono.ChVector3d(-0.5 * LENGTH, 0, 0),
            chrono.ChVector3d(0.5 * LENGTH, 0, 0),
            "ground-to-body visible coil",
            color(0.90, 0.20, 0.10),
        ),
        add_visual_connector(
            system,
            body_2,
            ground_2,
            chrono.ChVector3d(-0.5 * LENGTH, 0, 0),
            chrono.ChVector3d(0.5 * LENGTH, 0, 0),
            "reversed-order visible coil",
            color(0.90, 0.20, 0.10),
        ),
        add_visual_connector(
            system,
            body_3a,
            body_3b,
            chrono.ChVector3d(0.5 * LENGTH, 0, 0),
            chrono.ChVector3d(-0.5 * LENGTH, 0, 0),
            "free-pair visible coil",
            color(0.90, 0.20, 0.10),
        ),
    ]

    system._rigid_body_spring_damper_items = {
        "bodies": (body_1, body_2, body_3a, body_3b),
        "bushings": bushings,
        "visual_springs": visual_springs,
    }
    update_visuals(system)
    return system, body_1, body_2, body_3a, body_3b, bushings


def update_visuals(system):
    update_system_visuals(system)


def simulate(duration, step):
    system, body_1, body_2, body_3a, body_3b, bushings = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, body_1, body_2, body_3a, body_3b, bushings


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, body_1, body_2, body_3a, body_3b, bushings = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: rigidBodySpringDamperIntrinsic.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.55, -3.0, 2.0), chrono.ChVector3d(0.45, 0.25, 0.0))
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
            print_state(system, body_1, body_2, body_3a)
            next_log += 0.1


def norm3(vector):
    return math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)


def print_state(system, body_1, body_2, body_3a):
    p1 = body_1.GetPos() - chrono.ChVector3d(LENGTH, ROW_BODY_GROUND, 0.0)
    p2 = body_2.GetPos() - chrono.ChVector3d(LENGTH, ROW_REVERSED_MARKERS, 0.0)
    omega3 = body_3a.GetAngVelParent()
    result = norm3(p1) + norm3(p2) + 0.01 * norm3(omega3)
    print(
        f"t={system.GetChTime():6.3f}  "
        f"|p1|={norm3(p1):.6f}  |p2|={norm3(p2):.6f}  "
        f"omega3=({omega3.x:+.3f}, {omega3.y:+.3f}, {omega3.z:+.3f})  "
        f"result={result:.6f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: rigidBodySpringDamperIntrinsic.py -> PyChrono six-DOF bushing test")
    if args.no_vis:
        system, body_1, body_2, body_3a, body_3b, bushings = simulate(args.duration, args.step)
        print_state(system, body_1, body_2, body_3a)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

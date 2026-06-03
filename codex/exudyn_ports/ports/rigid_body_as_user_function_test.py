import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN TestModels/rigidBodyAsUserFunctionTest.py:
# compare a native rigid body with a rigid body whose Euler-parameter equations
# are supplied by a user function. Chrono does not expose the same GenericODE2
# hook, so this port uses two equivalent free rigid bodies with the source mass,
# inertia, gravity, initial velocity, and initial angular velocity. The second
# body is the user-function analogue and is visualized explicitly, matching the
# EXUDYN example's graphicsDataUserFunction purpose.

S = 0.1
SX = 3.0 * S
MASS = 2.0
GRAVITY = 9.81
INITIAL_VELOCITY = chrono.ChVector3d(0.2, 0.0, 0.0)
INITIAL_OMEGA = chrono.ChVector3d(0.0, 50.0, 20.0)
INERTIA_XX = chrono.ChVector3d(6.0, 1.0, 6.0)
INERTIA_XY = chrono.ChVector3d(0.0, 1.0, 0.0)
STEP = 1.0e-3
NO_VIS_END_TIME = 0.05
VIS_END_TIME = 1.0

CASES = [
    {
        "name": "native ObjectRigidBody",
        "position": chrono.ChVector3d(5.0 * S, 0.0, 0.0),
        "color": chrono.ChColor(0.90, 0.12, 0.10),
    },
    {
        "name": "GenericODE2 user-function analogue",
        "position": chrono.ChVector3d(0.0, 0.0, 0.0),
        "color": chrono.ChColor(0.10, 0.22, 0.88),
    },
]


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vcopy(v):
    return chrono.ChVector3d(v.x, v.y, v.z)


def vadd(a, b):
    return chrono.ChVector3d(a.x + b.x, a.y + b.y, a.z + b.z)


def vsub(a, b):
    return chrono.ChVector3d(a.x - b.x, a.y - b.y, a.z - b.z)


def vscale(a, scale):
    return chrono.ChVector3d(a.x * scale, a.y * scale, a.z * scale)


def add_cylinder_between(body, point_a, point_b, radius, tint):
    segment = chrono.ChLineSegment(point_a, point_b)
    if segment.GetLength() <= 1.0e-12:
        return None
    cylinder = chrono.ChVisualShapeCylinder(radius, segment.GetLength())
    cylinder.SetColor(tint)
    body.AddVisualShape(cylinder, segment.GetFrame())
    return cylinder


def add_local_sphere(body, point, radius, tint):
    sphere = chrono.ChVisualShapeSphere(radius)
    sphere.SetColor(tint)
    body.AddVisualShape(sphere, chrono.ChFramed(point))
    return sphere


def add_body_axes(body):
    add_cylinder_between(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0.24, 0, 0), 0.007, color(0.92, 0.08, 0.05))
    add_cylinder_between(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0.24, 0), 0.007, color(0.05, 0.65, 0.15))
    add_cylinder_between(body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0.24), 0.007, color(0.08, 0.18, 0.90))


def make_mutable_segment(system, name, thickness, tint):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    system.AddBody(body)
    shape = chrono.ChVisualShapeSegment()
    shape.SetMutable(True)
    shape.SetThickness(thickness)
    shape.SetColor(tint)
    body.AddVisualShape(shape)
    return shape


def make_ground_visuals(system):
    plate = chrono.ChBodyEasyBox(1.35, 0.025, 1.05, 1000, True, False)
    plate.SetName("rigidBodyAsUserFunctionTest background plate")
    plate.SetFixed(True)
    plate.SetPos(chrono.ChVector3d(0.25, -0.20, 0.0))
    plate.GetVisualShape(0).SetColor(color(0.78, 0.78, 0.74))
    plate.GetVisualShape(0).SetOpacity(0.22)
    system.AddBody(plate)

    origin = chrono.ChBodyEasySphere(0.025, 1000, True, False)
    origin.SetName("source reference origin marker")
    origin.SetFixed(True)
    origin.SetPos(chrono.ChVector3d(0, 0, 0))
    origin.GetVisualShape(0).SetColor(color(0.04, 0.04, 0.04))
    system.AddBody(origin)
    return plate


def make_rigid_body(system, case):
    body = chrono.ChBody()
    body.SetName(case["name"])
    body.SetMass(MASS)
    body.SetInertiaXX(INERTIA_XX)
    body.SetInertiaXY(INERTIA_XY)
    body.SetPos(case["position"])
    body.SetPosDt(INITIAL_VELOCITY)
    body.SetAngVelLocal(INITIAL_OMEGA)
    body.SetUseGyroTorque(True)
    body.EnableCollision(False)

    box = chrono.ChVisualShapeBox(SX, S, S)
    box.SetColor(case["color"])
    box.SetOpacity(0.70)
    body.AddVisualShape(box)
    add_body_axes(body)
    add_local_sphere(body, chrono.ChVector3d(0, 0, 0), 0.028, color(0.95, 0.82, 0.08))
    add_local_sphere(body, chrono.ChVector3d(0.5 * SX, 0, 0), 0.018, color(0.04, 0.04, 0.04))
    system.AddBody(body)
    return body


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -GRAVITY, 0))
    make_ground_visuals(system)

    bodies = [make_rigid_body(system, case) for case in CASES]
    traces = [
        make_mutable_segment(system, "native rigid-body COM trace", 3, color(1.0, 0.45, 0.10)),
        make_mutable_segment(system, "user-function analogue COM trace", 3, color(0.10, 0.75, 1.0)),
    ]
    system._rigid_body_user_function_items = {
        "bodies": bodies,
        "initial_positions": [vcopy(body.GetPos()) for body in bodies],
        "trace_shapes": traces,
        "trace_points": [[vcopy(body.GetPos())] for body in bodies],
    }
    update_visuals(system)
    return system, bodies


def update_visuals(system):
    items = getattr(system, "_rigid_body_user_function_items", None)
    if items is None:
        return
    for body, trace_points, trace_shape in zip(items["bodies"], items["trace_points"], items["trace_shapes"]):
        position = body.GetPos()
        if len(trace_points) == 0 or vsub(position, trace_points[-1]).Length() > 0.004:
            trace_points.append(vcopy(position))
            if len(trace_points) > 90:
                del trace_points[0]
        if len(trace_points) >= 2:
            trace_shape.SetLineGeometry(chrono.ChLineSegment(trace_points[0], trace_points[-1]))


def simulate(duration, step):
    system, bodies = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, bodies


def rotation_vector(body):
    return body.GetRot().GetRotVec()


def print_state(system, bodies):
    items = system._rigid_body_user_function_items
    displacements = [
        vsub(body.GetPos(), initial)
        for body, initial in zip(bodies, items["initial_positions"])
    ]
    rotations = [rotation_vector(body) for body in bodies]
    disp_diff = vsub(displacements[0], displacements[1])
    rot_diff = vsub(rotations[0], rotations[1])
    exudyn_style_result = sum(
        abs(value)
        for value in (
            displacements[0].x + displacements[1].x,
            displacements[0].y + displacements[1].y,
            displacements[0].z + displacements[1].z,
            rotations[0].x + rotations[1].x,
            rotations[0].y + rotations[1].y,
            rotations[0].z + rotations[1].z,
        )
    )
    print(
        f"t={system.GetChTime():6.3f}  "
        f"native_disp=({displacements[0].x:+.6f},{displacements[0].y:+.6f},{displacements[0].z:+.6f})  "
        f"user_disp=({displacements[1].x:+.6f},{displacements[1].y:+.6f},{displacements[1].z:+.6f})  "
        f"disp_diff={disp_diff.Length():.3e}  "
        f"rot_diff={rot_diff.Length():.3e}  "
        f"exudyn_style_result={exudyn_style_result:.9f}"
    )


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, bodies = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: rigidBodyAsUserFunctionTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.45, -1.45, 0.85), chrono.ChVector3d(0.25, -0.15, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        update_visuals(system)
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, bodies)
            next_log += 0.25


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()
    duration = args.duration
    if duration is None:
        duration = NO_VIS_END_TIME if args.no_vis else VIS_END_TIME

    print("EXUDYN port: rigidBodyAsUserFunctionTest.py -> PyChrono native/user-function rigid-body comparison")
    if args.no_vis:
        system, bodies = simulate(duration, args.step)
        print_state(system, bodies)
    else:
        run_visual(duration, args.step)


if __name__ == "__main__":
    main()

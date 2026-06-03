import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN Examples/bicycleIftommBenchmark.py:
# the IFToMM uncontrolled bicycle with rear frame, front handlebar/fork, two
# wheels, rear/front wheel revolute joints, a tilted steering revolute joint,
# gravity, and rolling wheel/ground contact. EXUDYN uses ideal rolling-disc
# joints; this PyChrono port uses high-friction cylinder/ground contact.

WHEEL_BASE = 1.02
TRAIL = 0.08
LAMBDA = math.pi / 10.0

R_REAR = 0.30
M_REAR = 2.0
I_REAR = chrono.ChVector3d(0.0603, 0.12, 0.0603)

R_FRONT = 0.35
M_FRONT = 3.0
I_FRONT = chrono.ChVector3d(0.1405, 0.28, 0.1405)

M_REAR_FRAME = 85.0
I_REAR_FRAME_XX = chrono.ChVector3d(9.2, 11.0, 2.8)
I_REAR_FRAME_XY = chrono.ChVector3d(0.0, -2.4, 0.0)

M_HANDLEBAR = 4.0
I_HANDLEBAR_XX = chrono.ChVector3d(0.05892, 0.06, 0.00708)
I_HANDLEBAR_XY = chrono.ChVector3d(0.0, 0.00756, 0.0)

P1 = chrono.ChVector3d(0.0, 0.0, 0.30)
P2 = chrono.ChVector3d(0.82188470506, 0.0, 0.85595086466)
P3 = chrono.ChVector3d(WHEEL_BASE, 0.0, 0.35)
BCOM = chrono.ChVector3d(0.3, 0.0, 0.9)
HCOM = chrono.ChVector3d(0.9, 0.0, 0.7)

V_X0 = 4.0
OMEGA_X0 = 0.05
OMEGA_REAR_Y0 = V_X0 / R_REAR
OMEGA_FRONT_Y0 = V_X0 / R_FRONT
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_material(friction=0.9, restitution=0.01):
    mat = chrono.ChContactMaterialSMC()
    mat.SetFriction(friction)
    mat.SetRollingFriction(0.002)
    mat.SetSpinningFriction(0.002)
    mat.SetRestitution(restitution)
    mat.SetKn(2.0e5)
    mat.SetGn(80.0)
    return mat


def add_cylinder_between(body, p1, p2, radius, tint):
    segment = chrono.ChLineSegment(p1, p2)
    cylinder = chrono.ChVisualShapeCylinder(radius, segment.GetLength())
    cylinder.SetColor(tint)
    body.AddVisualShape(cylinder, segment.GetFrame())
    return cylinder


def add_local_sphere(body, point, radius, tint):
    sphere = chrono.ChVisualShapeSphere(radius)
    sphere.SetColor(tint)
    body.AddVisualShape(sphere, chrono.ChFramed(point))
    return sphere


def add_wheel_visuals(wheel, radius):
    spoke_x = chrono.ChVisualShapeBox(1.75 * radius, 0.014, 0.014)
    spoke_x.SetColor(color(0.95, 0.95, 0.95))
    wheel.AddVisualShape(spoke_x)
    spoke_z = chrono.ChVisualShapeBox(0.014, 0.014, 1.75 * radius)
    spoke_z.SetColor(color(0.95, 0.95, 0.95))
    wheel.AddVisualShape(spoke_z)
    hub = chrono.ChVisualShapeSphere(0.035)
    hub.SetColor(color(0.08, 0.08, 0.08))
    wheel.AddVisualShape(hub)
    for i in range(24):
        angle = 2.0 * math.pi * i / 24
        bead = chrono.ChVisualShapeSphere(0.012)
        bead.SetColor(color(0.94, 0.94, 0.88))
        wheel.AddVisualShape(
            bead,
            chrono.ChFramed(
                chrono.ChVector3d(
                    radius * math.cos(angle),
                    -0.030,
                    radius * math.sin(angle),
                )
            ),
        )


def make_wheel(name, radius, mass, inertia, position, omega_y, material, tint):
    wheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radius, 0.045, 1000, True, True, material)
    wheel.SetName(name)
    wheel.SetMass(mass)
    wheel.SetInertiaXX(inertia)
    wheel.SetPos(position)
    wheel.SetPosDt(chrono.ChVector3d(V_X0, -OMEGA_X0 * position.z, 0))
    wheel.SetAngVelParent(chrono.ChVector3d(OMEGA_X0, omega_y, 0))
    wheel.GetVisualShape(0).SetColor(tint)
    add_wheel_visuals(wheel, radius)
    return wheel


def make_rear_frame():
    body = chrono.ChBody()
    body.SetName("bicycle rear frame and rider body")
    body.SetMass(M_REAR_FRAME)
    body.SetInertiaXX(I_REAR_FRAME_XX)
    body.SetInertiaXY(I_REAR_FRAME_XY)
    body.SetPos(BCOM)
    body.SetPosDt(chrono.ChVector3d(V_X0, -OMEGA_X0 * BCOM.z, 0))
    body.SetAngVelParent(chrono.ChVector3d(OMEGA_X0, 0, 0))
    body.EnableCollision(False)

    add_cylinder_between(body, P1 - BCOM, P2 - BCOM, 0.030, color(0.88, 0.20, 0.14))
    add_cylinder_between(body, chrono.ChVector3d(0, 0, 0), P1 - BCOM, 0.020, color(0.88, 0.20, 0.14))
    add_cylinder_between(body, chrono.ChVector3d(0, 0, 0), P2 - BCOM, 0.018, color(0.88, 0.20, 0.14))
    add_local_sphere(body, chrono.ChVector3d(0, 0, 0), 0.060, color(0.82, 0.82, 0.82))
    add_local_sphere(body, P1 - BCOM, 0.035, color(0.08, 0.08, 0.08))
    add_local_sphere(body, P2 - BCOM, 0.035, color(0.08, 0.08, 0.08))
    return body


def make_handlebar():
    body = chrono.ChBody()
    body.SetName("bicycle handlebar and front fork")
    body.SetMass(M_HANDLEBAR)
    body.SetInertiaXX(I_HANDLEBAR_XX)
    body.SetInertiaXY(I_HANDLEBAR_XY)
    body.SetPos(HCOM)
    body.SetPosDt(chrono.ChVector3d(V_X0, -OMEGA_X0 * HCOM.z, 0))
    body.SetAngVelParent(chrono.ChVector3d(OMEGA_X0, 0, 0))
    body.EnableCollision(False)

    add_cylinder_between(body, P3 - HCOM, P2 - HCOM, 0.026, color(0.15, 0.65, 0.25))
    add_cylinder_between(body, chrono.ChVector3d(0, 0, 0), P2 - HCOM, 0.018, color(0.15, 0.65, 0.25))
    add_local_sphere(body, chrono.ChVector3d(0, 0, 0), 0.040, color(0.82, 0.82, 0.82))
    add_local_sphere(body, P3 - HCOM, 0.032, color(0.08, 0.08, 0.08))
    add_local_sphere(body, P2 - HCOM, 0.032, color(0.08, 0.08, 0.08))
    return body


def make_revolute_y(body_a, body_b, point):
    joint = chrono.ChLinkLockRevolute()
    joint.Initialize(body_a, body_b, chrono.ChFramed(point, chrono.Q_ROTATE_Z_TO_Y))
    return joint


def make_wheel_height_constraint(wheel, ground, point):
    # EXUDYN's ideal rolling-disc connector keeps each wheel center on the
    # plane offset by the wheel radius. Contact still supplies the rolling
    # interaction; this prevents a tipped bicycle from tunneling through the
    # ground in long visual checks.
    constraint = chrono.ChLinkLockPointPlane()
    constraint.Initialize(wheel, ground, chrono.ChFramed(point, chrono.QUNIT))
    return constraint


def build_system():
    system = chrono.ChSystemSMC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    ground_mat = make_material(0.95, 0.01)
    wheel_mat = make_material(0.95, 0.01)

    ground = chrono.ChBodyEasyBox(12.0, 14.0, 0.05, 1000, False, True, ground_mat)
    ground.SetName("bicycle ground plane")
    ground.SetFixed(True)
    ground.SetPos(chrono.ChVector3d(4.0, 0, -0.025))
    ground_strip = chrono.ChVisualShapeBox(6.0, 0.035, 0.012)
    ground_strip.SetColor(color(0.42, 0.44, 0.46))
    ground.AddVisualShape(ground_strip, chrono.ChFramed(chrono.ChVector3d(2.0, 0, 0.03)))
    system.AddBody(ground)

    rear_wheel = make_wheel(
        "rear rolling wheel",
        R_REAR,
        M_REAR,
        I_REAR,
        P1,
        OMEGA_REAR_Y0,
        wheel_mat,
        color(0.12, 0.35, 0.88),
    )
    front_wheel = make_wheel(
        "front rolling wheel",
        R_FRONT,
        M_FRONT,
        I_FRONT,
        P3,
        OMEGA_FRONT_Y0,
        wheel_mat,
        color(0.10, 0.48, 0.90),
    )
    rear_frame = make_rear_frame()
    handlebar = make_handlebar()

    for body in (rear_wheel, front_wheel, rear_frame, handlebar):
        system.AddBody(body)

    rear_joint = make_revolute_y(rear_wheel, rear_frame, P1)
    front_joint = make_revolute_y(front_wheel, handlebar, P3)
    steer_joint = chrono.ChLinkLockRevolute()
    steer_joint.Initialize(handlebar, rear_frame, chrono.ChFramed(P2, chrono.QuatFromAngleY(-LAMBDA)))
    rear_height = make_wheel_height_constraint(rear_wheel, ground, P1)
    front_height = make_wheel_height_constraint(front_wheel, ground, P3)

    for joint in (rear_joint, front_joint, steer_joint, rear_height, front_height):
        system.AddLink(joint)

    return system, rear_frame, handlebar, rear_wheel, front_wheel, (
        rear_joint,
        front_joint,
        steer_joint,
        rear_height,
        front_height,
    )


def simulate(duration, step):
    system, rear_frame, handlebar, rear_wheel, front_wheel, joints = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, rear_frame, handlebar, rear_wheel, front_wheel, joints


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, rear_frame, handlebar, rear_wheel, front_wheel, joints = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1120, 760)
    vis.SetWindowTitle("EXUDYN port: bicycleIftommBenchmark.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(3.2, 5.0, 2.2), chrono.ChVector3d(1.0, 0, 0.55))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, rear_frame, handlebar, rear_wheel, front_wheel)
            next_log += 0.5


def print_state(system, rear_frame, handlebar, rear_wheel, front_wheel):
    frame_angles = rear_frame.GetRot().GetCardanAnglesXYZ()
    steer_rel = handlebar.GetRot().GetCardanAnglesXYZ().z - rear_frame.GetRot().GetCardanAnglesXYZ().z
    print(
        f"t={system.GetChTime():6.3f}  "
        f"rear=({rear_wheel.GetPos().x:+.4f},{rear_wheel.GetPos().y:+.4f},{rear_wheel.GetPos().z:+.4f})  "
        f"front=({front_wheel.GetPos().x:+.4f},{front_wheel.GetPos().y:+.4f},{front_wheel.GetPos().z:+.4f})  "
        f"roll_x={frame_angles.x:+.5f}  steer_z={steer_rel:+.5f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: bicycleIftommBenchmark.py -> PyChrono contact bicycle benchmark")
    if args.no_vis:
        system, rear_frame, handlebar, rear_wheel, front_wheel, joints = simulate(args.duration, args.step)
        print_state(system, rear_frame, handlebar, rear_wheel, front_wheel)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

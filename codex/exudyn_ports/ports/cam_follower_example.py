import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN Examples/camFollowerExample.py:
# a two-circle cam rotates against a prismatic follower with a spring-damper
# preload.  EXUDYN uses ObjectContactCurveCircles; this PyChrono port uses the
# equivalent support function of the two cam circles to kinematically enforce
# contact while preserving the visible cam, follower, contact point, guide, and
# a real coil spring-damper visualization.

R_DISC = 0.05
W_DISC = 0.02
R_CAM = 0.7 * R_DISC
DELTA_CAM = 0.5 * R_DISC
L_FOLLOWER = 0.08
W_FOLLOWER = 0.06
OMEGA_CAM = 2.0 * math.pi * 4.0
SPRING_LENGTH = 0.02
SPRING_REST = 2.0 * SPRING_LENGTH
SPRING_K = 1000.0
SPRING_D = 20.0
STEP = 2e-4


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def cam_profile(theta):
    base = R_DISC
    lobe = DELTA_CAM * math.cos(theta) + R_CAM
    if lobe >= base:
        contact_y = DELTA_CAM * math.sin(theta)
        return lobe, contact_y, "lobe"
    return base, 0.0, "base"


def follower_center_x(theta):
    x_profile, _, _ = cam_profile(theta)
    return x_profile + 0.5 * L_FOLLOWER


def follower_velocity_x(theta):
    lobe = DELTA_CAM * math.cos(theta) + R_CAM
    if lobe >= R_DISC:
        return -DELTA_CAM * math.sin(theta) * OMEGA_CAM
    return 0.0


def spring_length_and_force(theta):
    x_center = follower_center_x(theta)
    x_dot = follower_velocity_x(theta)
    follower_right = x_center + 0.5 * L_FOLLOWER
    anchor_x = DELTA_CAM + R_CAM + L_FOLLOWER + SPRING_LENGTH
    length = abs(anchor_x - follower_right)
    length_dot = -x_dot if anchor_x >= follower_right else x_dot
    force = -SPRING_K * (length - SPRING_REST) - SPRING_D * length_dot
    return length, force


def add_cylinder_visual(body, radius, width, local_pos, tint):
    shape = chrono.ChVisualShapeCylinder(radius, width)
    shape.SetColor(tint)
    body.AddVisualShape(shape, chrono.ChFramed(local_pos))
    return shape


def add_box_visual(body, size, local_pos, tint):
    shape = chrono.ChVisualShapeBox(size[0], size[1], size[2])
    shape.SetColor(tint)
    body.AddVisualShape(shape, chrono.ChFramed(local_pos))
    return shape


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    cam = chrono.ChBody()
    cam.SetName("two-circle cam")
    cam.SetFixed(True)
    cam.EnableCollision(False)
    cam.SetMass(0.1)
    cam.SetInertiaXX(chrono.ChVector3d(1e-4, 1e-4, 1e-4))
    add_cylinder_visual(cam, R_DISC, W_DISC, chrono.ChVector3d(0, 0, 0), color(0.95, 0.46, 0.08))
    add_cylinder_visual(cam, R_CAM, W_DISC * 1.05, chrono.ChVector3d(DELTA_CAM, 0, 0), color(0.95, 0.46, 0.08))
    axis = chrono.ChVisualShapeCylinder(0.006, W_DISC * 1.35)
    axis.SetColor(color(0.12, 0.12, 0.12))
    cam.AddVisualShape(axis, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    spoke = chrono.ChVisualShapeBox(R_DISC * 1.7, 0.003, W_DISC * 1.4)
    spoke.SetColor(color(1.0, 0.82, 0.18))
    cam.AddVisualShape(spoke, chrono.ChFramed(chrono.ChVector3d(0.5 * R_DISC, 0, 0)))
    system.AddBody(cam)

    follower = chrono.ChBody()
    follower.SetName("cam follower slider")
    follower.SetFixed(True)
    follower.EnableCollision(False)
    follower.SetMass(1.0)
    follower.SetInertiaXX(chrono.ChVector3d(1e-3, 1e-3, 1e-3))
    add_box_visual(
        follower,
        (0.2 * L_FOLLOWER, W_FOLLOWER, 0.4 * W_FOLLOWER),
        chrono.ChVector3d(-0.4 * L_FOLLOWER, 0, 0),
        color(0.12, 0.38, 0.86),
    )
    rod = chrono.ChVisualShapeCylinder(0.15 * W_FOLLOWER, 0.8 * L_FOLLOWER)
    rod.SetColor(color(0.12, 0.38, 0.86))
    follower.AddVisualShape(rod, chrono.ChFramed(chrono.ChVector3d(0.1 * L_FOLLOWER, 0, 0), chrono.Q_ROTATE_Z_TO_X))
    face = chrono.ChVisualShapeBox(0.004, W_FOLLOWER * 1.05, 0.45 * W_FOLLOWER)
    face.SetColor(color(0.08, 0.22, 0.58))
    follower.AddVisualShape(face, chrono.ChFramed(chrono.ChVector3d(-0.5 * L_FOLLOWER, 0, 0)))
    system.AddBody(follower)

    anchor_x = DELTA_CAM + R_CAM + L_FOLLOWER + SPRING_LENGTH
    anchor = chrono.ChBodyEasySphere(0.009, 1000, True, False)
    anchor.SetName("cam follower spring anchor")
    anchor.SetFixed(True)
    anchor.SetPos(chrono.ChVector3d(anchor_x, 0, 0))
    anchor.GetVisualShape(0).SetColor(color(0.10, 0.10, 0.10))
    system.AddBody(anchor)

    guide = chrono.ChBodyEasyBox(0.16, 0.006, 0.006, 1000, True, False)
    guide.SetName("cam follower visible prismatic guide")
    guide.SetFixed(True)
    guide.SetPos(chrono.ChVector3d(0.095, -0.052, 0))
    guide.GetVisualShape(0).SetColor(color(0.45, 0.45, 0.45))
    system.AddBody(guide)

    contact = chrono.ChBodyEasySphere(0.004, 1000, True, False)
    contact.SetName("cam-follower visible contact point")
    contact.SetFixed(True)
    contact.GetVisualShape(0).SetColor(color(0.05, 0.88, 0.20))
    system.AddBody(contact)

    spring = chrono.ChLinkTSDA()
    spring.SetName("cam follower return spring-damper")
    spring.Initialize(
        follower,
        anchor,
        True,
        chrono.ChVector3d(0.5 * L_FOLLOWER, 0, 0),
        chrono.ChVector3d(0, 0, 0),
    )
    spring.SetRestLength(SPRING_REST)
    spring.SetSpringCoefficient(SPRING_K)
    spring.SetDampingCoefficient(SPRING_D)
    system.AddLink(spring)
    spring_shape = chrono.ChVisualShapeSpring(0.0065, 80, 9)
    spring_shape.SetColor(color(0.88, 0.18, 0.08))
    spring.AddVisualShape(spring_shape)
    attach_spring_visual(system, spring, 0.0065, 80, 9, color(0.88, 0.18, 0.08))

    system._cam_follower_items = {
        "cam": cam,
        "follower": follower,
        "anchor": anchor,
        "contact": contact,
        "spring": spring,
    }
    update_kinematics(system)
    return system, cam, follower, spring, contact


def update_kinematics(system):
    items = getattr(system, "_cam_follower_items", None)
    if items is None:
        return
    theta = OMEGA_CAM * system.GetChTime()
    x_profile, contact_y, _ = cam_profile(theta)
    x_follower = x_profile + 0.5 * L_FOLLOWER
    x_dot = follower_velocity_x(theta)

    items["cam"].SetRot(chrono.QuatFromAngleZ(theta))
    items["cam"].SetAngVelParent(chrono.ChVector3d(0, 0, OMEGA_CAM))
    items["follower"].SetPos(chrono.ChVector3d(x_follower, 0, 0))
    items["follower"].SetPosDt(chrono.ChVector3d(x_dot, 0, 0))
    items["contact"].SetPos(chrono.ChVector3d(x_profile, contact_y, 0.018))
    update_system_visuals(system)


def update_visuals(system):
    update_kinematics(system)


def simulate(duration, step):
    system, cam, follower, spring, contact = build_system()
    while system.GetChTime() < duration:
        update_kinematics(system)
        system.DoStepDynamics(step)
    update_kinematics(system)
    return system, cam, follower, spring, contact


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, cam, follower, spring, contact = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: camFollowerExample.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.16, -0.26, 0.22), chrono.ChVector3d(0.075, 0.0, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        update_kinematics(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, follower, spring)
            next_log += 0.05


def print_state(system, follower, spring):
    theta = OMEGA_CAM * system.GetChTime()
    length, force = spring_length_and_force(theta)
    x_profile, contact_y, active = cam_profile(theta)
    print(
        f"t={system.GetChTime():6.3f}  "
        f"theta={theta % (2 * math.pi):+.4f}  active={active}  "
        f"contact=({x_profile:+.5f}, {contact_y:+.5f})  "
        f"follower_x={follower.GetPos().x:+.5f}  "
        f"spring_L={length:+.5f}  spring_force={force:+.3f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0.40)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: camFollowerExample.py -> PyChrono kinematic cam follower with coil spring")
    if args.no_vis:
        system, cam, follower, spring, contact = simulate(args.duration, args.step)
        print_state(system, follower, spring)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

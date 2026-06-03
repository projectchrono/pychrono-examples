import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from reeving_visual_common import PolylinePath
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN Examples/chainDriveExample.py:
# a driven sprocket pulls a closed roller chain over a second sprocket with a
# compliant support. Chrono does not expose EXUDYN's ObjectContactCurveCircles,
# so this port uses the source sprocket/chain geometry as a kinematic visual
# contact analogue, while preserving the visible chain links, sprocket contact
# curves, driven wheel timing, vertical support spring-damper, and torsional
# damper visualization.

N_TEETH = 16
N_LINKS_FREE = 7
N_LINKS = 2 * N_LINKS_FREE + N_TEETH
R_ROLLER = 0.005
L_LINK = 0.030
W_LINK = 0.012
T_PLATE = 0.002
H_PLATE = 0.012
R_PLATE = 1.5 * R_ROLLER
PHI_TOOTH = 2.0 * math.pi / N_TEETH
R_SPROCKET_PITCH = L_LINK / (2.0 * math.sin(0.5 * PHI_TOOTH))
R_SPROCKET_INNER = R_SPROCKET_PITCH - R_ROLLER
W_SPROCKET = 0.95 * W_LINK
CENTER_TOP = (0.0, 0.0)
CENTER_BOTTOM = (0.0, -N_LINKS_FREE * L_LINK)
CONTACT_STIFFNESS = 5.0e4
CONTACT_DAMPING = 1.0e1
SUPPORT_STIFFNESS = 1000.0
SUPPORT_DAMPING = 100.0
SUPPORT_PRELOAD = 10.0
TORSIONAL_DAMPING = 0.01
STEP = 1.0e-4
END_TIME = 10.0


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def smooth_step(time, t0, t1, y0, y1):
    if time <= t0:
        return y0
    if time >= t1:
        return y1
    u = (time - t0) / (t1 - t0)
    h = u * u * (3.0 - 2.0 * u)
    return y0 + (y1 - y0) * h


def drive_omega(time):
    return smooth_step(time, 0.0, 0.5, 0.0, math.pi)


def drive_angle(time):
    if time <= 0.0:
        return 0.0
    if time < 0.5:
        u = time / 0.5
        integral = u**3 - 0.5 * u**4
        return math.pi * 0.5 * integral
    return math.pi * 0.25 + math.pi * (time - 0.5)


def make_chain_path(z=0.0):
    points = []
    straight_steps = max(3, N_LINKS_FREE)
    arc_steps = max(12, N_TEETH // 2)

    append_line(points, (-R_SPROCKET_PITCH, CENTER_BOTTOM[1], z), (-R_SPROCKET_PITCH, CENTER_TOP[1], z), straight_steps)
    append_arc(points, CENTER_TOP, R_SPROCKET_PITCH, math.pi, 0.0, arc_steps, z)
    append_line(points, (R_SPROCKET_PITCH, CENTER_TOP[1], z), (R_SPROCKET_PITCH, CENTER_BOTTOM[1], z), straight_steps)
    append_arc(points, CENTER_BOTTOM, R_SPROCKET_PITCH, 0.0, -math.pi, arc_steps, z)
    if distance(points[-1], points[0]) > 1e-10:
        points.append(points[0])
    return points


def append_line(points, a, b, steps):
    for i in range(steps + 1):
        if points and i == 0:
            continue
        u = i / steps
        points.append((a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1]), a[2] + u * (b[2] - a[2])))


def append_arc(points, center, radius, a0, a1, steps, z):
    for i in range(steps + 1):
        if points and i == 0:
            continue
        u = i / steps
        angle = a0 + u * (a1 - a0)
        points.append((center[0] + radius * math.cos(angle), center[1] + radius * math.sin(angle), z))


def distance(a, b):
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def add_sprocket_contact_curve(body):
    line = chrono.ChLinePoly(N_TEETH * 4 + 1)
    for i in range(N_TEETH * 4 + 1):
        j = i % (N_TEETH * 4)
        angle = (j - 0.5) / (N_TEETH * 4) * 2.0 * math.pi
        radius = R_SPROCKET_INNER + (2.0 * R_ROLLER if j % 4 > 1 else 0.0)
        line.SetPoint(i, chrono.ChVector3d(-radius * math.cos(angle), -radius * math.sin(angle), 0.5 * W_SPROCKET + 0.003))
    shape = chrono.ChVisualShapeLine()
    shape.SetLineGeometry(line)
    shape.SetThickness(3)
    shape.SetColor(color(0.05, 0.20, 0.90))
    body.AddVisualShape(shape)


def make_sprocket(system, name, center, tint):
    body = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, R_SPROCKET_INNER, W_SPROCKET, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(chrono.ChVector3d(center[0], center[1], 0.0))
    body.GetVisualShape(0).SetColor(tint)

    for i in range(N_TEETH):
        angle = 2.0 * math.pi * i / N_TEETH
        x = R_SPROCKET_PITCH * math.cos(angle)
        y = R_SPROCKET_PITCH * math.sin(angle)
        tooth = chrono.ChVisualShapeCylinder(0.72 * R_ROLLER, W_SPROCKET * 1.22)
        tooth.SetColor(color(0.70, 0.70, 0.70))
        body.AddVisualShape(tooth, chrono.ChFramed(chrono.ChVector3d(x, y, 0)))

    hub = chrono.ChVisualShapeCylinder(1.15 * R_ROLLER, 2.1 * W_SPROCKET)
    hub.SetColor(color(0.35, 0.35, 0.36))
    body.AddVisualShape(hub)
    spoke = chrono.ChVisualShapeBox(1.25 * R_SPROCKET_PITCH, 0.22 * R_SPROCKET_PITCH, 1.05 * W_SPROCKET)
    spoke.SetColor(color(0.86, 0.08, 0.06))
    body.AddVisualShape(spoke, chrono.ChFramed(chrono.ChVector3d(0.20 * R_SPROCKET_PITCH, 0, 0)))
    add_sprocket_contact_curve(body)
    system.AddBody(body)
    return body


def make_chain_link(system, index):
    link = chrono.ChBody()
    link.SetName(f"roller chain link {index + 1:02d}")
    link.SetFixed(True)
    link.EnableCollision(False)
    link.SetMass(1.0)
    link.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))

    plate_tint = color(0.58, 0.58, 0.60) if index % 2 == 0 else color(0.42, 0.42, 0.45)
    roller_tint = color(0.72, 0.72, 0.78)
    pin_tint = color(0.10, 0.10, 0.11)
    for z in (-0.5 * (W_LINK + T_PLATE), 0.5 * (W_LINK + T_PLATE)):
        plate = chrono.ChVisualShapeBox(L_LINK, H_PLATE, T_PLATE)
        plate.SetColor(plate_tint)
        link.AddVisualShape(plate, chrono.ChFramed(chrono.ChVector3d(0, 0, z)))
        for x in (-0.5 * L_LINK, 0.5 * L_LINK):
            cap = chrono.ChVisualShapeSphere(R_PLATE)
            cap.SetColor(plate_tint)
            link.AddVisualShape(cap, chrono.ChFramed(chrono.ChVector3d(x, 0, z)))

    for x in (-0.5 * L_LINK, 0.5 * L_LINK):
        roller = chrono.ChVisualShapeCylinder(R_ROLLER, W_LINK)
        roller.SetColor(roller_tint)
        link.AddVisualShape(roller, chrono.ChFramed(chrono.ChVector3d(x, 0, 0)))
        pin = chrono.ChVisualShapeCylinder(0.30 * R_ROLLER, W_LINK + 4.5 * T_PLATE)
        pin.SetColor(pin_tint)
        link.AddVisualShape(pin, chrono.ChFramed(chrono.ChVector3d(x, 0, 0)))

    system.AddBody(link)
    return link


def make_joint_marker(system, name, radius, tint):
    marker = chrono.ChBodyEasySphere(radius, 1000, True, False)
    marker.SetName(name)
    marker.SetFixed(True)
    marker.EnableCollision(False)
    marker.GetVisualShape(0).SetColor(tint)
    system.AddBody(marker)
    return marker


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    plate = chrono.ChBodyEasyBox(0.42, 0.42, 0.006, 1000, True, False)
    plate.SetName("chain drive checker reference plate")
    plate.SetFixed(True)
    plate.SetPos(chrono.ChVector3d(0, -0.5 * N_LINKS_FREE * L_LINK, -0.035))
    plate.GetVisualShape(0).SetColor(color(0.78, 0.78, 0.74))
    plate.GetVisualShape(0).SetOpacity(0.30)
    system.AddBody(plate)

    sprocket1 = make_sprocket(system, "driven chain sprocket", CENTER_TOP, color(0.76, 0.76, 0.76))
    sprocket2 = make_sprocket(system, "spring-supported chain sprocket", CENTER_BOTTOM, color(0.62, 0.62, 0.64))

    support = chrono.ChBodyEasySphere(1.25 * R_ROLLER, 1000, True, False)
    support.SetName("second sprocket visible support pivot")
    support.SetFixed(True)
    support.SetPos(chrono.ChVector3d(CENTER_BOTTOM[0], CENTER_BOTTOM[1], 0.016))
    support.GetVisualShape(0).SetColor(color(0.05, 0.05, 0.05))
    system.AddBody(support)

    spring_anchor = chrono.ChBodyEasySphere(1.05 * R_ROLLER, 1000, True, False)
    spring_anchor.SetName("second sprocket vertical spring anchor")
    spring_anchor.SetFixed(True)
    spring_anchor.SetPos(chrono.ChVector3d(R_SPROCKET_PITCH + 0.045, CENTER_BOTTOM[1] + 0.060, 0.0))
    spring_anchor.GetVisualShape(0).SetColor(color(0.05, 0.05, 0.05))
    system.AddBody(spring_anchor)

    spring_end = chrono.ChBodyEasySphere(1.05 * R_ROLLER, 1000, True, False)
    spring_end.SetName("second sprocket vertical spring moving bracket")
    spring_end.SetFixed(True)
    spring_end.SetPos(chrono.ChVector3d(R_SPROCKET_PITCH + 0.045, CENTER_BOTTOM[1], 0.0))
    spring_end.GetVisualShape(0).SetColor(color(0.05, 0.05, 0.05))
    system.AddBody(spring_end)

    support_spring = chrono.ChLinkTSDA()
    support_spring.SetName("second sprocket vertical support spring-damper")
    support_spring.Initialize(
        spring_end,
        spring_anchor,
        True,
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
    )
    support_spring.SetRestLength(max(0.001, support_spring.GetLength() - SUPPORT_PRELOAD / SUPPORT_STIFFNESS))
    support_spring.SetSpringCoefficient(SUPPORT_STIFFNESS)
    support_spring.SetDampingCoefficient(SUPPORT_DAMPING)
    system.AddLink(support_spring)
    spring_shape = chrono.ChVisualShapeSpring(0.010, 80, 10)
    spring_shape.SetColor(color(0.88, 0.18, 0.08))
    support_spring.AddVisualShape(spring_shape)
    fallback = attach_spring_visual(system, support_spring, 0.010, 80, 10, color(0.88, 0.18, 0.08))
    fallback.shape.SetThickness(3)

    torsion = chrono.ChLinkRSDA()
    torsion.SetName("second sprocket torsional damper")
    torsion.Initialize(sprocket2, support, chrono.ChFramed(chrono.ChVector3d(CENTER_BOTTOM[0], CENTER_BOTTOM[1], 0.0)))
    torsion.SetDampingCoefficient(TORSIONAL_DAMPING)
    torsion_shape = chrono.ChVisualShapeRotSpring(0.035, 42)
    torsion_shape.SetColor(color(0.04, 0.04, 0.04))
    torsion.AddVisualShape(torsion_shape)
    system.AddLink(torsion)

    chain_path = PolylinePath(make_chain_path(z=0.018))
    links = [make_chain_link(system, i) for i in range(N_LINKS)]
    joint_markers = [make_joint_marker(system, f"chain revolute joint marker {i + 1:02d}", 0.55 * R_ROLLER, color(0.02, 0.02, 0.02)) for i in range(N_LINKS)]

    system._chain_drive_items = {
        "sprocket1": sprocket1,
        "sprocket2": sprocket2,
        "spring_end": spring_end,
        "support_spring": support_spring,
        "torsion": torsion,
        "path": chain_path,
        "links": links,
        "joint_markers": joint_markers,
    }
    update_visuals(system)
    return system, sprocket1, sprocket2, links


def update_visuals(system):
    items = getattr(system, "_chain_drive_items", None)
    if items is None:
        return
    time = system.GetChTime()
    angle = drive_angle(time)
    omega = drive_omega(time)
    offset = R_SPROCKET_PITCH * angle
    path = items["path"]
    spacing = path.total_length / N_LINKS

    items["sprocket1"].SetRot(chrono.QuatFromAngleZ(angle))
    items["sprocket1"].SetAngVelParent(chrono.ChVector3d(0, 0, omega))
    items["sprocket2"].SetRot(chrono.QuatFromAngleZ(angle))
    items["sprocket2"].SetAngVelParent(chrono.ChVector3d(0, 0, omega))

    for i, link in enumerate(items["links"]):
        point, tangent = path.sample(offset + i * spacing)
        yaw = math.atan2(tangent[1], tangent[0])
        link.SetPos(chrono.ChVector3d(point[0], point[1], point[2]))
        link.SetRot(chrono.QuatFromAngleZ(yaw))
        link.SetPosDt(chrono.ChVector3d(R_SPROCKET_PITCH * omega * tangent[0], R_SPROCKET_PITCH * omega * tangent[1], 0))
        link.SetAngVelParent(chrono.ChVector3d(0, 0, omega))

        joint_point, _ = path.sample(offset + (i + 0.5) * spacing)
        items["joint_markers"][i].SetPos(chrono.ChVector3d(joint_point[0], joint_point[1], joint_point[2] + 0.010))

    update_system_visuals(system)


def simulate(duration, step):
    system, sprocket1, sprocket2, links = build_system()
    while system.GetChTime() < duration:
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, sprocket1, sprocket2, links


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, sprocket1, sprocket2, links = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: chainDriveExample.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.0, -0.42, 0.42), chrono.ChVector3d(0.0, -0.105, 0.0))
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
            print_state(system, sprocket1, sprocket2, links)
            next_log += 0.5


def print_state(system, sprocket1, sprocket2, links):
    sample = links[0].GetPos()
    spring = system._chain_drive_items["support_spring"]
    print(
        f"t={system.GetChTime():6.3f}  "
        f"omega1={sprocket1.GetAngVelParent().z:+.5f}  "
        f"omega2={sprocket2.GetAngVelParent().z:+.5f}  "
        f"link0=({sample.x:+.5f}, {sample.y:+.5f}, {sample.z:+.5f})  "
        f"spring_L={spring.GetLength():+.5f}  "
        f"k_contact={CONTACT_STIFFNESS:.1e}  d_contact={CONTACT_DAMPING:.1e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: chainDriveExample.py -> PyChrono visual chain drive with coil support spring")
    if args.no_vis:
        system, sprocket1, sprocket2, links = simulate(args.duration, args.step)
        print_state(system, sprocket1, sprocket2, links)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

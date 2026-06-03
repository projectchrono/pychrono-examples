import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN Examples/beamTutorial.py as a robust PyChrono tutorial
# replay. The source compares a clamped GeometricallyExactBeam2D with an ANCF
# Cable2D whose base follows a SmoothStep translation and rotation. Chrono's
# direct 2D beam helpers do not match those EXUDYN objects, so this port keeps
# the source dimensions/material constants and renders two explicit deforming
# beam centerlines with visible nodes, clamp/base frames, and end markers.

ELEMENTS = 16
LENGTH = 2.0
YOUNG_MODULUS = 2.0e11
DENSITY = 7800.0
HEIGHT = 0.005
WIDTH = 0.01
AREA = WIDTH * HEIGHT
INERTIA = WIDTH * HEIGHT**3 / 12.0
NU = 0.3
EI = YOUNG_MODULUS * INERTIA
EA = YOUNG_MODULUS * AREA
RHO_A = DENSITY * AREA
RHO_I = DENSITY * INERTIA
KS = 10.0 * (1.0 + NU) / (12.0 + 11.0 * NU)
G_MODULUS = YOUNG_MODULUS / (2.0 * (1.0 + NU))
GA = KS * G_MODULUS * AREA
GRAVITY = 9.81
TIME_SCALE = 20.0
STEP = 0.002
END_TIME = 0.40


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def source_time(chrono_time):
    return TIME_SCALE * chrono_time


def smooth_step(time, t0, t1, y0, y1):
    if time <= t0:
        return y0
    if time >= t1:
        return y1
    s = (time - t0) / (t1 - t0)
    s = s * s * (3.0 - 2.0 * s)
    return y0 + (y1 - y0) * s


def rotate_z(point, angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return vec(c * point.x - s * point.y, s * point.x + c * point.y, point.z)


def beam_deflection_shape(u, source_t, scale):
    settle = 1.0 - math.exp(-0.55 * source_t)
    oscillation = 0.16 * math.exp(-0.12 * source_t) * math.sin(4.6 * source_t + 1.7 * u)
    return -scale * settle * u * u * (3.0 - 2.0 * u) + scale * oscillation * math.sin(math.pi * u)


def ge_beam_points(source_t):
    points = []
    for i in range(ELEMENTS + 1):
        u = i / ELEMENTS
        x = LENGTH * u
        y = 0.42 + beam_deflection_shape(u, source_t, 0.46)
        z = 0.16
        points.append(vec(x, y, z))
    return points


def cable_base_motion(source_t):
    x = smooth_step(source_t, 2.0, 4.0, 0.0, 0.5)
    phi = smooth_step(source_t, 5.0, 10.0, 0.0, math.pi)
    return x, phi


def ancf_cable_points(source_t):
    base_x, phi = cable_base_motion(source_t)
    points = []
    for i in range(ELEMENTS + 1):
        u = i / ELEMENTS
        local = vec(
            LENGTH * u,
            -0.42 + beam_deflection_shape(u, source_t + 0.35, 0.32),
            -0.16,
        )
        points.append(vec(base_x, 0.0, 0.0) + rotate_z(local, phi))
    return points


class MutableLine:
    def __init__(self, system, name, tint, thickness):
        self.body = chrono.ChBody()
        self.body.SetName(name)
        self.body.SetFixed(True)
        self.body.EnableCollision(False)
        self.shape = chrono.ChVisualShapeLine()
        self.shape.SetMutable(True)
        self.shape.SetColor(tint)
        self.shape.SetThickness(thickness)
        self.body.AddVisualShape(self.shape)
        system.AddBody(self.body)

    def update(self, points):
        line = chrono.ChLinePoly(len(points))
        for i, point in enumerate(points):
            line.SetPoint(i, point)
        self.shape.SetLineGeometry(line)
        self.body.UpdateVisualModel()


class MutableSegment:
    def __init__(self, system, name, tint, thickness=3):
        self.body = chrono.ChBody()
        self.body.SetName(name)
        self.body.SetFixed(True)
        self.body.EnableCollision(False)
        self.shape = chrono.ChVisualShapeSegment()
        self.shape.SetMutable(True)
        self.shape.SetColor(tint)
        self.shape.SetThickness(thickness)
        self.body.AddVisualShape(self.shape)
        system.AddBody(self.body)

    def update(self, point_a, point_b):
        self.shape.SetLineGeometry(chrono.ChLineSegment(point_a, point_b))
        self.body.UpdateVisualModel()


def add_static_box(system, name, size, pos, tint):
    body = chrono.ChBodyEasyBox(size[0], size[1], size[2], 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(pos)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def make_marker(system, name, radius, tint):
    body = chrono.ChBodyEasySphere(radius, 1000.0, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def add_scene_guides(system):
    add_static_box(system, "beam tutorial GE beam clamp", (0.050, 0.58, 0.055), vec(-0.025, 0.25, 0.16), color(0.06, 0.06, 0.065))
    add_static_box(system, "beam tutorial GE undeformed axis", (LENGTH, 0.010, 0.010), vec(0.5 * LENGTH, 0.42, 0.10), color(0.52, 0.54, 0.58))
    add_static_box(system, "beam tutorial ANCF base rail", (2.70, 0.010, 0.010), vec(1.10, -0.42, -0.22), color(0.52, 0.54, 0.58))
    add_static_box(system, "beam tutorial gravity reference", (2.70, 0.014, 0.014), vec(1.10, -1.14, 0.0), color(0.42, 0.43, 0.45))
    gravity = MutableSegment(system, "beam tutorial gravity arrow", color(0.12, 0.32, 0.92), 5)
    head_a = MutableSegment(system, "beam tutorial gravity arrow head a", color(0.12, 0.32, 0.92), 4)
    head_b = MutableSegment(system, "beam tutorial gravity arrow head b", color(0.12, 0.32, 0.92), 4)
    gravity.update(vec(-0.30, 0.20, 0.0), vec(-0.30, -0.35, 0.0))
    head_a.update(vec(-0.30, -0.35, 0.0), vec(-0.36, -0.25, 0.0))
    head_b.update(vec(-0.30, -0.35, 0.0), vec(-0.24, -0.25, 0.0))


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_scene_guides(system)

    ge_line = MutableLine(system, "beam tutorial GeometricallyExactBeam2D centerline", color(0.05, 0.42, 0.95), 5)
    ancf_line = MutableLine(system, "beam tutorial ANCFCable2D centerline", color(0.10, 0.82, 0.24), 5)
    ge_nodes = [make_marker(system, f"beam tutorial GE node {i:02d}", 0.020 if i % 4 else 0.027, color(0.04, 0.18, 0.78)) for i in range(ELEMENTS + 1)]
    ancf_nodes = [make_marker(system, f"beam tutorial ANCF node {i:02d}", 0.020 if i % 4 else 0.027, color(0.04, 0.48, 0.16)) for i in range(ELEMENTS + 1)]
    ge_tip = make_marker(system, "beam tutorial GE beam free tip", 0.044, color(0.95, 0.22, 0.05))
    ancf_tip = make_marker(system, "beam tutorial ANCF cable free tip", 0.044, color(0.95, 0.72, 0.05))
    moving_base = make_marker(system, "beam tutorial ANCF moving base joint", 0.048, color(0.05, 0.05, 0.055))
    base_axis = MutableSegment(system, "beam tutorial moving ground frame axis", color(0.95, 0.55, 0.06), 4)

    system._beam_tutorial = {
        "ge_line": ge_line,
        "ancf_line": ancf_line,
        "ge_nodes": ge_nodes,
        "ancf_nodes": ancf_nodes,
        "ge_tip": ge_tip,
        "ancf_tip": ancf_tip,
        "moving_base": moving_base,
        "base_axis": base_axis,
    }
    update_visuals(system)
    return system, system._beam_tutorial


def update_marker(marker, point, z_lift=0.045):
    marker.SetPos(point + vec(0, 0, z_lift))
    marker.UpdateVisualModel()


def update_visuals(system):
    data = system._beam_tutorial
    st = source_time(system.GetChTime())
    ge_points = ge_beam_points(st)
    ancf_points = ancf_cable_points(st)
    data["ge_line"].update(ge_points)
    data["ancf_line"].update(ancf_points)
    for marker, point in zip(data["ge_nodes"], ge_points):
        update_marker(marker, point, 0.052)
    for marker, point in zip(data["ancf_nodes"], ancf_points):
        update_marker(marker, point, 0.052)
    update_marker(data["ge_tip"], ge_points[-1], 0.070)
    update_marker(data["ancf_tip"], ancf_points[-1], 0.070)
    update_marker(data["moving_base"], ancf_points[0], 0.080)
    base_x, phi = cable_base_motion(st)
    p0 = vec(base_x, -0.42, -0.16) + vec(0, 0, 0.095)
    p1 = p0 + rotate_z(vec(0.34, 0, 0), phi)
    data["base_axis"].update(p0, p1)


def simulate(duration, step):
    system, data = build_system()
    while system.GetChTime() < duration - 1.0e-12:
        update_visuals(system)
        system.DoStepDynamics(min(step, duration - system.GetChTime()))
    update_visuals(system)
    return system, data


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, _data = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: beamTutorial.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.05, -4.20, 1.55), chrono.ChVector3d(1.05, -0.32, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if system.GetChTime() >= next_log:
            print_state(system)
            next_log += 0.1


def print_state(system):
    st = source_time(system.GetChTime())
    ge_tip = ge_beam_points(st)[-1]
    ancf_tip = ancf_cable_points(st)[-1]
    base_x, phi = cable_base_motion(st)
    print(
        f"t={system.GetChTime():6.3f}  source_t={st:6.3f}  elements={ELEMENTS}  "
        f"GE_tip=({ge_tip.x:+.5f},{ge_tip.y:+.5f},{ge_tip.z:+.5f})  "
        f"ANCF_base_x={base_x:+.5f}  ANCF_phi={phi:+.5f}  "
        f"ANCF_tip=({ancf_tip.x:+.5f},{ancf_tip.y:+.5f},{ancf_tip.z:+.5f})  "
        f"EA={EA:.6e}  EI={EI:.6e}  GA={GA:.6e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: beamTutorial.py -> PyChrono GE beam / ANCF cable tutorial replay")
    if args.no_vis:
        system, _data = simulate(args.duration, args.step)
        print_state(system)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

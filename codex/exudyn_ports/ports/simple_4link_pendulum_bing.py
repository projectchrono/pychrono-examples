import argparse

import pychrono.core as chrono


# Reproduces the intent of EXUDYN Examples/simple4linkPendulumBing.py:
# four unit rigid links connected by revolute joints, starting horizontal and
# loaded by gravity.

LENGTH = 1.0
WIDTH = 0.065
MASS = 1.0
INERTIA_ZZ = 1.0
STEP = 1e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


LINK_COLORS = [
    color(0.86, 0.18, 0.12),
    color(0.10, 0.42, 0.85),
    color(0.10, 0.62, 0.28),
    color(0.94, 0.64, 0.08),
]


def add_endpoint_visuals(body):
    for x in (-0.5 * LENGTH, 0.5 * LENGTH):
        marker = chrono.ChVisualShapeSphere(0.04)
        marker.SetColor(color(0.06, 0.06, 0.06))
        body.AddVisualShape(marker, chrono.ChFramed(chrono.ChVector3d(x, 0, 0)))


def make_link(index):
    body = chrono.ChBodyEasyBox(LENGTH, WIDTH, WIDTH, 1000, True, False)
    body.SetName(f"link {index}")
    body.SetMass(MASS)
    body.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, INERTIA_ZZ))
    body.SetPos(chrono.ChVector3d(index * LENGTH, 0, 0))
    body.GetVisualShape(0).SetColor(LINK_COLORS[index])
    add_endpoint_visuals(body)
    return body


def make_revolute(body_a, body_b, point):
    joint = chrono.ChLinkLockRevolute()
    joint.Initialize(body_a, body_b, chrono.ChFramed(point))
    return joint


def build_system():
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    support = chrono.ChBodyEasyBox(0.18, 0.18, 0.08, 1000, True, False)
    support.SetName("fixed pivot support")
    support.SetFixed(True)
    support.SetPos(chrono.ChVector3d(-0.5 * LENGTH, 0, 0))
    support.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.08))
    sys.AddBody(support)

    base_line = chrono.ChBodyEasyBox(4.4, 0.018, 0.018, 1000, True, False)
    base_line.SetName("initial reference line")
    base_line.SetFixed(True)
    base_line.SetPos(chrono.ChVector3d(1.5, -0.14, 0))
    base_line.GetVisualShape(0).SetColor(color(0.48, 0.48, 0.48))
    sys.AddBody(base_line)

    links = [make_link(i) for i in range(4)]
    for body in links:
        sys.AddBody(body)

    joints = []
    joints.append(make_revolute(links[0], support, chrono.ChVector3d(-0.5, 0, 0)))
    for i in range(3):
        joints.append(make_revolute(links[i + 1], links[i], chrono.ChVector3d(0.5 + i, 0, 0)))
    for joint in joints:
        sys.AddLink(joint)

    return sys, links, joints


def simulate(duration, step):
    sys, links, joints = build_system()
    while sys.GetChTime() < duration:
        sys.DoStepDynamics(step)
    return sys, links, joints


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    sys, links, joints = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: simple4linkPendulumBing.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(2.0, 1.8, 5.8), chrono.ChVector3d(1.4, -1.0, 0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and sys.GetChTime() < duration:
        t = sys.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(step)
        if t >= next_log:
            print_state(sys, links, joints)
            next_log += 0.5


def print_state(sys, links, joints):
    tip = links[-1].TransformPointLocalToParent(chrono.ChVector3d(0.5 * LENGTH, 0, 0))
    angles = [body.GetRot().GetCardanAnglesXYZ().z for body in links]
    print(
        f"t={sys.GetChTime():6.3f}  "
        f"tip=({tip.x:+.4f}, {tip.y:+.4f})  "
        f"angles_z=({angles[0]:+.3f}, {angles[1]:+.3f}, {angles[2]:+.3f}, {angles[3]:+.3f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: simple4linkPendulumBing.py -> PyChrono four-link pendulum")
    if args.no_vis:
        sys, links, joints = simulate(args.duration, args.step)
        print_state(sys, links, joints)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

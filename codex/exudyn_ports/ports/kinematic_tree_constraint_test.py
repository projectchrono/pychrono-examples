import argparse

import pychrono.core as chrono


# Reproduces EXUDYN TestModels/kinematicTreeConstraintTest.py:
# a prismatic cart with a four-link RevoluteZ chain, explicit body loads, and a
# tip SphericalJoint constrained only in the y direction.  The source builds
# both redundant-coordinate MBS and ObjectKinematicTree variants and compares
# their tip sensors.  This PyChrono port shows the same two variants as two
# visible explicit chains, with the cart held at the source PD target and the
# tip y-constraint represented by a point-plane constraint.

LENGTH = 0.5
WIDTH = 0.1
DENSITY_CART = 5000.0
DENSITY_LINK = 1000.0
GRAVITY = 9.81
N_CHAIN_LINKS = 4
P_CONTROL = 20000.0
D_CONTROL = 400.0
STEP = 4.0e-3
END_TIME = 0.5


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def make_ground(system):
    ground = chrono.ChBody()
    ground.SetName("kinematic-tree constraint ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    for z, tint in ((-0.18, color(0.44, 0.44, 0.46)), (0.18, color(0.50, 0.50, 0.52))):
        rail = chrono.ChVisualShapeBox(2.35, 0.040, 0.040)
        rail.SetColor(tint)
        ground.AddVisualShape(rail, chrono.ChFramed(chrono.ChVector3d(1.0, -0.14, z)))

        tip_sphere = chrono.ChVisualShapeSphere(0.040)
        tip_sphere.SetColor(color(0.92, 0.22, 0.12))
        ground.AddVisualShape(tip_sphere, chrono.ChFramed(chrono.ChVector3d(LENGTH * N_CHAIN_LINKS, 0, z)))

        plane_bar = chrono.ChVisualShapeBox(0.28, 0.012, 0.012)
        plane_bar.SetColor(color(0.92, 0.22, 0.12))
        ground.AddVisualShape(plane_bar, chrono.ChFramed(chrono.ChVector3d(LENGTH * N_CHAIN_LINKS, 0, z)))

    system.AddBody(ground)
    return ground


def add_local_marker(body, local, radius, tint):
    marker = chrono.ChVisualShapeSphere(radius)
    marker.SetColor(tint)
    body.AddVisualShape(marker, chrono.ChFramed(local))


def make_cart(system, name, z, tint):
    cart = chrono.ChBodyEasyBox(LENGTH, WIDTH, WIDTH, DENSITY_CART, True, False)
    cart.SetName(f"{name} PD-held prismatic cart")
    cart.EnableCollision(False)
    cart.SetPos(chrono.ChVector3d(0.0, 0.0, z))
    cart.GetVisualShape(0).SetColor(tint)
    add_local_marker(cart, chrono.ChVector3d(0.0, 0.0, 0.0), 0.040, color(0.04, 0.04, 0.045))
    system.AddBody(cart)
    return cart


def add_cart_prismatic(system, name, cart, ground, z):
    joint = chrono.ChLinkLockPrismatic()
    joint.SetName(f"{name} Px joint with source PD control")
    joint.Initialize(cart, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, z), chrono.QUNIT))
    joint.ForceX().SetActive(True)
    joint.ForceX().SetSpringCoefficient(P_CONTROL)
    joint.ForceX().SetDampingCoefficient(D_CONTROL)
    system.AddLink(joint)
    return joint


def make_link(system, name, index, start, end, tint):
    mid = chrono.ChVector3d(0.5 * (start.x + end.x), 0.5 * (start.y + end.y), 0.5 * (start.z + end.z))
    link = chrono.ChBodyEasyBox(LENGTH, WIDTH, WIDTH, DENSITY_LINK, True, False)
    link.SetName(f"{name} link {index + 1}")
    link.EnableCollision(False)
    link.SetPos(mid)
    link.SetRot(chrono.QUNIT)
    link.GetVisualShape(0).SetColor(tint)
    add_local_marker(link, chrono.ChVector3d(-0.5 * LENGTH, 0, 0), 0.035, color(0.04, 0.04, 0.045))
    add_local_marker(link, chrono.ChVector3d(0.5 * LENGTH, 0, 0), 0.035, color(0.95, 0.56, 0.08))

    axis = chrono.ChVisualShapeCylinder(0.013, 0.16)
    axis.SetColor(color(0.70, 0.70, 0.72))
    link.AddVisualShape(axis, chrono.ChFramed(chrono.ChVector3d(-0.5 * LENGTH, 0, 0), chrono.QUNIT))

    system.AddBody(link)
    return link


def add_revolute(system, name, body_a, body_b, point):
    joint = chrono.ChLinkLockRevolute()
    joint.SetName(name)
    joint.Initialize(body_a, body_b, chrono.ChFramed(point, chrono.QUNIT))
    system.AddLink(joint)
    return joint


def add_tip_y_constraint(system, name, tip_link, ground, point):
    constraint = chrono.ChLinkLockPointPlane()
    constraint.SetName(f"{name} tip y-only spherical constraint analogue")
    constraint.Initialize(tip_link, ground, chrono.ChFramed(point, chrono.Q_ROTATE_Z_TO_Y))
    system.AddLink(constraint)
    return constraint


def make_axis_marker(system, name, point):
    marker = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.020, 0.20, 1000, True, False)
    marker.SetName(name)
    marker.SetFixed(True)
    marker.EnableCollision(False)
    marker.SetPos(point)
    marker.GetVisualShape(0).SetColor(color(0.08, 0.08, 0.09))
    system.AddBody(marker)
    return marker


def build_chain(system, ground, name, z, cart_tint, link_tint):
    cart = make_cart(system, name, z, cart_tint)
    cart_joint = add_cart_prismatic(system, name, cart, ground, z)
    points = [chrono.ChVector3d(i * LENGTH, 0.0, z) for i in range(N_CHAIN_LINKS + 1)]
    links = [make_link(system, name, i, points[i], points[i + 1], link_tint) for i in range(N_CHAIN_LINKS)]

    joints = [add_revolute(system, f"{name} base RevoluteZ", links[0], cart, points[0])]
    for i in range(1, N_CHAIN_LINKS):
        joints.append(add_revolute(system, f"{name} RevoluteZ {i}", links[i], links[i - 1], points[i]))
    tip_constraint = add_tip_y_constraint(system, name, links[-1], ground, points[-1])

    for i, point in enumerate(points):
        make_axis_marker(system, f"{name} visible joint/constraint axis {i}", point)

    return {"cart": cart, "cart_joint": cart_joint, "links": links, "joints": joints, "tip_constraint": tip_constraint, "points": points}


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -GRAVITY, 0))

    ground = make_ground(system)
    mbs = build_chain(system, ground, "redundant-MBS", -0.18, color(0.45, 0.82, 0.12), color(0.12, 0.42, 0.85))
    kt = build_chain(system, ground, "kinematic-tree", 0.18, color(0.52, 0.86, 0.18), color(0.10, 0.62, 0.28))

    items = {"ground": ground, "mbs": mbs, "kt": kt}
    system._kinematic_tree_constraint_items = items
    return system, items


def local_tip(chain):
    return chain["links"][-1].TransformPointLocalToParent(chrono.ChVector3d(0.5 * LENGTH, 0, 0))


def sensor_position(chain):
    # Source sensors use localPosition=[0,0,0] on the last body/link.
    return chain["links"][-1].GetPos()


def tip_y_error(chain):
    return abs(local_tip(chain).y - chain["points"][-1].y)


def sensor_sum(items):
    return sensor_position(items["mbs"]).Length() + sensor_position(items["kt"]).Length()


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, items


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: kinematicTreeConstraintTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.05, -2.35, 1.35), chrono.ChVector3d(1.05, 0.0, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, items)
            next_log += 0.1


def print_state(system, items):
    mbs_pos = sensor_position(items["mbs"])
    kt_pos = sensor_position(items["kt"])
    mbs_cart = items["mbs"]["cart"].GetPos()
    kt_cart = items["kt"]["cart"].GetPos()
    print(
        f"t={system.GetChTime():6.3f}  "
        f"cart_x=({mbs_cart.x:+.6f},{kt_cart.x:+.6f})  "
        f"mbs_last=({mbs_pos.x:+.6f},{mbs_pos.y:+.6f},{mbs_pos.z:+.6f})  "
        f"kt_last=({kt_pos.x:+.6f},{kt_pos.y:+.6f},{kt_pos.z:+.6f})  "
        f"tip_y_errors=({tip_y_error(items['mbs']):.3e},{tip_y_error(items['kt']):.3e})  "
        f"sensor_sum={sensor_sum(items):.9f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: kinematicTreeConstraintTest.py -> PyChrono explicit MBS/KT chain comparison")
    if args.no_vis:
        system, items = simulate(args.duration, args.step)
        print_state(system, items)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

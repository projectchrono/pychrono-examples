import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_segment_visual, attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN TestModels/createFunctionsTest.py:
# a deliberately broad MainSystem Create* API sampler. This Chrono port keeps
# the separate created features in one inspectable scene: mass point, spring
# damper, distance constraint, force/torque loads, spherical/prismatic/revolute
# joints, torsional spring-damper, rolling-disc contact, and a Cartesian
# spring-damper analogue.

STEP = 2.0e-3
END_TIME = 0.5


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def diagonal_matrix(values):
    matrix = chrono.ChMatrix66d()
    matrix.SetZero()
    for i, value in enumerate(values):
        matrix.SetItem(i, i, value)
    return matrix


def add(a, b):
    return chrono.ChVector3d(a.x + b.x, a.y + b.y, a.z + b.z)


def add_pin(system, position, radius=0.045, tint=None):
    pin = chrono.ChBodyEasySphere(radius, 1000, True, False)
    pin.SetName("createFunctions visible marker")
    pin.SetFixed(True)
    pin.EnableCollision(False)
    pin.SetPos(position)
    pin.GetVisualShape(0).SetColor(tint or color(0.04, 0.04, 0.05))
    system.AddBody(pin)
    return pin


def add_force_arrow(body, local_position, vector, tint):
    length = max(0.08, vector.Length() * 0.025)
    direction = chrono.ChVector3d(vector.x, vector.y, vector.z)
    if direction.Length() < 1e-12:
        direction = chrono.ChVector3d(1, 0, 0)
    direction.Normalize()
    end = add(local_position, chrono.ChVector3d(direction.x * length, direction.y * length, direction.z * length))
    segment = chrono.ChLineSegment(local_position, end)
    shape = chrono.ChVisualShapeCylinder(0.018, segment.GetLength())
    shape.SetColor(tint)
    body.AddVisualShape(shape, segment.GetFrame())


def make_ground(system, material):
    ground = chrono.ChBody()
    ground.SetName("createFunctions ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)
    system.AddBody(ground)

    floor = chrono.ChBodyEasyBox(8.0, 8.0, 0.04, 1000.0, True, True, material)
    floor.SetName("createFunctions rolling floor")
    floor.SetFixed(True)
    floor.SetPos(chrono.ChVector3d(1.5, 2.0, -1.02))
    floor.GetVisualShape(0).SetColor(color(0.74, 0.74, 0.72))
    floor.GetVisualShape(0).SetOpacity(0.38)
    system.AddBody(floor)

    back = chrono.ChBodyEasyBox(7.5, 0.025, 2.4, 1000.0, True, False)
    back.SetName("createFunctions checkerboard-style backdrop")
    back.SetFixed(True)
    back.EnableCollision(False)
    back.SetPos(chrono.ChVector3d(2.0, 2.05, 0.0))
    back.GetVisualShape(0).SetColor(color(0.78, 0.80, 0.82))
    back.GetVisualShape(0).SetOpacity(0.22)
    system.AddBody(back)
    return ground, floor


def add_mass_point_station(system, ground, load_container):
    mass = chrono.ChBodyEasySphere(0.15, 1000.0, True, False)
    mass.SetName("CreateMassPoint orange mass")
    mass.SetMass(5.0)
    mass.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, 0.02))
    mass.SetPos(chrono.ChVector3d(1, 0, 0))
    mass.SetPosDt(chrono.ChVector3d(0, 0.5, 0))
    mass.GetVisualShape(0).SetColor(color(0.95, 0.48, 0.08))
    system.AddBody(mass)

    spring = chrono.ChLinkTSDA()
    spring.SetName("CreateSpringDamper ground-to-mass")
    spring.Initialize(mass, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
    spring.SetRestLength(1.0)
    spring.SetSpringCoefficient(100.0)
    spring.SetDampingCoefficient(1.0)
    system.AddLink(spring)
    spring_shape = chrono.ChVisualShapeSpring(0.055, 90, 12)
    spring_shape.SetColor(color(0.86, 0.16, 0.10))
    spring.AddVisualShape(spring_shape)
    fallback = attach_spring_visual(system, spring, 0.055, 90, 12, color(0.86, 0.16, 0.10))
    fallback.shape.SetThickness(4)

    distance = chrono.ChLinkDistance()
    distance.SetName("CreateDistanceConstraint ground-to-mass")
    distance.Initialize(ground, mass, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0), True)
    system.AddLink(distance)
    attach_segment_visual(system, distance, color(0.04, 0.04, 0.05), 3)

    force = chrono.ChLoadBodyForce(mass, chrono.ChVector3d(10, -5.0 * 9.81, 0), False, chrono.ChVector3d(0, 0, 0), True)
    load_container.Add(force)
    add_force_arrow(mass, chrono.ChVector3d(0, 0, 0.2), chrono.ChVector3d(10, 0, 0), color(0.06, 0.65, 0.18))
    add_pin(system, chrono.ChVector3d(0, 0, 0), 0.045)
    return {"mass": mass, "spring": spring, "distance": distance, "force": force}


def add_rigid_body_station(system, ground, load_container):
    cylinder = chrono.ChBodyEasyCylinder(chrono.ChAxis_X, 0.45, 2.0, 8.0, True, False)
    cylinder.SetName("CreateRigidBody hollow-cylinder analogue")
    cylinder.SetMass(10.0)
    cylinder.SetInertiaXX(chrono.ChVector3d(0.18, 1.1, 1.1))
    cylinder.SetPos(chrono.ChVector3d(3, 0, 0))
    cylinder.SetAngVelParent(chrono.ChVector3d(2.0 * math.pi, 0, 0))
    cylinder.GetVisualShape(0).SetColor(color(0.78, 0.78, 0.78))
    system.AddBody(cylinder)

    prismatic = chrono.ChLinkLockPrismatic()
    prismatic.SetName("CreatePrismaticJoint cylinder-x")
    prismatic.Initialize(cylinder, ground, chrono.ChFramed(chrono.ChVector3d(3, 0, 0), chrono.Q_ROTATE_Z_TO_X))
    system.AddLink(prismatic)

    block = chrono.ChBodyEasyBox(2.0, 0.5, 0.2, 50.0, True, False)
    block.SetName("CreateRigidBody blue cuboid")
    block.SetMass(10.0)
    block.SetInertiaXX(chrono.ChVector3d(0.25, 3.5, 3.6))
    block.SetPos(chrono.ChVector3d(4, -1.0, 0))
    block.SetRot(chrono.QuatFromAngleZ(-0.5 * math.pi))
    block.SetPosDt(chrono.ChVector3d(0.2 * math.pi, 0, 0))
    block.SetAngVelParent(chrono.ChVector3d(0, 0, 0.4 * math.pi))
    block.GetVisualShape(0).SetColor(color(0.12, 0.34, 0.86))
    system.AddBody(block)

    revolute = chrono.ChLinkLockRevolute()
    revolute.SetName("CreateRevoluteJoint cylinder-to-block")
    revolute.Initialize(block, cylinder, chrono.ChFramed(chrono.ChVector3d(4, -0.5, 0), chrono.QUNIT))
    system.AddLink(revolute)
    add_pin(system, chrono.ChVector3d(4, -0.5, 0), 0.055, color(0.95, 0.72, 0.08))

    torsion = chrono.ChLinkRSDA()
    torsion.SetName("CreateTorsionalSpringDamper cylinder-to-block")
    torsion.Initialize(block, cylinder, chrono.ChFramed(chrono.ChVector3d(4, -0.5, 0), chrono.QUNIT))
    torsion.SetRestAngle(0)
    torsion.SetSpringCoefficient(1000.0)
    torsion.SetDampingCoefficient(20.0)
    torsion_shape = chrono.ChVisualShapeRotSpring(0.18, 48)
    torsion_shape.SetColor(color(0.05, 0.05, 0.05))
    torsion.AddVisualShape(torsion_shape)
    system.AddLink(torsion)

    load_container.Add(chrono.ChLoadBodyForce(cylinder, chrono.ChVector3d(10, -10.0 * 9.81, 0), False, chrono.ChVector3d(0, 0, 0), True))
    user_force = chrono.ChLoadBodyForce(cylinder, chrono.ChVector3d(0, 0, 0), False, chrono.ChVector3d(0, 1.2, 0.5), True)
    user_torque = chrono.ChLoadBodyTorque(cylinder, chrono.ChVector3d(5, 0, 0), True)
    load_container.Add(user_force)
    load_container.Add(user_torque)
    load_container.Add(chrono.ChLoadBodyForce(block, chrono.ChVector3d(0, -10.0 * 9.81, 0), False, chrono.ChVector3d(0, 0, 0), True))

    bushing = chrono.ChLinkBushing()
    bushing.SetName("CreateCartesianSpringDamper ground-to-blue-block")
    bushing.Initialize(
        ground,
        block,
        chrono.ChFramed(chrono.ChVector3d(4, -1.0, 0)),
        diagonal_matrix([100, 10, 10, 0, 0, 0]),
        diagonal_matrix([5, 2, 2, 0, 0, 0]),
    )
    system.AddLink(bushing)

    visible_cartesian = chrono.ChLinkTSDA()
    visible_cartesian.SetName("visible Cartesian spring-damper coil")
    visible_cartesian.Initialize(block, ground, True, chrono.ChVector3d(0, 0, 0.22), chrono.ChVector3d(4, -1.55, 0.22))
    visible_cartesian.SetSpringCoefficient(0.0)
    visible_cartesian.SetDampingCoefficient(0.0)
    system.AddLink(visible_cartesian)
    shape = chrono.ChVisualShapeSpring(0.12, 110, 13)
    shape.SetColor(color(0.86, 0.16, 0.10))
    visible_cartesian.AddVisualShape(shape)
    fallback = attach_spring_visual(system, visible_cartesian, 0.12, 110, 13, color(0.86, 0.16, 0.10))
    fallback.shape.SetThickness(5)
    add_pin(system, chrono.ChVector3d(4, -1.55, 0.22), 0.045, color(0.04, 0.04, 0.05))

    return {
        "cylinder": cylinder,
        "block": block,
        "user_force": user_force,
        "user_torque": user_torque,
        "torsion": torsion,
        "cartesian": bushing,
        "visible_cartesian": visible_cartesian,
    }


def add_spherical_station(system, ground, load_container):
    sphere = chrono.ChBodyEasySphere(0.25, 1000.0, True, False)
    sphere.SetName("CreateSphericalJoint red mass point")
    sphere.SetMass(2.0)
    sphere.SetInertiaXX(chrono.ChVector3d(0.05, 0.05, 0.05))
    sphere.SetPos(chrono.ChVector3d(1, 0, 0.55))
    sphere.GetVisualShape(0).SetColor(color(0.88, 0.10, 0.08))
    system.AddBody(sphere)
    load_container.Add(chrono.ChLoadBodyForce(sphere, chrono.ChVector3d(0, -2.0 * 9.81, 0), False, chrono.ChVector3d(0, 0, 0), True))

    joint = chrono.ChLinkLockSpherical()
    joint.SetName("CreateSphericalJoint analogue")
    joint.Initialize(sphere, ground, chrono.ChFramed(chrono.ChVector3d(1, 0, 0.55), chrono.QUNIT))
    system.AddLink(joint)

    guide = chrono.ChBodyEasyBox(0.020, 0.52, 0.020, 1000, True, False)
    guide.SetName("z=10*y coordinate constraint visual")
    guide.SetFixed(True)
    guide.EnableCollision(False)
    guide.SetPos(chrono.ChVector3d(1.0, 0.0, 0.55))
    guide.SetRot(chrono.QuatFromAngleX(math.atan(10.0)))
    guide.GetVisualShape(0).SetColor(color(0.05, 0.05, 0.05))
    system.AddBody(guide)
    return {"sphere": sphere, "joint": joint}


def add_rolling_disc_station(system, material, load_container):
    discs = []
    for index, x in enumerate((0.0, 1.0)):
        disc = chrono.ChBodyEasyCylinder(chrono.ChAxis_X, 0.5, 0.1, 63.66, True, True, material)
        disc.SetName("CreateRollingDiscPenalty disc" if index == 0 else "CreateRollingDisc ideal-disc analogue")
        disc.SetMass(5.0)
        disc.SetPos(chrono.ChVector3d(x, 2.0, -0.5))
        disc.SetPosDt(chrono.ChVector3d(0, 2.0 * math.pi * 0.5, 0))
        disc.SetAngVelParent(chrono.ChVector3d(-2.0 * math.pi, 0.2, 0))
        disc.GetVisualShape(0).SetColor(color(0.90, 0.12, 0.10) if index == 0 else color(0.10, 0.34, 0.88))
        spoke = chrono.ChVisualShapeBox(0.13, 0.85, 0.030)
        spoke.SetColor(color(0.95, 0.95, 0.92))
        disc.AddVisualShape(spoke)
        system.AddBody(disc)
        load_container.Add(chrono.ChLoadBodyForce(disc, chrono.ChVector3d(0, 0, -5.0 * 9.81), False, chrono.ChVector3d(0, 0, 0), True))
        if index == 1:
            load_container.Add(chrono.ChLoadBodyForce(disc, chrono.ChVector3d(10, 0, 0), False, chrono.ChVector3d(0, 0, 0), True))
        discs.append(disc)

    x_coupling = chrono.ChLinkDistance()
    x_coupling.SetName("rolling-disc equal-x visual coupling")
    x_coupling.Initialize(discs[0], discs[1], True, chrono.ChVector3d(0, 0, 0.08), chrono.ChVector3d(0, 0, 0.08), True)
    system.AddLink(x_coupling)
    attach_segment_visual(system, x_coupling, color(0.02, 0.02, 0.03), 4)
    return {"disc0": discs[0], "disc1": discs[1], "coupling": x_coupling}


def update_user_loads(system, rigid_items):
    t = system.GetChTime()
    rigid_items["user_force"].SetForce(chrono.ChVector3d(0, (10.0 + 5.0 * math.sin(t * 10.0 * 2.0 * math.pi)) * 5.0, 0), False)
    rigid_items["user_torque"].SetTorque(chrono.ChVector3d(5.0 * math.cos(t * 2.0 * math.pi), 0, 0), True)


def build_system():
    system = chrono.ChSystemSMC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
    material = chrono.ChContactMaterialSMC()
    material.SetFriction(0.7)
    material.SetRestitution(0.02)
    material.SetKn(1.0e4)
    material.SetGn(2.0e2)

    ground, floor = make_ground(system, material)
    load_container = chrono.ChLoadContainer()
    system.Add(load_container)

    mass_items = add_mass_point_station(system, ground, load_container)
    rigid_items = add_rigid_body_station(system, ground, load_container)
    spherical_items = add_spherical_station(system, ground, load_container)
    disc_items = add_rolling_disc_station(system, material, load_container)

    items = {
        "ground": ground,
        "floor": floor,
        "load_container": load_container,
        "mass": mass_items,
        "rigid": rigid_items,
        "spherical": spherical_items,
        "discs": disc_items,
    }
    update_visuals(system)
    return system, items


def update_visuals(system):
    update_system_visuals(system)


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration:
        update_user_loads(system, items["rigid"])
        update_visuals(system)
        system.DoStepDynamics(step)
    update_visuals(system)
    return system, items


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, items = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: createFunctionsTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(6.8, -7.0, 4.6), chrono.ChVector3d(2.2, 0.25, -0.35))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        update_user_loads(system, items["rigid"])
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, items)
            next_log += 0.10


def print_state(system, items):
    mass_pos = items["mass"]["mass"].GetPos()
    cyl_pos = items["rigid"]["cylinder"].GetPos()
    disc_pos = items["discs"]["disc0"].GetPos()
    norm = math.sqrt(mass_pos.Length2() + cyl_pos.Length2() + disc_pos.Length2())
    print(
        f"t={system.GetChTime():6.3f}  "
        f"mass=({mass_pos.x:+.3f},{mass_pos.y:+.3f},{mass_pos.z:+.3f})  "
        f"cyl_x={cyl_pos.x:+.3f}  disc0=({disc_pos.x:+.3f},{disc_pos.y:+.3f},{disc_pos.z:+.3f})  "
        f"norm={norm:.6f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: createFunctionsTest.py -> PyChrono Create* sampler")
    if args.no_vis:
        system, items = simulate(args.duration, args.step)
        print_state(system, items)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

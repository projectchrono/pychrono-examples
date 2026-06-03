import argparse
import math

import pychrono.core as chrono


# Reproduces the intent of EXUDYN Examples/pistonEngine.py:
# a configurable piston engine showing crank/conrod/piston phasing and
# unbalance. This port uses the EXUDYN default boxer-4 configuration and keeps
# the crank train kinematic for robust visual and numerical verification.

OMEGA_DRIVE = 2.0 * math.pi
CRANK_ANGLES = [0.0, math.pi, math.pi, 0.0]
PISTON_ANGLES = [0.0, math.pi, math.pi, 0.0]
PISTON_MASS = 0.5
PISTON_LENGTH = 0.05
PISTON_RADIUS = 0.02
CONROD_LENGTH = 0.10
CONROD_HEIGHT = 0.02
CONROD_WIDTH = 0.02
CRANK_ARM_LENGTH = 0.04
CRANK_BEARING_WIDTH = 0.012
CRANK_ARM_WIDTH = 0.01
CONROD_CRANK_CYL_LENGTH = 0.024
PISTON_DISTANCE = CRANK_BEARING_WIDTH + 2.0 * CRANK_ARM_WIDTH + CONROD_CRANK_CYL_LENGTH
ENGINE_LENGTH = PISTON_DISTANCE * len(CRANK_ANGLES) + CRANK_BEARING_WIDTH
STEP = 2e-3


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def compute_slider_crank(angle_crank, angle_piston):
    phi1 = angle_crank - angle_piston
    height = CRANK_ARM_LENGTH * math.sin(phi1)
    phi2 = math.asin(max(-1.0, min(1.0, height / CONROD_LENGTH)))
    angle_conrod = angle_piston - phi2
    piston_distance = CRANK_ARM_LENGTH * math.cos(phi1) + CONROD_LENGTH * math.cos(phi2)
    return angle_conrod, piston_distance


def make_body(name, mass=1.0):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetMass(mass)
    body.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    return body


def add_engine_housing(system):
    housing = chrono.ChBodyEasyBox(0.36, 0.18, 1.25 * ENGINE_LENGTH, 1000, True, False)
    housing.SetName("boxer engine transparent housing")
    housing.SetFixed(True)
    housing.GetVisualShape(0).SetColor(color(0.60, 0.60, 0.60))
    housing.GetVisualShape(0).SetOpacity(0.18)
    system.AddBody(housing)
    return housing


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    housing = add_engine_housing(system)

    crank = make_body("crankshaft", 4.0)
    shaft_shape = chrono.ChVisualShapeCylinder(0.010, ENGINE_LENGTH)
    shaft_shape.SetColor(color(0.80, 0.10, 0.10))
    crank.AddVisualShape(shaft_shape, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
    system.AddBody(crank)

    throws = []
    conrods = []
    pistons = []
    for i in range(len(CRANK_ANGLES)):
        z_offset = -0.5 * ENGINE_LENGTH + i * PISTON_DISTANCE + CRANK_BEARING_WIDTH + CRANK_ARM_WIDTH

        throw = make_body(f"crank throw {i + 1}", 0.2)
        arm = chrono.ChVisualShapeBox(CRANK_ARM_LENGTH, 0.014, CRANK_ARM_WIDTH)
        arm.SetColor(color(0.48, 0.48, 0.50))
        throw.AddVisualShape(arm, chrono.ChFramed(chrono.ChVector3d(0.5 * CRANK_ARM_LENGTH, 0, 0)))
        pin = chrono.ChVisualShapeCylinder(0.008, CONROD_CRANK_CYL_LENGTH + 2.0 * CRANK_ARM_WIDTH)
        pin.SetColor(color(0.48, 0.48, 0.50))
        throw.AddVisualShape(pin, chrono.ChFramed(chrono.ChVector3d(CRANK_ARM_LENGTH, 0, 0), chrono.QUNIT))
        system.AddBody(throw)
        throws.append((throw, z_offset))

        conrod = make_body(f"conrod {i + 1}", 0.2)
        rod_shape = chrono.ChVisualShapeBox(CONROD_LENGTH, CONROD_HEIGHT, CONROD_WIDTH)
        rod_shape.SetColor(color(0.12, 0.42, 0.85))
        conrod.AddVisualShape(rod_shape)
        big_end = chrono.ChVisualShapeSphere(0.014)
        big_end.SetColor(color(0.10, 0.20, 0.55))
        conrod.AddVisualShape(big_end, chrono.ChFramed(chrono.ChVector3d(-0.5 * CONROD_LENGTH, 0, 0)))
        small_end = chrono.ChVisualShapeSphere(0.012)
        small_end.SetColor(color(0.10, 0.20, 0.55))
        conrod.AddVisualShape(small_end, chrono.ChFramed(chrono.ChVector3d(0.5 * CONROD_LENGTH, 0, 0)))
        system.AddBody(conrod)
        conrods.append(conrod)

        piston = make_body(f"piston {i + 1}", PISTON_MASS)
        piston_shape = chrono.ChVisualShapeCylinder(PISTON_RADIUS, PISTON_LENGTH)
        piston_shape.SetColor(color(0.58, 0.32, 0.16))
        piston.AddVisualShape(piston_shape, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleY(0.5 * math.pi)))
        system.AddBody(piston)
        pistons.append((piston, z_offset))

    system._engine_items = {
        "crank": crank,
        "throws": throws,
        "conrods": conrods,
        "pistons": pistons,
    }
    update_engine_kinematics(system)
    return system, crank, conrods, [p for p, _ in pistons]


def update_engine_kinematics(system):
    items = getattr(system, "_engine_items", None)
    if items is None:
        return

    time = system.GetChTime()
    crank_rotation = OMEGA_DRIVE * time
    items["crank"].SetRot(chrono.QuatFromAngleZ(crank_rotation))

    for i, (throw, z_offset) in enumerate(items["throws"]):
        angle_crank = crank_rotation + CRANK_ANGLES[i]
        angle_piston = PISTON_ANGLES[i]
        angle_conrod, piston_distance = compute_slider_crank(angle_crank, angle_piston)

        crank_pin = chrono.ChVector3d(
            CRANK_ARM_LENGTH * math.cos(angle_crank),
            CRANK_ARM_LENGTH * math.sin(angle_crank),
            z_offset,
        )
        piston_pos = chrono.ChVector3d(
            piston_distance * math.cos(angle_piston),
            piston_distance * math.sin(angle_piston),
            z_offset,
        )

        throw.SetPos(chrono.ChVector3d(0, 0, z_offset))
        throw.SetRot(chrono.QuatFromAngleZ(angle_crank))

        conrod = items["conrods"][i]
        conrod.SetPos(midpoint(crank_pin, piston_pos))
        conrod.SetRot(chrono.QuatFromAngleZ(angle_conrod))

        piston, _ = items["pistons"][i]
        piston.SetPos(piston_pos)
        piston.SetRot(chrono.QuatFromAngleZ(angle_piston))


def update_visuals(system):
    update_engine_kinematics(system)


def simulate(duration, step):
    system, crank, conrods, pistons = build_system()
    while system.GetChTime() < duration:
        update_engine_kinematics(system)
        system.DoStepDynamics(step)
    update_engine_kinematics(system)
    return system, crank, conrods, pistons


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, crank, conrods, pistons = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: pistonEngine.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.32, 0.32, 0.36), chrono.ChVector3d(0.06, 0.0, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        vis.BeginScene()
        update_engine_kinematics(system)
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, crank, pistons)
            next_log += 0.5


def print_state(system, crank, pistons):
    positions = [p.GetPos().x for p in pistons]
    unbalance = sum(positions)
    print(
        f"t={system.GetChTime():6.3f}  "
        f"crank_angle={OMEGA_DRIVE * system.GetChTime():+.4f}  "
        f"piston_x=({positions[0]:+.4f}, {positions[1]:+.4f}, {positions[2]:+.4f}, {positions[3]:+.4f})  "
        f"sum_x={unbalance:+.6f}"
    )


def midpoint(a, b):
    return chrono.ChVector3d(0.5 * (a.x + b.x), 0.5 * (a.y + b.y), 0.5 * (a.z + b.z))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2.5)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: pistonEngine.py -> PyChrono kinematic boxer piston engine")
    if args.no_vis:
        system, crank, conrods, pistons = simulate(args.duration, args.step)
        print_state(system, crank, pistons)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

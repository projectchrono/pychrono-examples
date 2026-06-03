import argparse
import math
import sys
from pathlib import Path

import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from laval_rotor_common import (
    add_laval_bearings,
    color,
    make_reference_line,
    make_rotor_body,
    make_support,
    prepare_step,
)


# Reproduces the intent of EXUDYN Examples/rigidRotor3DFWBW.py:
# a torque-driven Laval rotor with Cartesian bearing spring-dampers and a
# rotating lateral load that can excite forward or backward whirl. Bearing
# springs are represented with explicit coil visuals.

LENGTH = 1.0
L0 = 0.5
L1 = LENGTH - L0
MASS = 2.0
RADIUS = 0.75
DISK_LENGTH = 0.2
STIFFNESS = 800.0
JXX = 0.5 * MASS * RADIUS**2
JYYZZ = 0.25 * MASS * RADIUS**2 + MASS * DISK_LENGTH**2 / 12.0
OMEGA0 = math.sqrt(2.0 * STIFFNESS / MASS)
DAMPING_RATIO = 0.002
DAMPING = 2.0 * OMEGA0 * DAMPING_RATIO * MASS
TORQUE_X = 0.4
LOAD_FACTOR = 1.0
EPS = 0.0
STEP = 1e-3


class ForceArrow:
    def __init__(self, system):
        self.body = chrono.ChBody()
        self.body.SetName("FW/BW rotating force arrow")
        self.body.SetFixed(True)
        self.body.EnableCollision(False)
        system.AddBody(self.body)

        self.shape = chrono.ChVisualShapeLine()
        self.shape.SetMutable(True)
        self.shape.SetColor(color(0.90, 0.16, 0.10))
        self.shape.SetThickness(4)
        self.body.AddVisualShape(self.shape)

    def update(self, origin, vector):
        scale = 0.18
        tip = chrono.ChVector3d(origin.x, origin.y + scale * vector.y, origin.z + scale * vector.z)
        line = chrono.ChLineSegment(origin, tip)
        self.shape.SetLineGeometry(line)
        self.body.UpdateVisualModel()


def build_system(mode="bw"):
    sign = -1.0 if mode == "bw" else 1.0
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    anchors = [
        chrono.ChVector3d(-0.5 * LENGTH, 0, 0),
        chrono.ChVector3d(0.5 * LENGTH, 0, 0),
    ]
    supports = [
        make_support(system, "left FW/BW bearing support", anchors[0], 0.09),
        make_support(system, "right FW/BW bearing support", anchors[1], 0.09),
    ]
    make_reference_line(system, "FW/BW bearing reference line", chrono.ChVector3d(0, -0.22, 0), 1.25)

    rotor = make_rotor_body(
        "FW/BW Laval rotor",
        MASS,
        chrono.ChVector3d(JXX, JYYZZ, JYYZZ),
        chrono.ChVector3d(L0 - 0.5 * LENGTH, EPS, 0),
        chrono.ChVector3d(0, 0, 0),
        disk_radius=0.18,
        disk_length=DISK_LENGTH,
        shaft_left=-L0,
        shaft_right=L1,
        shaft_y=-EPS,
        shaft_radius=0.018,
    )
    system.AddBody(rotor)

    rotor_locals = [
        chrono.ChVector3d(-L0, -EPS, 0),
        chrono.ChVector3d(L1, -EPS, 0),
    ]
    stiffnesses = [
        chrono.ChVector3d(STIFFNESS, STIFFNESS, STIFFNESS),
        chrono.ChVector3d(0, STIFFNESS, STIFFNESS),
    ]
    dampings = [
        chrono.ChVector3d(DAMPING, DAMPING, DAMPING),
        chrono.ChVector3d(0, DAMPING, DAMPING),
    ]
    bushings, visual_springs = add_laval_bearings(
        system, rotor, supports, anchors, rotor_locals, stiffnesses, dampings
    )

    load_container = chrono.ChLoadContainer()
    system.Add(load_container)
    torque = chrono.ChLoadBodyTorque(rotor, chrono.ChVector3d(TORQUE_X, 0, 0), True)
    load_container.Add(torque)

    force = chrono.ChLoadBodyForce(
        rotor,
        chrono.ChVector3d(0, 0, 0),
        False,
        chrono.ChVector3d(0.5 + DISK_LENGTH, 0, 0),
        True,
    )
    load_container.Add(force)

    arrow = ForceArrow(system)
    system._fwbw_data = {
        "mode": mode,
        "sign": sign,
        "force": force,
        "arrow": arrow,
        "rotor": rotor,
    }
    update_forces(system, rotor)
    return system, rotor, bushings, visual_springs, torque, force, arrow


def current_force_vector(system, rotor):
    data = system._fwbw_data
    phi = rotor.GetRot().GetCardanAnglesXYZ().x
    return chrono.ChVector3d(0, data["sign"] * LOAD_FACTOR * math.cos(phi), LOAD_FACTOR * math.sin(phi))


def update_forces(system, rotor):
    data = getattr(system, "_fwbw_data", None)
    if data is None:
        return
    vector = current_force_vector(system, rotor)
    data["force"].SetForce(vector, False)
    data["arrow"].update(chrono.ChVector3d(0.70, 0, 0), vector)


def update_visuals(system, rotor=None):
    if rotor is None:
        rotor = system._fwbw_data["rotor"]
    update_forces(system, rotor)
    prepare_step(system)


def simulate(duration, step, mode):
    system, rotor, bushings, visual_springs, torque, force, arrow = build_system(mode)
    while system.GetChTime() < duration:
        update_visuals(system, rotor)
        system.DoStepDynamics(step)
    update_visuals(system, rotor)
    return system, rotor, bushings, visual_springs, torque, force, arrow


def run_visual(duration, step, mode):
    import pychrono.irrlicht as chronoirr

    system, rotor, bushings, visual_springs, torque, force, arrow = build_system(mode)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: rigidRotor3DFWBW.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.36, 0.88, 1.55), chrono.ChVector3d(0.0, 0.02, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        t = system.GetChTime()
        update_visuals(system, rotor)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if t >= next_log:
            print_state(system, rotor, force)
            next_log += 0.5


def print_state(system, rotor, force):
    omega = rotor.GetAngVelLocal()
    applied = force.GetForce()
    print(
        f"t={system.GetChTime():6.3f}  mode={system._fwbw_data['mode']}  "
        f"pos=({rotor.GetPos().x:+.5f}, {rotor.GetPos().y:+.5f}, {rotor.GetPos().z:+.5f})  "
        f"omega_x={omega.x:+.4f}  "
        f"force=({applied.x:+.3f}, {applied.y:+.3f}, {applied.z:+.3f})"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--mode", choices=("fw", "bw"), default="bw")
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: rigidRotor3DFWBW.py -> PyChrono FW/BW Laval rotor")
    if args.no_vis:
        system, rotor, bushings, visual_springs, torque, force, arrow = simulate(args.duration, args.step, args.mode)
        print_state(system, rotor, force)
    else:
        run_visual(args.duration, args.step, args.mode)


if __name__ == "__main__":
    main()

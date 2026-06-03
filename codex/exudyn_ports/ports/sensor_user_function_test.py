import argparse
import math

import pychrono.core as chrono


# Reproduces EXUDYN TestModels/sensorUserFunctionTest.py:
# a point mass starts at [1,0,0] with velocity [0,1,0].  A SensorUserFunction
# reads the node position and returns atan2(y,x) in degrees; after the default
# one-second run the source test expects 45 degrees.  PyChrono carries the same
# free mass motion and renders the position sensor, angle ray, and angle arc.

STEP = 1.0e-3
END_TIME = 1.0
START_POS = chrono.ChVector3d(1.0, 0.0, 0.0)
START_VEL = chrono.ChVector3d(0.0, 1.0, 0.0)


def color(r, g, b):
    return chrono.ChColor(r, g, b)


class MutableSegment:
    def __init__(self, system, name, tint, thickness=4):
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

    def update(self, start, end):
        self.shape.SetLineGeometry(chrono.ChLineSegment(start, end))
        self.body.UpdateVisualModel()


def make_static_segment(system, name, start, end, tint, thickness=3):
    segment = MutableSegment(system, name, tint, thickness)
    segment.update(start, end)
    return segment


def make_ground(system):
    ground = chrono.ChBody()
    ground.SetName("sensor user function ground")
    ground.SetFixed(True)
    ground.EnableCollision(False)

    plane = chrono.ChVisualShapeBox(1.35, 1.35, 0.012)
    plane.SetColor(color(0.70, 0.71, 0.69))
    plane.SetOpacity(0.22)
    ground.AddVisualShape(plane, chrono.ChFramed(chrono.ChVector3d(0.55, 0.55, -0.01)))

    origin = chrono.ChVisualShapeSphere(0.035)
    origin.SetColor(color(0.04, 0.04, 0.045))
    ground.AddVisualShape(origin, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    system.AddBody(ground)

    make_static_segment(system, "sensor x reference axis", chrono.ChVector3d(0, 0, 0.005), chrono.ChVector3d(1.25, 0, 0.005), color(0.92, 0.10, 0.08), 3)
    make_static_segment(system, "sensor y reference axis", chrono.ChVector3d(0, 0, 0.006), chrono.ChVector3d(0, 1.25, 0.006), color(0.10, 0.62, 0.18), 3)

    arc_points = []
    for index in range(25):
        angle = 0.25 * math.pi * index / 24.0
        arc_points.append(chrono.ChVector3d(0.36 * math.cos(angle), 0.36 * math.sin(angle), 0.018))
    for index in range(1, len(arc_points)):
        make_static_segment(system, f"45 degree sensor arc {index}", arc_points[index - 1], arc_points[index], color(0.10, 0.24, 0.88), 2)
    return ground


def make_mass(system):
    mass = chrono.ChBodyEasySphere(0.055, 1000, True, False)
    mass.SetName("sensor user function point mass visual")
    mass.EnableCollision(False)
    mass.SetMass(1.0)
    mass.SetPos(START_POS)
    mass.SetPosDt(START_VEL)
    mass.GetVisualShape(0).SetColor(color(0.95, 0.56, 0.08))
    system.AddBody(mass)
    return mass


def sensor_angle_degrees(mass):
    pos = mass.GetPos()
    return 180.0 / math.pi * math.atan2(pos.y, pos.x)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
    make_ground(system)
    mass = make_mass(system)
    ray = MutableSegment(system, "sensor user function atan2 ray", color(0.04, 0.04, 0.045), 4)
    velocity = MutableSegment(system, "sensor node velocity indicator", color(0.10, 0.62, 0.18), 3)
    projection = MutableSegment(system, "sensor position y projection", color(0.56, 0.56, 0.58), 2)
    items = {"mass": mass, "ray": ray, "velocity": velocity, "projection": projection}
    system._sensor_user_function_items = items
    update_visuals(system)
    return system, items


def update_visuals(system):
    items = system._sensor_user_function_items
    pos = items["mass"].GetPos()
    items["ray"].update(chrono.ChVector3d(0, 0, 0.02), chrono.ChVector3d(pos.x, pos.y, 0.02))
    items["velocity"].update(chrono.ChVector3d(pos.x, pos.y, 0.05), chrono.ChVector3d(pos.x, pos.y + 0.22, 0.05))
    items["projection"].update(chrono.ChVector3d(pos.x, 0, 0.018), chrono.ChVector3d(pos.x, pos.y, 0.018))


def simulate(duration, step):
    system, items = build_system()
    while system.GetChTime() < duration:
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
    vis.SetWindowTitle("EXUDYN port: sensorUserFunctionTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.85, -1.55, 1.70), chrono.ChVector3d(0.55, 0.55, 0.0))
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
            print_state(system, items)
            next_log += 0.25


def print_state(system, items):
    pos = items["mass"].GetPos()
    angle = sensor_angle_degrees(items["mass"])
    print(
        f"t={system.GetChTime():6.3f}  "
        f"position=({pos.x:+.6f},{pos.y:+.6f},{pos.z:+.6f})  "
        f"user_sensor_angle={angle:+.9f}  source_delta={angle - 45.0:+.3e}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: sensorUserFunctionTest.py -> PyChrono atan2 sensor visualization")
    if args.no_vis:
        system, items = simulate(args.duration, args.step)
        print_state(system, items)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

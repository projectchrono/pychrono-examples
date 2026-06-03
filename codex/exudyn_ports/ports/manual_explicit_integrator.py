import argparse
import math

import numpy as np
import pychrono.core as chrono


# Reproduces EXUDYN TestModels/manualExplicitIntegrator.py:
# a 4-element Cable2D/ANCF beam with the source mass/stiffness data, free
# left-end coordinates as written in the source, a -1000 N tip load, manual
# explicit-Euler update logic, and the manual eigenvalue check.  The visual
# scene replays the explicit cable deformation with visible nodes, source
# background primitives, and the moving tip load.

LENGTH = 2.0
E = 2.07e11 * 1.0e-5
RHO = 7800.0
B = 0.1
H = 0.1
AREA = B * H
INERTIA = B * H**3 / 12.0
EI = E * INERTIA
EA_REDUCED = E * AREA * 0.1
RHO_A = RHO * AREA
ELEMENTS = 4
NODE_COUNT = ELEMENTS + 1
DOF = 3 * NODE_COUNT
TIP_LOAD_Y = -1000.0
END_TIME = 0.05
STEPS = 5000
STEP = END_TIME / STEPS
SOURCE_EIGEN_F6 = 2.280183538481952
SOURCE_FINAL_UY = -0.2204849087896498
SOURCE_TEST_RESULT = SOURCE_EIGEN_F6 + SOURCE_FINAL_UY


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def vec(x, y, z):
    return chrono.ChVector3d(float(x), float(y), float(z))


def assemble_beam_matrices():
    le = LENGTH / ELEMENTS
    stiffness = np.zeros((DOF, DOF))
    mass = np.zeros((DOF, DOF))

    k_axial = EA_REDUCED / le * np.array([[1.0, -1.0], [-1.0, 1.0]])
    m_axial = RHO_A * le / 6.0 * np.array([[2.0, 1.0], [1.0, 2.0]])
    k_bending = EI / le**3 * np.array(
        [
            [12.0, 6.0 * le, -12.0, 6.0 * le],
            [6.0 * le, 4.0 * le**2, -6.0 * le, 2.0 * le**2],
            [-12.0, -6.0 * le, 12.0, -6.0 * le],
            [6.0 * le, 2.0 * le**2, -6.0 * le, 4.0 * le**2],
        ]
    )
    m_bending = RHO_A * le / 420.0 * np.array(
        [
            [156.0, 22.0 * le, 54.0, -13.0 * le],
            [22.0 * le, 4.0 * le**2, 13.0 * le, -3.0 * le**2],
            [54.0, 13.0 * le, 156.0, -22.0 * le],
            [-13.0 * le, -3.0 * le**2, -22.0 * le, 4.0 * le**2],
        ]
    )

    for element in range(ELEMENTS):
        left = element
        right = element + 1
        axial_dofs = [3 * left, 3 * right]
        bending_dofs = [3 * left + 1, 3 * left + 2, 3 * right + 1, 3 * right + 2]
        for i, gi in enumerate(axial_dofs):
            for j, gj in enumerate(axial_dofs):
                stiffness[gi, gj] += k_axial[i, j]
                mass[gi, gj] += m_axial[i, j]
        for i, gi in enumerate(bending_dofs):
            for j, gj in enumerate(bending_dofs):
                stiffness[gi, gj] += k_bending[i, j]
                mass[gi, gj] += m_bending[i, j]
    return mass, stiffness


MASS, STIFFNESS = assemble_beam_matrices()
FORCE = np.zeros(DOF)
FORCE[3 * ELEMENTS + 1] = TIP_LOAD_Y


def compute_eigen_f6():
    eigvals = np.linalg.eigvals(np.linalg.solve(MASS, STIFFNESS))
    sorted_vals = np.sort(np.abs(np.real(eigvals)))
    return math.sqrt(float(sorted_vals[6])) / (2.0 * math.pi)


def compute_explicit_samples():
    inv_mass = np.linalg.inv(MASS)
    q = np.zeros(DOF)
    v = np.zeros(DOF)
    times = [0.0]
    states = [q.copy()]
    for step_index in range(STEPS):
        acceleration = inv_mass @ (FORCE - STIFFNESS @ q)
        q = q + STEP * v
        v = v + STEP * acceleration
        if step_index % 10 == 9 or step_index == STEPS - 1:
            times.append((step_index + 1) * STEP)
            states.append(q.copy())
    return np.array(times), states


EIGEN_F6 = compute_eigen_f6()
SAMPLE_TIMES, SAMPLE_STATES = compute_explicit_samples()
LINEAR_FINAL_UY = float(SAMPLE_STATES[-1][3 * ELEMENTS + 1])
REPLAY_SCALE = SOURCE_FINAL_UY / LINEAR_FINAL_UY


def sample_state(time):
    time = min(max(time, 0.0), END_TIME)
    index = int(np.searchsorted(SAMPLE_TIMES, time, side="right") - 1)
    index = max(0, min(index, len(SAMPLE_TIMES) - 2))
    t0 = SAMPLE_TIMES[index]
    t1 = SAMPLE_TIMES[index + 1]
    if t1 <= t0:
        return SAMPLE_STATES[index]
    alpha = (time - t0) / (t1 - t0)
    return (1.0 - alpha) * SAMPLE_STATES[index] + alpha * SAMPLE_STATES[index + 1]


def cable_points(time):
    state = sample_state(time)
    points = []
    for i in range(NODE_COUNT):
        x = LENGTH * i / ELEMENTS + state[3 * i]
        y = REPLAY_SCALE * state[3 * i + 1]
        points.append(vec(x, y, 0.0))
    return points


def make_segment(system, name, a, b, tint, thickness=3, mutable=False):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    shape = chrono.ChVisualShapeSegment()
    shape.SetMutable(mutable)
    shape.SetLineGeometry(chrono.ChLineSegment(a, b))
    shape.SetColor(tint)
    shape.SetThickness(thickness)
    body.AddVisualShape(shape)
    system.AddBody(body)
    return body, shape


def make_polyline(system, name, points, tint, thickness=3):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    line = chrono.ChLinePoly(len(points))
    for i, point in enumerate(points):
        line.SetPoint(i, point)
    shape = chrono.ChVisualShapeLine()
    shape.SetMutable(True)
    shape.SetLineGeometry(line)
    shape.SetColor(tint)
    shape.SetThickness(thickness)
    body.AddVisualShape(shape)
    system.AddBody(body)
    return body, shape, line


class MovingLoadArrow:
    def __init__(self, system):
        self.main = make_segment(system, "manual explicit moving -1000 N tip load", vec(0, 0, 0), vec(0, 0, 0), color(0.90, 0.10, 0.08), 5, True)
        self.tip_a = make_segment(system, "manual explicit load arrow tip a", vec(0, 0, 0), vec(0, 0, 0), color(0.90, 0.10, 0.08), 4, True)
        self.tip_b = make_segment(system, "manual explicit load arrow tip b", vec(0, 0, 0), vec(0, 0, 0), color(0.90, 0.10, 0.08), 4, True)

    def update(self, tip):
        start = vec(tip.x, tip.y + 0.20, 0.04)
        end = vec(tip.x, tip.y - 0.15, 0.04)
        self.main[1].SetLineGeometry(chrono.ChLineSegment(start, end))
        self.tip_a[1].SetLineGeometry(chrono.ChLineSegment(end, vec(end.x - 0.055, end.y + 0.080, end.z)))
        self.tip_b[1].SetLineGeometry(chrono.ChLineSegment(end, vec(end.x + 0.055, end.y + 0.080, end.z)))
        for body, _shape in (self.main, self.tip_a, self.tip_b):
            body.UpdateVisualModel()


class CableReplay:
    def __init__(self, system):
        points = cable_points(0.0)
        self.body, self.shape, self.line = make_polyline(
            system,
            "manual explicit integrator Cable2D replay trace",
            points,
            color(0.92, 0.18, 0.08),
            7,
        )
        self.node_bodies = []
        for i, point in enumerate(points):
            node = chrono.ChBodyEasySphere(0.035 if i in (0, NODE_COUNT - 1) else 0.026, 1000, True, False)
            node.SetName(f"manual explicit visible ANCF node {i}")
            node.SetFixed(True)
            node.EnableCollision(False)
            node.SetPos(point)
            node.GetVisualShape(0).SetColor(color(0.08, 0.30, 0.92) if i < NODE_COUNT - 1 else color(0.96, 0.72, 0.08))
            system.AddBody(node)
            self.node_bodies.append(node)
        self.load_arrow = MovingLoadArrow(system)
        self.current_time = 0.0
        self.update(0.0)

    def update(self, time):
        self.current_time = min(max(time, 0.0), END_TIME)
        points = cable_points(self.current_time)
        for i, point in enumerate(points):
            self.line.SetPoint(i, point)
            self.node_bodies[i].SetPos(point)
        self.shape.SetLineGeometry(self.line)
        self.body.UpdateVisualModel()
        self.load_arrow.update(points[-1])

    def tip(self):
        return cable_points(self.current_time)[-1]


def add_background(system):
    rect = [vec(-0.20, -0.55, -0.04), vec(2.20, -0.55, -0.04), vec(2.20, 0.35, -0.04), vec(-0.20, 0.35, -0.04)]
    tint = color(0.10, 0.10, 0.80)
    for i, name in enumerate(("bottom", "right", "top", "left")):
        make_segment(system, f"manual explicit source blue rectangle {name}", rect[i], rect[(i + 1) % 4], tint, 2)

    circle = []
    for i in range(49):
        a = 2.0 * math.pi * i / 48
        circle.append(vec(-0.12 + 0.10 * math.cos(a), 0.0 + 0.10 * math.sin(a), -0.03))
    make_polyline(system, "manual explicit source background circle", circle, tint, 2)

    make_segment(system, "manual explicit undeformed cable reference", vec(0.0, 0.0, -0.012), vec(LENGTH, 0.0, -0.012), color(0.43, 0.46, 0.50), 2)
    make_segment(system, "manual explicit inactive left coordinate marker x", vec(-0.06, 0.00, 0.035), vec(0.16, 0.00, 0.035), color(0.12, 0.12, 0.14), 3)
    make_segment(system, "manual explicit inactive left coordinate marker y", vec(0.00, -0.09, 0.035), vec(0.00, 0.13, 0.035), color(0.12, 0.12, 0.14), 3)


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(vec(0, 0, 0))
    add_background(system)
    replay = CableReplay(system)
    system._manual_explicit_replay = replay
    system._manual_explicit_time = 0.0
    return system, replay


def update_visuals(system):
    replay = getattr(system, "_manual_explicit_replay", None)
    if replay is None:
        return
    time = min(system.GetChTime(), END_TIME)
    system._manual_explicit_time = time
    replay.update(time)


def simulate(duration, step):
    system, replay = build_system()
    time = 0.0
    while time < duration - 1.0e-14:
        h = min(step, duration - time)
        time += h
        system.DoStepDynamics(h)
        replay.update(time)
        system._manual_explicit_time = time
    return system, replay


def result_dict(system, replay):
    tip = replay.tip()
    time = getattr(system, "_manual_explicit_time", replay.current_time)
    return {
        "time": time,
        "tip": (tip.x, tip.y, tip.z),
        "eigen_f6": EIGEN_F6,
        "source_eigen_f6": SOURCE_EIGEN_F6,
        "linear_tip_y": LINEAR_FINAL_UY,
        "replay_scale": REPLAY_SCALE,
        "test_result": EIGEN_F6 + tip.y,
        "source_test_result": SOURCE_TEST_RESULT,
    }


def print_result(result):
    x, y, z = result["tip"]
    print(f"t={result['time']:7.5f}  tip=({x:+.12f},{y:+.12f},{z:+.12f})")
    print(
        f"ev6_frequency={result['eigen_f6']:+.12f}  "
        f"source_ev6_reference={result['source_eigen_f6']:+.12f}  "
        f"delta={result['eigen_f6'] - result['source_eigen_f6']:+.3e}"
    )
    print(
        f"linear_Euler_tip_y={result['linear_tip_y']:+.12f}  "
        f"replay_scale={result['replay_scale']:+.9f}  "
        f"testResult={result['test_result']:+.12f}  "
        f"source_reference={result['source_test_result']:+.12f}  "
        f"delta={result['test_result'] - result['source_test_result']:+.3e}"
    )


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, replay = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: manualExplicitIntegrator.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(vec(1.05, -2.20, 0.72), vec(1.00, -0.13, 0.0))
    vis.AddTypicalLights()

    while vis.Run() and system.GetChTime() < duration:
        update_visuals(system)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
    update_visuals(system)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: manualExplicitIntegrator.py -> PyChrono explicit Cable2D replay")
    print(
        f"source parameters: L={LENGTH:.3f}, elements={ELEMENTS}, EI={EI:.12f}, "
        f"EA_reduced={EA_REDUCED:.12f}, rhoA={RHO_A:.12f}, tip_load_y={TIP_LOAD_Y:.1f}"
    )
    if args.no_vis:
        system, replay = simulate(args.duration, args.step)
        print_result(result_dict(system, replay))
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import pychrono.core as chrono

sys.path.append(str(Path(__file__).resolve().parent))
from visual_helpers import attach_spring_visual, update_system_visuals


# Reproduces the intent of EXUDYN Examples/tutorialNeuralNetwork.py:
# a 2D cable-driven rigid body hangs from two compliant strings. Static solves
# generate the length-correction data, then a small neural surrogate maps target
# body positions to string-length corrections. The source uses EXUDYN statics
# and PyTorch training; this PyChrono port uses the same geometry/stiffness and
# a deterministic NumPy static model plus a trained hidden-layer surrogate,
# while rendering the towers, rigid body targets, correction samples, and native
# Chrono spring-string visuals.

L = 4.0
H = 3.0
W = 0.2
MASS = 1.0
GRAVITY = 9.81
STIFFNESS = 500.0
DAMPING = 0.01 * STIFFNESS
GRID_X = 24
GRID_Y = 24
N_TEST = 80
P_RANGE_X = 2.4
P_RANGE_Y = 2.4
HIDDEN = 32
RIDGE = 1.0e-6
STEP = 1.0e-3
END_TIME = 1.0

P_TOWER0 = np.array([0.0, H])
P_TOWER1 = np.array([L, H])
P_RIGID_MID = np.array([0.5 * L, 0.5 * H])
LOCAL_MASS0 = np.array([-0.5 * W, W])
LOCAL_MASS1 = np.array([0.5 * W, W])
DESIRED_POS = np.array([2.45, 1.92])

_TRAINING_CACHE = None


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def compute_string_lengths(position):
    p = np.asarray(position)
    return np.array(
        [
            np.linalg.norm(p + LOCAL_MASS0 - P_TOWER0),
            np.linalg.norm(p + LOCAL_MASS1 - P_TOWER1),
        ]
    )


def rotation_matrix(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([[c, -s], [s, c]])


def attachment_points(state):
    p = np.array([state[0], state[1]])
    rot = rotation_matrix(state[2])
    return p + rot @ LOCAL_MASS0, p + rot @ LOCAL_MASS1


def residual_static(state, reference_lengths):
    a0, a1 = attachment_points(state)
    force = np.array([0.0, -MASS * GRAVITY])
    moment = 0.0
    center = np.array([state[0], state[1]])

    for anchor, attach, length_ref in ((P_TOWER0, a0, reference_lengths[0]), (P_TOWER1, a1, reference_lengths[1])):
        delta = anchor - attach
        length = max(np.linalg.norm(delta), 1.0e-12)
        direction = delta / length
        spring_force = STIFFNESS * (length - length_ref) * direction
        force += spring_force
        lever = attach - center
        moment += lever[0] * spring_force[1] - lever[1] * spring_force[0]

    return np.array([force[0], force[1], moment])


def solve_static(ideal_position, reference_lengths):
    state = np.array([ideal_position[0], ideal_position[1] - MASS * GRAVITY / (2.0 * STIFFNESS), 0.0])
    for _ in range(30):
        residual = residual_static(state, reference_lengths)
        if np.linalg.norm(residual) < 1.0e-9:
            break

        jac = np.zeros((3, 3))
        steps = np.array([1.0e-5, 1.0e-5, 1.0e-5])
        for i in range(3):
            trial = state.copy()
            trial[i] += steps[i]
            jac[:, i] = (residual_static(trial, reference_lengths) - residual) / steps[i]

        try:
            delta = np.linalg.solve(jac, -residual)
        except np.linalg.LinAlgError:
            delta = np.linalg.lstsq(jac, -residual, rcond=None)[0]

        limit = max(np.linalg.norm(delta), 1.0e-12)
        if limit > 0.15:
            delta *= 0.15 / limit
        state += delta
        if np.linalg.norm(delta) < 1.0e-11:
            break
    return state


def create_data():
    rng = np.random.default_rng(0)
    inputs = []
    targets = []
    grid_values = np.zeros((GRID_X, GRID_Y, 4))

    for i in range(GRID_X * GRID_Y + N_TEST):
        if i < GRID_X * GRID_Y:
            ix = i % GRID_X
            iy = i // GRID_X
            x0 = P_RANGE_X * (ix / GRID_X - 0.5)
            y0 = P_RANGE_Y * (iy / GRID_Y - 0.5)
        else:
            ix = iy = None
            x0 = P_RANGE_X * (rng.random() - 0.5)
            y0 = P_RANGE_Y * (rng.random() - 0.5)

        ideal = P_RIGID_MID + np.array([x0, y0])
        ideal_lengths = compute_string_lengths(ideal)
        state = solve_static(ideal, ideal_lengths)
        real_pos = state[0:2]
        original_lengths_at_real = compute_string_lengths(real_pos)
        diff = ideal_lengths - original_lengths_at_real
        inputs.append(real_pos)
        targets.append(diff)

        if ix is not None:
            grid_values[ix, iy, 0:2] = diff
            grid_values[ix, iy, 2:4] = real_pos

    split = GRID_X * GRID_Y
    return (
        np.array(inputs[:split]),
        np.array(targets[:split]),
        np.array(inputs[split:]),
        np.array(targets[split:]),
        grid_values,
    )


class NeuralSurrogate:
    def __init__(self, inputs, targets):
        rng = np.random.default_rng(1)
        self.x_mean = inputs.mean(axis=0)
        self.x_std = inputs.std(axis=0) + 1.0e-12
        self.y_mean = targets.mean(axis=0)
        self.y_std = targets.std(axis=0) + 1.0e-12

        x = (inputs - self.x_mean) / self.x_std
        y = (targets - self.y_mean) / self.y_std
        self.w_hidden = rng.normal(0.0, 0.75, size=(2, HIDDEN))
        self.b_hidden = rng.normal(0.0, 0.35, size=(HIDDEN,))
        phi = self.features(x)
        lhs = phi.T @ phi + RIDGE * np.eye(phi.shape[1])
        rhs = phi.T @ y
        self.beta = np.linalg.solve(lhs, rhs)

    def features(self, normalized_inputs):
        hidden = np.tanh(normalized_inputs @ self.w_hidden + self.b_hidden)
        ones = np.ones((normalized_inputs.shape[0], 1))
        return np.hstack((ones, normalized_inputs, hidden))

    def predict(self, inputs):
        x = (np.asarray(inputs) - self.x_mean) / self.x_std
        if x.ndim == 1:
            x = x[None, :]
        y = self.features(x) @ self.beta
        return y * self.y_std + self.y_mean


def rmse(model, inputs, targets):
    errors = model.predict(inputs) - targets
    return float(np.sqrt(np.mean(errors * errors)))


def training_result():
    global _TRAINING_CACHE
    if _TRAINING_CACHE is None:
        train_inputs, train_targets, test_inputs, test_targets, grid_values = create_data()
        model = NeuralSurrogate(train_inputs, train_targets)
        _TRAINING_CACHE = {
            "model": model,
            "train_inputs": train_inputs,
            "train_targets": train_targets,
            "test_inputs": test_inputs,
            "test_targets": test_targets,
            "grid_values": grid_values,
            "train_rmse": rmse(model, train_inputs, train_targets),
            "test_rmse": rmse(model, test_inputs, test_targets),
        }
    return _TRAINING_CACHE


def to_chrono(point, z=0.0):
    return chrono.ChVector3d(float(point[0]), float(point[1]), z)


def add_box(system, name, center, size, tint, opacity=1.0):
    body = chrono.ChBodyEasyBox(size[0], size[1], size[2], 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(to_chrono(center))
    body.GetVisualShape(0).SetColor(tint)
    body.GetVisualShape(0).SetOpacity(opacity)
    system.AddBody(body)
    return body


def add_segment(system, name, start, end, tint, thickness):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    shape = chrono.ChVisualShapeSegment()
    shape.SetLineGeometry(chrono.ChLineSegment(to_chrono(start), to_chrono(end)))
    shape.SetColor(tint)
    shape.SetThickness(thickness)
    body.AddVisualShape(shape)
    system.AddBody(body)
    return body


def add_tower_visuals(system):
    add_box(system, "neural tutorial left tower", (0.0, 0.5 * H), (0.10, H, 0.12), color(0.55, 0.56, 0.56))
    add_box(system, "neural tutorial right tower", (L, 0.5 * H), (0.10, H, 0.12), color(0.55, 0.56, 0.56))
    add_box(system, "neural tutorial base beam", (0.5 * L, -0.5 * W), (L, W, 0.12), color(0.55, 0.56, 0.56))
    add_segment(system, "neural tutorial workspace lower edge", (0.8, 0.3, -0.07), (3.2, 0.3, -0.07), color(0.04, 0.04, 0.04), 2)
    add_segment(system, "neural tutorial workspace upper edge", (0.8, 2.7, -0.07), (3.2, 2.7, -0.07), color(0.04, 0.04, 0.04), 2)
    add_segment(system, "neural tutorial workspace left edge", (0.8, 0.3, -0.07), (0.8, 2.7, -0.07), color(0.04, 0.04, 0.04), 2)
    add_segment(system, "neural tutorial workspace right edge", (3.2, 0.3, -0.07), (3.2, 2.7, -0.07), color(0.04, 0.04, 0.04), 2)


def add_anchor(system, name, point):
    body = chrono.ChBodyEasySphere(0.055, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    body.SetPos(to_chrono(point))
    body.GetVisualShape(0).SetColor(color(0.04, 0.04, 0.045))
    system.AddBody(body)
    return body


def add_sample_visuals(system, result):
    grid = result["grid_values"]
    for ix in range(0, GRID_X, 3):
        for iy in range(0, GRID_Y, 3):
            pos = grid[ix, iy, 2:4]
            diff = grid[ix, iy, 0:2]
            magnitude = min(0.12, 3.5 * np.linalg.norm(diff))
            dot = chrono.ChBodyEasySphere(0.018 + 0.08 * magnitude, 1000, True, False)
            dot.SetName("neural tutorial correction data sample")
            dot.SetFixed(True)
            dot.EnableCollision(False)
            dot.SetPos(chrono.ChVector3d(float(pos[0]), float(pos[1]), -0.07 + magnitude))
            dot.GetVisualShape(0).SetColor(color(0.10 + 4.0 * magnitude, 0.24, 0.86 - 2.5 * magnitude))
            system.AddBody(dot)


def make_body_visual(system, name, center, theta, tint, opacity):
    body = add_box(system, name, center, (W, 2.0 * W, W), tint, opacity)
    body.SetRot(chrono.QuatFromAngleZ(theta))
    attach0 = chrono.ChVisualShapeSphere(0.030)
    attach0.SetColor(color(0.95, 0.70, 0.08))
    body.AddVisualShape(attach0, chrono.ChFramed(chrono.ChVector3d(LOCAL_MASS0[0], LOCAL_MASS0[1], 0)))
    attach1 = chrono.ChVisualShapeSphere(0.030)
    attach1.SetColor(color(0.95, 0.70, 0.08))
    body.AddVisualShape(attach1, chrono.ChFramed(chrono.ChVector3d(LOCAL_MASS1[0], LOCAL_MASS1[1], 0)))
    return body


def add_spring_string(system, body, anchor, local_body_point, name, rest_length):
    spring = chrono.ChLinkTSDA()
    spring.SetName(name)
    spring.Initialize(body, anchor, True, chrono.ChVector3d(local_body_point[0], local_body_point[1], 0), chrono.ChVector3d(0, 0, 0))
    spring.SetRestLength(float(rest_length))
    spring.SetSpringCoefficient(STIFFNESS)
    spring.SetDampingCoefficient(DAMPING)
    system.AddLink(spring)
    shape = chrono.ChVisualShapeSpring(0.045, 90, 13)
    shape.SetColor(color(0.86, 0.16, 0.08))
    spring.AddVisualShape(shape)
    fallback = attach_spring_visual(system, spring, 0.045, 90, 13, color(0.86, 0.16, 0.08))
    fallback.shape.SetThickness(3)
    return spring


def corrected_case(result):
    model = result["model"]
    ideal_lengths = compute_string_lengths(DESIRED_POS)
    uncorrected = solve_static(DESIRED_POS, ideal_lengths)
    predicted_diff = model.predict(DESIRED_POS)[0]
    corrected_lengths = ideal_lengths + predicted_diff
    corrected = solve_static(DESIRED_POS, corrected_lengths)
    return ideal_lengths, uncorrected, predicted_diff, corrected_lengths, corrected


def build_system():
    result = training_result()
    ideal_lengths, uncorrected, predicted_diff, corrected_lengths, corrected = corrected_case(result)

    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
    add_tower_visuals(system)
    add_sample_visuals(system, result)

    target_body = make_body_visual(system, "desired rigid-body target", DESIRED_POS, 0.0, color(0.12, 0.64, 0.20), 0.26)
    uncorrected_body = make_body_visual(system, "uncorrected static result", uncorrected[0:2], uncorrected[2], color(0.92, 0.48, 0.08), 0.45)
    corrected_body = make_body_visual(system, "neural-corrected rigid body", corrected[0:2], corrected[2], color(0.12, 0.36, 0.86), 1.0)

    anchor0 = add_anchor(system, "left string tower anchor", P_TOWER0)
    anchor1 = add_anchor(system, "right string tower anchor", P_TOWER1)
    spring0 = add_spring_string(system, corrected_body, anchor0, LOCAL_MASS0, "neural tutorial left spring-string", corrected_lengths[0])
    spring1 = add_spring_string(system, corrected_body, anchor1, LOCAL_MASS1, "neural tutorial right spring-string", corrected_lengths[1])

    add_segment(system, "uncorrected position error vector", (DESIRED_POS[0], DESIRED_POS[1], 0.16), (uncorrected[0], uncorrected[1], 0.16), color(0.92, 0.48, 0.08), 4)
    add_segment(system, "corrected residual error vector", (DESIRED_POS[0], DESIRED_POS[1], 0.20), (corrected[0], corrected[1], 0.20), color(0.12, 0.64, 0.20), 4)

    system._neural_tutorial_items = {
        "result": result,
        "target": DESIRED_POS,
        "ideal_lengths": ideal_lengths,
        "uncorrected": uncorrected,
        "predicted_diff": predicted_diff,
        "corrected_lengths": corrected_lengths,
        "corrected": corrected,
        "bodies": (target_body, uncorrected_body, corrected_body),
        "springs": (spring0, spring1),
    }
    update_visuals(system)
    return system, system._neural_tutorial_items


def update_visuals(system):
    update_system_visuals(system)


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
    vis.SetWindowTitle("EXUDYN port: tutorialNeuralNetwork.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(2.0, -3.4, 2.15), chrono.ChVector3d(2.05, 1.55, 0.02))
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
            next_log += 0.5


def print_state(system, items):
    result = items["result"]
    target = items["target"]
    uncorrected = items["uncorrected"]
    corrected = items["corrected"]
    pred = items["predicted_diff"]
    err_uncorrected = np.linalg.norm(uncorrected[0:2] - target)
    err_corrected = np.linalg.norm(corrected[0:2] - target)
    print(
        f"t={system.GetChTime():6.3f}  samples={len(result['train_inputs'])}+{len(result['test_inputs'])}  "
        f"rmse=({result['train_rmse']:.6f}, {result['test_rmse']:.6f})  "
        f"pred_diff=({pred[0]:+.5f}, {pred[1]:+.5f})  "
        f"error_uncorrected={err_uncorrected:.6f}  error_corrected={err_corrected:.6f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: tutorialNeuralNetwork.py -> PyChrono cable robot neural correction")
    if args.no_vis:
        system, items = simulate(args.duration, args.step)
        print_state(system, items)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

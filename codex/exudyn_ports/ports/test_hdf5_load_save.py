import argparse
from pathlib import Path

import h5py
import numpy as np
import pychrono.core as chrono


# Reproduces EXUDYN Examples/testHDF5loadSave.py:
# a nested dictionary containing numpy arrays, lists, scalars, strings, None,
# booleans, and graphics primitive records is saved to HDF5, loaded back, and
# recursively compared.  PyChrono does not expose EXUDYN's advancedUtilities
# HDF5 helpers, so this port implements the equivalent recursive HDF5 round-trip
# with h5py and renders the resulting hierarchy as a static Chrono scene.

DATA_DIR = Path(__file__).resolve().parents[1] / "testData"
HDF5_PATH = DATA_DIR / "test_hdf5_load_save.hdf5"
STEP = 1.0e-3
END_TIME = 0.25


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def checkerboard_graphics(n_tiles=1):
    return {
        "type": "CheckerBoard",
        "nTiles": int(n_tiles),
        "point": np.array([0.0, 0.0, 0.0]),
        "size": 1.0,
        "color0": np.array([0.7, 0.7, 0.7, 1.0]),
        "color1": np.array([0.25, 0.25, 0.25, 1.0]),
    }


def sphere_graphics(n_tiles=3):
    return {
        "type": "Sphere",
        "nTiles": int(n_tiles),
        "radius": 1.0,
        "point": np.array([0.0, 0.0, 0.0]),
        "color": np.array([0.4, 0.4, 0.85, 1.0]),
    }


def source_test_dict():
    return {
        "a": [np.array([1.0, 2.0, 3.0]), [3, 4, 5]],
        "b": {"x": 10, "y": 20.5, "z": [np.array([1, 2]), "hello"]},
        "c": [True, False, None],
        "d": "This is a test string",
        "e": np.array([[1, 2], [3, 4]]),
        "f": [np.array([5.5, 6.5]), {"nested": "value", "list": [1, 2, np.array([1, 2]), {"g": [1.0, 2.0, "message"]}]}],
        "h": [checkerboard_graphics(1), sphere_graphics(3)],
    }


def save_value(parent, name, value):
    if isinstance(value, dict):
        group = parent.create_group(name)
        group.attrs["kind"] = "dict"
        group.attrs["keys"] = np.array(list(value.keys()), dtype=h5py.string_dtype("utf-8"))
        for key, item in value.items():
            save_value(group, key, item)
        return

    if isinstance(value, list):
        group = parent.create_group(name)
        group.attrs["kind"] = "list"
        group.attrs["length"] = len(value)
        for index, item in enumerate(value):
            save_value(group, f"{index:06d}", item)
        return

    if value is None:
        group = parent.create_group(name)
        group.attrs["kind"] = "none"
        return

    if isinstance(value, np.ndarray):
        dataset = parent.create_dataset(name, data=value)
        dataset.attrs["kind"] = "ndarray"
        return

    if isinstance(value, str):
        dataset = parent.create_dataset(name, data=value, dtype=h5py.string_dtype("utf-8"))
        dataset.attrs["kind"] = "str"
        return

    if isinstance(value, (bool, np.bool_)):
        dataset = parent.create_dataset(name, data=bool(value))
        dataset.attrs["kind"] = "bool"
        return

    if isinstance(value, (int, np.integer)):
        dataset = parent.create_dataset(name, data=int(value))
        dataset.attrs["kind"] = "int"
        return

    if isinstance(value, (float, np.floating)):
        dataset = parent.create_dataset(name, data=float(value))
        dataset.attrs["kind"] = "float"
        return

    raise TypeError(f"unsupported HDF5 value for {name}: {type(value)!r}")


def load_value(node):
    kind = node.attrs["kind"]
    if isinstance(kind, bytes):
        kind = kind.decode("utf-8")

    if kind == "dict":
        keys = [key.decode("utf-8") if isinstance(key, bytes) else str(key) for key in node.attrs["keys"]]
        return {key: load_value(node[key]) for key in keys}

    if kind == "list":
        return [load_value(node[f"{index:06d}"]) for index in range(int(node.attrs["length"]))]

    if kind == "none":
        return None

    if kind == "ndarray":
        return np.array(node[()])

    raw = node[()]
    if kind == "str":
        return raw.decode("utf-8") if isinstance(raw, bytes) else str(raw)
    if kind == "bool":
        return bool(raw)
    if kind == "int":
        return int(raw)
    if kind == "float":
        return float(raw)

    raise TypeError(f"unsupported HDF5 kind: {kind!r}")


def save_dict_to_hdf5(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(path, "w") as h5:
        h5.attrs["format"] = "pychrono-exudyn-port-dict"
        save_value(h5, "root", data)


def load_dict_from_hdf5(path):
    with h5py.File(path, "r") as h5:
        return load_value(h5["root"])


def compare_dicts(left, right):
    if isinstance(left, dict) and isinstance(right, dict):
        if left.keys() != right.keys():
            return False
        return all(compare_dicts(left[key], right[key]) for key in left)

    if isinstance(left, list) and isinstance(right, list):
        if len(left) != len(right):
            return False
        return all(compare_dicts(a, b) for a, b in zip(left, right))

    if isinstance(left, np.ndarray) and isinstance(right, np.ndarray):
        return np.array_equal(left, right)

    return left == right


def summarize(value):
    if isinstance(value, dict):
        child = [summarize(item) for item in value.values()]
        return {
            "nodes": 1 + sum(item["nodes"] for item in child),
            "dicts": 1 + sum(item["dicts"] for item in child),
            "lists": sum(item["lists"] for item in child),
            "arrays": sum(item["arrays"] for item in child),
            "scalars": sum(item["scalars"] for item in child),
        }
    if isinstance(value, list):
        child = [summarize(item) for item in value]
        return {
            "nodes": 1 + sum(item["nodes"] for item in child),
            "dicts": sum(item["dicts"] for item in child),
            "lists": 1 + sum(item["lists"] for item in child),
            "arrays": sum(item["arrays"] for item in child),
            "scalars": sum(item["scalars"] for item in child),
        }
    if isinstance(value, np.ndarray):
        return {"nodes": 1, "dicts": 0, "lists": 0, "arrays": 1, "scalars": 0}
    return {"nodes": 1, "dicts": 0, "lists": 0, "arrays": 0, "scalars": 1}


def run_hdf5_roundtrip(path=HDF5_PATH):
    original = source_test_dict()
    save_dict_to_hdf5(path, original)
    loaded = load_dict_from_hdf5(path)
    success = compare_dicts(original, loaded)
    summary = summarize(original)
    file_size = path.stat().st_size if path.exists() else 0
    summary.update({"success": success, "file": str(path), "file_size": file_size})
    return summary


def add_box(system, name, size, pos, tint, opacity=1.0):
    body = chrono.ChBodyEasyBox(size.x, size.y, size.z, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.SetPos(pos)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(tint)
    body.GetVisualShape(0).SetOpacity(opacity)
    system.AddBody(body)
    return body


def add_sphere(system, name, radius, pos, tint):
    body = chrono.ChBodyEasySphere(radius, 1000, True, False)
    body.SetName(name)
    body.SetFixed(True)
    body.SetPos(pos)
    body.EnableCollision(False)
    body.GetVisualShape(0).SetColor(tint)
    system.AddBody(body)
    return body


def add_line(system, name, points, tint, thickness=3):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(True)
    body.EnableCollision(False)
    line = chrono.ChLinePoly(len(points))
    for index, point in enumerate(points):
        line.SetPoint(index, point)
    shape = chrono.ChVisualShapeLine()
    shape.SetLineGeometry(line)
    shape.SetThickness(thickness)
    shape.SetColor(tint)
    body.AddVisualShape(shape)
    system.AddBody(body)
    return body


def build_system():
    summary = run_hdf5_roundtrip()
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    add_box(
        system,
        "HDF5 round-trip summary base",
        chrono.ChVector3d(4.6, 2.3, 0.035),
        chrono.ChVector3d(0.0, 0.0, -0.04),
        color(0.76, 0.78, 0.75),
        0.38,
    )

    status_color = color(0.08, 0.68, 0.20) if summary["success"] else color(0.90, 0.08, 0.05)
    add_sphere(system, "HDF5 comparison status marker", 0.16, chrono.ChVector3d(-1.75, 0.82, 0.16), status_color)

    rows = [
        ("dict nodes", summary["dicts"], color(0.12, 0.42, 0.85), 0.55),
        ("list nodes", summary["lists"], color(0.10, 0.62, 0.26), 0.20),
        ("array datasets", summary["arrays"], color(0.95, 0.64, 0.08), -0.15),
        ("scalar datasets", summary["scalars"], color(0.76, 0.20, 0.76), -0.50),
    ]
    for label, count, tint, y in rows:
        length = 0.22 + 0.12 * count
        add_box(system, f"HDF5 {label} count bar", chrono.ChVector3d(length, 0.13, 0.13), chrono.ChVector3d(-1.15 + 0.5 * length, y, 0.09), tint)
        add_box(system, f"HDF5 {label} count tick", chrono.ChVector3d(0.04, 0.22, 0.18), chrono.ChVector3d(-1.28, y, 0.11), tint)

    # Render the top-level dictionary keys as a small hierarchy graph.
    root = chrono.ChVector3d(0.75, 0.82, 0.08)
    add_box(system, "HDF5 root group", chrono.ChVector3d(0.40, 0.22, 0.16), root, color(0.04, 0.04, 0.045))
    keys = list(source_test_dict().keys())
    for index, key in enumerate(keys):
        x = 0.15 + 0.34 * (index % 4)
        y = 0.24 - 0.36 * (index // 4)
        node = chrono.ChVector3d(x, y, 0.09)
        tint = [color(0.12, 0.42, 0.85), color(0.10, 0.62, 0.26), color(0.95, 0.64, 0.08), color(0.76, 0.20, 0.76)][index % 4]
        add_box(system, f"HDF5 top-level key {key}", chrono.ChVector3d(0.20, 0.14, 0.13), node, tint)
        add_line(system, f"HDF5 hierarchy edge root to {key}", [root, node], color(0.05, 0.05, 0.055), 2)

    # Visual analogues for the two EXUDYN graphics objects embedded in key h.
    for ix in range(2):
        for iy in range(2):
            tint = color(0.18, 0.18, 0.19) if (ix + iy) % 2 else color(0.82, 0.82, 0.78)
            add_box(
                system,
                f"HDF5 checkerboard tile {ix} {iy}",
                chrono.ChVector3d(0.16, 0.16, 0.025),
                chrono.ChVector3d(1.65 + 0.16 * ix, -0.56 + 0.16 * iy, 0.04),
                tint,
            )
    add_sphere(system, "HDF5 stored graphics sphere", 0.13, chrono.ChVector3d(2.08, -0.46, 0.14), color(0.40, 0.40, 0.85))

    system._hdf5_summary = summary
    return system, summary


def simulate(duration, step):
    system, summary = build_system()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
    return system, summary


def run_visual(duration, step):
    import pychrono.irrlicht as chronoirr

    system, summary = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("EXUDYN port: testHDF5loadSave.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.85, -3.8, 2.65), chrono.ChVector3d(0.35, 0.02, 0.0))
    vis.AddTypicalLights()

    next_log = 0.0
    while vis.Run() and system.GetChTime() < duration:
        time = system.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        if time >= next_log:
            print_state(system, summary)
            next_log += 0.25


def print_state(system, summary):
    print(
        f"t={system.GetChTime():6.3f}  success={summary['success']}  "
        f"nodes={summary['nodes']}  dicts={summary['dicts']}  lists={summary['lists']}  "
        f"arrays={summary['arrays']}  scalars={summary['scalars']}  bytes={summary['file_size']}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: testHDF5loadSave.py -> PyChrono HDF5 dictionary round-trip")
    if args.no_vis:
        system, summary = simulate(args.duration, args.step)
        print_state(system, summary)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

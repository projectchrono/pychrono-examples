import argparse
import math

import numpy as np
import pychrono.core as chrono


# Reproduces EXUDYN TestModels/symbolicModuleTest.py:
# scalar, vector, and matrix symbolic operations are evaluated against the
# corresponding Python/math/numpy operations.  PyChrono does not provide
# EXUDYN's symbolic module, so this port implements the small Real/Vector/Matrix
# surface used by the source test and visualizes the verification summary as a
# static Chrono scene.

SOURCE_RESULT = 0.9480053738744615
STEP = 1.0e-3
END_TIME = 0.25


def color(r, g, b):
    return chrono.ChColor(r, g, b)


def scalar(value):
    if isinstance(value, Real):
        return value.value
    return float(value)


def array(value):
    if isinstance(value, (Vector, Matrix)):
        return value.Evaluate()
    return np.array(value, dtype=float)


class Real:
    new_count = 0
    delete_count = 0

    def __init__(self, *args):
        Real.new_count += 1
        if len(args) == 2:
            self.name = str(args[0])
            self.value = float(args[1])
        elif len(args) == 1:
            self.name = ""
            self.value = float(args[0])
        else:
            raise TypeError("Real expects value or name,value")

    def Evaluate(self):
        return float(self.value)

    def __float__(self):
        return float(self.value)

    def __bool__(self):
        return bool(self.value)

    def _real(self, value):
        return Real(value)

    def __add__(self, other):
        return self._real(self.value + scalar(other))

    def __radd__(self, other):
        return self._real(scalar(other) + self.value)

    def __sub__(self, other):
        return self._real(self.value - scalar(other))

    def __rsub__(self, other):
        return self._real(scalar(other) - self.value)

    def __mul__(self, other):
        if isinstance(other, Vector):
            return other * self
        if isinstance(other, Matrix):
            return other * self
        return self._real(self.value * scalar(other))

    def __rmul__(self, other):
        return self.__mul__(other)

    def __truediv__(self, other):
        return self._real(self.value / scalar(other))

    def __rtruediv__(self, other):
        return self._real(scalar(other) / self.value)

    def __pow__(self, other):
        return self._real(self.value ** scalar(other))

    def __rpow__(self, other):
        return self._real(scalar(other) ** self.value)

    def __neg__(self):
        return self._real(-self.value)

    def __pos__(self):
        return self._real(+self.value)

    def __iadd__(self, other):
        self.value += scalar(other)
        return self

    def __isub__(self, other):
        self.value -= scalar(other)
        return self

    def __imul__(self, other):
        self.value *= scalar(other)
        return self

    def __itruediv__(self, other):
        self.value /= scalar(other)
        return self

    def __lt__(self, other):
        return self.value < scalar(other)

    def __le__(self, other):
        return self.value <= scalar(other)

    def __gt__(self, other):
        return self.value > scalar(other)

    def __ge__(self, other):
        return self.value >= scalar(other)

    def __eq__(self, other):
        try:
            return self.value == scalar(other)
        except (TypeError, ValueError):
            return False

    def __ne__(self, other):
        return not self.__eq__(other)


class Vector:
    new_count = 0
    delete_count = 0

    def __init__(self, *args):
        Vector.new_count += 1
        if len(args) == 2:
            self.name = str(args[0])
            values = args[1]
        elif len(args) == 1:
            self.name = ""
            values = args[0]
        else:
            raise TypeError("Vector expects values or name,values")
        self.data = np.array(values, dtype=float)

    def Evaluate(self):
        return self.data.copy()

    def SetVector(self, values):
        self.data = np.array(values, dtype=float)

    def NumberOfItems(self):
        return int(self.data.size)

    def NormL2(self):
        return Real(float(np.linalg.norm(self.data)))

    def MultComponents(self, other):
        return Vector(self.data * array(other))

    def __getitem__(self, index):
        return Real(self.data[index])

    def __setitem__(self, index, value):
        self.data[index] = scalar(value)

    def __add__(self, other):
        return Vector(self.data + array(other))

    def __radd__(self, other):
        return Vector(array(other) + self.data)

    def __sub__(self, other):
        return Vector(self.data - array(other))

    def __rsub__(self, other):
        return Vector(array(other) - self.data)

    def __mul__(self, other):
        if isinstance(other, Vector):
            return Real(float(self.data @ other.data))
        if isinstance(other, Matrix):
            return Vector(self.data @ other.data)
        return Vector(self.data * scalar(other))

    def __rmul__(self, other):
        if isinstance(other, Matrix):
            return Vector(other.data @ self.data)
        return Vector(scalar(other) * self.data)

    def __neg__(self):
        return Vector(-self.data)

    def __iadd__(self, other):
        self.data = self.data + array(other)
        return self

    def __isub__(self, other):
        self.data = self.data - array(other)
        return self

    def __imul__(self, other):
        self.data = self.data * scalar(other)
        return self

    def __eq__(self, other):
        return np.array_equal(self.data, array(other))

    def __ne__(self, other):
        return not self.__eq__(other)


class Matrix:
    new_count = 0
    delete_count = 0

    def __init__(self, values):
        Matrix.new_count += 1
        self.data = np.array(values, dtype=float)

    def Evaluate(self):
        return self.data.copy()

    def SetMatrix(self, values):
        self.data = np.array(values, dtype=float)

    def Get(self, row, col):
        return Real(self.data[row, col])

    def NumberOfRows(self):
        return int(self.data.shape[0])

    def NumberOfColumns(self):
        return int(self.data.shape[1])

    def __getitem__(self, index):
        row, col = index
        return Real(self.data[row, col])

    def __setitem__(self, index, value):
        row, col = index
        self.data[row, col] = scalar(value)

    def __add__(self, other):
        return Matrix(self.data + array(other))

    def __radd__(self, other):
        return Matrix(array(other) + self.data)

    def __sub__(self, other):
        return Matrix(self.data - array(other))

    def __rsub__(self, other):
        return Matrix(array(other) - self.data)

    def __mul__(self, other):
        if isinstance(other, Matrix):
            return Matrix(self.data @ other.data)
        if isinstance(other, Vector):
            return Vector(self.data @ other.data)
        return Matrix(self.data * scalar(other))

    def __rmul__(self, other):
        if isinstance(other, Vector):
            return Vector(other.data @ self.data)
        return Matrix(scalar(other) * self.data)

    def __iadd__(self, other):
        self.data = self.data + array(other)
        return self

    def __isub__(self, other):
        self.data = self.data - array(other)
        return self

    def __imul__(self, other):
        self.data = self.data * scalar(other)
        return self


class SymbolicLib:
    Real = Real
    Vector = Vector
    Matrix = Matrix
    recording = False

    @staticmethod
    def SetRecording(recording):
        SymbolicLib.recording = bool(recording)

    @staticmethod
    def IfThenElse(condition, if_true, if_false):
        return if_true if bool(condition) else if_false


def symbolic_function(name):
    if name == "abs":
        return lambda x: Real(abs(scalar(x)))
    if name == "sign":
        return lambda x: Real(np.sign(scalar(x)))
    if name == "Not":
        return lambda x: not bool(x)
    if name == "round":
        return lambda x: Real(np.round(scalar(x)))
    if name == "ceil":
        return lambda x: Real(np.ceil(scalar(x)))
    if name == "floor":
        return lambda x: Real(np.floor(scalar(x)))
    if name == "mod":
        return lambda a, b: Real(math.fmod(scalar(a), scalar(b)))
    if name == "min":
        return lambda a, b: a if a < b else b
    if name == "max":
        return lambda a, b: a if a > b else b
    if name == "pow":
        return lambda a, b: a**b
    if name == "atan2":
        return lambda a, b: Real(math.atan2(scalar(a), scalar(b)))
    func = getattr(math, name)
    return lambda x: Real(func(scalar(x)))


for _name in (
    "abs",
    "sign",
    "Not",
    "round",
    "ceil",
    "floor",
    "exp",
    "sqrt",
    "log",
    "sin",
    "cos",
    "tan",
    "asin",
    "acos",
    "atan",
    "sinh",
    "cosh",
    "tanh",
    "asinh",
    "acosh",
    "atanh",
    "pow",
    "atan2",
    "mod",
    "min",
    "max",
):
    setattr(SymbolicLib, _name, staticmethod(symbolic_function(_name)))


def py_not(value):
    return not value


def py_real_name(name, value):
    return value


def py_named_vector(name, value):
    return np.array(value, dtype=float)


def py_min(a, b):
    return a if a < b else b


def py_max(a, b):
    return a if a > b else b


def evaluate(value):
    if isinstance(value, (Real, Vector, Matrix)):
        return value.Evaluate()
    if isinstance(value, np.ndarray):
        return value.copy()
    return value


def scalar_test_functions():
    math.sign = np.sign
    math.Not = py_not
    math.abs = np.abs
    math.mod = math.fmod
    math.min = py_min
    math.max = py_max
    math.round = np.round
    math.ceil = np.ceil
    math.floor = np.floor
    math.acosh = np.arccosh
    return [
        "abs",
        "sign",
        "Not",
        "round",
        "ceil",
        "floor",
        "exp",
        "sqrt",
        "log",
        "sin",
        "cos",
        "tan",
        "asin",
        "acos",
        "atan",
        "sinh",
        "cosh",
        "tanh",
        "asinh",
        "acosh",
        "atanh",
    ], ["pow", "atan2", "mod", "min", "max"]


def run_scalar_tests():
    list_functions, list_bin_functions = scalar_test_functions()
    list_numbers = [-3, 0, 2, -0.13157932734543, 0.0, 0.2345473645342, math.pi]
    cnt_wrong = 0
    cnt_tests = 0
    sum_results = 0.0

    for recording in [True, False]:
        SymbolicLib.SetRecording(recording)
        for number in list_numbers:
            result = [[], []]
            for case in [0, 1]:
                if case == 0:
                    real = Real
                    named_real = Real
                    lib = SymbolicLib
                else:
                    real = float
                    named_real = py_real_name
                    lib = math

                a = real(number)
                b = named_real("b", math.pi)
                c = named_real("c", -0.1)
                result[case] += [evaluate(a), evaluate(b), evaluate(c)]

                d = a + lib.sin(a / b) ** 2 + lib.tan(a) * lib.cos(a) * c * 3
                result[case] += [
                    evaluate(d),
                    evaluate(a + b),
                    evaluate(b + a),
                    evaluate(b - a),
                    evaluate(a - b),
                    evaluate(b * a),
                    evaluate(a * b),
                    evaluate(a / b),
                    evaluate(a / 3),
                    evaluate(-a),
                    evaluate(+a),
                    evaluate(b**a),
                ]

                d = real(0)
                result[case] += [evaluate(d)]
                d += a
                result[case] += [evaluate(d)]
                d -= a * b
                result[case] += [evaluate(d)]
                d *= a
                result[case] += [evaluate(d)]
                if float(a) != 0.0:
                    d /= a
                    result[case] += [evaluate(d)]

                for comparison in (a < b, a <= b, a > b, a >= b, a == b, a != b):
                    result[case] += [evaluate(comparison)]

                for func_str in list_functions:
                    if func_str == "log" and number <= 0:
                        continue
                    if func_str == "sqrt" and number < 0:
                        continue
                    if func_str == "acosh" and number < 1:
                        continue
                    if func_str == "atanh" and abs(number) >= 1:
                        continue
                    if func_str in ("acos", "asin") and abs(number) > 1:
                        continue
                    result[case] += [evaluate(getattr(lib, func_str)(a))]

                for func_str in list_bin_functions:
                    result[case] += [evaluate(getattr(lib, func_str)(a, 2))]

                if case == 0:
                    f = lib.IfThenElse(a, a + 1, b + a)
                else:
                    f = a + 1 if a else b + a
                result[case] += [evaluate(f)]

            for res in np.array(result).T.tolist():
                cnt_tests += 1
                sum_results += res[0]
                if res[0] != res[1]:
                    cnt_wrong += 1

    return cnt_tests, cnt_wrong, sum_results


def run_vector_matrix_tests():
    cnt_wrong = 0
    cnt_tests = 0
    sum_results = 0.0

    for recording in [False, True]:
        SymbolicLib.SetRecording(recording)
        result = [[], []]
        for case in [1, 0]:
            if case == 0:
                real = Real
                named_real = Real
                vector = Vector
                named_vector = Vector
                matrix = Matrix
            else:
                real = float
                named_real = py_real_name
                vector = np.array
                named_vector = py_named_vector
                matrix = np.array

            a = named_real("a", 0.5)
            b = named_real("b", math.pi)
            _ = b

            v = vector([1.3])
            w = vector([4.2])
            result[case] += [evaluate(v + w)]

            v = named_vector("vv", [1, 2])
            w = vector([4.2 - 1, -1.2 - 1])
            x = vector([0.0, 0.0])
            if case == 0:
                w.SetVector([4.2, -1.2])
                d = 3.3 * v + a * w + (v * w) * v
                x[0] += d.NormL2()
                x[1] += evaluate(v == v)
                x[1] += evaluate(v == Vector([1, 2.1]))
                x[1] += evaluate(v != Vector([1, 2.1]))
                x[1] += evaluate(v == Vector([1, 2.0]))
                x[1] += evaluate(v != Vector([1, 2.0]))
                dot = v * w
                n = w.NumberOfItems()
            else:
                w = np.array([4.2, -1.2])
                d = 3.3 * v + a * w + (v @ w) * v
                x[0] += np.linalg.norm(d)
                x[1] += (v == v).all()
                x[1] += (v == vector([1, 2.1])).all()
                x[1] += (v != vector([1, 2.1])).any()
                x[1] += (v == vector([1, 2.0])).all()
                x[1] += (v != vector([1, 2.0])).any()
                dot = v @ w
                n = len(w)

            result[case] += [
                evaluate(x),
                evaluate(d),
                evaluate(v + w),
                evaluate(v - w),
                evaluate(dot),
                evaluate(vector([n, 1.1])),
                evaluate(v),
                evaluate(-v),
                evaluate(a * v),
                evaluate(v * a),
                evaluate(3.3 * v),
                evaluate(v * 3.3),
            ]

            v = vector([-0.33, 0.347, 1.5])
            w = vector([4.2, -1.2 + 10, 7.7])
            w[1] = -1.2
            result[case] += [evaluate(w), evaluate(d), evaluate(v + w), evaluate(v - w)]
            if case == 0:
                result[case] += [evaluate(v * w), evaluate(v.MultComponents(w))]
            else:
                result[case] += [evaluate(v @ w), evaluate(v * w)]

            result[case] += [
                evaluate(v),
                evaluate(-v),
                evaluate(a * v),
                evaluate(v * a),
                evaluate(3.3 * v),
                evaluate(v * 3.3),
            ]

            u = vector([0.0, 0.0, 0.0])
            u += 2 * v
            u = u + 2 * v
            result[case] += [evaluate(u)]
            u -= v
            result[case] += [evaluate(u), evaluate(u == v), evaluate(u != v)]
            u *= a
            result[case] += [evaluate(u)]
            u *= 1 / 3
            result[case] += [evaluate(u), evaluate(u == v), evaluate(u != v)]

            m = matrix(np.array([[2.0, 0.1, 0.33], [-0.1, 2.3, 0.7], [0, 0.34, 1.8]]))
            n_matrix = matrix(np.array([[1.0, 0.3, -0.33], [-0.9, 1.3, -0.7], [0, 0.64, -1.8]]))
            result[case] += [evaluate(n_matrix)]

            if case == 0:
                m02 = m[0, 2].Evaluate()
                m20 = m[2, 0].Evaluate()
                if m02 != m.Get(0, 2).Evaluate():
                    m02 = -1000
                nr = m.NumberOfRows()
                nc = m.NumberOfColumns()
                n_matrix.SetMatrix(np.array([[(nr + nc) / 6, 0.3, -m02], [-0.9, 1.3, -0.7], [m20, 100.64, -1.8]]))
                n_matrix[2, 1] = 0.64

            result[case] += [
                evaluate(n_matrix),
                evaluate(m),
                evaluate(a * m),
                evaluate(0.5 * m),
                evaluate(m * a),
                evaluate(m * 0.5),
                evaluate(m + n_matrix),
                evaluate(m - n_matrix),
            ]
            if case == 0:
                result[case] += [evaluate(m * n_matrix), evaluate(m * v), evaluate(v * m)]
            else:
                result[case] += [evaluate(m @ n_matrix), evaluate(m @ v), evaluate(v @ m)]

            p = 0.0 * m
            result[case] += [evaluate(p)]
            p += n_matrix
            result[case] += [evaluate(p)]
            p -= m
            result[case] += [evaluate(p)]
            p *= 1.3
            result[case] += [evaluate(p)]

        cnt_tests += len(result[0])
        for i in range(len(result[0])):
            res0 = np.array(result[0][i], dtype=float)
            res1 = np.array(result[1][i], dtype=float)
            sum_results += float(np.linalg.norm(res0))
            if float(np.linalg.norm(res0 - res1)) > 1.0e-15:
                cnt_wrong += 1

    return cnt_tests, cnt_wrong, sum_results


def run_symbolic_tests():
    Real.new_count = Real.delete_count = 0
    Vector.new_count = Vector.delete_count = 0
    Matrix.new_count = Matrix.delete_count = 0

    scalar_count, scalar_wrong, scalar_sum = run_scalar_tests()
    vector_count, vector_wrong, vector_sum = run_vector_matrix_tests()
    new_delete_balance = 0
    total_sum = scalar_sum + vector_sum + new_delete_balance
    result = total_sum / 1000.0
    return {
        "scalar_count": scalar_count,
        "vector_matrix_count": vector_count,
        "count": scalar_count + vector_count,
        "wrong": scalar_wrong + vector_wrong,
        "scalar_wrong": scalar_wrong,
        "vector_matrix_wrong": vector_wrong,
        "sum": total_sum,
        "result": result,
        "source": SOURCE_RESULT,
        "delta": result - SOURCE_RESULT,
        "new_delete_balance": new_delete_balance,
        "real_new": Real.new_count,
        "real_delete": Real.delete_count,
        "vector_new": Vector.new_count,
        "vector_delete": Vector.delete_count,
        "matrix_new": Matrix.new_count,
        "matrix_delete": Matrix.delete_count,
    }


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


def build_system():
    summary = run_symbolic_tests()
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    add_box(system, "symbolic test summary base", chrono.ChVector3d(4.8, 1.7, 0.04), chrono.ChVector3d(0, 0, -0.03), color(0.76, 0.78, 0.75), 0.42)

    rows = [
        ("scalar operation checks", summary["scalar_count"], summary["scalar_wrong"], 0.45, color(0.12, 0.42, 0.85)),
        ("vector matrix operation checks", summary["vector_matrix_count"], summary["vector_matrix_wrong"], 0.00, color(0.12, 0.62, 0.26)),
        ("source result agreement", max(1, int(round((1.0 - min(abs(summary["delta"]) * 1000.0, 1.0)) * 100))), summary["wrong"], -0.45, color(0.95, 0.65, 0.08)),
    ]

    for label, count, wrong, y, tint in rows:
        length = min(3.4, 0.35 + count / 95.0)
        add_box(system, f"{label} count bar", chrono.ChVector3d(length, 0.16, 0.16), chrono.ChVector3d(-0.35 + 0.5 * length, y, 0.08), tint)
        status = color(0.08, 0.68, 0.20) if wrong == 0 else color(0.88, 0.08, 0.05)
        add_sphere(system, f"{label} pass marker", 0.105, chrono.ChVector3d(-0.85, y, 0.12), status)
        add_box(system, f"{label} reference tick", chrono.ChVector3d(0.045, 0.28, 0.24), chrono.ChVector3d(3.05, y, 0.12), color(0.04, 0.04, 0.045))

    # Small matrix of 3x3 cubes mirrors the source matrix section and provides
    # an immediately visible non-mechanical Chrono artifact for this test-only
    # example.
    matrix_values = np.array([[2.0, 0.1, 0.33], [-0.1, 2.3, 0.7], [0.0, 0.34, 1.8]])
    for i in range(3):
        for j in range(3):
            height = 0.05 + 0.08 * abs(matrix_values[i, j])
            add_box(
                system,
                f"matrix sample cube {i} {j}",
                chrono.ChVector3d(0.18, 0.18, height),
                chrono.ChVector3d(-1.55 + 0.24 * j, -0.70 - 0.24 * i, 0.5 * height),
                color(0.58, 0.25 + 0.12 * i, 0.65 - 0.10 * j),
            )

    system._symbolic_summary = summary
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
    vis.SetWindowTitle("EXUDYN port: symbolicModuleTest.py")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.35, -3.4, 2.55), chrono.ChVector3d(0.6, -0.05, 0.05))
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
        f"t={system.GetChTime():6.3f}  tests={summary['count']}  wrong={summary['wrong']}  "
        f"u={summary['result']:.15f}  source={summary['source']:.15f}  "
        f"delta={summary['delta']:+.3e}  new_delete_balance={summary['new_delete_balance']}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: symbolicModuleTest.py -> PyChrono symbolic operation test scene")
    if args.no_vis:
        system, summary = simulate(args.duration, args.step)
        print_state(system, summary)
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

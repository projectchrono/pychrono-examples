import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))
from fem_import_ffrf_test_common import build_system as _build_system, run_main, update_visuals


def build_system():
    return _build_system("compare_abaqus_ansys_rotor_eigenfrequencies")


def main():
    run_main("compare_abaqus_ansys_rotor_eigenfrequencies")


if __name__ == "__main__":
    main()

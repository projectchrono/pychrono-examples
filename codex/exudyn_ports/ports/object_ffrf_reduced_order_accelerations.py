import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))
from fem_import_ffrf_test_common import build_system as _build_system, run_main, update_visuals


def build_system():
    return _build_system("object_ffrf_reduced_order_accelerations")


def main():
    run_main("object_ffrf_reduced_order_accelerations")


if __name__ == "__main__":
    main()

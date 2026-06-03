import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))
from modal_fem_replay_common import build_system as _build_system, run_main, update_visuals


def build_system():
    return _build_system("object_ffrf_convergence_test_hinge")


def main():
    run_main("object_ffrf_convergence_test_hinge")


if __name__ == "__main__":
    main()

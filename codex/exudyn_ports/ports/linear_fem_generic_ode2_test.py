import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))
from modal_fem_replay_common import build_system as _build_system, run_main, update_visuals


def build_system():
    return _build_system("linear_fem_generic_ode2_test")


def main():
    run_main("linear_fem_generic_ode2_test")


if __name__ == "__main__":
    main()

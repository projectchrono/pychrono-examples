import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))
from test_suite_replay_common import build_system as _build_system, run_main, update_visuals


def build_system():
    return _build_system("perf_3d_rigid_bodies")


def main():
    run_main("perf_3d_rigid_bodies")


if __name__ == "__main__":
    main()

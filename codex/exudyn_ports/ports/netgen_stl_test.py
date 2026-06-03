import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))
from netgen_replay_common import build_system as _build_system, run_main, update_visuals


def build_system():
    return _build_system("netgen_stl_test")


def main():
    run_main("netgen_stl_test")


if __name__ == "__main__":
    main()

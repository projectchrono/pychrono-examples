import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))
from ngsolve_replay_common import build_system as _build_system, run_main, update_visuals


def build_system():
    return _build_system("ngsolve_craig_bampton")


def main():
    run_main("ngsolve_craig_bampton")


if __name__ == "__main__":
    main()

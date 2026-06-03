import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))
from ancf_sliding_ale_common import build_system as _build_system, run_main, update_visuals


def build_system():
    return _build_system("ANCFslidingAndALEjointTest.py")


def main():
    run_main("ANCFslidingAndALEjointTest.py")


if __name__ == "__main__":
    main()

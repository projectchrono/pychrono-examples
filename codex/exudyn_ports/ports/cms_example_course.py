import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))
from modal_fem_replay_common import build_system as _build_system, run_main, update_visuals


def build_system():
    return _build_system("cms_example_course")


def main():
    run_main("cms_example_course")


if __name__ == "__main__":
    main()

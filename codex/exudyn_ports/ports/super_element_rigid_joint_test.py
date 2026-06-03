import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))
from fem_import_ffrf_test_common import build_system as _build_system, run_main, update_visuals


def build_system():
    return _build_system("super_element_rigid_joint_test")


def main():
    run_main("super_element_rigid_joint_test")


if __name__ == "__main__":
    main()

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))
from slider_crank_belt_cms_common import build_system as _build_system, run_main, update_visuals


def build_system():
    return _build_system("slider_crank_3d_with_ancf_belt_drive")


def main():
    run_main("slider_crank_3d_with_ancf_belt_drive")


if __name__ == "__main__":
    main()

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from mini_ancf_cable_common import build_cable_system, run_main, update_visuals


LABEL = "mini_object_ancf_cable"
SOURCE = "ObjectANCFCable.py"


def build_system():
    return build_cable_system(SOURCE)


def main():
    run_main(LABEL, SOURCE, build_system)


if __name__ == "__main__":
    main()

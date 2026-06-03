import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))
from spatial_slider_crank_common import build_system as _build_system
from spatial_slider_crank_common import run_main


def build_system():
    return _build_system("test")


if __name__ == "__main__":
    run_main("test")

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))
from beam3d_replay_common import EigenConfig, build_eigen_system as _build_system, main_eigen, update_eigen_visuals as update_visuals


CONFIG = EigenConfig("ANCFBeam3DeigTest.py", "NodePoint3DSlope23")


def build_system():
    return _build_system(CONFIG)


if __name__ == "__main__":
    main_eigen(CONFIG)

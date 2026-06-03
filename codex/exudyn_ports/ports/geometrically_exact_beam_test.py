import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))
from beam3d_replay_common import STATIC_CASES_8, StaticConfig, build_static_system as _build_system, main_static, update_static_visuals as update_visuals


CONFIG = StaticConfig("geometricallyExactBeamTest.py", "GeometricallyExactBeam", STATIC_CASES_8, reference_sum=1.012822053539261)


def build_system():
    return _build_system(CONFIG)


if __name__ == "__main__":
    main_static(CONFIG)

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))
from beam3d_replay_common import TwistConfig, build_twist_system as _build_system, main_twist, update_twist_visuals as update_visuals


CONFIG = TwistConfig()


def build_system():
    return _build_system(CONFIG)


if __name__ == "__main__":
    main_twist(CONFIG)

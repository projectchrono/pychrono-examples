import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from gym_replay_common import build_double_pendulum_system, main_double_pendulum, update_double_pendulum_visuals


def build_system():
    return build_double_pendulum_system("testGymDoublePendulumEnv.py", interface=False, driver=False)


update_visuals = update_double_pendulum_visuals


def main():
    main_double_pendulum(
        build_system,
        "EXUDYN port: testGymDoublePendulumEnv.py",
        "EXUDYN port: testGymDoublePendulumEnv.py -> PyChrono double-pendulum Gym environment replay",
    )


if __name__ == "__main__":
    main()

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from gym_replay_common import build_cartpole_system, main_cartpole, update_cartpole_visuals


def build_system():
    return build_cartpole_system("testGymCartpoleEnv.py", slow_solver=False, driver=False)


update_visuals = update_cartpole_visuals


def main():
    main_cartpole(
        build_system,
        "EXUDYN port: testGymCartpoleEnv.py",
        "EXUDYN port: testGymCartpoleEnv.py -> PyChrono cart-pole Gym environment replay",
    )


if __name__ == "__main__":
    main()

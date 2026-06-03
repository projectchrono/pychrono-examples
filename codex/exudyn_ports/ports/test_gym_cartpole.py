import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from gym_replay_common import build_cartpole_system, main_cartpole, update_cartpole_visuals


def build_system():
    return build_cartpole_system("testGymCartpole.py", slow_solver=False, driver=True)


update_visuals = update_cartpole_visuals


def main():
    main_cartpole(
        build_system,
        "EXUDYN port: testGymCartpole.py",
        "EXUDYN port: testGymCartpole.py -> PyChrono cart-pole Gym driver replay",
    )


if __name__ == "__main__":
    main()

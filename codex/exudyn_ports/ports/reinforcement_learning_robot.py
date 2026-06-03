import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from robotics_replay_common import (
    DD_REPLAY_END,
    build_reinforcement_learning_robot_system,
    generic_main,
    print_reinforcement_learning_robot_state,
    update_reinforcement_learning_robot_visuals,
)


build_system = build_reinforcement_learning_robot_system
update_visuals = update_reinforcement_learning_robot_visuals


def main():
    generic_main(
        build_system,
        update_visuals,
        print_reinforcement_learning_robot_state,
        "EXUDYN port: reinforcementLearningRobot.py",
        DD_REPLAY_END,
        0.002,
        (2.8, -4.1, 2.8),
        (0.0, 0.0, 0.12),
        "EXUDYN port: reinforcementLearningRobot.py -> PyChrono differential-drive RL environment replay",
    )


if __name__ == "__main__":
    main()

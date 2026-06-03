import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from robotics_replay_common import (
    build_spot_reinforcement_learning_system,
    generic_main,
    print_spot_reinforcement_learning_state,
    update_spot_reinforcement_learning_visuals,
)


build_system = build_spot_reinforcement_learning_system
update_visuals = update_spot_reinforcement_learning_visuals


def main():
    generic_main(
        build_system,
        update_visuals,
        print_spot_reinforcement_learning_state,
        "EXUDYN port: spotReinforcementLearning.py",
        5.0,
        0.001,
        (3.2, -2.75, 1.75),
        (1.55, 0.0, 0.44),
        "EXUDYN port: FurtherExamples/spotReinforcementLearning.py -> PyChrono Spot RL environment replay",
    )


if __name__ == "__main__":
    main()

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from robotics_replay_common import (
    HUMAN_DURATION,
    build_human_robot_interaction_system,
    generic_main,
    print_human_robot_interaction_state,
    update_human_robot_interaction_visuals,
)


build_system = build_human_robot_interaction_system
update_visuals = update_human_robot_interaction_visuals


def main():
    generic_main(
        build_system,
        update_visuals,
        print_human_robot_interaction_state,
        "EXUDYN port: humanRobotInteraction.py",
        HUMAN_DURATION,
        0.001,
        (0.82, -2.05, 1.52),
        (-0.46, -0.08, 0.82),
        "EXUDYN port: humanRobotInteraction.py -> PyChrono human/PUMA contact replay",
    )


if __name__ == "__main__":
    main()

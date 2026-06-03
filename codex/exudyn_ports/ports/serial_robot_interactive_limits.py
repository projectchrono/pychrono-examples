import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from interactive_replay_common import (
    build_serial_robot_interactive_limits_system,
    main_serial_robot_interactive_limits,
    update_serial_robot_interactive_limits_visuals,
)


build_system = build_serial_robot_interactive_limits_system
update_visuals = update_serial_robot_interactive_limits_visuals


def main():
    main_serial_robot_interactive_limits(
        build_system,
        "EXUDYN port: serialRobotInteractiveLimits.py",
        "EXUDYN port: serialRobotInteractiveLimits.py -> PyChrono PUMA interactive-limit replay",
        duration=2.4,
    )


if __name__ == "__main__":
    main()

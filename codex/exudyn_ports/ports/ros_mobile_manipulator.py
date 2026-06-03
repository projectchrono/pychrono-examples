import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from ros_replay_common import (
    build_ros_mobile_manipulator_system,
    main_ros_mobile_manipulator,
    update_ros_mobile_manipulator_visuals,
)


build_system = build_ros_mobile_manipulator_system
update_visuals = update_ros_mobile_manipulator_visuals


def main():
    main_ros_mobile_manipulator(
        build_system,
        "EXUDYN port: ROSMobileManipulator.py",
        "EXUDYN port: ROSMobileManipulator.py -> PyChrono deterministic ROS KAIROS/UR5 replay",
        duration=3.0,
    )


if __name__ == "__main__":
    main()

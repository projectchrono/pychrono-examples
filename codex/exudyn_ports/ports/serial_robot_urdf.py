import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from robotics_replay_common import (
    SERIAL_URDF_END,
    build_serial_robot_urdf_system,
    generic_main,
    print_serial_robot_urdf_state,
    update_serial_robot_urdf_visuals,
)


build_system = build_serial_robot_urdf_system
update_visuals = update_serial_robot_urdf_visuals


def main():
    generic_main(
        build_system,
        update_visuals,
        print_serial_robot_urdf_state,
        "EXUDYN port: serialRobotURDF.py",
        SERIAL_URDF_END,
        0.001,
        (0.82, -1.15, 0.78),
        (-0.22, -0.02, 0.20),
        "EXUDYN port: serialRobotURDF.py -> PyChrono UR5 URDF trajectory replay",
    )


if __name__ == "__main__":
    main()

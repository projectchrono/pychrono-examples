import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from ros_replay_common import build_ros_mass_point_system, main_ros_mass_point, update_ros_mass_point_visuals


def build_system():
    return build_ros_mass_point_system("ROSMassPoint.py", modern_api=True)


update_visuals = update_ros_mass_point_visuals


def main():
    main_ros_mass_point(
        build_system,
        "EXUDYN port: ROSMassPoint.py",
        "EXUDYN port: ROSMassPoint.py -> PyChrono deterministic ROS mass-point replay",
        duration=2.0,
    )


if __name__ == "__main__":
    main()

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from ros_replay_common import build_ros_turtle_system, main_ros_turtle, update_ros_turtle_visuals


def build_system():
    return build_ros_turtle_system("ROSTurtle.py", stl_name="ROSTurtle.stl", modern_api=True)


update_visuals = update_ros_turtle_visuals


def main():
    main_ros_turtle(
        build_system,
        "EXUDYN port: ROSTurtle.py",
        "EXUDYN port: ROSTurtle.py -> PyChrono deterministic ROS turtle replay",
        duration=2.2,
    )


if __name__ == "__main__":
    main()

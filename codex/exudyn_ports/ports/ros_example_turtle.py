import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from ros_replay_common import build_ros_turtle_system, main_ros_turtle, update_ros_turtle_visuals


def build_system():
    return build_ros_turtle_system("ROSExampleTurtle.py", stl_name="Turtle.stl", modern_api=False)


update_visuals = update_ros_turtle_visuals


def main():
    main_ros_turtle(
        build_system,
        "EXUDYN port: ROSExampleTurtle.py",
        "EXUDYN port: ROSExampleTurtle.py -> PyChrono deterministic ROS turtle replay",
        duration=2.2,
    )


if __name__ == "__main__":
    main()

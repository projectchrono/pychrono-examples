import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from gym_replay_common import build_nlink_system, main_nlink, update_nlink_visuals


def build_system():
    return build_nlink_system("openAIgymTriplePendulum.py", n_links=3, continuous=False, source_kind="triple")


update_visuals = update_nlink_visuals


def main():
    main_nlink(
        build_system,
        "EXUDYN port: openAIgymTriplePendulum.py",
        "EXUDYN port: openAIgymTriplePendulum.py -> PyChrono triple-pendulum Gym replay",
        duration=2.0,
        camera=(3.0, -4.2, 3.0),
        target=(0.0, 0.0, 1.35),
    )


if __name__ == "__main__":
    main()

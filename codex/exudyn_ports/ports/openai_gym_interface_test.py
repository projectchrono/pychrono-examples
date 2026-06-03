import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from gym_replay_common import build_nlink_system, main_nlink, update_nlink_visuals


def build_system():
    return build_nlink_system("openAIgymInterfaceTest.py", n_links=2, continuous=False, source_kind="interface")


update_visuals = update_nlink_visuals


def main():
    main_nlink(
        build_system,
        "EXUDYN port: openAIgymInterfaceTest.py",
        "EXUDYN port: openAIgymInterfaceTest.py -> PyChrono OpenAIGymInterface double-pendulum replay",
        duration=2.0,
        camera=(2.25, -3.4, 2.25),
        target=(0.0, 0.0, 0.95),
    )


if __name__ == "__main__":
    main()

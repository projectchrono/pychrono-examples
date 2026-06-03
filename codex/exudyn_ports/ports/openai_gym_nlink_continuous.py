import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from gym_replay_common import build_nlink_system, main_nlink, update_nlink_visuals


def build_system():
    return build_nlink_system("openAIgymNLinkContinuous.py", n_links=2, continuous=True, source_kind="continuous")


update_visuals = update_nlink_visuals


def main():
    main_nlink(
        build_system,
        "EXUDYN port: openAIgymNLinkContinuous.py",
        "EXUDYN port: openAIgymNLinkContinuous.py -> PyChrono continuous-action n-link Gym replay",
        duration=2.0,
        camera=(2.65, -3.8, 2.35),
        target=(0.0, 0.0, 1.0),
    )


if __name__ == "__main__":
    main()

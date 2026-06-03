import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from interactive_replay_common import build_interactive_tutorial_system, main_interactive_tutorial, update_interactive_tutorial_visuals


build_system = build_interactive_tutorial_system
update_visuals = update_interactive_tutorial_visuals


def main():
    main_interactive_tutorial(
        build_system,
        "EXUDYN port: interactiveTutorial.py",
        "EXUDYN port: interactiveTutorial.py -> PyChrono interactive-mode construction replay",
        duration=1.0,
    )


if __name__ == "__main__":
    main()

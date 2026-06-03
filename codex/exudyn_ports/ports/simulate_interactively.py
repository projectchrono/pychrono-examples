import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from interactive_replay_common import build_simulate_interactively_system, main_simulate_interactively, update_simulate_interactively_visuals


build_system = build_simulate_interactively_system
update_visuals = update_simulate_interactively_visuals


def main():
    main_simulate_interactively(
        build_system,
        "EXUDYN port: simulateInteractively.py",
        "EXUDYN port: simulateInteractively.py -> PyChrono InteractiveDialog oscillator replay",
        duration=2.0,
    )


if __name__ == "__main__":
    main()

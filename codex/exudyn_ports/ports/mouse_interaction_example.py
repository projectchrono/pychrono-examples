import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from external_workflow_replay_common import build_mouse_interaction_system, main_mouse_interaction, update_mouse_interaction_visuals


build_system = build_mouse_interaction_system
update_visuals = update_mouse_interaction_visuals


def main():
    main_mouse_interaction(
        build_system,
        "EXUDYN port: mouseInteractionExample.py",
        "EXUDYN port: mouseInteractionExample.py -> PyChrono deterministic mouse-drag chain replay",
        duration=1.6,
    )


if __name__ == "__main__":
    main()

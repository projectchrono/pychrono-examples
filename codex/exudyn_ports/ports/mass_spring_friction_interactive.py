import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from interactive_replay_common import (
    build_mass_spring_friction_interactive_system,
    main_mass_spring_friction_interactive,
    update_mass_spring_friction_interactive_visuals,
)


build_system = build_mass_spring_friction_interactive_system
update_visuals = update_mass_spring_friction_interactive_visuals


def main():
    main_mass_spring_friction_interactive(
        build_system,
        "EXUDYN port: massSpringFrictionInteractive.py",
        "EXUDYN port: massSpringFrictionInteractive.py -> PyChrono friction InteractiveDialog replay",
        duration=2.0,
    )


if __name__ == "__main__":
    main()

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from interactive_replay_common import (
    build_n_mass_oscillator_interactive_system,
    main_n_mass_oscillator_interactive,
    update_n_mass_oscillator_interactive_visuals,
)


build_system = build_n_mass_oscillator_interactive_system
update_visuals = update_n_mass_oscillator_interactive_visuals


def main():
    main_n_mass_oscillator_interactive(
        build_system,
        "EXUDYN port: nMassOscillatorInteractive.py",
        "EXUDYN port: nMassOscillatorInteractive.py -> PyChrono interactive n-mass oscillator replay",
        duration=2.0,
    )


if __name__ == "__main__":
    main()

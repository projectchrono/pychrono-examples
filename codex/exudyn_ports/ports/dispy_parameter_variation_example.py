import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from external_workflow_replay_common import (
    build_dispy_parameter_variation_system,
    main_dispy_parameter_variation,
    update_dispy_parameter_variation_visuals,
)


build_system = build_dispy_parameter_variation_system
update_visuals = update_dispy_parameter_variation_visuals


def main():
    main_dispy_parameter_variation(
        build_system,
        "EXUDYN port: dispyParameterVariationExample.py",
        "EXUDYN port: dispyParameterVariationExample.py -> PyChrono deterministic dispy parameter-variation replay",
        duration=1.2,
    )


if __name__ == "__main__":
    main()

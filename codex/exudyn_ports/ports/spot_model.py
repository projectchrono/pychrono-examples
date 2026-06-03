import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from robotics_replay_common import (
    SPOT_MODEL_END,
    build_spot_model_system,
    generic_main,
    print_spot_model_state,
    update_spot_model_visuals,
)


build_system = build_spot_model_system
update_visuals = update_spot_model_visuals


def main():
    generic_main(
        build_system,
        update_visuals,
        print_spot_model_state,
        "EXUDYN port: spotModel.py",
        SPOT_MODEL_END,
        0.001,
        (1.35, -1.65, 1.30),
        (0.0, 0.0, 0.42),
        "EXUDYN port: FurtherExamples/spotModel.py -> PyChrono quadruped URDF/contact replay",
    )


if __name__ == "__main__":
    main()

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))
from ancf_ale_publication_common import build_system as _build_system, run_main, update_visuals


def build_system():
    return _build_system("fixed_fixed_ancf_ale_discrete_masses_postprocessing")


def main():
    run_main("fixed_fixed_ancf_ale_discrete_masses_postprocessing")


if __name__ == "__main__":
    main()

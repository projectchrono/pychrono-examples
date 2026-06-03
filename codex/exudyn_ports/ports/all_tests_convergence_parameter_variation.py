import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))
from ancf_ale_publication_common import build_system as _build_system, run_main, update_visuals


def build_system():
    return _build_system("all_tests_convergence_parameter_variation")


def main():
    run_main("all_tests_convergence_parameter_variation")


if __name__ == "__main__":
    main()

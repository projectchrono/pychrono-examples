import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))
from genetic_optimization_mass_spring_common import build_system as _build_system, run_main, update_visuals


def build_system():
    return _build_system("test")


def main():
    run_main("test")


if __name__ == "__main__":
    main()

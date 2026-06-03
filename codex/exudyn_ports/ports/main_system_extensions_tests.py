import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))
from test_suite_replay_common import build_system as _build_system, run_main, update_visuals


def build_system():
    return _build_system("main_system_extensions_tests")


def main():
    run_main("main_system_extensions_tests")


if __name__ == "__main__":
    main()

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from external_workflow_replay_common import build_multiprocessing_system, main_multiprocessing, update_multiprocessing_visuals


build_system = build_multiprocessing_system
update_visuals = update_multiprocessing_visuals


def main():
    main_multiprocessing(
        build_system,
        "EXUDYN port: multiprocessingTest.py",
        "EXUDYN port: multiprocessingTest.py -> PyChrono deterministic multiprocessing free-mass replay",
        duration=1.2,
    )


if __name__ == "__main__":
    main()

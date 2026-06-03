import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from external_workflow_replay_common import build_mpi4py_system, main_mpi4py, update_mpi4py_visuals


build_system = build_mpi4py_system
update_visuals = update_mpi4py_visuals


def main():
    main_mpi4py(
        build_system,
        "EXUDYN port: mpi4pyExample.py",
        "EXUDYN port: mpi4pyExample.py -> PyChrono deterministic MPI parameter-variation replay",
        duration=1.4,
    )


if __name__ == "__main__":
    main()

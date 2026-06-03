import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))
from ancf_ale_publication_common import build_system as _build_system, run_main, update_visuals


def build_system():
    return _build_system("finite_segment_method")


def main():
    run_main("finite_segment_method")


if __name__ == "__main__":
    main()

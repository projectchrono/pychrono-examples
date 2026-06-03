import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from external_workflow_replay_common import build_openvr_engine_system, main_openvr_engine, update_openvr_engine_visuals


build_system = build_openvr_engine_system
update_visuals = update_openvr_engine_visuals


def main():
    main_openvr_engine(
        build_system,
        "EXUDYN port: openVRengine.py",
        "EXUDYN port: openVRengine.py -> PyChrono deterministic OpenVR engine-room replay",
        duration=1.4,
    )


if __name__ == "__main__":
    main()

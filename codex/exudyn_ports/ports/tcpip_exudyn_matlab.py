import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from tcpip_replay_common import build_tcpip_matlab_system, main_tcpip_matlab, update_tcpip_matlab_visuals


build_system = build_tcpip_matlab_system
update_visuals = update_tcpip_matlab_visuals


def main():
    main_tcpip_matlab(
        build_system,
        "EXUDYN port: TCPIPexudynMatlab.py",
        "EXUDYN port: TCPIPexudynMatlab.py -> PyChrono deterministic MATLAB TCP/IP double-pendulum replay",
        duration=1.5,
    )


if __name__ == "__main__":
    main()

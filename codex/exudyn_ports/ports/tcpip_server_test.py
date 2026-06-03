import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))

from tcpip_replay_common import build_tcpip_python_system, main_tcpip_python, update_tcpip_python_visuals


def build_system():
    return build_tcpip_python_system("TCPIPserverTest.py", "server")


update_visuals = update_tcpip_python_visuals


def main():
    main_tcpip_python(
        build_system,
        "EXUDYN port: TCPIPserverTest.py",
        "EXUDYN port: TCPIPserverTest.py -> PyChrono deterministic TCP/IP server replay",
        duration=2.0,
    )


if __name__ == "__main__":
    main()

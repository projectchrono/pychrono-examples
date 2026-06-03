import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))
from pendulum_geom_exact_beam_common import Config, build_system as _build_system, main, run_visual as _run_visual, simulate as _simulate, update_visuals


CONFIG = Config(
    source_name="pendulumGeomExactBeam2Dsimple.py",
    title_suffix="GenerateStraightBeam flexible pendulum",
    duration=1.0,
    generated_beam=True,
)


def build_system():
    return _build_system(CONFIG)


def simulate(duration, step):
    return _simulate(CONFIG, duration, step)


def run_visual(duration, step):
    return _run_visual(CONFIG, duration, step)


if __name__ == "__main__":
    main(CONFIG)

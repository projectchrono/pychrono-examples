import argparse
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent))
from ancf_belt_drive import (
    CONTACT_SEGMENTS_PER_CABLE,
    DRY_FRICTION,
    ELEMENTS,
    END_TIME,
    NODE_COUNT,
    PRESTRETCH,
    ROLL_DAMPING,
    ROLL_STIFFNESS,
    SECTIONS,
    STEP,
    build_system,
    print_state,
    run_visual,
    simulate,
    update_visuals,
)


SOURCE_REFERENCE_TIP_Y = -0.4842656133238705


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=END_TIME)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()

    print("EXUDYN port: TestModels/ANCFbeltDrive.py -> PyChrono ANCF belt-drive test replay")
    print(
        f"source parameters: elements={ELEMENTS} nodes={NODE_COUNT} sections={SECTIONS} "
        f"contact_segments={ELEMENTS * CONTACT_SEGMENTS_PER_CABLE} mu={DRY_FRICTION:.3f} "
        f"preStretch={PRESTRETCH:+.4f} rollSpring=({ROLL_STIFFNESS:.3e},{ROLL_DAMPING:.3e})"
    )
    if args.no_vis:
        system, _items = simulate(args.duration, args.step)
        print_state(system)
        print(f"source_reference_tip_y={SOURCE_REFERENCE_TIP_Y:+.12f}")
    else:
        run_visual(args.duration, args.step)


if __name__ == "__main__":
    main()

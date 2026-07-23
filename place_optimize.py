"""
PCB Placement Optimizer - Perturbative refinement of an existing placement.

Usage:
  python place_optimize.py input.kicad_pcb [output.kicad_pcb] [options]

Starts from the current (hand- or AI-made) placement and applies a greedy
quench: small nudges, 90-degree rotations, and same-footprint swaps that
reduce airwire length + crossings + a whitespace (halo) penalty. Every move
type keeps each part within --max-displacement of its original position -
same-footprint swaps obey the same cap (or the tighter
--swap-max-displacement) - so the output is "your placement, nudged" and
implicit constraints encoded in the original placement survive. Locked
footprints never move.

See docs/placement-optimization.md for the background research.
"""

import os

from kicad_parser import parse_kicad_pcb
import routing_defaults as defaults
from placement.quench import quench
from placement.writer import write_placed_output


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="Perturbative placement optimizer (greedy quench).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python place_optimize.py input.kicad_pcb optimized.kicad_pcb
  python place_optimize.py input.kicad_pcb --max-displacement 5 --no-rotate
"""
    )
    parser.add_argument("input_file", help="Input KiCad PCB file (unrouted)")
    parser.add_argument("output_file", nargs="?",
                        help="Output file (default: input_optimized.kicad_pcb)")
    parser.add_argument("--max-displacement", type=float, default=10.0,
                        help="Max distance a part may move from its original "
                             "position in mm (default: 10)")
    parser.add_argument("--swap-max-displacement", type=float, default=None,
                        help="Max distance a swap may move each part from its "
                             "original position in mm; must not exceed "
                             "--max-displacement (default: equal to it)")
    parser.add_argument("--step", type=float, default=1.0,
                        help="Candidate grid step in mm (default: 1.0)")
    parser.add_argument("--grid-step", type=float, default=defaults.GRID_STEP,
                        help=f"Routing grid snap in mm (default: {defaults.GRID_STEP})")
    parser.add_argument("--clearance", type=float, default=defaults.CLEARANCE,
                        help=f"Min courtyard gap in mm (default: {defaults.CLEARANCE})")
    parser.add_argument("--board-edge-clearance", type=float, default=0.55,
                        help="Hard clearance from board edge in mm (default: 0.55)")
    parser.add_argument("--crossing-penalty", type=float, default=10.0,
                        help="Cost in mm per airwire crossing (default: 10)")
    parser.add_argument("--length-weight", type=float, default=1.0,
                        help="Weight on total airwire length (default: 1.0)")
    parser.add_argument("--halo-base", type=float, default=0.5,
                        help="Base whitespace halo around each part in mm "
                             "(default: 0.5)")
    parser.add_argument("--halo-coef", type=float, default=0.25,
                        help="Extra halo per sqrt(pin count) in mm; spreads "
                             "high-pin-count parts (default: 0.25)")
    parser.add_argument("--halo-weight", type=float, default=2.0,
                        help="Quadratic penalty weight for halo shortfall "
                             "(default: 2.0)")
    parser.add_argument("--edge-halo", type=float, default=2.0,
                        help="Soft margin inside the board edge in mm "
                             "(default: 2.0)")
    parser.add_argument("--edge-weight", type=float, default=2.0,
                        help="Quadratic penalty weight for edge-margin "
                             "shortfall (default: 2.0)")
    parser.add_argument("--ignore-nets", nargs="+", default=None,
                        metavar="NET",
                        help="Net name patterns to exclude from airwire "
                             "scoring (e.g. plane-routed power nets)")
    parser.add_argument("--lock", nargs="+", default=None, metavar="REF",
                        help="Additional reference patterns to lock in place "
                             "(e.g. connectors)")
    parser.add_argument("--no-rotate", action="store_true",
                        help="Disable rotation moves")
    parser.add_argument("--no-swap", action="store_true",
                        help="Disable same-footprint swap moves")
    parser.add_argument("--max-passes", type=int, default=10,
                        help="Max quench passes (default: 10)")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Print each accepted move")

    args = parser.parse_args()

    if args.swap_max_displacement is not None:
        if args.swap_max_displacement < 0:
            parser.error("--swap-max-displacement must be >= 0")
        if args.swap_max_displacement > args.max_displacement:
            parser.error("--swap-max-displacement must not exceed "
                         "--max-displacement")

    if args.output_file is None:
        base, ext = os.path.splitext(args.input_file)
        args.output_file = base + '_optimized' + ext
        print(f"Output file: {args.output_file}")

    print(f"Loading {args.input_file}...")
    pcb_data = parse_kicad_pcb(args.input_file)

    placements = quench(
        pcb_data,
        pcb_file=args.input_file,
        max_displacement=args.max_displacement,
        swap_max_displacement=args.swap_max_displacement,
        step=args.step,
        grid_step=args.grid_step,
        clearance=args.clearance,
        board_edge_clearance=args.board_edge_clearance,
        crossing_penalty=args.crossing_penalty,
        length_weight=args.length_weight,
        halo_base=args.halo_base,
        halo_coef=args.halo_coef,
        halo_weight=args.halo_weight,
        edge_halo=args.edge_halo,
        edge_weight=args.edge_weight,
        allow_rotations=not args.no_rotate,
        allow_swaps=not args.no_swap,
        max_passes=args.max_passes,
        ignore_nets=args.ignore_nets,
        lock_refs=args.lock,
        verbose=args.verbose,
    )

    print(f"{len(placements)} parts moved")
    write_placed_output(args.input_file, args.output_file, placements)


if __name__ == "__main__":
    main()

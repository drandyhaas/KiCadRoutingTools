"""
PCB Component Placement Tool - Spaces components evenly within the board boundary.

Usage:
  python place.py input.kicad_pcb [output.kicad_pcb] [options]

Assumes an unrouted PCB. Places components in rows within the board boundary,
sorted largest-first, snapped to the routing grid.
"""

import os
import sys

from kicad_parser import parse_kicad_pcb
import routing_defaults as defaults
from placement.engine import place_components_initially
from placement.writer import write_placed_output


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="PCB Component Placement - Spaces components evenly within the board boundary.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python place.py input.kicad_pcb placed.kicad_pcb
  python place.py input.kicad_pcb --clearance 0.5 --grid-step 0.05
  python place.py input.kicad_pcb -O  # overwrite input file
"""
    )
    parser.add_argument("input_file", help="Input KiCad PCB file (assumed unrouted)")
    parser.add_argument("output_file", nargs="?",
                        help="Output KiCad PCB file (default: input_placed.kicad_pcb)")
    parser.add_argument("--overwrite", "-O", action="store_true",
                        help="Overwrite input file instead of creating _placed copy")
    parser.add_argument("--grid-step", type=float, default=defaults.GRID_STEP,
                        help=f"Grid resolution in mm (default: {defaults.GRID_STEP})")
    parser.add_argument("--clearance", type=float, default=defaults.CLEARANCE,
                        help=f"Minimum gap between components in mm (default: {defaults.CLEARANCE})")
    parser.add_argument("--board-edge-clearance", type=float, default=0.55,
                        help="Clearance from board edge in mm (default: 0.55)")
    parser.add_argument("--crossing-penalty", type=float, default=10.0,
                        help="Cost in mm per airwire crossing for placement scoring (default: 10.0)")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Print detailed placement info")

    args = parser.parse_args()

    # Determine output file
    if args.output_file is None:
        if args.overwrite:
            args.output_file = args.input_file
        else:
            base, ext = os.path.splitext(args.input_file)
            args.output_file = base + '_placed' + ext
            print(f"Output file: {args.output_file}")

    # Load PCB
    print(f"Loading {args.input_file}...")
    pcb_data = parse_kicad_pcb(args.input_file)

    # Place components
    placements = place_components_initially(
        pcb_data,
        pcb_file=args.input_file,
        grid_step=args.grid_step,
        clearance=args.clearance,
        board_edge_clearance=args.board_edge_clearance,
        crossing_penalty=args.crossing_penalty,
        verbose=args.verbose,
    )

    # Write output
    write_placed_output(args.input_file, args.output_file, placements)


if __name__ == "__main__":
    main()

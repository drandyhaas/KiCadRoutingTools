"""
Fanout-clearance placement repair (issue #130).

Run this AFTER bga_fanout.py. It nudges decoupling caps near a BGA so their
pads clear every foreign-net fanout via by `clearance`, and pulls each cap
pad toward the nearest same-net ball (so a GND/power via dropped there later
also lands on the cap pad). Caps move as little as possible (90-degree
rotations allowed) and never overlap each other; a cap that can't clear a
foreign via grows its displacement budget until it fits or is reported
unresolved.

Usage:
  python place_fanout_clearance.py fanned.kicad_pcb [output.kicad_pcb] [options]

Run it ONCE after ALL fanouts, not after each: the pass is board-global (it
reads every via and every BGA footprint), so one late run sees every constraint
at once. Per-BGA runs compound displacement -- each cap's seed is wherever it
sits on the board it is handed, so a second run re-seeds at the already-moved
position -- and they change what later fanouts route around, since cap pads are
in the escape router's obstacle map.

Pipeline: bga_fanout.py (all of them) -> place_fanout_clearance.py -> (gnd/power
vias, route)
"""
import _path  # noqa: F401  (py_placer -> py_router/py_tools on sys.path)

import os
import sys  # declare_lever() below reads sys.argv -- without this the module
             # raised NameError on EVERY invocation, --help included (run 20)

from kicad_parser import parse_kicad_pcb
import routing_defaults as defaults
from bga_fanout.constants import DEFAULT_VIA_SIZE
from placement.fanout_clearance import repair_fanout_clearance
from placement.writer import write_placed_output


def main():
    import argparse

    # This step mutates the board mid-pipeline (moves caps), so it must appear in
    # the stress-test redo manifest -- otherwise a pure redo_stress_test.py replay
    # breaks at the next step that reads the *_capopt board. No-op unless
    # REDO_MANIFEST is set (#132).
    from redo_record import record_invocation
    record_invocation()

    parser = argparse.ArgumentParser(
        description="Tidy near-BGA decoupling caps around fanout vias (#130).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python place_fanout_clearance.py fanned.kicad_pcb
  python place_fanout_clearance.py fanned.kicad_pcb cleared.kicad_pcb \\
      --clearance 0.1 --max-displacement 2
"""
    )
    parser.add_argument("input_file", help="Input KiCad PCB file (post-fanout)")
    parser.add_argument("output_file", nargs="?",
                        help="Output file (default: input_capcleared.kicad_pcb)")
    parser.add_argument("--clearance", type=float, default=defaults.CLEARANCE,
                        help=f"DRC clearance in mm (default: {defaults.CLEARANCE})")
    parser.add_argument("--grid-step", type=float, default=defaults.GRID_STEP,
                        help=f"Position snap in mm (default: {defaults.GRID_STEP})")
    parser.add_argument("--board-edge-clearance", type=float, default=0.55,
                        help="Hard clearance from board edge in mm (default: 0.55)")
    parser.add_argument("--capture-radius", type=float, default=2.0,
                        help="Max distance over which a same-net ball attracts "
                             "a cap pad in mm (default: 2.0)")
    parser.add_argument("--default-via-size", type=float, default=DEFAULT_VIA_SIZE,
                        help=f"Fallback via outer diameter in mm for vias whose "
                             f"size can't be read (default: {DEFAULT_VIA_SIZE})")
    parser.add_argument("--near-margin", type=float, default=1.0,
                        help="A cap counts as 'near' a BGA if its courtyard is "
                             "within this many mm of the ball field (default: 1.0)")
    parser.add_argument("--step", type=float, default=0.2,
                        help="Candidate nudge grid step in mm (default: 0.2)")
    parser.add_argument("--max-displacement", type=float, default=2.0,
                        help="Initial max move from seed in mm (default: 2.0); "
                             "grown automatically if a cap can't clear")
    parser.add_argument("--max-displacement-cap", type=float, default=3.0,
                        help="Hard ceiling on displacement after growth in mm "
                             "(default: 3.0, decap-sane); a cap that can't clear "
                             "within this is reported unresolved for manual fixing")
    parser.add_argument("--displacement-growth", type=float, default=1.5,
                        help="Budget growth factor when a cap is stuck "
                             "(default: 1.5)")
    parser.add_argument("--no-rotate", action="store_true",
                        help="Disable 90-degree rotation moves")
    parser.add_argument("--cap-prefix", default="C,R,FB",
                        help="Comma-separated reference prefix(es) treated as "
                             "movable passives near a BGA (default: C,R,FB = caps, "
                             "resistors and ferrite beads (#252); multi-pad parts "
                             "like RN arrays are excluded by the 2-copper-pad test)")
    parser.add_argument("--lock", nargs="+", default=None, metavar="REF",
                        help="Additional reference patterns to lock in place")
    parser.add_argument("--max-passes", type=int, default=30,
                        help="Max repair passes (default: 30)")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Print each accepted move")

    args = __import__("cli_nets").pin_dash_digit_values(parser).parse_args()
    from fix_kicad_drc_settings import warn_if_missing_project_floor
    warn_if_missing_project_floor(args.input_file)  # #441: a dropped sibling .kicad_pro strands the DRC floor

    if args.output_file is None:
        base, ext = os.path.splitext(args.input_file)
        args.output_file = base + '_capcleared' + ext
        print(f"Output file: {args.output_file}")

    print(f"Loading {args.input_file}...")
    pcb_data = parse_kicad_pcb(args.input_file)

    result = repair_fanout_clearance(
        pcb_data,
        pcb_file=args.input_file,
        clearance=args.clearance,
        grid_step=args.grid_step,
        board_edge_clearance=args.board_edge_clearance,
        near_margin=args.near_margin,
        capture_radius=args.capture_radius,
        default_via_size=args.default_via_size,
        step=args.step,
        max_displacement=args.max_displacement,
        max_displacement_cap=args.max_displacement_cap,
        displacement_growth=args.displacement_growth,
        allow_rotations=not args.no_rotate,
        cap_prefix=args.cap_prefix,
        lock_refs=args.lock,
        max_passes=args.max_passes,
        verbose=args.verbose,
    )

    if result['placements'] or result.get('via_moves') or result.get('new_segments'):
        write_placed_output(args.input_file, args.output_file,
                            result['placements'],
                            via_moves=result.get('via_moves'),
                            new_segments=result.get('new_segments'),
                            pcb_data=pcb_data)
        print(f"Wrote {args.output_file}")
        # Carry the input board's .kicad_pro to the output (issue #160 chain of
        # custody). Without this, the next pipeline step finds no sibling
        # project, seeds a minimal one, and the board's own DRC rules -- notably
        # min_copper_edge_clearance -- are silently lost for the REST of the
        # chain (ottercast_audio: the 0.5mm edge rule vanished here, so the
        # signal-routing step stamped its board-edge band at the 0.1 track-
        # clearance fallback and shipped 18 board-edge violations). No
        # edge_clearance is passed: --board-edge-clearance here is a placement
        # margin, not a routing-enforced floor, and must not tighten the rule.
        try:
            from fix_kicad_drc_settings import fix_project_for_output
            fix_project_for_output(args.output_file, input_pcb=args.input_file,
                                   clearance=args.clearance)
        except Exception as e:
            print(f"  (skipped DRC-settings fix: {e})")
    else:
        # A no-op must still WRITE the output board (input copied verbatim,
        # sibling .kicad_pro included): recorded chains reference this step's
        # output by name, so skipping the write starves every later step
        # (lpddr4: an all-perimeter BGA escape legitimately places 0 vias,
        # the #445 via gate declines to move caps, and the whole chain died
        # on the missing file). copy_board keeps the DRC-floor custody.
        print("No caps moved; passing the board through unchanged.")
        if os.path.abspath(args.input_file) == os.path.abspath(args.output_file):
            # In-place invocation (X X): the output already IS the input;
            # copying a file onto itself raises SameFileError (recorded
            # manifests use the in-place form, so a no-op crashed replays).
            print(f"{args.output_file} unchanged (in-place)")
            return
        try:
            from copy_board import copy_board
            copy_board(args.input_file, args.output_file)
        except Exception:
            import shutil
            shutil.copyfile(args.input_file, args.output_file)
            _pro = os.path.splitext(args.input_file)[0] + '.kicad_pro'
            if os.path.isfile(_pro):
                shutil.copyfile(_pro, os.path.splitext(args.output_file)[0]
                                + '.kicad_pro')
        print(f"Wrote {args.output_file} (unchanged copy)")


if __name__ == "__main__":
    # In LEVER_REGISTRY, so it must DECLARE -- an entry that writes
    # poses without declaring makes an armed regime refuse the engine
    # itself, which is the failure the registry exists to prevent.
    from placement.provenance import declare_lever
    with declare_lever('place_fanout_clearance.py', sys.argv):
        from console_encoding import enable_utf8_console
        enable_utf8_console()  # cp1252-safe non-ASCII prints (issue #152)
        main()

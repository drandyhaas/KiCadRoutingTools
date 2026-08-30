"""
Fanout-clearance placement repair (issue #130).

Run this AFTER bga_fanout.py. It nudges decoupling caps near a BGA so their
pads clear every foreign-net fanout via by `clearance`, and pulls each cap
pad toward the nearest same-net ball (so a GND/power via dropped there later
also lands on the cap pad). Caps move as little as possible (90-degree
rotations allowed) and never overlap each other; a cap that can't clear
foreign copper -- a via (#130), a track (#278) or a component pad (#275) --
grows its displacement budget until it fits or is reported unresolved.

The summary counts both mechanisms from ONE board state, at the end of the
pass (#746): `resolved R/V initial violations` means "was grazing at the seed
and is clean now", with `(F freed by via-nudge)` naming the share the #313
last resort freed by moving a via rather than the cap. A `Re-grazed by this
pass's own connector copper:` line means those caps were CLEAN before the
nudge and are grazing after it -- whether or not the pass had fixed them
first, since copper this pass draws can also break a cap that arrived clean.
They are in the unresolved list too; this line names the cause, which is us.

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

import math
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
    parser.add_argument("--clearance", type=float, default=None,
                        help="Copper clearance CEILING in mm (#768). GIVEN: every "
                             "net class (Default included) is priced at "
                             "min(class, this), AND the output project is clamped "
                             "down to it. OMITTED: each pair is priced at its own "
                             "net-class clearance (base = the board's Default "
                             "class from the sibling .kicad_pro, else "
                             f"{defaults.CLEARANCE}) and the net CLASSES are "
                             "preserved. The PRESENCE of the flag is the clamp "
                             "switch, exactly as in route.py. Note OMITTED "
                             "preserves the clearance family specifically, not "
                             "the whole project: the size minima, the "
                             "severities and the fab-floor provenance are "
                             "written either way (#441 chain of custody).")
    parser.add_argument("--grid-step", type=float, default=defaults.GRID_STEP,
                        help=f"Position snap in mm (default: {defaults.GRID_STEP})")
    parser.add_argument("--board-edge-clearance", type=float, default=None,
                        help="Hard clearance from board edge in mm (default: the "
                             "board's own min_copper_edge_clearance when it asks "
                             "for MORE than 0.55, else 0.55). #733: resolved by "
                             "the shared engine, so the GUI plugin and "
                             "animate_fanout_clearance.py get the same answer.")
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
    # #768: `type=float` accepts nan and inf. `min(v, nan)` is `v`, so a nan
    # ceiling silently disables the cap while the writeback still clamps -- the
    # #768 defect reproduced through the code that fixes it. Refused rather than
    # guarded downstream, so the message names the argument.
    if args.clearance is not None and not math.isfinite(args.clearance):
        parser.error("--clearance must be a finite number, got %r"
                     % (args.clearance,))
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
        # Both unresolved: None means "the flag was omitted" and the engine
        # resolves the base from the board, the same contract
        # board_edge_clearance has had since #733.
        clearance=args.clearance,
        netclass_ceiling=args.clearance,
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

    def _write_drc_floors():
        """Write back WHAT THIS PASS PRICED AT -- per key (#768/#769).

        Not "write or don't write". Measured at cd623938, this call does two
        jobs: it clamps the clearance family, and it writes the board's own size
        minima / severities / fab_floor_origin -- 16 values on glasgow_revC with
        no --clearance given at all. The custody half must keep happening in
        every exit path; it is only the clearance family that was dishonest.

        Three keys, three rules, each one "the number the run actually used":

        * `clearance` -- the resolved pair floor. GIVEN, that is the ceiling,
          and the netclass map was capped at it, so clamping the classes down to
          it is exactly what was priced. OMITTED, it is the board's own Default
          class, which is already what the project says, so the lower-only
          writeback is a no-op on it. Before #768 this step priced at
          max(base, netclass) and clamped to `--clearance` regardless: measured,
          glasgow_revC at `--clearance 0.1` disclosed 19 pairs priced at 0.2mm
          (netclass) and then shipped a project declaring 0.1; ottercast_audio,
          79 pairs, same two numbers.

        * `clamp_nondefault_netclasses` -- follows the same switch, and must.
          It defaults True, so before #768 a bare run with NO flag at all
          clamped every non-Default class down to the argparse default 0.25.
          A class was only capped if a ceiling was given; clamping one that was
          honoured writes a number this pass never priced at.

        * `hole_clearance` -- the board's own `min_hole_clearance`, passed
          explicitly because `compute_targets` otherwise defaults it to the
          copper clearance. `--clearance` is a copper-clearance ceiling and says
          nothing about copper-to-hole, and the pass does NOT price that from it
          -- it resolves it board-first and prints it. Measured, the same
          glasgow run that printed "Copper-to-hole clearance 0.25mm (from the
          board's own min_hole_clearance)" then wrote 0.25 -> 0.1 into the
          project. Same defect as #768, one key over.

        Called from EVERY exit, which is #769. The clamp used to run only when a
        cap actually moved, so a run asked for a ceiling that legitimately moved
        nothing took the unchanged-copy path and shipped the INPUT's spec for the
        next step to grade against (5754 violations on a board forced to a
        declared 0.80). A zero-move run is a perfectly legitimate outcome and is
        exactly the case where the shipped spec and the work done diverge
        silently.

        No edge_clearance is passed: --board-edge-clearance here is a placement
        margin, not a routing-enforced floor, and must not tighten the rule.
        """
        try:
            from fix_kicad_drc_settings import fix_project_for_output
            from list_nets import board_constraint
            from placement.fanout_clearance import resolve_pair_clearance
            from placement.legality import resolve_npth_floor
            _priced, _ = resolve_pair_clearance(args.input_file, args.clearance)
            # GIVEN, the value to clamp to is the CEILING, not the base. The
            # writeback is lower-only, so passing the ceiling leaves every class
            # at min(its own, ceiling) -- which IS what was priced, per class.
            # Passing the base instead ships a class that sits BETWEEN the
            # board's Default and the ceiling below the value it was priced at:
            # measured, Default 0.2 / Wide 0.4 / --clearance 0.3 prices 10 pairs
            # at 0.3 and shipped 0.2. That is #768's own shape in the safe
            # direction, and it is still #768's shape.
            # OMITTED, there is no ceiling and `_priced` is the board's own
            # class, which the lower-only write then leaves alone.
            _target = args.clearance if args.clearance is not None else _priced
            # And the copper-to-hole floor is the one the pass USED. The board's
            # own declaration when it has one; otherwise the NPTH floor the
            # keep-outs were actually priced at, because `compute_targets`
            # defaults this key to the copper clearance and `apply_targets`
            # CREATES the rule when the project has none -- so a board that
            # declared nothing got `--clearance` written in as its copper-to-hole
            # rule while the pass had priced that geometry at 0.2.
            _hc = board_constraint(args.input_file, 'min_hole_clearance')
            if _hc is None:
                try:
                    _hc = resolve_npth_floor(pcb_data, args.input_file)
                except Exception:                              # noqa: BLE001
                    _hc = None
            fix_project_for_output(
                args.output_file, input_pcb=args.input_file,
                clearance=_target,
                hole_clearance=_hc,
                clamp_nondefault_netclasses=args.clearance is not None)
        except Exception as e:
            print(f"  (skipped DRC-settings fix: {e})")

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
        # clearance fallback and shipped 18 board-edge violations).
        _write_drc_floors()
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
            # Still the same rule: in-place means the output project IS the
            # input's, so a ceiling that was asked for must land on it.
            _write_drc_floors()
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
        _write_drc_floors()


if __name__ == "__main__":
    # In LEVER_REGISTRY, so it must DECLARE -- an entry that writes
    # poses without declaring makes an armed regime refuse the engine
    # itself, which is the failure the registry exists to prevent.
    from placement.provenance import declare_lever
    with declare_lever('place_fanout_clearance.py', sys.argv):
        from console_encoding import enable_utf8_console
        enable_utf8_console()  # cp1252-safe non-ASCII prints (issue #152)
        main()

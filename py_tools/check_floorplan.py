#!/usr/bin/env python3
"""Grade a board against a declared floorplan intent (#549).

Placement is judged here by `crossings` and `hpwl`, and those are indifferent
between a sensible layout and a scattered one with the same wirelength. Nothing
declares where parts BELONG, so nothing can check whether they went there, and a
render only moves the judgement from a number to a vibe.

This makes the intent falsifiable. Write what the floorplan is meant to be, and
this measures the board against it and exits non-zero with the number that broke.

    # start from what the board already is, then edit it
    python3 check_floorplan.py board.kicad_pcb --emit-intent floorplan.json

    # grade
    python3 check_floorplan.py board.kicad_pcb --intent floorplan.json
    python3 check_floorplan.py board.kicad_pcb --intent floorplan.json --json out.json

THE BOARD OUTLINE IS NOT EDITABLE BY THIS TOOLCHAIN. `envelope` is READ from the
board; a part outside it is a finding about the PART. Board size, cutouts and
slots are mechanical decisions -- enclosure fit, panel rails, connector
apertures -- that belong to the user. The honest response to a board that is
genuinely too small is to say so with the measured number and stop, never to
resize it. Nothing here writes Edge.Cuts.

Exit codes:

    0   graded, no error-severity violations (or --exit-zero)
    1   crash
    2   argparse, including an unreadable or malformed intent file
    3   the board is not in a state this tool can work on (unplaced / no
        trustworthy outline) -- the placement-family convention
    4   graded successfully, error-severity violations found

4 rather than 1 because 3 is already taken and 1 is ambiguous with a crash: a
caller has to be able to tell "the floorplan is wrong" from "the grader is
broken", and that distinction is the entire product. (`check_drc.py` uses 1 for
its violations; this diverges knowingly.)
"""
from __future__ import annotations
import _path  # noqa: F401  (py_tools -> py_router/py_placer on sys.path)

import argparse
import json
import os
import sys

from kicad_parser import parse_kicad_pcb
from placement.cli_gates import add_board_state_args
from placement.floorplan import (UntrustworthyOutline, emit_intent, format_text,
                                 grade, load_intent, summary, to_json)
from placement.placement_state import UNPLACED_EXIT, gate_or_exit
from placement.groups import GroupError, parse_sources
from placement.floorplan import IntentError

VIOLATIONS_EXIT = 4


def build_parser():
    p = argparse.ArgumentParser(
        description="Grade a board against a declared floorplan intent",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split('Exit codes:')[0].strip())
    p.add_argument('board', help='the .kicad_pcb to grade')
    p.add_argument('--intent', metavar='PATH',
                   help='the floorplan intent JSON to grade against')
    p.add_argument('--emit-intent', metavar='PATH',
                   help='write a starter intent READ OFF this board and exit. '
                        'It grades clean by construction -- a baseline to '
                        'tighten, with the real block names filled in')
    p.add_argument('--declare-classes', action='store_true',
                   help='with --emit-intent: ALSO declare edge-class parts '
                        '(part_class KB, run-4 A) that are not currently '
                        'overhanging -- a USB receptacle parked interior gets '
                        'an edge_connectors entry with a class-default band '
                        'and NO edge (reconstruct derives it). DELIBERATELY '
                        'breaks grades-clean-by-construction on a damaged '
                        'board: an implausibly-posed receptacle then FAILS '
                        'the proximity rule, which is the detection working. '
                        'Default off to preserve the observation-only round '
                        'trip')
    p.add_argument('--declare-decaps', action='store_true',
                   help='with --emit-intent: ALSO derive decaps.'
                        'max_distance_mm from the board\'s own measured '
                        'tethers (#704), so rule_decap_distance can fire on '
                        'an auto intent at all. The limit is ceil(max) over '
                        'placement.groups.decap_tethers, so it grades CLEAN '
                        'by construction, and it is WITHHELD into '
                        'context.budget_withheld when the number would be '
                        'dishonest: no tethers, fewer than 3, or more than '
                        'a quarter of the rail-sharing caps already beyond '
                        'the search radius. SEPARATE from --declare-classes '
                        'on purpose -- this key is not a part CLASS, and '
                        'declaring it CHANGES PLACEMENT: place_seed reads it '
                        'and moves the caps that elect a tether out of zone '
                        'packing into its per-supply-pin stage (ulx3s: 60 of '
                        '70, the other 10 having no rail-carrying chip to be '
                        'seated at). It also arms decap_ungraded, which reports at WARN '
                        'the caps beyond the search radius that the limit '
                        'was never derived from. The tether census is '
                        'written to context.decap_census either way')
    p.add_argument('--json', metavar='PATH',
                   help='write the full findings (every measurement) as JSON')
    p.add_argument('--group-by', default='auto', metavar='SOURCES',
                   help="how blocks are derived when an intent block names a "
                        "`group`: comma list of kicad,sheet,netprefix,decap; "
                        "'auto' = kicad,sheet (default); 'none' to disable. "
                        "Deriving is read-only here, which is why this defaults "
                        "ON unlike on the placement CLIs")
    p.add_argument('--clearance', type=float, metavar='MM',
                   help="clearance the legality model grades at (default: "
                        "the board's own Default netclass, else 0.25). "
                        "Grading a fixed constant tighter than the board's "
                        "floor manufactures phantom violations")
    p.add_argument('--board-edge-clearance', type=float, metavar='MM',
                   help="board-edge clearance for the outline gate (default: "
                        "the board's own min_copper_edge_clearance, else "
                        "0.55)")
    p.add_argument('--health', action='store_true',
                   help="also report the routability signals from the intent's "
                        "`health` block: how far each block sits from the parts "
                        "it connects to, and what crosses each declared bus "
                        "corridor. Advisory -- these say the floorplan will "
                        "fight the router, not that it breaks the intent")
    p.add_argument('--require-rules', type=int, default=0, metavar='N',
                   help='fail unless at least N rules actually RAN. Default 0 '
                        '(off), which keeps the long-standing contract that a '
                        'vacuous intent exits 0 -- but that contract is how an '
                        'intent running ZERO rules constrained two placement '
                        'laps with nothing, twice, and every consumer stayed '
                        'silent. A caller that means "grade this" should pass '
                        '--require-rules 1.')
    p.add_argument('--exit-zero', action='store_true',
                   help='report violations but exit 0 anyway')
    p.add_argument('-q', '--quiet', action='store_true',
                   help='suppress the text report; keep JSON_SUMMARY')
    add_board_state_args(p)
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    parser_error = build_parser().error

    if not args.intent and not args.emit_intent:
        parser_error('one of --intent or --emit-intent is required')
    if not os.path.exists(args.board):
        parser_error(f"{args.board}: no such file")

    try:
        sources = parse_sources(args.group_by)
    except GroupError as exc:
        parser_error(str(exc))

    pcb = parse_kicad_pcb(args.board)

    # The unplaced gate first: grading a pile of parts at the origin fires every
    # rule at once, which is noise rather than a verdict.
    gate_or_exit(pcb, args.board, 'check_floorplan',
                 allow_unplaced=args.allow_unplaced,
                 allow_routed=True)          # copper is irrelevant to a floorplan

    if args.emit_intent:
        try:
            doc = emit_intent(pcb, args.board, group_sources=sources or (),
                              declare_classes=args.declare_classes,
                              derive_decaps=args.declare_decaps)
        except UntrustworthyOutline as exc:
            print(f"ERROR: {args.board}: {exc}", file=sys.stderr)
            return UNPLACED_EXIT
        with open(args.emit_intent, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh, indent=1, sort_keys=True)
            fh.write('\n')
        if not args.quiet:
            zoned = sum(1 for b in doc['blocks'] if 'zone' in b)
            print(f"Wrote {args.emit_intent}")
            print(f"  {len(doc['blocks'])} block(s), {zoned} with a zone, "
                  f"{len(doc['edge_connectors'])} edge connector(s), "
                  f"{len(doc['must_lock'])} locked part(s)")
            print(f"  envelope {doc['envelope']['rect']} -- read from the "
                  f"board. The outline is not editable by this toolchain")
            # #704: the decap number and what it COSTS, next to each other.
            cen = (doc.get('context') or {}).get('decap_census') or {}
            lim = (doc.get('decaps') or {}).get('max_distance_mm')
            held = ((doc.get('context') or {}).get('budget_withheld')
                    or {}).get('decaps.max_distance_mm')
            if lim is not None:
                print(f"  decaps: max_distance_mm {lim} from "
                      f"{cen.get('tethers')} tether(s) -- NOTE: place_seed "
                      f"READS this key and will seat "
                      f"{cen.get('seeder_pin_scope')} cap(s) per supply pin "
                      f"instead of zone-packing them")
            elif held:
                print(f"  decaps: max_distance_mm WITHHELD -- {held}")
            # The two causes are printed SEPARATELY (#792). One number
            # used to carry both, and the doc explained it with a third
            # cause -- a predicate mismatch -- that measurement says does
            # not exist. A reader cannot act on a conflated count: the
            # first is a grading hole to widen or accept, the second is a
            # design fact about caps that have no IC at all.
            if cen.get('beyond_radius'):
                print(f"  decap census: {cen['beyond_radius']} rail-sharing "
                      f"cap(s) lie beyond the {cen['search_radius_mm']}mm "
                      f"search radius (worst {cen['worst_beyond_mm']}mm), "
                      f"so the limit was not derived from them -- "
                      f"decap_ungraded reports them at WARN")
            if cen.get('no_rail_chip'):
                print(f"  decap census: {cen['no_rail_chip']} cap(s) have "
                      f"NO chip carrying their rail "
                      f"({', '.join(cen.get('no_rail_chip_refs') or [])}) "
                      f"-- bulk or filter caps with no IC to be near; "
                      f"graded by nothing, and correctly so")
        return 0

    try:
        intent = load_intent(args.intent)
    except IntentError as exc:
        parser_error(str(exc))

    # Run-7 S1: unset knobs grade at the BOARD's floor, not fixed constants
    # (a 0.25 default on a 0.15 board manufactured phantom oob findings).
    from list_nets import board_floor_knobs
    clearance, edge_clearance, knobs = board_floor_knobs(
        args.board, args.clearance, args.board_edge_clearance)
    if not args.quiet:
        print(f"grading at clearance {clearance} "
              f"({knobs['clearance']['source']}), edge {edge_clearance} "
              f"({knobs['board_edge_clearance']['source']})")

    try:
        result = grade(intent, pcb, args.board, group_sources=sources or (),
                       clearance=clearance,
                       board_edge_clearance=edge_clearance,
                       with_health=args.health)
    except UntrustworthyOutline as exc:
        print(f"ERROR: {args.board}: {exc}", file=sys.stderr)
        print("  Refused rather than graded: with no usable outline every "
              "containment check silently degrades to the bounding box, and a "
              "clean report would mean 'stopped checking'.", file=sys.stderr)
        return UNPLACED_EXIT

    if not args.quiet:
        print(format_text(result))

    if args.json:
        with open(args.json, 'w', encoding='utf-8') as fh:
            json.dump(to_json(result), fh, indent=1, sort_keys=True)
            fh.write('\n')
        if not args.quiet:
            print(f"  wrote {args.json}")

    s = summary(result)
    s['clearance_used'] = knobs['clearance']
    s['edge_clearance_used'] = knobs['board_edge_clearance']
    print("JSON_SUMMARY: " + json.dumps(s, sort_keys=True))
    # "0 violations" and "0 rules ran" must not look the same to a machine --
    # this module's own docstring says so, and until now only the DATA said it
    # (`rules_run` / `rules_skipped`); the exit code could not. Opt-in, so the
    # pinned default contract is untouched.
    _ran = s.get('rules_run', 0)
    if args.require_rules and _ran < args.require_rules:
        print(f"  FAIL: --require-rules {args.require_rules}, but {_ran} "
              f"rule(s) ran. A grade that ran no rules is a VACUOUS PASS, not "
              f"a clean board: {s.get('rules_skipped', 0)} rule(s) were "
              f"skipped because the intent declares nothing they apply to.",
              file=sys.stderr)
        if not args.exit_zero:
            return VIOLATIONS_EXIT
    if result.errors and not args.exit_zero:
        return VIOLATIONS_EXIT
    return 0


if __name__ == '__main__':
    import cli_banner; cli_banner.install()  # CMD/EXIT self-echo (run-3 B1)
    sys.exit(main())

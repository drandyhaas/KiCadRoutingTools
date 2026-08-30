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
import _path  # noqa: F401  (py_placer -> py_router/py_tools on sys.path)

import json
import os
import sys

from kicad_parser import parse_kicad_pcb
import routing_defaults as defaults
from placement.groups import GroupError, derive_groups, describe, parse_sources
from placement.cli_gates import (add_board_state_args, add_intent_arg,
                                 add_lock_advisor_args, add_tidiness_args)
from placement.portfolio import copy_siblings
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
    # BOARD-FIRST, like check_floorplan and converge. `None` is the "not
    # given" signal these knobs never had: a fixed 0.25 on a board whose
    # declared floor is 0.2 grades 34% more shortfall and double the oob count,
    # and this tool VETOES candidate moves on that number -- so the wrong floor
    # does not just mis-report, it steers the search.
    parser.add_argument("--clearance", type=float, default=None,
                        help="Min courtyard gap in mm (default: the board's "
                             "own Default net-class clearance, else "
                             f"{defaults.CLEARANCE})")
    parser.add_argument("--board-edge-clearance", type=float, default=None,
                        help="Hard clearance from board edge in mm (default: "
                             "the board's own min_copper_edge_clearance, else "
                             "0.55)")
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
    parser.add_argument("--group-by", default="none",
                        help="Move blocks of parts as one rigid body. Comma "
                             "list of: kicad (KiCad (group ...) blocks), sheet "
                             "(schematic sheet), netprefix, decap (caps tethered "
                             "to their IC); 'auto' = kicad,sheet "
                             "(default: none, i.e. per-part moves only)")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Print each accepted move")
    parser.add_argument("--courtyard-only", action="store_true",
                        help="Disable the pad+drill legality layer and gate on "
                             "courtyard rects only (the pre-layer behavior). "
                             "Escape hatch for A/B comparison; the layer is ON "
                             "by default because a quench that walks copper "
                             "into a neighbor's clearance is a defect")
    parser.add_argument("--min-gain-per-mm", type=float, default=0.1,
                        help="A move must improve the objective by at least "
                             "this much per mm of displacement (anti-churn; "
                             "0 restores the accept-any-improvement behavior; "
                             "default: 0.1)")
    parser.add_argument("--move-unconnected", action="store_true",
                        help="Allow parts with no connected pins (mounting "
                             "holes, fiducials) to move. They are frozen by "
                             "default: the airwire cost cannot see them, so "
                             "only halo/edge terms decide where they go")
    add_board_state_args(parser)
    add_intent_arg(parser)
    add_lock_advisor_args(parser)
    add_tidiness_args(parser)

    args = parser.parse_args()

    # Run-7 S1: unset knobs grade at the BOARD's floor, not fixed constants.
    from list_nets import board_floor_knobs
    args.clearance, args.board_edge_clearance, _knobs = board_floor_knobs(
        args.input_file, args.clearance, args.board_edge_clearance)
    print(f"legality at clearance {args.clearance} "
          f"({_knobs['clearance']['source']}), edge "
          f"{args.board_edge_clearance} "
          f"({_knobs['board_edge_clearance']['source']})")

    if args.swap_max_displacement is not None:
        if args.swap_max_displacement < 0:
            parser.error("--swap-max-displacement must be >= 0")
        if args.swap_max_displacement > args.max_displacement:
            parser.error("--swap-max-displacement must not exceed "
                         "--max-displacement")

    # #702: the intent is loaded BEFORE record_invocation, because an exit 2 on
    # an unreadable one touches no file -- and a manifest that carries a
    # command which produced nothing leaves the next step's input made by
    # nothing, which is exactly why the recorder is conditional below.
    # Not parser.error on the --suggest-locks combination: a caller that always
    # passes --intent must not have to special-case the report mode, which
    # writes no board and never quenches.
    from placement.cli_gates import load_intent_or_exit
    intent, _rc = load_intent_or_exit(args)
    if _rc:
        return _rc

    # Recorded AFTER parsing and only for a real run: this tool MUTATES the
    # board, so a stress manifest that omits it leaves the next step's input
    # produced by nothing and the pruner drops legitimate upstream steps.
    # Report-only modes are skipped so a manifest never carries a command that
    # touched no file (route.py records --list-groups for exactly that reason,
    # and manifest_to_plan then has to refuse it loudly).
    if not args.suggest_locks:
        try:
            from redo_record import record_invocation
            record_invocation()
        except Exception:
            pass

    if args.output_file is None:
        base, ext = os.path.splitext(args.input_file)
        args.output_file = base + '_optimized' + ext

    print(f"Loading {args.input_file}...")
    pcb_data = parse_kicad_pcb(args.input_file)

    if args.suggest_locks:
        from placement.lock_advisor import advise_locks, format_text, to_json
        advice = advise_locks(pcb_data, args.input_file,
                              lock_patterns=args.lock,
                              min_confidence=args.lock_confidence,
                              edge_margin=args.lock_edge_margin,
                              use_globs=args.suggest_locks_globs)
        print(format_text(advice))
        if args.suggest_locks_json:
            with open(args.suggest_locks_json, 'w', encoding='utf-8') as f:
                json.dump(to_json(advice), f, indent=1, sort_keys=True)
            print(f"Wrote {args.suggest_locks_json}")
        print("JSON_SUMMARY: " + json.dumps(advice.tally(), sort_keys=True))
        return 0

    # Board-state gates (#431): refuse rather than produce a result-shaped
    # non-result. See placement/placement_state.py for why each fires.
    from placement.placement_state import gate_or_exit
    gate_or_exit(pcb_data, args.input_file, 'place_optimize.py',
                 allow_unplaced=args.allow_unplaced,
                 allow_routed=args.allow_routed)
    print(f"Output file: {args.output_file}")

    try:
        sources = parse_sources(args.group_by)
    except GroupError as exc:
        parser.error(str(exc))
    blocks = derive_groups(pcb_data, sources) if sources else {}
    if sources:
        print(describe(blocks))

    intent_gate = None
    if intent is not None:
        from placement.cli_gates import resolve_intent_gate_for_cli
        intent_gate, _problems = resolve_intent_gate_for_cli(
            intent, pcb_data, sources, args.intent)

    # Exact pad/hole legality of the INPUT (file poses): the before half of
    # the report pair. Gate-currency tallies live inside the quench; this is
    # the phantom-free number an auditor compares.
    from placement.legality import (grade_pad_legality,
                                    format_oob_clause as _oob_clause,
                                    format_required_clause as _req_clause)
    legality_before = None
    if not args.courtyard_only:
        legality_before = grade_pad_legality(pcb_data, args.clearance,
                                             pcb_file=args.input_file)
        print(f"Pad legality before: {legality_before['pad_conflicts']} "
              f"conflict pair(s), {legality_before['hole_conflicts']} hole "
              f"conflict(s), {legality_before['oob_pad_count']} part(s) with "
              f"pad copper off-board"
              + (": " + _oob_clause(legality_before)
                 if _oob_clause(legality_before) else ""))
        if _req_clause(legality_before):
            print(f"  above the {args.clearance}mm floor: "
                  f"{_req_clause(legality_before)}")

    ratsnest = {}
    summary = {'parts_moved': 0}
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
        align_weight=args.align_weight,
        align_radius=args.align_radius,
        align_span=args.align_span,
        orient_weight=args.orient_weight,
        metrics_out=ratsnest,
        groups=blocks,
        verbose=args.verbose,
        pad_legality=not args.courtyard_only,
        min_gain_per_mm=args.min_gain_per_mm,
        move_unconnected=args.move_unconnected,
        intent_gate=intent_gate,
    )

    print(f"{len(placements)} parts moved")
    write_placed_output(args.input_file, args.output_file, placements)
    # #441: the sibling .kicad_pro carries the DRC floor the chain routed to.
    # Without it the next step resolves its floor from the STOCK netclass and
    # stamps that looser value over tighter copper, so KiCad grades correct
    # sub-floor copper as phantom clearance DRC. place_seed and place_portfolio
    # already did this; these two did not, and SKILL.md documented the gap as a
    # manual `cp` the caller had to remember.
    copy_siblings(args.input_file, args.output_file)

    # #504: the ratsnest and legality numbers used to be printed and discarded,
    # so nothing downstream could gate on what a quench run actually achieved.
    # Same shape as route_planes.py's JSON_SUMMARY (#487's plane-resistance fix).
    before, after = ratsnest.get('before', {}), ratsnest.get('after', {})
    summary.update({
        'parts_moved': len(placements),
        'blocks': len(blocks),
        'block_parts': sum(len(v) for v in blocks.values()),
        # UNWEIGHTED, comparable across runs.
        'crossings_before': before.get('crossings'),
        'crossings_after': after.get('crossings'),
        'hpwl_before': before.get('hpwl'),
        'hpwl_after': after.get('hpwl'),
        # Scaled by net_weights -- compare before/after within THIS run only.
        'airwire_length_before': before.get('length'),
        'airwire_length_after': after.get('length'),
        'cost_before': before.get('total'),
        'cost_after': after.get('total'),
        # The two pad-conflict numbers here are DIFFERENT CURRENCIES and sat
        # adjacent under near-identical names, ~2x apart on one unchanged
        # board. `pad_conflict_pairs` is the quench's AABB gate currency
        # (conservative: a bbox graze on a rotated or round pad counts, and a
        # pair over PAIR_TEST_CAP falls back to an extent-level verdict).
        # `pad_conflicts_before/_after` re-grade the written file with exact
        # pad geometry and carry no phantoms. A >= B is structural, not a bug.
        'pad_conflict_pairs_currency': 'aabb-gate (conservative, phantoms)',
        'pad_conflicts_currency': 'exact geometry (phantom-free)',
    })
    summary.update(ratsnest.get('legality', {}))
    # #702: what the declared-intent gate DID. Reported even when it refused
    # nothing, for the reason docs/floorplan-intent.md gives about the grader's
    # own rules_run/rules_skipped pair -- "0 violations" and "0 rules ran" must
    # not look the same to a machine. Here: "the gate refused nothing" and
    # "there was no gate" must not either.
    _ig = ratsnest.get('intent_gate')
    if _ig is not None:
        print(f"intent gate: enforced {', '.join(_ig['rules_enforced'])} over "
              f"{_ig['refs_bound']} bound part(s); refused "
              f"{_ig['rejected']} candidate pose(s)"
              + (" (" + ", ".join(f"{r} {n}" for r, n
                                  in sorted(_ig['by_rule'].items())) + ")"
                 if _ig['by_rule'] else ""))
        summary['intent'] = args.intent
        summary['intent_rules_enforced'] = _ig['rules_enforced']
        summary['intent_refs_bound'] = _ig['refs_bound']
        summary['intent_moves_refused'] = _ig['rejected']
        summary['intent_moves_refused_by_rule'] = _ig['by_rule']
        summary['intent_moves_refused_by_site'] = _ig['by_site']
    elif args.intent:
        summary['intent'] = args.intent
        summary['intent_moves_refused'] = None    # gate built nothing
    # After half of the exact report pair, graded on the WRITTEN file so it
    # covers exactly what the next step will read. WARN on any worsened
    # category -- the in-run gates should make this impossible; a warning here
    # means a gate has a hole and is worth a bug report.
    if legality_before is not None:
        legality_after = grade_pad_legality(parse_kicad_pcb(args.output_file),
                                            args.clearance,
                                            pcb_file=args.output_file)
        print(f"Pad legality after: {legality_after['pad_conflicts']} "
              f"conflict pair(s), {legality_after['hole_conflicts']} hole "
              f"conflict(s), {legality_after['oob_pad_count']} part(s) with "
              f"pad copper off-board"
              + (": " + _oob_clause(legality_after)
                 if _oob_clause(legality_after) else ""))
        if _req_clause(legality_after):
            print(f"  above the {args.clearance}mm floor: "
                  f"{_req_clause(legality_after)}")
        summary['pad_clearance_required'] = legality_after['required']
        for key in ('pad_conflicts', 'hole_conflicts', 'oob_pad_count'):
            summary[f'{key}_before'] = legality_before[key]
            summary[f'{key}_after'] = legality_after[key]
            if legality_after[key] > legality_before[key]:
                print(f"WARNING: {key} WORSENED "
                      f"{legality_before[key]} -> {legality_after[key]} -- "
                      f"the legality gate should have prevented this")
        summary['pad_shortfall_before'] = legality_before['pad_shortfall']
        summary['pad_shortfall_after'] = legality_after['pad_shortfall']
    print('JSON_SUMMARY: ' + json.dumps(summary, sort_keys=True, default=str),
          flush=True)


if __name__ == "__main__":
    # Declare the lever for the WHOLE run, so every pose this CLI
    # writes carries its name. Nothing called declare_lever outside
    # tests, so the unaided instrument had no armed state at all:
    # unarmed it is silent, and armed by hand it refused the engine.
    from placement.provenance import declare_lever
    with declare_lever('place_optimize.py', sys.argv):
        import cli_banner; cli_banner.install()  # CMD/EXIT self-echo (run-3 B1)
        # main() already returns 0 from the --suggest-locks branch; that value was
        # dropped here, so the process exited 0 regardless of what main() decided
        # (the #551 family). Propagate it so a future refusal is visible to a caller.
        sys.exit(main() or 0)

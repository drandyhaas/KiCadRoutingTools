#!/usr/bin/env python3
"""Generate an initial placement for an UNPLACED board from its floorplan intent.

Usage:
  python place_seed.py input.kicad_pcb output.kicad_pcb --intent floorplan.json

The placement stack refines and deliberately does not place from scratch
UNAIDED -- but a declared intent carries the constraints a from-scratch run
lacks: zones, edge bands, locks, decap rules. This tool turns that intent into
a legal starting placement (see placement/seeder.py for exactly what each
construct becomes), stamps the intent's must_lock refs `(locked yes)` into the
output, runs a quench polish over the free parts, and then GRADES its own
output against the same intent -- a seed that fails the intent it was built
from is a defect, not a result.

Different --seed values produce genuinely different legal seeds (packing order
and target jitter); the same seed reproduces byte for byte. Compose with
place_portfolio.py to diversify and rank what this emits.

Exit codes: 0 seeded and graded clean; 2 bad arguments; 3 the board cannot be
seeded (no Edge.Cuts outline -- the outline is spec-owned and will not be
invented -- or the board is already placed / carries copper); 4 the seed was
written but parts could not be seated or the intent grade has errors.
"""
import _path  # noqa: F401  (py_placer -> py_router/py_tools on sys.path)
import argparse
import json
import os
import sys


def main():
    import routing_defaults as defaults
    from placement.cli_gates import add_intent_arg

    p = argparse.ArgumentParser(
        description="Intent-driven initial placement for an unplaced board.",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog="""
Examples:
  python place_seed.py board.kicad_pcb seed.kicad_pcb --intent floorplan.json
  python place_seed.py board.kicad_pcb seed.kicad_pcb --intent fp.json --seed 3
""")
    p.add_argument("input_file", help="Input KiCad PCB (unplaced parts + outline)")
    p.add_argument("output_file", help="Output board with the seeded placement")
    add_intent_arg(p, required=True, extra=(
        "Here it is also the CONSTRUCTION source for the seed and the "
        "acceptance gate the emitted seed is graded against."))
    p.add_argument("--seed", type=int, default=0,
                   help="Packing-order/jitter seed; same seed reproduces byte "
                        "for byte (default: 0)")
    p.add_argument("--group-by", default="auto",
                   help="Block sources for resolving the intent's `group` "
                        "references (default: auto = kicad,sheet)")
    p.add_argument("--ignore-nets", nargs="+", default=None, metavar="NET",
                   help="Net patterns excluded from the polish's airwire "
                        "scoring (plane-routed rails)")
    p.add_argument("--clearance", type=float, default=None)
    p.add_argument("--board-edge-clearance", type=float, default=None)
    p.add_argument("--grid-step", type=float, default=defaults.GRID_STEP)
    p.add_argument("--max-displacement", type=float, default=3.0,
                   help="Polish displacement cap in mm (default: 3.0)")
    p.add_argument("--no-polish", action="store_true",
                   help="Skip the quench polish; emit the raw packed seed")
    p.add_argument("--corridor-weight", type=float, default=0.0, metavar="W",
                   help="Price the length each foreign airwire cuts through "
                        "the intent's health.bus_corridors during the polish, "
                        "at W per mm. 0 = off (default). See "
                        "place_portfolio.py --corridor-weight")
    p.add_argument("--force", action="store_true",
                   help="Re-seed a board that already looks placed. The "
                        "existing placement is DISCARDED; to explore around "
                        "it instead, use place_portfolio.py")
    p.add_argument("--anchors-first", action="store_true",
                   help="Seed the ANCHOR tier (pad-extent >= the P75 "
                        "threshold, the same tiering reconstruct uses) by "
                        "descending extent BEFORE any small part -- the "
                        "default queue is pin-count descending, which seeds "
                        "a large low-pin connector late, after the smalls "
                        "claimed its space. The smalls are parked as "
                        "non-obstacles either way (the existing exclude "
                        "mechanism); this changes only WHO goes first "
                        "(run-4 C)")
    p.add_argument("--evict-depth", type=int, default=0, choices=(0, 1, 2),
                   metavar="N",
                   help="Eviction rung (#630, #699). At every depth a part "
                        "with NO legal pose gets a census of its seated "
                        "neighbours (JSON_SUMMARY no_pose_blockers: how many "
                        "poses lifting each one would free). 0 (default) "
                        "moves nothing. 1 evicts the neighbour that frees "
                        "the most, seats the part, then re-seats the "
                        "neighbour (inside its own zone) with the part in "
                        "place; the trade is kept only if both seats are "
                        "legal against every seated part and the seated "
                        "board's overlap count did not rise, else both parts "
                        "are put back and the revert is recorded. 2 also "
                        "censuses PAIRS when no single lift frees a pose, "
                        "and trades the best pair under the same rule -- at "
                        "most 16 of the pairs its 8 candidates form, so it "
                        "costs nothing on a part a single lift already "
                        "solves. APPLIES TO --reseat TOO, and there it "
                        "relaxes that pass's contract: --reseat normally "
                        "holds every part outside its scope fixed, and a "
                        "depth >= 1 lets it trade one out. Those parts are "
                        "named in the JSON's `evicted` and in a NOTE, and "
                        "the pass additionally refuses any trade that raised "
                        "the board's stack count or overlap area. Parts "
                        "locked in the file or by the intent's must_lock, "
                        "and declared edge connectors, are never evicted; a "
                        "blocker's own blocker is not chased, at either "
                        "depth, and there is one trade per part. Only fires "
                        "on a part that was going to be reported unseated. "
                        "Opt-in until an A/B row on three boards exists")
    p.add_argument("--anchor-rounds", type=int, default=1,
                   help="With --anchors-first: gated re-seat passes after "
                        "the first full placement (default 1 = none). Each "
                        "round re-seats anchors then smalls at their partner "
                        "centroids over the FULL placement and keeps the "
                        "round only if the legality/hpwl gate tuple does not "
                        "worsen; stops early when a round moves nothing")
    p.add_argument("--repair", action="store_true",
                   help="Violation-driven minimal-move repair of a PLACED "
                        "board: only parts violating the intent or pad/hole "
                        "legality move, worst first, each seated nearest its "
                        "current pose with an escalating displacement cap. "
                        "The opposite contract of --force")
    p.add_argument("--reseat", nargs="*", default=None, metavar="REF",
                   help="LIFT the named parts and re-seat them FROM SCRATCH "
                        "at their net centroids, holding every other part "
                        "fixed as an obstacle. With no REF the scope is the "
                        "off-outline pad-CENTRE census "
                        "(reconstruct.damage_witnesses), which is zero on all "
                        "33 corpus boards -- so a bare --reseat on a healthy "
                        "board is a no-op that exits 0. Unlike --repair, the "
                        "part's CURRENT POSE IS NOT THE SEARCH CENTRE: a part "
                        "30mm from where it belongs carries no information "
                        "about where it belongs, and --repair's cap ladder "
                        "tops out at 5mm from the wrong centre. REF accepts "
                        "fnmatch globs. Any edge_connector declaration on a "
                        "scope ref is DROPPED (the band was measured off the "
                        "pose being discarded). Composes with --repair and "
                        "runs BEFORE it. Judge it on witnesses_after, not on "
                        "how far anything moved")
    p.add_argument("--reseat-region", nargs=4, type=float, action="append",
                   default=None, metavar=("X0", "Y0", "X1", "Y1"),
                   help="Name the reseat scope by GEOMETRY instead of by ref: "
                        "every part with a pad in the rectangle X0 Y0 X1 Y1 "
                        "(board mm, half-open on X1/Y1) joins the scope. "
                        "Repeatable; unions with any --reseat REF. This is "
                        "#459's 'a way to name a REGION rather than a ref "
                        "list', whose other half -- a gate that can accept an "
                        "on-board part -- landed as #698. "
                        "`check_pockets` prints a ready-made rectangle for "
                        "its top cold region. IT IS SCOPE NAMING, NOT AIMING: "
                        "a re-seat lands each part at its own NET CENTROID, "
                        "so passing an empty region does not move anything "
                        "INTO it. Declaring the rectangle as an intent block "
                        "`zone` is what makes it a destination. Resolves "
                        "through placement.utility.refs_in_rect, the same "
                        "call check_pockets names its windows with, so the "
                        "rectangle cannot mean two different sets of parts")
    p.add_argument("--reseat-min-gain", type=float, default=0.0, metavar="MM",
                   help="With --reseat REF (an EXPLICIT scope): the smallest "
                        "wirelength win, in mm, that counts as a re-seat. 0 "
                        "(the default) means any strict win. It gates the "
                        "MILLIMETRE basis only -- the scope's own HPWL, which "
                        "is where a sideways shuffle can score. The "
                        "intent-violation, blocking-pair and stack bases are "
                        "COUNTS, whose smallest meaningful gain is one whole "
                        "defect: one number compared against both currencies "
                        "would be asserting an exchange rate between half a "
                        "millimetre of wire and half a keep-out violation. "
                        "Inert on the AUTO scope (bare --reseat), whose rule "
                        "is unchanged. JSON_SUMMARY accept_basis reports every "
                        "basis, whether this applied to it, and by how much "
                        "each one missed")
    p.add_argument("--dry-run", action="store_true",
                   help="With --repair/--reseat: print the move list and "
                        "grades, write nothing")
    args = p.parse_args()

    # Run-7 S1 / run-13 F6: unset floors come from the BOARD, not a constant.
    # A fixed 0.25 on a board declaring 0.2 measured 34% more shortfall and
    # double the oob count -- and these tools VETO candidate moves on it, so a
    # wrong floor steers the search, it does not merely mis-report.
    from list_nets import board_floor_knobs
    args.clearance, args.board_edge_clearance, _knobs = board_floor_knobs(
        args.input_file, args.clearance, args.board_edge_clearance)
    print(f"legality at clearance {args.clearance} "
          f"({_knobs['clearance']['source']}), edge {args.board_edge_clearance} "
          f"({_knobs['board_edge_clearance']['source']})")
    # #459: a region is an EXPLICIT scope that stands on its own, so it turns
    # --reseat on without the flag being typed. Normalised before every check
    # below, so `--reseat-region ... --dry-run` is not rejected by a rule that
    # only knows about --reseat.
    if args.reseat_region:
        for _r in args.reseat_region:
            if _r[2] <= _r[0] or _r[3] <= _r[1]:
                p.error("--reseat-region takes X0 Y0 X1 Y1 with X1>X0 and "
                        "Y1>Y0 (board mm); got %g %g %g %g" % tuple(_r))
        if args.reseat == []:
            # #698 gave the two scopes DIFFERENT acceptance policies: an
            # explicit scope is graded on its own terms, the AUTO damage-
            # witness scope on 'the off-board amount strictly improved'.
            # Silently picking one of the two is how that distinction gets
            # lost, so a bare --reseat beside a region is a usage error rather
            # than a quiet merge.
            p.error("--reseat-region is an EXPLICIT scope and a bare --reseat "
                    "is the AUTO damage-witness scope; #698 grades them by "
                    "different rules, so they do not combine. Use "
                    "--reseat-region on its own (it implies an explicit "
                    "--reseat), or name refs: --reseat REF ... "
                    "--reseat-region X0 Y0 X1 Y1")
        if args.reseat is None:
            args.reseat = []          # explicit; the refs come from the rects
    if (args.repair or args.reseat is not None) and args.force:
        p.error("--repair/--reseat and --force are mutually exclusive (they "
                "move only the parts that need it; force re-derives "
                "everything)")
    if args.dry_run and not (args.repair or args.reseat is not None):
        p.error("--dry-run only applies to --repair / --reseat")
    if args.reseat_min_gain and args.reseat is None:
        p.error("--reseat-min-gain only applies to --reseat")
    if args.reseat_min_gain < 0:
        p.error("--reseat-min-gain is a magnitude in mm; negative is not a "
                "looser threshold, it is a typo")
    if args.reseat_min_gain and args.reseat == [] and not args.reseat_region:
        # An inert knob says so, rather than reading as a threshold that
        # happened to find nothing wrong (cli_gates' own disclosure rule).
        print("--reseat-min-gain is inert on the AUTO scope: that rule is "
              "'the off-board amount strictly improved', which this does not "
              "gate", file=sys.stderr)

    try:
        from redo_record import record_invocation
        record_invocation()
    except Exception:
        pass

    import random
    from kicad_parser import parse_kicad_pcb
    from placement import floorplan, seeder
    from placement.groups import GroupError, parse_sources
    from placement.placement_state import UNPLACED_EXIT, assess_placement
    from placement.portfolio import copy_siblings
    from placement.writer import write_placed_output

    try:
        sources = parse_sources(args.group_by)
    except GroupError as exc:
        p.error(str(exc))
    try:
        intent = floorplan.load_intent(args.intent)
    except (OSError, ValueError) as exc:
        print(f"cannot load intent {args.intent}: {exc}", file=sys.stderr)
        return 2

    print(f"Loading {args.input_file}...")
    pcb = parse_kicad_pcb(args.input_file)
    if pcb.board_info.board_bounds is None:
        print("place_seed: this board has no Edge.Cuts outline. The outline "
              "is spec-owned -- draw it (or have the repo's seeder write it "
              "from the spec) before seeding a placement.", file=sys.stderr)
        return UNPLACED_EXIT
    st = assess_placement(pcb, args.input_file)
    if st.has_copper:
        print(f"place_seed: this board carries {st.segments} segment(s) and "
              f"{st.vias} via(s); seeding moves footprints and would strand "
              f"every track. Seed the unrouted board.", file=sys.stderr)
        return UNPLACED_EXIT
    if args.repair or args.reseat is not None:
        import math as _math
        import tempfile
        if st.unplaced:
            print("place_seed: --repair/--reseat need a PLACED board (this "
                  "one is unplaced -- seed it instead).", file=sys.stderr)
            return UNPLACED_EXIT
        summary = {'dry_run': args.dry_run,
                   'output': None if args.dry_run else args.output_file}

        # Both passes stage into a temp dir and the finished board is copied
        # to the output path once, at the end. That keeps --dry-run honest
        # (--reseat then --repair previews the repair against the RE-SEATED
        # board, which is what the real run would do) and keeps the output
        # path from ever holding a half-finished result (run-7 A11).
        _stage = tempfile.TemporaryDirectory()
        cur, cur_pcb = args.input_file, pcb
        exit_rc = 0

        def _advance(moves, tag):
            """Apply `moves` onto a fresh staged board; advances cur/cur_pcb."""
            nonlocal cur, cur_pcb
            if not moves:
                return
            nxt = os.path.join(_stage.name, f'{tag}.kicad_pcb')
            write_placed_output(cur, nxt, moves)
            copy_siblings(cur, nxt)
            cur = nxt
            cur_pcb = parse_kicad_pcb(cur)

        reseat = None
        if args.reseat is not None:
            # `nargs='*'`: bare --reseat is [] and means AUTO scope; None is
            # the flag being absent.
            _refs = list(args.reseat or ())
            _selector = None
            if args.reseat_region:
                # #459's "a way to name a REGION rather than a ref list".
                # Resolved HERE, at the CLI layer, into the ordinary `refs`
                # argument -- never by inventing a new scope_source. The
                # acceptance policy in seeder.reseat_accept switches on
                # `scope_source != 'explicit'` by string equality, so a
                # 'region:...' source would drop every region re-seat into the
                # auto:oob-strict policy, which a legal on-board part can
                # never satisfy: the pass would refuse everything and look
                # broken rather than wrong.
                #
                # refs_in_rect is the same call check_pockets names its own
                # windows with, so the rectangle it prints and the rectangle
                # this lifts cannot mean two different sets of parts.
                from placement.utility import refs_in_rect
                _hits = []
                for _r in args.reseat_region:
                    _in = refs_in_rect(cur_pcb, tuple(_r))
                    print("  region [%g,%g]-[%g,%g]: %d part(s)%s"
                          % (_r[0], _r[1], _r[2], _r[3], len(_in),
                             ('  ' + ', '.join(_in[:12])
                              + (' ...' if len(_in) > 12 else ''))
                             if _in else ''))
                    _hits.extend(_in)
                _region_refs = sorted(set(_hits))
                if not _region_refs:
                    # A result, not a failure -- and it must not fall through
                    # to `refs=None`, which is the AUTO scope under a
                    # different acceptance rule entirely.
                    print("Reseat (explicit): the region(s) contain no part; "
                          "nothing to lift. This is a RESULT, not a failure -- "
                          "an empty region is exactly what check_pockets ranks."
                          )
                    return 0
                _refs.extend(_region_refs)
                _selector = 'region:' + ';'.join(
                    '%g,%g,%g,%g' % tuple(_r) for _r in args.reseat_region)
                _refs = sorted(set(_refs))
            reseat = seeder.reseat_scope(
                cur_pcb, cur, intent,
                refs=(_refs or None), group_sources=sources,
                clearance=args.clearance,
                board_edge_clearance=args.board_edge_clearance,
                grid_step=args.grid_step, seed=args.seed,
                # The same flag, not a second one: it was parsed and
                # silently ignored on this path (#699).
                evict_depth=args.evict_depth,
                min_gain=args.reseat_min_gain)
            for note in reseat['notes']:
                print(f"  NOTE: {note}")
            # Over the SCOPE only: the line prints it as "{n} re-seated
            # (max X mm)", and `n` deliberately excludes the parts the
            # eviction rung moved, so measuring over both mixes the two
            # counts the engine went to the trouble of separating.
            _scope = set(reseat['scope'])
            _rmax = 0.0
            for mv in reseat['moves']:
                if mv['reference'] not in _scope:
                    continue
                fp = cur_pcb.footprints.get(mv['reference'])
                if fp is not None:
                    _rmax = max(_rmax, _math.hypot(mv['new_x'] - fp.x,
                                                  mv['new_y'] - fp.y))
            if _selector:
                # A SEPARATE key. Overloading scope_source is what would have
                # broken the acceptance policy switch, so the provenance of
                # the scope is reported beside it rather than inside it.
                print(f"  scope_selector: {_selector} "
                      f"({len(_region_refs)} from the region, "
                      f"{len(args.reseat or ())} named)")
            print(f"Reseat ({reseat['scope_source']}): "
                  f"{len(reseat['scope'])} in scope, "
                  f"{len(reseat['reseated'])} re-seated "
                  f"(max {_rmax:.2f}mm), {len(reseat['unseated'])} unseated, "
                  f"{len(reseat['refused'])} refused, "
                  f"{len(reseat.get('evicted') or [])} evicted; "
                  f"OFF-OUTLINE PARTS {len(reseat['witnesses_before'])} -> "
                  f"{len(reseat['witnesses_after'])}"
                  + ('' if reseat['accepted'] else '  [GATE REFUSED]'))
            print(f"  gate {reseat['gate_before']} -> {reseat['gate_after']}")
            # #698: WHICH scope-relevant term carried the pass, or -- on a
            # refusal -- what every basis measured. A verdict with no basis is
            # how this pass came to refuse on a term the operator never asked
            # about for a whole release.
            _ab = reseat.get('accept_basis') or {}
            if _ab.get('fired'):
                _t = next((t for t in _ab.get('terms') or []
                           if t['term'] == _ab['fired']), {})
                print(f"  accepted on {_ab['fired']}: "
                      f"{_t.get('before')} -> {_t.get('after')} "
                      f"({_t.get('units')}); {_ab.get('policy')}")
            elif _ab.get('policy') == 'explicit:one-term-strict':
                # "would have fired", never "no basis fired": on a safety or
                # licence refusal the top-level `fired` is None by design while
                # a term still carries `first`, and printing "no basis fired"
                # then contradicts the record right beside it.
                _first = next((t['term'] for t in (_ab.get('terms') or [])
                               if t.get('first')), None)
                print(("  refused despite " + _first + " improving: "
                       if _first else "  no basis improved: ") + ", ".join(
                    f"{t['term']} {t['before']}->{t['after']}"
                    for t in (_ab.get('terms') or [])))
            summary.update({
                'reseat': True,
                'scope': reseat['scope'],
                'scope_source': reseat['scope_source'],
                # #459: how the scope was NAMED, beside (never inside) what
                # policy graded it. None when refs were typed directly.
                'scope_selector': _selector,
                'reseated': len(reseat['reseated']),
                'reseated_refs': reseat['reseated'],
                'unseated': reseat['unseated'],
                'no_pose_blockers': reseat.get('no_pose_blockers') or {},
                'no_pose_verdict': reseat.get('no_pose_verdict') or {},
                # Parts moved OUTSIDE the declared scope by the eviction
                # rung. Empty at depth 0, which is the default.
                'evicted': reseat.get('evicted') or [],
                'no_pose_census': reseat.get('no_pose_census') or {},
                'evictions': reseat.get('evictions', 0),
                'evictions_reverted': reseat.get('evictions_reverted', 0),
                'refused': reseat['refused'],
                'edge_bands_dropped': reseat['edge_bands_dropped'],
                # THE load-bearing number: it is the one that predicts
                # routability. `reseated` counts moves, which is effort.
                'witnesses_before': len(reseat['witnesses_before']),
                'witnesses_after': len(reseat['witnesses_after']),
                'witnesses_after_refs': reseat['witnesses_after'],
                'gate_before': reseat['gate_before'],
                'gate_after': reseat['gate_after'],
                'accepted': reseat['accepted'],
                # #698: the whole basis record, not just the winner -- a basis
                # that measured nothing and a basis that measured no change
                # must not look alike to a reader of the summary either.
                'accept_basis': reseat.get('accept_basis'),
                'reseat_min_gain': args.reseat_min_gain,
                'reseat_max_move_mm': round(_rmax, 3),
            })
            _advance(reseat['moves'], 'reseat')
            if reseat['edge_bands_dropped']:
                # Grade against what the pass actually honoured. Keeping the
                # dropped declarations would charge the repair for repairing:
                # a part brought home from 160mm out then reads "sits nearest
                # the west edge but is declared on the east edge".
                intent = reseat['intent_used']
                print(f"  (final grade excludes the "
                      f"{len(reseat['edge_bands_dropped'])} dropped edge "
                      f"declaration(s): a band measured off a discarded pose "
                      f"is not a spec to grade the homecoming against)")
            if reseat['unseated'] or reseat['refused'] \
                    or not reseat['accepted']:
                exit_rc = 4

        result = None
        if args.repair:
            result = seeder.repair_placement(
                cur_pcb, cur, intent, group_sources=sources,
                clearance=args.clearance,
                board_edge_clearance=args.board_edge_clearance,
                grid_step=args.grid_step)
            for note in result['notes']:
                print(f"  NOTE: {note}")
            max_move = 0.0
            for mv in result['moves']:
                fp = cur_pcb.footprints.get(mv['reference'])
                if fp is not None:
                    max_move = max(max_move, _math.hypot(mv['new_x'] - fp.x,
                                                         mv['new_y'] - fp.y))
            print(f"Repair: {len(result['violators'])} violator(s), "
                  f"{len(result['repaired'])} repaired "
                  f"({len(result['moves'])} moved, max {max_move:.2f}mm), "
                  f"{len(result.get('unresolved') or [])} unresolved, "
                  f"{len(result['unrepairable'])} unrepairable")
            summary.update({
                'repaired': len(result['repaired']),
                'unrepairable': len(result['unrepairable']),
                'moved_refs': [m['reference'] for m in result['moves']],
                'max_move_mm': round(max_move, 3),
            })
            summary.update({f'{k}_before': v
                            for k, v in result['pad_report_before'].items()})
            _advance(result['moves'], 'repair')
            if result['unrepairable']:
                exit_rc = 4

        if not args.dry_run:
            # A no-op still writes a board: the next step in a chain is handed
            # a path, and "nothing needed doing" must not look like "the tool
            # produced nothing".
            write_placed_output(cur, args.output_file, [])
            copy_siblings(cur, args.output_file)
            from placement.legality import grade_pad_legality
            pcb_out = parse_kicad_pcb(args.output_file)
            pads_after = grade_pad_legality(pcb_out, args.clearance,
                                            pcb_file=args.output_file)
            graded = floorplan.grade(intent, pcb_out, args.output_file,
                                     group_sources=sources,
                                     clearance=args.clearance,
                                     board_edge_clearance=args.board_edge_clearance)
            for v in graded.errors[:10]:
                print(f"  GRADE ERROR [{v.rule}] {v.message}")
            summary['grade_errors'] = len(graded.errors)
            summary['pad_conflicts_after'] = pads_after['pad_conflicts']
            # #697: the requirement each counted pair was graded at, when it
            # sits above args.clearance, so the count is explainable.
            summary['pad_clearance_required'] = pads_after.get('required') or []
            from placement.legality import format_required_clause as _req_cl
            if _req_cl(pads_after):
                print(f"  above the {args.clearance}mm floor: "
                      f"{_req_cl(pads_after)}")
            summary['hole_conflicts_after'] = pads_after['hole_conflicts']
            summary['oob_pad_count_after'] = pads_after['oob_pad_count']
            if graded.errors:
                exit_rc = 4
        _stage.cleanup()
        summary.setdefault('complete', True)
        summary.setdefault('status', 'ok')
        print('JSON_SUMMARY: ' + json.dumps(summary, sort_keys=True,
                                            default=str), flush=True)
        return exit_rc

    if not st.unplaced and not st.partially_unplaced and not args.force:
        # PARTIALLY unplaced boards (a stacked pile beside real placements --
        # a netlist re-import, or a seeder that pinned only the spec-fixed
        # parts) are a legitimate seeding target, not a refusal: locked parts
        # are treated as authoritative and the pile is what gets placed.
        print("place_seed: this board already looks PLACED. Seeding would "
              "discard that placement; use place_portfolio.py to explore "
              "variations of it, or --force to re-seed anyway.",
              file=sys.stderr)
        return UNPLACED_EXIT

    # Partially unplaced without --force: seed ONLY the stacked pile. The
    # genuinely-placed unlocked parts are someone's work, not this tool's to
    # re-derive; --force widens the scope back to everything unlocked.
    seed_refs = None
    if st.partially_unplaced and not st.unplaced and not args.force:
        # SCOPE FOLLOWS THE GATE. `partially_unplaced` is now decided on the
        # SUSPECT subset -- co-located parts that are not markers and not on
        # opposite sides -- so scoping the seed from the full `stacked_refs`
        # would re-seed the very parts the gate had just exonerated. A
        # front/back fiducial pair shares a coordinate BY DESIGN; a run that
        # tripped over one genuine pile would have moved every such pair on the
        # board as a side effect, and nothing downstream would attribute it.
        seed_refs = set(st.stacked_suspect_refs)
        _benign = len(set(st.stacked_refs) - seed_refs)
        print(f"place_seed: partially unplaced -- seeding only the "
              f"{len(seed_refs)} stacked part(s) that look like a pile; the "
              f"rest stand as placed (--force re-seeds everything unlocked)"
              + (f". {_benign} other co-located part(s) are left alone "
                 f"(markers, or opposite sides of the board)" if _benign else ""))
        # ...unless every one of them is (locked yes) IN THE FILE, in which
        # case `seed_from_intent` treats them as authoritatively placed
        # (seeder.py:396-400) and THE PILE CANNOT BE SEEDED AT ALL. Not exotic:
        # this toolchain STAMPS its own locks (seeder.stamp_locked, on
        # place_seed's own output), so a pile created by an earlier seeding run
        # arrives here locked and gets announced as the scope of a run that
        # then cannot touch it.
        #
        # Care with the claim: this is NOT "the run does nothing". The polish
        # pass is on by default and moves plenty -- measured on a 65-part
        # fixture, 41 parts moved while all three piled refs stayed put. That
        # is exactly why refusing is right rather than pedantic: continuing
        # would exit 0 having rearranged two thirds of the board and left the
        # one thing this branch exists to fix untouched.
        _movable = [r for r in sorted(seed_refs)
                    if not getattr(pcb.footprints.get(r), 'locked', False)]
        if not _movable:
            print(f"place_seed: all {len(seed_refs)} of those part(s) are "
                  f"(locked yes) in the file, so the pile CANNOT be seeded -- "
                  f"seed_from_intent treats a file-locked ref as already "
                  f"placed. Continuing would still move other parts (the "
                  f"polish pass is on by default) and exit 0 with the pile "
                  f"exactly as it is, so this refuses instead. Unlock those "
                  f"refs to seed them; --force re-seeds the WHOLE board and "
                  f"discards the existing placement; place_optimize.py is the "
                  f"tool if polish was all you wanted.", file=sys.stderr)
            return UNPLACED_EXIT

    rng = random.Random(f"{args.seed}")
    result = seeder.seed_from_intent(
        pcb, args.input_file, intent, rng, group_sources=sources,
        clearance=args.clearance,
        board_edge_clearance=args.board_edge_clearance,
        grid_step=args.grid_step, seed_refs=seed_refs,
        anchors_first=args.anchors_first,
        anchor_rounds=args.anchor_rounds,
        evict_depth=args.evict_depth)
    for note in result['notes']:
        print(f"  NOTE: {note}")
    print(f"Seeded {len(result['placements'])} part(s); "
          f"{len(result['unseated'])} unseated; "
          f"{len(result['lock_refs'])} to lock")

    write_placed_output(args.input_file, args.output_file,
                        result['placements'])
    n_locked = seeder.stamp_locked(args.output_file, result['lock_refs'])
    copy_siblings(args.input_file, args.output_file)
    print(f"Stamped (locked yes) on {n_locked} part(s)")

    ratsnest = {}
    if not args.no_polish:
        from placement.quench import quench
        pcb_seeded = parse_kicad_pcb(args.output_file)
        # Guidance weights, same as place_portfolio: the seed should be
        # polished by the objective the later steps rank with. Locks ride in
        # from the file (must_lock was just stamped); edge connectors are
        # locked per-call so the polish cannot walk them off their band.
        # #702: edge_claims(), must_lock and the declared zones all now
        # ride in through `intent_gate`, resolved by the ONE function
        # every quenching CLI uses. The edge_claims()-not-edge_connectors
        # filter that used to live here moved with it -- see
        # `floorplan.resolve_intent_gate`, which is the point: a filter
        # that must be remembered at each call site is one that will be
        # forgotten at the next.
        from placement.cli_gates import resolve_intent_gate_for_cli
        _gate, _ = resolve_intent_gate_for_cli(
            intent, pcb_seeded, sources, args.intent)
        placements = quench(
            pcb_seeded, pcb_file=args.output_file,
            max_displacement=args.max_displacement,
            step=1.0, grid_step=args.grid_step, clearance=args.clearance,
            board_edge_clearance=args.board_edge_clearance,
            crossing_penalty=30.0, length_weight=0.3, halo_base=0.5,
            halo_coef=0.15, halo_weight=2.0, edge_halo=2.0, edge_weight=2.0,
            ignore_nets=args.ignore_nets,
            metrics_out=ratsnest, intent_gate=_gate,
            corridor_weight=args.corridor_weight,
            corridor_specs=list((intent.health or {}).get('bus_corridors')
                                or ()) or None)
        if placements:
            tmp = args.output_file + '.polish'
            write_placed_output(args.output_file, tmp, placements)
            os.replace(tmp, args.output_file)

    # ---- self-check: the seed must grade clean against its own intent ------
    def _grade():
        pcb_out = parse_kicad_pcb(args.output_file)
        return floorplan.grade(intent, pcb_out, args.output_file,
                               group_sources=sources,
                               clearance=args.clearance,
                               board_edge_clearance=args.board_edge_clearance)

    try:
        graded = _grade()
        # The quench has no zone term, so a polish nudge can walk a declared
        # zone member past its tolerance (measured: a crystal load cap,
        # 0.86mm past its zone's edge at one seed). A plain revert to the
        # seeded pose is not enough -- the polish moved NEIGHBORS into that
        # space too (measured: 0.784mm2 of new overlap) -- so the part is
        # RE-SEATED: the seeder's own search, targeted at its seeded pose,
        # constrained to its zone, against the post-polish board.
        # #701: and it has no keep-out term either, so the same nudge can walk
        # a part into a declared keep-out -- on a board this tool's own seeder
        # placed correctly, which then exits 4 against its own intent. Same
        # repair, same reason, one more rule name.
        if not args.no_polish:
            _repairable = ('zone_containment', 'keepout')
            broke = sorted({v.ref for v in graded.errors
                            if v.rule in _repairable and v.ref})
            _rules = sorted({v.rule for v in graded.errors
                             if v.rule in _repairable and v.ref})
            if broke:
                import pose_score
                pcb_cur = parse_kicad_pcb(args.output_file)
                st = pose_score.make_state(
                    pcb_cur, args.output_file, clearance=args.clearance,
                    board_edge_clearance=args.board_edge_clearance,
                    grid_step=args.grid_step,
                    # #701: the ONLY seat-predicate call site outside
                    # seeder.py. Without this the re-seat below would be free
                    # to put the part back into a declared keep-out while
                    # fixing its zone.
                    keepouts=intent.keepouts)
                blocks2, _p = floorplan.resolve_blocks(intent, pcb_cur,
                                                       sources)
                zone_of = {}
                for z in intent.blocks:
                    if z.rect is None:
                        continue
                    for r in blocks2.get(z.name, ()):
                        zone_of.setdefault(r, z)
                seeded_pose = {p['reference']: p for p in result['placements']}
                fixes = []
                for ref in broke:
                    z = zone_of.get(ref)
                    sp = seeded_pose.get(ref)
                    # #701: a keep-out violation does NOT imply a zone. The
                    # zone-only version required one and skipped a zone-less
                    # part entirely, which would have left the keep-out half
                    # of this repair silently inert on most boards.
                    if sp is None or ref not in st.parts:
                        continue
                    clr = seeder._try_place(
                        st, ref, sp['new_x'], sp['new_y'], set(),
                        constraint=z.rect if z is not None else None,
                        tol=intent.zone_tolerance(z) if z is not None else 0.5)
                    if clr is not None:
                        p2 = st.parts[ref]
                        fixes.append({'reference': ref, 'new_x': p2.x,
                                      'new_y': p2.y, 'new_rotation': p2.rot})
                if fixes:
                    print(f"  polish walked "
                          f"{', '.join(f['reference'] for f in fixes)} out of "
                          f"a declared {' / '.join(_rules)}; re-seated "
                          f"against the polished board")
                    tmp = args.output_file + '.reseat'
                    write_placed_output(args.output_file, tmp, fixes)
                    os.replace(tmp, args.output_file)
                    graded = _grade()
    except floorplan.UntrustworthyOutline as exc:
        print(f"place_seed: outline cannot be trusted for grading: {exc}",
              file=sys.stderr)
        return UNPLACED_EXIT
    for v in graded.errors[:10]:
        print(f"  GRADE ERROR [{v.rule}] {v.message}")
    after = ratsnest.get('after', {})
    summary = {'placed': len(result['placements']),
               'unseated': len(result['unseated']),
               # NAMES, not just a count. #629's complaint is that a verdict
               # you cannot act on is a dead end, and a count names nobody.
               'unseated_refs': list(result['unseated']),
               'no_pose_blockers': result.get('no_pose_blockers') or {},
               # WHY each of them has no pose, not just who is nearby (#699).
               # "nothing is near it" and "everything near it is locked" were
               # the same empty dict, and they need different answers.
               'no_pose_verdict': result.get('no_pose_verdict') or {},
               'no_pose_census': result.get('no_pose_census') or {},
               # Trades KEPT, and trades REVERTED, separately: a reader who
               # sees `evictions: 1` must not have to guess whether the board
               # changed. The records themselves are in the NOTE lines.
               'evictions': sum(1 for e in (result.get('evictions') or [])
                                if e.get('accepted')),
               'evictions_reverted': sum(
                   1 for e in (result.get('evictions') or [])
                   if not e.get('accepted')),
               'locked': n_locked,
               'grade_errors': len(graded.errors),
               'grade_warnings': len(graded.warnings),
               'crossings': after.get('crossings'),
               'hpwl': (round(after['hpwl'], 3)
                        if after.get('hpwl') is not None else None),
               'output': args.output_file}
    print("JSON_SUMMARY: " + json.dumps(summary, sort_keys=True))
    if result['unseated'] or graded.errors:
        print("place_seed: the seed does NOT satisfy its intent -- see the "
              "errors above. It was still written, for inspection.",
              file=sys.stderr)
        return 4
    return 0


if __name__ == "__main__":
    # Declare the lever for the WHOLE run, so every pose this CLI writes
    # carries its name in the provenance record -- place_reconstruct.py
    # does the same. Without it a seeded board carries no lever at all, and
    # the provenance instrument is silent about where its poses came from.
    from placement.provenance import declare_lever
    with declare_lever('place_seed.py', sys.argv):
        import cli_banner; cli_banner.install()  # CMD/EXIT self-echo (run-3 B1)
        sys.exit(main())

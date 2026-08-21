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

    p = argparse.ArgumentParser(
        description="Intent-driven initial placement for an unplaced board.",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog="""
Examples:
  python place_seed.py board.kicad_pcb seed.kicad_pcb --intent floorplan.json
  python place_seed.py board.kicad_pcb seed.kicad_pcb --intent fp.json --seed 3
""")
    p.add_argument("input_file", help="Input KiCad PCB (unplaced parts + outline)")
    p.add_argument("output_file", help="Output board with the seeded placement")
    p.add_argument("--intent", required=True, metavar="JSON",
                   help="Floorplan intent: the constraint source AND the "
                        "acceptance gate for the emitted seed")
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
    p.add_argument("--evict-depth", type=int, default=1, metavar="N",
                   help="When a part has NO legal pose, census the seated "
                        "neighbours, evict the one that frees the most poses, "
                        "seat the part, then put the blocker back with it in "
                        "place -- gated by the same reconstruct measure the "
                        "anchor rounds use, and reverted whole if it does not "
                        "improve (#630). 0 disables it and restores the bare "
                        "'no legal pose' verdict. Only ever fires on a part "
                        "that was already going to be reported unseated, so "
                        "a run that seats everything is unaffected. Depth is "
                        "1: a blocker's own blocker is not chased")
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
    p.add_argument("--deadline", type=float, default=None, metavar="SECONDS",
                   help="Wall-clock budget for --repair/--reseat. The repair "
                        "sweep has no internal bound (violators x caps x 36 "
                        "ring sweeps x O(parts)); with a budget it stops "
                        "between violators, keeps the seats it made, reports "
                        "the rest in deadline_skipped and exits 7. Default: "
                        "no budget (a wall-clock default would break replay "
                        "determinism). ANY harness with an external timeout "
                        "should pass this at ~0.8x its own -- on Windows an "
                        "external kill is TerminateProcess and leaves NO "
                        "output at all. Env: KRT_DEADLINE_S")
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
    if (args.repair or args.reseat is not None) and args.force:
        p.error("--repair/--reseat and --force are mutually exclusive (they "
                "move only the parts that need it; force re-derives "
                "everything)")
    if args.dry_run and not (args.repair or args.reseat is not None):
        p.error("--dry-run only applies to --repair / --reseat")

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
        import krt_deadline
        if st.unplaced:
            print("place_seed: --repair/--reseat need a PLACED board (this "
                  "one is unplaced -- seed it instead).", file=sys.stderr)
            return UNPLACED_EXIT
        summary = {'dry_run': args.dry_run,
                   'output': None if args.dry_run else args.output_file}
        # Armed BEFORE the passes, so the flush hook covers every return
        # below -- the refusals above print no summary at all today, which is
        # how a killed run and a refused one look identical to a scraper.
        _dl = krt_deadline.arm(args.deadline, tool='place_seed',
                               on_partial=lambda: summary)
        _prog = krt_deadline.stdout_progress(deadline=_dl)

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
            reseat = seeder.reseat_scope(
                cur_pcb, cur, intent,
                refs=(args.reseat or None), group_sources=sources,
                clearance=args.clearance,
                board_edge_clearance=args.board_edge_clearance,
                grid_step=args.grid_step, seed=args.seed,
                deadline=_dl, progress=_prog)
            for note in reseat['notes']:
                print(f"  NOTE: {note}")
            _rmax = 0.0
            for mv in reseat['moves']:
                fp = cur_pcb.footprints.get(mv['reference'])
                if fp is not None:
                    _rmax = max(_rmax, _math.hypot(mv['new_x'] - fp.x,
                                                  mv['new_y'] - fp.y))
            print(f"Reseat ({reseat['scope_source']}): "
                  f"{len(reseat['scope'])} in scope, "
                  f"{len(reseat['reseated'])} re-seated "
                  f"(max {_rmax:.2f}mm), {len(reseat['unseated'])} unseated, "
                  f"{len(reseat['refused'])} refused; "
                  f"OFF-OUTLINE PARTS {len(reseat['witnesses_before'])} -> "
                  f"{len(reseat['witnesses_after'])}"
                  + ('' if reseat['accepted'] else '  [GATE REFUSED]'))
            print(f"  gate {reseat['gate_before']} -> {reseat['gate_after']}")
            summary.update({
                'reseat': True,
                'scope': reseat['scope'],
                'scope_source': reseat['scope_source'],
                'reseated': len(reseat['reseated']),
                'reseated_refs': reseat['reseated'],
                'unseated': reseat['unseated'],
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
                grid_step=args.grid_step, deadline=_dl, progress=_prog)
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
                'deadline_skipped': result.get('deadline_skipped') or [],
            })
            summary.update({f'{k}_before': v
                            for k, v in result['pad_report_before'].items()})
            _advance(result['moves'], 'repair')
            if result['unrepairable']:
                exit_rc = 4

        summary['complete'] = ((result or {}).get('complete', True)
                               and (reseat or {}).get('complete', True))
        if not summary['complete'] and _dl is not None:
            # Unlike place_reconstruct, the output IS written on expiry: the
            # partial is coherent by construction here (a part is either fully
            # seated, with every seat legality-checked before it was taken, or
            # untouched), so there is no pre-legalize staging step whose result
            # would be invalid to hand on.
            krt_deadline.mark(
                summary, _dl,
                reseat_skipped=(reseat or {}).get('deadline_skipped') or [],
                repair_skipped=(result or {}).get('deadline_skipped') or [],
                output=None if args.dry_run else args.output_file)
        if not args.dry_run:
            # A no-op still writes a board: the next step in a chain is handed
            # a path, and "nothing needed doing" must not look like "the tool
            # produced nothing".
            write_placed_output(cur, args.output_file, [])
            copy_siblings(cur, args.output_file)
            from placement.legality import grade_pad_legality
            pcb_out = parse_kicad_pcb(args.output_file)
            pads_after = grade_pad_legality(pcb_out, args.clearance)
            graded = floorplan.grade(intent, pcb_out, args.output_file,
                                     group_sources=sources,
                                     clearance=args.clearance,
                                     board_edge_clearance=args.board_edge_clearance)
            for v in graded.errors[:10]:
                print(f"  GRADE ERROR [{v.rule}] {v.message}")
            summary['grade_errors'] = len(graded.errors)
            summary['pad_conflicts_after'] = pads_after['pad_conflicts']
            summary['hole_conflicts_after'] = pads_after['hole_conflicts']
            summary['oob_pad_count_after'] = pads_after['oob_pad_count']
            if graded.errors:
                exit_rc = 4
        _stage.cleanup()
        krt_deadline.emit(summary, deadline=_dl)
        if not summary['complete']:
            return krt_deadline.DEADLINE_EXIT
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
    # A CLOCK ON THE PLAIN SEED PATH. `--deadline` was accepted here and
    # threaded only into the --repair/--reseat branch, so a plain seed ran
    # unbounded while its own help text promised otherwise -- measured: a
    # 2400s budget on an 85-part pile ran past 50 minutes without firing.
    # The eviction rung makes that worse, not better: it adds a census sweep
    # per unseated part to the one path with no budget at all.
    import krt_deadline
    _seed_dl = krt_deadline.arm(args.deadline, tool='place_seed')
    _seed_prog = krt_deadline.stdout_progress(deadline=_seed_dl)
    result = seeder.seed_from_intent(
        pcb, args.input_file, intent, rng, group_sources=sources,
        clearance=args.clearance,
        board_edge_clearance=args.board_edge_clearance,
        grid_step=args.grid_step, seed_refs=seed_refs,
        anchors_first=args.anchors_first,
        anchor_rounds=args.anchor_rounds,
        evict_depth=args.evict_depth,
        deadline=_seed_dl, progress=_seed_prog)
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
        edge_refs = [c['ref'] for c in intent.edge_connectors]
        placements = quench(
            pcb_seeded, pcb_file=args.output_file,
            max_displacement=args.max_displacement,
            step=1.0, grid_step=args.grid_step, clearance=args.clearance,
            board_edge_clearance=args.board_edge_clearance,
            crossing_penalty=30.0, length_weight=0.3, halo_base=0.5,
            halo_coef=0.15, halo_weight=2.0, edge_halo=2.0, edge_weight=2.0,
            ignore_nets=args.ignore_nets,
            lock_refs=edge_refs or None, metrics_out=ratsnest,
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
        if not args.no_polish:
            broke = sorted({v.ref for v in graded.errors
                            if v.rule == 'zone_containment' and v.ref})
            if broke:
                import pose_score
                pcb_cur = parse_kicad_pcb(args.output_file)
                st = pose_score.make_state(
                    pcb_cur, args.output_file, clearance=args.clearance,
                    board_edge_clearance=args.board_edge_clearance,
                    grid_step=args.grid_step)
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
                    if z is None or sp is None or ref not in st.parts:
                        continue
                    clr = seeder._try_place(
                        st, ref, sp['new_x'], sp['new_y'], set(),
                        constraint=z.rect, tol=intent.zone_tolerance(z))
                    if clr is not None:
                        p2 = st.parts[ref]
                        fixes.append({'reference': ref, 'new_x': p2.x,
                                      'new_y': p2.y, 'new_rotation': p2.rot})
                if fixes:
                    print(f"  polish walked "
                          f"{', '.join(f['reference'] for f in fixes)} out "
                          f"of a declared zone; re-seated in-zone against "
                          f"the polished board")
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
               'evictions': len(result.get('evictions') or []),
               'locked': n_locked,
               'grade_errors': len(graded.errors),
               'grade_warnings': len(graded.warnings),
               'crossings': after.get('crossings'),
               'hpwl': (round(after['hpwl'], 3)
                        if after.get('hpwl') is not None else None),
               'output': args.output_file}
    print("JSON_SUMMARY: " + json.dumps(summary, sort_keys=True))
    if not result.get('complete', True):
        # A budget that ran out is NOT a graded failure: the parts in
        # `deadline_skipped` were never tried, and reporting them as unseated
        # would be a measurement nobody made.
        print(f"place_seed: the deadline expired with "
              f"{len(result.get('deadline_skipped') or [])} part(s) never "
              f"tried -- they are in deadline_skipped, NOT unseated. The "
              f"partial seed was written.", file=sys.stderr)
        return krt_deadline.DEADLINE_EXIT
    if result['unseated'] or graded.errors:
        print("place_seed: the seed does NOT satisfy its intent -- see the "
              "errors above. It was still written, for inspection.",
              file=sys.stderr)
        return 4
    return 0


if __name__ == "__main__":
    # Declare the lever for the WHOLE run, so every pose this CLI
    # writes carries its name. Nothing called declare_lever outside
    # tests, so the unaided instrument had no armed state at all:
    # unarmed it is silent, and armed by hand it refused the engine.
    from placement.provenance import declare_lever
    with declare_lever('place_seed.py', sys.argv):
        import cli_banner; cli_banner.install()  # CMD/EXIT self-echo (run-3 B1)
        sys.exit(main())

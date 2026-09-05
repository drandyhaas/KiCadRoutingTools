#!/usr/bin/env python3
"""Reconstruct a PLACED-BUT-WRONG board: the puzzle solver.

Usage:
  python place_reconstruct.py damaged.kicad_pcb repaired.kicad_pcb [--intent fp.json]

For boards whose placement is structurally damaged (swapped regions, dragged
selections, re-imported netlists) -- the case the nudge-scale quench and the
from-scratch seeder both fail. Pipeline (placement/reconstruct.py):
classify tiers (locked/mechanical frame -> anchors -> smalls), corner-inset
pattern fit on mounting holes (propose-only), rigid +/-v vector detection, an
EXACT simultaneous candidate assignment (small ILP via scipy.optimize.milp;
breakout-descent fallback), then a violation-driven minimal-move legalize
sweep. Every stage applies only if the legality gate tuple (pad conflicts,
hole shortfall, banded pad-oob, stack pairs, hpwl, courtyard overlap) does
not worsen -- see placement/reconstruct.measure() for the ordering rationale.

Exit codes: 0 wrote a board that improved (or matched) every gate axis with
no residual pad conflicts; 3 the board cannot be reconstructed here (no
outline / carries copper); 4 residual violations remain (board still written
for inspection).
"""
import _path  # noqa: F401  (py_placer -> py_router/py_tools on sys.path)
import argparse
import json
import math
import os
import sys

# Sibling files a staged board carries with it (the .kicad_pro is the DRC floor
# a later step reads; #441).
from copy_board import SIBLING_EXTS as _SIBLING_EXTS  # ONE list (#711)


def _promote_staged(staged: str, final: str) -> None:
    """Move a completed staged board (and its siblings) onto the output path.

    One rename per file, after every stage that can still move copper or parts
    has finished -- so a killed run leaves the staging files behind rather than
    a half-finished board at the name the caller will hand to the next step.
    """
    staged_base = os.path.splitext(staged)[0]
    final_base = os.path.splitext(final)[0]
    os.replace(staged, final)
    for ext in _SIBLING_EXTS:
        src = staged_base + ext
        if os.path.isfile(src):
            os.replace(src, final_base + ext)


#: The stage names that actually gate code. `classify` runs unconditionally
#: (it is a prerequisite for everything below it, `reconstruct.classify` at the
#: top of the pipeline) and is listed here so the default string round-trips.
_STAGES = ('classify', 'fit', 'vector', 'assign', 'exchange', 'reseat',
           'legalize')


def main():
    import routing_defaults as defaults

    p = argparse.ArgumentParser(
        description="Structure-level placement reconstruction.",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog="""
Examples:
  python place_reconstruct.py damaged.kicad_pcb repaired.kicad_pcb
  python place_reconstruct.py damaged.kicad_pcb repaired.kicad_pcb \\
      --intent floorplan.json --dry-run
""")
    p.add_argument("input_file")
    p.add_argument("output_file")
    p.add_argument("--intent", default=None, metavar="JSON",
                   help="Optional floorplan intent (edge connectors exempt "
                        "from off-board repair; zones constrain re-seating)")
    p.add_argument("--stages",
                   default="classify,fit,vector,assign,exchange,reseat,"
                           "legalize",
                   help="Comma list of stages to run. Valid: classify, fit, "
                        "vector, assign, exchange, reseat, legalize "
                        "(default: all). `classify` runs regardless -- it is "
                        "a prerequisite. `assign` also needs fit or vector to "
                        "have produced candidates. `reseat` lifts the parts "
                        "whose pad CENTRES are off the outline and re-seats "
                        "them from scratch at their net centroids -- it runs "
                        "before `legalize` because no cap ladder anchored on "
                        "a part's current pose can carry it back onto the "
                        "board, and legalize's minimal-move cleanup is much "
                        "cheaper once everything is on it. An unknown name is "
                        "an error, not a silent no-op.")
    p.add_argument("--anchor-extent", default="auto",
                   help="Anchor tier threshold in mm, or 'auto' = "
                        "max(3.5, P75 of pad-extent diagonals)")
    p.add_argument("--move-penalty", type=float, default=0.75,
                   help="Assignment objective: flat cost (mm-equivalent) per "
                        "moved part -- the fewest-parts-moved term "
                        "(default: 0.75)")
    p.add_argument("--assign-rounds", type=int, default=1,
                   help="Gated assign+prune passes (default 1). Round 1's "
                        "homecomings make the net-anchor centroids TRUER, so "
                        "a second round can justify moves the first could "
                        "not: a self-consistent displaced ISLAND (members "
                        "conflict-free where they sit, anchored to each "
                        "other) presents no per-member gradient until its "
                        "boundary partners are home (run-4 F2/F4). Each "
                        "round is gated and pruned; the loop stops early "
                        "when a round moves nothing")
    p.add_argument("--lock", nargs="+", default=None, metavar="REF",
                   help="Reference globs this run may NOT move, on top of the "
                        "board's own (locked yes) stamps. Every sibling "
                        "placement tool has this and reconstruct did not -- "
                        "the tool whose whole job is moving parts was the one "
                        "with no way to say which parts it may not move. "
                        "Scoping it therefore meant stamping temporary lock "
                        "bits INTO the board being measured, which "
                        "check_assembly then waives on and #521 reads.")
    p.add_argument("--max-move", type=float, default=5.0,
                   help="Legalize-stage displacement cap ladder tops out here "
                        "(default: 5.0)")
    p.add_argument("--clearance", type=float, default=None)
    p.add_argument("--board-edge-clearance", type=float, default=None)
    p.add_argument("--grid-step", type=float, default=defaults.GRID_STEP)
    p.add_argument("--dry-run", action="store_true",
                   help="Print the stage reports and move list; write nothing")
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
    stages = {s.strip() for s in args.stages.split(',') if s.strip()}
    # A misspelt stage used to be accepted silently and gate nothing, so
    # `--stages legalise` ran no legalize and exited 0 with a clean-looking
    # summary -- the same silent-lie class as the hang below.
    _unknown = sorted(stages - set(_STAGES))
    if _unknown:
        p.error(f"unknown stage(s) {', '.join(_unknown)}; valid stages are "
                f"{', '.join(_STAGES)}")

    # Armed HERE, before the refusal paths below, not next to the first stage.
    # The refusals (no Edge.Cuts, board carries copper, no legality layer)
    # printed to stderr and returned with no JSON_SUMMARY at all, so a caller
    # scraping the summary could not tell a refusal from a kill. `report` is
    # mutated in place from here on and the flush hook sees whatever it holds.
    report = {}
    report['stages_requested'] = sorted(stages)

    if not args.dry_run:
        try:
            from redo_record import record_invocation
            record_invocation()
        except Exception:
            pass

    import pose_score
    from kicad_parser import parse_kicad_pcb
    from placement import floorplan, reconstruct, seeder
    from placement.legality import grade_pad_legality
    from placement.placement_state import UNPLACED_EXIT, assess_placement
    from placement.portfolio import copy_siblings
    from placement.writer import write_placed_output

    intent = None
    if args.intent:
        try:
            intent = floorplan.load_intent(args.intent)
        except (OSError, ValueError) as exc:
            print(f"cannot load intent {args.intent}: {exc}", file=sys.stderr)
            return 2

    print(f"Loading {args.input_file}...")
    pcb = parse_kicad_pcb(args.input_file)
    if pcb.board_info.board_bounds is None:
        print("place_reconstruct: no Edge.Cuts outline -- the outline is "
              "spec-owned and will not be invented.", file=sys.stderr)
        return UNPLACED_EXIT
    st = assess_placement(pcb, args.input_file)
    if st.has_copper:
        print(f"place_reconstruct: board carries {st.segments} segment(s) / "
              f"{st.vias} via(s); reconstruction moves footprints and would "
              f"strand them. Strip or re-run from the unrouted board.",
              file=sys.stderr)
        return UNPLACED_EXIT

    # Two lock sources, resolved once so every consumer agrees: the file's
    # own (locked yes) stamps (read inside QuenchState) and these globs.
    _extra_locked = set()
    if args.lock:
        import fnmatch as _fn
        _extra_locked = {r for r in pcb.footprints
                         if any(_fn.fnmatch(r, pat) for pat in args.lock)}
        print(f"Locked via --lock: {len(_extra_locked)} ref(s)"
              + (f": {', '.join(sorted(_extra_locked)[:8])}"
                 if _extra_locked else " -- NO ref matched, check the globs"))
    state = pose_score.make_state(
        pcb, args.input_file, clearance=args.clearance,
        board_edge_clearance=args.board_edge_clearance,
        grid_step=args.grid_step, extra_locked_refs=_extra_locked or None)
    if state.legality_ctx is None:
        print("place_reconstruct: pad legality layer unavailable on this "
              "state; refusing to run blind.", file=sys.stderr)
        return 2

    notes = []

    tiers = reconstruct.classify(state, intent, args.anchor_extent)
    report['tiers'] = tiers.as_dict()
    print(f"Tiers: {len(tiers.locked)} locked, {len(tiers.zero_net)} "
          f"zero-net (frame), {len(tiers.anchors)} anchors "
          f"(extent >= {tiers.threshold}mm), {len(tiers.smalls)} smalls")

    # Run-4 F2: declared edge parts may overhang up to their band -- in the
    # candidate cull AND in the gate tuple (both sites, or the gate reverts
    # the homecoming the cull just allowed). Printed up front so the declared
    # set is part of the run's record.
    edge_bands = {}
    if intent is not None:
        # edge_claims(): a connector_affinity entry declares a class, not a
        # band. Reading the raw key gave every generic header the 2.0mm
        # default allowance below, which is an off-outline licence nobody
        # asked for.
        for c in intent.edge_claims():
            if c['ref'] not in state.parts:
                continue
            # EVERY banded entry keeps its allowance, suspect or not: with
            # banded_pad_oob ABOVE hpwl in the gate tuple, charging a
            # suspect's observed overhang would hand the ILP a strict tuple
            # improvement for pulling a healthy edge part (a false-positive
            # suspect) inboard off its correct seat. The exchange stage does
            # not need oob pressure -- it accepts on the hpwl homecoming.
            #
            # Run-8 A3 proposed the opposite (exclude suspects so the oob term
            # charges the damage) and it is REFUTED by the reason above, which
            # was measured. The suspect bit is machine-readable now so a
            # consumer CAN act on it; this consumer deliberately does not, and
            # the real fix for the blindness that motivated A3 was making the
            # oob term follow the board OUTLINE instead of its bounding box.
            band = c.get('overhang_mm') or {}
            edge_bands[c['ref']] = float(band.get('max') or 2.0)
    if edge_bands:
        print("Declared edge parts (band max mm): "
              + ", ".join(f"{r} ({m:g})"
                          for r, m in sorted(edge_bands.items())))
    report['edge_bands'] = {r: m for r, m in sorted(edge_bands.items())}

    base = reconstruct.measure(state, edge_bands)
    print(f"Gate before: {reconstruct.format_gate(base)}")
    report['gate_before'] = list(base)

    proposals = {}
    if 'fit' in stages:
        proposals = reconstruct.fit_corner_insets(state, tiers)
        for ref, cands in sorted(proposals.items()):
            print(f"  fit: {ref} proposed at {cands}")
        report['fit_proposals'] = {r: c for r, c in proposals.items()}

    vectors = []
    if 'vector' in stages:
        vectors = reconstruct.rigid_vectors(state, proposals)
        if vectors:
            print(f"  rigid vectors (up to sign): {vectors}")
        report['vectors'] = vectors
        # Run-7 A1: conflict-derived candidate vectors -- the source for
        # boards with NO mounting-hole pattern (measured: both solver
        # stages sat idle on rp2350 and recovery went negative). COARSE
        # (part-size error, absorbed by legalize) and structurally no-op
        # on healthy boards (no conflicts, no vectors); every application
        # is still refereed by the stage gates. Kept SEPARATE from the
        # pattern vectors in the report, and NOT fed to the emit guard's
        # suspect test (damage hypotheses, not blessing evidence).
        # FALLBACK ONLY: pattern vectors are precise evidence; feeding
        # coarse derived vectors beside them measurably regressed the
        # tigard exchange (risk-5 gate). Derive only when the pattern
        # source is empty -- the exact gap this source exists to fill.
        derived = ([] if vectors
                   else reconstruct.conflict_offset_vectors(state))
        if derived:
            print("  conflict-derived vectors (coarse, gate-refereed): "
                  + ", ".join(f"{d['v']} (support {d['support']})"
                              for d in derived))
            vectors = vectors + [d['v'] for d in derived]
        report['vectors_derived'] = [list(d['v']) for d in derived]

        # SAY SO WHEN THERE IS NO STRUCTURAL HYPOTHESIS.
        #
        # Run 9: three derived vectors with support 6, 6 and 4 against a
        # 107-part displaced block -- they explained ~15% of the damage -- and
        # the run then moved 80 parts on that basis and reported nothing
        # unusual. The near-inert recovery (+0.045, 0/107 home) was visible in
        # this ratio at minute four and was not discovered until the audit.
        # A render of the same board shows it instantly: the move arrows are
        # broadly parallel over one region, so there IS structure the detector
        # is not recovering. That is a finding about the DETECTOR, and it
        # belongs in the log while there is still time to act on it.
        _sup = sum(d['support'] for d in derived) if derived else 0
        _movable = len([r for r in state.parts if not state.parts[r].locked])
        report['vector_support'] = {
            'derived_support_total': _sup,
            'pattern_vectors': len(vectors) - len(derived),
            'movable_parts': _movable,
            'fraction_explained': (round(_sup / _movable, 4)
                                   if _movable else None),
        }
        if derived and _movable and (_sup / _movable) < 0.25:
            print(f"  WARNING: the derived vectors explain only {_sup} of "
                  f"{_movable} movable part(s) ({_sup / _movable:.0%}). This "
                  f"run is proceeding WITHOUT a confident structural "
                  f"hypothesis -- expect the assignment to move parts on weak "
                  f"evidence and recovery to be near-inert. If the damage "
                  f"looks like a coherent block in a render, the detector is "
                  f"missing it; that is a finding about the detector, not "
                  f"about the board.")

    # F2: declared edge entries whose edge could not be named (implausible
    # pose) -- their objective term is the EDGE metric, not the net-anchor
    # proxy (R1: an edge part's position is not a netlist question; the
    # proxy would anchor them to their possibly-misplaced partners).
    edge_pref = {}
    if intent is not None:
        # edge_claims(): the receptacle filter below already excluded the
        # weak class, but every engine read goes through the one split so a
        # future edit to this filter cannot re-open it.
        for c in intent.edge_claims():
            # Receptacles ONLY (run-5): the edge metric is the right
            # objective for a part whose class says the mating face must
            # reach the edge. An edge-less ACTUATOR entry (a suspect
            # overhang, run-5 suspect-and-derive) makes no seat claim --
            # its true home may be interior, and pinning it to an edge
            # would re-manufacture the damage; the exchange stage owns it.
            if (c['ref'] in state.parts and not c.get('edge')
                    and c.get('class') == 'edge_receptacle'
                    and not state.parts[c['ref']].locked):
                edge_pref[c['ref']] = edge_bands.get(c['ref'], 2.0)
    if edge_pref:
        print("Edge-preference refs (no declared edge; class outranks the "
              "netlist proxy): " + ", ".join(sorted(edge_pref)))

    # Exchange exclusion set (run-5 F4). An edge-declared part is excluded
    # only while SEATED (its pose is then mechanical). Unseated it is a
    # displaced part like any other -- measured: J1 at its damaged pose was
    # the squatter blocking the island's homecoming targets, and excluding
    # it cascaded the whole group to nothing. SUSPECT entries (run-5 guard)
    # are always eligible: their in-band overhang is exactly the damage
    # under investigation, not a seat.
    excl_x = set(tiers.locked) | set(tiers.zero_net)
    if intent is not None:
        for c in intent.edge_claims():   # seat claims only; see edge_claims
            ref = c['ref']
            if ref not in state.parts:
                continue
            if 'SUSPECT' in (c.get('note') or ''):
                continue
            p_ = state.parts[ref]
            em = reconstruct.edge_metric(
                state, ref, p_.x, p_.y, edge_bands.get(ref, 2.0))
            if em is not None and em <= 1e-6:
                excl_x.add(ref)

    xrep_all = {'attempts': 0, 'accepted': [], 'pruned': [], 'solver': True}

    def run_exchange():
        """The exchange fixpoint at the current state; returns moved refs."""
        nonlocal base
        xrep, xold = reconstruct.exchange_stage(
            state, vectors, exclude=excl_x, edge_bands=edge_bands,
            notes=notes)
        xrep_all['attempts'] += xrep['attempts']
        xrep_all['accepted'].extend(xrep['accepted'])
        xrep_all['solver'] = xrep_all['solver'] and xrep.get('solver', True)
        xpruned = []
        if xold:
            # Prune RUNS over the exchanged island, deliberately (the plan
            # said exempt; measured logic says otherwise): reverting any
            # single member alone breaks the island apart and worsens hpwl,
            # so a genuinely self-consistent exchange survives the sweep --
            # while a smuggled per-part mis-move (run-3 J7) does not.
            xpruned = reconstruct.prune_assignment(
                state, xold, notes, edge_bands=edge_bands,
                exempt=set(edge_pref))
            xrep_all['pruned'].extend(xpruned)
            base = reconstruct.measure(state, edge_bands)
            n_refs = len(set(xold) - set(xpruned))
            print(f"  exchange: {n_refs} part(s) moved in "
                  f"{len(xrep['accepted'])} joint move(s)"
                  + (f" ({len(xpruned)} pruned back)" if xpruned else "")
                  + f"; gate now {reconstruct.format_gate(base)}")
        return sorted(set(xold) - set(xpruned))

    moved = []
    all_pruned = []
    xmoved_all = []
    if 'assign' in stages and (vectors or proposals):
        for rnd in range(max(1, args.assign_rounds)):
            cands, pattern = reconstruct.build_candidates(
                state, tiers, vectors, proposals, edge_bands=edge_bands)
            n_multi = sum(1 for c in cands.values() if len(c) > 1)
            if rnd == 0:
                print(f"  assign: {n_multi} part(s) with a real candidate set")
            choice = reconstruct.solve_assignment(state, cands, tiers,
                                                  args.move_penalty, notes,
                                                  pattern=pattern,
                                                  edge_pref=edge_pref)
            old = {r: (state.parts[r].x, state.parts[r].y,
                       state.parts[r].rot)
                   for r in choice}
            for ref, k in sorted(choice.items()):
                if k > 0:
                    x, y = cands[ref][k]
                    state.apply_move(ref, x, y, state.parts[ref].rot)
            after = reconstruct.measure(state, edge_bands)
            before_round = base
            rmoved = []
            if after <= base:
                # Run-4 F3(b): the gate is one board-wide tuple, so an
                # accepted assignment can smuggle individual mis-moves past
                # it (run 3's spurious vector carried J7 WORSE than its
                # input inside a hugely-improving set). Per-part revert
                # sweep, gated on strict tuple improvement -- monotone,
                # board-only.
                # ...and a move the tuple rates EQUAL is reverted too unless
                # it agrees with a kept vector. A zero-net part in free space
                # touches no term the tuple has, so `strictly better` never
                # fires for it and a 26.27mm carry to another hole's seat
                # survived a sweep written to catch exactly that.
                pruned = reconstruct.prune_assignment(
                    state, old, notes, edge_bands=edge_bands,
                    exempt=set(edge_pref),
                    evidenced=reconstruct.evidenced_moves(
                        state, old, list(vectors) + list(derived)))
                all_pruned.extend(pruned)
                after = reconstruct.measure(state, edge_bands)
                base = after
                rmoved = [r for r, k in choice.items()
                          if k > 0 and r not in set(pruned)]
                moved = sorted(set(moved) | set(rmoved))
                print(f"  assign round {rnd + 1} APPLIED: {len(rmoved)} "
                      f"part(s) moved"
                      + (f" ({len(pruned)} pruned back)" if pruned else "")
                      + f"; gate now {reconstruct.format_gate(after)}")
                # The accept rule is smaller-OR-EQUAL, so a round can move
                # parts and change nothing the gate can see. That is not
                # neutral: displacement is unbounded, and a run was measured
                # taking a mounting hole 26mm and rotating a connector 90 deg
                # on an equal tuple -- both had been at their correct poses.
                # The rung above says an unimproving proposal means "the
                # determinant was not on the board", so at minimum say it out
                # loud rather than reporting a move as progress.
                if rmoved and after == before_round:
                    dists = []
                    for r in rmoved:
                        ox, oy, orot = old[r]
                        p = state.parts[r]
                        dists.append((r, math.hypot(p.x - ox, p.y - oy),
                                      (p.rot - orot) % 360.0))
                    worst = max(dists, key=lambda t: t[1])
                    print(f"    UNEVIDENCED: the gate did not change, so "
                          f"nothing measured justifies {len(rmoved)} move(s). "
                          f"Largest: {worst[0]} {worst[1]:.2f}mm"
                          + (f", rot {worst[2]:+.0f} deg" if worst[2] else "")
                          + ". Check these against their input poses before "
                            "trusting the result.")
                    report.setdefault('unevidenced_moves', []).extend(
                        {'ref': r, 'mm': round(d, 4), 'rot': round(ro, 1)}
                        for r, d, ro in sorted(dists, key=lambda t: -t[1]))
            else:
                for ref, (x, y, rot) in old.items():
                    state.apply_move(ref, x, y, rot)
                notes.append(f"assign round {rnd + 1} REVERTED: gate "
                             f"worsened {base} -> {after}")
                print(f"  assign round {rnd + 1} REVERTED "
                      f"(gate {base} -> {after})")
            # Run-5 F4: the exchange INTERLEAVES with the assign rounds --
            # each accepted exchange re-prices the next round's anchors
            # (and vice versa: a round's homecomings shrink the joint
            # solve). Runs even after a REVERTED round: the round's
            # rejection says nothing about the joint move's feasibility.
            xmoved = run_exchange() if 'exchange' in stages else []
            xmoved_all = sorted(set(xmoved_all) | set(xmoved))
            if not rmoved and not xmoved:
                break     # neither solver moved anything: converged
    elif 'exchange' in stages and vectors:
        xmoved_all = run_exchange()
    report['assign_pruned'] = sorted(set(all_pruned))
    report['assign_moved'] = sorted(moved)
    report['gate_after_assign'] = list(base)
    if 'exchange' in stages:
        report['exchange'] = {
            'attempts': xrep_all['attempts'],
            'accepted': xrep_all['accepted'],
            'pruned': sorted(set(xrep_all['pruned'])),
            'moved': xmoved_all,
            'solver': xrep_all['solver']}
        report['gate_after_exchange'] = list(base)
        if not xrep_all['accepted']:
            print(f"  exchange: no accepted joint move "
                  f"({xrep_all['attempts']} attempt(s))")
    if edge_pref:
        report['edge_pref'] = sorted(edge_pref)
        report['edge_seated'] = {
            r: [state.parts[r].x, state.parts[r].y]
            for r in sorted(edge_pref) if r in set(moved)}

    placements = [{'reference': r, 'new_x': pt.x, 'new_y': pt.y,
                   'new_rotation': pt.rot}
                  for r, pt in state.parts.items()
                  if (abs(pt.x - pt.seed_x) > 1e-9
                      or abs(pt.y - pt.seed_y) > 1e-9
                      or pt.rot != pt.orig_rot % 360)]
    for n in notes:
        print(f"  NOTE: {n}")

    def _run_reseat(board_path):
        """Re-seat the off-outline parts of `board_path` IN PLACE.

        The rung between `exchange` and `legalize`: exchange carries parts the
        damage moved by a DETECTED vector, and legalize nudges parts that are
        nearly right, so a part that is neither -- tens of millimetres out with
        no surviving family to over-determine a vector -- falls between them
        and nothing in the ladder moves it. Measured on a 107-part board: 11
        such parts, and every one of the 13 nets the router could not attempt
        had a pad on one of them.
        """
        pcb2 = parse_kicad_pcb(board_path)
        rep = seeder.reseat_scope(
            pcb2, board_path, intent, group_sources=(),
            lock_globs=args.lock,
            clearance=args.clearance,
            board_edge_clearance=args.board_edge_clearance,
            grid_step=args.grid_step,
            # The scope's own bands are filtered out INSIDE: a ref being
            # re-seated has forfeited its declared band (the band was measured
            # off the pose being discarded), and leaving it in makes the gate
            # read `oob 4.348` on a board with parts 158mm off the outline.
            edge_bands=edge_bands)
        for n in rep['notes']:
            print(f"  NOTE: {n}")
        if rep['edge_bands_dropped']:
            print("  reseat dropped the declared band of: "
                  + ", ".join(f"{r} ({m:g})" for r, m in
                              sorted(rep['edge_bands_dropped'].items())))
        if rep['moves']:
            tmp = board_path + '.reseat'
            write_placed_output(board_path, tmp, rep['moves'])
            os.replace(tmp, board_path)
        print(f"  reseat ({rep['scope_source']}): {len(rep['scope'])} in "
              f"scope, {len(rep['reseated'])} re-seated, "
              f"{len(rep['unseated'])} unseated, {len(rep['refused'])} "
              f"refused; OFF-OUTLINE PARTS {len(rep['witnesses_before'])} -> "
              f"{len(rep['witnesses_after'])}"
              + ('' if rep['accepted'] else '  [GATE REFUSED]'))
        print(f"  reseat gate: {reconstruct.format_gate(rep['gate_before'])}"
              f"  ->  {reconstruct.format_gate(rep['gate_after'])}")
        return rep

    def _reseat_report(rep, preview=False):
        # #698: the same vocabulary as `place_seed`'s summary. This rung's
        # scope is always `auto:damage_witnesses`, so the basis is `oob` or
        # None -- but the two CLIs must not describe one pass differently.
        d = {'accept_basis': rep.get('accept_basis'),
             'scope': rep['scope'], 'scope_source': rep['scope_source'],
             'reseated': rep['reseated'], 'unseated': rep['unseated'],
             'refused': rep['refused'], 'pruned': rep['pruned'],
             'edge_bands_dropped': rep['edge_bands_dropped'],
             'gate_before': rep['gate_before'],
             'gate_after': rep['gate_after'],
             'accepted': rep['accepted'],
             # `witnesses_after` is the number that predicts routability --
             # NOT `reseated`, which counts effort.
             'witnesses_before': rep['witnesses_before'],
             'witnesses_after': rep['witnesses_after']}
        if preview:
            d['preview'] = True
        return d

    def _run_legalize(board_path):
        """Legalize `board_path` IN PLACE; returns the seeder's report."""
        pcb2 = parse_kicad_pcb(board_path)
        caps = [c for c in seeder.REPAIR_CAPS_MM if c <= args.max_move]
        if not caps or caps[-1] < args.max_move:
            caps = list(caps) + [args.max_move]
        rep = seeder.repair_placement(
            pcb2, board_path, intent, group_sources=(),
            lock_globs=args.lock,
            clearance=args.clearance,
            board_edge_clearance=args.board_edge_clearance,
            grid_step=args.grid_step, caps=caps)
        for n in rep['notes']:
            print(f"  NOTE: {n}")
        if rep['moves']:
            tmp = board_path + '.legalize'
            write_placed_output(board_path, tmp, rep['moves'])
            os.replace(tmp, board_path)
        print(f"  legalize: {len(rep['repaired'])} repaired, "
              f"{len(rep['unrepairable'])} unrepairable")
        return rep

    if args.dry_run:
        report['dry_run'] = True
        report['would_move'] = sorted(p_['reference'] for p_ in placements)
        # Run-7 A11: --dry-run used to return HERE, so `--stages legalize
        # --dry-run` reported nothing about the stage it was asked to preview,
        # and a worker recorded "legalize: no-op" for a stage that repaired 7
        # parts when it really ran. A preview that silently skips the stage is
        # worse than no preview. Stage it in a temp dir instead and report what
        # legalize WOULD do; the caller's output path is still never written.
        if {'reseat', 'legalize'} & stages:
            import tempfile
            with tempfile.TemporaryDirectory() as _td:
                _preview = os.path.join(
                    _td, os.path.basename(args.output_file) or 'preview.kicad_pcb')
                write_placed_output(args.input_file, _preview, placements)
                copy_siblings(args.input_file, _preview)
                # Same ORDER as the real run, so the legalize preview is taken
                # against the RE-SEATED board rather than one the real run
                # would never hand it.
                if 'reseat' in stages:
                    report['reseat'] = _reseat_report(_run_reseat(_preview),
                                                      preview=True)
                if 'legalize' in stages:
                    _rep = _run_legalize(_preview)
                    report['legalize'] = {
                        'preview': True,
                        'repaired': _rep['repaired'],
                        'unrepairable': _rep['unrepairable'],
                        'would_move': sorted(m['reference']
                                             for m in _rep['moves']),
                    }
        report.setdefault('complete', True)
        report.setdefault('status', 'ok')
        print('JSON_SUMMARY: ' + json.dumps(report, sort_keys=True,
                                            default=str), flush=True)
        return 0

    # Run-7 A11: the output used to be written BEFORE legalize ran, so a run
    # killed in between left a board at the caller's chosen path that looks
    # like a finished result and is not one (the caps sweep can move parts for
    # minutes). Stage it beside the target and promote once, so the output path
    # only ever holds a complete board.
    staged = args.output_file + '.staging.kicad_pcb'
    write_placed_output(args.input_file, staged, placements)
    copy_siblings(args.input_file, staged)

    if 'reseat' in stages:
        _rs = _run_reseat(staged)
        report['reseat'] = _reseat_report(_rs)

    if 'legalize' in stages:
        rep = _run_legalize(staged)
        report['legalize'] = {'repaired': rep['repaired'],
                              'unrepairable': rep['unrepairable']}

    _promote_staged(staged, args.output_file)

    out_pcb = parse_kicad_pcb(args.output_file)
    final = grade_pad_legality(out_pcb, args.clearance,
                               pcb_file=args.output_file)
    report['final'] = {k: final[k] for k in
                       ('pad_conflicts', 'pad_shortfall', 'hole_conflicts',
                        'oob_pad_count', 'oob_pad_amount', 'exact')}
    # Run-6: the assembly channel joins the final verdict -- run 5's output
    # carried a two-part STACK the pad/hole channels cannot see, and this
    # report (plus the exit code) was the last silent gate.
    from placement.legality import grade_body_overlap
    body = grade_body_overlap(out_pcb, args.clearance,
                              intent_waivers=(intent.waiver_pairs()
                                              if intent is not None else ()),
                              pcb_file=args.output_file)
    report['final']['body_blocking'] = body['blocking']
    report['final']['body_advisory'] = body['advisory']
    report['final']['body_pairs'] = [q._asdict() for q in body['pairs']
                                     if not q.waived]
    report['output'] = args.output_file
    print(f"Final (exact): {final['pad_conflicts']} pad conflict pair(s), "
          f"{final['hole_conflicts']} hole conflict(s), "
          f"{final['oob_pad_count']} part(s) with pad copper off-board, "
          f"{body['blocking']} blocking body pair(s)")
    _leg_mod = __import__('placement.legality', fromlist=['x'])
    # #697: name the pairs graded above args.clearance and what raised them,
    # before the count above is read as a violation of the announced floor.
    _rq = _leg_mod.format_required_clause(final)
    if _rq:
        print(f"  above the {args.clearance}mm floor: {_rq}")
    _oc = _leg_mod.format_oob_clause(final)
    if _oc:
        # Printed BEFORE the edge_connectors advice below, because that advice
        # tells the reader to declare a by-design overhang -- and this measure
        # fires on a part sitting INSIDE the board by a clearance band.
        print(f"  off-board parts: {_oc}")
    for q in body['blocking_pairs']:
        print(f"  BODY STACK: {q.a} <-> {q.b} {q.kind} {q.area_mm2}mm2 "
              f"side {q.side} -- not physically buildable")
    if final['oob_pad_count']:
        print("  (off-board residue that no cap could repair: if it is a "
              "by-design overhang -- a card edge, a switch actuator -- "
              "declare it in an intent's edge_connectors; it is then exempt)")
    report.setdefault('complete', True)
    report.setdefault('status', 'ok')
    print('JSON_SUMMARY: ' + json.dumps(report, sort_keys=True, default=str),
          flush=True)
    # Exit 4 = residual PAD/HOLE conflicts or a blocking BODY pair -- the
    # defect classes this tool exists to remove. Off-board residue is
    # reported (and by-design overhang is indistinguishable from defect
    # without an intent), not an exit code.
    return 4 if (final['pad_conflicts'] or final['hole_conflicts']
                 or body['blocking']) else 0


if __name__ == "__main__":
    # Declare the lever for the WHOLE run, so every pose this CLI
    # writes carries its name. Nothing called declare_lever outside
    # tests, so the unaided instrument had no armed state at all:
    # unarmed it is silent, and armed by hand it refused the engine.
    from placement.provenance import declare_lever
    with declare_lever('place_reconstruct.py', sys.argv):
        import cli_banner; cli_banner.install()  # CMD/EXIT self-echo (run-3 B1)
        sys.exit(main())

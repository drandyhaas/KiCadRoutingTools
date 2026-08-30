#!/usr/bin/env python3
"""Is this placement physically BUILDABLE? The assembly gate (run-6).

One command, one verdict: the body-overlap channel (blocking = cross-
footprint pad intersections, corpus-calibrated to zero on healthy boards;
advisory = fab/courtyard pairs with class/intent waiver labels) plus the
pad/hole/oob legality echo -- both conjuncts in one JSON.

Needs NO intent to be meaningful (unlike check_floorplan's legality rule,
which skips without a budget): a bare board grades honestly. --intent adds
authored overlap waivers only.

--baseline <board> computes the loop currency: advisory pairs NEW relative
to the baseline board (dense real boards ship hundreds of by-design
courtyard kisses -- the corpus measured 235 -- so the placement fix loop
targets the pairs OUR moves introduced, never a shipped design's own).
--baseline also ARMS the courtyard gate (run-23): an unwaived courtyard
interpenetration past the area+depth floors flips the verdict when a member
MOVED relative to the baseline. Without a baseline the courtyard census is
report-only -- healthy human boards ship such pairs by design (measured:
5 of 34 corpus boards), so an absolute gate would be unshippable.

Exit codes: 0 = clean, 2 = usage/load error, 4 = a blocking pair OR copper
landing on a KiCad-locked part (see LOCKED-PART CONTACT) OR a coincident-
origin stack (see COINCIDENT ORIGINS) OR a containment OR a moved-vs-
baseline courtyard interpenetration (see COURTYARD BLOCKING).
"""
import _path  # noqa: F401  (py_tools -> py_router/py_placer on sys.path)

import argparse
import json
import sys


def main():
    p = argparse.ArgumentParser(
        description="Assembly (body-overlap) audit of a placed board.")
    p.add_argument("board")
    p.add_argument("--intent", default=None, metavar="JSON",
                   help="Floorplan intent; only its overlap_waivers are read")
    p.add_argument("--clearance", type=float, default=None,
                   help="Pad-model clearance in mm. Default: the board's own "
                        "Default net-class clearance, else routing_defaults. "
                        "The effective value and its source are printed and "
                        "written to --json")
    p.add_argument("--baseline", default=None, metavar="BOARD",
                   help="Report advisory pairs NEW relative to this board "
                        "(the placement-loop currency)")
    p.add_argument("--json", default=None, metavar="PATH",
                   help="Write the full grade as JSON")
    args = p.parse_args()

    import routing_defaults as defaults
    from kicad_parser import parse_kicad_pcb
    from placement import legality
    from placement.legality import grade_body_overlap, grade_pad_legality

    # GRADE AT THE BOARD'S OWN FLOOR, and say where that came from.
    #
    # This default USED to be a flat routing_defaults.CLEARANCE (0.25) that did
    # not read the board, while every router resolves the board's own Default
    # netclass first. So the tool routinely graded STRICTER than the board it
    # was grading. Measured on one 0.2mm board: pad_conflicts 96 at the default
    # vs 39 at the board's own floor -- `blocking` and `locked_contacts` were
    # identical, because those pairs are true pad INTERSECTIONS rather than
    # clearance grazes, which is why it survived this long.
    #
    # The previous decision here was to keep the constant and merely DISCLOSE
    # it, on the grounds that board_score and the stress harness shell this
    # tool and re-basing their numbers was the bigger change. That trade is now
    # reversed deliberately: a disclosure only helps a reader who acts on it,
    # and the run that motivated this read the numbers, not the note. The
    # re-basing is real but bounded -- it moves the pad/hole ECHO counts toward
    # the truth and leaves `blocking` (the field board_score consumes) alone.
    #
    # "Every board in kicad_files/ declares no floor and so is bit-identical"
    # is what stood here, and it is FALSE. Two project siblings are committed:
    #
    #     $ git ls-files kicad_files/*.kicad_pro
    #     kicad_files/flat_hierarchy.kicad_pro     Default clearance 0.2
    #     kicad_files/routed_output.kicad_pro      Default clearance 0.09
    #
    # so on those two this tool re-based from the 0.25 constant to 0.2 and to
    # 0.09. What IS bit-identical is the VERDICT, which is the weaker claim
    # that should have been made: measured at 5894b95^ vs HEAD, both boards
    # report pad_conflicts 0 -> 0 and buildable True -> True; only the graded
    # clearance and its recorded source moved. Grading at the board's own
    # clearance is right HERE because this tool grades existing geometry --
    # check_channels PREDICTS routability and therefore needs a fab floor
    # instead (see list_nets.board_floor, grade-vs-predict).
    _board_clr = None
    _decl = None
    try:
        from list_nets import (board_default_netclass_clearance,
                               board_floor_declaration)
        _board_clr = board_default_netclass_clearance(args.board)
        _decl = board_floor_declaration(args.board)
    except Exception:                                          # noqa: BLE001
        pass
    from list_nets import board_floor
    clearance, _src = board_floor(args.board, 'clearance', args.clearance,
                                  defaults.CLEARANCE)
    print(f"  grading at clearance {clearance}mm  [{_src}]")
    # A board that declares NOTHING is a DIFFERENT case from one whose Default
    # class happens to match (run-12 Tier 1.3). The comparison below can only
    # fire when there IS a declared value to compare against, so on a
    # project-less board -- tigard ships none -- this tool said "grading at
    # 0.25mm [routing_defaults]" and nothing recorded that 0.25 was a fallback
    # rather than agreement. Name it.
    if _decl is not None and _decl['declares_nothing']:
        print(f"  NOTE: this board declares NO net class and NO board "
              f"constraint (no sibling .kicad_pro, no (net_class) block), so "
              f"{clearance}mm is a FALLBACK rather than the board's own floor. "
              f"Pass --clearance <the value the copper was routed to> if you "
              f"know it; the pad/hole ECHO counts move with it, `blocking` "
              f"usually does not.")
    # The note now fires on the OPPOSITE case, which is the only one left that
    # can be a mistake: the caller OVERRODE a floor the board does declare.
    # Before, the tool could silently disagree with the board; now only a human
    # can, and they should be told they did.
    if args.clearance is not None and _board_clr is not None \
            and abs(_board_clr - clearance) > 1e-9:
        print(f"  NOTE: --clearance {clearance}mm overrides this board's own "
              f"Default net-class clearance of {_board_clr}mm, so this grade "
              f"is {'STRICTER' if clearance > _board_clr else 'LOOSER'} than "
              f"the board asks for. Drop --clearance to grade at the board's "
              f"floor. Expect the pad/hole ECHO counts to move; `blocking` "
              f"usually will not, because blocking pairs are pad "
              f"intersections rather than clearance grazes.")

    waivers = ()
    if args.intent:
        try:
            from placement.floorplan import load_intent
            waivers = load_intent(args.intent).waiver_pairs()
        except Exception as exc:
            print(f"cannot load intent {args.intent}: {exc}", file=sys.stderr)
            return 2

    try:
        pcb = parse_kicad_pcb(args.board)
    except Exception as exc:
        print(f"cannot parse {args.board}: {exc}", file=sys.stderr)
        return 2

    g = grade_body_overlap(pcb, clearance, intent_waivers=waivers,
                           pcb_file=args.board)
    leg = grade_pad_legality(pcb, clearance, worst_n=0, pcb_file=args.board)
    # #697: name any pair graded ABOVE `clearance` and what raised it, or the
    # echo below reports a count the announced floor cannot explain.
    from placement.legality import format_required_clause as _req_clause
    _leg_required = _req_clause(leg)
    _leg_notes = list(leg.get('clearance_notes') or ())

    # COINCIDENT ORIGINS (run-19, measured twice): SW17+SW34+REF_PUCK_R all at
    # one point graded `buildable (blocking 0)` -- the pair currency counts pad
    # INTERSECTIONS, and the rotated pads happened to interleave. A stack of
    # parts at one origin is unbuildable whatever the pads do, so it is its own
    # blocking channel. The grouping and the exoneration are placement_state's
    # prior art, reused rather than re-derived: assess_placement buckets every
    # pad-bearing footprint by round(coord, 3), partitions each bucket by
    # physical side (a drilled part is on both), and exonerates all-marker
    # side-groups (fiducials, mounting holes, testpoints -- mouse-bites and
    # graphics markers are co-located by design and must NOT flag). A bucket is
    # a finding here only when it holds >= 2 suspect NON-marker parts: one real
    # part sitting on a fiducial is not a stack of parts.
    from placement.part_class import classify_part
    from placement.placement_state import assess_placement
    _MARKER_CLASSES = ('fiducial', 'mount_hole', 'testpoint')

    def _marker(ref):
        try:
            return classify_part(pcb.footprints[ref],
                                 ref).name in _MARKER_CLASSES
        except Exception:                                      # noqa: BLE001
            return False

    _suspect = assess_placement(pcb, pcb_file=args.board).stacked_suspect_refs
    _buckets = {}
    for _ref in _suspect:
        _fp = pcb.footprints.get(_ref)
        if _fp is None:
            continue
        _buckets.setdefault((round(_fp.x, 3), round(_fp.y, 3)),
                            []).append(_ref)
    # NOTE this iterates DISTINCT REFERENCES. `pcb.footprints` is a dict keyed
    # by reference, so two footprint blocks sharing one reference are ONE entry
    # here and cannot form a coincident pair -- the check that exists to catch
    # two parts at one point is structurally blind to two parts with one name.
    # `duplicate_references` below is that case, reported separately.
    stack_groups = [{'point': [pt[0], pt[1]], 'refs': refs}
                    for pt, refs in sorted(_buckets.items())
                    if sum(1 for r in refs if not _marker(r)) >= 2]
    dup_refs = dict(getattr(pcb, 'duplicate_references', None) or {})

    new_advisory = None
    moved_refs = None
    if args.baseline:
        try:
            base_pcb = parse_kicad_pcb(args.baseline)
        except Exception as exc:
            print(f"cannot parse baseline {args.baseline}: {exc}",
                  file=sys.stderr)
            return 2
        gb = grade_body_overlap(base_pcb, clearance, intent_waivers=waivers,
                                pcb_file=args.baseline)
        base_keys = {(q.a, q.b, q.kind) for q in gb['pairs']}
        new_advisory = [q for q in g['advisory_pairs']
                        if (q.a, q.b, q.kind) not in base_keys]
        # Refs whose POSE differs from the baseline (position, rotation mod
        # 360, or layer). This is the courtyard gate's currency: a pair is
        # chargeable only when OUR moves put a member there. Pair-membership
        # ("new vs baseline") is NOT enough -- run-23's RN3<->U5 existed in
        # the damaged baseline (the staged containment), the repair moved RN3
        # 3.28mm and left the pair blocking, and a membership test would have
        # called it pre-existing. A ref absent from the baseline counts as
        # moved: something put it there.
        moved_refs = set()
        for _ref, _fp in pcb.footprints.items():
            _bp = base_pcb.footprints.get(_ref)
            if _bp is None:
                moved_refs.add(_ref)
                continue
            _drot = ((_fp.rotation or 0.0) - (_bp.rotation or 0.0)) % 360.0
            if (abs(_fp.x - _bp.x) > 1e-3 or abs(_fp.y - _bp.y) > 1e-3
                    or min(_drot, 360.0 - _drot) > 1e-3
                    or (_fp.layer or '') != (_bp.layer or '')):
                moved_refs.add(_ref)

    print(f"Assembly audit of {args.board} (clearance {clearance}):")
    if dup_refs:
        # ADVISORY, never blocking: a duplicate reference is legal in KiCad and
        # can be deliberate. But it must not be silent -- on run 20's board
        # `coincident_origins` read 0 while TWO pairs sat at exactly coincident
        # positions, because each pair was one dict entry.
        _n = sum(dup_refs.values())
        print(f"  DUPLICATE REFERENCES (advisory): {_n} footprint block(s) "
              f"share {len(dup_refs)} reference(s) -- "
              + ', '.join(f'{r} x{c}' for r, c in sorted(dup_refs.items())))
        print(f"    Only the LAST block of each is parsed, so this audit sees "
              f"{len(pcb.footprints)} parts and `coincident_origins` cannot "
              f"compare the dropped ones. Legal, but rename them if they are "
              f"meant to be distinct parts.")
    print(f"  blocking {g['blocking']}  advisory {g['advisory']}"
          f"  waived {g['waived']}  contained {g['contained']}"
          f"  courtyard_blocking {g['courtyard_blocking']}"
          + (f"  new-vs-baseline {len(new_advisory)}"
             if new_advisory is not None else ""))
    _cb_keys = {(q.a, q.b) for q in g['courtyard_blocking_pairs']}
    for q in g['pairs']:
        label = ('BLOCKING' if q.kind == 'pad_intersection'
                 else ('COURTYARD-BLOCKING'
                       if q.kind == 'courtyard' and (q.a, q.b) in _cb_keys
                       else (f'waived:{q.waiver}' if q.waived
                             else 'advisory')))
        star = ''
        if new_advisory is not None and q in new_advisory:
            star = '  <-- NEW vs baseline'
        cont = ''
        if q.contained:
            cont = f"  CONTAINED {q.contained_frac:.0%}"
        print(f"    {q.a} <-> {q.b}  {q.kind}  {q.area_mm2}mm2 "
              f"side {q.side}  {label}{cont}{star}")
        # Different-net pads touching is a short on top of the overlap. Say so
        # here rather than making a reader re-derive it from the board.
        for sh in getattr(q, 'shorts', ()) or ():
            print(f"        SHORT: {sh}")

    if stack_groups:
        print(f"  COINCIDENT ORIGINS ({len(stack_groups)}): parts stacked at "
              f"one point. Rotated pads can interleave, so the pad-"
              f"intersection channel alone can grade a stack buildable.")
        for grp in stack_groups:
            print(f"    {' '.join(grp['refs'])} @ "
                  f"({grp['point'][0]}, {grp['point'][1]})")

    locked_contact = g.get('locked_contact_pairs') or []
    if locked_contact:
        print(f"  LOCKED-PART CONTACT ({len(locked_contact)}): copper lands on "
              f"a part KiCad marks (locked yes)")
        for q in locked_contact:
            print(f"    {q.a} <-> {q.b}  {q.kind}  {q.area_mm2}mm2  "
                  f"locked: {q.locked_ref}"
                  + ('  [' + q.waiver + ']' if q.waived else ''))
        print("    A locked pose is a decision somebody made (an enclosure "
              "standoff, a panel cut-out). A placement search may not settle "
              "this by moving the other part somewhere it likes better, and a "
              "waiver class chosen for unlocked parts does not apply here.")
    from placement.legality import format_oob_clause
    _clause = format_oob_clause(leg)
    if _leg_required:
        print(f"  above the {clearance}mm floor: {_leg_required}")
    for _n in _leg_notes:
        print(f"  pad clearance: {_n}")
    print(f"  pad/hole/oob echo: {leg['pad_conflicts']} pad pair(s), "
          f"{leg['hole_conflicts']} hole conflict(s), "
          f"{leg['oob_pad_count']} part(s) with pad copper off-board"
          + (": " + _clause if _clause else ""))
    # ONE predicate, used verbatim at all three sites (verdict, JSON
    # `buildable`, exit code). Three re-derivations of `blocking or
    # locked_contact` is how the coincident-origin channel would have reached
    # two of them and silently missed the third.
    if g['contained']:
        print(f"  CONTAINMENT ({g['contained']}): a part's .Fab body lies "
              f"wholly or mostly inside another part's.")
        for q in g['containment_pairs']:
            tag = f"  [waived:{q.waiver}]" if q.waived else ''
            print(f"    {q.a} <-> {q.b}  {q.area_mm2}mm2  "
                  f"{q.contained_frac:.0%} of the smaller body{tag}")
        if g['containment_blocking']:
            print(f"    {g['containment_blocking']} of these BLOCK: a "
                  f"containment gates unless a mount-hole/fiducial/testpoint "
                  f"or a board-sized container is involved, or the pair is "
                  f"named in the intent's `overlap_waivers`. An `edge_class` "
                  f"waiver does NOT exempt -- it is a part-class lookup with "
                  f"no geometry in it, and it is what hid a part wholly "
                  f"inside a switch body in run 22.")
            print(f"    To accept one deliberately, name the pair in the "
                  f"intent rather than relying on its class.")
        else:
            print(f"    None of these BLOCK: each is a by-design containment "
                  f"(a marker or a board-sized container), which the corpus "
                  f"ships legitimately -- orangecrab FID2/J5 at 100%.")
    if g['fab_unjudged']:
        _u = g['fab_unjudged_refs']
        print(f"  BODY COVERAGE: {g['fab_unjudged']} of "
              f"{len(pcb.footprints)} part(s) draw no .Fab outline, so the "
              f"containment channel cannot judge them: "
              + ', '.join(_u[:8]) + (' ...' if len(_u) > 8 else ''))

    # Courtyard gate currency (run-23): the census below is ABSOLUTE, but the
    # GATE is moved-vs-baseline. Measured on this repo's own corpus: 5 healthy
    # human boards ship unwaived courtyard interpenetrations past any sane
    # floor (ulx3s GPDI1<->U11 at 38.5mm2 depth 5.1; rp2350 U3 frac-1.0 inside
    # J2 -- both documented by-design), so an absolute conjunct flips 5 of 34
    # corpus boards NOT BUILDABLE and is unshippable. A pair gates only when a
    # MEMBER MOVED relative to --baseline: a pristine board graded against
    # itself can never flip, while a repair run owns every pair its moves
    # created or failed to clear (run-23: J4/J3/RN3 all moved; all three
    # defects gate).
    #
    # The currency's ONE blind spot, named rather than papered over: a pair
    # the DAMAGE created and the repair never touched (neither member moved)
    # reads as the baseline's own -- run-23's FB1<->SW2 (0.70mm2, real body
    # contact) is exactly that. It stays in the census and the review-sheet
    # facts, and the boundary review must disposition it; no movement test
    # can charge it without also flipping pristine boards.
    courtyard_gating = []
    if g['courtyard_blocking'] and moved_refs is not None:
        courtyard_gating = [q for q in g['courtyard_blocking_pairs']
                            if q.a in moved_refs or q.b in moved_refs]
    if g['courtyard_blocking']:
        _gate_note = (
            f"{len(courtyard_gating)} of {g['courtyard_blocking']} GATE "
            f"(a member moved vs the baseline)" if moved_refs is not None
            else f"REPORT-ONLY: pass --baseline <the board the run started "
                 f"from> to gate the pairs your moves created")
        print(f"  COURTYARD BLOCKING ({g['courtyard_blocking']}): unwaived "
              f"courtyard interpenetration past the floors (area >= "
              f"{legality.COURTYARD_BLOCKING_MIN_MM2}mm2 OR >= "
              f"{legality.COURTYARD_BLOCKING_MIN_FRAC:.0%} of the smaller "
              f"courtyard, AND depth >= "
              f"{legality.COURTYARD_BLOCKING_MIN_DEPTH_MM}mm) -- {_gate_note}")
        for q in g['courtyard_blocking_pairs']:
            _mv = ''
            if moved_refs is not None:
                _who = [r for r in (q.a, q.b) if r in moved_refs]
                _mv = ('  GATES (moved: ' + ' '.join(_who) + ')' if _who
                       else '  baseline\'s own (no member moved)')
            print(f"    {q.a} <-> {q.b}  {q.area_mm2}mm2  depth "
                  f"{q.depth_mm}mm  side {q.side}{_mv}")
        print(f"    Run-23 shipped J4 0.90mm inside U6 as `buildable` "
              f"because courtyard overlap was advisory everywhere. A "
              f"deliberate overlap is accepted by naming the pair in the "
              f"intent's `overlap_waivers`, where the acceptance is visible.")

    # A FOURTH conjunct, and `g['blocking']` is deliberately NOT touched.
    # `blocking` means "pad intersections" to board_score, to the seeder's
    # repair census, and -- with INVERTED polarity -- to placement_driver's
    # _guard_damage, which refuses to run the repair stages when `not
    # blocking`. Folding containment into that count would change all three.
    # This is the same shape the coincident-origin channel used.
    # `courtyard_gating` is the FIFTH conjunct (run-23): the moved-vs-baseline
    # subset of the courtyard census -- see the currency comment above for
    # why the absolute census must not gate.
    not_buildable = bool(g['blocking'] or locked_contact or stack_groups
                         or g['containment_blocking']
                         or courtyard_gating)
    verdict = 'NOT BUILDABLE' if not_buildable else 'buildable (blocking 0)'
    print(f"  VERDICT: {verdict}")

    if args.json:
        doc = {
            'board': args.board,
            'clearance': clearance,
            # 'cli' | 'board netclass' | 'fixed default' -- two grades are
            # comparable only when this agrees, and the scalar alone cannot
            # say whether 0.25 was the board's answer or this tool's.
            'clearance_source': _src,
            # run-12 Tier 1.3: True when `clearance` above is this tool's
            # fallback because the board declared no floor at all -- which a
            # reader comparing the scalar across boards cannot otherwise tell
            # from a board that genuinely asks for it.
            'board_declares_no_floor': bool(_decl and _decl['declares_nothing']),
            'blocking': g['blocking'],
            # The verdict itself, and the scalar behind half of it. Without
            # these every reader re-derives `blocking == 0 and not
            # locked_contact_pairs` for itself -- and the ones that got it
            # wrong got it wrong quietly: board_score's assembly component
            # reads `blocking` alone, and the recovery arm scraped this
            # verdict back out of stdout rather than reading the JSON.
            'buildable': not not_buildable,
            'verdict': verdict,
            'locked_contacts': len(locked_contact),
            'advisory': g['advisory'],
            'waived': g['waived'],
            'pairs': [q._asdict() for q in g['pairs']],
            'contained': g['contained'],
            'containments': [q._asdict() for q in g['containment_pairs']],
            'fab_unjudged': g['fab_unjudged'],
            'fab_unjudged_refs': g['fab_unjudged_refs'],
            # Run-23 courtyard channel. `courtyard_pairs` is EVERY
            # courtyard-kind pair (waived included, so a reader never
            # re-derives the census); `courtyard_blocking*` is the gated
            # subset; `courtyard_advisory` the unwaived-but-not-gating rest.
            # NOTE `b_body_overlap_pairs` in render_placement's checklist is
            # PAD INTERSECTIONS, not this -- the name predates this channel.
            'courtyard_blocking': g['courtyard_blocking'],
            'courtyard_blocking_pairs': [q._asdict()
                                         for q in g['courtyard_blocking_pairs']],
            # The subset that actually GATES buildable: census pairs where a
            # member MOVED vs --baseline. None (not 0) without a baseline --
            # "not measured" must never read as "measured clean".
            'courtyard_blocking_gating': (len(courtyard_gating)
                                          if moved_refs is not None else None),
            'courtyard_blocking_gating_pairs': [q._asdict()
                                                for q in courtyard_gating],
            'courtyard_gating_basis': ('moved-vs-baseline'
                                       if moved_refs is not None
                                       else 'no-baseline: report-only'),
            'courtyard_pairs': [q._asdict() for q in g['pairs']
                                if q.kind == 'courtyard'],
            'courtyard_advisory': sum(
                1 for q in g['advisory_pairs'] if q.kind == 'courtyard'
                and q not in g['courtyard_blocking_pairs']),
            'courtyard_synthetic_refs': g['courtyard_synthetic_refs'],
            'blocking_pairs': [q._asdict() for q in g['blocking_pairs']],
            'advisory_pairs': [q._asdict() for q in g['advisory_pairs']],
            'pad_conflicts': leg['pad_conflicts'],
            'pad_clearance_required': leg.get('required') or [],
            'hole_conflicts': leg['hole_conflicts'],
            'oob_pad_count': leg['oob_pad_count'],
            'oob_pad_amount': leg['oob_pad_amount'],
            # The MACHINE path, which is the one that matters here: this doc is
            # what loop_driver's L2 gate reads, and that gate refuses with "N
            # part(s) carry pad copper OFF the board -- their nets cannot be
            # routed at all". It could not name the part, and the count it
            # gates on moves with --clearance, so a clearance-band graze reads
            # as copper in the air. Both facts now travel with the number.
            'oob_pad_refs': leg.get('oob_pad_refs') or [],
            'oob_pad_basis': leg.get('oob_pad_basis'),
            'locked_contact_pairs': [q._asdict() for q in locked_contact],
            # run-19: parts stacked at one origin, marker classes exonerated.
            # Groups, not fake N*(N-1)/2 pair entries -- a stack is one
            # finding about one point, and the fix is one re-seat per part.
            'coincident_origin_groups': stack_groups,
            'coincident_origins': len(stack_groups),
            'coincident_origins_basis': (
                'distinct references only -- footprints are keyed by '
                'reference, so blocks sharing one reference are a single '
                'entry here and cannot form a pair. See duplicate_references.'),
            'duplicate_references': dup_refs,
            'footprint_blocks': len(pcb.footprints) + sum(dup_refs.values())
                                - len(dup_refs),
        }
        if new_advisory is not None:
            doc['baseline'] = args.baseline
            doc['new_advisory_pairs'] = [q._asdict() for q in new_advisory]
        with open(args.json, 'w', encoding='utf-8') as f:
            json.dump(doc, f, indent=1, sort_keys=True)
        print(f"  JSON -> {args.json}")

    return 4 if not_buildable else 0


if __name__ == "__main__":
    import cli_banner
    cli_banner.install()   # CMD/EXIT self-echo (run-3 B1)
    sys.exit(main())

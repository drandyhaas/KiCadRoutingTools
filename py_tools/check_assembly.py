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

Exit codes: 0 = clean, 2 = usage/load error, 4 = a blocking pair OR copper
landing on a KiCad-locked part (see LOCKED-PART CONTACT) OR a coincident-
origin stack (see COINCIDENT ORIGINS).
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
    leg = grade_pad_legality(pcb, clearance, worst_n=0)

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
    stack_groups = [{'point': [pt[0], pt[1]], 'refs': refs}
                    for pt, refs in sorted(_buckets.items())
                    if sum(1 for r in refs if not _marker(r)) >= 2]

    new_advisory = None
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

    print(f"Assembly audit of {args.board} (clearance {clearance}):")
    print(f"  blocking {g['blocking']}  advisory {g['advisory']}"
          f"  waived {g['waived']}"
          + (f"  new-vs-baseline {len(new_advisory)}"
             if new_advisory is not None else ""))
    for q in g['pairs']:
        label = ('BLOCKING' if q.kind == 'pad_intersection'
                 else (f'waived:{q.waiver}' if q.waived else 'advisory'))
        star = ''
        if new_advisory is not None and q in new_advisory:
            star = '  <-- NEW vs baseline'
        print(f"    {q.a} <-> {q.b}  {q.kind}  {q.area_mm2}mm2 "
              f"side {q.side}  {label}{star}")
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
    print(f"  pad/hole/oob echo: {leg['pad_conflicts']} pad pair(s), "
          f"{leg['hole_conflicts']} hole conflict(s), "
          f"{leg['oob_pad_count']} part(s) with pad copper off-board"
          + (": " + _clause if _clause else ""))
    # ONE predicate, used verbatim at all three sites (verdict, JSON
    # `buildable`, exit code). Three re-derivations of `blocking or
    # locked_contact` is how the coincident-origin channel would have reached
    # two of them and silently missed the third.
    not_buildable = bool(g['blocking'] or locked_contact or stack_groups)
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
            'blocking_pairs': [q._asdict() for q in g['blocking_pairs']],
            'advisory_pairs': [q._asdict() for q in g['advisory_pairs']],
            'pad_conflicts': leg['pad_conflicts'],
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

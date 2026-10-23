#!/usr/bin/env python3
"""Is this placement physically BUILDABLE? The assembly gate (run-6).

One command, one verdict: the body-overlap channel (blocking = cross-
footprint pad intersections, corpus-calibrated to zero on healthy boards;
advisory = fab/courtyard pairs with class/intent waiver labels) plus the
pad/hole/oob legality echo -- both conjuncts in one JSON.

Needs NO intent to be meaningful (unlike check_floorplan's legality rule,
which skips without a budget): a bare board grades honestly. --intent adds
authored overlap waivers, the courtyard pairs a declared pose accepts
(`fixed_poses[].accept_courtyard_overlap`, courtyard only, #1298), and a
declared `mating:<ref>` keep-out, which replaces the plug region derived from
the board (#1098).

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
baseline courtyard interpenetration (see COURTYARD BLOCKING) OR a test point
a moved member left under another part's body (see TEST POINT COVERED,
#1250; armed by --baseline like the courtyard gate) OR a body inside the
body-to-edge band the --intent DECLARES (`assembly.body_edge_mm`; see
BODY-TO-EDGE BAND, #1255 -- undeclared, it reports against JLCPCB's 2.5 mm
and never gates) OR a pad touching another part's footprint graphic copper
with a member moved vs --baseline (see PAD ON ANOTHER PART'S COPPER, #1266;
armed by --baseline like the courtyard gate) OR two of one part's own pads
touching on two different real nets, KiCad's shorting_items (see OWN PADS
TOUCHING, #1306; with --baseline a pair the baseline's footprint already had
does not gate).
"""

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['placement', 'combined'], 'kind': 'instrument'}

import _path  # noqa: F401  (py_tools -> py_router/py_placer on sys.path)

import argparse
import json
import os
import sys

#: #1201. Why a contained pair does not gate, as check_assembly prints it --
#: keyed by `BodyOverlapPair.gate_exempt`.
_GATE_EXEMPT_WHY = {
    'marker_class': 'a mount-hole/fiducial/testpoint is involved',
    'container_class': 'a board-sized container is involved',
    'intent_declared': "named in the intent's overlap_waivers",
    'silk_drawn': 'a silk-drawn body, which never gates (#896)',
}


def main():
    p = argparse.ArgumentParser(
        description="Assembly (body-overlap) audit of a placed board.")
    p.add_argument("board")
    p.add_argument("--intent", default=None, metavar="JSON",
                   help="Floorplan intent; its overlap_waivers are read, "
                        "its fixed_poses[].accept_courtyard_overlap waive "
                        "those COURTYARD pairs only (#1298), "
                        "a keep-out it names `mating:<ref>` replaces the "
                        "plug mating region derived from the board (#1098), "
                        "and a declared assembly.body_edge_mm makes the "
                        "body-to-edge band gate (#1255)")
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
    p.add_argument("--ignore-project-severity", action="store_true",
                   help="Grade courtyard overlaps at KiCad's default "
                        "(error) even when the board's .kicad_pro sets "
                        "courtyards_overlap to ignore (#1095). Default: the "
                        "project's own severity (a 'warning' is graded as "
                        "error either way, since KiCad still reports it)")
    args = p.parse_args()

    import routing_defaults as defaults
    from kicad_parser import parse_kicad_pcb
    from placement import legality
    from placement.legality import (assembly_census, grade_body_overlap,
                                    grade_pad_legality)

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
    # a tool that PREDICTS routability needs a fab floor instead (see
    # list_nets.board_floor, grade-vs-predict).
    #
    # #1338: "the board's own clearance" is what KiCad grades -- the Default
    # class floored at Board Setup min_clearance, as check_drc has read it
    # since #1210 -- not the class alone. multichannel_mixer declares class
    # 0.2 and min_clearance 0.3; at 0.2 this tool called a board buildable
    # with no pad pair while kicad-cli and check_drc reported two at 0.24 and
    # 0.25 mm, and `--clearance 0.3` was told it was STRICTER than the board.
    _board_clr = None
    _decl = None
    from list_nets import board_grading_clearance
    try:
        from list_nets import board_floor_declaration
        _board_clr, _board_src = board_grading_clearance(args.board)
        _decl = board_floor_declaration(args.board)
    except Exception:                                          # noqa: BLE001
        pass
    clearance, _src = board_grading_clearance(args.board, args.clearance,
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
        print(f"  NOTE: --clearance {clearance}mm overrides the clearance KiCad "
              f"grades this board at, {_board_clr}mm [{_board_src}], so this "
              f"grade is {'STRICTER' if clearance > _board_clr else 'LOOSER'} "
              f"than the board asks for. Drop --clearance to grade at the board's "
              f"floor. Expect the pad/hole ECHO counts to move; `blocking` "
              f"usually will not, because blocking pairs are pad "
              f"intersections rather than clearance grazes.")

    waivers = ()
    # #1298: the pairs a declared pose accepts a COURTYARD overlap with
    # (`fixed_poses[].accept_courtyard_overlap`). Courtyard-only: they reach
    # the courtyard channel through `grade_body_overlap(courtyard_waivers=)`
    # and never the containment gate, which `waivers` above also reach.
    courtyard_waivers = ()
    declared_keepouts = ()
    # #1255: the body-to-edge band the intent declares (None = nothing
    # declared, and the census reports against the derived default without
    # gating), and the refs whose edge it declares.
    body_edge_mm = None
    edge_claim_refs = ()
    if args.intent:
        try:
            from placement.floorplan import load_intent
            _intent = load_intent(args.intent)
            waivers = _intent.waiver_pairs()
            courtyard_waivers = _intent.fixed_pose_courtyard_waivers()
            body_edge_mm = _intent.body_edge_mm()
            edge_claim_refs = _intent.edge_claim_refs()
            # #1098: the intent's keep-outs, so a declared `mating:<ref>`
            # replaces the derived plug region here as it does in the seat
            # predicate.
            declared_keepouts = tuple(_intent.keepouts or ())
        except Exception as exc:
            print(f"cannot load intent {args.intent}: {exc}", file=sys.stderr)
            return 2

    try:
        pcb = parse_kicad_pcb(args.board)
    except Exception as exc:
        print(f"cannot parse {args.board}: {exc}", file=sys.stderr)
        return 2

    _cy_sev_arg = None if args.ignore_project_severity else 'auto'
    g = grade_body_overlap(pcb, clearance, intent_waivers=waivers,
                           pcb_file=args.board,
                           courtyard_severity=_cy_sev_arg,
                           courtyard_waivers=courtyard_waivers)
    leg = grade_pad_legality(pcb, clearance, worst_n=0, pcb_file=args.board,
                             declared_keepouts=declared_keepouts)
    # #697: name any pair graded ABOVE `clearance` and what raised it, or the
    # echo below reports a count the announced floor cannot explain.
    from placement.legality import format_required_clause as _req_clause
    _leg_required = _req_clause(leg)
    _leg_notes = list(leg.get('clearance_notes') or ())

    # COINCIDENT ORIGINS (run-19, measured twice): SW17+SW34+REF_PUCK_R all at
    # one point graded `buildable (blocking 0)` -- the pair currency counts pad
    # INTERSECTIONS, and the rotated pads happened to interleave. A stack of
    # parts at one origin is unbuildable even when no pad intersects, so it is
    # its own blocking channel. The grouping and the exoneration are
    # placement_state's prior art, reused rather than re-derived
    # (`stack_suspects`, the rule assess_placement applies): every pad-bearing
    # footprint is bucketed by round(coord, 3), each bucket is partitioned by
    # physical side (a drilled part is on both), and all-marker side-groups
    # are exonerated (fiducials, mounting holes, testpoints -- mouse-bites and
    # graphics markers are co-located by design and must NOT flag). A bucket is
    # a finding here only when it holds >= 2 suspect NON-marker parts: one real
    # part sitting on a fiducial is not a stack of parts.
    # `coincident_stack_groups` is that rule (fa10 P1: one function, so no
    # two readers can disagree about a stack).
    # #1204: an origin is a library's datum, not a body, so a pair at one
    # origin is a stack only when it shares something PHYSICAL on a face they
    # both occupy -- copper pad boxes (run 19's interleaved pads still
    # overlap), an NPTH hole on copper, or what this grade's body channel
    # found (`body_contact_pairs`). CM5 IO's two module halves share one
    # origin with 34 mm between their pads, and an `overlap_waivers` entry
    # naming a pair now accepts its shared origin as it accepts its
    # courtyard pair.
    from placement.placement_state import (body_contact_pairs,
                                           coincident_stack_groups)
    # Every footprint BLOCK is an entry here since #726: two blocks sharing a
    # reference are keyed `TP4` and `TP4~2`, so two parts at one point form a
    # coincident pair even when they answer to one name. Before that they
    # collapsed to a single entry and this check -- the one that exists to
    # catch exactly that -- was structurally blind to it: run 20's board read
    # `coincident_origins 0` with TWO coincident pairs on it.
    # `duplicate_references` below still reports the naming, which is a
    # schematic question rather than a geometric one.
    stack_exempt: list = []
    stack_groups = coincident_stack_groups(
        pcb, args.board, contacts=body_contact_pairs(g), waivers=waivers,
        exempt_out=stack_exempt)
    # #897: a waiver that resolves to nothing excuses nothing, and said nothing.
    # Formatted by the engine (`format_waiver_warnings`) rather than here, so
    # every caller says the same words. A waiver that accepted a shared
    # origin did excuse something (#1204), so it is not "unused".
    _stack_used = {tuple(p) for e in stack_exempt
                   for p in e.get('waived_pairs') or ()}
    if _stack_used and g.get('waivers_unused'):
        g = dict(g, waivers_unused=[
            p for p in g['waivers_unused']
            if tuple(sorted(p)) not in _stack_used])
    from placement.legality import format_waiver_warnings as _waiver_warnings
    for _line in _waiver_warnings(g):
        print("  " + _line, file=sys.stderr)
    dup_refs = dict(getattr(pcb, 'duplicate_references', None) or {})

    new_advisory = None
    moved_refs = None
    base_pile = False
    own_pad_inherited = None
    if args.baseline:
        try:
            base_pcb = parse_kicad_pcb(args.baseline)
        except Exception as exc:
            print(f"cannot parse baseline {args.baseline}: {exc}",
                  file=sys.stderr)
            return 2
        if not base_pcb.footprints:
            # Every part would read as MOVED against it, and the courtyard
            # gate would arm against all of them (phase-6 verifier: an empty
            # file gated 8 of 8 of pristine ulx3s's pairs).
            print(f"baseline {args.baseline} has no footprints: pass the "
                  f"board this one was derived from", file=sys.stderr)
            return 2
        # A PILE's overlaps are staging, not decisions (#1103, as emit_intent
        # reads no overlap budget off one): no pair on the placed board is
        # excused by having overlapped there, so every advisory pair is new,
        # and the pile's own body grade -- 96% of this check on a 272-part
        # pile (glasgow_revC: 8.9 s of 9.3) -- is not paid for. The courtyard
        # gate's currency (`moved_refs`) is read off the pile as before.
        from placement.placement_state import assess_placement
        base_pile = assess_placement(base_pcb, args.baseline).pile
        if base_pile:
            base_keys = set()
        else:
            gb = grade_body_overlap(base_pcb, clearance, intent_waivers=waivers,
                                    pcb_file=args.baseline,
                                    courtyard_severity=_cy_sev_arg)
            base_keys = {(q.a, q.b, q.kind) for q in gb['pairs']}
        new_advisory = [q for q in g['advisory_pairs']
                        if (q.a, q.b, q.kind) not in base_keys]
        # Refs whose POSE differs from the baseline: the courtyard gate's
        # currency (`legality.moved_refs` says why membership is not enough).
        # A function in legality, so a mover can ask the same question.
        moved_refs = legality.moved_refs(pcb, base_pcb)
        # #1306: the own-pad shorts the baseline's footprints already had.
        own_pad_inherited = {
            (r['ref'], r['index_a'], r['index_b'])
            for r in legality.own_pad_channel(base_pcb)['own_pad_contact_rows']
            if r['short']}

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
        print(f"    Every block is audited ({len(pcb.footprints)} parts): the "
              f"later ones are keyed with an ordinal suffix, so "
              f"`coincident_origins` above compares them. Legal in KiCad, but "
              f"rename them if they are meant to be distinct parts -- two "
              f"parts answering to one name cannot be told apart on a BOM, on "
              f"a pick-and-place file, or by `--lock`.")
    print(f"  blocking {g['blocking']}  advisory {g['advisory']}"
          f"  waived {g['waived']}  contained {g['contained']}"
          f"  courtyard_blocking {g['courtyard_blocking']}"
          + (f"  new-vs-baseline {len(new_advisory)}"
             if new_advisory is not None else ""))
    if base_pile:
        print(f"  baseline {args.baseline} is a pile: its overlaps are staging, "
              f"not a baseline (#1103), so every advisory pair counts as new")
    _cb_keys = {(q.a, q.b) for q in g['courtyard_blocking_pairs']}
    _body_src = g.get('body_sources') or {}
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
        # #896. Name the geometry the claim rests on. `kind` says which
        # CHANNEL judged the pair; it does not say whether the body came from
        # a drawn .Fab outline or from silkscreen, and those are claims of
        # very different strength -- a silk body may be an assembly marking,
        # which is why it never gates.
        kind = q.kind
        if kind == 'fab':
            _sa, _sb = _body_src.get(q.a, ''), _body_src.get(q.b, '')
            if 'silk' in (_sa, _sb):
                kind = f'body({_sa}/{_sb})'
        print(f"    {q.a} <-> {q.b}  {kind}  {q.area_mm2}mm2 "
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
    if stack_exempt:
        # #1204: said, never silent -- an origin shared without a stack.
        print(f"  SHARED ORIGINS, not stacks ({len(stack_exempt)}): no copper "
              f"pad box, hole or drawn body shared on a face both occupy, or "
              f"the pair is in the intent's overlap_waivers")
        for grp in stack_exempt:
            _w = ('waived: ' + ', '.join('<->'.join(p) for p in
                                         grp.get('waived_pairs') or ())
                  if grp['why'] == 'waived' else 'apart')
            print(f"    {' '.join(grp['refs'])} @ "
                  f"({grp['point'][0]}, {grp['point'][1]})  [{_w}]")

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
    # BOTH off-outline channels, side by side, whenever either fires (#937).
    # The line above is the part AABB inflated by the grading clearance; this
    # is the per-PAD measure at margin 0, which is the one CLAUDE.md
    # designates for the top-priority placement defect. Printed together
    # because their DISAGREEMENT is the useful signal: a coarse hit with an
    # empty precise list is the bounding box of an edge-mounted part, not
    # copper in the air, and a reader who sees only the count cannot tell.
    _exact = leg.get('oob_pad_copper_refs') or []
    _overrun = leg.get('oob_pad_copper_overrun_mm') or {}
    if leg['oob_pad_count'] or _exact:
        if _exact:
            # #1096: the DISTANCE the copper reaches past the outline. The
            # magnitude in `oob_pad_copper_refs` is a ranking sum (C20 read
            # "36.8mm" for copper 7.84 mm out) and is kept in the JSON.
            _gate = set(leg.get('oob_pad_copper_gating_refs') or ())
            if _gate:
                print("    pad copper genuinely off the outline (per-pad, "
                      "margin 0): "
                      + ', '.join(f'{r} ({_overrun.get(r, a)}mm past the '
                                  f'outline)' for r, a in _exact if r in _gate)
                      + (" -- NOT BUILDABLE: a part not on the board cannot "
                         "be assembled or routed" if _gate else ''))
            _edge = [r for r, _a in _exact if r not in _gate]
            if _edge:
                print("    ...on the outline by design, not gated: "
                      + ', '.join(_edge) + " (castellated pads, or a round "
                      "pad whose bounding box, not its copper, crosses)")
        else:
            print("    ...but NO PAD crosses the real outline (per-pad, "
                  "margin 0, is empty). The count above is the part's "
                  "bounding box against an outline inflated by the grading "
                  "clearance -- an edge-mounted part reports a breach its "
                  "copper does not make.")
    # #962: the second off-outline channel, footprint GRAPHIC copper (a drawn
    # SOT-89 tab, an antenna). Pads can all be inside while the tab hangs off
    # the board: esp_prog U2 at 115.34 reported blocking 0 with its tab
    # 1.11 mm past the outline. Printed, and in JSON; it is not a
    # `not_buildable` conjunct (#937's decision, which #1096 reversed for the
    # PAD channel above only), and check_drc grades it as graphic-off-board.
    _g_refs = leg.get('oob_graphic_copper_refs') or []
    if _g_refs:
        print("    footprint GRAPHIC copper past the outline (margin 0): "
              + ', '.join(f'{r} ({a}mm)' for r, a in _g_refs))
    _g_un = leg.get('oob_graphic_copper_unmeasured') or []
    if _g_un:
        print("    footprint copper NOT measured against the outline: "
              + ', '.join(f'{u[0]} ({u[1]})' for u in _g_un[:6]))
    # #1031: pads in a board rule-area keep-out band, where the router can
    # land no track. Printed and in JSON; like the two channels above it is
    # NOT a `not_buildable` conjunct (#937) -- place_pose and
    # render_placement --gate are what gate on it.
    _k_pads = leg.get('keepout_copper_pads') or []
    if _k_pads:
        print(f"    pads in a rule-area keep-out band "
              f"({leg.get('keepout_copper_band_mm')}mm band, no track can "
              f"land): "
              + ', '.join(f'{r[0]}.{r[1]} {r[2]} ({r[3]}mm)'
                          for r in _k_pads[:12])
              + (f" ... +{len(_k_pads) - 12} more" if len(_k_pads) > 12
                 else ''))
    _k_tht = leg.get('keepout_copper_tht_refs') or []
    if _k_tht:
        print("    through-hole pads in a keep-out band, reachable on an "
              "uncovered layer (reported, not failed): "
              + ', '.join(sorted({f'{r[0]}.{r[1]}' for r in _k_tht})[:12]))
    # ONE predicate, used verbatim at all three sites (verdict, JSON
    # `buildable`, exit code). Three re-derivations of `blocking or
    # locked_contact` is how the coincident-origin channel would have reached
    # two of them and silently missed the third.
    if g['contained']:
        print(f"  CONTAINMENT ({g['contained']}): a part's drawn body (its "
              f".Fab outline, else its silk united with its pads) lies "
              f"wholly or mostly inside another part's.")
        for q in g['containment_pairs']:
            # #1201: the reason a pair does not gate, by name -- a silk-drawn
            # body is not "a marker or a board-sized container".
            _ex = getattr(q, 'gate_exempt', '')
            tag = (f"  [does not gate: {_GATE_EXEMPT_WHY.get(_ex, _ex)}]"
                   if _ex else
                   f"  [waived:{q.waiver}, still gates]" if q.waived else '')
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
            _why = {}
            for q in g['containment_pairs']:
                _ex = getattr(q, 'gate_exempt', '')
                _why[_ex] = _why.get(_ex, 0) + 1
            print("    None of these BLOCK: " + '; '.join(
                f"{n} {_GATE_EXEMPT_WHY.get(k, k)}"
                for k, n in sorted(_why.items())) + '.')
            if set(_why) <= {'marker_class', 'container_class'}:
                print(f"    Each is a by-design containment (a marker or a "
                      f"board-sized container), which the corpus ships "
                      f"legitimately -- orangecrab FID2/J5 at 100%.")
    # BODY COVERAGE (#896). Printed UNCONDITIONALLY, including the fully
    # covered case: "which geometry was this board graded on" is a fact about
    # every run, and a line that appears only when something is missing cannot
    # tell a reader that a board was judged on silk rather than on drawn
    # bodies. The mix comes from `grade_body_overlap`'s own `body_sources`, so
    # the coverage claim and the geometry cannot drift apart.
    _srcs = g.get('body_sources') or {}
    _mix = {}
    for _v in _srcs.values():
        _mix[_v] = _mix.get(_v, 0) + 1
    _judged = ', '.join(f"{_mix[k]} {k}" for k in ('fab', 'silk')
                        if _mix.get(k))
    print(f"  BODY COVERAGE: {len(_srcs)} of {len(pcb.footprints)} part(s) "
          f"draw a body the containment channel can judge"
          + (f" ({_judged})" if _judged else ""))
    if _mix.get('silk'):
        print(f"    A silk body is the LAST resort and never gates: a "
              f"library may draw an assembly outline there rather than the "
              f"part (esp_prog's SOT89 draws corner brackets 5.2mm apart "
              f"around a 4.5mm part). Such pairs are reported, with their "
              f"source, and excluded from the blocking channels.")
    _fr = g.get('fab_rejected_refs') or []
    from placement.body import FAB_MIN_PAD_AREA_FRAC as _FAB_FRAC
    if _fr:
        print(f"    {len(_fr)} part(s) draw only a .Fab marking (under "
              f"{_FAB_FRAC:.0%} of the pad box: a pin-1 dot, a single line; "
              f"#1201), which is not their body: their drawn body is their "
              f"silk where they draw one, and what they occupy is unchanged "
              f"(their courtyard, else the marking united with their pads): "
              + ', '.join(_fr[:8]) + (' ...' if len(_fr) > 8 else ''))
    if g['fab_unjudged']:
        _u = g['fab_unjudged_refs']
        print(f"    {g['fab_unjudged']} part(s) draw no usable body (no .Fab "
              f"body, no usable silk), so the channel cannot judge them: "
              + ', '.join(_u[:8]) + (' ...' if len(_u) > 8 else ''))

    # ASSEMBLY SIDES (#837). Report-only, and deliberately not a conjunct:
    # which faces a board is populated on is a FACT about the board, not a
    # defect, and the verdict predicate below is untouched. It is here because
    # this is the tool named `check_assembly` and it could not say how many
    # reflow passes the board it just graded would take.
    #
    # Printed unconditionally, including the all-on-one-face case: a section
    # that appears only when there is something on the back would make
    # "single-sided" and "not measured" look identical.
    _cen = assembly_census(pcb)
    _pb, _bl = _cen['pad_bearing'], _cen['blocks']
    print(f"  ASSEMBLY SIDES: F {_pb['F']} / B {_pb['B']} pad-bearing part(s)"
          f" of {_bl['F'] + _bl['B']} block(s); "
          f"{_cen['reflow_passes']} reflow pass(es), "
          f"{_cen['through_hole']} through-hole part(s)")
    # BOTH faces' zero-pad blocks, so the printed arithmetic CLOSES. Listing
    # only the back left interf_u reporting "24 pad-bearing of 25 blocks" with
    # nothing to explain the 25th (a zero-pad graphic on the FRONT), and
    # glasgow 6 blocks short. A census a reader cannot reconcile is a census
    # they have to trust.
    for _side in ('F', 'B'):
        _z = _cen['zero_pad'][_side]
        if not _z:
            continue
        # The distinction that makes esp_prog single-sided: three OLIMEX logo
        # footprints sit on B.Cu carrying no pads at all. Counting blocks it
        # is a two-sided board; counting copper it is not, and the fab builds
        # the second one. Named by FOOTPRINT where the reference is a bare
        # uuid -- esp_prog's three are `#00000000-...`, which tells a reader
        # nothing about what they are being asked to ignore.
        _named = []
        for _r in _z[:8]:
            _f = pcb.footprints.get(_r)
            _n = (getattr(_f, 'footprint_name', '') or '').split(':')[-1]
            _named.append(f"{_r} ({_n})" if _n and _r.startswith('#') else _r)
        print(f"    {len(_z)} {_side}-side block(s) carry NO pads and are "
              f"excluded from the verdict: "
              + ', '.join(_named) + (' ...' if len(_z) > 8 else ''))
    if _cen['through_hole']:
        _t = _cen['through_hole_by_side']
        # `pad_is_plated_through`, never bare `drill > 0`: an NPTH hole is an
        # alignment post or a screw hole, not a soldered pin. Counting those
        # told the reader watchy has 5 hand-soldered parts when it has 1 --
        # SW1-SW4 are SMD switches with unplated alignment posts.
        print(f"    Through-hole F {_t['F']} / B {_t['B']} (plated barrels "
              f"only), counted and never folded in: a drilled part on the "
              f"front needs wave, selective or hand soldering, not a second "
              f"reflow pass.")
        if _cen['sides'] != 'both':
            # Only where the two answers actually DIFFER. On a board the
            # census already calls two-sided this sentence implies a
            # disagreement that does not exist.
            print(f"      (`sides_occupied` answers the OBSTRUCTION question "
                  f"instead, and would call this board two-sided.)")
    if sum(_cen['unsoldered'].values()):
        _u = _cen['unsoldered']
        print(f"    Unsoldered F {_u['F']} / B {_u['B']}: every pad an "
              f"unplated hole -- mounting, tooling or alignment. No process "
              f"attaches these, so they are in neither count above.")
    if not sum(_pb.values()):
        # Guarded on PAD-BEARING parts, not on blocks: a board of nothing but
        # graphics is assembled by no process at all, and the through-hole
        # sentence below would have described it as assembled entirely by one.
        print(f"    No pad-bearing parts at all, so there is no assembly "
              f"policy to observe.")
    elif not _cen['reflow_passes']:
        # flat_hierarchy: 58 through-hole parts and 6 NPTH mounting holes.
        # Counting POPULATED faces would report one reflow pass for a board
        # that gets none.
        print(f"    No SMD parts at all, so 0 reflow passes -- this board is "
              f"assembled entirely by through-hole process.")
    # The basis in one clause here and in full in the JSON. It has to appear
    # in BOTH: a per-side number quoted without its counting rule cannot be
    # checked against any other one (#726 moved ulx3s from 234 blocks to 235),
    # and a reader of the text channel never sees the JSON.
    print(f"    observed policy: sides={_cen['sides']} -- counted per "
          f"footprint BLOCK, verdict from the pad-bearing subset "
          f"(parts_by_side_basis in --json has the rule in full)")

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
    # #1095: the board's own severity for KiCad's courtyard rule. Said
    # whenever it waived anything, with the file it came from, because a
    # courtyard census that silently shrank would read as a fix.
    _cy_w = g.get('courtyard_severity_waiver') or ''
    _cy_basis = g.get('courtyard_severity_basis') or ''
    if _cy_basis.startswith('legacy severity plan'):
        print(f"  courtyard severity: graded at error -- {_cy_basis}.")
    if _cy_w:
        _n_sev = sum(1 for q in g['pairs']
                     if q.kind == 'courtyard' and q.waiver == _cy_w)
        print(f"  courtyard severity: the project sets courtyards_overlap to "
              f"'{g['courtyard_severity']}' "
              f"({os.path.splitext(args.board)[0]}.kicad_pro), so {_n_sev} "
              f"courtyard pair(s) are waived '{_cy_w}' and none gates -- "
              f"KiCad's own DRC reports none of them. Fab containment, pad "
              f"and locked-contact channels are graded as usual. "
              f"--ignore-project-severity grades them at error.")
    # #1298: the courtyard pairs a DECLARED pose accepted by name -- waived,
    # and said so, never silently. A pair can only get here from the intent
    # (`accept_courtyard_overlap`), which check_floorplan --emit-intent
    # writes, with a warning, for declared poses that overlap each other.
    fixed_waived = [q for q in g['pairs']
                    if q.waiver == legality.FIXED_POSE_WAIVER]
    if fixed_waived:
        print(f"  COURTYARD ACCEPTED BY A DECLARED POSE ({len(fixed_waived)}): "
              f"waived, named in the intent's "
              f"fixed_poses[].accept_courtyard_overlap -- courtyard only; a "
              f"drawn-body containment, pads and holes keep their gates")
        for q in fixed_waived:
            _what = (f"pin(s) {', '.join(q.pins)} under its courtyard"
                     if q.kind == 'pin_in_courtyard'
                     else f"depth {q.depth_mm}mm")
            print(f"    {q.a} <-> {q.b}  {q.kind}  {q.area_mm2}mm2  {_what}  "
                  f"side {q.side}")
    courtyard_gating = []
    if g['courtyard_blocking'] and moved_refs is not None:
        courtyard_gating = legality.courtyard_gating(
            g['courtyard_blocking_pairs'], moved_refs)
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
        print("    A deliberate overlap is accepted by naming the pair in the "
              "intent's `overlap_waivers`, where the acceptance is visible.")

    # #1250: a TEST POINT whose pad copper lies under another part's drawn
    # body -- no probe or bed-of-nails pin can reach it. The marker waiver
    # every other channel gives a test point (courtyard `blocking_waived`,
    # containment `_GATE_EXEMPT`) looks only at whether the test point is in
    # someone's way, never at whether something now sits over it: glasgow's
    # TP6 moved into U19's TSSOP body read `buildable`. Gated on the courtyard
    # gate's currency (a member MOVED vs --baseline), because orangecrab ships
    # TP26/TP27 under J4 by design; an author accepts a pair by naming it in
    # the intent's `overlap_waivers`. A silk-drawn cover never gates (#896).
    tp_cov = list(g.get('testpoint_covered') or [])
    tp_gating = (legality.testpoint_covered_gating(tp_cov, moved_refs)
                 if moved_refs is not None else [])
    if tp_cov:
        _gate_note = (
            f"{len(tp_gating)} of {len(tp_cov)} GATE (a member moved vs the "
            f"baseline)" if moved_refs is not None
            else "REPORT-ONLY: pass --baseline <the board the run started "
                 "from> to gate the ones your moves created")
        print(f"  TEST POINT COVERED ({len(tp_cov)}): a test point's pad "
              f"copper lies under another part's drawn body, where no probe "
              f"can reach it -- {_gate_note}")
        _gk = {(t.testpoint, t.cover) for t in tp_gating}
        for t in tp_cov:
            if not t.gates_if_moved:
                _why = (f'  waived:{t.waiver}' if t.waiver
                        else '  never gates: the body is drawn on silk')
            elif moved_refs is None:
                _why = ''
            elif (t.testpoint, t.cover) in _gk:
                _who = [r for r in (t.testpoint, t.cover) if r in moved_refs]
                _why = '  GATES (moved: ' + ' '.join(_who) + ')'
            else:
                _why = "  baseline's own (no member moved)"
            print(f"    {t.testpoint} under {t.cover}  {t.frac:.0%} of its "
                  f"pad copper  body {t.body_source}  side {t.side}{_why}")
        print("    A deliberate pose is accepted by naming the pair in the "
              "intent's `overlap_waivers`.")

    # #1255: each pad-bearing part's BODY against the real outline at margin
    # 0. An assembler's rule, not the board's: JLCPCB asks 2.5 mm and adds
    # edge rails when the file has none, and every human board measured
    # breaks it -- so this GATES only when the intent declares
    # `assembly.body_edge_mm`, and then every part in the band gates.
    # Printed unconditionally, like BODY COVERAGE: a section that appears
    # only when something is in the band cannot say what was measured.
    band = legality.body_edge_band(pcb, args.board,
                                   threshold_mm=body_edge_mm,
                                   edge_claim_refs=edge_claim_refs,
                                   moved=moved_refs)
    edge_band_gating = []
    if band['gates']:
        edge_band_gating = list(band['in_band'])
        if band['error']:
            # Declared and unmeasurable: fail closed, and say why.
            edge_band_gating = [{'ref': '?', 'mm': None, 'source': None,
                                 'side': None, 'unmeasured': band['error']}]
    if band['error']:
        print(f"  BODY-TO-EDGE BAND NOT MEASURED -- {band['error']}"
              + (" -- NOT BUILDABLE until it is" if band['gates'] else ''))
    else:
        _basis = ('declared in the intent (assembly.body_edge_mm) -- GATES'
                  if band['gates'] else
                  'derived_default: JLCPCB\'s assembly rule, one '
                  'assembler\'s -- REPORT-ONLY; declare '
                  'assembly.body_edge_mm in the intent to gate')
        print(f"  BODY-TO-EDGE BAND: {len(band['in_band'])} of "
              f"{band['eligible']} part(s) have a body within "
              f"{band['threshold_mm']:g}mm of the board edge [{_basis}]"
              + (f"  new_in_band {band['new_in_band']}"
                 if band['new_in_band'] is not None else ''))
        _new = set(band['new_in_band_refs'] or ())
        for r in band['in_band'][:12]:
            print(f"    {r['ref']} {r['mm']}mm ({r['source']}, side "
                  f"{r['side']})" + ('  MOVED' if r['ref'] in _new else ''))
        if len(band['in_band']) > 12:
            print(f"    ... +{len(band['in_band']) - 12} more (--json)")
        if band['excluded']:
            print("    in the band but belonging at the edge, never counted "
                  "(edge class, marker, or an edge the intent declares): "
                  + ', '.join(f"{r['ref']} {r['mm']}mm [{r['why']}]"
                              for r in band['excluded'][:8])
                  + (' ...' if len(band['excluded']) > 8 else ''))
        if band['unmeasured']:
            print("    no body, courtyard or pad box to measure: "
                  + ', '.join(band['unmeasured'][:8]))

    # A FOURTH conjunct, and `g['blocking']` is deliberately NOT touched.
    # `blocking` means "pad intersections" to board_score; folding
    # containment into that count would change it.
    # This is the same shape the coincident-origin channel used.
    # `courtyard_gating` is the FIFTH conjunct (run-23): the moved-vs-baseline
    # subset of the courtyard census -- see the currency comment above for
    # why the absolute census must not gate.
    # #1096, the SIXTH conjunct: pad copper wholly or partly off the real
    # outline, per pad at margin 0. #937 kept it out of the verdict as "the
    # wrong channel" and run 36 then routed a board with C20 7.84 mm below
    # its south edge on a `buildable` -- the router took GND off the board to
    # reach it. CLAUDE.md ranks this the top-priority placement defect; a
    # part that is not on the board cannot be built. A lock does not exempt
    # it (placement stamps locks itself, #962's reasoning).
    # The GATING subset: a real distance past the outline on the true pad
    # outlines, castellated pads left out (rp2350's Teensy U8 is ON the edge
    # by design).
    off_outline_pads = list(leg.get('oob_pad_copper_gating_refs') or [])
    # #1098, the SEVENTH: a part on a PCB-edge plug's mating region (run 36
    # put 8 back-side parts on StickHub's USB tongue, which must enter a
    # socket). Absolute, and no class waiver reaches it: the region is the
    # plug's own courtyard, and whatever sits there cannot be plugged in.
    mating = leg.get('mating_keepout_refs') or []
    if leg.get('mating_keepout_error'):
        # Unmeasured is not clean: fail closed, and say why.
        print(f"  PLUG MATING REGION NOT MEASURED -- "
              f"{leg['mating_keepout_error']} -- NOT BUILDABLE until it is")
        mating = mating or [{'ref': '?', 'keepout': 'unmeasured',
                             'side': '?', 'area_mm2': 0.0}]
    if mating:
        print(f"  ON A PLUG'S MATING REGION ({len(mating)}): these must "
              f"enter the socket with the plug -- NOT BUILDABLE")
        for m in mating:
            print(f"    {m['ref']} ({m['side']}) in {m['keepout']}  "
                  f"{m['area_mm2']}mm2")
    # #1212, the EIGHTH conjunct: a pin frame's (rp2350's Teensy U8) drilled
    # pins inside another part's courtyard -- KiCad's pth_inside_courtyard.
    # The frame's RECT left the courtyard channel (it flagged every part
    # inside it and could not find the real hit); its pins are graded here,
    # absolutely, as KiCad grades them: 0 on both healthy frame boards.
    pin_hits = list(g.get('pin_in_courtyard_pairs') or [])
    if g.get('containers'):
        _kinds = ', '.join(f"{r} ({k})"
                           for r, k in sorted(g['containers'].items()))
        print(f"  CONTAINERS: {_kinds} -- a pin frame is graded on its "
              f"pins, an outline is waived, neither on its rect")
    if pin_hits:
        # One finding per part pair AND hole rule (a part over a frame's
        # PTH and NPTH pins is two KiCad rules): the count says both.
        _pin_parts = len({(q.a, q.b) for q in pin_hits})
        print(f"  PIN IN COURTYARD ({_pin_parts} part pair(s), "
              f"{len(pin_hits)} by hole rule): a frame's drilled pin lies "
              f"inside another part's courtyard (KiCad: "
              f"pth_inside_courtyard / npth_inside_courtyard) -- NOT "
              f"BUILDABLE")
        for q in pin_hits:
            frame, other = ((q.a, q.b) if q.a in g['containers']
                            else (q.b, q.a))
            _kind = (q.hole or 'pth').upper()
            print(f"    {other} over {frame} {_kind} pin(s) "
                  f"{', '.join(x or f'(unnumbered {_kind})' for x in q.pins)}"
                  f"  {q.area_mm2}mm2  side {q.side}")
    # #1266, the ELEVENTH: a part's pad copper touching ANOTHER part's
    # footprint graphic copper (esp_prog C3 placed on U2's SOT-89 tab, one pad
    # across its edge and one wholly inside it, graded `buildable`). Two
    # parts' copper in one place, any net but a net tie's own (`stack`);
    # measured by check_drc's `footprint_graphic_pad_census`, which place_pose
    # and the seat gate also read. The courtyard gate's currency: a pair
    # GATES when a member MOVED vs --baseline, and without one it is
    # REPORT-ONLY, because healthy human boards ship such contacts by design
    # -- measured over the stress manifests, 3 boards: ZYNQ_Ruler som's JP2
    # on antenna AE2's feed copper, DIN_41612_Backplane's Kelvin ties on
    # R3/R21 (a net tie's own net, never a stack), OpenWeatherStation's
    # abutting ties NT1-NT4.
    graphic_contacts = [r for r in (leg.get('graphic_pad_rows') or [])
                        if r['contact']]
    graphic_stack_pairs = sorted({(r['ref'], r['owner_ref'])
                                  for r in graphic_contacts if r['stack']})
    graphic_gating = ([p for p in graphic_stack_pairs
                       if p[0] in moved_refs or p[1] in moved_refs]
                      if moved_refs is not None else [])
    if graphic_contacts:
        _gate = set(graphic_gating)
        print(f"  PAD ON ANOTHER PART'S COPPER ({len(graphic_stack_pairs)} "
              f"part pair(s), "
              + (f"{len(graphic_gating)} GATE (a member moved vs the "
                 f"baseline)" if moved_refs is not None
                 else "REPORT-ONLY: pass --baseline <the board the run "
                      "started from> to gate")
              + "): a pad touches another part's footprint graphic copper "
                "(a drawn tab, an antenna)"
              + (" -- NOT BUILDABLE" if graphic_gating else ''))
        for r in graphic_contacts:
            p = (r['ref'], r['owner_ref'])
            _why = ("  [a net tie's own net: what the tie is for, never "
                    "gates]" if not r['stack']
                    else "  baseline's own (no member moved)"
                    if moved_refs is not None and p not in _gate else '')
            print(f"    {r['pad_ref']} ({r['net'] or 'no net'}) on "
                  f"{r['owner_ref']}'s copper, {r['layer']}"
                  + (", wholly inside the filled shape"
                     if r['inside_filled'] else '') + _why)
    _gp_un = leg.get('graphic_pad_unmeasured') or []
    if _gp_un:
        print("    footprint copper NOT measured against other parts' pads: "
              + ', '.join(f"{u['owner_ref']} ({u['kind']})"
                          for u in _gp_un[:6]))
    # #1306, the TWELFTH: two of ONE part's pads touching on two different
    # real nets -- KiCad's shorting_items. Every channel above is between two
    # parts, so steppenprobe U1, whose pads the placement writer left at
    # their old angle (#1305), read `buildable` with 164 such pairs. Measured
    # by check_drc's `footprint_own_pad_census`, which place_pose reads too.
    # Absolute without --baseline: over the stress corpus's 478 human boards
    # one ships any (bldc_tester's two Kelvin shunts, which declare no
    # net-tie group), and with --baseline a pair its footprint already had
    # there is that board's own and does not gate. A contact with a no-net
    # pad (a shield tab, a thermal-via pad: 21 pairs on 3 of those boards)
    # or between two pins' `unconnected-(...)` nets (cm4_edge_ai's USB-C
    # VBUS, one pin drawn as four pads) is listed and never gates: it
    # shorts no signal.
    own_contacts = leg.get('own_pad_contact_rows') or []
    own_shorts = [r for r in own_contacts if r['short']]
    own_gating = [r for r in own_shorts
                  if own_pad_inherited is None
                  or (r['ref'], r['index_a'], r['index_b'])
                  not in own_pad_inherited]
    if own_contacts:
        print(f"  OWN PADS TOUCHING ({len(own_contacts)} pair(s) of one part's "
              f"pads on two nets; {len(own_shorts)} short two real nets "
              f"(KiCad: shorting_items), {len(own_gating)} GATE"
              + (" -- a pair the --baseline footprint did not have"
                 if own_pad_inherited is not None
                 else "; pass --baseline to exempt a pair the board started "
                      "with")
              + ")" + (" -- NOT BUILDABLE" if own_gating else ''))
        _gate_ids = {id(r) for r in own_gating}
        for r in (own_gating + [r for r in own_contacts
                                if id(r) not in _gate_ids])[:20]:
            _why = ('' if id(r) in _gate_ids
                    else '  [a no-net pad: shorts no net, never gates]'
                    if r['no_net']
                    else '  [two unconnected pins: shorts no signal, never '
                         'gates]' if r['floating'] else "  baseline's own")
            print(f"    {r['ref']}.{r['pad_a']} ({r['net_a'] or 'no net'}) <-> "
                  f"{r['ref']}.{r['pad_b']} ({r['net_b'] or 'no net'}), "
                  f"{r['layer']}{_why}")
        if len(own_contacts) > 20:
            print(f"    ... and {len(own_contacts) - 20} more (--json lists "
                  f"every one)")
    # #1250, the NINTH: a test point a move left under another part's body
    # (`tp_gating`, above). #1255, the TENTH: a body inside the body-to-edge
    # band the intent DECLARES (`edge_band_gating`, empty when undeclared).
    not_buildable = bool(g['blocking'] or locked_contact or stack_groups
                         or g['containment_blocking']
                         or courtyard_gating or off_outline_pads or mating
                         or graphic_gating
                         or pin_hits or tp_gating or edge_band_gating
                         or own_gating)
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
            # The BLOCKING subset, published beside the total for exactly the
            # reason `buildable` is published above: it is one of the five
            # conjuncts of the verdict, and a consumer that wants to say WHICH
            # conjunct fired otherwise has only `contained` -- which counts the
            # by-design containments the corpus ships legitimately (orangecrab
            # FID2/J5 at 100%) and so names a defect where there is none. The
            # number has existed in the grade dict since the channel was added
            # and has decided the verdict at its `not_buildable` line ever since; it just never
            # reached a reader (#918).
            'containment_blocking': g['containment_blocking'],
            'containments': [q._asdict() for q in g['containment_pairs']],
            # #1212 / #1184: the containers (`legality.container_kinds`) and
            # the eighth conjunct -- a pin frame's holes inside another
            # part's courtyard, graded absolutely.
            'containers': dict(g.get('containers') or {}),
            'pin_in_courtyard': len(pin_hits),
            'pin_in_courtyard_pairs': [q._asdict() for q in pin_hits],
            'fab_unjudged': g['fab_unjudged'],
            'fab_unjudged_refs': g['fab_unjudged_refs'],
            # #1201: parts whose .Fab drawing is a marking, not a body.
            'fab_rejected_refs': list(g.get('fab_rejected_refs') or []),
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
            # #1095: the severity the courtyard channel was graded at (None
            # = KiCad's default, error) and the waiver label it gave.
            'courtyard_severity': g.get('courtyard_severity'),
            'courtyard_severity_basis': g.get('courtyard_severity_basis'),
            'courtyard_severity_waiver': g.get('courtyard_severity_waiver')
            or None,
            'courtyard_gating_basis': ('moved-vs-baseline'
                                       if moved_refs is not None
                                       else 'no-baseline: report-only'),
            'courtyard_pairs': [q._asdict() for q in g['pairs']
                                if q.kind == 'courtyard'],
            'courtyard_advisory': sum(
                1 for q in g['advisory_pairs'] if q.kind == 'courtyard'
                and q not in g['courtyard_blocking_pairs']),
            'courtyard_synthetic_refs': g['courtyard_synthetic_refs'],
            # #1298: courtyard / pin pairs waived because a declared pose
            # accepts them by name (`fixed_poses[].accept_courtyard_overlap`),
            # and the accepted pairs that named nothing or matched nothing.
            'courtyard_fixed_pose_waived': [q._asdict() for q in fixed_waived],
            'courtyard_waivers_unresolved':
                g.get('courtyard_waivers_unresolved') or [],
            'courtyard_waivers_unused': g.get('courtyard_waivers_unused') or [],
            # #1250: test points whose pad copper lies under another part's
            # drawn body, every one (`testpoint_covered_pairs`), and the
            # subset that GATES -- a member moved vs --baseline. Like
            # `courtyard_blocking_gating`, None (not 0) without a baseline.
            'testpoint_covered': len(tp_cov),
            'testpoint_covered_pairs': [t._asdict() for t in tp_cov],
            'testpoint_covered_gating': (len(tp_gating)
                                         if moved_refs is not None else None),
            'testpoint_covered_gating_pairs': [t._asdict()
                                               for t in tp_gating],
            'testpoint_covered_basis': (
                'a test point (part_class testpoint) whose copper pads, as '
                'boxes about their centres (legality.bodyless_pad_shape), lie '
                'at least CONTAINMENT_FRAC under another same-face part\'s '
                'drawn body (fab, else silk). Gates when a member moved vs '
                '--baseline, the body is .Fab, and the pair is not in the '
                'intent\'s overlap_waivers or a container\'s. Fiducials are '
                'not graded here.'),
            # #1255: every pad-bearing part's body against the real outline
            # at margin 0 (`legality.body_edge_band`: threshold and its basis,
            # in_band / excluded rows, eligible and measured counts,
            # new_in_band with --baseline). `body_edge_band_gating` is the
            # count that GATES: the in-band parts when the intent declares
            # `assembly.body_edge_mm`, else 0 -- the derived default (JLCPCB's
            # 2.5 mm) is reported and never gates.
            'body_edge_band': band,
            'body_edge_band_gating': len(edge_band_gating),
            'blocking_pairs': [q._asdict() for q in g['blocking_pairs']],
            'advisory_pairs': [q._asdict() for q in g['advisory_pairs']],
            'pad_conflicts': leg['pad_conflicts'],
            'pad_edge': leg['pad_edge'],
            'pad_edge_conflicts': leg['pad_edge_conflicts'],
            'pad_edge_shortfall': leg['pad_edge_shortfall'],
            'pad_edge_unmeasured': leg['pad_edge_unmeasured'],
            'pad_clearance_required': leg.get('required') or [],
            'hole_conflicts': leg['hole_conflicts'],
            'oob_pad_count': leg['oob_pad_count'],
            'oob_pad_amount': leg['oob_pad_amount'],
            # The MACHINE path, which is the one that matters here: this doc
            # is what the retired loop_driver's L2 gate read, refusing with "N
            # part(s) carry pad copper OFF the board -- their nets cannot be
            # routed at all". It could not name the part, and the count it
            # gates on moves with --clearance, so a clearance-band graze reads
            # as copper in the air. Both facts now travel with the number.
            'oob_pad_refs': leg.get('oob_pad_refs') or [],
            'oob_pad_basis': leg.get('oob_pad_basis'),
            # The PER-PAD channel beside the AABB one (#937). The retired
            # loop_driver's L2 gate read `oob_pad_count` and was right to -- it
            # was justified over 119 graded rows -- but a consumer holding only
            # this document could not tell a real off-outline pad from the
            # bounding box of an edge part, and the refusal it writes says
            # "their nets cannot be routed at all", which is true of one and
            # not the other. Both keys travel; neither replaces the other.
            'oob_pad_copper_count': leg.get('oob_pad_copper_count', 0),
            'oob_pad_copper_refs': leg.get('oob_pad_copper_refs') or [],
            'oob_pad_copper_overrun_mm':
                leg.get('oob_pad_copper_overrun_mm') or {},
            'oob_pad_copper_gating_count':
                leg.get('oob_pad_copper_gating_count', 0),
            'oob_pad_copper_gating_refs':
                leg.get('oob_pad_copper_gating_refs') or [],
            'mating_keepout_count': leg.get('mating_keepout_count', 0),
            'mating_keepout_refs': leg.get('mating_keepout_refs') or [],
            'oob_pad_copper_basis': leg.get('oob_pad_copper_basis'),
            # #962: footprint GRAPHIC copper against the outline -- the second
            # off-outline channel, same non-gating contract as the pad one.
            'oob_graphic_copper_count': leg.get('oob_graphic_copper_count', 0),
            'oob_graphic_copper_amount': leg.get('oob_graphic_copper_amount', 0.0),
            'oob_graphic_copper_refs': leg.get('oob_graphic_copper_refs') or [],
            'oob_graphic_copper_waived': leg.get('oob_graphic_copper_waived') or [],
            'oob_graphic_copper_unmeasured': leg.get('oob_graphic_copper_unmeasured') or [],
            'oob_graphic_copper_basis': leg.get('oob_graphic_copper_basis'),
            'graphic_edge_shortfall_refs': leg.get('graphic_edge_shortfall_refs') or [],
            # #1266: pads touching another part's graphic copper -- a
            # conjunct. `_stack_pairs` are the `[pad part, copper owner]`
            # contacts that count, `_gating_pairs` the subset with a member
            # MOVED vs --baseline: None (not []) without one, as
            # `courtyard_blocking_gating` is.
            'graphic_pad_gating': (len(graphic_gating)
                                   if moved_refs is not None else None),
            'graphic_pad_contacts': graphic_contacts,
            'graphic_pad_stack_pairs': [list(p) for p in graphic_stack_pairs],
            'graphic_pad_gating_pairs': ([list(p) for p in graphic_gating]
                                         if moved_refs is not None else None),
            'graphic_pad_unmeasured': _gp_un,
            'graphic_pad_basis': leg.get('graphic_pad_basis'),
            # #1306: one part's own pads touching on two real nets -- a
            # conjunct, measured (an int) with or without --baseline, which
            # only exempts the pairs its footprint already had.
            'own_pad_shorts': len(own_gating),
            'own_pad_short_pairs': [[f"{r['ref']}.{r['pad_a']}",
                                     f"{r['ref']}.{r['pad_b']}"]
                                    for r in own_shorts],
            'own_pad_gating_pairs': [[f"{r['ref']}.{r['pad_a']}",
                                      f"{r['ref']}.{r['pad_b']}"]
                                     for r in own_gating],
            'own_pad_contacts': own_contacts,
            'own_pad_basis': leg.get('own_pad_basis'),
            # #1031: pads in a board rule-area keep-out band -- the same
            # non-gating contract as the two off-outline channels.
            'oob_keepout_copper_count': leg.get('oob_keepout_copper_count', 0),
            'oob_keepout_copper_amount': leg.get('oob_keepout_copper_amount', 0.0),
            'oob_keepout_copper_refs': leg.get('oob_keepout_copper_refs') or [],
            'keepout_copper_pads': leg.get('keepout_copper_pads') or [],
            'keepout_copper_tht_refs': leg.get('keepout_copper_tht_refs') or [],
            'keepout_copper_exempt': leg.get('keepout_copper_exempt') or [],
            'keepout_copper_unmeasured': leg.get('keepout_copper_unmeasured') or [],
            'keepout_copper_basis': leg.get('keepout_copper_basis'),
            'locked_contact_pairs': [q._asdict() for q in locked_contact],
            # run-19: parts stacked at one origin, marker classes exonerated.
            # Groups, not fake N*(N-1)/2 pair entries -- a stack is one
            # finding about one point, and the fix is one re-seat per part.
            'coincident_origin_groups': stack_groups,
            'coincident_origins': len(stack_groups),
            # #1204: origins shared WITHOUT a stack, each with its `why`
            # ('apart' or 'waived' + `waived_pairs`). Not a conjunct.
            'coincident_origins_exempt': stack_exempt,
            'coincident_origins_basis': (
                'every footprint BLOCK (#726): blocks sharing one reference '
                'are keyed with an ordinal suffix, so they form a pair here '
                'like any other two parts. See duplicate_references for the '
                'names the board itself uses. A pair at one origin stacks '
                'only when it shares something physical on a face both '
                'occupy -- copper pad boxes (a plated barrel on both faces), '
                'an NPTH hole on copper, or pad copper / a drawn body the '
                'body channel found -- and is not in the intent\'s '
                'overlap_waivers (#1204); coincident_origins_exempt names '
                'the rest.'),
            'duplicate_references': dup_refs,
            # #837, report-only: additive keys, no exit code and no conjunct
            # moves. `parts_by_side` is the pad-bearing census -- the one the
            # `assembly_sides` verdict is taken from -- and
            # `parts_by_side_blocks` is every block, so a reader can see which
            # rule produced which number instead of guessing.
            'parts_by_side': _cen['pad_bearing'],
            'parts_by_side_blocks': _cen['blocks'],
            'parts_by_side_basis': _cen['basis'],
            'back_side_zero_pad_blocks': _cen['zero_pad_back'],
            'through_hole_parts': _cen['through_hole'],
            'through_hole_by_side': _cen['through_hole_by_side'],
            'smd_parts_by_side': _cen['smd'],
            'unsoldered_parts_by_side': _cen['unsoldered'],
            'zero_pad_blocks_by_side': _cen['zero_pad'],
            'assembly_sides': _cen['sides'],
            # Counted from the SMD population, not from the populated faces:
            # flat_hierarchy is 64 parts, every one through-hole, and takes
            # ZERO reflow passes on a face it is certainly populated on.
            'reflow_passes': _cen['reflow_passes'],
            # Just the entry count now: every block is one. The old formula
            # (len + sum(values) - len(values)) reconstructed the block total
            # from the dict of survivors, and after #726 it OVERCOUNTS -- on
            # watchy it would report 86 + 4 - 2 = 88 blocks for a board with 86.
            'footprint_blocks': len(pcb.footprints),
        }
        if new_advisory is not None:
            doc['baseline'] = args.baseline
            doc['baseline_pile'] = base_pile
            doc['new_advisory_pairs'] = [q._asdict() for q in new_advisory]
        with open(args.json, 'w', encoding='utf-8') as f:
            json.dump(doc, f, indent=1, sort_keys=True)
        print(f"  JSON -> {args.json}")

    return 4 if not_buildable else 0


if __name__ == "__main__":
    import cli_banner
    cli_banner.install()   # CMD/EXIT self-echo (run-3 B1)
    sys.exit(main())

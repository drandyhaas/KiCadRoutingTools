#!/usr/bin/env python3
"""#897: an AUTHORED overlap waiver outranks every part-class label.

`legality._waiver_for` tested `waiver_sets` -- the intent's `overlap_waivers` --
LAST, after the container / marker / edge class labels. So a pair the intent
explicitly waives never read `intent_declared` whenever either part was a
marker, an edge part or a container, which is exactly the kind of pair anyone
waives.

The label is not cosmetic. `_blocking_waived` returns True for
`intent_declared` BEFORE it tests locked-ness, and `_GATE_EXEMPT` lists it, so
an authored waiver is *designed* to be the strongest label -- while
`marker_class` is honoured only for a fiducial or a testpoint, `edge_class`
only when the overlap actually reaches the outline, and NO class label survives
a KiCad-locked part. Reading the classes first therefore voided real waivers.

Run 25: a fiducial inside USB1's pad box, both poses mechanical, both locked,
waived in intent.json. Every review sheet of the run carried
`BLOCKING, past the floors (1): Ref*~2<->USB1`, `render_placement --json-out`
listed it under `b_courtyard_blocking_pairs` on every lap, and check_assembly
--baseline called the same pair baseline's own. Two instruments, two answers,
one waived pair.

WHY NO EXISTING TEST SAW IT. `tests/test_run6_body_overlap.py:171` waives two
generic parts -- the fall-through branch, where the reorder changes nothing --
and every arm in `tests/test_run23_courtyard_channel.py:122-163` runs
check_assembly with NO --intent, so `waiver_sets` is empty in all of them. No
test anywhere built a pair that is both class-labelled and authored-waived.
"""
import os
import sys
import tempfile

RUN_ALL_FAST_OK = True

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement.legality import grade_body_overlap             # noqa: E402
from placement.part_class import classify_part                # noqa: E402

# H1 is NPTH-only -> part_class 'mount_hole', which is in _MARKER but NOT in
# _MARKER_NONPHYSICAL, so `marker_class` does NOT waive its blocking -- that is
# what makes this fixture discriminate. A testpoint would be waived either way.
BOARD = (
    '(kicad_pcb (version 20221018) (generator pcbnew)\n'
    '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
    '  (net 0 "") (net 1 "VCC")\n'
    '  (gr_rect (start 0 0) (end 30 30) (stroke (width 0.1) (type default))'
    ' (layer "Edge.Cuts"))\n'
    '  (footprint "MountingHole:MountingHole_2.2mm" (layer "F.Cu") (at 10 10)'
    '{lockA}\n'
    '    (property "Reference" "H1" (at 0 0) (layer "F.SilkS"))\n'
    '    (fp_rect (start -1.6 -1.6) (end 1.6 1.6) (stroke (width 0.05)'
    ' (type default)) (layer "F.CrtYd"))\n'
    '    (pad "" np_thru_hole circle (at 0 0) (size 2.2 2.2) (drill 2.2)'
    ' (layers "F&B.Cu" "*.Mask")))\n'
    '  (footprint "t:B" (layer "F.Cu") (at 11.2 10){lockB}\n'
    '    (property "Reference" "CB" (at 0 0) (layer "F.SilkS"))\n'
    '    (fp_rect (start -1.6 -1.6) (end 1.6 1.6) (stroke (width 0.05)'
    ' (type default)) (layer "F.CrtYd"))\n'
    '    (pad "1" smd rect (at 1.0 0) (size 0.6 0.6) (layers "F.Cu")'
    ' (net 1 "VCC")))\n'
    # R9 sits far away and touches nothing. It exists so a declared waiver can
    # be UNUSED without being unresolved -- the two populations are only
    # distinguishable when a real, non-overlapping pair is available.
    '  (footprint "t:R" (layer "F.Cu") (at 24 24)\n'
    '    (property "Reference" "R9" (at 0 0) (layer "F.SilkS"))\n'
    '    (fp_rect (start -1 -0.6) (end 1 0.6) (stroke (width 0.05)'
    ' (type default)) (layer "F.CrtYd"))\n'
    '    (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu")'
    ' (net 1 "VCC")))\n'
    ')\n')

fails = []


def check(label, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}  {label}"
          + (f"   [{detail}]" if not ok and detail != '' else ''))
    if not ok:
        fails.append(label)


def _board(locked=False):
    lock = ' (locked yes)' if locked else ''
    text = BOARD.format(lockA=lock, lockB=lock)
    path = os.path.join(tempfile.mkdtemp(), 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    return path


def _grade(path, **kw):
    return grade_body_overlap(parse_kicad_pcb(path), 0.09, pcb_file=path, **kw)


def _pair_of(g, kind='courtyard'):
    return next((p for p in g['pairs'] if p.kind == kind), None)


def test_the_fixture_is_on_the_branch():
    """Without this, every assertion below could pass on a board that simply
    has no overlapping pair, or whose H1 is not a marker after all."""
    print('\n-- 0. the fixture really builds a class-labelled overlap --')
    path = _board()
    pcb = parse_kicad_pcb(path)
    cls = classify_part(pcb.footprints['H1'], 'H1').name
    check("H1 classifies as 'mount_hole'", cls == 'mount_hole', cls)
    g = _grade(path)
    p = _pair_of(g)
    check('the two courtyards overlap', p is not None)
    check('and with no waiver the pair reads a CLASS label',
          p is not None and p.waiver == 'marker_class',
          p.waiver if p else None)
    check('and it IS courtyard-blocking without a waiver',
          g['courtyard_blocking'] == 1, g['courtyard_blocking'])


def test_an_authored_waiver_wins_over_the_class_label():
    print('\n-- 1. the same pair, named in overlap_waivers --')
    path = _board()
    g = _grade(path, intent_waivers=[('H1', 'CB')])
    p = _pair_of(g)
    check("reads 'intent_declared', not 'marker_class'",
          p is not None and p.waiver == 'intent_declared',
          p.waiver if p else None)
    # The CONSEQUENCE. `marker_class` is honoured only for a fiducial or a
    # testpoint, so a mount hole's pair stayed blocking despite the waiver.
    check('and it is no longer courtyard-blocking',
          g['courtyard_blocking'] == 0, g['courtyard_blocking'])


def test_an_authored_waiver_survives_a_locked_part():
    """No CLASS label blesses contact with a KiCad-locked part, deliberately.
    An authored one does -- `_blocking_waived` returns True for
    'intent_declared' before it looks at locked_refs. This is the run-25 shape:
    both poses mechanical, both locked, waived, and blocking on every lap."""
    print('\n-- 2. both parts KiCad-locked --')
    path = _board(locked=True)
    g0 = _grade(path)
    check('locked + class label alone is still blocking (unchanged)',
          g0['courtyard_blocking'] == 1, g0['courtyard_blocking'])
    g1 = _grade(path, intent_waivers=[('H1', 'CB')])
    check('the authored waiver is honoured on a locked pair',
          g1['courtyard_blocking'] == 0, g1['courtyard_blocking'])


def test_a_waiver_naming_a_missing_ref_is_reported():
    print('\n-- 3. a waiver that resolves to nothing --')
    path = _board()
    g = _grade(path, intent_waivers=[('H1', 'CB'), ('H1', 'GONE9')])
    check('the unresolvable pair is named',
          [['GONE9', 'H1']] == g.get('waivers_unresolved'),
          g.get('waivers_unresolved'))
    check('the pair that DID waive something is in neither population',
          g.get('waivers_unused') == []
          and not any('CB' in p for p in (g.get('waivers_unresolved') or [])),
          (g.get('waivers_unused'), g.get('waivers_unresolved')))


def test_an_unused_waiver_is_kept_apart_from_a_stale_one():
    """Two populations, two keys: a ref the board does not have is a rename or
    a deletion; a pair that exists and matched no waivable overlap is harmless.

    THE POSITIVE CASE IS THE POINT. The first cut of this arm waived H1/CB --
    which DOES overlap -- twice under two spellings, so `waivers_unused` was
    asserted `[]` and nothing ever put anything in it: replacing the whole
    expression with a constant `[]` passed all 13 checks.
    """
    print('\n-- 4. a real pair with no waivable overlap --')
    path = _board()
    g = _grade(path, intent_waivers=[('H1', 'CB'), ('CB', 'R9')])
    check('the non-overlapping declared pair is named unused',
          g.get('waivers_unused') == [['CB', 'R9']], g.get('waivers_unused'))
    check('...and R9 is really ON the board, so this is not an unresolved pair '
          'wearing the wrong label',
          g.get('waivers_unresolved') == [], g.get('waivers_unresolved'))
    # ('CB','H1') is the same frozenset as ('H1','CB'): one entry, and used.
    g2 = _grade(path, intent_waivers=[('H1', 'CB'), ('CB', 'H1')])
    check('a pair declared twice under two spellings counts once, and is used',
          g2.get('waivers_unused') == [], g2.get('waivers_unused'))


def test_a_degenerate_pair_does_not_break_the_instrument():
    """`load_intent` checks the pair's LENGTH, not its distinctness, so
    `["U1","U1"]` -- a rename typo -- reaches the grader. It collapses to a
    one-element frozenset, and a consumer formatting `a <-> b` from it died
    with an IndexError: a mistyped intent turned check_assembly into a broken
    tool, exit 1, which a caller reads as "the tool is broken" and not "your
    intent is wrong"."""
    print('\n-- 5. a waiver naming the same ref twice --')
    from placement.legality import format_waiver_warnings
    path = _board()
    g = _grade(path, intent_waivers=[('CB', 'CB'), ('ZZ', 'ZZ')])
    rows = (g.get('waivers_unresolved') or []) + (g.get('waivers_unused') or [])
    check('every reported row has exactly two refs',
          rows and all(len(r) == 2 for r in rows), rows)
    check('the off-board one is reported unresolved',
          ['ZZ', 'ZZ'] in (g.get('waivers_unresolved') or []),
          g.get('waivers_unresolved'))
    try:
        lines = format_waiver_warnings(g)
        ok = any('ZZ' in ln for ln in lines)
    except Exception as exc:                                   # noqa: BLE001
        ok, lines = False, repr(exc)
    check('and the shared formatter renders it instead of raising', ok, lines)


def test_the_render_can_see_a_waiver_at_all():
    """THE SYMPTOM THE ISSUE DESCRIBES, which fixing `_waiver_for` alone does
    not cure: run 25's banner was on the REVIEW SHEET, and
    `render_placement.legality_findings` graded waiver-blind -- it passed no
    `intent_waivers` and the tool had no --intent flag. So the sheet and --gate
    went on calling a waived pair blocking however the precedence was ordered.

    In-process on purpose: the mechanism is `model.intent_waivers`, and driving
    it here keeps this file fast. The flag that sets it is checked against the
    real parser rather than assumed.
    """
    print('\n-- 6. the renderer honours an intent --')
    sys.path.insert(0, os.path.join(ROOT, 'py_tools'))
    import render_placement as RP
    from kicad_parser import parse_kicad_pcb as _parse
    path = _board()

    def _blocking(waivers):
        m = RP.PlacementModel(_parse(path), path, exact=True,
                              quench_kwargs={'clearance': 0.09,
                                             'ignore_net_ids': set()})
        m.intent_waivers = waivers
        return RP.legality_findings(m).get('courtyard_blocking_pairs_refs') or []

    blind = _blocking(())
    check('waiver-blind, the render calls the pair blocking (the run-25 banner)',
          any({'H1', 'CB'} <= set(r[:2]) for r in blind), blind)
    seeing = _blocking((('H1', 'CB'),))
    check('given the intent, it does not', seeing == [], seeing)
    check('--intent is a real flag on the real parser',
          any('--intent' in (a.option_strings or [])
              for a in RP.build_parser()._actions))


def test_the_issues_own_acceptance_criterion():
    """#897's Fix section states the acceptance test literally: "an intent that
    waives a marker<->edge pair reads `intent_declared` in the courtyard census
    and does not appear in `b_courtyard_blocking_pairs`."

    Both halves matter and neither was covered by the arms above. The pair here
    is marker<->EDGE, not marker<->ordinary -- `edge_class` is the label that is
    NOT gate-exempt for containment, so it is the harder of the two. And
    `b_courtyard_blocking_pairs` is the published CHECKLIST key on
    render_placement's document, not the engine's list: asserting the engine's
    and calling the criterion met would be checking a different thing with a
    similar name.
    """
    print('\n-- 7. the issue\'s own acceptance criterion, literally --')
    import json
    sys.path.insert(0, os.path.join(ROOT, 'py_tools'))   # not via arm 6's insert
    import render_placement as RP
    from kicad_parser import parse_kicad_pcb as _parse
    text = BOARD.format(lockA='', lockB='').replace(
        '"t:B" (layer "F.Cu")', '"Connector_USB:USB_C_Receptacle" (layer "F.Cu")')
    path = os.path.join(tempfile.mkdtemp(), 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    cls = classify_part(_parse(path).footprints['CB'], 'CB').name
    check('CB now classifies edge_receptacle (the pair is marker<->edge)',
          cls == 'edge_receptacle', cls)

    g = _grade(path, intent_waivers=[('H1', 'CB')])
    p = _pair_of(g)
    check("it reads 'intent_declared' in the courtyard census",
          p is not None and p.waiver == 'intent_declared', p.waiver if p else None)

    def _checklist(extra):
        out = os.path.join(os.path.dirname(path), 'r.json')
        rc = RP.main([path, '--clearance', '0.09', '--quiet',
                      '--json-out', out, '-o',
                      os.path.join(os.path.dirname(path), 'r.png')] + extra)
        doc = json.load(open(out, encoding='utf-8'))
        return rc, doc['checklist']['b_courtyard_blocking_pairs']

    intent = os.path.join(os.path.dirname(path), 'i.json')
    json.dump({'schema': 1, 'kind': 'floorplan-intent',
               'overlap_waivers': [{'pair': ['H1', 'CB'], 'reason': 'mechanical'}]},
              open(intent, 'w', encoding='utf-8'))
    _rc0, blind = _checklist([])
    check('waiver-blind, it IS in b_courtyard_blocking_pairs (control)',
          any({'H1', 'CB'} <= set(r[:2]) for r in blind), blind)
    _rc1, seeing = _checklist(['--intent', intent])
    check('and given the intent it does NOT appear there',
          seeing == [], seeing)


def main():
    test_the_fixture_is_on_the_branch()
    test_an_authored_waiver_wins_over_the_class_label()
    test_an_authored_waiver_survives_a_locked_part()
    test_a_waiver_naming_a_missing_ref_is_reported()
    test_an_unused_waiver_is_kept_apart_from_a_stale_one()
    test_a_degenerate_pair_does_not_break_the_instrument()
    test_the_render_can_see_a_waiver_at_all()
    test_the_issues_own_acceptance_criterion()
    print()
    if fails:
        print(f"FAIL: {len(fails)} check(s) failed: {fails}")
        return 1
    print('PASS: an authored overlap waiver outranks the part-class labels, '
          'and a waiver that resolves to nothing is named (#897)')
    return 0


if __name__ == '__main__':
    sys.exit(main())

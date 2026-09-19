#!/usr/bin/env python3
"""#959 / #1002: the decap limit as a tri-state, labelled, and superseded by
declared relations.

Run 29 passed `--declare-decaps` zero times, so the decap rules never ran;
and passing it on run 29's pile would have written a limit of 0.0, because
the census there was measured on a pile. So:

  * `derive_decaps` is 'off' | 'strict' | 'auto', compared by EQUALITY --
    'off' is a truthy string, and a truthiness test would derive under it;
  * 'auto' derives only off a PLACED board. On a pile it records why in
    `context.decap_census.auto_withheld`, never in `budget_withheld` (that
    would add an exit-4 abstention to every default emit);
  * every number the emitter chose is labelled `observed_baseline` in
    `context.basis`, and the decap message says so;
  * a proximity relation the BRIEF declares, naming the cap's pads on its
    rail, supersedes that cap's inferred tether at grade time -- without
    writing `decaps.exempt`, which would weaken a declared pin claim.
"""
import json
import os
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _sub in ('', 'py_router', 'py_tools', 'py_placer'):
    _p = os.path.join(REPO, _sub) if _sub else REPO
    if _p not in sys.path:
        sys.path.insert(0, _p)

import run_utils                                            # noqa: E402
from kicad_parser import parse_kicad_pcb                    # noqa: E402
from placement import design_brief as db                    # noqa: E402
from placement import floorplan as fp                       # noqa: E402

RUN_ALL_TIMEOUT = 1800

ESP = os.path.join(REPO, 'kicad_files', 'esp_prog.kicad_pcb')
SPLIT = os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb')
PILE = os.path.join(REPO, 'tests', 'fixtures', '959', 'run29_pile.kicad_pcb')
BRIEF_902 = os.path.join(REPO, 'tests', 'fixtures', '902',
                         'esp_prog_proximity.design-brief.json')


def _emit(board, mode, **kw):
    return fp.emit_intent(parse_kicad_pcb(board), board, derive_decaps=mode,
                          **kw)


def test_the_three_states_and_no_truthy_leak():
    assert fp._decap_mode(True) == 'strict'
    assert fp._decap_mode(False) == 'off' and fp._decap_mode(None) == 'off'
    for bad in ('on', 1, 'yes'):
        try:
            fp._decap_mode(bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f'accepted {bad!r}')
    off = _emit(SPLIT, 'off')
    assert 'max_distance_mm' not in off['decaps'], off['decaps']
    strict = _emit(SPLIT, 'strict')
    auto = _emit(SPLIT, 'auto')
    lim = strict['decaps'].get('max_distance_mm')
    assert lim is not None and auto['decaps'].get('max_distance_mm') == lim
    print(f"  PASS: 'off' never derives; strict and auto agree on a placed "
          f"board (splitflap {lim} mm)")


def test_auto_withholds_on_a_pile_without_an_exit_change():
    """Run 29's pile reads `partially_unplaced` (duplicate fraction 0.833);
    strict wrote 0.0 there. Auto withholds, and records it where the roster
    reads it -- NOT in budget_withheld, so `budget_abstained` and the exit
    code do not move."""
    doc = _emit(PILE, 'auto')
    cen = doc['context']['decap_census']
    assert 'max_distance_mm' not in doc['decaps'], doc['decaps']
    assert 'not placed' in cen.get('auto_withheld', ''), cen
    assert 'decaps.max_distance_mm' not in doc['context']['budget_withheld']
    # PLACED boards whose census the derivation itself refuses (sonde_u: no
    # cap inside the radius; interf_u_unrouted: 1 tether). Strict records
    # that in budget_withheld -- an exit-4 abstention; auto must not.
    for board in ('sonde_u.kicad_pcb', 'interf_u_unrouted.kicad_pcb'):
        path = os.path.join(REPO, 'kicad_files', board)
        strict = _emit(path, 'strict')
        assert 'decaps.max_distance_mm' in strict['context'][
            'budget_withheld'], board
        a = _emit(path, 'auto')
        assert 'max_distance_mm' not in a['decaps'], (board, a['decaps'])
        assert a['context']['decap_census'].get('auto_withheld'), board
        assert 'decaps.max_distance_mm' not in a['context'][
            'budget_withheld'], board
    # A FULLY unplaced board (`unplaced`, not `partially_unplaced`): every
    # part at one coordinate, where strict would read a limit off a heap.
    with tempfile.TemporaryDirectory() as tmp:
        from kicad_parser import iter_footprint_blocks
        text = open(ESP, encoding='utf-8').read()
        out, last = [], 0
        for start, end, _t, _r, _key in iter_footprint_blocks(text):
            block = text[start:end]
            i = block.index('(at ')
            j = block.index(')', i)
            out.append(text[last:start] + block[:i] + '(at 120 95'
                       + block[j:])
            last = end
        heap = os.path.join(tmp, 'heap.kicad_pcb')
        with open(heap, 'w', encoding='utf-8') as fh:
            fh.write(''.join(out) + text[last:])
        from placement.placement_state import assess_placement
        st = assess_placement(parse_kicad_pcb(heap), heap)
        assert st.unplaced and not st.partially_unplaced, st.signals
        h = _emit(heap, 'auto')
        assert 'max_distance_mm' not in h['decaps'], h['decaps']
        assert 'not placed' in h['context']['decap_census']['auto_withheld']
    print("  PASS: auto withholds on run 29's pile, on a fully unplaced heap "
          "and on the two boards the derivation refuses, into the census "
          "rather than budget_withheld")


def _labelled(doc):
    basis = doc['context']['basis']
    for k in doc.get('legality_budget') or {}:
        assert basis.get(f'legality_budget.{k}') == 'observed_baseline', k
    for c in doc['edge_connectors']:
        for k in ('edge', 'overhang_mm'):
            if k in c:
                assert basis[f"edge_connectors[{c['ref']}].{k}"] == \
                    'observed_baseline', (k, c)
    for b in doc['blocks']:
        for k in ('side', 'zone'):
            if k in b:
                assert basis[f"blocks[{b['name']}].{k}"] == \
                    'observed_baseline', (k, b['name'])
    assert basis['envelope.tolerance_mm'] == 'derived_default', basis
    assert basis['defaults.zone_tolerance_mm'] == 'derived_default', basis
    if (doc.get('assembly') or {}).get('sides'):
        assert basis['assembly.sides'] == 'observed_baseline', basis
    return basis


def test_every_emitted_number_is_labelled():
    doc = _emit(SPLIT, 'auto')
    basis = _labelled(doc)
    assert basis.get('decaps.max_distance_mm') == 'observed_baseline', basis
    assert doc['context']['decap_census']['decaps_basis'] == \
        'observed_baseline'
    # A board with zoned, sided blocks, so the block labels are reached.
    ulx = os.path.join(REPO, 'kicad_files', 'ulx3s.kicad_pcb')
    ub = _labelled(_emit(ulx, 'off'))
    assert any(k.endswith('.zone') for k in ub) and any(
        k.endswith('.side') for k in ub), sorted(ub)
    # A brief merged over it re-labels what it declares.
    pcb = parse_kicad_pcb(ESP)
    frag, rep = db.compile_with_consequences(
        db.load_brief(os.path.join(REPO, 'tests', 'fixtures', '711',
                                   'esp_prog.design-brief.json')), pcb, ESP)
    merged = db.merge_into_intent(fp.emit_intent(pcb, ESP), frag, rep)
    mb = merged['context']['basis']
    assert mb['edge_connectors[USB1].edge'] == 'declared', mb
    assert mb['edge_connectors[USB1].max_setback_mm'] == 'derived_default'
    # A brief that declares an edge "unknown" drops the observed edge, and
    # its label goes with it.
    with open(os.path.join(REPO, 'tests', 'fixtures', '711',
                           'esp_prog.design-brief.json'),
              encoding='utf-8') as fh:
        rb = json.load(fh)
    rb['interfaces'][0]['edge'] = 'unknown'
    for k in ('along_edge', 'along_edge_tolerance_mm'):
        rb['interfaces'][0].pop(k, None)
    f2, r2 = db.compile_with_consequences(db.brief_from_dict(rb, ''), pcb,
                                          ESP)
    m2 = db.merge_into_intent(fp.emit_intent(pcb, ESP), f2, r2)
    usb = [c for c in m2['edge_connectors'] if c['ref'] == 'USB1'][0]
    assert 'edge' not in usb, usb
    assert 'edge_connectors[USB1].edge' not in m2['context']['basis']
    # The finding says what the number is -- while it still IS the
    # observation. The census repeats the emitted limit so a hand edit is
    # detectable: an edited limit is somebody's choice, and is not called an
    # observed baseline.
    splitflap = parse_kicad_pcb(SPLIT)
    for census_limit, observed in ((0.01, True), (None, False)):
        raw = json.loads(json.dumps(doc))
        raw['decaps']['max_distance_mm'] = 0.01
        if census_limit is not None:
            raw['context']['decap_census']['emitted_max_distance_mm'] = \
                census_limit
        res = fp.grade(fp.intent_from_dict(raw, ''), splitflap, SPLIT)
        d = [v for v in res.violations if v.rule == 'decap_distance']
        assert d and (('observed regression baseline' in d[0].message)
                      == observed), (census_limit, d[:1])
    print("  PASS: the decap limit, the legality budget, the observed edges, "
          "overhangs, sides and zones carry observed_baseline and the module "
          "tolerances derived_default; a brief re-labels what it states and "
          "drops what it unknows; the finding calls the limit observed only "
          "while it is the emitted one")


def _intent_with(pcb, board, frag, **extra):
    doc = db.merge_into_intent(fp.emit_intent(pcb, board), frag,
                               {'path': '', 'declared': [], 'unknown': [],
                                'absent': [], 'not_graded': [],
                                'unmatched': [], 'unmatched_checked': False,
                                'contradictions': [], 'counts': {},
                                'fixed': [], 'product': {}})
    doc.update(extra)
    return fp.intent_from_dict(doc, '')


def test_a_declared_relation_supersedes_the_inferred_tether():
    """Fixture 902 declares `C1` and `C3` near `U2` with their pad 1 -- on
    the rail U2 carries -- so those two caps are graded by the relation and
    not by the decap rules. `C2` and `C4`, tethered to Y1 by inference, are
    still graded. Nothing is written into `decaps.exempt`."""
    pcb = parse_kicad_pcb(ESP)
    frag, _rep = db.compile_brief(db.load_brief(BRIEF_902),
                                  board_refs=sorted(pcb.footprints))
    sup = fp.superseded_caps(pcb, frag['proximity'], frag)
    assert set(sup) == {'C1', 'C3'}, sup
    it = _intent_with(pcb, ESP, frag, decaps={'max_distance_mm': 0.5})
    assert not (it.decaps or {}).get('exempt'), it.decaps
    res = fp.grade(it, pcb, ESP, brief_fragment=frag)
    hit = {v.ref for v in res.violations if v.rule == 'decap_distance'}
    assert 'C3' not in hit and {'C2', 'C4'} <= hit, hit
    ung = {v.ref for v in res.violations if v.rule == 'decap_ungraded'}
    assert 'C1' not in ung, ung
    # Without the brief nothing is superseded: the same intent grades C3.
    res = fp.grade(it, pcb, ESP)
    assert 'C3' in {v.ref for v in res.violations
                    if v.rule == 'decap_distance'}
    # A row only the PLAN carries (a hypothesis) supersedes nothing, and
    # neither does a brief row naming no pads on the cap's rail.
    plan_only = fp.superseded_caps(pcb, frag['proximity'], {'proximity': []})
    assert plan_only == {}, plan_only
    nopads = json.loads(json.dumps(frag))
    for p in nopads['proximity']:
        p.pop('pads', None)
    assert fp.superseded_caps(pcb, nopads['proximity'], nopads) == {}
    print("  PASS: C1 and C3 are graded by their declared relation; C2 and "
          "C4 by the decap rule; a plan-only row or a pad-less row "
          "supersedes nothing")


def test_supersession_cannot_be_laundered():
    """The guards the commit claims, each on its own row (the Phase-6
    verifier found all of them untested). esp_prog: C3's pad 1 is /+3.3V,
    pad 2 GND; U2 carries both, Y1 carries neither /+3.3V pin."""
    pcb = parse_kicad_pcb(ESP)

    def sup(brief_row, plan_row=None):
        return set(fp.superseded_caps(pcb, [plan_row or brief_row],
                                      {'proximity': [brief_row]}))

    base = {'ref': 'C3', 'near': 'U2', 'max_mm': 2.0, 'basis': 'pad_edge',
            'pads': {'C3': ['1'], 'U2': ['3']}}
    assert sup(base) == {'C3'}
    # Named pad on GROUND, which the partner also carries: not a rail.
    assert sup(dict(base, pads={'C3': ['2'], 'U2': ['1']})) == set()
    # Named pad on the rail, but the PARTNER does not carry it.
    assert sup(dict(base, near='Y1', pads={'C3': ['1']})) == set()
    # The plan's row drifted from the brief's: a hypothesis, not a claim.
    assert sup(base, dict(base, max_mm=5.0)) == set()
    assert sup(base, dict(base, pads={'C3': ['1']})) == set()
    # Swapped rows name the same relation.
    swapped = {'ref': 'U2', 'near': 'C3', 'max_mm': 2.0,
               'pads': {'C3': ['1'], 'U2': ['3']}}
    assert sup(swapped) == {'C3'}
    # The default basis spelled out is the same claim as it left implicit.
    implicit = {k: v for k, v in base.items() if k != 'basis'}
    assert sup(implicit, base) == {'C3'}
    # The emitted limit is NOT tightened by leaving superseded caps out: the
    # placement engines grade without the brief, and must pass the board it
    # was emitted from. The census discloses the supersession instead.
    frag, _rep = db.compile_brief(db.load_brief(BRIEF_902),
                                  board_refs=sorted(pcb.footprints))
    plain = fp.emit_intent(pcb, ESP, derive_decaps='strict')
    with_b = fp.emit_intent(pcb, ESP, derive_decaps='strict',
                            brief_fragment=frag)
    assert plain['decaps'] == with_b['decaps'], (plain['decaps'],
                                                 with_b['decaps'])
    assert with_b['context']['decap_census']['superseded'] == {
        'C1': 'proximity C1 near U2', 'C3': 'proximity C3 near U2'}, \
        with_b['context']['decap_census']
    assert 'superseded' not in plain['context']['decap_census']
    print("  PASS: a ground pad, a partner off the rail and a drifted plan "
          "row supersede nothing; swapped rows and a spelled-out default "
          "do; the emitted limit ignores supersession and the census "
          "discloses it")


def test_every_cap_superseded_owes_nothing_and_stays_complete():
    """When a declared relation covers every cap with an IC on its rail, the
    decap rules have nothing of their own to grade. Dark, they are not
    applicable (nothing owed). Armed, they RUN and skip every cap -- not an
    abstention, which made the grade incomplete: declaring more turned exit
    0 into exit 4 (Phase-6 verifier)."""
    pcb = parse_kicad_pcb(ESP)
    frag, _rep = db.compile_brief(db.load_brief(BRIEF_902),
                                  board_refs=sorted(pcb.footprints))
    extra = [{'ref': 'C2', 'near': 'Y1', 'max_mm': 3.0, 'basis': 'pad_edge',
              'pads': {'C2': ['1'], 'Y1': ['1']}, 'source': 'brief'},
             {'ref': 'C4', 'near': 'Y1', 'max_mm': 3.0, 'basis': 'pad_edge',
              'pads': {'C4': ['1'], 'Y1': ['2']}, 'source': 'brief'}]
    frag = dict(frag, proximity=list(frag['proximity']) + extra)
    sup = fp.superseded_caps(pcb, frag['proximity'], frag)
    assert {'C1', 'C2', 'C3', 'C4'} <= set(sup), sup
    armed = _intent_with(pcb, ESP, frag, decaps={'max_distance_mm': 0.5})
    rows = {r['rule']: r for r in fp.rule_roster(armed, pcb, ESP,
                                                 brief_fragment=frag)}
    assert rows['decap_distance']['state'] == 'armed', rows['decap_distance']
    res = fp.grade(armed, pcb, ESP, brief_fragment=frag)
    assert not [v for v in res.violations
                if v.rule in ('decap_distance', 'decap_ungraded')], [
        v for v in res.violations if v.rule.startswith('decap')]
    assert 'decap_distance' in res.rules_run, res.rules_skipped
    assert res.complete, (res.budget_abstained, res.rules_skipped)
    dark = _intent_with(pcb, ESP, frag)
    rows = {r['rule']: r for r in fp.rule_roster(dark, pcb, ESP,
                                                 brief_fragment=frag)}
    assert not rows['decap_distance']['needs_disposition'], rows[
        'decap_distance']
    assert 'declared proximity' in rows['decap_distance'][
        'applicability_reason'], rows['decap_distance']
    print("  PASS: all caps superseded -> armed decap rules run, skip every "
          "cap and leave the grade complete; dark ones owe nothing")


def test_the_cli_flags():
    with tempfile.TemporaryDirectory() as tmp:
        out = {}
        for flag in ('--no-declare-decaps', '--declare-decaps',
                     '--auto-declare-decaps', None):
            p = os.path.join(tmp, (flag or 'none').strip('-') + '.json')
            run_utils.check([sys.executable, '-X', 'utf8',
                             run_utils.tool('check_floorplan.py'), SPLIT,
                             '--emit-intent', p] + ([flag] if flag else []),
                            accept=True)
            out[flag] = json.load(open(p, encoding='utf-8'))['decaps']
        assert 'max_distance_mm' not in out['--no-declare-decaps']
        assert out['--declare-decaps'].get('max_distance_mm') == \
            out['--auto-declare-decaps'].get('max_distance_mm') is not None
        # No flag is the DEFAULT, which stays off (the A/B rejected auto).
        assert out[None] == out['--no-declare-decaps'], out
    # ...and the default is the constant, not whatever the first of three
    # shared-dest flags declared: argparse takes a shared dest's default from
    # the FIRST action, so a `default=` on the third was dead (Phase-6
    # verifier -- the constant set to 'auto' still parsed as None).
    sys.path.insert(0, os.path.join(REPO, 'py_tools'))
    import check_floorplan as cf
    saved = cf.DECLARE_DECAPS_DEFAULT
    try:
        for want in ('auto', 'strict', 'off'):
            cf.DECLARE_DECAPS_DEFAULT = want
            got = cf.build_parser().parse_args([SPLIT]).declare_decaps
            assert got == want, (want, got)
    finally:
        cf.DECLARE_DECAPS_DEFAULT = saved
    print("  PASS: --no-declare-decaps / --declare-decaps / "
          "--auto-declare-decaps select the three states; no flag is the "
          "DECLARE_DECAPS_DEFAULT constant, which is off")


TESTS = [
    test_the_three_states_and_no_truthy_leak,
    test_auto_withholds_on_a_pile_without_an_exit_change,
    test_every_emitted_number_is_labelled,
    test_a_declared_relation_supersedes_the_inferred_tether,
    test_supersession_cannot_be_laundered,
    test_every_cap_superseded_owes_nothing_and_stays_complete,
    test_the_cli_flags,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")

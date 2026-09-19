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
    print("  PASS: auto withholds on run 29's pile and on the two boards the "
          "derivation refuses, into the census rather than budget_withheld")


def test_every_emitted_number_is_labelled():
    doc = _emit(SPLIT, 'auto')
    basis = doc['context']['basis']
    assert basis.get('decaps.max_distance_mm') == 'observed_baseline', basis
    assert doc['context']['decap_census']['decaps_basis'] == \
        'observed_baseline'
    for k in doc.get('legality_budget') or {}:
        assert basis.get(f'legality_budget.{k}') == 'observed_baseline', k
    for c in doc['edge_connectors']:
        if 'edge' in c:
            assert basis[f"edge_connectors[{c['ref']}].edge"] == \
                'observed_baseline', c
    # A brief merged over it re-labels what it declares.
    pcb = parse_kicad_pcb(ESP)
    frag, rep = db.compile_with_consequences(
        db.load_brief(os.path.join(REPO, 'tests', 'fixtures', '711',
                                   'esp_prog.design-brief.json')), pcb, ESP)
    merged = db.merge_into_intent(fp.emit_intent(pcb, ESP), frag, rep)
    mb = merged['context']['basis']
    assert mb['edge_connectors[USB1].edge'] == 'declared', mb
    assert mb['edge_connectors[USB1].max_setback_mm'] == 'derived_default'
    # And the finding says what the number is.
    raw = dict(doc)
    raw['decaps'] = dict(doc['decaps'], max_distance_mm=0.01)
    res = fp.grade(fp.intent_from_dict(raw, ''), parse_kicad_pcb(SPLIT),
                   SPLIT)
    d = [v for v in res.violations if v.rule == 'decap_distance']
    assert d and 'observed regression baseline' in d[0].message, d[:1]
    print("  PASS: the decap limit, the legality budget and the observed "
          "edges carry observed_baseline; a brief re-labels what it states; "
          "the finding says so")


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


def test_every_cap_superseded_abstains_the_decap_rules():
    """When a declared relation covers every cap with an IC on its rail, the
    decap rules have nothing of their own to grade: armed, they abstain
    (not a silent pass); dark, they are not applicable (nothing owed)."""
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
    assert rows['decap_distance']['state'] == 'abstained', rows[
        'decap_distance']
    dark = _intent_with(pcb, ESP, frag)
    rows = {r['rule']: r for r in fp.rule_roster(dark, pcb, ESP,
                                                 brief_fragment=frag)}
    assert not rows['decap_distance']['needs_disposition'], rows[
        'decap_distance']
    assert 'declared proximity' in rows['decap_distance'][
        'applicability_reason'], rows['decap_distance']
    print("  PASS: all caps superseded -> armed decap rules abstain, dark "
          "ones owe nothing")


def test_the_cli_flags():
    with tempfile.TemporaryDirectory() as tmp:
        out = {}
        for flag in ('--no-declare-decaps', '--declare-decaps',
                     '--auto-declare-decaps'):
            p = os.path.join(tmp, flag.strip('-') + '.json')
            run_utils.check([sys.executable, '-X', 'utf8',
                             run_utils.tool('check_floorplan.py'), SPLIT,
                             '--emit-intent', p, flag], accept=True)
            out[flag] = json.load(open(p, encoding='utf-8'))['decaps']
        assert 'max_distance_mm' not in out['--no-declare-decaps']
        assert out['--declare-decaps'].get('max_distance_mm') == \
            out['--auto-declare-decaps'].get('max_distance_mm') is not None
    print("  PASS: --no-declare-decaps / --declare-decaps / "
          "--auto-declare-decaps select the three states")


TESTS = [
    test_the_three_states_and_no_truthy_leak,
    test_auto_withholds_on_a_pile_without_an_exit_change,
    test_every_emitted_number_is_labelled,
    test_a_declared_relation_supersedes_the_inferred_tether,
    test_every_cap_superseded_abstains_the_decap_rules,
    test_the_cli_flags,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")

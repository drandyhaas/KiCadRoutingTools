#!/usr/bin/env python3
"""The placement terms measure, and say so when they cannot (#894).

On a copper-free board every lap scores the same `blocking` and the same
`quality`, so the ledger cannot rank two placements. These five terms are what
a lap can be ranked by. This file pins three things about them:

1. **They CALL the graders, they do not mirror them.** `pair_length`'s rows
   are asserted element-for-element against `board_context.pin_order_rows`'
   own output, and `cluster_to_pin`'s declared gaps against
   `floorplan.grade`'s. A re-implementation of a grader in this repo has been
   measured disagreeing with it 83 times, worst 0.234mm.

2. **A term that cannot measure says so and reports `value: None`.** Never 0.
   "No differential pair on this board" and "the pairs are all short" are
   different answers, and a score that cannot tell them apart ranks a board
   nothing examined above one that was examined and found wanting. Asserted as
   a sweep: every term is in exactly one of two shapes, never a third.

3. **Comparison is PARETO and refuses a moved basis.** Two laps that improve
   different terms are `mixed` with both named -- no weight, because #694's
   corridor inversion is what a collapsed verdict hides. And a term whose
   POPULATION changed is not judged at all: measured on the run-25 lineage,
   `plane_cut_proxy`'s blocker set went 3 -> 11 -> 3 as parts were frozen and
   its value went 6.8 -> 101.4 -> 0.0 with it. Most of that is bookkeeping
   about what the operator locked.

Two committed cross-checks, quoted from the tree rather than measured by hand:
the pair span is **8.10 mm** on lap5 (`references/boundary-criteria.md`, pinned
by `test_895_boundary_criteria.py`) and **8.131 mm** on the placed board
(#894's own follow-up comment). Both are right; they are different boards, and
an earlier draft of this work nearly encoded one of them as the other.

Run: python3 -X utf8 tests/test_894_placement_terms.py
"""
import json
import os
import subprocess
import sys
import tempfile

RUN_ALL_TIMEOUT = 900

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, 'tests'), os.path.join(ROOT, 'py_router'),
                os.path.join(ROOT, 'py_placer'), os.path.join(ROOT, 'py_tools')]
os.environ.setdefault('KRT_NO_BANNER', '1')

import run_utils                                              # noqa: E402
import placement_score as ps                                  # noqa: E402

TRACKED = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
PLACED = os.path.join(ROOT, 'tests', 'fixtures', 'run25',
                      'esp_prog_placed.kicad_pcb')
LAP3 = os.path.join(ROOT, 'tests', 'fixtures', 'run25',
                    'esp_prog_lap3.kicad_pcb')
LAP5 = os.path.join(ROOT, 'tests', 'fixtures', 'run25',
                    'esp_prog_lap5.kicad_pcb')
BRIEF = os.path.join(ROOT, 'tests', 'fixtures', '902',
                     'esp_prog_proximity.design-brief.json')
CRITERIA = os.path.join(ROOT, '.claude', 'skills',
                        'plan-pcb-placement-and-routing', 'references',
                        'boundary-criteria.md')

passed = 0
failed = 0
_cache = {}


def check(name, ok, detail=''):
    """`detail` reads as a MEASUREMENT: it prints on OK as well as on FAIL."""
    global passed, failed
    if ok:
        passed += 1
        print(f'  OK   {name}' + (f' -- {detail}' if detail else ''))
    else:
        failed += 1
        print(f'  FAIL {name}' + (f' -- {detail}' if detail else ''))


def terms(board, intent=None):
    key = (board, intent)
    if key not in _cache:
        argv = [sys.executable, '-X', 'utf8',
                os.path.join(ROOT, 'py_placer', 'placement_score.py'), board]
        if intent:
            argv += ['--intent', intent]
        r = subprocess.run(argv, capture_output=True, text=True,
                           encoding='utf-8', errors='replace', timeout=900,
                           cwd=ROOT, env=run_utils.tool_env())
        if r.returncode != 0:
            raise AssertionError(f'placement_score failed on {board}: '
                                 f'{r.stderr[-800:]}')
        _cache[key] = json.loads(r.stdout)
    return _cache[key]


def _intent_from_brief(tmp):
    """The 902 brief, compiled to an intent -- the same path test_902 uses."""
    from kicad_parser import parse_kicad_pcb
    from placement import design_brief as db
    from placement import floorplan as fp
    pcb = parse_kicad_pcb(PLACED)
    brief = db.load_brief(BRIEF)
    frag, _rep = db.compile_brief(brief, board_refs=sorted(pcb.footprints),
                                  refs_known=True)
    p = os.path.join(tmp, 'intent.json')
    with open(p, 'w', encoding='utf-8') as fh:
        json.dump({'schema': 1, 'kind': fp.KIND, 'units': 'mm',
                   'proximity': frag['proximity']}, fh)
    return p


def test_the_terms_call_board_context_rather_than_mirroring_it():
    """`pair_length` must be board_context's own `span_mm`, row for row."""
    from kicad_parser import parse_kicad_pcb
    import board_context
    from list_nets import board_floor_knobs
    pcb = parse_kicad_pcb(PLACED)
    clr = board_floor_knobs(PLACED, clearance=None)[0]
    theirs = board_context.pin_order_rows(pcb, PLACED, clr)
    mine = terms(PLACED)['terms']['pair_length']
    want = sorted((r['a'], r['b'], r['scope'], r['span_mm'])
                  for r in theirs['rows']
                  if str(r.get('scope') or '').startswith('pair ')
                  and isinstance(r.get('span_mm'), (int, float)))
    got = sorted((p['a'], p['b'], p['scope'], p['span_mm'])
                 for p in mine['pairs'])
    check('pair rows are board_context.pin_order_rows\' own, element for '
          'element', got == want, f'{len(got)} row(s)')
    check('...and the value is the worst of them',
          mine['value'] == round(max(s for _a, _b, _s, s in want), 3),
          f"value={mine['value']}")


def test_the_committed_pair_numbers_are_reproduced():
    """Two figures already in the tree, on two different boards."""
    lap5 = terms(LAP5)['terms']['pair_length']['value']
    placed = terms(PLACED)['terms']['pair_length']['value']
    text = open(CRITERIA, encoding='utf-8').read()
    check('lap5 reproduces the span boundary-criteria.md states',
          f'{lap5:.2f} mm' in text, f'{lap5} -> "{lap5:.2f} mm"')
    check('the placed board reproduces the figure #894 recorded (8.131)',
          abs(placed - 8.131) < 0.002, str(placed))
    check('...and the two boards genuinely differ, so neither figure is the '
          'other', abs(placed - lap5) > 0.02, f'placed {placed} vs lap5 {lap5}')


def test_cluster_to_pin_reports_a_clause_that_PASSES():
    """The #902 silence trap: a rule yields violations, so a clause that holds
    publishes nothing. These are the gaps #902's own acceptance states."""
    with tempfile.TemporaryDirectory(prefix='t894_') as tmp:
        doc = terms(PLACED, intent=_intent_from_brief(tmp))
    t = doc['terms']['cluster_to_pin']
    check('the declared channel measured every claim', t['declared'] >= 4,
          f"declared={t['declared']} inferred={t['inferred']}")
    passing = [r for r in t['rows']
               if r['source'] == 'declared' and r['passes']]
    check('...including the ones that PASS', len(passing) >= 3,
          f'{len(passing)} passing clause(s) carry a number')
    # PER SUBJECT PAD, not per pair. #902 reports a declared claim once per
    # declared pad on purpose -- "the crystal is too far" is one fact, but
    # "XI is 3.14mm away AND XO is 1.78mm away" is two, and collapsing them to
    # the worst hides a leg the author still has to move. An earlier version
    # of this check keyed on (ref, near) and silently kept whichever row came
    # last, which is the same collapse one level up.
    gaps = sorted(round(r['gap_mm'], 2) for r in t['rows']
                  if r['source'] == 'declared')
    for want in (3.14, 1.78, 1.12, 0.29):
        check(f"a declared clause reproduces #902's committed {want}mm",
              any(abs(g - want) < 0.01 for g in gaps), f'measured {gaps}')
    y1 = sorted(round(r['gap_mm'], 2) for r in t['rows']
                if (r['ref'], r['near']) == ('Y1', 'U1'))
    check('...and both of the crystal\'s legs are reported, not just the worst',
          y1 == [1.78, 3.14], str(y1))


def test_a_term_that_cannot_measure_says_so_and_never_reports_zero():
    for board in (TRACKED, PLACED, LAP3, LAP5):
        doc = terms(board)
        for name in doc['term_order']:
            t = doc['terms'][name]
            ok = ((t['ran'] is True and t['value'] is not None)
                  or (t['ran'] is False and t['value'] is None and t['reason']))
            check(f'{os.path.basename(board)}/{name} is one of the two legal '
                  f'shapes', ok,
                  f"ran={t['ran']} value={t['value']!r} "
                  f"reason={(t['reason'] or '')[:60]!r}")


def test_the_pillow_trap_is_a_reason_not_a_zero():
    """`board_context.pin_order_rows` swallows a missing Pillow and returns an
    EMPTY row list, so "no crossings" and "nothing measured" look identical."""
    doc = {'error': 'StartupCheckError: Pillow is required', 'rows': []}
    for fn in (ps.pin_order_crossings, ps.pair_length):
        r = fn(doc)
        check(f'{fn.__name__} refuses an unmeasured document',
              r['ran'] is False and r['value'] is None
              and 'Pillow' in (r['reason'] or ''), repr(r['reason']))


def test_undetermined_rows_are_not_counted_as_agreement():
    """A tie on the channel axis means the tie-break invented an order the
    geometry does not have. Counting it 0 would say the pin order is fine."""
    doc = {'error': None, 'rows': [
        {'a': 'A', 'b': 'B', 'scope': 'interface', 'inversions': 0,
         'ties': 2, 'verdict': 'UNDETERMINED (pads tie on the channel axis)',
         'span_mm': 1.0}]}
    r = ps.pin_order_crossings(doc)
    check('an all-undetermined board is skipped, not scored 0',
          r['ran'] is False and r['value'] is None, repr(r['reason']))
    doc['rows'].append({'a': 'C', 'b': 'D', 'scope': 'interface',
                        'inversions': 3, 'ties': 0, 'verdict': 'CROSSED',
                        'span_mm': 2.0})
    r = ps.pin_order_crossings(doc)
    check('with one real row it counts 1 crossing and reports the tie apart',
          r['value'] == 1 and r['undetermined'] == 1,
          f"value={r['value']} undetermined={r['undetermined']}")


def test_compare_is_pareto_and_never_a_scalar():
    def t(v, basis=None):
        return {'ran': True, 'value': v, 'unit': 'mm', 'basis': basis,
                'direction': 'lower-is-better'}
    A = {'pair_length': t(8.0), 'balance': t(0.10)}
    cases = [
        ('all terms improve', {'pair_length': t(7.0), 'balance': t(0.05)},
         'better'),
        ('all terms worsen', {'pair_length': t(9.0), 'balance': t(0.20)},
         'worse'),
        ('nothing moves', {'pair_length': t(8.0), 'balance': t(0.10)}, 'same'),
        ('terms TRADE', {'pair_length': t(7.0), 'balance': t(0.20)}, 'mixed'),
        ('nothing measured on both',
         {'pair_length': {'ran': False, 'value': None, 'basis': None}},
         'no-common-terms'),
    ]
    for name, B, want in cases:
        got, detail = ps.compare_terms(A, B)
        check(f'{name} -> {want}', got == want, got)
    # A term measured on ONE side only is EXCLUDED, never defaulted.
    got, detail = ps.compare_terms(
        {'pair_length': t(8.0)},
        {'pair_length': t(7.0), 'balance': t(0.05)})
    row = [r for r in detail if r['term'] == 'balance'][0]
    check('a term measured on one lap only is not judged',
          got == 'better' and row['judgement'] == 'not-comparable'
          and row.get('why') == 'measured on one lap only', str(row))


def test_a_moved_basis_is_refused_rather_than_judged():
    def t(v, basis):
        return {'ran': True, 'value': v, 'unit': 'mm', 'basis': basis,
                'direction': 'lower-is-better'}
    got, detail = ps.compare_terms(
        {'plane_cut_proxy': t(101.4, ['Q1', 'U1', 'U2'])},
        {'plane_cut_proxy': t(0.0, ['Q1'])})
    row = detail[TERM := [i for i, r in enumerate(detail)
                          if r['term'] == 'plane_cut_proxy'][0]]
    check('a 101.4 -> 0.0 drop over a SMALLER basis is not an improvement',
          got == 'no-common-terms' and row['judgement'] == 'not-comparable'
          and row.get('why') == 'the basis moved', f"{got} / {row}")
    check('...and the basis change is named',
          row.get('basis_removed') == ['U1', 'U2'], str(row.get('basis_removed')))
    # Same basis, real change: judged.
    got, _d = ps.compare_terms({'plane_cut_proxy': t(10.0, ['Q1'])},
                               {'plane_cut_proxy': t(4.0, ['Q1'])})
    check('the same basis with a real change IS judged', got == 'better', got)


def test_the_run25_lineage_no_longer_scores_identically():
    """#894's acceptance (a), restated against the boards that exist: the four
    esp_prog boards in the tree, not the seed1..lap7 lineage the issue names."""
    tuples = {}
    for b in (TRACKED, PLACED, LAP3, LAP5):
        doc = terms(b)
        tuples[os.path.basename(b)] = tuple(
            doc['terms'][k]['value'] for k in doc['term_order'])
    check('the four boards do not all score the same',
          len(set(tuples.values())) >= 3,
          '; '.join(f'{k}={v}' for k, v in tuples.items()))
    moved = [k for k in ps.TERM_ORDER
             if len({t[ps.TERM_ORDER.index(k)] for t in tuples.values()}) >= 2]
    check('at least two terms take more than one value across them',
          len(moved) >= 2, f'terms that move: {moved}')


def main():
    run_utils.evidence(TRACKED, 'the tracked esp_prog board')
    run_utils.evidence(PLACED, 'the run-25 placed fixture')
    for name in sorted(k for k in globals() if k.startswith('test_')):
        print(f'--- {name}')
        globals()[name]()
    print(f'\n{passed} passed, {failed} failed')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())

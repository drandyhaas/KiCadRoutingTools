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
    """The pin-order terms must READ board_context, not hold a copy.

    Asserted by SUBSTITUTION, not by agreement. An earlier version of this
    check compared the term's rows to board_context's on the same board, and
    a hardcoded literal for that board passed it -- a same-board agreement
    check cannot tell a call from a copy. Patching the source and watching the
    answer follow can.
    """
    import board_context
    from kicad_parser import parse_kicad_pcb
    sentinel = {'error': None, 'rows': [
        {'a': 'ZZ1', 'b': 'ZZ2', 'scope': 'pair /FAKE_P//FAKE_N', 'nets': 2,
         'inversions': 0, 'lis': 2, 'ties': 0, 'verdict': 'AGREES',
         'span_mm': 42.5},
        {'a': 'ZZ1', 'b': 'ZZ2', 'scope': 'interface', 'nets': 2,
         'inversions': 7, 'lis': 1, 'ties': 0, 'verdict': 'CROSSED',
         'span_mm': 42.5}]}
    real = board_context.pin_order_rows
    board_context.pin_order_rows = lambda *a, **k: sentinel
    try:
        pcb = parse_kicad_pcb(PLACED)
        doc = ps.placement_terms(pcb, PLACED)
    finally:
        board_context.pin_order_rows = real
    check('pair_length follows board_context, not a copy',
          doc['terms']['pair_length']['value'] == 42.5,
          str(doc['terms']['pair_length']['value']))
    check('pin_order_crossings follows it too',
          doc['terms']['pin_order_crossings']['value'] == 1,
          str(doc['terms']['pin_order_crossings']['value']))
    # ...and, separately, that on a REAL board the rows it publishes are the
    # grader's own rather than a re-derivation.
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
    check('the published rows are the grader\'s own, element for element',
          got == want, f'{len(got)} row(s)')


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


def test_plane_cut_credits_the_crossing_and_grows_with_the_blocker():
    """The three defects an adversarial review measured in the first version.

    It credited the WHOLE chord rather than the part inside the body, so an
    80mm net grazing 0.01mm of a part scored 5.3x worse than a 15mm net
    straight through it; it skipped any net with an endpoint INSIDE the body,
    which made a bigger blocker obstruct LESS (growing one part 1mm a side
    took the total DOWN 13% and then saturated); and it counted GND, which on
    two layers IS the reference copper the term claims to be about.
    """
    from kicad_parser import parse_kicad_pcb
    from placement import floorplan as fp
    R = (0.0, 0.0, 10.0, 10.0)
    for name, a, b, want in (
            ('a chord straight through', (-5, 5), (15, 5), 10.0),
            ('an endpoint inside the body', (5, 5), (100, 5), 5.0),
            ('both endpoints inside', (2, 2), (8, 8), 8.485),
            ('a long net entirely clear of it', (-50, 50), (50, 50), 0.0)):
        got = ps._clip_len(a, b, R)
        check(f'clip: {name}', abs(got - want) < 0.01, f'{got} vs {want}')
    # A GRAZE must not outscore a CUT, which is what crediting the whole
    # chord did.
    graze = ps._clip_len((-40, 9.999), (40, 9.999), R)
    cut = ps._clip_len((-2.5, 5), (12.5, 5), R)
    check('an 80mm graze scores below a 15mm cut', graze < cut,
          f'graze {graze:.3f} vs cut {cut:.3f}')

    pcb = parse_kicad_pcb(PLACED)
    real = fp.drawn_body_rect
    vals = []
    try:
        for grow in (0.0, 0.5, 1.0, 3.0):
            def patched(geom, fpo, _g=grow, _r=real):
                r, s = _r(geom, fpo)
                return ((None, s) if r is None
                        else ((r[0] - _g, r[1] - _g, r[2] + _g, r[3] + _g), s))
            fp.drawn_body_rect = patched
            vals.append(ps.plane_cut_proxy(pcb, PLACED)['value'])
    finally:
        fp.drawn_body_rect = real
    check('a bigger blocker obstructs MORE, monotonically',
          all(x < y for x, y in zip(vals, vals[1:])), str(vals))
    t = ps.plane_cut_proxy(pcb, PLACED)
    check('the reference nets are excluded and named',
          any('GND' in n for n in (t.get('excluded_nets') or [])),
          str(t.get('excluded_nets')))
    check('...and no reported row is a reference net',
          all('GND' not in (r.get('net') or '') for r in (t.get('rows') or [])),
          str([r.get('net') for r in (t.get('rows') or [])][:6]))


def test_balance_weighs_copper_and_not_mask():
    """NPTH pads carry no copper: their `size` is the mask opening. Weighing
    them biases the result by more than the signal this term must resolve."""
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(PLACED)
    t = ps.pad_area_balance(pcb)
    npth = [p for f in pcb.footprints.values() for p in (f.pads or ())
            if getattr(p, 'pad_type', '') == 'np_thru_hole']
    check('the board has NPTH pads to exclude, so this is not vacuous',
          len(npth) > 0, f'{len(npth)} NPTH pad(s)')
    check('...and the term excluded exactly them',
          t['npth_pads_excluded'] == len(npth),
          f"excluded={t['npth_pads_excluded']} of {len(npth)}")
    check('the axis is the long one', t['axis'] == 'x'
          and t['span_mm'] >= 20, f"axis={t['axis']} span={t['span_mm']}")
    check('the value is a fraction of span in [0, 0.5]',
          0.0 <= t['value'] <= 0.5, str(t['value']))
    # And the abstention is REACHABLE -- a term whose skip arm no board takes
    # is a claim no test checks.
    class _Sq:
        board_info = type('B', (), {'board_bounds': (0.0, 0.0, 20.0, 20.0),
                                    'copper_layers': ['F.Cu', 'B.Cu']})()
        footprints = {}
    r = ps.pad_area_balance(_Sq())
    check('a square board abstains rather than picking an axis',
          r['ran'] is False and r['value'] is None
          and 'square' in (r['reason'] or ''), repr(r['reason']))


def test_every_terms_skip_path_is_reachable():
    """The sweep over real boards takes the `ran: True` arm every time, so the
    refusal shapes are asserted directly. A skip arm no test reaches is a
    claim about the code that nothing checks."""
    class _NoOutline:
        board_info = type('B', (), {'board_bounds': None,
                                    'copper_layers': ['F.Cu', 'B.Cu']})()
        footprints = {}
        nets = {}
        segments = []
    class _FourLayer(_NoOutline):
        board_info = type('B', (), {
            'board_bounds': (0.0, 0.0, 30.0, 10.0),
            'copper_layers': ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']})()
    for name, fn, arg, want in (
            ('balance without an outline', ps.pad_area_balance, _NoOutline(),
             'outline'),
            ('plane_cut on a 4-layer board', ps.plane_cut_proxy, _FourLayer(),
             'layer'),
            ('cluster_to_pin with nothing declared or elected',
             lambda p: ps.cluster_to_pin(p, None), _NoOutline(), 'proximity')):
        r = fn(arg) if not isinstance(fn, type(lambda: 0)) or True else None
        check(f'{name} refuses with a reason, value None',
              r['ran'] is False and r['value'] is None
              and want in (r['reason'] or ''), repr((r['reason'] or '')[:90]))


def test_an_unresolved_claim_is_not_a_clean_number():
    """`proximity_measured` records nothing for a claim the grader could not
    resolve, so reading only the measured rows turned "your intent names a
    part that is not on the board" into a clean maximum over whatever else
    happened to resolve."""
    from placement import floorplan as fp
    with tempfile.TemporaryDirectory(prefix='t894u_') as tmp:
        p = os.path.join(tmp, 'intent.json')
        with open(p, 'w', encoding='utf-8') as fh:
            json.dump({'schema': 1, 'kind': fp.KIND, 'units': 'mm',
                       'proximity': [
                           {'ref': 'U404', 'near': 'U1', 'max_mm': 2.0},
                           {'ref': 'Y1', 'near': 'U1', 'max_mm': 2.0,
                            'pads': {'Y1': ['77']}}]}, fh)
        doc = terms(PLACED, intent=p)
    t = doc['terms']['cluster_to_pin']
    check('the unresolved claims are counted, not discarded',
          t.get('unresolved', 0) >= 2, f"unresolved={t.get('unresolved')}")
    check('...and named, so the author can fix the file',
          bool(t.get('unresolved_claims')),
          str((t.get('unresolved_claims') or [])[:1]))
    check('nothing declared resolved, so the term refuses rather than '
          'reporting a maximum over what happened to survive',
          t['ran'] is False and t['value'] is None
          and 'failed to resolve' in (t['reason'] or ''),
          f"ran={t['ran']} value={t['value']!r} reason={(t['reason'] or '')[:70]!r}")


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

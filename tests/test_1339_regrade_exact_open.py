#!/usr/bin/env python3
"""The final re-grade does not credit a pour the oracle left open (#1339).

route.py's end-of-run re-grade (#1069) grades every net the run owns on the
board it ships, through the raster fill model. A pour net that model credits
can still be split by KiCad's exact fill: the plane finalize's oracle said so
("5 link(s) still unconnected per KiCad"), the reconciliation's forced links
(#572) failed on it, and the re-grade then graded the net connected and
listed it as "recovered" -- JSON_SUMMARY_MIN failed 0, --json-out
regrade.recovered ["GND", ...] -- while check_connected and board_score read
GND in 5 pieces (multichannel_mixer pour_s3, the issue's route.py line).

The reconciliation already refuses that credit at its entrance (#572's
assume_open); the re-grade now does the same with the links still pending
after the last lap. A pending net the model credits is graded on KiCad's
exact fill of the written file (kicad_exact_fill.refill_islands +
exact_clusters), and where no fill can be had (no KiCad python, the GUI's
unsaved board) on the oracle's own verdict: one secondary cluster per link.

  1. exact fill splits the net: broken, the stranded pad named;
  2. exact fill joins it: the model's connected grade stands;
  3. no fill: broken on the verdict, one failed entry per pending link;
  4. through _final_regrade: the net is FAILED, not recovered, and the
     record says which grade it took;
  5. batch_route hands the pending links to the re-grade.

    python3 tests/test_1339_regrade_exact_open.py
"""
import ast
import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for p in (ROOT, TESTS_DIR, os.path.join(ROOT, 'py_router')):
    if p not in sys.path:
        sys.path.insert(0, p)

from synth import RoutingBoard                                  # noqa: E402
from kicad_parser import parse_kicad_pcb                        # noqa: E402
from run_utils import evidence                                  # noqa: E402

FAILS = []
LINK = ('GND', (30.0, 15.0, 'F.Cu', 'pad'), (10.0, 15.0, 'F.Cu', 'zone'))


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name
          + (f'  {detail}' if detail else ''))


def board(tmp):
    b = RoutingBoard(w=40, h=30)
    b.pad('G1', 10, 15, 'GND')
    b.pad('G2', 30, 15, 'GND')
    b.pad('S1', 5, 5, 'SIG')
    b.pad('S2', 35, 5, 'SIG')
    b.seg(5, 5, 35, 5, 'SIG', w=0.2)
    path = os.path.join(tmp, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(b.text())
    return path, parse_kicad_pcb(evidence(path, 'synthetic board'))


def square(cx, cy, r):
    return [(cx - r, cy - r), (cx + r, cy - r), (cx + r, cy + r), (cx - r, cy + r)]


def with_islands(islands, fn):
    """Run fn() with kicad_exact_fill.refill_islands answering `islands`."""
    import kicad_exact_fill as kef
    real = kef.refill_islands
    kef.refill_islands = lambda *a, **k: islands
    try:
        return fn()
    finally:
        kef.refill_islands = real


def unit(tmp):
    from route import _exact_open_grades
    path, pcb = board(tmp)
    print('1. exact fill splits the net')
    split = {('GND', 'F.Cu'): [square(10, 15, 2)]}       # reaches G1 only
    out = with_islands(split, lambda: _exact_open_grades(
        pcb, path, ['GND'], [LINK]))
    g, src = out.get('GND', (None, None))
    check('broken on the exact fill, G2 named',
          g and g['broken'] and src == 'exact fill'
          and [f['component_ref'] for f in g['failed_pads']] == ['G2']
          and g['pad_pairs'] == [1, 0], str(out))
    print('2. exact fill joins it')
    joined = {('GND', 'F.Cu'): [[(8, 13), (32, 13), (32, 17), (8, 17)]]}
    out = with_islands(joined, lambda: _exact_open_grades(
        pcb, path, ['GND'], [LINK]))
    check('no override: the model grade stands', out == {}, str(out))
    print('3. no fill to be had: the oracle verdict')
    out = with_islands(None, lambda: _exact_open_grades(
        pcb, path, ['GND'], [LINK, LINK]))
    g, src = out.get('GND', (None, None))
    check('broken on the verdict, one entry per pending link',
          g and g['broken'] and src == 'oracle verdict'
          and len(g['failed_pads']) == 2, str(out))
    out = _exact_open_grades(pcb, None, ['GND'], [LINK])
    check('no file at all (the GUI front): the verdict too',
          out.get('GND', (None, None))[1] == 'oracle verdict', str(out))
    return path, pcb


def through_regrade(path, pcb):
    print('4. through _final_regrade')
    import improvement_gate
    import route
    from route_summary import SUMMARY_SINK, FINAL_REGRADE
    real_grade = improvement_gate.grade_nets

    def model_credit(board, ids, **kw):
        # The raster model credits the pour: every net connected.
        return {nid: {'pads': len(board.pads_by_net.get(nid, [])),
                      'broken': False, 'copper': True,
                      'pad_pairs': [1, 1], 'failed_pads': []}
                for nid in ids}
    SUMMARY_SINK.clear()
    SUMMARY_SINK.append({'routed_single': ['SIG'], 'failed_single': ['GND'],
                         'open_single': [], 'failed_multipoint': []})
    improvement_gate.grade_nets = model_credit
    try:
        split = {('GND', 'F.Cu'): [square(10, 15, 2)]}
        rec = with_islands(split, lambda: route._final_regrade(
            pcb, path, False, None, None, ['SIG', 'GND'], None, {}, {},
            exact_links=[LINK]))
    finally:
        improvement_gate.grade_nets = real_grade
        SUMMARY_SINK.clear()
        FINAL_REGRADE[0] = None
    check('GND failed, not recovered',
          rec and 'GND' in rec['failed_single']
          and 'GND' not in (rec.get('recovered') or []) and rec['failed'] == 1,
          str({k: rec.get(k) for k in ('failed_single', 'recovered', 'failed')}
              if rec else rec))
    check('the record says which grade it took',
          rec and rec.get('exact_open') == {'GND': 'exact fill'},
          str(rec.get('exact_open') if rec else rec))
    from route_summary import _apply_regrade
    merged = {}
    _apply_regrade(merged, rec, [{'failed_single': ['GND']}])
    check('--json-out carries it: GND failed, regrade.exact_open set',
          'GND' in (merged.get('failed_single') or [])
          and (merged.get('regrade') or {}).get('exact_open')
          == {'GND': 'exact fill'}, str(merged.get('regrade')))


def wiring():
    print('5. batch_route hands the pending links to the re-grade')
    tree = ast.parse(open(os.path.join(ROOT, 'py_router', 'route.py'),
                          encoding='utf-8').read())
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
             and getattr(n.func, 'id', None) == '_final_regrade']
    check('the one _final_regrade call passes exact_links=',
          len(calls) == 1 and any(k.arg == 'exact_links'
                                  for k in calls[0].keywords),
          f'{len(calls)} call(s)')


def main():
    with tempfile.TemporaryDirectory() as tmp:
        path, pcb = unit(tmp)
        through_regrade(path, pcb)
    wiring()
    print(f"\n{'ALL PASS' if not FAILS else f'{len(FAILS)} FAILED: ' + ', '.join(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())

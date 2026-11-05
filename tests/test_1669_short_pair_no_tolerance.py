#!/usr/bin/env python3
"""#1669: a pair too short to couple records no intra-pair tolerance.

route_diff records a tolerance for each pair it matched P to N (#1503), and
check_drc and board_score then hold the pair to it. A pair shorter than the
coupled floor routes single-ended and was never matched, and no copper at that
step can match it, yet it recorded one, so check_drc reported SKEW-OUT-OF-RANGE
on a pair whose listed fix is "None". On a small USB pair that ships 0.14 mm of
skew against the 0.1 mm default:

  A. the pair reads `not-matchable` with `failure_reason` `electrically-short`,
     its `delta_mm` stays measured, and no tolerance is recorded;
  B. check_drc reports no skew violation on it;
  C. a .kicad_dru rule the board declares still grades it;
  D. the engine call the differential tab makes notes no tolerance either
     (`return_results=True` with no output file, as the tab calls it);
  E. only this case is dropped: a `not-matchable` pair for any other reason, an
     `unmatched` one and a `matched` one record; a `skipped` one does not.

    python3 -X utf8 tests/test_1669_short_pair_no_tolerance.py
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
import run_utils  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
BOARD = os.path.join('kicad_files', 'watchy.kicad_pcb')
RULE = ('(version 1)\n(rule "declared pair skew"\n  (condition "A.inDiffPair(\'D*\')")\n'
        '  (constraint skew (max 0.1mm) (within_diff_pairs)))\n')
FAILS = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        FAILS.append(label)


def run(argv):
    return subprocess.run(PY + argv, capture_output=True, text=True, encoding='utf-8',
                          errors='replace', cwd=REPO)


def route(td, name, rule=False):
    d = os.path.join(td, name)
    os.makedirs(d)
    src = os.path.join(d, 'in.kicad_pcb')
    run([os.path.join('py_router', 'copy_board.py'), BOARD, src])
    run_utils.evidence(src)
    if rule:
        with open(os.path.join(d, 'in.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write(RULE)
    out = os.path.join(d, 'out.kicad_pcb')
    js = os.path.join(d, 'out.json')
    r = run([os.path.join('py_router', 'route_diff.py'), src, out, '--nets', 'DP', 'DN',
             '--diff-pair-intra-match', '--json-out', js])
    if r.returncode != 0:
        raise AssertionError(f'BROKEN TEST: route_diff exited {r.returncode}: {r.stdout[-800:]}{r.stderr[-800:]}')
    run_utils.evidence(out)
    with open(run_utils.evidence(js), encoding='utf-8') as fh:
        return out, json.load(fh), src


def recorded(out):
    with open(os.path.splitext(out)[0] + '.kicad_pro', encoding='utf-8') as fh:
        return (json.load(fh).get('kicad_routing_tools') or {}).get('intra_pair_tolerance') or {}


def gui_engine_call(src):
    """The call the differential tab makes: PCBData handed in, no output file,
    results returned; what it noted is what gui_utils persists."""
    from kicad_parser import parse_kicad_pcb
    from protected_nets import consume_intra_pair_tolerances
    from route_diff import batch_route_diff_pairs
    consume_intra_pair_tolerances()
    with contextlib.redirect_stdout(io.StringIO()):
        batch_route_diff_pairs(src, '', ['DP', 'DN'], diff_pair_intra_match=True,
                               return_results=True, pcb_data=parse_kicad_pcb(src))
    return consume_intra_pair_tolerances()


def only_this_case():
    from route_diff import intra_pair_reports_to_record as pick
    rec = lambda pair, status, tol=0.1: {'pair': pair, 'status': status, 'tolerance_mm': tol}
    reports = [rec('short', 'not-matchable'), rec('fallback', 'not-matchable'),
               rec('unmatched', 'unmatched'), rec('matched', 'matched'),
               rec('ac', 'skipped'), rec('none', 'matched', None),
               rec('short-but-joined', 'within-tolerance')]
    pairs = [{'pair': 'short', 'failure_reason': 'electrically-short'},
             {'pair': 'short-but-joined', 'failure_reason': 'electrically-short'},
             {'pair': 'fallback', 'failure_reason': 'congestion'}]
    got = {r['pair'] for r in pick(reports, pairs)}
    check('a not-matchable pair for another reason, an unmatched and a matched one record',
          {'fallback', 'unmatched', 'matched'} <= got, f'{got}')
    check('a short not-matchable pair, a skipped one and one with no tolerance do not',
          not ({'short', 'ac', 'none'} & got), f'{got}')
    check('a short pair the step did match keeps its record', 'short-but-joined' in got, f'{got}')


def main():
    td = tempfile.mkdtemp(prefix='krt1669_')
    try:
        out, js, src = route(td, 'plain')
        print('A. the pair and its record')
        rows = {r['pair']: r for r in js['intra_pair_matching']}
        reports = {r['pair']: r for r in js['pair_reports']}
        check('precondition: the pair is too short to couple',
              reports.get('D', {}).get('failure_reason') == 'electrically-short', f'{reports}')
        d = rows.get('D', {})
        check('it reads not-matchable', d.get('status') == 'not-matchable', f'{d}')
        check('its skew stays measured, over the default tolerance',
              (d.get('delta_mm') or 0) > 0.1, f'{d}')
        check('no tolerance is recorded', recorded(out) == {}, json.dumps(recorded(out)))

        print('B. check_drc')
        r = run([os.path.join('py_router', 'check_drc.py'), out])
        check('no skew violation', 'SKEW-OUT-OF-RANGE' not in r.stdout, r.stdout[-300:])

        print('C. a declared rule still grades it')
        out2, _js, _src = route(td, 'rule', rule=True)
        check('no tolerance is recorded', recorded(out2) == {}, json.dumps(recorded(out2)))
        r = run([os.path.join('py_router', 'check_drc.py'), out2])
        check("check_drc grades the pair by the board's own rule",
              'SKEW-OUT-OF-RANGE' in r.stdout and 'declared pair skew' in r.stdout, r.stdout[-400:])

        print('D. the engine call the differential tab makes')
        noted = gui_engine_call(src)
        check('it notes no tolerance', noted == {}, json.dumps(noted))

        print('E. only this case')
        only_this_case()
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if FAILS:
        print(f'FAILED: {len(FAILS)}: ' + '; '.join(FAILS))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())

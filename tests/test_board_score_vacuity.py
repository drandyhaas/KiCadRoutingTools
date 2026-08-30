#!/usr/bin/env python3
"""board_score --impedance-nets vacuity: a zero-match must be UNKNOWN, not silence.

The defect this guards (test-board run 5, journal [11]): the comma-joined form
`--impedance-nets 'USB_DP,USB_DM,...'` became ONE fnmatch pattern requiring a
literal comma, matched nothing, and score_impedance returned skipped() -- so the
component landed in `ungraded`, `blocking` summed without it, and the exit code
read 0. The impedance component was vacuous for four whole runs on one board
while every summary looked complete. Two behaviors are pinned here:

  1. comma-joined tokens are split into globs (the natural way to type a list);
  2. a genuine zero-match reports ran=True/count=None -> the component lands in
     `unknown`, `blocking` is null, and the exit code is 4 -- per the script's
     own Vacuity doctrine, "asked for but not measured" can never read as pass.
"""
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
# #522 reorg + skill merge: the engine moved to py_router/, the placer to
# py_placer/, and board_score.py into the placement-and-routing skill. Tests
# that shell out to or import them need those roots on sys.path.
for _p in ('py_router', 'py_placer',
           os.path.join('.claude', 'skills', 'plan-pcb-placement-and-routing',
                        'scripts')):
    _d = os.path.join(ROOT, _p)
    if _d not in sys.path:
        sys.path.insert(0, _d)
BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
SCORE = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement-and-routing',
                     'scripts', 'board_score.py')


def _score(board, extra, td):
    js = os.path.join(td, 'score.json')
    r = subprocess.run([sys.executable, '-X', 'utf8', SCORE, board,
                       '--json', js] + extra,
                      capture_output=True, text=True, encoding='utf-8',
                      errors='replace', cwd=ROOT)
    data = json.load(open(js, encoding='utf-8')) if os.path.isfile(js) else {}
    return r.returncode, data, r.stdout + r.stderr


def _routed_board(td):
    """Route two nets of the fixture so the impedance component has copper."""
    out = os.path.join(td, 'routed.kicad_pcb')
    r = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_router', 'route.py'), BOARD, out,
                        '--nets', '/OUT_A_PHASE_A', '/OUT_A_PHASE_B'],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT)
    assert r.returncode == 0, r.stdout[-2000:]
    return out


def test_zero_match_is_unknown_and_exit_4():
    if not os.path.isfile(BOARD):
        print("  SKIP: fixture missing")
        return
    with tempfile.TemporaryDirectory() as td:
        rc, data, out = _score(BOARD, ['--impedance-nets', 'NO_SUCH,ALSO_NO'], td)
        assert rc == 4, f"exit {rc}, wanted 4 (vacuous ask must not pass)\n{out[-1500:]}"
        assert 'impedance' in (data.get('unknown') or []), \
            f"impedance not in unknown: {data.get('unknown')} / ungraded {data.get('ungraded')}"
        assert data.get('blocking') is None, \
            f"blocking summed without the asked-for component: {data.get('blocking')}"
        print("  PASS: zero-match lands in unknown, blocking null, exit 4")


def test_comma_joined_real_nets_grade():
    if not os.path.isfile(BOARD):
        print("  SKIP: fixture missing")
        return
    with tempfile.TemporaryDirectory() as td:
        routed = _routed_board(td)
        rc, data, out = _score(routed, ['--impedance-nets', '/OUT_A_PHASE_A,/OUT_A_PHASE_B'], td)
        comp = (data.get('components') or {}).get('impedance') or {}
        assert comp.get('ran') is True and comp.get('nets_analyzed'), \
            f"comma-joined real nets did not grade: {comp}\n{out[-1500:]}"
        assert 'impedance' not in (data.get('unknown') or []), data.get('unknown')
        print(f"  PASS: comma-joined pair graded ({comp.get('nets_analyzed')} nets analyzed)")


def test_net_min_widths_comment_key_is_not_a_pattern():
    """A "_comment" annotation key in --net-min-widths must not pollute
    patterns_matching_no_routed_net (the field a reader scans for typo'd
    globs) -- run 6 carried it in every score."""
    if not os.path.isfile(BOARD):
        print("  SKIP: fixture missing")
        return
    with tempfile.TemporaryDirectory() as td:
        spec = os.path.join(td, 'widths.json')
        with open(spec, 'w', encoding='utf-8') as fh:
            json.dump({'_comment': 'annotation, not a net', 'NO_SUCH_NET': 0.4}, fh)
        rc, data, out = _score(BOARD, ['--net-min-widths', spec], td)
        comp = (data.get('components') or {}).get('net_widths') or {}
        unmatched = comp.get('patterns_matching_no_routed_net') or []
        assert '_comment' not in unmatched, unmatched
        assert 'NO_SUCH_NET' in unmatched, unmatched
        print("  PASS: _comment filtered, real unmatched glob still reported")


if __name__ == '__main__':
    test_zero_match_is_unknown_and_exit_4()
    test_comma_joined_real_nets_grade()
    test_net_min_widths_comment_key_is_not_a_pattern()

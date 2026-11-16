#!/usr/bin/env python3
"""#1764: route_diff says WHY each net is left single-ended, and points a
failed pair at route_diff, not route.py.

The log banner read "N net(s) have far-apart terminal pads peeled from the
coupled chain - route them single-ended next" with a `route.py` command for
every net of `state.diff_pair_single_ended_nets` -- which also holds the
members of a pair whose COUPLED ROUTE FAILED (#289's 2-terminal defer, #165's
multipoint one). Routing such a pair with route.py de-pairs it (#1399). It
also printed before the step's own single-ended fallback, so it promised a
command whose outcome was not known yet.

Now:
  1. `single_ended_followup_reasons` ({net: why}) sits next to
     `single_ended_followup_nets`, whose contents are unchanged (autoroute
     reads it, failed pairs included);
  2. the banner groups the nets by why, prints after the fallback, gives a
     by-design net still open the route.py command and a failed pair a
     `route_diff.py ... --force-reroute` one;
  3. the GUI's completion dialog splits the deferred pairs the same way
     (`split_deferred_pairs`).

Fixtures: tigard /USB_D (multipoint, coupled chain fails -- pinned by
test_multipoint_coupled_fail_reason.py) and watchy DP/DN (electrically short,
deferred by design). About a minute.

    python3 -X utf8 tests/test_1764_followup_reasons.py
"""
import json
import os
import subprocess
import sys
import tempfile
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
from run_utils import evidence  # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    print(('  PASS  ' if cond else '  FAIL  ') + name + (f'  [{detail}]' if detail else ''))
    if not cond:
        FAILS.append(name)


def unit_rows():
    from diff_pair_custody import (single_ended_followup_reasons,
                                   single_ended_followup_lines,
                                   split_deferred_pairs, FOLLOWUP_BY_DESIGN)
    pair = lambda p, n, pn, nn: SimpleNamespace(p_net_id=p, n_net_id=n,
                                                p_net_name=pn, n_net_name=nn)
    pairs = [('A', pair(1, 2, 'A_P', 'A_N')),      # peeled: coupled trunk kept
             ('B', pair(3, 4, 'B_P', 'B_N')),      # electrically short
             ('C', pair(5, 6, 'C_P', 'C_N')),      # coupled route failed
             ('D', pair(7, 8, 'D_P', 'D_N'))]      # coupled, nothing deferred
    coupled = {'is_diff_pair': True, 'new_segments': [1]}
    fb = {'se_member_fallback': True}
    state = SimpleNamespace(
        diff_pair_single_ended_nets={1: 'A_P', 2: 'A_N', 3: 'B_P', 4: 'B_N',
                                     5: 'C_P', 6: 'C_N'},
        routed_results={1: coupled, 2: coupled, 3: fb, 4: fb, 5: fb,
                        7: coupled, 8: coupled},
        pair_diagnostics={'B': {'reason': 'electrically-short'},
                          'C': {'reason': 'no-escape-path'}})
    got = single_ended_followup_reasons(state, pairs)
    check('reasons: peeled / short / failed, one per deferred net',
          got == {'A_P': 'peeled-terminals', 'A_N': 'peeled-terminals',
                  'B_P': 'electrically-short', 'B_N': 'electrically-short',
                  'C_P': 'no-escape-path', 'C_N': 'no-escape-path'}, str(got))
    check('a fallback result is not a coupled one (C is not peeled)',
          got.get('C_N') not in FOLLOWUP_BY_DESIGN, got.get('C_N'))
    lines = single_ended_followup_lines(
        got, open_nets={'A_P', 'A_N', 'C_N'},
        fallback={'routed': ['B_P', 'B_N', 'C_P'], 'failed': ['C_N']})
    text = '\n'.join(lines)
    eg = [ln for ln in lines if 'e.g.' in ln]
    check('route.py is offered for the by-design nets still open only',
          any("route.py <this_output> <out2> --nets 'A_N' 'A_P'" in ln
              and 'B_' not in ln and 'C_' not in ln for ln in eg), str(eg))
    check('a failed pair gets route_diff --force-reroute, never route.py',
          any("route_diff.py <this_output> <out2> --nets 'C_N' 'C_P' --force-reroute"
              in ln for ln in eg)
          and not any('route.py' in ln and 'C_' in ln for ln in eg), str(eg))
    check('the block says what the fallback did',
          "C_N  (no-escape-path; this step's single-ended fallback: failed; still open)"
          in text and "B_P  (electrically-short; this step's single-ended "
                      "fallback: routed; connected)" in text, text)
    check('the old "peeled ... route them single-ended next" words are gone',
          'peeled from the coupled chain' not in text)
    lines = single_ended_followup_lines({'B_P': 'electrically-short'}, set())
    check('nothing open by design: no route.py command',
          not any('e.g.' in ln for ln in lines)
          and any('nothing is left for route.py' in ln for ln in lines),
          '\n'.join(lines))
    check('no deferred net: no block', single_ended_followup_lines({}, set()) == [])
    reps = [{'pair': 'B', 'failure_reason': 'electrically-short'},
            {'pair': 'C', 'failure_reason': 'coupled-route-failed'},
            {'pair': 'E', 'failure_reason': None}]
    check('GUI split: short by design, failed and unknown retried as pairs',
          split_deferred_pairs(reps, ['B', 'C', 'E']) == (['B'], ['C', 'E']),
          str(split_deferred_pairs(reps, ['B', 'C', 'E'])))


def route(td, name, board, nets):
    src = os.path.join(td, f'{name}.in.kicad_pcb')
    subprocess.run([sys.executable, '-X', 'utf8', 'py_router/copy_board.py',
                    os.path.join('kicad_files', f'{board}.kicad_pcb'), src],
                   cwd=ROOT, capture_output=True, text=True)
    evidence(src, f'{board} copy')
    js = os.path.join(td, f'{name}.json')
    r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/route_diff.py',
                        src, os.path.join(td, f'{name}.out.kicad_pcb'),
                        '--json-out', js, '--nets', *nets],
                       cwd=ROOT, capture_output=True, text=True,
                       encoding='utf-8', errors='replace')
    log = r.stdout + r.stderr
    check(f'{name}: route_diff exits 0', r.returncode == 0,
          '' if r.returncode == 0 else log[-400:])
    evidence(js, f'{name} --json-out')
    with open(js, encoding='utf-8') as fh:
        return log, json.load(fh)


def board_rows():
    os.environ['MSYS2_ARG_CONV_EXCL'] = '*'
    with tempfile.TemporaryDirectory() as td:
        log_a, a = route(td, 'A', 'tigard', ['/USB_D*'])
        log_b, b = route(td, 'B', 'watchy', ['DP', 'DN'])
    rep_a = {r['pair']: r for r in a.get('pair_reports', [])}.get('/USB_D', {})
    check('fixture A: /USB_D failed coupled (coupled-route-failed)',
          rep_a.get('failure_reason') == 'coupled-route-failed',
          str(rep_a.get('failure_reason')))
    check('A: single_ended_followup_nets unchanged (both members still listed)',
          a.get('single_ended_followup_nets') == ['/USB_DN', '/USB_DP'],
          str(a.get('single_ended_followup_nets')))
    check('A: single_ended_followup_reasons names the failure',
          a.get('single_ended_followup_reasons') == {
              '/USB_DN': 'coupled-route-failed', '/USB_DP': 'coupled-route-failed'},
          str(a.get('single_ended_followup_reasons')))
    check('A: the log offers no route.py for the failed pair',
          "e.g.  route.py" not in log_a
          and "e.g.  route_diff.py <this_output> <out2> --nets '/USB_DN' "
              "'/USB_DP' --force-reroute" in log_a)
    check('A: the banner follows the single-ended fallback',
          log_a.find('Single-ended member fallback') < log_a.find('left single-ended by this pair step')
          and log_a.find('Single-ended member fallback') >= 0)
    check('B: the short pair is listed by design',
          b.get('single_ended_followup_reasons') == {
              'DN': 'electrically-short', 'DP': 'electrically-short'},
          str(b.get('single_ended_followup_reasons')))
    check('B: no route_diff retry for a by-design pair',
          'route_diff.py <this_output>' not in log_b
          and 'by design' in log_b)


def main():
    unit_rows()
    board_rows()
    if FAILS:
        print(f'\nFAIL: {len(FAILS)} check(s): ' + '; '.join(FAILS))
        return 1
    print('\nALL CHECKS PASSED')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

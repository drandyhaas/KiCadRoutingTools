#!/usr/bin/env python3
"""A multipoint pair whose coupled chain failed is not reported electrically
short.

Issue #165 defers a multipoint pair to single-ended when no coupled chain
routes and relocation does not help, and signals it with the same return as
the all-legs-electrically-short case. The loop then recorded reason
'electrically-short' and printed "All N legs electrically short" for it.

Fixture: tigard /USB_D (USB-C J1 -> U3) takes exactly that path today: the
log carries "Coupled routing failed and relocation unavailable". Its
pair_reports entry must say 'coupled-route-failed', and the end-of-run
summary line -- which the GUI's completion dialog words the same way, through
diff_pair_custody.deferred_reasons_text -- must count it as that, not as
electrically short. The unit rows hold that wording to every deferred reason,
including a 2-terminal pair deferred after its coupled route failed, which the
old "electrically short" line also misnamed.

    python3 tests/test_multipoint_coupled_fail_reason.py
"""
import json
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')
FAILS = []


def check(name, cond, detail=''):
    print(('  PASS  ' if cond else '  FAIL  ') + name + (f'  [{detail}]' if detail else ''))
    if not cond:
        FAILS.append(name)


def unit_rows():
    sys.path.insert(0, os.path.join(ROOT, 'py_router'))
    try:
        from diff_pair_custody import deferred_reasons_text
    except ImportError as e:
        check('diff_pair_custody.deferred_reasons_text exists', False, str(e))
        return
    reps = [{'pair': 'A', 'outcome': 'deferred', 'failure_reason': 'electrically-short'},
            {'pair': 'B', 'outcome': 'deferred', 'failure_reason': 'electrically-short'},
            {'pair': 'C', 'outcome': 'deferred', 'failure_reason': 'coupled-route-failed'},
            {'pair': 'D', 'outcome': 'deferred', 'failure_reason': 'pose-router-failure'},
            {'pair': 'E', 'outcome': 'deferred', 'failure_reason': None},
            {'pair': 'F', 'outcome': 'coupled', 'failure_reason': None}]
    got = deferred_reasons_text(reps, ['A', 'B', 'C', 'D', 'E'])
    check('the deferred-reason wording counts each reason, unknown included',
          got == '1 coupled route failed, 2 electrically short, '
                 '1 pose router failure, 1 unknown', got)
    check('...and only the pairs it is asked about',
          deferred_reasons_text(reps, ['C']) == '1 coupled route failed',
          deferred_reasons_text(reps, ['C']))


def main():
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, 'out.kicad_pcb')
        r = subprocess.run([sys.executable, 'py_router/route_diff.py', BOARD,
                            '--output', out, '--nets', '/USB_D*'],
                           cwd=ROOT, capture_output=True, text=True)
    log = r.stdout + r.stderr
    check('route_diff exits 0', r.returncode == 0, f'exit {r.returncode}')
    check('fixture: the coupled chain failed and the pair was deferred',
          'Coupled routing failed and relocation unavailable' in log)
    m = re.search(r'JSON_SUMMARY:\s*(\{.*\})', log)
    check('JSON_SUMMARY present', m is not None)
    if m:
        reps = {p['pair']: p for p in json.loads(m.group(1)).get('pair_reports', [])}
        rep = reps.get('/USB_D', {})
        check('/USB_D deferred', rep.get('outcome') == 'deferred', str(rep.get('outcome')))
        check("/USB_D reason is 'coupled-route-failed', not 'electrically-short'",
              rep.get('failure_reason') == 'coupled-route-failed',
              str(rep.get('failure_reason')))
    check('the log does not call the failed legs electrically short',
          'legs electrically short - whole pair' not in log)
    check('the summary line counts it as a failed coupled route',
          'Single-ended:  1 (deferred to single-ended routing: 1 coupled route failed)'
          in log, next((ln for ln in log.splitlines() if 'Single-ended:' in ln), ''))
    unit_rows()
    if FAILS:
        print(f'\nFAIL: {len(FAILS)} check(s): ' + '; '.join(FAILS))
        return 1
    print('\nALL CHECKS PASSED')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

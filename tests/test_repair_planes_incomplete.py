#!/usr/bin/env python3
"""repair_planes' JSON_SUMMARY says what it left unconnected.

Its summary was `complete: true, status: ok` whatever the written board held:
nothing graded that board after the oracle reconnect. Measured: on
kit-dev-coldfire under `--escalation board` check_connected found 5 GND pads
open (U102/U301); on watchy under `--escalation off`, 6 (J3, U4, U5, U6) --
both runs printed `status: ok` and exited 0, so a chain reading the summary
saw a clean plane step.

  A. watchy, pour GND/+3V3 on In1/In2, repair under `--escalation off`: the
     run must leave pads open (check_connected says so, else this fixture
     tests nothing -- BROKEN), and the summary must then say
     `status: incomplete-pads` and name exactly the pads check_connected
     names, in route.py's failure vocabulary. The exit code stays 0: a
     routing shortfall exits 0 on route.py and route_planes too, and 4 stays
     reserved for rip casualties (damage the run did).
  B. control, the same board at the default `--escalation fab`, which
     connects every pad: the summary must still say `status: ok` with no
     open pads, so A is not an instrument that always says "incomplete".

    python3 -X utf8 tests/test_repair_planes_incomplete.py
"""
import json
import os
import re
import subprocess
import sys
import tempfile
from collections import Counter

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)

from run_utils import check as run_check, evidence  # noqa: E402

RUN_ALL_TIMEOUT = 1200   # a pour + two repairs (~3 min alone, more under load)

FAILS = []
PY = [sys.executable, '-X', 'utf8']


def check(name, cond, detail=''):
    print(('  PASS ' if cond else '  FAIL ') + name
          + (f'\n        {detail}' if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def summary_of(stdout):
    lines = [ln for ln in stdout.splitlines() if ln.startswith('JSON_SUMMARY: ')]
    if not lines:
        raise AssertionError('BROKEN TEST: repair_planes printed no JSON_SUMMARY')
    return json.loads(lines[-1].split(': ', 1)[1])


def connectivity(board, nets):
    """check_connected's verdict: Counter of (net, ref) over disconnected pads.
    It exits 1 exactly when it finds an issue -- the reason is asserted."""
    argv = PY + [os.path.join(ROOT, 'py_router', 'check_connected.py'),
                 board, '--nets'] + nets
    r = subprocess.run(argv, capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT, timeout=900)
    out = r.stdout + r.stderr
    if r.returncode not in (0, 1) or 'Traceback' in out:
        raise AssertionError(f'BROKEN TEST: check_connected exited '
                             f'{r.returncode}\n{out[-1500:]}')
    if r.returncode == 1 and 'Disconnected pads:' not in out:
        raise AssertionError('BROKEN TEST: check_connected failed without '
                             'naming disconnected pads\n' + out[-1500:])
    opened = Counter()
    net = None
    for ln in out.splitlines():
        m = re.match(r'^  (\S.*?) \(net \d+\):\s*$', ln)
        if m:
            net = m.group(1)
            continue
        m = re.match(r'^\s+\([-\d.]+, [-\d.]+\) on \S+ \[(.+)\]\s*$', ln)
        if m and net:
            opened[(net, m.group(1))] += 1
    if r.returncode == 1 and not opened:
        raise AssertionError('BROKEN TEST: could not parse check_connected\'s '
                             'disconnected pads\n' + out[-1500:])
    return opened


def summary_open(s):
    got = Counter()
    for e in s.get('unconnected_pads') or []:
        m = re.match(r'^(.+?)\.(\S+) \((.+)\)$', e)
        got[(m.group(3), m.group(1)) if m else ('?', e)] += 1
    return got


def pad_labels():
    """An open pad's number, resolved off the graded point."""
    from types import SimpleNamespace as P
    sys.path.insert(0, os.path.join(ROOT, 'py_router'))
    from check_connected import pad_label
    pads = [P(component_ref='U1', pad_number='33', global_x=5.0,
              global_y=5.0, size_x=3.0, size_y=3.0),
            P(component_ref='U1', pad_number='34', global_x=5.0,
              global_y=5.0, size_x=3.0, size_y=3.0),
            P(component_ref='U1', pad_number='1', global_x=1.0,
              global_y=2.0, size_x=0.6, size_y=1.2),
            P(component_ref='C1', pad_number='2', global_x=5.0,
              global_y=5.0, size_x=1.0, size_y=1.0)]
    print('C. pad labels')
    check('stacked pads at one point are all named, not the first one',
          pad_label(pads, 'U1', 5.0, 5.0) == '33/34',
          str(pad_label(pads, 'U1', 5.0, 5.0)))
    check('a point off the centre is named by the pad that holds it',
          pad_label(pads, 'U1', 1.2, 2.5) == '1',
          str(pad_label(pads, 'U1', 1.2, 2.5)))
    check("another part's pad is never borrowed",
          pad_label(pads, 'C1', 5.0, 5.0) == '2'
          and pad_label(pads, 'U1', 9.0, 9.0) is None)


def main():
    pad_labels()
    src = evidence(os.path.join(ROOT, 'kicad_files', 'watchy.kicad_pcb'), 'fixture')
    nets = ['GND', '+3V3']
    with tempfile.TemporaryDirectory(prefix='rp_incomplete_') as td:
        poured = os.path.join(td, 'poured.kicad_pcb')
        run_check(PY + [os.path.join(ROOT, 'py_router', 'route_planes.py'), src,
                        poured, '--nets'] + nets
                  + ['--plane-layers', 'In1.Cu', 'In2.Cu'], accept=True)
        evidence(poured, 'poured board')

        print('A. --escalation off leaves pads open: the summary says which')
        out_a = os.path.join(td, 'off.kicad_pcb')
        r = run_check(PY + [os.path.join(ROOT, 'py_router', 'repair_planes.py'),
                            poured, out_a, '--escalation', 'off'], accept=True)
        s = summary_of(r.stdout)
        truth = connectivity(evidence(out_a, 'repaired board'), nets)
        if not truth:
            raise AssertionError('BROKEN TEST: check_connected finds every pad '
                                 'connected after --escalation off -- this '
                                 'fixture no longer exercises a shortfall')
        n = sum(truth.values())
        print(f'  (check_connected: {n} pad(s) open: {dict(truth)})')
        check('status is not ok', s.get('status') == 'incomplete-pads',
              f"status={s.get('status')!r}")
        check('unconnected_pads names exactly the pads check_connected finds',
              summary_open(s) == truth,
              f'summary {dict(summary_open(s))} vs check_connected {dict(truth)}')
        broken = sorted({net for net, _r in truth})
        check('failed_multipoint names the broken plane nets',
              sorted(d.get('net_name') for d in s.get('failed_multipoint') or [])
              == broken, str(s.get('failed_multipoint')))
        deficit = ((s.get('multipoint_pads_total') or 0)
                   - (s.get('multipoint_pads_connected') or 0))
        check('the multipoint pad deficit counts them', deficit == n,
              f'deficit {deficit} vs {n}')
        check('`failed` counts the broken nets', s.get('failed') == len(broken),
              f"failed={s.get('failed')}")
        check('every open pad carries its number',
              bool(s.get('failed_multipoint'))
              and all(fp.get('pad_number') not in (None, '?')
                  for d in s.get('failed_multipoint') or []
                  for fp in d.get('failed_pads') or []),
              str(s.get('failed_multipoint')))
        check('the console names them too', 'PLANE PADS LEFT UNCONNECTED' in r.stdout)
        check('...and says the policy bounded the descent',
              '--escalation off bounds the fine-pitch tap descent' in r.stdout)

        print('B. control: --escalation fab connects every pad, and says ok')
        out_b = os.path.join(td, 'fab.kicad_pcb')
        r = run_check(PY + [os.path.join(ROOT, 'py_router', 'repair_planes.py'),
                            poured, out_b], accept=True)
        s = summary_of(r.stdout)
        truth = connectivity(evidence(out_b, 'repaired board'), nets)
        check('check_connected agrees with the summary',
              summary_open(s) == truth,
              f'summary {dict(summary_open(s))} vs check_connected {dict(truth)}')
        if truth:
            raise AssertionError(
                f'BROKEN TEST: the fab arm left pads open too ({dict(truth)}), '
                f'so it is no longer a complete-run control')
        check('a complete run still says status ok',
              s.get('status') == 'ok' and s.get('failed') == 0
              and s.get('unconnected_pads') == []
              and 'PLANE PADS LEFT UNCONNECTED' not in r.stdout,
              f"status={s.get('status')!r} failed={s.get('failed')} "
              f"open={s.get('unconnected_pads')}")


if __name__ == '__main__':
    main()
    print()
    if FAILS:
        print(f'FAILED: {len(FAILS)} check(s): {FAILS}')
        sys.exit(1)
    print('All checks passed.')

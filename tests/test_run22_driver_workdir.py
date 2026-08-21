#!/usr/bin/env python3
"""The skill drivers must be able to point at the work dir being audited.

Run 22 ran three watchers over `wk/run22/tigard` and they reported a clean
run. They could not have reported otherwise: `run_watch --workdir`,
`fence_audit --workdir` and `provenance_audit --workdir` each walk ONLY the
directory they are given, and both drivers emitted instructions hardcoding a
relative `wk/locks.json`, `wk/drc0.json`, `wk/score.json` and ~70 more. Those
resolve against the process CWD, so the artifacts landed outside every audit.

That is a hole in the EVIDENCE rather than a cosmetic path problem: the
watchers were not wrong, they were pointed at a directory that half the run's
artifacts never entered.

Run: python3 -X utf8 tests/test_run22_driver_workdir.py
"""
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
os.environ.setdefault('KRT_NO_BANNER', '1')

PLACE = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement',
                     'scripts', 'placement_driver.py')
LOOP = os.path.join(ROOT, '.claude', 'skills',
                    'plan-pcb-placement-and-routing', 'scripts',
                    'loop_driver.py')

FAILURES = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}'
          + (f'\n        {detail}' if not cond and detail else ''))
    if not cond:
        FAILURES.append(name)


def run(script, *argv):
    env = dict(os.environ, PYTHONIOENCODING='utf-8', KRT_NO_BANNER='1')
    r = subprocess.run([sys.executable, '-X', 'utf8', script, *argv],
                       capture_output=True, text=True, env=env, cwd=ROOT)
    return r.stdout + r.stderr


#: The self-tests build boards in a fresh TemporaryDirectory each run, so two
#: invocations legitimately differ by that name and nothing else.
_TMP = re.compile(r'tmp[a-z0-9_]{6,}')


def main():
    print('the emitted instructions follow --workdir')
    out = run(PLACE, '--stage', 'P0', '--board', 'b.kicad_pcb',
              '--workdir', 'wk/run23/tigard')
    check('a retargeted stage names the caller\'s dir',
          'wk/run23/tigard/drc0.json' in out, out[:300])
    check('...and no bare wk/ path survives',
          not re.search(r'(?<![\w./-])wk/(?!run23)', out),
          [m for m in re.findall(r'(?<![\w./-])wk/[\w./-]*', out)][:6])

    print('the default is byte-identical -- the compatibility gate')
    # Both drivers carry large embedded self-test suites that construct args
    # WITHOUT --workdir. The default short-circuits so their expectations do
    # not have to be rewritten, which is the whole reason the default is 'wk'
    # rather than None.
    for name, script in (('placement_driver', PLACE), ('loop_driver', LOOP)):
        a = _TMP.sub('TMP', run(script, '--dump-all'))
        b = _TMP.sub('TMP', run(script, '--dump-all', '--workdir', 'wk'))
        check(f'{name} --dump-all is unchanged by an explicit default',
              a == b, 'the short-circuit is broken')

    print('the ledger the loop driver resolves ITSELF follows too')
    sys.path.insert(0, os.path.dirname(LOOP))
    import loop_driver as L
    check('the old default is preserved exactly',
          L._args(['--stage', 'L1', '--board', 'b']).ledger
          == 'wk/ledger.jsonl')
    check('and a named work dir carries the ledger into it',
          L._args(['--stage', 'L1', '--board', 'b',
                   '--workdir', 'wk/run23/x']).ledger
          == 'wk/run23/x/ledger.jsonl',
          'the ledger would sit outside the audited dir')

    print('real repo artifacts under wk/ are NOT retargeted')
    # `wk/calibration/RESULT.md` and `wk/run11/...` are prose references to
    # things that exist in the repo, not work paths the caller owns.
    keep = L._retarget('see wk/calibration/RESULT.md and write wk/score.json',
                       'wk/run23/x')
    check('a calibration reference survives',
          'wk/calibration/RESULT.md' in keep, keep)
    check('...while a work path beside it is retargeted',
          'wk/run23/x/score.json' in keep, keep)

    print('both self-test suites still pass')
    for name, script in (('placement_driver', PLACE), ('loop_driver', LOOP)):
        env = dict(os.environ, PYTHONIOENCODING='utf-8', KRT_NO_BANNER='1')
        rc = subprocess.run([sys.executable, '-X', 'utf8', script,
                             '--self-test'], capture_output=True, text=True,
                            env=env, cwd=ROOT).returncode
        check(f'{name} --self-test exits 0', rc == 0, f'exit {rc}')

    print()
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s): {", ".join(FAILURES)}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())

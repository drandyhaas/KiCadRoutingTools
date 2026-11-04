"""#1499: route.py on an already-routed net cleans its stray copper, and
check_join stages a join at the net's own class width.

(c) A connected net takes route.py's "nothing to route" return, which ran
    only the dangling-via and orphan sweeps: a KiCad-dangling joint and a
    dead-end stub shipped again, and only --force-reroute (which rips the
    net) cleared them. The return now runs the end of a run's cleanups --
    the #1217 split, the #1063 strict collapse, the #162 dedup.
(d) check_join took the Default class width; it now takes the net's own
    class (as route.py routes it), then Default.

The sense_star fixture plus two 0.1 mm VDD branches, a 0.122 mm dead-end
stub and a 0.1 mm segment lying on a branch; VDD in a 0.1 mm class under a
0.16 mm Default (the issue's own mk_stub).

    python3 tests/test_1499_rerun_cleans_and_join_width.py
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

RUN_ALL_TIMEOUT = 600
PY = [sys.executable, '-B', '-X', 'utf8']
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def run(argv):
    return subprocess.run(PY + argv, capture_output=True, text=True, cwd=REPO)


def make_stub(src, dst):
    segs = [((15, 5), (15, 8)), ((20, 5), (20, 8)),
            ((15, 6.5), (15.122, 6.5)), ((20, 6), (20, 6.1))]
    s = open(src, encoding='utf-8').read().rstrip()
    s = '\n'.join(ln for ln in s.split('\n') if not ln.lstrip().startswith('(group "sense:'))
    i = s.rfind(')')
    add = ''.join(f'  (segment (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) (width 0.1) '
                  f'(layer "F.Cu") (net 1) (uuid "11111111-0000-0000-0000-0000000001{k:02d}"))\n'
                  for k, (a, b) in enumerate(segs))
    open(dst, 'w', encoding='utf-8').write(s[:i] + add + s[i:] + '\n')
    pro = json.load(open(os.path.splitext(src)[0] + '.kicad_pro', encoding='utf-8'))
    pro['board']['design_settings']['rules']['min_track_width'] = 0.1
    ns = pro['net_settings']
    default = next(c for c in ns['classes'] if c['name'] == 'Default')
    default['track_width'] = 0.16
    ns['classes'].append(dict(default, name='Narrow', track_width=0.1, priority=0))
    ns['netclass_patterns'] = [{'netclass': 'Narrow', 'pattern': 'VDD'}]
    json.dump(pro, open(os.path.splitext(dst)[0] + '.kicad_pro', 'w', encoding='utf-8'), indent=2)


def weird(board):
    out = run([os.path.join('py_router', 'check_weird.py'), board]).stdout
    return ('NO WEIRD THINGS FOUND' in out,
            re.findall(r'(dangling-end|kicad-dangling): (\d+)', out))


def main():
    td = tempfile.mkdtemp(prefix='krt1499_')
    try:
        base, stub = os.path.join(td, 'base.kicad_pcb'), os.path.join(td, 'stub.kicad_pcb')
        run([os.path.join('py_router', 'copy_board.py'),
             os.path.join('tests', 'fixtures', 'sense_branch', 'sense_star.kicad_pcb'), base])
        run_utils.evidence(base)
        make_stub(base, stub)
        clean, found = weird(stub)
        check('precondition: the stub board has a dangling end and a KiCad-dangling joint',
              not clean and dict(found) == {'dangling-end': '1', 'kicad-dangling': '1'}, f'{found}')

        print('(c) route.py --nets VDD on the connected net')
        out = os.path.join(td, 'rerun.kicad_pcb')
        r = run([os.path.join('py_router', 'route.py'), stub, out, '--nets', 'VDD'])
        run_utils.evidence(out)
        check('it takes the "nothing to route" return', 'nothing to route' in r.stdout,
              r.stdout[-400:])
        clean, found = weird(out)
        check('...and ships no stray copper', clean, f'{found}')
        con = run([os.path.join('py_router', 'check_connected.py'), out]).stdout
        check('...with every net still connected', 'ALL NETS FULLY CONNECTED' in con, con[-300:])

        print('(d) check_join stages at the net\'s own class width')
        staged = os.path.join(td, 'join.kicad_pcb')
        r = run([os.path.join('py_tools', 'check_join.py'), stub, 'VDD', '15,7,F.Cu', '16,7,F.Cu',
                 '--keep-staged', staged])
        run_utils.evidence(staged)
        text = open(staged, encoding='utf-8').read()
        m = re.search(r'\(start 15\.0+ 7\.0+\)\s*\(end 16\.0+ 7\.0+\)\s*\(width ([0-9.]+)\)', text)
        check("the staged join is VDD's class width, 0.1 (not Default's 0.16)",
              m is not None and float(m.group(1)) == 0.1, m.group(0) if m else 'no join found')
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())

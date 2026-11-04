#!/usr/bin/env python3
"""#1391: length matching aims INTO the board's .kicad_dru length window, counts
the group's board-resident members toward the target, and reports a verdict
for every member.

An 8-net bus (interf_u's /BIT0../BIT7) with a dru window of 39-41 mm:

  * the group target used to be the longest member (38.29 mm, below the
    window), so check_drc then graded 8/8 nets out of range; it is now moved
    into [min + tolerance, max - tolerance];
  * a member that cannot be meandered ("got 0, need 1") used to ship at its
    natural length still listed as matched, and JSON_SUMMARY carried only the
    member list; every member now has `member_status` (status, length,
    target, shortfall, the dru window and whether it sits inside it);
  * the same group routed in two calls used to split into two targets,
    because the first call's members (protected, so never in the second
    call's results) left the group; board copper matching the group's
    patterns now joins it and sets the target.

Rows:
  1. `clamp_target_to_window` and `dru_length_window` on small inputs.
  2. One call, the whole group: the printed target sits inside the window,
     every member has a status, an unmeanderable one says so with its
     shortfall, and a member stated `matched` is inside the window.
  3. Two calls: the second call's group has all 8 members, the first call's
     measured from the board, one target inside the window.

    python3 tests/test_1391_length_window_target.py        (about 1 min)
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from length_matching import clamp_target_to_window              # noqa: E402

FAILS = []
ARGS = ['--length-match-group', '/BIT[0-7]', '--length-match-tolerance', '0.2',
        '--no-bga-zones']


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def route(src, out, nets, *extra):
    r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/route.py', src, out,
                        '--nets', nets] + ARGS + list(extra), cwd=ROOT,
                       capture_output=True, text=True, encoding='utf-8', errors='replace')
    return r.stdout + r.stderr


def targets(log):
    return [float(x) for x in re.findall(r'Length matching group: \d+ nets .*target=([\d.]+)mm', log)]


def main():
    # 1. The clamp.
    t, note = clamp_target_to_window(38.29, (39.0, 41.0, ['r']), 0.2)
    check('1. a target below the window moves up to min + tolerance',
          abs(t - 39.2) < 1e-9 and note, f'{t} {note}')
    t, note = clamp_target_to_window(45.0, (39.0, 41.0, ['r']), 0.2)
    check('1. a target above the window moves down to max - tolerance',
          abs(t - 40.8) < 1e-9, str(t))
    t, note = clamp_target_to_window(40.0, (39.0, 41.0, ['r']), 0.2)
    check('1. a target inside the window is untouched', t == 40.0 and note == '')
    t, note = clamp_target_to_window(10.0, (39.0, 39.2, ['r']), 0.2)
    check('1. a window narrower than two tolerances aims at its middle',
          abs(t - 39.1) < 1e-9, str(t))
    t, note = clamp_target_to_window(10.0, (None, 12.0, ['r']), 0.2)
    check('1. a max-only window only caps', abs(t - 10.0) < 1e-9, str(t))

    tmp = tempfile.mkdtemp(prefix='t1391_')
    try:
        b = os.path.join(tmp, 'b.kicad_pcb')
        subprocess.run([sys.executable, '-X', 'utf8', 'py_router/copy_board.py',
                        os.path.join(ROOT, 'kicad_files', 'interf_u_unrouted_placed.kicad_pcb'),
                        b], cwd=ROOT, capture_output=True)
        open(os.path.join(tmp, 'b.kicad_dru'), 'w', encoding='utf-8').write(
            '(version 1)\n(rule "bit bus length"\n'
            "  (condition \"A.NetName == '/BIT*'\")\n"
            '  (constraint length (min 39mm) (max 41mm)))\n')

        # 2. One call.
        m = os.path.join(tmp, 'm.kicad_pcb')
        mj = os.path.join(tmp, 'm.json')
        log = route(b, m, '/BIT[0-7]', '--json-out', mj)
        tg = targets(log)
        check('2. the printed target sits inside the window', len(tg) == 1 and 39.2 - 1e-6 <= tg[0] <= 40.8 + 1e-6,
              str(tg))
        grp = (json.load(open(mj)).get('length_match_groups') or [{}])[0] if os.path.isfile(mj) else {}
        ms = grp.get('member_status') or {}
        check('2. every member has a status', len(ms) == 8 and set(ms) == set(grp.get('members') or ()),
              str(sorted(ms)))
        bad = [n for n, v in ms.items() if v['status'] == 'matched' and not v['in_window']]
        check('2. a member stated matched is inside the window', not bad, str(bad))
        unm = {n: v for n, v in ms.items() if v['status'] in ('unmeanderable', 'unmatched')}
        check('2. an unmatched member carries its shortfall and the window',
              bool(unm) and all(v.get('shortfall_mm', 0) > 0 and v['window_mm'] == [39.0, 41.0]
                                for v in unm.values()), str(unm)[:200])

        # 3. Two calls, one group.
        s1 = os.path.join(tmp, 's1.kicad_pcb')
        s2 = os.path.join(tmp, 's2.kicad_pcb')
        route(b, s1, '/BIT[0-3]')
        log2 = route(s1, s2, '/BIT[4-7]')
        tg2 = targets(log2)
        check('3. the second call matches all 8 members as one group',
              len(tg2) == 1 and "'/BIT0'" in log2 and "'/BIT7'" in log2
              and re.search(r'Length matching group: 8 nets', log2) is not None, str(tg2))
        check('3. the first call\'s members are measured from the board',
              re.search(r'4 member\(s\) measured from existing board copper.*BIT0', log2) is not None)
        check('3. the one target sits inside the window',
              len(tg2) == 1 and 39.2 - 1e-6 <= tg2[0] <= 40.8 + 1e-6, str(tg2))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())

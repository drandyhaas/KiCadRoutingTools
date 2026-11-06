#!/usr/bin/env python3
"""#1685: a fanout that knows its escapes graze says what can fix it, and
autoplan does not take a grazing rung over a clean one.

The half-pitch warning advised a --via-size the fab floor pins straight back
up, the summary said nothing of why, and autoplan stopped at the first rung
that escaped every ball however it did so.

1. escape_budget against the floors: None when it fits; nothing named that a
   floor would pin back up; a value named when it can take effect.
2. A real run on a 0.5 mm array at the floor via: exit 0, failed 0, the grazes
   and the budget in JSON_SUMMARY, the warning naming no via; dogbone (what it
   names) clears them and has no budget to report.
3. The advice taken literally under a --fab-overrides file that allows it:
   each value it names runs unpinned, with no graze left.
4. autoplan.fan_ladder: a clean rung beats a grazing one; a clean first rung
   runs alone; the earliest of equals; escaping more outranks grazing less;
   a graze the decap step clears does not count; a failed grade counts none.

    python3 tests/test_1685_escape_budget_advice.py
"""
import contextlib
import io
import itertools
import json
import os
import re
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

import autoplan as AP                                              # noqa: E402
import fab_tiers                                                   # noqa: E402
from bga_fanout import escape_budget                               # noqa: E402
from run_utils import check as run_check, evidence                 # noqa: E402

failures = []


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}" + (f' -- {detail}' if detail and not cond else ''))
    if not cond:
        failures.append(name)


def test_budget():
    print("1. escape_budget against the floors")
    ok("a via that fits: nothing to say", escape_budget(0.4, 0.3, 0.15, 0.1, 0.09, 4) is None)
    b = escape_budget(0.25, 0.25, 0.15, 0.1, 0.09, 8)
    ok("the overrun is the need less the half pitch",
       abs(b['overrun_mm'] - 0.015) < 1e-9 and abs(b['need_mm'] - 0.265) < 1e-9, b)
    ok("at the floor via on 0.5 mm only dogbone can take effect",
       b['can_take_effect'] == ['--escape-method dogbone'], b)
    ok("the sizes it cannot reach are named with their floor",
       'a via of 0.220 (fab floor 0.25)' in b['advice']
       and 'a track of 0.070 (fab floor 0.0762)' in b['advice'], b['advice'])
    ok("and no flag names one", '--via-size' not in b['advice']
       and '--track-width' not in b['advice'], b['advice'])

    b = escape_budget(0.325, 0.45, 0.2, 0.1, 0.1, 4)
    ok("a smaller via is advised when it can take effect, drill kept",
       b['can_take_effect'][0] == '--via-size 0.350', b)
    b = escape_budget(0.25, 0.25, 0.15, 0.06, 0.1, 4)
    ok("a narrower track can be what's left", '--via-size' not in str(b['can_take_effect']), b)
    for row in (escape_budget(p / 2, v, d, t, c, n)
                for p, v, d, t, c, n in itertools.product(
                    (0.4, 0.5, 0.65, 0.8), (0.25, 0.3, 0.45, 0.6), (0.15, 0.2),
                    (0.0762, 0.1, 0.127), (0.09, 0.1, 0.127), (2, 4))):
        for a in (row or {}).get('can_take_effect', []):
            if a.startswith('--via-size'):
                val = float(a.split()[1])
                if val < 0.25 - 1e-9:
                    ok(f"advice {a} is above the via floor", False)


def board(work, name):
    balls = []
    n = 0
    nets = []
    for r, c in itertools.product(range(8), range(8)):
        net = ''
        if 2 <= r <= 5 and 2 <= c <= 5:
            n += 1
            nets.append(n)
            net = f' (net {n} "/S{n}")'
        balls.append(f'    (pad "{chr(65 + r)}{c + 1}" smd circle (at {c * 0.5 - 1.75} '
                     f'{r * 0.5 - 1.75}) (size 0.3 0.3) (layers "F.Cu"){net})\n')
    conn = ''.join(f'    (pad "{i}" smd rect (at 0 {i * 0.8}) (size 0.5 0.5) (layers "F.Cu") '
                   f'(net {i} "/S{i}"))\n' for i in nets)
    p = os.path.join(work, name + '.kicad_pcb')
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write(
            '(kicad_pcb (version 20260206) (generator "pcbnew")\n'
            ' (general (thickness 1.6)) (paper "A4")\n'
            ' (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal) '
            '(31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
            ' (setup)\n (net 0 "") '
            + ' '.join(f'(net {i} "/S{i}")' for i in nets) + '\n'
            ' (gr_rect (start 20 20) (end 80 80) (stroke (width 0.1) (type solid)) '
            '(fill none) (layer "Edge.Cuts"))\n'
            ' (footprint "t:BGA64" (layer "F.Cu") (uuid "fu1") (at 50 50)\n'
            '    (property "Reference" "U1" (at 0 0) (layer "F.Fab") (uuid "ru1"))\n'
            + ''.join(balls) + '  )\n'
            ' (footprint "t:CONN" (layer "F.Cu") (uuid "fj1") (at 30 40)\n'
            '    (property "Reference" "J1" (at 0 0) (layer "F.Fab") (uuid "rj1"))\n'
            + conn + '  )\n)\n')
    return p


BASE = ['--component', 'U1', '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu', '--nets', '*']


def fanout(src, dst, *extra):
    r = run_check([sys.executable, '-u', '-X', 'utf8',
                   os.path.join(ROOT, 'py_router', 'bga_fanout.py'), src,
                   '--output', dst, *BASE, *extra], accept=True)
    evidence(dst, 'fanout output')
    m = re.search(r'^JSON_SUMMARY: (\{.*\})\s*$', r.stdout, re.M)
    assert m, 'BROKEN TEST: no JSON_SUMMARY\n' + r.stdout[-1500:]
    return r.stdout, json.loads(m.group(1))


AT_FLOOR = ['--track-width', '0.1', '--clearance', '0.09', '--clearance-ceiling', '0.09',
            '--via-size', '0.25', '--via-drill', '0.15']


def test_real_run(work):
    print("2. a real run at the floor via on 0.5 mm")
    src = board(work, 'a')
    log, s = fanout(src, os.path.join(work, 'a_out.kicad_pcb'), *AT_FLOOR)
    ok("every ball escaped, exit 0, failed 0", s['failed'] == 0 and s['escaped'] == s['requested'] > 0, s)
    ok("the fixture does graze", s['drc_grazes']['via_segment'] > 0, s['drc_grazes'])
    ok("the summary says why and what can fix it",
       s['escape_budget'].get('can_take_effect') == ['--escape-method dogbone'], s['escape_budget'])
    warn = [l for l in log.splitlines() if 'busts the half-pitch' in l]
    ok("the warning advises no value a floor pins back", warn and '--via-size' not in warn[0], warn)

    log, s = fanout(src, os.path.join(work, 'a_dog.kicad_pcb'), *AT_FLOOR,
                    '--escape-method', 'dogbone')
    ok("dogbone leaves no graze", s['drc_grazes']['total'] == 0, s)
    ok("and no budget to report", s['escape_budget'] == {}, s['escape_budget'])


def test_advice_takes_effect(work):
    print("3. the advice taken literally")
    src = board(work, 'b')
    fab = os.path.join(work, 'fab.txt')
    with open(fab, 'w') as fh:
        fh.write('via_diameter = 0.2\nvia_drill = 0.1\ntrack_width = 0.05\nclearance = 0.05\n')
    _, s = fanout(src, os.path.join(work, 'b0.kicad_pcb'), *AT_FLOOR, '--fab-overrides', fab)
    can = s['escape_budget'].get('can_take_effect', [])
    ok("the override opens three sizes", len(can) == 4, can)
    for i, advice in enumerate(can[:3]):
        args = dict(zip(AT_FLOOR[::2], AT_FLOOR[1::2]))
        words = advice.split()
        args.update(zip(words[::2], words[1::2]))
        if '--clearance' in words:
            args['--clearance-ceiling'] = args['--clearance']
        flat = [x for kv in args.items() for x in kv]
        log, r = fanout(src, os.path.join(work, f'b{i + 1}.kicad_pcb'), *flat, '--fab-overrides', fab)
        ok(f"{advice}: runs unpinned", 'pinning up' not in log, log[-400:])
        ok(f"{advice}: no budget left, no graze",
           r['escape_budget'] == {} and r['drc_grazes']['via_segment'] == 0,
           (r['escape_budget'], r['drc_grazes']))


class Scripted:
    """Chain.run scripted: the summary of each rung, by its index."""

    def __init__(self, rows):
        self.rows, self.runs = rows, []

    def run(self, kind, args, tag):
        i = len(self.runs)
        self.runs.append(tag)
        return f'r{i}', self.rows[i], {}


def _ladder(rows):
    ch = Scripted(rows)
    fan = {'ref': 'U1', 'tool': 'bga', 'ladder': [['{IN}', str(i)] for i in range(len(rows))]}
    return AP.fan_ladder(ch, fan), ch.runs


def test_fan_ladder():
    print("4. autoplan.fan_ladder")

    def s(esc, fail, total=None, deferred=None):
        g = {} if total is None else {'total': total}
        if deferred is not None:
            g['deferred_to_decap'] = deferred
        return {'escaped': esc, 'failed': fail, 'drc_grazes': g}
    b, runs = _ladder([s(23, 0, 12), s(23, 0, 12), s(23, 0, 0)])
    ok("a clean rung beats two grazing ones", b['rung'] == 2 and b['grazes'] == 0 and len(runs) == 3, b)
    b, runs = _ladder([s(23, 0, 0), s(23, 0, 12)])
    ok("a clean first rung runs alone", b['rung'] == 0 and len(runs) == 1, (b, runs))
    b, runs = _ladder([s(23, 0, 3), s(23, 0, 3)])
    ok("the earliest of equals", b['rung'] == 0 and len(runs) == 2, (b, runs))
    b, runs = _ladder([s(23, 0, 9), s(20, 3, 0)])
    ok("escaping more outranks grazing less", b['rung'] == 0, b)
    b, runs = _ladder([s(20, 3, 0), s(23, 0, 9)])
    ok("and from a later rung too", b['rung'] == 1, b)
    b, runs = _ladder([s(23, 0, 4, 4), s(23, 0, 0)])
    ok("grazes the decap step clears do not count", b['rung'] == 0 and len(runs) == 1, (b, runs))
    b, runs = _ladder([{'escaped': 23, 'failed': 0, 'drc_grazes': {'error': 'x'}}, s(23, 0, 0)])
    ok("a grade that errored counts none", b['rung'] == 0 and len(runs) == 1, (b, runs))
    b, runs = _ladder([s(20, 3, 2), s(20, 3, 1)])
    ok("among rungs that dropped balls, fewer grazes", b['rung'] == 1, b)


def main():
    work = tempfile.mkdtemp(prefix='krt1685_')
    try:
        test_budget()
        test_real_run(work)
        test_advice_takes_effect(work)
        test_fan_ladder()
    finally:
        shutil.rmtree(work, ignore_errors=True)
    if failures:
        print(f"\nFAILED: {len(failures)}")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("\nAll checks passed.")
    return 0


if __name__ == '__main__':
    sys.exit(main())

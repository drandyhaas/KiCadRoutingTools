#!/usr/bin/env python3
"""Tied ground and supply nets are ordered by name, not by the board file (#1755).

find_power_nets and autoroute sorted grounds and rails by pad count only, so a
tie kept the order the board file lists its nets in: the same netlist poured
AGND or GND, and listed its rails in either order, depending on that order.
Both now sort by `list_nets.power_net_order` -- pad count descending, then a
canonical ground leaf (plain GND / VSS) before AGND, PGND and the rest, then
the name.

  1. the key: on a tie GND and VSS lead (at any hierarchy depth), the rest go
     by name; pad count still decides first;
  2. the issue's boards (gen.py, 3 pads per net): both net orders give the
     same find_power_nets lists and the same autoroute `planes` / `power_nets`
     -- GND poured ahead of AGND, rails by name.

    python3 tests/test_1755_power_net_ties.py
"""
import contextlib
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb                         # noqa: E402
from list_nets import find_power_nets, power_net_order           # noqa: E402
import run_utils                                                 # noqa: E402

failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def gen(path, a, b):
    """The issue's gen.py: three 2-pin parts, each with one pad on `a` and one
    on `b`, so the two nets tie at 3 pads and only their file order differs."""
    def fp(i):
        pad = lambda n, dx, net: (f'(pad "{n}" smd rect (at {dx} 0) (size 1 1) '
                                  f'(layers "F.Cu" "F.Mask" "F.Paste") (net "{net}"))')
        return (f'(footprint "R" (layer "F.Cu") (at {10 + 5 * i} 10) '
                f'(property "Reference" "R{i}" (at 0 0) (layer "F.SilkS")) '
                f'{pad(1, -1, a)} {pad(2, 1, b)})')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('(kicad_pcb (version 20260206) (generator "pcbnew") (generator_version "10.0") '
                '(general (thickness 1.6)) '
                '(layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user)) '
                + ' '.join(fp(i) for i in range(3)) +
                ' (gr_rect (start 0 0) (end 40 20) (layer "Edge.Cuts") '
                '(stroke (width 0.1) (type default)) (fill no)))\n')
    return run_utils.evidence(path, 'generated board')


def analysis(board, work):
    r = subprocess.run([sys.executable, '-B', '-X', 'utf8', 'py_router/autoroute.py', board,
                        work, '--dry-run'], cwd=ROOT, capture_output=True, text=True,
                       encoding='utf-8', errors='replace')
    out = r.stdout + r.stderr
    if 'Traceback' in out:
        raise AssertionError(f'BROKEN TEST: autoroute crashed:\n{out[-1500:]}')
    m = re.search(r'^AUTOROUTE_ANALYSIS: (.*)$', out, re.M)
    if not m:
        raise AssertionError(f'BROKEN TEST: no AUTOROUTE_ANALYSIS line:\n{out[-1500:]}')
    a = json.loads(m.group(1))
    return a['planes'], a['power_nets']


def key_checks():
    print('1. power_net_order')
    tied = [('PGND', 3), ('AGND', 3), ('/sub/GND', 3), ('VSS', 3), ('DGND', 3)]
    got = [n for n, _ in sorted(tied, key=power_net_order)]
    ok('canonical grounds first, then by name', got == ['/sub/GND', 'VSS', 'AGND', 'DGND', 'PGND'],
       repr(got))
    ok('pad count still decides first',
       [n for n, _ in sorted([('GND', 2), ('AGND', 5)], key=power_net_order)] == ['AGND', 'GND'])
    ok('rails by name on a tie',
       [n for n, _ in sorted([('+5V', 3), ('+3V3', 3)], key=power_net_order)] == ['+3V3', '+5V'])


def board_checks(tmp):
    print('2. both net orders give the same answer')
    expect = {('GND', 'AGND'): ([['GND', 'B.Cu']], ['GND', 'AGND']),
              ('+5V', '+3V3'): ([], ['+3V3', '+5V'])}
    for (a, b), want in expect.items():
        got, fpn = {}, {}
        for x, y in ((a, b), (b, a)):
            p = gen(os.path.join(tmp, f'{x}_{y}.kicad_pcb'), x, y)
            with contextlib.redirect_stdout(io.StringIO()):
                fpn[(x, y)] = find_power_nets(parse_kicad_pcb(p))
            got[(x, y)] = analysis(p, os.path.join(tmp, f'w_{x}_{y}'))
        ok(f'{a}/{b}: find_power_nets order-independent', fpn[(a, b)] == fpn[(b, a)],
           repr(fpn))
        ok(f'{a}/{b}: autoroute order-independent', got[(a, b)] == got[(b, a)], repr(got))
        ok(f'{a}/{b}: autoroute answer', tuple(got[(a, b)]) == want, repr(got[(a, b)]))


def main():
    tmp = tempfile.mkdtemp(prefix='t1755_')
    try:
        key_checks()
        board_checks(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        print(f'FAILED: {len(failures)}: {failures}')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())

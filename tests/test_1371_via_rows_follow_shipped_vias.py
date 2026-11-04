#!/usr/bin/env python3
"""#1371: the design_rules via rows describe the vias that SHIP.

`via_diameter` rows were recorded per placement ATTEMPT: a last-resort via the
weld then rejected stayed a row, one placement was counted several times, and
the rung-1 via search -- which ships small vias -- recorded nothing at all.
route.py now replaces them with a census of the vias this run laid against the
diameter each net asked for (`routing_common.new_via_shortfall`,
`fab_tiers.replace_via_rows`).

Unit half: the census and the ledger replacement, on stand-in vias.
Integration half: route a small board whose run ships vias below the requested
size by more than one path, and require the SHIPPED design_rules (the
JSON_SHIPPED_RULES line, which is what the writeback and --json-out carry) to
list exactly the vias on the written board, per net and diameter.

    python3 tests/test_1371_via_rows_follow_shipped_vias.py
"""
import collections
import json
import os
import shutil
import sys
import tempfile
from types import SimpleNamespace

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
from run_utils import check, evidence, tool        # noqa: E402

import fab_tiers                                   # noqa: E402
from routing_common import new_via_shortfall, via_identity   # noqa: E402

BOARD = os.path.join(ROOT, 'tests', 'fixtures', 'run25', 'esp_prog_placed.kicad_pcb')
PY = [sys.executable, '-X', 'utf8']


def v(net, x, y, size):
    return SimpleNamespace(net_id=net, x=x, y=y, size=size)


def unit_half():
    names = {1: 'A', 2: 'B'}
    inp = {via_identity(v(1, 1.0, 1.0, 0.3), 'A')}
    vias = [v(1, 1.0, 1.0, 0.3),      # on the input board: not this run's
            v(1, 2.0, 2.0, 0.3),      # new, under 0.45
            v(1, 3.0, 3.0, 0.3),
            v(2, 4.0, 4.0, 0.45),     # new, at the request
            v(2, 5.0, 5.0, 0.25)]
    got = new_via_shortfall(vias, names.get, inp, lambda n: 0.45)
    assert set(got) == {'A', 'B'}, got
    assert got['A']['count'] == 2 and got['A']['sizes'] == {0.3: 2}, got
    assert got['B']['min_mm'] == 0.25 and got['B']['count'] == 1, got
    assert new_via_shortfall(vias, names.get, inp, lambda n: None) == {}

    fab_tiers.reset_ledger()
    fab_tiers.note_narrowing(1, 'via_diameter', 0.45, 0.25, 'last-resort via-in-pad')
    fab_tiers.note_narrowing(1, 'via_diameter', 0.45, 0.325, 'via-in-pad clamp')
    fab_tiers.note_narrowing(1, 'track_width', 0.2, 0.127, 'terminal neck')
    fab_tiers.replace_via_rows(
        [(1, 'A', 0.45, {0.3: 2}), (2, 'B', 0.45, {0.25: 1})],
        'via copper shipped under size')
    rows = fab_tiers.escalation_summary()['narrowed']
    assert [(r['net_name'], r['delivered'], r['count']) for r in rows
            if r['kind'] == 'via_diameter'] == [('A', 0.3, 2), ('B', 0.25, 1)], rows
    assert any(r['kind'] == 'track_width' for r in rows), 'other kinds are kept'
    fab_tiers.reset_ledger()


def integration_half():
    evidence(BOARD, 'esp_prog board')
    from kicad_parser import parse_kicad_pcb
    tmp = tempfile.mkdtemp(prefix='t1371_')
    try:
        inb = os.path.join(tmp, 'in.kicad_pcb')
        out = os.path.join(tmp, 'o.kicad_pcb')
        check(PY + [tool('copy_board.py'), BOARD, inb], accept=True)
        r = check(PY + [tool('route.py'), inb, out, '--clearance', '0.25'],
                  accept=True)
        text = r.stdout + (r.stderr or '')
        assert 'Boxed endpoint unblocked by rung-0.25/0.15 via search' in text, \
            'the rung-1 search did not fire (broken test)'
        line = next(l for l in text.splitlines()
                    if l.startswith('JSON_SHIPPED_RULES: '))
        rows = json.loads(line.split(': ', 1)[1])['design_rules']['narrowed']
        want_req = 0.45
        assert all(abs(r_['requested'] - want_req) < 1e-9
                   for r_ in rows if r_['kind'] == 'via_diameter')
        got = collections.Counter()
        for r_ in rows:
            if r_['kind'] == 'via_diameter':
                got[(r_['net_name'], r_['delivered'])] += r_['count']

        pin, pout = parse_kicad_pcb(inb), parse_kicad_pcb(out)
        have = {(pin.nets[x.net_id].name, round(x.x, 3), round(x.y, 3),
                 round(x.size, 3)) for x in pin.vias}
        ship = collections.Counter()
        for x in pout.vias:
            nm = pout.nets[x.net_id].name
            if (nm, round(x.x, 3), round(x.y, 3), round(x.size, 3)) in have:
                continue
            if x.size < want_req - 1e-3:
                ship[(nm, round(x.size, 4))] += 1
        assert ship, 'no sub-size via shipped (broken test)'
        assert got == ship, (
            f'design_rules via rows {dict(got)} differ from the vias that '
            f'ship {dict(ship)}')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == '__main__':
    unit_half()
    print('unit half ok')
    integration_half()
    print('PASS')

#!/usr/bin/env python3
"""A track end BESIDE its own net's fill is not joined to it, whatever the
pour mesh or where the mesh grid falls (#2069).

Fixture tests/fixtures/sense_branch/sense_star.kicad_pcb (+ its .kicad_pro):
VDD trace C1.1 (3, 5) -> U1.1 (27, 5), 0.2 mm, here drawn as two segments
meeting at (15, 5), loads U2.1 (15, 8) and U3.1 (20, 8). Each variant adds a
VDD zone on F.Cu x 1..29, y y0..12, and a copper-pour keep-out 0.25 mm
either side of the trace, so the fill stays 0.15 mm from the trace's copper.
Loads: source C1.1, U2.1 and U3.1 1 A each, U1.1 0 A -- a sense pin, whose
only copper path is the trace, which then carries no current, so its true
drop is 0.

Before #2069 a track end joined the pour mesh through the nearest mesh cell
within (half width + one mesh cell) of it, whether or not its copper met any
fill: with the zone's lower edge at y0 = 1.06 the vertex (15, 5) was joined to
the fill 0.15 mm away at the default mesh, U1.1 read 1.7 mV and the trace
58 mA; at y0 = 1.00 it read 0.

Invariants:
(a) every grid phase of the fill model's 0.08 mm raster (y0 = 1.00 .. 1.24)
    at meshes 0.5, 0.25, 0.15 and 0.1: U1.1 drops under 0.05 mV and the
    trace carries under 1 mA;
(b) boards a (y0 1.00) and b (y0 1.06) read the same U1.1 drop and the same
    trace current at the default mesh, through check_power_drop.py too;
(c) control: without the keep-out the trace lies in the fill (a real
    contact), and U1.1 reads above 1 mV with the trace carrying load
    current at every mesh -- a genuine contact still joins.

Run:
    python3 tests/test_2069_power_drop_fill_gap.py
"""
import json
import os
import re
import shutil
import sys
import tempfile
import traceback

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import power_drop  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from run_utils import check, evidence, tool  # noqa: E402

RUN_ALL_TIMEOUT = 600

SRC = os.path.join(TESTS_DIR, 'fixtures', 'sense_branch', 'sense_star')
BOARD, PRO = SRC + '.kicad_pcb', SRC + '.kicad_pro'
SEG = ('  (segment (start 3 5) (end 27 5) (width 0.2) (layer "F.Cu") (net 1) '
       '(uuid "11111111-0000-0000-0000-000000000001"))\n')
SPLIT = ('  (segment (start 3 5) (end 15 5) (width 0.2) (layer "F.Cu") (net 1) '
         '(uuid "11111111-0000-0000-0000-000000000001"))\n'
         '  (segment (start 15 5) (end 27 5) (width 0.2) (layer "F.Cu") (net 1) '
         '(uuid "11111111-0000-0000-0000-000000000002"))\n')
KEEPOUT = ('  (zone (net 0) (net_name "") (layer "F.Cu") (uuid "aaaaaaaa-0000-0000-0000-000000000002") '
           '(name "sense_ko") (hatch edge 0.5)\n'
           '    (connect_pads (clearance 0)) (min_thickness 0.25) (filled_areas_thickness no)\n'
           '    (keepout (tracks allowed) (vias allowed) (pads allowed) (copperpour not_allowed) '
           '(footprints allowed))\n'
           '    (fill (thermal_gap 0.5) (thermal_bridge_width 0.5))\n'
           '    (polygon (pts (xy 3.4 4.75) (xy 26.2 4.75) (xy 26.2 4.2) (xy 27.8 4.2) (xy 27.8 5.8) '
           '(xy 26.2 5.8) (xy 26.2 5.25) (xy 3.4 5.25))))\n')
LOADS = {'U2.1': 1.0, 'U3.1': 1.0, 'U1.1': 0.0}
MESHES = (0.5, 0.25, 0.15, 0.1)
PHASES = [round(1.0 + 0.02 * i, 2) for i in range(13)]          # 1.00 .. 1.24

_TMP = []


def _tmpdir():
    d = tempfile.mkdtemp(prefix='t2069_')
    _TMP.append(d)
    return d


def _zone(y0):
    return ('  (zone (net 1) (net_name "VDD") (layer "F.Cu") (uuid "aaaaaaaa-0000-0000-0000-000000000001") '
            '(hatch edge 0.5)\n'
            '    (connect_pads yes (clearance 0.2)) (min_thickness 0.2) (filled_areas_thickness no)\n'
            '    (fill yes (thermal_gap 0.5) (thermal_bridge_width 0.5))\n'
            f'    (polygon (pts (xy 1 {y0}) (xy 29 {y0}) (xy 29 12) (xy 1 12))))\n')


def _board(y0, keepout=True):
    """The variant written with the fixture's .kicad_pro in a fresh dir."""
    with open(BOARD, encoding='utf-8') as f:
        text = f.read()
    assert text.count(SEG) == 1 and text.count('  (gr_rect') == 1, 'fixture lines moved'
    text = text.replace(SEG, SPLIT).replace(
        '  (gr_rect', _zone(y0) + (KEEPOUT if keepout else '') + '  (gr_rect', 1)
    d = _tmpdir()
    path = os.path.join(d, f'b_{y0}{"" if keepout else "_open"}.kicad_pcb')
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text)
    shutil.copy(PRO, path[:-len('.kicad_pcb')] + '.kicad_pro')
    return evidence(path, 'the variant board')


def _solve(path, mesh):
    pcb = parse_kicad_pcb(path)
    nid = next(i for i, n in pcb.nets.items() if n.name == 'VDD')
    res = power_drop.solve_net(pcb, nid, ['C1.1'], LOADS, pour_mesh_mm=mesh)
    assert res['pours'], f'the pour was not solved: {res.get("not_modelled")}'
    drop = next(r for r in res['loads'] if r['pad'] == 'U1.1')
    assert 'drop_mv' in drop, f'U1.1 unreached: {drop}'
    # the trace's first half, (3, 5)-(15, 5): its pieces, should a contact cut it
    amps = [abs(s['current_a']) for s in res['segments']
            if abs(s['start'][1] - 5) < 1e-6 and abs(s['end'][1] - 5) < 1e-6
            and min(s['start'][0], s['end'][0]) >= 3 - 1e-6
            and max(s['start'][0], s['end'][0]) <= 15 + 1e-6]
    assert amps, 'the trace (3, 5)-(15, 5) is not in the solve'
    return drop['drop_mv'], max(amps)


def test_no_join_across_the_gap_at_any_phase_or_mesh():
    bad = []
    for y0 in PHASES:
        path = _board(y0)
        for mesh in MESHES:
            mv, amps = _solve(path, mesh)
            if abs(mv) >= 0.05 or amps >= 0.001:
                bad.append(f'y0 {y0}, mesh {mesh}: U1.1 {mv} mV, trace {amps * 1e3:.1f} mA')
    assert not bad, 'a track end joined the fill 0.15 mm away:\n  ' + '\n  '.join(bad)


def test_boards_a_and_b_agree_through_the_cli():
    loads = os.path.join(_tmpdir(), 'loads.json')
    with open(loads, 'w', encoding='utf-8') as f:
        json.dump({'nets': {'VDD': {'sources': ['C1.1'], 'loads': LOADS}}}, f)
    got = {}
    for name, y0 in (('a', 1.0), ('b', 1.06)):
        r = check([sys.executable, '-X', 'utf8', tool('check_power_drop.py'), _board(y0),
                   '--loads', loads], accept=True, timeout=300)
        out = r.stdout
        m = re.search(r'^\s+U1\.1: 0 A, drop (-?[\d.]+) mV', out, re.M)
        t = re.search(r'^\s+track F\.Cu 0\.2 mm \(3\.00,5\.00\)-\(15\.00,5\.00\): (-?[\d.]+) A', out, re.M)
        assert m and t, f'board {name}: no U1.1 / trace line in the report:\n{out[-2000:]}'
        got[name] = (float(m.group(1)), float(t.group(1)))
    (mv_a, a_a), (mv_b, a_b) = got['a'], got['b']
    assert abs(mv_a - mv_b) < 0.05 and abs(mv_b) < 0.05, got
    assert a_a < 0.001 and a_b < 0.001, got


def test_open_control_still_joins():
    path = _board(1.0, keepout=False)
    for mesh in MESHES:
        mv, amps = _solve(path, mesh)
        assert mv > 1.0 and amps > 0.01, (
            f'mesh {mesh}: the trace lies in the fill, yet U1.1 reads {mv} mV and '
            f'the trace {amps * 1e3:.1f} mA')


TESTS = [
    test_no_join_across_the_gap_at_any_phase_or_mesh,
    test_boards_a_and_b_agree_through_the_cli,
    test_open_control_still_joins,
]


def main():
    evidence(BOARD, 'the sense_star fixture')
    evidence(PRO, "the sense_star fixture's .kicad_pro")
    failed = []
    try:
        for t in TESTS:
            try:
                t()
                print(f'  PASS  {t.__name__}')
            except Exception:  # noqa: BLE001
                failed.append(t.__name__)
                print(f'  FAIL  {t.__name__}')
                traceback.print_exc()
    finally:
        for d in _TMP:
            shutil.rmtree(d, ignore_errors=True)
    print(f'{len(TESTS) - len(failed)}/{len(TESTS)} passed')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())

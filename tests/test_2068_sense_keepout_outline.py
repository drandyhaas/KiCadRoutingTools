#!/usr/bin/env python3
"""A pour keep-out round a sense trace protects it where the zone's OUTLINE
crosses the trace inside the keep-out (#2068).

Fixture tests/fixtures/sense_branch/sense_star.kicad_pcb (+ its .kicad_pro):
VDD sense trace C1.1 (3, 5) -> U1.1 (27, 5), 0.2 mm, group `sense:C1.1`.
Each variant adds a VDD zone on F.Cu (y 1..12) and, on most, a copper-pour
keep-out 0.35 mm either side of the trace (boxed round U1.1):

- crossing: zone x 8..22 -- its outline crosses the trace at x = 8 and 22,
  both inside the keep-out, so the area the pour may fill stays 0.25 mm off
  the trace's copper. The branch must resolve protected, and route.py must
  lay no new copper end on the trace off its join pad. (Before #2068 the
  zone test counted every ring edge within the copper's reach, whether or
  not it bounded fill, so the outline edge inside the keep-out refused the
  branch and the run then tapped the trace off the join pad.)
- wide: zone x 1..29, same keep-out -- the trace never meets the outline.
  Protected before and after (control).
- open: crossing without the keep-out -- the pour floods over the trace, so
  the branch is still refused.
- short: crossing with the keep-out starting 0.3 mm inside the zone
  (x 8.3) -- the fill between the outline and the keep-out reaches the
  trace, so the branch is still refused.
- The plane step's warning (`route_planes._sense_branches_poured_over`)
  reads the same test: no warning for the crossing pour, one for open.
- Premise leg (needs kicad-cli; SKIP without it): KiCad's own refill of the
  crossing board keeps every VDD fill polygon at least 0.2 mm from the
  trace's copper between the pads.

Run:
    python3 tests/test_2068_sense_keepout_outline.py
"""
import os
import re
import shutil
import subprocess
import sys
import tempfile
import traceback

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import parse_kicad_pcb  # noqa: E402
from run_utils import check, evidence, tool  # noqa: E402

RUN_ALL_TIMEOUT = 600

SRC = os.path.join(TESTS_DIR, 'fixtures', 'sense_branch', 'sense_star')
BOARD, PRO = SRC + '.kicad_pcb', SRC + '.kicad_pro'
MEMBER = '11111111-0000-0000-0000-000000000001'
PROTECTED = 'sense:C1.1: protected 1 track(s), 0 via(s), sense pad(s) U1.1; new copper joins only at C1.1'
POURS_OVER = 'zone on F.Cu pours over its F.Cu track'

_TMP = []


def _tmpdir():
    d = tempfile.mkdtemp(prefix='t2068_')
    _TMP.append(d)
    return d


def _keepout(x0):
    return ('  (zone (net 0) (net_name "") (layer "F.Cu") (uuid "aaaaaaaa-0000-0000-0000-000000000002") '
            '(name "sense_ko") (hatch edge 0.5)\n'
            '    (connect_pads (clearance 0)) (min_thickness 0.25) (filled_areas_thickness no)\n'
            '    (keepout (tracks allowed) (vias allowed) (pads allowed) (copperpour not_allowed) '
            '(footprints allowed))\n'
            '    (fill (thermal_gap 0.5) (thermal_bridge_width 0.5))\n'
            f'    (polygon (pts (xy {x0} 4.65) (xy 26.2 4.65) (xy 26.2 4.2) (xy 27.8 4.2) (xy 27.8 5.8) '
            f'(xy 26.2 5.8) (xy 26.2 5.35) (xy {x0} 5.35))))\n')


def _zone(x0, x1):
    return ('  (zone (net 1) (net_name "VDD") (layer "F.Cu") (uuid "aaaaaaaa-0000-0000-0000-000000000001") '
            '(hatch edge 0.5)\n'
            '    (connect_pads yes (clearance 0.2)) (min_thickness 0.2) (filled_areas_thickness no)\n'
            '    (fill yes (thermal_gap 0.5) (thermal_bridge_width 0.5))\n'
            f'    (polygon (pts (xy {x0} 1) (xy {x1} 1) (xy {x1} 12) (xy {x0} 12))))\n')


VARIANTS = {
    'crossing': (8, 22, 3.4),
    'wide': (1, 29, 3.4),
    'open': (8, 22, None),
    'short': (8, 22, 8.3),
}


def _variant(name):
    """The variant written with the fixture's .kicad_pro in a fresh dir."""
    x0, x1, ko = VARIANTS[name]
    with open(BOARD, encoding='utf-8') as f:
        text = f.read()
    assert text.count('  (gr_rect') == 1, 'fixture outline line moved'
    extra = _zone(x0, x1) + (_keepout(ko) if ko is not None else '')
    d = _tmpdir()
    path = os.path.join(d, f'{name}.kicad_pcb')
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text.replace('  (gr_rect', extra + '  (gr_rect', 1))
    shutil.copy(PRO, os.path.join(d, f'{name}.kicad_pro'))
    return evidence(path, f'the {name} variant')


def _branch(path):
    p = parse_kicad_pcb(path)
    assert len(p.sense_branches) == 1, p.sense_branches
    return p, p.sense_branches[0]


def test_crossing_outline_inside_keepout_is_protected():
    _p, b = _branch(_variant('crossing'))
    assert b.protected, f'refused: {b.reason}'


def test_wide_control_is_protected():
    _p, b = _branch(_variant('wide'))
    assert b.protected, f'refused: {b.reason}'


def test_open_pour_still_refuses():
    _p, b = _branch(_variant('open'))
    assert not b.protected and POURS_OVER in (b.reason or ''), b.reason


def test_fill_between_outline_and_keepout_still_refuses():
    _p, b = _branch(_variant('short'))
    assert not b.protected and POURS_OVER in (b.reason or ''), b.reason
    # the tap is in the strip the keep-out leaves open, x 8 .. 8.3
    m = re.search(r'at \(([\d.]+), ([\d.]+)\)', b.reason)
    assert m and 7.85 <= float(m.group(1)) <= 8.45, b.reason


def test_plane_step_warning_follows():
    """create_plane's warning reads the same zone test: silent for the
    crossing pour the keep-out cuts round the branch, loud for the open one."""
    import route_planes
    for name, want in (('crossing', False), ('open', True)):
        x0, x1, _ko = VARIANTS[name]
        # the warning is asked about a NEW pour: drop the board's own zone so
        # the branch resolves protected first, then offer the zone as new
        path = _variant(name)
        with open(path, encoding='utf-8') as f:
            text = f.read()
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write(text.replace(_zone(x0, x1), ''))
        p, b = _branch(path)
        assert b.protected, b.reason
        zd = [{'net_id': b.net_id, 'net_name': 'VDD', 'layer': 'F.Cu',
               'polygon_points': [(x0, 1), (x1, 1), (x1, 12), (x0, 12)]}]
        got = route_planes._sense_branches_poured_over(p, zd)
        assert bool(got) == want, (name, got)


def test_route_lays_no_copper_on_the_trace_off_the_join():
    board = _variant('crossing')
    out = os.path.join(_tmpdir(), 'out.kicad_pcb')
    r = check([sys.executable, '-X', 'utf8', tool('route.py'), board, out, '--nets', 'VDD',
               '--layers', 'F.Cu', 'B.Cu', '--track-width', '0.3', '--clearance', '0.2'],
              accept=True, timeout=600)
    log = r.stdout + r.stderr
    evidence(out, 'route.py output')
    said = [ln.strip() for ln in log.splitlines() if ln.strip().startswith('sense:C1.1:')]
    assert PROTECTED in log, f'route.py did not protect the branch: {said or log[-3000:]}'
    p = parse_kicad_pcb(out)
    new = [s for s in p.segments if s.uuid != MEMBER and not getattr(s, 'graphic', False)]
    ends = sorted({(round(x, 2), round(y, 2)) for s in new
                   for x, y in ((s.start_x, s.start_y), (s.end_x, s.end_y))
                   if abs(y - 5) < 0.15 and 3.5 < x < 26.5})
    assert not ends, f'new copper ends on the sense trace off C1.1 and U1.1: {ends}'
    _p, b = _branch(out)
    assert b.protected, f'the written board refuses the branch: {b.reason}'


def test_kicad_fill_stays_off_the_trace():
    """Premise: KiCad itself pours nothing within 0.2 mm of the trace's copper
    on the crossing board. Needs kicad-cli."""
    from kicad_oracle import find_kicad_cli
    kc = find_kicad_cli(warn=False)
    if not kc:
        print('  SKIP  kicad-cli not found: the premise leg is not run')
        return
    from shapely.geometry import LineString, Polygon
    board = _variant('crossing')
    rpt = os.path.join(os.path.dirname(board), 'drc.rpt')
    subprocess.run([kc, 'pcb', 'drc', '--refill-zones', '--save-board', '-o', rpt, board],
                   capture_output=True, text=True, timeout=300)
    with open(board, encoding='utf-8') as f:
        t = f.read()
    fills = [Polygon([(float(a), float(b)) for a, b in re.findall(r'\(xy ([-\d.]+) ([-\d.]+)\)', m.group(1))])
             for m in re.finditer(r'\(filled_polygon\s+\(layer "F\.Cu"\)\s+\(pts(.*?)\)\s*\)', t, re.S)]
    assert fills, 'kicad-cli refilled nothing: the premise cannot be checked'
    track = LineString([(4, 5), (26, 5)]).buffer(0.1, cap_style=2)
    gap = min(f.distance(track) for f in fills)
    assert gap >= 0.2, f'KiCad fill comes {gap:.3f} mm from the trace'


TESTS = [
    test_crossing_outline_inside_keepout_is_protected,
    test_wide_control_is_protected,
    test_open_pour_still_refuses,
    test_fill_between_outline_and_keepout_still_refuses,
    test_plane_step_warning_follows,
    test_route_lays_no_copper_on_the_trace_off_the_join,
    test_kicad_fill_stays_off_the_trace,
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

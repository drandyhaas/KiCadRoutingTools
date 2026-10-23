#!/usr/bin/env python3
"""Tap points sampled off existing copper sit on the routing grid (#1291).

get_all_segment_tap_points samples an existing segment for the cells a route
may launch from or land on, each with an OWNER float point the converter
welds to. It counted its steps as the Euclidean length over the grid step,
truncated. A 45-degree segment takes sqrt(2) as many steps as it has cells,
and an axial one whose span is an exact multiple of the step loses a step to
float subtraction (80.6 - 79.3 = 1.2999999999999972, int(12.99..) = 12), so
the samples were spaced off grid: a route tapping one welded in through a
35-71 um T stub that KiCad's DRC reports as track_dangling. An axial or
45-degree segment between grid points is now sampled once per cell it
crosses, on that cell's grid point. Other copper (off grid, any angle) has no
grid points to land on and keeps the length-based count exactly.

Checks:
  A. axial and diagonal segments between grid points, both directions,
     negative coordinates, at 0.1 / 0.05 / 0.025 mm steps: one sample per
     cell along the line, each exactly at its grid point (no off-line
     corner cell, whose owner sat 71 um from its grid point)
  B. off-grid and any-angle copper samples exactly as before; ends within
     1 nm of the grid count as on grid; point segments, empty input, layer
     dedup and via owner points unchanged
  C. the diff-pair caller (_terminal_island_taps) gets on-grid diagonal taps
  D. a 5-pad board whose net already carries a diagonal track routes in two
     terminal orders that left a 57 um stub before the fix: every pad
     connected, the route taps the track mid-body, every such tap is on
     grid, check_weird finds no dangling copper, and the copper repeats.
     KiCad's own DRC (track_dangling, unconnected items) when kicad-cli is
     installed; skipped, and said, without it

    python3 tests/test_1291_tap_grid.py
"""
import contextlib
import io
import json
import math
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from run_utils import evidence, tool                   # noqa: E402
from check_weird import check_weird                     # noqa: E402
from kicad_parser import Segment, Via, parse_kicad_pcb  # noqa: E402
from routing_config import GridCoord                    # noqa: E402
from single_ended_routing import get_all_segment_tap_points  # noqa: E402

FAILS = []
EPS = 1e-9


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def quiet(fn, *a, **kw):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **kw)


def on_grid(points, step):
    return all(abs(x - gx * step) < EPS and abs(y - gy * step) < EPS
               for gx, gy, _l, x, y in points)


def line_cells(ga, gb):
    """The grid cells of an axial or 45-degree line from ga to gb."""
    n = max(abs(gb[0] - ga[0]), abs(gb[1] - ga[1]))
    sx = (gb[0] > ga[0]) - (gb[0] < ga[0])
    sy = (gb[1] > ga[1]) - (gb[1] < ga[1])
    return {(ga[0] + i * sx, ga[1] + i * sy) for i in range(n + 1)}


def t_grid_segments():
    print('A. axial and 45-degree segments between grid points')
    cases = (((79.3, 22.4), (80.6, 22.4)),     # the float-subtraction span
             ((3, 5), (9, 5)), ((3, 3), (9, 9)), ((9, 3), (3, 9)),
             ((-3, -3), (-3, 3)), ((-2.5, 1.5), (0.5, -1.5)))
    for step in (0.1, 0.05, 0.025):
        coord = GridCoord(step)
        bad = []
        for a, b in cases:
            for s, e in ((a, b), (b, a)):
                pts = get_all_segment_tap_points(
                    [Segment(*s, *e, 0.2, 'F.Cu', 1)], coord, ['F.Cu', 'B.Cu'])
                want = line_cells(coord.to_grid(*s), coord.to_grid(*e))
                got = [(p[0], p[1]) for p in pts]
                if (sorted(got) != sorted(want) or not on_grid(pts, step)
                        or any(p[2] != 0 for p in pts) or pts != sorted(pts)):
                    bad.append((s, e))
        check(f'step {step}: one exact grid sample per cell on every line',
              not bad, f'wrong on {bad}' if bad else '')


def length_samples(a, b, coord):
    """{cell: owner} of the length-based count, the sampling every segment
    had before #1291 (first sample to reach a cell owns it)."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    n = max(1, int(math.hypot(dx, dy) / coord.grid_step))
    out = {}
    for i in range(n + 1):
        t = i / n
        x, y = a[0] + t * dx, a[1] + t * dy
        out.setdefault(coord.to_grid(x, y), (x, y))
    return out


def t_off_grid_and_bookkeeping():
    print('B. off-grid copper, point segments, dedup and vias')
    coord = GridCoord(0.1)
    for a, b in (((1.023, 2.017), (1.027, 2.021)),
                 ((1.023, 2.017), (3.139, 3.841)),
                 ((0.0, 0.0), (1.0, 0.3)),
                 ((1.03, 1.07), (2.53, 2.57)),       # 45 degrees, off grid
                 ((0.03, 0.5), (1.33, 0.5))):        # axial, off grid
        pts = get_all_segment_tap_points([Segment(*a, *b, 0.2, 'F.Cu', 1)],
                                         coord, ['F.Cu'])
        got = {p[:2]: p[3:] for p in pts}
        check(f'{a}->{b}: cells and owners exactly as before',
              got == length_samples(a, b, coord))
    # Ends within 1 nm of grid points count as on grid (KiCad's resolution).
    pts = get_all_segment_tap_points([Segment(3.0000004, 3, 3.6, 3.6000003, 0.2, 'F.Cu', 1)],
                                     coord, ['F.Cu'])
    check('a 45-degree segment whose ends are within 1 nm of the grid is on grid',
          len(pts) == 7 and all(abs(x - gx * 0.1) < 1e-6 and abs(y - gy * 0.1) < 1e-6
                                for gx, gy, _l, x, y in pts))
    check('empty input gives no taps',
          get_all_segment_tap_points([], coord, ['F.Cu']) == [])
    for end in ((1.023, 2.017), (1.0231, 2.0171)):
        check(f'a point segment ending {end} is one tap at its start',
              get_all_segment_tap_points([Segment(1.023, 2.017, *end, 0.2, 'F.Cu', 1)],
                                         coord, ['F.Cu']) == [(10, 20, 0, 1.023, 2.017)])
    seg = Segment(3, 3, 3.7, 3, 0.2, 'B.Cu', 1)
    via = Via(3.223, 3.017, 0.6, 0.3, ['F.Cu', 'B.Cu'], 1)
    pts = get_all_segment_tap_points([seg, seg], coord, ['F.Cu', 'B.Cu'], [via])
    check('a duplicated segment and a via dedupe to 8 + 1 cells', len(pts) == 9,
          f'{len(pts)}')
    check('the via owns its cell on the layer no segment covers',
          (32, 30, 0, via.x, via.y) in pts)
    check('the segment keeps its own on-grid owner where both cover a cell',
          any(p[:3] == (32, 30, 1) and abs(p[3] - 3.2) < EPS and abs(p[4] - 3.0) < EPS
              for p in pts))


def t_diff_pair_caller():
    print('C. the diff-pair island tap set')
    from diff_pair_routing import _terminal_island_taps
    coord = GridCoord(0.1)
    diag = Segment(3, 3, 9, 9, 0.2, 'F.Cu', 1)
    unrelated = Segment(12, 12, 13, 13, 0.2, 'F.Cu', 1)
    pts = _terminal_island_taps((3, 3), [diag, unrelated], [], coord, ['F.Cu'])
    check('the island is the diagonal alone, one tap per cell', len(pts) == 61,
          f'{len(pts)}')
    check('every diagonal tap is on its grid point',
          on_grid(pts, 0.1) and all(p[0] == p[1] for p in pts))


# ----------------------------------------------------------------- board
PADS = [(3, 3), (9, 9), (7.3, 5.5), (5.2, 7.9), (10.5, 4)]
TRACK = (3, 3, 9, 9)
# Footprint orders (indices into PADS) that each routed a 57 um F.Cu stub
# at (6.57, 6.63) before the fix; the identity order routed clean.
ORDERS = ((4, 3, 2, 1, 0), (1, 2, 3, 4, 0))
_N = [0]


def _uid():
    _N[0] += 1
    return '00000000-0000-4000-8000-%012d' % _N[0]


def board_text(order):
    out = ['(kicad_pcb\n\t(version 20260206)\n\t(generator "test_1291")\n'
           '\t(generator_version "10.0")\n'
           '\t(general (thickness 1.6) (legacy_teardrops no))\n\t(paper "A4")\n'
           '\t(layers\n\t\t(0 "F.Cu" signal)\n\t\t(2 "B.Cu" signal)\n'
           '\t\t(1 "F.Mask" user)\n\t\t(3 "B.Mask" user)\n'
           '\t\t(25 "Edge.Cuts" user)\n\t)\n'
           '\t(setup (pad_to_mask_clearance 0))\n'
           '\t(net 0 "")\n\t(net 1 "/T")\n']
    for i in order:
        x, y = PADS[i]
        out.append(
            f'\t(footprint "t:Pad" (layer "F.Cu") (uuid "{_uid()}") (at {x} {y})\n'
            f'\t\t(property "Reference" "TP{i + 1}" (at 0 -1 0) (layer "F.Fab") '
            f'(uuid "{_uid()}") (effects (font (size 0.5 0.5) (thickness 0.1))))\n'
            f'\t\t(pad "1" smd rect (at 0 0) (size 0.4 0.4) (layers "F.Cu" "F.Mask") '
            f'(net "/T") (uuid "{_uid()}"))\n\t)\n')
    out.append(
        f'\t(gr_rect (start 0 0) (end 20 15) (stroke (width 0.1) (type default)) '
        f'(fill no) (layer "Edge.Cuts") (uuid "{_uid()}"))\n'
        f'\t(segment (start {TRACK[0]} {TRACK[1]}) (end {TRACK[2]} {TRACK[3]}) '
        f'(width 0.2) (layer "F.Cu") (net "/T") (uuid "{_uid()}"))\n)\n')
    return ''.join(out)


def _route(board, out):
    r = subprocess.run([sys.executable, '-X', 'utf8', tool('route.py'), board,
                        '--output', out, '--nets', '*'],
                       cwd=ROOT, capture_output=True, text=True, timeout=900)
    if r.returncode != 0:
        print(r.stdout[-3000:], r.stderr[-3000:])
    return r.returncode


def _copper(path):
    pcb = quiet(parse_kicad_pcb, path)
    return (sorted((round(s.start_x, 4), round(s.start_y, 4), round(s.end_x, 4),
                    round(s.end_y, 4), s.width, s.layer) for s in pcb.segments),
            sorted((round(v.x, 4), round(v.y, 4), v.size) for v in pcb.vias))


def _interior_taps(path):
    """New-copper endpoints lying on the pre-existing track, short of its ends."""
    pcb = quiet(parse_kicad_pcb, path)
    (x0, y0, x1, y1) = TRACK
    taps = []
    for s in pcb.segments:
        if (s.start_x, s.start_y, s.end_x, s.end_y) == TRACK:
            continue
        for x, y in ((s.start_x, s.start_y), (s.end_x, s.end_y)):
            cross = (x - x0) * (y1 - y0) - (y - y0) * (x1 - x0)
            t = ((x - x0) * (x1 - x0) + (y - y0) * (y1 - y0)) / 72.0
            if abs(cross) < 1e-6 and 1e-6 < t < 1 - 1e-6:
                taps.append((x, y))
    return taps


def _kicad_drc(cli, path, report):
    subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--severity-all',
                    '-o', report, path], capture_output=True, text=True, timeout=300)
    with open(evidence(report, 'kicad-cli DRC report'), encoding='utf-8') as f:
        return json.load(f)


def t_routed_board():
    print('D. the diagonal-track board, routed')
    from kicad_oracle import find_kicad_cli
    cli = find_kicad_cli(warn=False)
    with tempfile.TemporaryDirectory(prefix='t1291_') as d:
        for order in ORDERS:
            tag = ''.join(map(str, order))
            board = os.path.join(d, f'b{tag}.kicad_pcb')
            with open(board, 'w', encoding='utf-8', newline='\n') as f:
                f.write(board_text(order))
            outs = []
            for run in (1, 2) if order == ORDERS[0] else (1,):
                out = os.path.join(d, f'b{tag}_r{run}.kicad_pcb')
                rc = _route(evidence(board, 'staged board'), out)
                check(f'order {tag} run {run}: route.py exits 0', rc == 0)
                if rc != 0:
                    return
                outs.append(evidence(out, 'routed board'))
            out = outs[0]
            c = subprocess.run([sys.executable, '-X', 'utf8', tool('check_connected.py'), out],
                               cwd=ROOT, capture_output=True, text=True, timeout=300)
            check(f'order {tag}: every pad connected',
                  c.returncode == 0 and 'ALL NETS FULLY CONNECTED' in c.stdout)
            taps = _interior_taps(out)
            check(f'order {tag}: the route taps the track mid-body (the path under test)',
                  bool(taps), f'{taps}')
            check(f'order {tag}: every mid-body tap is on the 0.1 mm grid',
                  all(abs(v * 10 - round(v * 10)) < 1e-6 for p in taps for v in p),
                  f'{taps}')
            # tolerance 0: the stub is 57 um, under check_weird's 0.1 default.
            found, _ = check_weird(quiet(parse_kicad_pcb, out), tolerance=0)
            dang = [f"{f['category']} at ({f['x']:.4f}, {f['y']:.4f})" for f in found
                    if f['category'] in ('kicad-dangling', 'dangling-end')]
            check(f'order {tag}: no dangling copper (check_weird)', not dang,
                  '; '.join(dang[:3]))
            if len(outs) == 2:
                check(f'order {tag}: a second run lays identical copper',
                      _copper(outs[0]) == _copper(outs[1]))
            if cli:
                rep = _kicad_drc(cli, out, os.path.join(d, f'b{tag}_drc.json'))
                td = [v for v in rep.get('violations', []) if v['type'] == 'track_dangling']
                check(f'order {tag}: KiCad DRC reports no track_dangling', not td,
                      f'{[v["items"][0]["pos"] for v in td]}')
                check(f'order {tag}: KiCad DRC reports no unconnected items',
                      not rep.get('unconnected_items'))
        if not cli:
            print('  SKIP: no kicad-cli found; KiCad\'s own track_dangling grade is unchecked here')


if __name__ == '__main__':
    t_grid_segments()
    t_off_grid_and_bookkeeping()
    t_diff_pair_caller()
    t_routed_board()
    print()
    if FAILS:
        print(f'FAILED: {len(FAILS)} check(s)')
        for f in FAILS:
            print('  - ' + f)
        sys.exit(1)
    print('ALL PASS')

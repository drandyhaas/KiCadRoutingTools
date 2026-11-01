#!/usr/bin/env python3
"""#1566: a dog-bone escape never crosses foreign copper under its own stub,
and a fanout's `drc_grazes` names a crossing.

The under-pad engine exempts a ball's HOME from the occupancy grid so its
route can leave its own keep-out ring: the pad disk, the gap-via disk and,
for a dog-bone, the cells along the pad-to-via stub -- on every layer.
`_carve_foreign` re-blocks the foreign copper inside that exemption, but it
looked for it only near the anchor POINTS (ball, elbow, site). A lane-walked
stub (#652) runs a pitch or more between two anchors, so foreign copper
crossing the middle of it -- on an inner layer the stub does not even use --
stayed exempt, and the ball's inner run was laid straight across it. One
fanout run wrote two pairs of its own escapes crossing that way, exited 0,
and counted the two crossings in `drc_grazes.total` under no named key.

The fixture boxes one ball in so that the defect decides its escape:

  * foreign B.Cu copper through every gap site within one lane of the ball,
    so its gap via has to walk two lanes out (a 1.6 mm lane at 0.8 mm pitch);
  * a closed foreign B.Cu box around that site, one wall crossing the middle
    of the lane.

The only way out of the box on B.Cu is across the wall. The escape must keep
its clearance from every planted track (it used to run through the wall
where the stub crosses it). Then two crossing tracks of different nets on a
board make `drc_grazes.segment_crossing` 1, and the named keys add up to
`total`.

    python3 tests/test_1566_dogbone_stub_carve.py
"""
import contextlib
import io
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

from run_utils import evidence                                      # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'ulx3s.kicad_pcb')
REF = 'U1'
PITCH = 0.8
CLEARANCE = 0.1
TRACK = 0.12
TOL = 0.005

failures = []


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}" + (f' -- {detail}' if detail and not cond else ''))
    if not cond:
        failures.append(name)


def _quiet(fn):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn()


def plants():
    """The foreign B.Cu tracks, relative to the ball: a short track through
    every gap site within one lane (vias cannot sit on them), and a box around
    the gap two lanes west, whose east wall crosses the lane's middle."""
    h = PITCH / 2.0
    out = []
    for gx in (-3 * h, -h, h, 3 * h):
        for gy in (-3 * h, -h, h, 3 * h):
            if abs(gx + 3 * h) > 1e-9:       # the box's east wall covers x = -3h
                out.append((gx - 0.15, gy, gx + 0.15, gy))
    x_e, x_w, y_s, y_n = -3 * h, -6.5 * h, -3.75 * h, 3.75 * h
    out += [(x_e, y_s, x_e, y_n), (x_w, y_s, x_w, y_n),
            (x_w, y_s, x_e, y_s), (x_w, y_n, x_e, y_n)]
    return out


def escape_leg():
    from collections import Counter
    from kicad_parser import parse_kicad_pcb, Segment
    from bga_fanout import generate_bga_fanout
    from geometry_utils import segment_to_segment_distance

    pcb = _quiet(lambda: parse_kicad_pcb(BOARD))
    fp = pcb.footprints[REF]
    n = Counter(p.net_id for p in fp.pads)
    # a single-ball signal net five columns in from the west edge, mid-array
    xs = sorted({round(p.global_x, 4) for p in fp.pads})
    ys = sorted({round(p.global_y, 4) for p in fp.pads})
    ball = min((p for p in fp.pads if p.net_id and n[p.net_id] == 1
                and len(pcb.nets[p.net_id].pads) >= 2),
               key=lambda p: (p.global_x - xs[8]) ** 2 + (p.global_y - ys[10]) ** 2)
    foreign = next(p.net_id for p in fp.pads
                   if p.net_id and p.net_id != ball.net_id and n[p.net_id] == 1)
    planted = [Segment(start_x=ball.global_x + a, start_y=ball.global_y + b,
                       end_x=ball.global_x + c, end_y=ball.global_y + d,
                       width=TRACK, layer='B.Cu', net_id=foreign)
               for a, b, c, d in plants()]
    pcb.segments.extend(planted)
    tracks, vias, _vr, failed = _quiet(lambda: generate_bga_fanout(
        fp, pcb, layers=['F.Cu', 'B.Cu'], escape_method='dogbone',
        net_filter=[ball.net_name], track_width=TRACK, clearance=CLEARANCE,
        via_size=0.35, via_drill=0.2, plane_drop='off'))
    print(f"1. ball {ball.pad_number} ({ball.net_name}) boxed in by {len(planted)} "
          f"foreign B.Cu tracks: {len(tracks)} track(s), {len(vias)} via(s), "
          f"failed {failed}")
    mine = [t for t in tracks if t['net_id'] == ball.net_id]
    ok('the ball escaped, or is reported unescaped',
       bool(mine) or ball.net_name in failed, f'tracks {len(mine)}, failed {failed}')
    worst = None
    for t in mine:
        if t['layer'] != 'B.Cu':
            continue
        for s in planted:
            gap = segment_to_segment_distance(
                t['start'][0], t['start'][1], t['end'][0], t['end'][1],
                s.start_x, s.start_y, s.end_x, s.end_y) - (t['width'] + s.width) / 2.0
            if worst is None or gap < worst[0]:
                worst = (gap, t['start'], t['end'])
    ok('its escape keeps clearance from the box it was laid in (it ran through '
       'the wall where its stub crosses it)',
       worst is None or worst[0] >= CLEARANCE - TOL,
       f'{worst[0]:.3f} mm edge to edge from a foreign track, segment '
       f'{worst[1]} - {worst[2]}' if worst else '')
    for v in vias:
        if v['net_id'] != ball.net_id:
            continue
        gap = min(segment_to_segment_distance(v['x'], v['y'], v['x'], v['y'],
                                              s.start_x, s.start_y, s.end_x, s.end_y)
                  - v['size'] / 2.0 - s.width / 2.0 for s in planted)
        ok(f"its via at ({v['x']:.3f}, {v['y']:.3f}) keeps clearance from the box",
           gap >= CLEARANCE - TOL, f'{gap:.3f} mm')


def grazes_leg(d):
    from kicad_parser import parse_kicad_pcb
    from check_drc import copper_keys, fanout_drc_grazes
    src = os.path.join(ROOT, 'kicad_files', 'interf_u_unrouted_placed.kicad_pcb')
    evidence(src, 'fixture board')
    dst = os.path.join(d, 'crossing.kicad_pcb')
    with open(src, encoding='utf-8') as fh:
        text = fh.read().rstrip()
    assert text.endswith(')'), 'BROKEN TEST: the fixture does not end in )'
    # two tracks of nets 1 and 2 crossing on F.Cu, far from the part's pads
    segs = ('\t(segment (start 90 40) (end 100 40) (width 0.2) (layer "F.Cu") '
            '(net 1) (uuid "cross-1"))\n'
            '\t(segment (start 95 35) (end 95 45) (width 0.2) (layer "F.Cu") '
            '(net 2) (uuid "cross-2"))\n')
    with open(dst, 'w', encoding='utf-8') as fh:
        fh.write(text[:-1] + segs + ')\n')
    evidence(dst, 'board with the crossing')
    base = copper_keys(_quiet(lambda: parse_kicad_pcb(src)))
    g = _quiet(lambda: fanout_drc_grazes(dst, 0.25, base))
    print(f"2. drc_grazes of two crossing tracks the step laid: {g}")
    ok('a crossing is counted', g.get('total', 0) >= 1, g)
    ok('...under its own key', g.get('segment_crossing') == 1, g)
    named = [k for k in g if k not in ('scope', 'total', 'deferred_to_decap',
                                       'deferred_parts')]
    ok('the named keys add up to total', sum(g[k] for k in named) == g.get('total'),
       f'{named} vs total {g.get("total")}')


def main():
    print('=' * 60)
    print('#1566: dog-bone stubs carve the foreign copper under them; '
          'drc_grazes names a crossing')
    print('=' * 60)
    evidence(BOARD, 'fixture board')
    escape_leg()
    with tempfile.TemporaryDirectory(prefix='t1566_') as d:
        grazes_leg(d)
    if failures:
        print(f'FAIL: {len(failures)} check(s)')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())

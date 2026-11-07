#!/usr/bin/env python3
"""#1684: a differential pair's two escapes do not cross, and when they would
the step says so.

A pair on the outermost row of an array, escaping across it, took the channel
on its partner's side (the channel on its own side does not exist there), so P
ran through the bend of N. The step printed "Validated: No collisions", since
that check leaves a pair's own two legs out, and counted segment_crossing 0,
since check_drc's crossing test leaves out a touch at a segment's end, which is
exactly how a track meets another's bend.

1. Every adjacent or diagonal pair of balls on arrays of several sizes, in
   every escape orientation and channel mode: the two legs keep their copper
   apart. (Geometry, ~1 s.)
2. pair_self_collisions sees legs that cross and passes legs at pair spacing.
3. check_drc: a track through another net's bend is ONE segment-crossing, a
   short, and a track ending on another's middle is one; a net's own bend is
   not, and neither is a gap (control).
4. A real run: the pair on the outermost row, through the CLI. Its escapes do
   not cross, the Validated line is true, drc_grazes has no segment row.
   Control: the same call without the pair.
5. A pair whose legs still touch (forced, through the builder): reported
   failed, both nets, its copper off the output and the Validated line says so.
   Legs merely closer than their spacing: the line says so and they ship, for
   drc_grazes to count.

    python3 tests/test_1684_pair_escape_crossing.py
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

import bga_fanout as BF                                            # noqa: E402
from bga_fanout.escape import find_diff_pair_escape                # noqa: E402
from bga_fanout.grid import calculate_channels                     # noqa: E402
from bga_fanout.types import BGAGrid, DiffPairPads                 # noqa: E402
from geometry_utils import segment_to_segment_distance             # noqa: E402
from kicad_parser import Pad, parse_kicad_pcb                      # noqa: E402
from run_utils import check as run_check, evidence                 # noqa: E402

PITCH = 0.5
TRACK, GAP = 0.1, 0.1
PAIR_P, PAIR_N = '/DRV_P', '/DRV_N'
ARGS = ['--component', 'U1', '--layers', 'F.Cu', 'B.Cu', '--nets', '*',
        '--diff-pair-gap', str(GAP), '--track-width', str(TRACK),
        '--clearance', '0.09', '--via-size', '0.25', '--via-drill', '0.15']

failures = []


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}" + (f' -- {detail}' if detail and not cond else ''))
    if not cond:
        failures.append(name)


def _pad(num, x, y, net):
    return Pad(component_ref='U1', pad_number=num, global_x=x, global_y=y,
               local_x=x, local_y=y, size_x=0.3, size_y=0.3, shape='circle',
               layers=['F.Cu'], net_id=net, net_name=f'N{net}')


def _gap(tracks):
    p = [t for t in tracks if t['net_id'] == 1]
    n = [t for t in tracks if t['net_id'] == 2]
    return min(segment_to_segment_distance(*a['start'], *a['end'], *b['start'], *b['end'])
               for a in p for b in n)


def test_every_pair_geometry():
    print("1. every pair geometry keeps its legs apart")
    bad, total = [], 0
    for nr, nc in [(2, 2), (2, 5), (5, 2), (3, 3), (6, 6)]:
        rows = [10 + i * PITCH for i in range(nr)]
        cols = [20 + i * PITCH for i in range(nc)]
        grid = BGAGrid(pitch_x=PITCH, pitch_y=PITCH, rows=rows, cols=cols,
                       center_x=sum(cols) / nc, center_y=sum(rows) / nr,
                       min_x=cols[0], max_x=cols[-1], min_y=rows[0], max_y=rows[-1])
        chans = calculate_channels(grid)
        for (px, py), (qx, qy) in itertools.permutations(itertools.product(cols, rows), 2):
            if max(abs(px - qx), abs(py - qy)) > PITCH * 1.01:
                continue
            pair = DiffPairPads(base_name='X', p_pad=_pad('1', px, py, 1),
                                n_pad=_pad('2', qx, qy, 2))
            for orient, adj_h, adj_v in itertools.product(
                    ('auto', 'horizontal', 'vertical'), (False, True), (False, True)):
                ch, d = find_diff_pair_escape(px, py, qx, qy, grid, chans, orient)
                routes = BF.build_diff_pair_routes(
                    'X', pair, {'X': (ch, d)}, grid, chans, ['F.Cu', 'B.Cu'], 0.5,
                    (TRACK + GAP) / 2, adj_h, adj_v, {})
                with contextlib.redirect_stdout(io.StringIO()):
                    tracks, _, _ = BF.generate_tracks_from_routes(routes, TRACK, 'F.Cu')
                total += 1
                if _gap(tracks) < TRACK + GAP - 1e-6:
                    bad.append(((nr, nc), (px, py), (qx, qy), d, adj_h, adj_v))
    ok(f"no pair of {total} comes closer than its own spacing", not bad,
       f"{len(bad)} did, first {bad[:3]}")


def test_validator():
    print("2. pair_self_collisions")
    from bga_fanout.tracks import pair_self_collisions, detect_collisions

    def t(a, b, net, pid='X'):
        return {'start': a, 'end': b, 'net_id': net, 'layer': 'F.Cu', 'pair_id': pid,
                'width': TRACK}
    crossed = [t((0, 0), (1, 1), 1), t((0, 1), (1, 0), 2)]
    bend = [t((0, 0), (1, 1), 1), t((1, 0), (0.5, 0.5), 2), t((0.5, 0.5), (0, 0.5), 2)]
    parallel = [t((0, 0), (5, 0), 1), t((0, 0.2), (5, 0.2), 2)]
    other = [t((0, 0), (1, 1), 1, 'X'), t((0, 1), (1, 0), 2, 'Y')]
    layer = [t((0, 0), (1, 1), 1), dict(t((0, 1), (1, 0), 2), layer='B.Cu')]
    sp = TRACK + GAP
    ok("legs that cross are seen", len(pair_self_collisions(crossed, sp)) == 1)
    ok("a leg through its partner's bend is seen", len(pair_self_collisions(bend, sp)) == 2)
    ok("legs exactly at the pair's spacing are not", pair_self_collisions(parallel, sp) == [])
    ok("two different pairs are detect_collisions' to see",
       pair_self_collisions(other, sp) == [] and detect_collisions(other, [], sp)[0] == 1)
    ok("legs on two layers are not", pair_self_collisions(layer, sp) == [])


def _grade_board(work, name, segs):
    p = os.path.join(work, name + '.kicad_pcb')
    body = ''.join(
        f' (segment (start {a} {b}) (end {c} {d}) (width 0.2) (layer "F.Cu") '
        f'(net "{net}") (uuid "s{i}"))\n' for i, (a, b, c, d, net) in enumerate(segs))
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20260206) (generator "pcbnew")\n'
                 ' (general (thickness 1.6)) (paper "A4")\n'
                 ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
                 ' (setup)\n'
                 ' (gr_rect (start 0 0) (end 20 20) (stroke (width 0.1) (type solid)) '
                 '(fill none) (layer "Edge.Cuts"))\n' + body + ')\n')
    js = p + '.json'
    argv = [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'check_drc.py'),
            p, '--clearance', '0.1', '--clearance-margin', '0', '--no-size-checks',
            '--json', js]
    import subprocess
    subprocess.run(argv, capture_output=True, text=True, cwd=ROOT)
    evidence(js)
    with open(js, encoding='utf-8') as fh:
        return [i for i in json.load(fh)['items'] if i['type'] == 'segment-crossing']


def test_grade(work):
    print("3. check_drc counts a touch of two nets as a crossing")
    bend = [(5, 5, 15, 15, '/A'), (15, 5, 10, 10, '/B'), (10, 10, 5, 10, '/B')]
    x = _grade_board(work, 'bend', bend)
    ok("a track through another net's bend: one crossing, a short",
       len(x) == 1 and x[0]['short'] is True, str(x))
    x = _grade_board(work, 'tee', [(5, 10, 15, 10, '/A'), (10, 5, 10, 10, '/B')])
    ok("a track ending on another's middle: one crossing", len(x) == 1, str(x))
    x = _grade_board(work, 'own', [(5, 5, 10, 10, '/A'), (10, 10, 15, 10, '/A')])
    ok("a net's own bend (control): none", x == [], str(x))
    x = _grade_board(work, 'gap', [(5, 10, 15, 10, '/A'), (10, 5, 10, 9, '/B')])
    ok("two nets with a gap (control): none", x == [], str(x))


def board(work, name):
    """A 6x6 0.5 mm array; the pair on the outermost row of its first column,
    P on that row and N one row in, each net also on a pad away from the array."""
    balls = []
    for r, c in itertools.product(range(6), range(6)):
        net = ''
        if (r, c) == (5, 0):
            net = f' (net 1 "{PAIR_P}")'
        elif (r, c) == (4, 0):
            net = f' (net 2 "{PAIR_N}")'
        balls.append(f'    (pad "{chr(65 + r)}{c + 1}" smd circle (at {c * PITCH - 1.25} '
                     f'{r * PITCH - 1.25}) (size 0.3 0.3) (layers "F.Cu"){net})\n')
    conn = ''.join(
        f'    (pad "{i + 1}" smd rect (at 0 {i}) (size 0.6 0.6) (layers "F.Cu") '
        f'(net {i + 1} "{n}"))\n' for i, n in enumerate((PAIR_P, PAIR_N)))
    p = os.path.join(work, name + '.kicad_pcb')
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write(
            '(kicad_pcb (version 20260206) (generator "pcbnew")\n'
            ' (general (thickness 1.6)) (paper "A4")\n'
            ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
            ' (setup)\n'
            f' (net 0 "") (net 1 "{PAIR_P}") (net 2 "{PAIR_N}")\n'
            ' (gr_rect (start 30 30) (end 70 70) (stroke (width 0.1) (type solid)) '
            '(fill none) (layer "Edge.Cuts"))\n'
            ' (footprint "t:BGA36" (layer "F.Cu") (uuid "fu1") (at 50 50)\n'
            '    (property "Reference" "U1" (at 0 0) (layer "F.Fab") (uuid "ru1"))\n'
            + ''.join(balls) + '  )\n'
            ' (footprint "t:CONN2" (layer "F.Cu") (uuid "fj1") (at 35 50)\n'
            '    (property "Reference" "J1" (at 0 0) (layer "F.Fab") (uuid "rj1"))\n'
            + conn + '  )\n)\n')
    return p


def fanout(src, dst, extra=()):
    r = run_check([sys.executable, '-u', '-X', 'utf8',
                   os.path.join(ROOT, 'py_router', 'bga_fanout.py'), src,
                   '--output', dst, *ARGS, *extra], accept=True)
    evidence(dst, 'fanout output')
    m = re.search(r'^JSON_SUMMARY: (\{.*\})\s*$', r.stdout, re.M)
    assert m, 'BROKEN TEST: no JSON_SUMMARY\n' + r.stdout[-1500:]
    return r.stdout, json.loads(m.group(1))


def test_real_run(work):
    print("4. the pair on the outermost row, through the CLI")
    src = board(work, 'array')
    log, s = fanout(src, os.path.join(work, 'out.kicad_pcb'),
                    ['--diff-pairs', '/DRV_*'])
    ok("it ran as a pair, in adjacent channels",
       'Found 1 differential pairs' in log and 'adjacent-channel' in log,
       log[-800:])
    ok("the escapes do not touch", s['drc_grazes']['segment_segment'] == 0
       and s['drc_grazes']['segment_crossing'] == 0, s['drc_grazes'])
    ok("both escaped", s['escaped'] == 2 and s['failed'] == 0, s)
    ok("the Validated line says what was checked",
       'Validated: no track-to-track collisions' in log, log[-800:])
    pcb = parse_kicad_pcb(os.path.join(work, 'out.kicad_pcb'))
    ids = {n.name: i for i, n in pcb.nets.items()}
    p = [g for g in pcb.segments if g.net_id == ids[PAIR_P]]
    n = [g for g in pcb.segments if g.net_id == ids[PAIR_N]]
    ok("each net has copper", bool(p) and bool(n))
    ok("P stays on its own side of N: below it at the array's edge",
       max(g.end_y for g in p if g.end_x < 48.75) > max(g.end_y for g in n if g.end_x < 48.75),
       [(g.start_x, g.start_y, g.end_x, g.end_y) for g in p + n])

    _, c = fanout(src, os.path.join(work, 'ctl.kicad_pcb'))
    ok("control: the same call without the pair grades no segment row either",
       c['drc_grazes']['segment_segment'] == 0 and c['drc_grazes']['segment_crossing'] == 0,
       c['drc_grazes'])


def _forced_run(work, name, bend_legs):
    """A channel fanout of the pair with its built routes bent by `bend_legs`.
    At the 2-layer standard floors, which the engine does not narrow."""
    pcb = parse_kicad_pcb(board(work, name))
    orig = BF.build_diff_pair_routes

    def bent(*a, **k):
        routes = orig(*a, **k)
        bend_legs(routes)
        return routes
    BF.build_diff_pair_routes = bent
    out = io.StringIO()
    try:
        with contextlib.redirect_stdout(out):
            tracks, _, _, failed = BF.generate_bga_fanout(
                pcb.footprints['U1'], pcb, net_filter=['*'], diff_pair_patterns=['/DRV_*'],
                layers=['F.Cu', 'B.Cu'], track_width=0.127, clearance=0.127,
                diff_pair_gap=0.127, via_size=0.25, via_drill=0.15, escape_method='channel')
    finally:
        BF.build_diff_pair_routes = orig
    return tracks, failed, out.getvalue()


def test_forced_collision(work):
    print("5. legs that touch fail, disclosed; legs that are only close ship")

    def crossed(routes):
        routes[0].stub_end, routes[1].stub_end = routes[1].stub_end, routes[0].stub_end
        routes[0].exit_pos, routes[1].exit_pos = routes[1].exit_pos, routes[0].exit_pos

    def close(routes):
        sx, sy = routes[1].stub_end
        ex, ey = routes[1].exit_pos
        routes[0].stub_end, routes[0].exit_pos = (sx, sy + 0.2), (ex, ey + 0.2)

    line = 'Validated: the two legs of 1 diff pair(s) run closer than their spacing'
    tracks, failed, log = _forced_run(work, 'forced', crossed)
    ok("touching: Validated does not claim none",
       line in log and 'no track-to-track collisions' not in log, log[-600:])
    ok("touching: both nets are reported failed", set(failed) >= {PAIR_P, PAIR_N}, failed)
    ok("touching: neither ships copper", not [t for t in tracks if t.get('pair_id')], tracks[:2])

    tracks, failed, log = _forced_run(work, 'close', close)
    ok("close: Validated does not claim none", line in log, log[-600:])
    ok("close: not failed", not set(failed) & {PAIR_P, PAIR_N}, failed)
    ok("close: both legs ship",
       len({t['net_id'] for t in tracks if t.get('pair_id')}) == 2, tracks[:2])


def main():
    work = tempfile.mkdtemp(prefix='krt1684_')
    try:
        test_every_pair_geometry()
        test_validator()
        test_grade(work)
        test_real_run(work)
        test_forced_collision(work)
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

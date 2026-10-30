#!/usr/bin/env python3
"""The channel escape keeps inner balls off the top layer when the gap
between the balls cannot hold a track at clearance (#1518).

  python3 tests/test_1518_channel_top_layer_gap.py

An inner ball's escape on the top layer -- the balls' own layer -- runs down
a channel between two rows of balls. The engine laid it there whatever the
gap: a 0.5 mm-pitch array of 0.35 mm balls leaves 0.15 mm of copper gap, so
a 0.1 mm track at clearance 0.09 sat 0.025 mm from the balls on both sides,
was reported only in `drc_grazes`, and reached the routed board. `auto` kept
that result because the channel pass dropped no ball, so the under-pad
retry never ran.

Pinned on glasgow_revC's U30 (0.8 mm pitch, 0.35 mm balls: a 0.45 mm gap),
one channel pass, no retries:

1. the geometry: `ball_row_gap` reads 0.45 mm;
2. at track 0.15 / clearance 0.16 (needs 0.47 mm) no top-layer track the
   channel engine lays comes within clearance of one of the array's own
   balls, and the run says why the inner balls left the top layer;
3. at clearance 0.15 the gap holds the track exactly (0.45 = 0.15 + 2 x 0.15)
   and the rule stays out of the way: inner balls keep the top layer.
"""
import contextlib
import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

BOARD = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
REF = 'U30'
LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
TRACK = 0.15
NOTE = 'Inner balls stay off'


def _pcb():
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(BOARD)


def _run(clearance):
    import bga_fanout
    pcb = _pcb()
    fp = pcb.footprints[REF]
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        res = bga_fanout._generate_bga_fanout_core(
            fp, pcb, layers=LAYERS, escape_method='channel', _single_pass=True,
            track_width=TRACK, clearance=clearance, via_size=0.3, via_drill=0.15)
    return fp, res[0], buf.getvalue()


def _own_ball_grazes(fp, tracks, clearance):
    """Top-layer tracks within `clearance` of a ball of another net."""
    from check_drc import segment_pad_gap
    top = LAYERS[0]
    out = []
    for t in tracks:
        if t['layer'] != top:
            continue
        for p in fp.pads:
            if p.net_id == t['net_id'] or top not in (p.layers or []):
                continue
            gap = segment_pad_gap(*t['start'], *t['end'], p) - TRACK / 2
            if gap < clearance - 1e-6:
                out.append((p.pad_number, round(gap, 4)))
    return out


def _inner_on_top(fp, tracks):
    """Top-layer tracks that start on a ball NOT in the array's outer ring
    (an edge ball's escape exits outward and crosses no ball gap)."""
    from bga_fanout.grid import analyze_bga_grid, is_edge_pad
    grid = analyze_bga_grid(fp)
    balls = {(round(p.global_x, 4), round(p.global_y, 4)) for p in fp.pads}
    n = 0
    for t in tracks:
        if t['layer'] != LAYERS[0]:
            continue
        for x, y in (t['start'], t['end']):
            if (round(x, 4), round(y, 4)) in balls and not is_edge_pad(x, y, grid)[0]:
                n += 1
                break
    return n


def check_geometry(fails):
    from bga_fanout.grid import analyze_bga_grid, ball_row_gap
    fp = _pcb().footprints[REF]
    gap = ball_row_gap(fp, analyze_bga_grid(fp))
    if abs(gap - 0.45) > 1e-6:
        fails.append(f"fixture: {REF}'s ball gap reads {gap:.4f}mm, expected 0.45")


def check_too_narrow(fails):
    cl = 0.16
    fp, tracks, log = _run(cl)
    bad = _own_ball_grazes(fp, tracks, cl)
    if bad:
        fails.append(f"clearance {cl}: {len(bad)} top-layer track/ball pair(s) "
                     f"below clearance against the array's own balls, e.g. {bad[:4]}")
    if NOTE not in log:
        fails.append(f"clearance {cl}: the run did not say the inner balls left "
                     f"{LAYERS[0]}")


def check_exact_fit(fails):
    cl = 0.15
    fp, tracks, log = _run(cl)
    if NOTE in log:
        fails.append(f"clearance {cl}: the gap holds the track exactly, yet the "
                     f"inner balls were sent off {LAYERS[0]}")
    if not _inner_on_top(fp, tracks):
        fails.append(f"clearance {cl}: no inner ball escaped on {LAYERS[0]} "
                     f"(fixture: the case no longer exercises the top layer)")
    bad = _own_ball_grazes(fp, tracks, cl)
    if bad:
        fails.append(f"clearance {cl}: own-ball grazes at an exact fit: {bad[:4]}")


def main():
    if not os.path.isfile(BOARD):
        print(f"FAIL: missing board {BOARD}")
        return 1
    fails = []
    for check in (check_geometry, check_too_narrow, check_exact_fit):
        check(fails)
    if fails:
        for f in fails:
            print(f"FAIL: {f}")
        return 1
    print("PASS: inner balls leave the top layer exactly when its ball gap "
          "cannot hold the track at clearance")
    return 0


if __name__ == '__main__':
    sys.exit(main())

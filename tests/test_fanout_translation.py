#!/usr/bin/env python3
"""The BGA fanout is translation-invariant: the same board moved in memory by
a whole number of routing cells fans out to the same copper, moved back.

Measured before the fix (the #622 pose gate, 2026-09-08): the same array
shifted 1 mm in x fanned out to 447 tracks instead of 502. The under-pad
engine orders its balls by `depth`, a real-valued minimum of four coordinate
differences; balls on one ring are equally deep in exact arithmetic, and
the last bit of the difference then decided their order -- a different
order routed one ball into a corner, the rip-swap rescue re-assigned five
nets. The key is rounded to a nanometre now, so equal depths stay equal and
the stable sort keeps the footprint's own pad order.

Covered here, wx-free: a real 96-ball BGA (orangecrab U4) fanned out in
place and after three lattice translations, the routing compared moved back
-- the same vias, every segment endpoint within one occupancy cell of the
other run's copper on its net (the last cell of a jog is the routers'
own rounding and may differ; measured 0.025 mm); the under-pad engine's
INITIAL OCCUPANCY STAMP byte-identical between the frames (the ordering
fix above is not the only last-bit decision: the disk and capsule
rasterisers decided boundary cells at an exact integer radius by the
last bit of the centre coordinate -- 244 and 288 cells on this board --
and one net's jog then took the other side; the centres are quantised to
a billionth of a cell now); plus the non-vacuity check that this board
HAS ring ties whose raw depths differ in the last bit under the
translation (the case the rounding exists for). If that last check ever
fails the test has gone vacuous on this board and needs another fixture.
"""
import contextlib
import io
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))

from kicad_parser import parse_kicad_pcb  # noqa: E402
from bga_fanout import generate_bga_fanout  # noqa: E402
from bga_fanout import underpad as _underpad  # noqa: E402

# Every occupancy grid the engine builds, so a run's INITIAL stamp (at the
# first per-ball progress message) can be compared between frames.
_GRIDS = []
_occ_init = _underpad._Occ.__init__


def _occ_init_hook(self, *a, **kw):
    _occ_init(self, *a, **kw)
    _GRIDS.append(self)


_underpad._Occ.__init__ = _occ_init_hook

BOARD = os.path.join(HERE, '..', 'kicad_files', 'orangecrab_ext_pll.kicad_pcb')
REF = 'U4'
SHIFTS = ((1.0, 0.0), (9.85, -11.75), (55.55, 55.55))


def _ok(results, name, cond, detail=''):
    results.append((name, bool(cond)))
    print(f'  {"PASS" if cond else "FAIL"}  {name}' + (f'  ({detail})' if detail else ''))


def _board(dx, dy):
    with contextlib.redirect_stdout(io.StringIO()):
        p = parse_kicad_pcb(BOARD)
    for f in p.footprints.values():
        f.x += dx
        f.y += dy
        for q in f.pads:
            q.global_x += dx
            q.global_y += dy
            if q.hole_x is not None and q.hole_y is not None:
                q.hole_x += dx
                q.hole_y += dy
    for s in p.segments:
        s.start_x += dx
        s.start_y += dy
        s.end_x += dx
        s.end_y += dy
    for v in p.vias:
        v.x += dx
        v.y += dy
    for z in p.zones or []:
        z.polygon = [(x + dx, y + dy) for (x, y) in z.polygon]
    b = p.board_info.board_bounds
    if b:
        p.board_info.board_bounds = (b[0] + dx, b[1] + dy, b[2] + dx, b[3] + dy)
    p._fanout_all_foreign_immovable = True
    return p


def _fan(p, nets):
    """The fanout plus the engine's initial occupancy stamp: per layer the
    blocked and soft bytes of the grid at the first per-ball message."""
    _GRIDS.clear()
    snap = []

    def cb(cur, tot, what):
        if 'escape ' in what and not snap and _GRIDS:
            o = _GRIDS[-1]
            snap.append(([bytes(g) for g in o.grid], [bytes(g) for g in o.soft],
                         o.nx, o.ny))
    with contextlib.redirect_stdout(io.StringIO()):
        tracks, vias, _rm, failed = generate_bga_fanout(
            p.footprints[REF], p, net_filter=nets, layers=['F.Cu', 'B.Cu'],
            track_width=0.1, clearance=0.1, via_size=0.25, via_drill=0.15,
            exit_margin=0.5, escape_method='auto', plane_drop='off',
            progress_callback=cb)
    return tracks, vias, failed, (snap[0] if snap else None)


def _stamp_diff(s0, s1):
    """Cells whose blocked or soft byte differs between two stamps; None
    when a stamp is missing or the grids are not the same shape."""
    if s0 is None or s1 is None or s0[2:] != s1[2:]:
        return None
    n = 0
    for a, b in zip(s0[0] + s0[1], s1[0] + s1[1]):
        n += sum(1 for x, y in zip(a, b) if x != y)
    return n


def _vias(vias, dx, dy):
    return {(round(v['x'] - dx, 4), round(v['y'] - dy, 4), v['net_id']) for v in vias}


def _seg_pt(a, b, p):
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


CELL = 0.03   # one occupancy cell of a 0.8 mm pitch array (pitch / 32 = 0.025 mm)


def _same_routing(t0, t1, dx, dy):
    """Every segment endpoint of each run lies within one occupancy cell of
    the other run's copper on the same net and layer: the same escapes,
    faces and paths, the last cell of a jog free to differ. Returns the
    nets that fail that."""
    def segs(tracks, ox, oy):
        out = {}
        for t in tracks:
            out.setdefault((t['net_id'], t['layer']), []).append(
                ((t['start'][0] - ox, t['start'][1] - oy), (t['end'][0] - ox, t['end'][1] - oy)))
        return out
    A, B = segs(t0, 0.0, 0.0), segs(t1, dx, dy)
    bad = set()
    for X, Y in ((A, B), (B, A)):
        for key, sl in X.items():
            other = Y.get(key, [])
            for a, b in sl:
                if not other:
                    bad.add(key[0])
                    continue
                for p in (a, b):
                    if min(_seg_pt(c, d, p) for c, d in other) > CELL:
                        bad.add(key[0])
    return bad


def main():
    results = []
    p0 = _board(0.0, 0.0)
    fp = p0.footprints[REF]
    nets = sorted({q.net_name for q in fp.pads if q.net_id and len(p0.nets[q.net_id].pads) > 1})
    print(f'{REF}: {len(fp.pads)} balls, {len(nets)} nets with a far end')
    t0, v0, f0, s0 = _fan(p0, nets)
    _ok(results, 'the board fans out in place', bool(t0) and s0 is not None,
        f'{len(t0)} tracks {len(v0)} vias {len(f0)} failed; stamp {s0[2]}x{s0[3]} cells' if s0 else 'NO STAMP')
    for dx, dy in SHIFTS:
        t, v, f, s1 = _fan(_board(dx, dy), nets)
        dv = _vias(v, dx, dy) ^ _vias(v0, 0.0, 0.0)
        bad = _same_routing(t0, t, dx, dy)
        nd = _stamp_diff(s0, s1)
        _ok(results, f'shifted by ({dx}, {dy}) mm the initial occupancy stamp is byte-identical',
            nd == 0, 'grids differ in shape' if nd is None else f'{nd} cell(s) differ')
        _ok(results, f'shifted by ({dx}, {dy}) mm the fanout is the same routing',
            not dv and not bad and sorted(f) == sorted(f0),
            f'{len(dv)} via(s) differ, {len(bad)} net(s) routed elsewhere, '
            f'{len(t)} vs {len(t0)} tracks (the last cell of a jog may differ)')
    # non-vacuity: ring ties whose RAW depths differ in the last bit under a shift
    dx, dy = SHIFTS[2]
    p1 = _board(dx, dy)
    def raw_depths(p):
        f = p.footprints[REF]
        xs = [q.global_x for q in f.pads]
        ys = [q.global_y for q in f.pads]
        mnx, mxx, mny, mxy = min(xs), max(xs), min(ys), max(ys)
        return {q.pad_number: min(q.global_x - mnx, mxx - q.global_x, q.global_y - mny, mxy - q.global_y)
                for q in f.pads}
    d0, d1 = raw_depths(p0), raw_depths(p1)
    ties = 0
    moved = 0
    nums = list(d0)
    for i in range(len(nums)):
        for j in range(i + 1, len(nums)):
            a, b = nums[i], nums[j]
            if abs(d0[a] - d0[b]) < 1e-9:
                ties += 1
                if (d0[a] == d0[b]) != (d1[a] == d1[b]) or ((d0[a] < d0[b]) != (d1[a] < d1[b]) and d1[a] != d1[b]):
                    moved += 1
    _ok(results, 'NON-VACUITY: the board has ring ties the shift re-orders in the last bit',
        ties > 0 and moved > 0, f'{ties} tied pairs, {moved} of them ordered differently by raw arithmetic')
    n_ok = sum(1 for _, ok in results if ok)
    print('=' * 60)
    print(f'{n_ok}/{len(results)} fanout translation tests passed')
    return 0 if n_ok == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())

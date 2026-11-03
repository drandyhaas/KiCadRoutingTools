#!/usr/bin/env python3
"""The oracle's exact-fill strap never seeds a fill EDGE point.

A seed overrides the obstacle map -- a route has to be allowed to begin
there -- so `_exact_fill_endpoints` seeds a fill island by its interior,
inset by the strap's half-width, because the edge of a fill sits exactly
zone clearance from the copper that carved it, which is closer than a track
may come. It did that only when the strap's nearest approach was itself a
fill point. When it was a track, pad or via point, the side's seeds were
every point of its cluster -- the islands' raw edge samples included -- and
one GND strap ended on such a point, 0.032 mm inside a foreign track's
clearance.

Net A on B.Cu: the main island (x 1..20) with a track running out of it to
x 22, and an orphan island (x 25..39) with a track end on it. The nearest
approach is the main track's end, so the main side takes the copper-point
branch. Every seed must be either the net's own track copper or at least the
inset from its island's edge.

The exact islands are given, so no KiCad is needed.

    python3 tests/test_oracle_seed_interior.py
"""
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))


def _rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


MAIN = (1.0, 1.0, 20.0, 39.0)
ORPHAN = (25.0, 1.0, 39.0, 39.0)
ISLANDS = {('A', 'B.Cu'): [_rect(*MAIN), _rect(*ORPHAN)]}
SEGS = [(5, 20, 22, 20), (30, 20, 35, 20)]
CORNER = (-3.0, -3.0, 'B.Cu', 'zone')     # kicad-cli's off-board zone stand-in
TRACK = (30.0, 20.0, 'B.Cu', 'track')
TRACK_HALF = 0.1
INSET = TRACK_HALF + 0.05


def _board(path):
    s = ''.join(f'\n  (segment (start {a} {b}) (end {c} {d}) (width 0.2) '
                f'(layer "B.Cu") (net "A"))' for a, b, c, d in SEGS)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(f'''(kicad_pcb (version 20250114) (generator "pcbnew") (generator_version "10.0")
  (general (thickness 1.6))
  (paper "A4")
  (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (gr_rect (start 0 0) (end 40 40) (stroke (width 0.1) (type default)) (fill none) (layer "Edge.Cuts")){s}
)
''')


def _on_track(x, y):
    return any(min(a, c) - 1e-6 <= x <= max(a, c) + 1e-6
               and abs(y - b) < 1e-6 for a, b, c, d in SEGS)


def _edge_distance(x, y):
    """Distance to the nearest island boundary, for a point inside one."""
    for x0, y0, x1, y1 in (MAIN, ORPHAN):
        if x0 <= x <= x1 and y0 <= y <= y1:
            return min(x - x0, x1 - x, y - y0, y1 - y)
    return None


def main():
    from kicad_parser import parse_kicad_pcb
    from kicad_oracle import _exact_fill_endpoints
    with tempfile.TemporaryDirectory() as td:
        b = os.path.join(td, 'b.kicad_pcb')
        _board(b)
        pcb = parse_kicad_pcb(b)
    nid = next(i for i, n in pcb.nets.items() if n.name == 'A')
    r = _exact_fill_endpoints(pcb, nid, 'A', CORNER, TRACK, ISLANDS,
                              track_half=TRACK_HALF, edge_clearance=0.2)
    assert isinstance(r, tuple), f"no strap: {r}"
    src, tgt, pa, pb, _layer = r
    ends = {(round(pa[0], 3), round(pa[1], 3)), (round(pb[0], 3), round(pb[1], 3))}
    assert (22.0, 20.0) in ends, \
        f"the nearest approach is not the main track's end (22, 20): {pa} {pb}"
    bad = []
    fill = 0
    for side, seeds in (('src', src), ('tgt', tgt)):
        for p in seeds:
            if _on_track(p[0], p[1]):
                continue
            fill += 1
            d = _edge_distance(p[0], p[1])
            if d is None or d < INSET - 1e-6:
                bad.append((side, round(p[0], 4), round(p[1], 4), d))
    assert fill, "no fill seeds at all: the islands were not seeded"
    assert not bad, (f"{len(bad)} seed(s) on or near a fill edge (need "
                     f">= {INSET} mm in): {bad[:5]}")
    print(f"  PASS: {fill} fill seeds, every one >= {INSET} mm inside its island")


if __name__ == '__main__':
    main()
    print("PASS: test_oracle_seed_interior")

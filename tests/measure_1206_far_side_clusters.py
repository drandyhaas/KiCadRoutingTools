#!/usr/bin/env python3
"""#1206: what one far-side box per CLUSTER of drilled pads changes.

A through-hole part's far side used to be ONE box over all its drilled pads,
so two mounting holes 48 mm apart (CM5's Module302) became a 3 x 51 mm strip
on the other face. `legality.far_side_local` now gives one box per cluster
(`FAR_SIDE_CLUSTER_GAP_MM`). This measures every courtyard pair whose area
moved between the two models, on the same boards, in one process:

  * UNION   -- `far_side_local` patched to the old single box;
  * CLUSTER -- the shipped model.

and classifies each moved pair by the one question that decides whether the
change is safe: does any of the part's drilled-pad boxes (the per-pad boxes,
not the clusters) meet the other part's occupancy? A pair where one does is
REAL far-side contact and must keep a non-zero area; a pair where none does
is a phantom of the union box.

    python3 tests/measure_1206_far_side_clusters.py [boards...] [--corpus]

Exit 1 when a real pair lost its area, or a pair was ADDED (every cluster box
lies inside the union box, so the cluster model can only remove area).
"""
from __future__ import annotations

import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))


def _pairs(pcb, path):
    from placement import legality
    return {(p.a, p.b): p for p in legality.body_overlap_pairs(
        legality.graded_parts_from_file(pcb, path))}


def measure(path):
    from kicad_parser import parse_kicad_pcb
    from placement import legality
    from shapely.geometry import box
    pcb = parse_kicad_pcb(path)
    real = legality.far_side_local
    legality.far_side_local = legality.through_pad_bounds_local
    try:
        old = _pairs(pcb, path)
    finally:
        legality.far_side_local = real
    new = _pairs(pcb, path)
    graded = {g.ref: g for g in legality.graded_parts_from_file(pcb, path)}
    rows, bad = [], []
    for key in sorted(set(old) | set(new)):
        a_old = old[key].area_mm2 if key in old else 0.0
        a_new = new[key].area_mm2 if key in new else 0.0
        if abs(a_old - a_new) <= 1e-9:
            continue
        if key not in old:
            bad.append(('ADDED', key, a_old, a_new))
            continue
        real_contact = False
        for me, other in (key, key[::-1]):
            fp = pcb.footprints[me]
            g, go = graded[me], graded[other]
            if not g.has_tht or go.side == g.side:
                continue
            rot = fp.rotation or 0.0
            for b in legality.drilled_pad_boxes_local(fp):
                x0, y0, x1, y1 = legality.rotate_local_bounds(*b, rot)
                hb = box(fp.x + x0, fp.y + y0, fp.x + x1, fp.y + y1)
                shape = go.poly if go.poly is not None else box(*go.rect)
                if hb.intersection(shape).area > 1e-9:
                    real_contact = True
        kind = 'real' if real_contact else 'phantom'
        rows.append((kind, key, a_old, a_new))
        if real_contact and a_new <= 1e-9:
            bad.append(('REAL-LOST', key, a_old, a_new))
    return rows, bad


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('boards', nargs='*')
    ap.add_argument('--corpus', action='store_true')
    args = ap.parse_args(argv)
    boards = list(args.boards)
    if args.corpus:
        import run_utils
        boards += [os.path.join(ROOT, b) for b in run_utils.corpus_boards()]
    worst = 0
    tot = {'phantom': 0, 'real': 0}
    for b in boards:
        rows, bad = measure(b)
        for kind, key, ao, an in rows:
            tot[kind] += 1
            print(f"{os.path.basename(b)[:34]:34s} {kind:8s} {key[0]}<->{key[1]}"
                  f"  {ao:.4f} -> {an:.4f} mm2")
        for what, key, ao, an in bad:
            worst = 1
            print(f"  {what}: {os.path.basename(b)} {key} {ao} -> {an}")
    print(f"\nmoved pairs: {tot['phantom']} phantom (removed or shrunk), "
          f"{tot['real']} real (kept, area shrunk to the touching clusters)")
    return worst


if __name__ == '__main__':
    sys.exit(main())

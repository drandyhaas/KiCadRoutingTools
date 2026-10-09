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
not the clusters) meet the other part's occupancy -- or, for two parts on one
face, which share the other face only through their far sides, the other
part's drilled-pad boxes? A pair where one does is REAL far-side contact and
must keep a non-zero area; a pair where none does is a phantom of the union
box. (The first version skipped same-face pairs, so CM5's Module301<->Module302
-- coincident NPTH posts, which KiCad reports as holes_co_located -- read as a
phantom while it was kept; phase-2 verifier.)

AREA IS NOT THE VERDICT. `verdict_changes` grades both models with
`grade_body_overlap` and lists every pair whose `courtyard_blocking` status
flipped: the relative floor (25% of the smaller part) has the far side in its
denominator, so a pair can turn blocking at EQUAL area. Reported, not failed:
the smaller denominator is the cluster's real size.

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
    legality.far_side_local = (
        lambda fp, *_a, **_k: legality.through_pad_bounds_local(fp))
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
            if not g.has_tht:
                continue
            if go.side == g.side:
                # One face for both: their far sides are what meet.
                if not go.has_tht:
                    continue
                shape = _hole_boxes(pcb.footprints[other])
            else:
                shape = go.poly if go.poly is not None else box(*go.rect)
            if _hole_boxes(fp).intersection(shape).area > 1e-9:
                real_contact = True
        kind = 'real' if real_contact else 'phantom'
        rows.append((kind, key, a_old, a_new))
        if real_contact and a_new <= 1e-9:
            bad.append(('REAL-LOST', key, a_old, a_new))
    return rows, bad


def _hole_boxes(fp):
    """The union of a footprint's per-pad drilled boxes, on the board."""
    from placement import legality
    from shapely.geometry import box
    from shapely.ops import unary_union
    rot = fp.rotation or 0.0
    out = []
    for b in legality.drilled_pad_boxes_local(fp):
        x0, y0, x1, y1 = legality.rotate_local_bounds(*b, rot)
        out.append(box(fp.x + x0, fp.y + y0, fp.x + x1, fp.y + y1))
    return unary_union(out)


def verdict_changes(path):
    """[(change, (a, b), area_union, area_cluster)]: every pair whose
    courtyard_blocking status differs between the two models. `change` is
    'BLOCKING-ADDED' or 'BLOCKING-REMOVED'."""
    from kicad_parser import parse_kicad_pcb
    from placement import legality

    def grade():
        g = legality.grade_body_overlap(parse_kicad_pcb(path), 0.2,
                                        pcb_file=path)
        return {(p.a, p.b): p.area_mm2 for p in g['courtyard_blocking_pairs']}
    real = legality.far_side_local
    legality.far_side_local = (
        lambda fp, *_a, **_k: legality.through_pad_bounds_local(fp))
    try:
        old = grade()
    finally:
        legality.far_side_local = real
    new = grade()
    out = []
    for key in sorted(set(old) | set(new)):
        if key in old and key not in new:
            out.append(('BLOCKING-REMOVED', key, old[key], None))
        elif key in new and key not in old:
            out.append(('BLOCKING-ADDED', key, None, new[key]))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('boards', nargs='*')
    ap.add_argument('--corpus', action='store_true')
    ap.add_argument('--verdicts', action='store_true',
                    help='also grade both models and list every pair whose '
                         'courtyard_blocking status flipped')
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
        if args.verdicts:
            for what, key, ao, an in verdict_changes(b):
                print(f"  {what}: {os.path.basename(b)} {key[0]}<->{key[1]}"
                      f"  {ao} -> {an}")
    print(f"\nmoved pairs: {tot['phantom']} phantom (removed or shrunk), "
          f"{tot['real']} real (kept, area shrunk to the touching clusters)")
    return worst


if __name__ == '__main__':
    sys.exit(main())

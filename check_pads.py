#!/usr/bin/env python3
"""Pad-geometry sanity check: report different-net pads whose copper overlaps.

Two pads on different nets must never have overlapping copper (that is a short).
A handful of overlapping pairs almost always means the pad *geometry* is being
modelled wrong - most commonly a rotated rectangular pad whose orientation was
dropped, so a long pad that should point outward is modelled across its
neighbours (an LQFP/QFN placed at a non-orthogonal angle is the classic trigger).

Because the fanout tools (`qfn_fanout.py`, `bga_fanout.py`) escape pads using
exactly this geometry, run this FIRST as a sanity check: if the pad rectangles
overlap here, the escape stubs will be placed wrong (across neighbouring pads).

Exit code is the number of overlapping pairs (0 = clean), so it gates a pipeline.

    python3 check_pads.py board.kicad_pcb [--tolerance 0.05] [--quiet]
"""
from __future__ import annotations

import argparse
import math
import sys
from typing import List, Tuple

from kicad_parser import parse_kicad_pcb, Pad


def _pad_outline_polygon(pad, k: int = 4) -> List[Tuple[float, float]]:
    """Convex outline of a pad's copper in global coords, honouring shape and
    rect_rotation. Circle/oval/roundrect corners are ARCS (sampled k points each),
    not the sharp bounding-box corners -- so a round pad is not over-modelled by
    its bbox (which manufactures phantom overlaps, #232/#260). Rect -> 4 corners."""
    hx, hy = pad.size_x / 2.0, pad.size_y / 2.0
    shape = getattr(pad, 'shape', 'rect')
    if shape in ('circle', 'oval'):
        r = min(hx, hy)
    elif shape == 'roundrect':
        r = (getattr(pad, 'roundrect_rratio', 0.0) or 0.0) * min(pad.size_x, pad.size_y)
    else:
        r = 0.0
    r = max(0.0, min(r, hx, hy))
    corners = [(hx - r, -(hy - r), -90, 0), (hx - r, hy - r, 0, 90),
               (-(hx - r), hy - r, 90, 180), (-(hx - r), -(hy - r), 180, 270)]
    local = []
    for cx, cy, a0, a1 in corners:
        if r > 1e-9:
            for i in range(k + 1):
                a = math.radians(a0 + (a1 - a0) * i / k)
                local.append((cx + r * math.cos(a), cy + r * math.sin(a)))
        else:
            local.append((cx, cy))
    th = math.radians(getattr(pad, 'rect_rotation', 0.0) or 0.0)
    c, s = math.cos(th), math.sin(th)
    out = []
    for ox, oy in local:
        gx = pad.global_x + ox * c - oy * s
        gy = pad.global_y + ox * s + oy * c
        if not out or math.hypot(gx - out[-1][0], gy - out[-1][1]) > 1e-9:
            out.append((gx, gy))
    return out


def _overlap_depth(a: List[Tuple[float, float]], b: List[Tuple[float, float]]) -> float:
    """Separating-axis penetration depth between two convex polygons (mm), for any
    vertex count. >0 = overlap by that depth; <=0 = (negative) gap."""
    max_gap = -1e18
    for poly in (a, b):
        n = len(poly)
        for i in range(n):
            ex = poly[(i + 1) % n][0] - poly[i][0]
            ey = poly[(i + 1) % n][1] - poly[i][1]
            nx, ny = -ey, ex
            L = math.hypot(nx, ny) or 1.0
            nx, ny = nx / L, ny / L
            amin = min(nx * p[0] + ny * p[1] for p in a)
            amax = max(nx * p[0] + ny * p[1] for p in a)
            bmin = min(nx * p[0] + ny * p[1] for p in b)
            bmax = max(nx * p[0] + ny * p[1] for p in b)
            gap = max(bmin - amax, amin - bmax)
            if gap > max_gap:
                max_gap = gap
    return -max_gap


def _copper_layers(pad) -> set:
    """Copper layers a pad occupies. Through-hole pads (drill > 0) and pads with a
    '*.Cu' wildcard span every copper layer, so they conflict with anything."""
    if pad.drill and pad.drill > 0:
        return {"*"}
    out = set()
    for lyr in (pad.layers or []):
        if lyr == "*.Cu":
            return {"*"}
        if lyr.endswith(".Cu"):
            out.add(lyr)
    return out


def _shares_layer(a, b) -> bool:
    la, lb = _copper_layers(a), _copper_layers(b)
    if "*" in la or "*" in lb:
        return True
    return bool(la & lb)


def _overlaps_in(pads, tolerance):
    """Different-net copper overlaps among a flat pad list (deeper than tolerance).
    Only pads sharing a copper layer can short (edge-connector fingers on opposite
    sides, for example, never conflict). Overlap depth is measured with the pad's
    TRUE copper shape (check_drc's exact circle/oval/roundrect/custom-polygon
    model), not a bounding rectangle -- a round pad's bbox corner otherwise reads
    as overlapping a neighbour it doesn't actually touch (#232/#260 phantom)."""
    pads = [p for p in pads if p.size_x > 0 and p.size_y > 0 and p.net_id != 0]
    polys = [_pad_outline_polygon(p) for p in pads]
    reach = [math.hypot(p.size_x, p.size_y) / 2.0 for p in pads]
    hits = []
    for i, a in enumerate(pads):
        for j in range(i + 1, len(pads)):
            b = pads[j]
            if a.net_id == b.net_id:
                continue
            near = reach[i] + reach[j] + tolerance
            if abs(a.global_x - b.global_x) > near or abs(a.global_y - b.global_y) > near:
                continue
            if not _shares_layer(a, b):
                continue
            depth = _overlap_depth(polys[i], polys[j])
            if depth > tolerance:
                hits.append((a, b, depth))
    return hits


def find_pad_overlaps(pcb, tolerance: float = 0.05, component: str = None,
                      cross_footprint: bool = False):
    """Return [(padA, padB, depth_mm), ...] for different-net pad-copper overlaps.

    By default checks pads WITHIN each footprint (the rotation-modelling bug makes
    a part's own pads collide - exactly what matters before fanout). Net-0 (no
    connection) pads are skipped, so fiducials and mechanical pads don't trip it.
    `component` restricts the check to one footprint; `cross_footprint=True` also
    tests pads across different footprints (a broader board-level short check, but
    noisier on tightly-placed passives).
    """
    if cross_footprint:
        pads = [pd for fp in pcb.footprints.values() for pd in fp.pads]
        return _overlaps_in(pads, tolerance)
    hits = []
    for ref, fp in pcb.footprints.items():
        if component and ref != component:
            continue
        hits.extend(_overlaps_in(fp.pads, tolerance))
    return hits


def main():
    ap = argparse.ArgumentParser(description="Check for overlapping different-net pad copper")
    ap.add_argument("pcb", help="Input PCB file")
    ap.add_argument("--tolerance", type=float, default=0.05,
                    help="Minimum overlap depth (mm) to report (default 0.05)")
    ap.add_argument("--component", "-c", default=None,
                    help="Restrict the check to one footprint reference")
    ap.add_argument("--cross-footprint", action="store_true",
                    help="Also check overlaps across different footprints "
                         "(broader, noisier board-level short check)")
    ap.add_argument("--quiet", "-q", action="store_true",
                    help="Print only the PASS/FAIL summary line")
    args = ap.parse_args()

    pcb = parse_kicad_pcb(args.pcb)
    hits = find_pad_overlaps(pcb, tolerance=args.tolerance, component=args.component,
                             cross_footprint=args.cross_footprint)

    if not args.quiet:
        print(f"Checking pad geometry in {args.pcb} ...")
    if not hits:
        print(f"OK: no overlapping different-net pads (tolerance {args.tolerance} mm)")
        return 0

    hits.sort(key=lambda h: -h[2])
    if not args.quiet:
        print(f"FAILED: {len(hits)} overlapping different-net pad pair(s):")
        for a, b, depth in hits[:40]:
            print(f"  {a.component_ref}.{a.pad_number} ({a.net_name or '?'}) <-> "
                  f"{b.component_ref}.{b.pad_number} ({b.net_name or '?'})  "
                  f"overlap {depth:.3f} mm  at ({a.global_x:.2f},{a.global_y:.2f})")
        if len(hits) > 40:
            print(f"  ... and {len(hits) - 40} more")
        print("Likely a pad-rotation modelling error - fix before running fanout.")
    else:
        print(f"FAILED ({len(hits)} overlapping pad pairs)")
    return len(hits)


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Flag non-orthonormal track segments longer than ~1 grid spacing.

An on-grid router emits only 0/45/90-degree segments. The ONLY legitimate
non-orthonormal segment is a SHORT terminal connector joining an on-grid path
to an off-grid pad/ball/fanout-stub end -- which is <= ~1 grid cell long. Any
LONGER non-orthonormal segment is a routing defect: a terminal connector or a
merged terminal that spanned many cells (it can cut straight across foreign
copper -- see issues #157 / #159). This check finds them directly in the
output, independent of DRC.
"""
import argparse
import math
from kicad_parser import parse_kicad_pcb


def angle_off_45(dx, dy):
    """Degrees between a segment's direction and the nearest multiple of 45."""
    ang = math.degrees(math.atan2(dy, dx))
    return abs(((ang + 22.5) % 45) - 22.5), ang


# Perimeter packages handled by qfn_fanout (matches LQFP/TQFP/VQFP, VQFN/WQFN, etc.).
QFN_KEYWORDS = ("QFN", "QFP", "DFN")


def _qfn_pad_points(pcb):
    """Pad centres of QFN/QFP/DFN perimeter packages only -- the ones whose
    escape stubs qfn_fanout produces. NOT all pads (too general)."""
    pts = []
    for fp in pcb.footprints.values():
        name = (fp.footprint_name or "").upper()
        if any(k in name for k in QFN_KEYWORDS):
            for pad in fp.pads:
                pts.append((pad.global_x, pad.global_y))
    return pts


def _near_pad(x, y, pad_pts, tol):
    for px, py in pad_pts:
        if abs(px - x) <= tol and abs(py - y) <= tol and math.hypot(px - x, py - y) <= tol:
            return True
    return False


def _stub_segment_ids(pcb, pad_tol=0.06, via_tol=0.06):
    """Segment indices that are QFN_FANOUT STUBS specifically: copper reachable
    from a QFN/QFP/DFN pad by walking connected same-net segments WITHOUT crossing
    a via. The escape (pad -> tilted run -> drop via) is all reachable before its
    via; a mid-route defect like #159 sits behind a via, so it is NOT reached. Only
    perimeter-package pads seed the walk, so ordinary terminal connections and BGA
    stubs are not swept up."""
    from collections import defaultdict, deque
    R = lambda v: round(v, 3)
    via_nodes = {(R(v.x), R(v.y)) for v in pcb.vias}

    def is_via(n):
        return any(abs(n[0] - vx) <= via_tol and abs(n[1] - vy) <= via_tol
                   for vx, vy in via_nodes)

    pad_pts = _qfn_pad_points(pcb)
    segs = pcb.segments
    by_net = defaultdict(list)
    for i, s in enumerate(segs):
        by_net[s.net_id].append(i)

    stub = set()
    for idxs in by_net.values():
        adj = defaultdict(list)
        nodes = set()
        for i in idxs:
            s = segs[i]
            a, b = (R(s.start_x), R(s.start_y)), (R(s.end_x), R(s.end_y))
            adj[a].append((i, b))
            adj[b].append((i, a))
            nodes.add(a); nodes.add(b)
        seeds = [n for n in nodes if not is_via(n) and _near_pad(n[0], n[1], pad_pts, pad_tol)]
        seen = set(seeds)
        dq = deque(seeds)
        while dq:
            n = dq.popleft()
            for i, other in adj[n]:
                stub.add(i)                       # pad-reachable segment = stub
                if other not in seen and not is_via(other):
                    seen.add(other)
                    dq.append(other)              # do not expand past a via
    return stub


def find_long_nonortho(pcb, max_len, angle_tol, exclude_stubs=True, pad_tol=0.06):
    stub_ids = _stub_segment_ids(pcb, pad_tol=pad_tol) if exclude_stubs else set()
    flagged = []
    excluded = 0
    for i, s in enumerate(pcb.segments):
        dx, dy = s.end_x - s.start_x, s.end_y - s.start_y
        length = math.hypot(dx, dy)
        if length < 1e-9:
            continue
        off, ang = angle_off_45(dx, dy)
        if off > angle_tol and length > max_len:
            if exclude_stubs and i in stub_ids:
                excluded += 1
                continue
            flagged.append((length, off, ang, s))
    flagged.sort(key=lambda t: -t[0])
    return flagged, excluded


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("board", help="Path to .kicad_pcb file")
    ap.add_argument("--grid-step", type=float, default=0.05,
                    help="Router grid step in mm (default: 0.05)")
    ap.add_argument("--max-len", type=float, default=None,
                    help="Flag non-orthonormal segments longer than this (mm). "
                         "Default: 5 * grid-step (0.25mm at 0.05 grid). bga_fanout's "
                         "legitimate stub-end jogs are <=~0.15mm, so this clears them "
                         "and flags only segments significantly longer.")
    ap.add_argument("--angle-tol", type=float, default=1.0,
                    help="Degrees off a 45-multiple still considered orthonormal "
                         "(default: 1.0, absorbs float noise).")
    ap.add_argument("--max-list", type=int, default=40,
                    help="Max segments to list (default: 40).")
    ap.add_argument("--include-stub-pads", action="store_true",
                    help="Also flag non-orthonormal segments that terminate at a pad "
                         "(by default these -- bga/qfn fanout stubs -- are excluded).")
    ap.add_argument("--pad-tol", type=float, default=0.06,
                    help="Distance (mm) for an endpoint to count as on a pad (default 0.06).")
    args = ap.parse_args()

    max_len = args.max_len if args.max_len is not None else 5.0 * args.grid_step
    pcb = parse_kicad_pcb(args.board)
    names = {nid: n.name for nid, n in pcb.nets.items()}

    flagged, excluded = find_long_nonortho(pcb, max_len, args.angle_tol,
                                           exclude_stubs=not args.include_stub_pads,
                                           pad_tol=args.pad_tol)

    print(f"Checked {len(pcb.segments)} segments "
          f"(max-len={max_len:.4f}mm, angle-tol={args.angle_tol} deg, "
          f"excluded {excluded} pad-terminating stub segment(s))")
    if not flagged:
        print("NO long non-orthonormal segments found.")
        return 0

    print(f"FOUND {len(flagged)} non-orthonormal segment(s) longer than {max_len:.4f}mm:")
    for length, off, ang, s in flagged[:args.max_list]:
        print(f"  {names.get(s.net_id, '?')}  L={length:.3f}mm  ang={ang:.1f} "
              f"(off {off:.1f}deg)  {s.layer}  "
              f"({s.start_x:.3f},{s.start_y:.3f})->({s.end_x:.3f},{s.end_y:.3f})")
    if len(flagged) > args.max_list:
        print(f"  ... and {len(flagged) - args.max_list} more")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())

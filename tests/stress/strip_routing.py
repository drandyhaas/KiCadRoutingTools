#!/usr/bin/env python3
"""Strip all routing (tracks, vias, copper pour zones) from boards, producing
unrouted variants for router stress-testing. Keeps rule areas / keepouts.
Also sets all copper layer types to 'signal' so kicad_parser sees them
(works around parser dropping 'power'-type layers).

Run with KiCad's bundled Python.
"""
import sys
from pathlib import Path
import os
STRESS = Path(os.environ.get("STRESS_DIR", str(Path.home() / "Documents/kicad_stress_test")))
import pcbnew


def rdp(points, eps):
    """Iterative Ramer-Douglas-Peucker polyline simplification."""
    if len(points) < 3:
        return list(points)
    keep = [False] * len(points)
    keep[0] = keep[-1] = True
    stack = [(0, len(points) - 1)]
    while stack:
        i0, i1 = stack.pop()
        ax, ay = points[i0]
        bx, by = points[i1]
        dx, dy = bx - ax, by - ay
        norm = (dx * dx + dy * dy) ** 0.5
        dmax, imax = -1.0, -1
        for i in range(i0 + 1, i1):
            px, py = points[i]
            if norm < 1e-9:
                d = ((px - ax) ** 2 + (py - ay) ** 2) ** 0.5
            else:
                d = abs(dx * (ay - py) - dy * (ax - px)) / norm
            if d > dmax:
                dmax, imax = d, i
        if dmax > eps:
            keep[imax] = True
            stack.append((i0, imax))
            stack.append((imax, i1))
    return [p for p, k in zip(points, keep) if k]


def simplify_closed(points, eps):
    """RDP for a closed loop: simplify as open polyline anchored at far point."""
    if len(points) < 8:
        return list(points)
    closed = list(points) + [points[0]]
    out = rdp(closed, eps)
    return out[:-1] if len(out) > 3 else list(points)

SRC = STRESS / "boards"
DST = STRESS / "boards_unrouted"
DST.mkdir(parents=True, exist_ok=True)

# pcbnew segfaults processing multiple boards per process; pass one name as argv
names = sys.argv[1:]
files = [SRC / f"{n}.kicad_pcb" for n in names] if names else sorted(SRC.glob("*.kicad_pcb"))

for f in files:
    board = pcbnew.LoadBoard(str(f))

    # Copper layer ids, computed once (repeated CuStack() temporaries segfault)
    copper_ids = [lid for lid in board.GetEnabledLayers().CuStack()]

    # 1. Compute outline polygons FIRST (before any board mutation)
    outline_paths = []
    outlines = pcbnew.SHAPE_POLY_SET()
    if board.GetBoardPolygonOutlines(outlines, True):
        for i in range(outlines.OutlineCount()):
            paths = [outlines.Outline(i)] + [outlines.Hole(i, h)
                                             for h in range(outlines.HoleCount(i))]
            for path in paths:
                pts = [(path.CPoint(k).x, path.CPoint(k).y)
                       for k in range(path.PointCount())]
                # Simplify (0.1 mm tol ~ router grid): kicad_parser needs chained lines,
                # but obstacle_map memory scales with grid_cells x vertices
                # (known finding), so keep the vertex count low.
                pts = simplify_closed(pts, eps=200000)  # nm; 0.2 mm tol
                # Near-rectangular outlines: replace with the exact 4-corner
                # bbox so kicad_parser takes its cheap rect-fallback path
                # (obstacle-map memory scales with vertex count).
                xs = [q[0] for q in pts]; ys = [q[1] for q in pts]
                def _area(poly):
                    s = 0
                    for j in range(len(poly)):
                        x1, y1 = poly[j]; x2, y2 = poly[(j + 1) % len(poly)]
                        s += x1 * y2 - x2 * y1
                    return abs(s) / 2
                bbox = (max(xs) - min(xs)) * (max(ys) - min(ys))
                if bbox > 0 and _area(pts) >= 0.98 * bbox:
                    pts = [(min(xs), min(ys)), (max(xs), min(ys)),
                           (max(xs), max(ys)), (min(xs), max(ys))]
                outline_paths.append(pts)
    else:
        print(f"  WARN {f.stem}: GetBoardPolygonOutlines failed, outline left as-is")

    # 2. Snapshot item lists before removal
    tracks = list(board.GetTracks())
    zones = list(board.Zones())
    board_edge_drawings = [d for d in board.GetDrawings()
                           if d.GetLayer() == pcbnew.Edge_Cuts]
    fp_edge_items = []
    for fp in board.GetFootprints():
        for d in fp.GraphicalItems():
            if d.GetLayer() == pcbnew.Edge_Cuts:
                fp_edge_items.append((fp, d))

    # 3. Remove tracks and vias
    for t in tracks:
        board.Remove(t)

    # 4. Remove copper zones (pours), keep rule areas (keepouts)
    removed_zones = kept_zones = 0
    for z in zones:
        if z.GetIsRuleArea():
            kept_zones += 1
            continue
        board.Remove(z)
        removed_zones += 1

    # 5. Replace Edge.Cuts with chained line segments (kicad_parser cannot
    # chain arc/line mixes or footprint-embedded outlines reliably).
    if outline_paths:
        for d in board_edge_drawings:
            board.Remove(d)
        for fp, d in fp_edge_items:
            fp.Remove(d)
        for pts in outline_paths:
            n = len(pts)
            for k in range(n):
                (ax, ay), (bx, by) = pts[k], pts[(k + 1) % n]
                seg = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_SEGMENT)
                seg.SetStart(pcbnew.VECTOR2I(ax, ay))
                seg.SetEnd(pcbnew.VECTOR2I(bx, by))
                seg.SetLayer(pcbnew.Edge_Cuts)
                seg.SetWidth(int(0.05 * 1e6))
                board.Add(seg)

    # 5b. Remove board-level graphics that are neither Edge.Cuts nor copper.
    # kicad_parser's Edge.Cuts regexes cross-match through other gr_* elements
    # (silk artwork swallows edge lines); silk/fab graphics don't affect routing.
    removed_gfx = 0
    for d in list(board.GetDrawings()):
        lid = d.GetLayer()
        if lid == pcbnew.Edge_Cuts or lid in copper_ids:
            continue
        board.Remove(d)
        removed_gfx += 1

    # 6. Set all enabled copper layers to signal type
    for lid in copper_ids:
        board.SetLayerType(lid, pcbnew.LT_SIGNAL)

    out = DST / f.name
    pcbnew.SaveBoard(str(out), board)
    print(f"{f.stem:18s} removed {len(tracks):5d} tracks, {removed_zones:2d} zones "
          f"(kept {kept_zones} rule areas)")

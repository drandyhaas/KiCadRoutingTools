#!/usr/bin/env python3
"""Normalize + strip routing for ONE set-2 board, in a single pcbnew process.

pcbnew segfaults if you process several boards in one process, so this script
takes exactly one (src, dst) pair as argv and is invoked once per board by
prep_set2.sh. It LoadBoard()s the raw GitHub file (normalizing format on save),
strips tracks/vias/copper pours, rebuilds Edge.Cuts as chained segments, drops
non-copper/non-edge graphics, and sets all copper layers to 'signal' so
kicad_parser sees them. Mirrors tests/stress/strip_routing.py.

Run with KiCad's bundled Python:
  /Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3 prep_set2.py <src.kicad_pcb> <dst.kicad_pcb>
"""
import sys
import pcbnew


def rdp(points, eps):
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
    if len(points) < 8:
        return list(points)
    closed = list(points) + [points[0]]
    out = rdp(closed, eps)
    return out[:-1] if len(out) > 3 else list(points)


def main():
    # argv: <src> <routed_dst> <stripped_dst>
    # Loads once, writes a NORMALIZED-but-routed reference (routed_dst, mirrors
    # set-1 boards/ for compare_to_original + measure_routing), then strips and
    # writes the unrouted board (stripped_dst).
    src, routed_dst, dst = sys.argv[1], sys.argv[2], sys.argv[3]
    board = pcbnew.LoadBoard(src)
    pcbnew.SaveBoard(routed_dst, board)  # normalized, routing intact

    copper_ids = [lid for lid in board.GetEnabledLayers().CuStack()]

    outline_paths = []
    outlines = pcbnew.SHAPE_POLY_SET()
    if board.GetBoardPolygonOutlines(outlines, True):
        for i in range(outlines.OutlineCount()):
            paths = [outlines.Outline(i)] + [outlines.Hole(i, h)
                                             for h in range(outlines.HoleCount(i))]
            for path in paths:
                pts = [(path.CPoint(k).x, path.CPoint(k).y)
                       for k in range(path.PointCount())]
                pts = simplify_closed(pts, eps=200000)
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
        print(f"  WARN: GetBoardPolygonOutlines failed, outline left as-is")

    tracks = list(board.GetTracks())
    zones = list(board.Zones())
    board_edge_drawings = [d for d in board.GetDrawings()
                           if d.GetLayer() == pcbnew.Edge_Cuts]
    fp_edge_items = []
    for fp in board.GetFootprints():
        for d in fp.GraphicalItems():
            if d.GetLayer() == pcbnew.Edge_Cuts:
                fp_edge_items.append((fp, d))

    for t in tracks:
        board.Remove(t)

    removed_zones = kept_zones = 0
    for z in zones:
        if z.GetIsRuleArea():
            kept_zones += 1
            continue
        board.Remove(z)
        removed_zones += 1

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

    removed_gfx = 0
    for d in list(board.GetDrawings()):
        lid = d.GetLayer()
        if lid == pcbnew.Edge_Cuts or lid in copper_ids:
            continue
        board.Remove(d)
        removed_gfx += 1

    for lid in copper_ids:
        board.SetLayerType(lid, pcbnew.LT_SIGNAL)

    pcbnew.SaveBoard(dst, board)
    print(f"  OK removed {len(tracks)} tracks, {removed_zones} zones "
          f"(kept {kept_zones} rule areas, {removed_gfx} gfx), {len(copper_ids)} cu layers")


if __name__ == "__main__":
    import os
    try:
        main()
    except Exception:
        import traceback
        # write a clean traceback sidecar (swig noise floods stderr otherwise)
        with open(sys.argv[3] + ".preperr", "w") as f:
            traceback.print_exc(file=f)
        raise
    # pcbnew's GC segfaults during Python interpreter teardown (harmless, the
    # board files are already saved) and pops a macOS crash report. Skip the
    # buggy finalization entirely by exiting hard once our work is done.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)

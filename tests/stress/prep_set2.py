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
import os
import re
import sys
import pcbnew
from pathlib import Path

# #795: KiCad's own __iter__ for GetTracks()/GetDrawings() calls the py2
# `SwigPyIterator.next()`, which a current SWIG no longer supplies.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'py_router'))
from swig_compat import patch_swig_iterators  # noqa: E402
patch_swig_iterators()


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


def strip_zero_length_edge_cuts(path):
    """Delete null-length Edge.Cuts primitives from a WRITTEN board file.

    KiCad flags each one as `invalid_outline` ("segment has null or very small
    length: 0 nm") AND grinds on the malformed outline: on pinci, 335 of them
    (12 board-level, 323 inside footprints) made `kicad-cli pcb drc` take
    **656 seconds**; with them removed the same DRC takes **2.7 seconds**, a
    ~280x speedup, and the real findings are unchanged (199 phantom
    invalid_outline vanish; shorting_items 8->8, solder_mask_bridge 16->16,
    unconnected 101->101, all other violations byte-identical).

    Deleting them cannot change the board shape -- a segment from a point to
    itself draws nothing and mills nothing. This is deliberately ONLY the
    zero-length class: tests/stress/fix_outline_gaps.py also culls open
    fragments and bridges gaps, which on pcie_test_edge and kbic65 made DRC
    WORSE (invalid_outline 4->6 and 3->48), and `--max-bridge 0` deletes the
    fragments instead of bridging them, which wiped kbic65's outline entirely.
    Those repairs stay opt-in and hand-verified; this one is safe to run on
    every board.

    Done as a TEXT pass on the saved file, deliberately: most of pinci's
    degenerate primitives live INSIDE footprints, and removing those through
    the SWIG object API (`FOOTPRINT.Remove`) kills the interpreter with no
    traceback -- prep_set2 then writes nothing at all. Editing the s-expression
    is both safer and exactly the transformation verified on the corpus.
    """
    with open(path, 'r', encoding='utf-8') as f:
        text = f.read()

    spans = []
    for tag in ('gr_line', 'fp_line'):
        for m in re.finditer(r'\(' + tag + r'\b', text):
            i = m.start()
            depth = 0
            for j in range(i, min(i + 4000, len(text))):
                if text[j] == '(':
                    depth += 1
                elif text[j] == ')':
                    depth -= 1
                    if depth == 0:
                        blk = text[i:j + 1]
                        if '"Edge.Cuts"' in blk:
                            s = re.search(r'\(start ([-\d.]+) ([-\d.]+)\)', blk)
                            e = re.search(r'\(end ([-\d.]+) ([-\d.]+)\)', blk)
                            if s and e and (float(s.group(1)) == float(e.group(1))
                                            and float(s.group(2)) == float(e.group(2))):
                                spans.append((i, j + 1))
                        break
    if not spans:
        return 0
    out, prev = [], 0
    for a, b in sorted(spans):
        out.append(text[prev:a])
        prev = b
    out.append(text[prev:])
    with open(path, 'w', encoding='utf-8') as f:
        f.write(''.join(out))
    return len(spans)


def main():
    # argv: <src> <routed_dst> <stripped_dst>
    # Loads once, writes a NORMALIZED-but-routed reference (routed_dst, mirrors
    # set-1 boards/ for compare_to_original + measure_routing), then strips and
    # writes the unrouted board (stripped_dst).
    src, routed_dst, dst = sys.argv[1], sys.argv[2], sys.argv[3]
    # Strip null-length Edge.Cuts primitives from a COPY of the source, then load
    # that -- so both outputs are clean AND the outline rebuild below actually
    # runs. On pinci the degenerate segments make GetBoardPolygonOutlines() fail
    # outright ("outline left as-is"), so the unrouted board kept the malformed
    # outline it was supposed to be normalizing.
    import shutil
    import tempfile
    _clean_src = os.path.join(tempfile.mkdtemp(prefix='prep_clean_'),
                              os.path.basename(src))
    shutil.copy(src, _clean_src)
    _pro = os.path.splitext(src)[0] + '.kicad_pro'
    if os.path.exists(_pro):   # keep the DRC floor beside it (#441)
        shutil.copy(_pro, os.path.splitext(_clean_src)[0] + '.kicad_pro')
    _zl = strip_zero_length_edge_cuts(_clean_src)
    if _zl:
        print(f"  outline: removed {_zl} zero-length Edge.Cuts primitive(s)")

    board = pcbnew.LoadBoard(_clean_src)
    # aSkipSettings: the implicit settings save SIGABRTs on KiCad-9-saved
    # projects (PR #613); the DRC floor travels as explicit sibling copies.
    pcbnew.SaveBoard(routed_dst, board, aSkipSettings=True)  # normalized, routing intact
    for _ext in ('.kicad_pro', '.kicad_dru'):
        _sib = os.path.splitext(src)[0] + _ext
        if os.path.exists(_sib):
            shutil.copy(_sib, os.path.splitext(routed_dst)[0] + _ext)

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

    # RemoveNative, never Remove: outside a running KiCad action plugin
    # pcbnew's Remove() sets item.thisown=1 on PCB_TRACK/PCB_VIA, which have no
    # SWIG destructor, and freeing one corrupts the pcbnew type registry
    # PROCESS-WIDE. This script loops over boards and rebinds `tracks` each
    # iteration, so the previous board's proxies are freed at the top of the
    # next one -- the first board WITH copper poisons every board after it
    # ("'swig_runtime_data5.SwigPyObject' object has no attribute 'GetTracks'").
    for t in tracks:
        board.RemoveNative(t)

    removed_zones = kept_zones = 0
    for z in zones:
        if z.GetIsRuleArea():
            kept_zones += 1
            continue
        board.RemoveNative(z)
        removed_zones += 1

    if outline_paths:
        for d in board_edge_drawings:
            board.RemoveNative(d)
        for fp, d in fp_edge_items:
            fp.RemoveNative(d)
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
        board.RemoveNative(d)
        removed_gfx += 1

    for lid in copper_ids:
        board.SetLayerType(lid, pcbnew.LT_SIGNAL)

    pcbnew.SaveBoard(dst, board, aSkipSettings=True)  # see routed_dst above
    for _ext in ('.kicad_pro', '.kicad_dru'):
        _sib = os.path.splitext(src)[0] + _ext
        if os.path.exists(_sib):
            shutil.copy(_sib, os.path.splitext(dst)[0] + _ext)
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

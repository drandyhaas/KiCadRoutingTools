#!/usr/bin/env python3
"""The outline-gate bbox prefilter changes speed and never an answer.

The oracle below re-implements the CURRENT probe bodies verbatim, including
the #628 `_swallow_pts` loop (cutouts AND reclassified milled contours). An
oracle that mirrors an older body tests a version of the code that no longer
exists -- it went stale exactly once, on the rebase that brought #628 in, and
reported a 0.5mm "mismatch" that was the oracle being wrong.

`BoardOutlineGate.rect_blocked` and `rect_outside_amount` measured every ring
edge of the board against all four sides of the rect. `edges_near` is supposed
to shorten that list, but it prunes per PART, sized by a displacement budget --
and the seeding path has no budget: `pose_score.make_state` builds its
QuenchState without `build_neighbor_lists`, so `_travel_budget` is infinite,
`reach` is infinite, and `edges_near` hands back every edge there is.

Measured on run 19's urchin (638 edges, one milled ring): 9-24 ms per
`rect_outside_amount` depending on machine load. `_try_place` calls it once
per candidate offset, and `_offsets(16, 0.25)` alone is 16,641 offsets --
which is why a single failed seat on that board cost minutes.

The speedup this file prints is load- and population-dependent (7x to 14x
observed). It asserts only `> 3.0` on a many-edge outline, because pinning a
specific multiple would make the test a machine detector.

The prefilter drops edges whose bounding box is separated from the rect's by
more than the margin. That is exact rather than approximate, so the only thing
worth testing is that claim, hard: this compares against a re-implementation of
the ORIGINAL unfiltered loops, over thousands of rects, on every real outline
available -- including rects deliberately straddling, containing and sitting
just off each edge, where an off-by-epsilon filter would show.
"""
import math
import os
import random
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (REPO,):
    if p not in sys.path:
        sys.path.insert(0, p)
        sys.path.insert(0, os.path.join(p, 'py_router'))
        sys.path.insert(0, os.path.join(p, 'py_tools'))
        sys.path.insert(0, os.path.join(p, 'py_placer'))

from kicad_parser import parse_kicad_pcb
from placement.legality import BoardOutlineGate

passed = failed = 0


def check(name, ok, detail=""):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}{(' -- ' + detail) if detail else ''}")


# --- the ORIGINAL implementations, verbatim, as the oracle -------------------
def ref_rect_blocked(g, rect):
    from check_drc import _point_on_board, _seg_seg_dist_coords
    x0, y0, x1, y1 = rect
    for (px, py) in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
        if not _point_on_board(px, py, g.outer, g.cutouts):
            return True
    for (ax, ay, bx, by) in ((x0, y0, x1, y0), (x1, y0, x1, y1),
                             (x1, y1, x0, y1), (x0, y1, x0, y0)):
        for (ex1, ey1, ex2, ey2) in g.edges():
            if _seg_seg_dist_coords(ax, ay, bx, by,
                                    ex1, ey1, ex2, ey2) < g.margin:
                return True
    for (_rid, cx, cy) in g._swallow_pts:
        if x0 <= cx <= x1 and y0 <= cy <= y1:
            return True
    return False


def ref_outside_amount(g, rect):
    from check_drc import (_point_on_board, _point_to_rings_distance,
                           _seg_seg_dist_coords)
    amt = g.bbox_outside_amount(rect)
    if not g.active:
        return amt
    x0, y0, x1, y1 = rect
    for (px, py) in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
        if not _point_on_board(px, py, g.outer, g.cutouts):
            amt += max(g.margin, _point_to_rings_distance(px, py, g.rings))
    for (ax, ay, bx, by) in ((x0, y0, x1, y0), (x1, y0, x1, y1),
                             (x1, y1, x0, y1), (x0, y1, x0, y0)):
        d = min((_seg_seg_dist_coords(ax, ay, bx, by, *e) for e in g.edges()),
                default=float('inf'))
        if d < g.margin:
            amt += g.margin - d
    for (_rid, cx, cy) in g._swallow_pts:
        if x0 <= cx <= x1 and y0 <= cy <= y1:
            amt += g.margin
    return amt


# --- rect generators ---------------------------------------------------------
def rects_for(g, bounds, rng, n):
    """Random rects, plus adversarial ones pinned to the outline itself: a
    filter that is wrong by an epsilon shows up on a rect sitting exactly at
    the margin from an edge, not on a random one in open space."""
    x0, y0, x1, y1 = bounds
    out = []
    for _ in range(n):
        w = rng.uniform(0.2, 12.0)
        h = rng.uniform(0.2, 12.0)
        cx = rng.uniform(x0 - 5, x1 + 5)
        cy = rng.uniform(y0 - 5, y1 + 5)
        out.append((cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2))
    m = g.margin
    for (ax, ay, bx, by) in g.edges()[:400]:
        mx, my = (ax + bx) / 2.0, (ay + by) / 2.0
        for pad in (0.0, m * 0.5, m, m * 1.001, m * 2, 1.0):
            out.append((mx - pad, my - pad, mx + pad, my + pad))
        # a rect straddling the edge, and one just clear of each end
        out.append((min(ax, bx) - 0.3, min(ay, by) - 0.3,
                    max(ax, bx) + 0.3, max(ay, by) + 0.3))
        out.append((ax - m * 0.999, ay - m * 0.999, ax + m * 0.999,
                    ay + m * 0.999))
    return out


BOARDS = [os.path.join(REPO, 'kicad_files', n) for n in (
    'glasgow_revC.kicad_pcb', 'tigard.kicad_pcb', 'splitflap_driver.kicad_pcb',
    'watchy.kicad_pcb')]
URCHIN = os.path.join(REPO, 'wk', 'run19', 'urchin', 'base.kicad_pcb')
if os.path.isfile(URCHIN):
    BOARDS.append(URCHIN)

rng = random.Random(20260815)
tested = 0
timed = []

for path in BOARDS:
    if not os.path.isfile(path):
        continue
    name = os.path.basename(path)
    pcb = parse_kicad_pcb(path)
    if pcb.board_info.board_bounds is None:
        continue
    g = BoardOutlineGate(pcb.board_info, 0.5)
    if not g.active:
        check(f"{name}: skipped (outline IS its bounding box)", True)
        continue
    rects = rects_for(g, pcb.board_info.board_bounds, rng, 300)
    bad_blocked = bad_amount = 0
    worst = 0.0
    for r in rects:
        if g.rect_blocked(r) != ref_rect_blocked(g, r):
            bad_blocked += 1
        a, b = g.rect_outside_amount(r), ref_outside_amount(g, r)
        if not (math.isclose(a, b, rel_tol=0.0, abs_tol=1e-12)):
            bad_amount += 1
            worst = max(worst, abs(a - b))
    tested += len(rects)
    check(f"{name} ({len(g.edges())} edges): rect_blocked identical on "
          f"{len(rects)} rects", bad_blocked == 0, f"{bad_blocked} differed")
    check(f"{name}: rect_outside_amount identical, to the bit",
          bad_amount == 0, f"{bad_amount} differed, worst {worst:g}")

    # speed, on the rects a candidate loop actually asks about (near the board)
    sample = rects[:200]
    t0 = time.perf_counter()
    for r in sample:
        g.rect_outside_amount(r)
    fast = time.perf_counter() - t0
    t0 = time.perf_counter()
    for r in sample:
        ref_outside_amount(g, r)
    slow = time.perf_counter() - t0
    timed.append((name, len(g.edges()), slow / max(fast, 1e-9)))
    print(f"       {name}: {len(g.edges())} edges, "
          f"{slow / len(sample) * 1e3:.2f}ms -> {fast / len(sample) * 1e3:.2f}ms "
          f"({slow / max(fast, 1e-9):.1f}x)")

check("something was actually tested", tested > 0, f"{tested} rects")
# The prefilter must never be a slowdown on a board with a real outline.
check("no board got slower", all(sp > 0.9 for _, _, sp in timed),
      str([(n, round(sp, 2)) for n, _, sp in timed]))
# On a board with many edges it must be a large win, or it is not worth having.
big = [(n, e, sp) for n, e, sp in timed if e >= 200]
if big:
    check("a many-edge outline gets a big speedup",
          all(sp > 3.0 for _, _, sp in big),
          str([(n, e, round(sp, 1)) for n, e, sp in big]))
else:
    print("  NOTE no >=200-edge outline available; speedup claim untested")

print(f"\n{passed} passed, {failed} failed")
sys.exit(1 if failed else 0)

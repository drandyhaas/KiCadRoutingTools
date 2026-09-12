"""Choose one escape move per berth pad from the menu.

The braid's own entry assignment only ever considered moves that go WEST
-- toward the corridor -- and when a net could not take one it was split
off into a separate hand-written flow. The menu (escape_moves.py) offers
every direction, so the choice becomes a real decision:

  * a move costs its own vias, plus the corridor length needed to reach
    its exit point from the net's launch;
  * moves compete for the same channels -- one net per row gap, one net
    per column gap, one via per inter-ball cell -- so the choice is an
    assignment problem, not a per-net one.

Selection is greedy over a cost, in most-constrained-first order, which
is enough to reproduce the old westward choices when only westward moves
are offered (`only_dirs={'left'}`) and to spread onto the other edges
when they are not. It reports WHY each net got what it got, because a
silent assignment is impossible to audit.
"""
from __future__ import annotations

import math
import os
from typing import (Callable, Dict, List, Optional, Sequence,
                    Tuple)

from escape_moves import Move
import escape_moves as em
from schedule import lis_keep, lis_keep_weighted

Pt = Tuple[float, float]


def _lane_span(m: Move) -> Tuple[Tuple, float, float]:
    """The channel this move occupies and the interval it takes along
    it: a row gap (left/right escape) or a column gap (up/down), on its
    own layer, from where the escape enters the gap to where it leaves
    the array."""
    src = m.site if m.site is not None else m.legs[0][0]
    if m.direction in ('left', 'right'):
        key = ('row', round(m.exit_pt[1], 3), m.layer)
        a, b = src[0], m.exit_pt[0]
    else:
        key = ('col', round(m.exit_pt[0], 3), m.layer)
        a, b = src[1], m.exit_pt[1]
    return key, min(a, b), max(a, b)


def _lane_spans(m: Move) -> List[Tuple[Tuple, float, float]]:
    """Every gap stretch a move occupies: a plain move's one lane
    (_lane_span); a CLIMB's (escape_moves climb=) axis-aligned runs on its
    run layer -- the gap it climbs along and the row or column it leaves
    by -- so the conflict test prices what the copper will take."""
    if not m.legs and m.site is None:
        # a move synthesized from copper the engine laid (replan.synth_move)
        # carries no legs: it occupies no lane the selector can price
        return []
    if not getattr(m, 'climb', 0) and not getattr(m, 'walk', 0):
        return [_lane_span(m)]
    out = []
    if getattr(m, 'walk', 0):
        # the walked SURFACE leg (elbow -> site) takes its lane on the home
        # layer, and the run from the site takes its own
        out.append(_lane_span(m))
        # the ELBOW stands at the crossing of a column gap and a row gap on
        # the home layer: a neighbour's stub running either gap through
        # that crossing cannot be laid (K28 dv2: SDQ15/SWE/SDQ7 surface
        # stubs asked next to walked berths, via-in-pad laid, 13 bans)
        if len(m.legs) >= 2:
            (_b, elbow, home) = m.legs[0]
            e = 0.05
            out.append((('col', round(elbow[0], 3), home), elbow[1] - e, elbow[1] + e))
            out.append((('row', round(elbow[1], 3), home), elbow[0] - e, elbow[0] + e))
    for (p, q, L) in m.legs:
        if L != m.layer and not getattr(m, 'walk', 0):
            continue
        if getattr(m, 'walk', 0) and (p, q, L) == m.legs[0]:
            continue                    # the ball -> elbow diagonal: no lane
        if abs(p[0] - q[0]) < 1e-6 and abs(p[1] - q[1]) > 1e-6:
            out.append((('col', round(p[0], 3), L), min(p[1], q[1]), max(p[1], q[1])))
        elif abs(p[1] - q[1]) < 1e-6 and abs(p[0] - q[0]) > 1e-6:
            out.append((('row', round(p[1], 3), L), min(p[0], q[0]), max(p[0], q[0])))
    return out or [_lane_span(m)]


def _length(m: Move) -> float:
    return sum(math.hypot(q[0] - p[0], q[1] - p[1])
               for (p, q, _L) in m.legs)


def _seg_hits_box(a: Pt, b: Pt, box) -> bool:
    """Does segment a-b pass through the axis-aligned box?"""
    x0, y0, x1, y1 = box
    # Liang-Barsky
    dx, dy = b[0] - a[0], b[1] - a[1]
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, a[0] - x0), (dx, x1 - a[0]),
                 (-dy, a[1] - y0), (dy, y1 - a[1])):
        if abs(p) < 1e-12:
            if q < 0:
                return False
            continue
        r = q / p
        if p < 0:
            if r > t1:
                return False
            if r > t0:
                t0 = r
        else:
            if r < t0:
                return False
            if r < t1:
                t1 = r
    return t0 < t1


def _boxes(box):
    """A keep-out is ONE box (x0, y0, x1, y1) or a list of boxes -- the
    blocks of a banded array (escape_moves.blocks_of), between which the
    band is open. One box, or a list of one, takes the single-box code
    unchanged."""
    if box and isinstance(box[0], (tuple, list)):
        return list(box)
    return [box]


def bands_of_boxes(boxes) -> List[Tuple[float, float, float, float]]:
    """The open streets between a banded array's blocks (escape_moves.
    bands_of, from the boxes alone): two boxes whose x extents overlap
    with a gap in y bound a band (x0, y0, x1, y1) on their ball lines;
    likewise in x. Empty for one box."""
    out = []
    bs = _boxes(boxes)
    for i, a in enumerate(bs):
        for b in bs[i + 1:]:
            lo, hi = (a, b) if a[1] <= b[1] else (b, a)
            if lo[3] < hi[1] and min(a[2], b[2]) > max(a[0], b[0]):
                out.append((max(a[0], b[0]), lo[3], min(a[2], b[2]), hi[1]))
                continue
            lo, hi = (a, b) if a[0] <= b[0] else (b, a)
            if lo[2] < hi[0] and min(a[3], b[3]) > max(a[1], b[1]):
                out.append((lo[2], max(a[1], b[1]), hi[0], min(a[3], b[3])))
    return out


def band_of(pt: Pt, bands, tol: float = 0.5) -> Optional[int]:
    """Index of the band `pt` lies in -- strictly between its two ball
    lines, within its length by `tol` -- or None."""
    for i, (x0, y0, x1, y1) in enumerate(bands):
        if x1 - x0 >= y1 - y0:
            if y0 < pt[1] < y1 and x0 - tol <= pt[0] <= x1 + tol:
                return i
        elif x0 < pt[0] < x1 and y0 - tol <= pt[1] <= y1 + tol:
            return i
    return None


# The band's lane model (SPLIT_BLOCKS). A stub in a band is reached
# ALONG the band from the mouth nearer the launch, and the lanes in a
# band NEST: a lane turning off to the north line at column c crosses
# every lane north of it that continues past c, so the north-line
# exiters run north-to-south in exit order and the south-line exiters
# south-to-north. The leg drawn for the crossing tests follows that:
# launch -> the mouth at the lane's nested offset -> along the band ->
# the stub; two band legs then cross exactly when their launches are
# inverted against their nesting, and a band leg crosses a west-face
# leg where the copper would. BAND_TIP is how far a band stub's tip
# stands off its ball line (half a pitch + the engine's exit margin);
# the band's capacity per layer is what fits between the tip lines at
# the block pitch. The caller sets BAND_TIP from the array it plans.
BAND_TIP = 0.9
# BAND_CHAN=0: a band exit's run along the band is NOT priced as a channel
# (an A/B knob for the selector's cost; 1 = priced)
BAND_CHAN = int(os.environ.get('BAND_CHAN', '1'))
# SEL_EXT (2026-09-10, default ON): a menu SUPERSET must never select
# worse, and the greedy does -- K28: the walked menu's greedy plan judged
# 103.5 against 97.9 for the plain menu and routed 39 against 36 vias.
# The first walked pick undercuts a surface berth by dodging one
# plan-model crossing (priced 6 > a via 3 + its channel), and every
# later pick then sees a different world; ordering the greedy by other
# keys did not help (48 routed). So the greedy is SEEDED on the plain
# menu (no walked moves) and the walked moves are left to the judged
# passes -- and the judge that pays is the ROUTE (replan.py --walk
# --length), not the planner: judged searches over the walked menu
# routed 42-46. SEL_EXT=0 restores the greedy over the whole menu.
SEL_EXT = int(os.environ.get('SEL_EXT', '1'))
# SEL_XING (2026-09-10): a row-gap run and a column-gap run on one layer
# that cross are a conflict (see _conflict). 1 (default) = for pairs with
# a walked or climbing move, whose long legs cross the plain stubs' gaps
# (K28: 13 bans -> 0 with it); 2 = every pair, which reaches the plain
# menu and changed the flag-off K28 chain for the worse (37 vias / 692 mm
# on the frozen source against 36 / 670: the seed dodges the two bans the
# passes used to repair, and picks worse); 0 = off.
SEL_XING = int(os.environ.get('SEL_XING', '1'))
SEL_FORCE = int(os.environ.get('SEL_FORCE', '0'))
# SEL_XLAYER (2026-09-11, TODO 13 i): the greedy's crossing price by
# LAYER. Two legs that cross cost the braid only when both lanes must
# share one layer -- both nets born on that layer AND berthed on it, with
# no via owed that a page change could ride; every other crossing pair
# can be put on different pages and routed free (the two-page braid).
# Priced alike (cross_weight for every geometric crossing) the greedy
# dodged free crossings and accepted the costly ones: the K41 chain plan
# carried 121 inversions among front-born front-stub lanes (the human's
# 0) and 13 swimmers. 1: a must-share pair costs cross_weight, any other
# crossing XLAYER_FREE of it (the page it consumes). 0 = as recorded.
SEL_XLAYER = int(os.environ.get('SEL_XLAYER', '0'))
XLAYER_FREE = float(os.environ.get('XLAYER_FREE', '0.15'))
SEL_RETRY = int(os.environ.get('SEL_RETRY', '0'))


# NEST_IN / NEST_STEP / BAND_LPITCH were USED here and DEFINED NOWHERE --
# not in this file, not in the tree, not in any commit in git history. So
# every path through band_leg / band_capacity raised NameError, i.e. the
# first net whose exit lands in a band crashed the planner. That is only
# reachable with SPLIT_BLOCKS=1 (it is what makes `keep_out` a list of
# block boxes and `Corridor.bands` non-empty), which is why it never fired
# on the default chain -- and it means the README's recorded verdict for
# SPLIT_BLOCKS, "complete, general, LOST", was never measured. It is a
# crash, not a loss. Values below are the braid's own band geometry
# (Corridor.offsets' comb): the nested rider sits a track+clearance inside
# the tip line and each deeper rider steps by the same, and a band packs
# at the braid's lane pitch.
NEST_IN = 0.232                     # TRACK 0.127 + CLEAR 0.105: one lane's slice
NEST_STEP = 0.0                     # no per-depth ramp: the comb is parallel
BAND_LPITCH = 0.35                  # braid.LPITCH -- the comb's lane pitch


def band_leg(launch: Pt, pt: Pt, band) -> List[Pt]:
    x0, y0, x1, y1 = band
    if x1 - x0 >= y1 - y0:
        mx = x0 if abs(launch[0] - x0) <= abs(launch[0] - x1) else x1
        depth = abs(pt[0] - mx)
        if pt[1] < (y0 + y1) / 2:
            yn = y0 + BAND_TIP + NEST_IN + NEST_STEP * depth
        else:
            yn = y1 - BAND_TIP - NEST_IN - NEST_STEP * depth
        return [launch, (mx, yn), (pt[0], yn), pt]
    my = y0 if abs(launch[1] - y0) <= abs(launch[1] - y1) else y1
    depth = abs(pt[1] - my)
    if pt[0] < (x0 + x1) / 2:
        xn = x0 + BAND_TIP + NEST_IN + NEST_STEP * depth
    else:
        xn = x1 - BAND_TIP - NEST_IN - NEST_STEP * depth
    return [launch, (xn, my), (xn, pt[1]), pt]


BAND_BLOCK_GAP = 0.30   # the braid's BAND_GAP: the comb starts this far inside a tip line


def band_capacity(band) -> int:
    """Lanes a band takes PER LAYER, as the braid packs them (its band
    comb, Corridor.offsets): a comb of side exits between the stub-tip
    lines starting a block gap inside each, at the block pitch, per
    page. The berth's layer is the selector's proxy for the page the
    schedule will give the lane."""
    x0, y0, x1, y1 = band
    w = (y1 - y0) if x1 - x0 >= y1 - y0 else (x1 - x0)
    room = w - 2 * BAND_TIP - 2 * BAND_BLOCK_GAP
    if room < 0:
        return 1
    return int(room / BAND_LPITCH + 1e-9) + 1


def around_boxes_path(a: Pt, b: Pt, boxes, pad: float = 0.3):
    """`around_box_path` over several boxes: the shortest polyline from
    a to b through the padded corners that crosses none of them (a
    visibility graph over the corners, Dijkstra). The straight line when
    it misses every box; a to b straight when nothing at all connects
    them (both ends walled in)."""
    padded = [(x0 - pad, y0 - pad, x1 + pad, y1 + pad) for x0, y0, x1, y1 in boxes]
    e = 0.05
    inner = [(bx[0] + e, bx[1] + e, bx[2] - e, bx[3] - e) for bx in padded]

    def free(p, q):
        return not any(_seg_hits_box(p, q, ib) for ib in inner)
    if free(a, b):
        return [a, b]
    nodes = [a, b]
    for x0, y0, x1, y1 in padded:
        nodes += [(x0, y0), (x1, y0), (x0, y1), (x1, y1)]
    n = len(nodes)
    dist = [float('inf')] * n
    prev = [-1] * n
    dist[0] = 0.0
    done = [False] * n
    for _ in range(n):
        u = min((i for i in range(n) if not done[i]), key=lambda i: dist[i], default=None)
        if u is None or dist[u] == float('inf'):
            break
        done[u] = True
        if u == 1:
            break
        for v in range(n):
            if done[v] or v == u:
                continue
            if not free(nodes[u], nodes[v]):
                continue
            d = dist[u] + math.hypot(nodes[v][0] - nodes[u][0], nodes[v][1] - nodes[u][1])
            if d < dist[v]:
                dist[v] = d
                prev[v] = u
    if dist[1] == float('inf'):
        return [a, b]
    path = []
    v = 1
    while v != -1:
        path.append(nodes[v])
        v = prev[v]
    return path[::-1]


def around_box_path(a: Pt, b: Pt, box, pad: float = 0.3):
    """The polyline `around_box` measures: the straight line when it
    misses the box, otherwise the shorter way round its padded corners.
    Returned so the corridor leg can be DRAWN, not just priced."""
    bs = _boxes(box)
    if len(bs) > 1:
        return around_boxes_path(a, b, bs, pad)
    x0, y0, x1, y1 = bs[0]
    bx = (x0 - pad, y0 - pad, x1 + pad, y1 + pad)
    # hit tests against a box shrunk by a hair (see around_box): a leg
    # from a tooth to a corner runs along the face and only touches
    e = 0.05
    inner = (bx[0] + e, bx[1] + e, bx[2] - e, bx[3] - e)
    if not _seg_hits_box(a, b, inner):
        return [a, b]
    x0, y0, x1, y1 = bx
    corners = ((x0, y0), (x1, y0), (x0, y1), (x1, y1))
    best, path = float('inf'), [a, b]
    for c1 in corners:
        for c2 in corners:
            if _seg_hits_box(a, c1, inner) or _seg_hits_box(c2, b, inner):
                continue
            if c1 != c2 and _seg_hits_box(c1, c2, inner):
                continue
            d = (math.hypot(c1[0] - a[0], c1[1] - a[1])
                 + math.hypot(c2[0] - c1[0], c2[1] - c1[1])
                 + math.hypot(b[0] - c2[0], b[1] - c2[1]))
            if d < best:
                best = d
                path = [a, c1, c2, b] if c1 != c2 else [a, c1, b]
    return path


# The router's own via/length exchange rate: a via costs 75 units on
# the 0.1 mm grid, i.e. one via == 7.5 mm of track. Every objective
# that mixes vias with distance converts at this ONE rate.
# RIDE_MM_PER_VIA overrides it for calibration studies ONLY -- the
# open unit question is that plan_floor counts CROSSINGS (a dive is
# ~2 vias), so the honest rate for the floor+ride sum may be 2x.
VIA_MM = 7.5

# SEL_CONTEND (2026-09-11): vias charged for the ROOM THEY TAKE, in vias per
# contending net (escape_moves.site_contention). 0 = off, the cost unchanged.
# Calibrated, not guessed: over K41's DU1 menu, comparing each net's cheapest
# via-bearing in-array move with its cheapest off-array one, the off-array
# move costs a median of +0.035 vias more and is wanted by 4.07 fewer nets,
# so the weight that flips the median net is 0.035/4.07 = 0.016 vias per
# contender (p25 0.000, p75 0.237).
SEL_CONTEND = float(os.environ.get('SEL_CONTEND', '0') or 0)
VIA_NEED_SITE = 0.36     # two barrels plus clearance: the room one site denies


def ride_mm(sel: Dict[str, 'Move'], launch: Dict[str, Pt],
            keep_out, src_box=None) -> float:
    """Total corridor ride length the choice implies: each net's
    around-the-arrays distance from its launch point to its berth exit.
    The judged plan objective adds this at VIA_MM per via so a berth on
    a far face pays its wrap -- the general cost that replaces face
    restrictions (a floor-only objective walked berths to far faces for
    free and unrestricted plans routed WORSE than restricted ones). With
    `src_box` the SOURCE array is a wall too: a tooth on the face away
    from the destination pays its way round it (K28: the plan sent
    SDQ15 out U1's west face for free, and the braid made it a corridor
    of one)."""
    tot = 0.0
    for n, m in sel.items():
        if n not in launch:
            continue
        a, b = launch[n], m.exit_pt
        d = around_box(a, b, keep_out)
        if src_box is not None:
            straight = math.hypot(b[0] - a[0], b[1] - a[1])
            d += around_box(a, b, src_box) - straight
        tot += d
    return tot


def around_box(a: Pt, b: Pt, box, pad: float = 0.3) -> float:
    """Distance from a to b that does not cross `box`. The straight
    line when it misses; otherwise the shorter of the two ways round,
    bending at the padded corners. The corridor cannot cross the array,
    so a straight-line reach through it is not a distance the router
    could ever realise. Several boxes (a banded array's blocks): the
    length of `around_boxes_path`."""
    bs = _boxes(box)
    if len(bs) > 1:
        pth = around_boxes_path(a, b, bs, pad)
        return sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(pth, pth[1:]))
    x0, y0, x1, y1 = bs[0]
    box = (x0 - pad, y0 - pad, x1 + pad, y1 + pad)
    # the hit tests run against a box shrunk by a hair: a tooth sits a
    # few tens of microns outside the padded box, and the leg from it to
    # a corner runs ALONG the face -- it touches the boundary, and a touch
    # counted as a hit vetoed every way round, so a west-face tooth was
    # priced at the straight line through the array (K4 SDQ11)
    e = 0.05
    inner = (box[0] + e, box[1] + e, box[2] - e, box[3] - e)
    if not _seg_hits_box(a, b, inner):
        return math.hypot(b[0] - a[0], b[1] - a[1])
    x0, y0, x1, y1 = box
    corners = ((x0, y0), (x1, y0), (x0, y1), (x1, y1))
    best = float('inf')
    for c1 in corners:
        for c2 in corners:
            if _seg_hits_box(a, c1, inner) or _seg_hits_box(c2, b, inner):
                continue
            if c1 != c2 and _seg_hits_box(c1, c2, inner):
                continue
            d = (math.hypot(c1[0] - a[0], c1[1] - a[1])
                 + math.hypot(c2[0] - c1[0], c2[1] - c1[1])
                 + math.hypot(b[0] - c2[0], b[1] - c2[1]))
            best = min(best, d)
    if best == float('inf'):
        # both endpoints inside the padded box: fall back to straight
        return math.hypot(b[0] - a[0], b[1] - a[1])
    return best


def _site_key(m: Move) -> Optional[Tuple]:
    if m.site is None:
        return None
    return (round(m.site[0], 3), round(m.site[1], 3))


def side_capacity(menu: Dict[str, List[Move]],
                  bus: Sequence[str], side: str) -> int:
    """How many of this bus can leave on `side` at once: the number of
    distinct (exit line, layer) slots its members can actually reach.
    This is the cut the bundle has to cross."""
    axis = 1 if side in ('left', 'right') else 0
    slots = set()
    for n in bus:
        for m in menu.get(n, ()):
            if m.direction == side:
                slots.add((round(m.exit_pt[axis], 3), m.layer))
    return len(slots)


def certify(menu: Dict[str, List[Move]], bus: Sequence[str],
            side: str) -> Tuple[bool, str]:
    """Maley-style capacity check: every member must have a move on
    this side, and the side must offer at least as many distinct
    (line, layer) slots as there are members. Returns (ok, why)."""
    missing = [n for n in bus
               if not any(m.direction == side for m in menu.get(n, ()))]
    if missing:
        return False, (f'{len(missing)} member(s) have no {side} move '
                       f'({",".join(missing[:4])})')
    cap = side_capacity(menu, bus, side)
    if cap < len(bus):
        return False, (f'cut too narrow: {cap} slots for {len(bus)} nets')
    return True, f'{cap} slots for {len(bus)} nets'


def bus_sides(menu: Dict[str, List[Move]],
              launch: Dict[str, Pt],
              buses: Sequence[Sequence[str]],
              cost_fn, geo: Optional['Corridor'] = None,
              cross_weight: float = 6.0, log=None) -> Dict[str, str]:
    """One exit side per bus: the cheapest side that PASSES the capacity
    certificate AND does not cut through the corridors already placed.

    Certifying first means a side that cannot hold the bundle is never
    offered, instead of being discovered one unplaceable net at a time
    inside greedy assignment.

    The crossing term is the whole point of doing this in order rather
    than per bus. Corridors are as expensive to each other as their
    members are among themselves -- measured at K21, 30 crossings
    BETWEEN corridors against 34 within them -- and a per-bus choice
    cannot see that, because the cost of a side depends on what is
    already there. A 2-net bus sent `up` around the array cost 30
    crossings against two bundles it had no business touching, and
    scored as the cheapest side available.

    Buses are placed largest first, so the big bundles lay down the
    reference and the small ones fit around them, rather than a two-net
    bus dictating terms to an eleven-net one.
    """
    out: Dict[str, str] = {}
    placed: List[List[Pt]] = []
    for bus in sorted(buses, key=len, reverse=True):
        scored = []
        for d in ('left', 'right', 'up', 'down'):
            ok, why = certify(menu, bus, d)
            if not ok:
                if log:
                    log(f'  bus[{len(bus)}] {d}: REFUSED -- {why}')
                continue
            pick, s = {}, 0.0
            for n in bus:
                m = min((m for m in menu[n] if m.direction == d),
                        key=lambda m: cost_fn(n, m))
                pick[n] = m
                s += cost_fn(n, m)
            xs = 0
            if geo is not None and placed:
                for n in bus:
                    leg = geo.leg(n, pick[n])
                    xs += sum(1 for other in placed
                              if geo.paths_cross(leg, other))
            scored.append((s + cross_weight * xs, d, why, xs, pick))
        if not scored:
            continue
        scored.sort(key=lambda r: r[0])
        s, best, why, xs, pick = scored[0]
        if log:
            alt = ', '.join(f'{d}:{c:.0f}(+{x} cross)'
                            for c, d, _w, x, _p in scored[1:])
            log(f'  bus[{len(bus)}] -> {best} ({why}, cost {s:.0f}, '
                f'{xs} crossings into placed corridors)'
                + (f'   over {alt}' if alt else ''))
        for n in bus:
            out[n] = best
        if geo is not None:
            placed.extend(geo.leg(n, pick[n]) for n in bus)
    return out


def _other(layer: str, layers=('F.Cu', 'B.Cu')) -> str:
    return layers[1] if layer == layers[0] else layers[0]


def _proper_cross(p1: Pt, p2: Pt, p3: Pt, p4: Pt) -> bool:
    """Do segments p1p2 and p3p4 properly intersect? Shared endpoints
    and collinear touching do not count."""
    def d(a, b, c):
        return ((b[0] - a[0]) * (c[1] - a[1])
                - (b[1] - a[1]) * (c[0] - a[0]))
    d1, d2 = d(p3, p4, p1), d(p3, p4, p2)
    d3, d4 = d(p1, p2, p3), d(p1, p2, p4)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


class Corridor:
    """How a corridor is ordered, and which of its nets can stay put.

    The via floor is 2*(K - a), where `a` is the largest set of legs
    that pairwise do not cross: everyone else has to dive and come back.
    Reading that off a 1-D permutation is only valid when a coordinate
    exists that really does order the crossings, and the obvious
    candidates were each measured wrong on this bench:

      launch y / exit axis  assumes the launches are a VERTICAL comb.
                            The `down` corridor launches from a
                            horizontal comb south-west of the array,
                            every member within 0.3 mm of one y, so
                            ordering by y is a coin flip -- it called
                            9 of 28 pairs crossing where 3 do.
      angle about the array assumes the bundle WRAPS the array. That
                            corridor approaches the bottom edge head-on
                            from the south and wraps nothing: 6 pairs
                            predicted, 5 of them wrong.

    So the projection is used only to PROPOSE an order -- the transverse
    axis of the bundle's own mean travel, which needs no comb
    orientation and no side cases, and which measured best of the five
    tried (2 wrong of 28 on `down`, 0 of 55 on `left`, 0 of 1 on `up`).
    The proposal is then CHECKED against the drawn legs and any pair
    that really crosses is dropped. So the kept set is always genuinely
    non-crossing, and the floor it gives is never optimistic -- which is
    the failure that mattered, a floor of 0 reported for four nets piled
    on one exit point.
    """

    def __init__(self, box, launch: Dict[str, Pt], pad: float = 0.3,
                 cache: Optional[Dict[str, Dict]] = None):
        self.box = box
        self.pad = pad
        self.launch = launch
        self.bands = bands_of_boxes(box) if box else []
        # the caches key on the LAUNCH point as well as the exit, so a
        # caller that rebuilds the frame with one net moved -- which is
        # every step of a source-side search -- keeps the other nets'
        # work instead of paying O(N^2) crossings again per candidate
        if cache is None:
            cache = {}
        self._legs = cache.setdefault('legs', {})
        self._x = cache.setdefault('x', {})

    def leg(self, n: str, m) -> List[Pt]:
        """The polyline the corridor must draw for this net: launch to
        the escape's exit, around the array rather than through it."""
        pt = m if isinstance(m, tuple) else m.exit_pt
        lp = self.launch[n]
        key = (n, round(lp[0], 4), round(lp[1], 4),
               round(pt[0], 4), round(pt[1], 4), self.pad)
        hit = self._legs.get(key)
        if hit is None:
            bi = band_of(pt, self.bands) if self.bands else None
            if bi is not None:
                hit = band_leg(self.launch[n], pt, self.bands[bi])
            else:
                hit = around_box_path(self.launch[n], pt, self.box, self.pad)
            self._legs[key] = hit
        return hit

    def axis(self, grp: Sequence[str], sel: Dict[str, Move]) -> Pt:
        """Unit vector ACROSS the bundle -- perpendicular to where it is
        on average going."""
        dx = dy = 0.0
        for n in grp:
            dx += sel[n].exit_pt[0] - self.launch[n][0]
            dy += sel[n].exit_pt[1] - self.launch[n][1]
        h = math.hypot(dx, dy) or 1.0
        return (-dy / h, dx / h)

    def launch_key(self, n: str, t: Pt) -> float:
        return self.launch[n][0] * t[0] + self.launch[n][1] * t[1]

    def exit_key(self, n: str, m, t: Pt) -> float:
        pt = m if isinstance(m, tuple) else m.exit_pt
        if self.bands:
            # a band exit's place in the target order is its NESTED offset
            # at the band's mouth (band_leg), not its stub's: every stub on
            # one line projects alike, and an order by launch among them
            # hid the nesting the copper must keep
            bi = band_of(pt, self.bands)
            if bi is not None:
                pt = band_leg(self.launch[n], pt, self.bands[bi])[1]
        return pt[0] * t[0] + pt[1] * t[1]

    def order(self, grp: Sequence[str], sel: Dict[str, Move],
              t: Optional[Pt] = None) -> List[str]:
        t = t or self.axis(grp, sel)
        return sorted(grp, key=lambda n: self.launch_key(n, t))

    @staticmethod

    def paths_cross(pa: Sequence[Pt], pb: Sequence[Pt]) -> bool:
        return any(_proper_cross(p, q, r, s)
                   for p, q in zip(pa, pa[1:])
                   for r, s in zip(pb, pb[1:]))


    def crosses(self, a: str, b: str, sel: Dict[str, Move]) -> bool:
        ea, eb = sel[a].exit_pt, sel[b].exit_pt
        la, lb = self.launch[a], self.launch[b]
        key = (a, round(la[0], 4), round(la[1], 4),
               round(ea[0], 4), round(ea[1], 4),
               b, round(lb[0], 4), round(lb[1], 4),
               round(eb[0], 4), round(eb[1], 4), self.pad)
        hit = self._x.get(key)
        if hit is None:
            hit = self.paths_cross(self.leg(a, sel[a]), self.leg(b, sel[b]))
            self._x[key] = hit
        return hit

    def keep(self, grp: Sequence[str], sel: Dict[str, Move],
             weight: Optional[Dict[str, float]] = None) -> List[str]:
        """The nets that can travel the corridor without diving: the
        proposal from the transverse order, pruned until no two of them
        actually cross."""
        if len(grp) < 2:
            return list(grp)
        t = self.axis(grp, sel)
        lo = self.order(grp, sel, t)
        li = {n: i for i, n in enumerate(lo)}
        tgt = sorted(grp, key=lambda n: (round(self.exit_key(n, sel[n], t),
                                               6), li[n]))
        tr = {n: i for i, n in enumerate(tgt)}
        ranks = [tr[n] for n in lo]
        if weight:
            idx = lis_keep_weighted(ranks, [weight.get(n, 0.0)
                                                for n in lo])
        else:
            idx = lis_keep(ranks)
        kept = [lo[i] for i in sorted(idx)]
        # prune: drop the worst offender until the set really is
        # crossing-free. The projection is a proposal, not a proof.
        while True:
            bad: Dict[str, int] = {}
            for i, a in enumerate(kept):
                for b in kept[i + 1:]:
                    if self.crosses(a, b, sel):
                        bad[a] = bad.get(a, 0) + 1
                        bad[b] = bad.get(b, 0) + 1
            if not bad:
                return kept
            worst = max(bad, key=lambda n: (bad[n], -(weight or {}).get(n, 0)))
            kept = [n for n in kept if n != worst]


def corridor_groups(choice: Dict[str, Move]) -> List[List[str]]:
    """The unit a corridor actually routes: EVERY net leaving on one
    side, not one taut-path cluster.

    Clusters answer "which nets are going the same way", which is the
    right question for CHOOSING a side. But two clusters that pick the
    same side share one channel and one permutation, so their mutual
    crossings are real and a per-cluster floor cannot see them. Measured
    at K21: three clusters (5/4/2 nets) all left, 23 crossings between
    them, and the summed per-cluster floor understated the truth by 8."""
    g: Dict[str, List[str]] = {}
    for n, m in choice.items():
        g.setdefault(m.direction, []).append(n)
    return [v for _k, v in sorted(g.items())]


def delivered_layers(choice: Dict[str, Move], groups, geo: 'Corridor',
                     tooth_layer: Dict[str, str]) -> Dict[str, str]:
    """The layer the corridor hands each net over on: its tooth layer
    if the permutation makes it a keeper, the other one if it must
    dive."""
    out: Dict[str, str] = {}
    for bus in groups:
        if not all(n in choice for n in bus):
            continue
        # among the equally-good keep sets, take the one that holds on
        # to the nets whose escape starts on their TOOTH layer -- those
        # are the ones that pair for free with staying put
        w = {n: (1.0 if choice[n].layer == tooth_layer.get(n, 'F.Cu')
                 else 0.0) for n in bus}
        kept = set(geo.keep(bus, choice, w))
        for n in bus:
            L = tooth_layer.get(n, 'F.Cu')
            out[n] = L if n in kept else _other(L)
    return out


def true_vias(choice: Dict[str, Move], groups, geo: 'Corridor',
              tooth_layer: Dict[str, str]) -> int:
    """The vias a route ACTUALLY needs, per net:

        1 if the corridor makes it dive at the tooth
      + 1 if the layer it is handed over on is not the one its escape
          starts on
      + the escape's own vias

    which gives 0 for a keeper taking a surface escape and 2 for
    everything else -- an aligned diver's dive and its escape's surface
    via being the SAME two, not four. Counting escape vias and the
    corridor floor as independent totals double-counts exactly that
    merge, and would score an aligned plan as though nothing had been
    saved."""
    dl = delivered_layers(choice, groups, geo, tooth_layer)
    n_v = 0
    for n, m in choice.items():
        if n not in dl:
            continue
        n_v += 1 if dl[n] != tooth_layer.get(n, 'F.Cu') else 0
        n_v += 1 if dl[n] != m.layer else 0
        n_v += m.vias
    return n_v


def score(choice: Dict[str, Move], groups, geo: 'Corridor',
          tooth_layer: Dict[str, str]) -> Tuple[int, int, int]:
    """(true vias, corridor via floor, layer mismatches). The first is
    what a round is judged on; the others are reported to show where it
    came from."""
    tv = true_vias(choice, groups, geo, tooth_layer)
    fl = sum(_floor(b, choice, geo) for b in groups
             if len(b) >= 2 and all(n in choice for n in b))
    dl = delivered_layers(choice, groups, geo, tooth_layer)
    mm = sum(1 for n, m in choice.items()
             if dl.get(n) and dl[n] != m.layer)
    return tv, fl, mm




_TOUCH = 1e-6        # two spans that meet at a point DO conflict (see below)


def _conflict(m: Move, om: Move, tol: float = 0.16, strict: bool = True) -> bool:
    """Two moves that cannot both be laid: a shared lane stretch or a
    shared site -- and, `strict`, a lane matched within `tol` (half a
    fine-pitch gap) or one's via site in the other's lane. The strict
    form is the SOURCE refinement's: it re-fans real moves against stubs
    already on the board, which wander a few tens of microns off their
    gap's centreline, and a dogbone's via is on every layer (K15
    SDQM0/SDQ15: an F surface escape planned through a B dogbone's gap,
    9 grazes). The DESTINATION choice is not strict: the fanout takes
    only the plan's DIRECTION from it, never its move geometry, so the
    strict test only removes moves the plan was free to use -- applied
    there it took the restricted K19 plan from floor 12 to 20, moved
    SDQ0 from the south face to the west, split the corridor in two and
    left 5 lanes open (2026-08-30, measured after the fact: the ladder
    had been run on fanout boards recorded before the change)."""
    spans, ospans = _lane_spans(m), _lane_spans(om)
    for key, a, b in spans:
        for ok, oa, ob in ospans:
            if strict:
                same_lane = (ok[0] == key[0] and ok[2] == key[2]
                             and abs(ok[1] - key[1]) < tol)
            else:
                same_lane = ok == key
            # TOUCHING IS OVERLAPPING. Strict `<` let two stubs that meet
            # at exactly one point pass: two vertically adjacent balls both
            # stubbing half a pitch into their shared row gap and then
            # diverging left/right share a lane key and abut at the ball's
            # x, so `oa < b` was `x < x` -> False and BOTH were selected --
            # a planned dead short between two nets (measured on DU1
            # column x=138.7286, GND and VCC-DRAM meeting at (138.7286,
            # 61.7608) on F.Cu).
            if same_lane and a < ob + _TOUCH and oa < b + _TOUCH:
                return True
            # a row-gap run and a column-gap run on ONE layer that cross:
            # two stubs through one point (K28 dv3: SWE's walked leg west
            # along row 66.56 and SCKE0's through-run down column 140.73,
            # the second refused by the engine, the plan blind to it)
            if ok[0] != key[0] and ok[2] == key[2] and (
                    SEL_XING >= 2 or (SEL_XING and (
                        getattr(m, 'walk', 0) or getattr(om, 'walk', 0)
                        or getattr(m, 'climb', 0) or getattr(om, 'climb', 0)))):
                (rk, ra, rb), (ck, ca, cb) = ((key, a, b), (ok, oa, ob)) if key[0] == 'row' \
                    else ((ok, oa, ob), (key, a, b))
                if ra - tol < ck[1] < rb + tol and ca - tol < rk[1] < cb + tol:
                    return True
    if m.site is not None and _site_key(om) == _site_key(m):
        return True
    # a dog-bone's via spans every layer: if it sits in the other
    # move's gap, inside the stretch that move runs along it, the two
    # cannot both be laid whatever their layers (K28: SODT0's site in
    # SCKE0's column gap, SDQ0's in SDQ14's row gap -- the fanout, which
    # now lays the plan's moves exactly, refused the second of each pair)
    if any(_site_in_lane(om, key, a, b) for key, a, b in spans) \
            or any(_site_in_lane(m, ok, oa, ob) for ok, oa, ob in ospans):
        return True
    # two teeth cannot share one exit point, whatever their layers: the
    # braid orders lanes by their offset at the array, and two lanes at
    # one offset have no pitch between them (K28: SA6 on F and SBA1 on B
    # at DU1's (146.35, 62.56), a corridor of two, both refused)
    if (abs(m.exit_pt[0] - om.exit_pt[0]) < _EXIT_TOL
            and abs(m.exit_pt[1] - om.exit_pt[1]) < _EXIT_TOL):
        return True
    return False


_EXIT_TOL = 0.16    # half a fine-pitch gap


_VIA_REACH = 0.30   # via radius + clearance + half a track, rounded up


def _site_in_lane(dm: Move, key, a, b) -> bool:
    """Does dm's dog-bone via (any layer) sit inside the lane `key`
    over [a, b]?"""
    if dm.kind != 'dogbone' or dm.site is None:
        return False
    sx, sy = dm.site
    if key[0] == 'row':
        return abs(sy - key[1]) < _VIA_REACH and a - _VIA_REACH < sx < b + _VIA_REACH
    return abs(sx - key[1]) < _VIA_REACH and a - _VIA_REACH < sy < b + _VIA_REACH


def band_room(m: Move, others, bands) -> bool:
    """Is there room in the band for this move's lane on its layer? A
    band's capacity per layer is what fits between its stub-tip lines at
    the block pitch (band_capacity); a move whose exit is not in a band
    always has room."""
    if not bands:
        return True
    bi = band_of(m.exit_pt, bands)
    if bi is None:
        return True
    n = sum(1 for om in others
            if om.layer == m.layer and band_of(om.exit_pt, bands) == bi)
    return n < band_capacity(bands[bi])


def lanes_free(m: Move, sel: Dict[str, Move], me: str,
               strict: bool = True, bands=()) -> bool:
    """Is this move's channel and via site free, ignoring the net's own
    current claim? The same check select() applies inside its greedy
    pass (non-strict there), exposed so a later refinement cannot
    quietly propose a move that two nets would have to share. `bands`:
    the destination's bands, whose capacity the move must also fit."""
    if any(_conflict(m, om, strict=strict) for other, om in sel.items()
           if other != me):
        return False
    return band_room(m, [om for other, om in sel.items() if other != me], bands)


def plan_floor(sel: Dict[str, Move], geo: 'Corridor') -> int:
    """The via floor for the WHOLE plan, corridor boundaries ignored.

    A per-corridor floor prices only the crossings inside a corridor,
    which makes any change that pushes crossings across a corridor
    boundary look free. It is not: the board is one surface, and the
    corridor split is our decomposition for search, not an accounting
    boundary. Measured at K32, moving from no crossing pricing to
    cross_weight 6 cut total crossings 241 -> 112 while the per-corridor
    via count ROSE 37 -> 45, purely because the crossings it removed
    were the ones nobody was charging for."""
    nets = list(sel)
    return 2 * (len(nets) - len(geo.keep(nets, sel)))


def _floor(bus: Sequence[str], sel: Dict[str, Move],
           geo: 'Corridor') -> int:
    """The corridor's via floor: 2 vias for every net that cannot stay
    on the layer it arrives on, i.e. everything outside the largest
    crossing-free set."""
    return 2 * (len(bus) - len(geo.keep(bus, sel)))


def refine_lis(choice: Dict[str, Move], groups, menu, geo: 'Corridor',
               free: Callable[[Move, Dict[str, Move], str], bool],
               rounds: int = 6, prefer_layer=None,
               log=None) -> Dict[str, Move]:
    """Choose exit coordinates to MAXIMISE the corridor's LIS, exactly.

    The corridor's via floor is 2*(K - LIS) of the launch->exit
    permutation, and the floor depends only on the ORDER of the exits.
    So walk the bus in launch order and solve for the longest
    non-decreasing chain of exit coordinates by dynamic programming --
    dp[c] = longest chain ending at coordinate c -- which is optimal
    for this objective. Hill-climbing on single-net flips was tried
    first and stalls: reaching the best assignment needs several nets
    to move together, so it sat at the floor it started from.

    The chain runs over SLOTS -- (exit line, layer) -- and must increase
    STRICTLY through them, which is what makes the answer physical. With
    a merely non-decreasing chain over exit coordinates the DP has a
    trivial optimum: put every net on ONE exit line, where all the ties
    count as ordered, and report LIS = K and a floor of 0. It did
    exactly that, and nothing downstream objected, because the chain
    nets were also applied WITHOUT the channel check the other nets go
    through. Measured at K21: four nets of the `down` corridor assigned
    to the single point (140.73, 68.16), and a floor of 0 that no route
    could ever realise. Strict slot order caps a line at one net per
    layer, and the chain is now applied through `free` like everything
    else, so an unrealisable chain loses its members instead of being
    reported as an achievement.
    """
    for bus in groups:
        if len(bus) < 3 or any(n not in choice for n in bus):
            continue
        side = choice[bus[0]].direction
        t = geo.axis(bus, choice)
        lo = geo.order(bus, choice, t)
        opts = {}
        for n in lo:
            seen, keep = {}, []
            # Collapse each SLOT to one representative -- but rank a
            # move that starts on the layer the corridor will deliver
            # FIRST. Ranking purely by via count kept the 0-via surface
            # move at every coordinate and threw the dive escape away,
            # which silently undid the layer alignment: SA7 had a
            # matching dogbone at cost 27.5 against its 29.0 surface,
            # and never got to keep it.
            pl = (prefer_layer or {}).get(n)

            def _rank(m, _pl=pl):
                return (0 if _pl and m.layer == _pl else 1,
                        m.vias, _length(m))

            for m in sorted(menu[n], key=_rank):
                if m.direction != side:
                    continue
                s = (round(geo.exit_key(n, m, t), 6), m.layer)
                if s not in seen:
                    seen[s] = m
                    keep.append((s, m))
            opts[n] = sorted(keep)
        if not all(opts[n] for n in lo):
            continue
        # dp over nets in launch order: longest STRICTLY increasing
        # chain of slots
        best_end = {}          # slot -> (len, net index, move)
        back = {}
        for i, n in enumerate(lo):
            cur = {}
            for s, m in opts[n]:
                bl, bi, bs = 0, None, None
                for s2, (l2, i2, _m2) in best_end.items():
                    if s2 < s and l2 > bl:
                        bl, bi, bs = l2, i2, s2
                cur[s] = (bl + 1, i, m)
                back[(i, s)] = (bi, bs)
            for s, v in cur.items():
                if s not in best_end or v[0] > best_end[s][0]:
                    best_end[s] = v
        if not best_end:
            continue
        ends = max(best_end, key=lambda s: best_end[s][0])
        chain = {}
        i, s = best_end[ends][1], ends
        while i is not None:
            chain[lo[i]] = dict(opts[lo[i]])[s]
            i, s = back[(i, s)]
        # apply: chain first, then the rest -- but every net through the
        # SAME channel check, chain members included
        trial = {n: m for n, m in choice.items() if n not in bus}
        for n in lo:
            if n in chain and free(chain[n], trial, n):
                trial[n] = chain[n]
        for n in lo:
            if n in trial:
                continue
            for s, m in opts[n]:
                if free(m, trial, n):
                    trial[n] = m
                    break
            else:
                # nothing free: keep what it had -- IF that is still free
                # against the others' new slots. Kept unchecked, it shared a
                # gap with a chain member (K41: SCKE1's through-run and
                # SRAS's stub both at 139.93, every pass refusing one), the
                # one conflict the greedy's own pass never makes.
                if not free(choice[n], trial, n):
                    trial = None
                    break
                trial[n] = choice[n]
        if trial is None or len(trial) != len(choice):
            continue
        before = _floor(bus, choice, geo)
        after = _floor(bus, trial, geo)
        if after < before:
            for n in bus:
                choice[n] = trial[n]
            if log:
                log(f'  LIS: bus[{len(bus)}] floor {before} -> {after}')
    return choice


_FLIP_FACE = {'up': 'down', 'down': 'up', 'left': 'left', 'right': 'right'}


def pair_chirality(src_pads: Dict[str, Pt], dst_pads: Dict[str, Pt],
                   src_box, dst_box) -> int:
    """+1 or -1: which side of the source -> destination axis the PAIR's
    own BALLS lie on -- the run's nets' pads on both arrays, nothing else
    on the board -- as the sign of their moment about it. The selector
    is handed (menu and face order, sort ties, a quarter-turn axis, a
    crossing test that counts a shared endpoint on one side only), so a
    board and its mirror got different plans; the pair's chirality flips
    exactly under the mirror (+1395 against -1395 on the origin board
    and the board turned over at K28), so the selector runs every pair
    in its +1 frame (PairFrame) and the mirror gets the mirror of the
    plan. Balls, not teeth: the teeth move as the plan's rounds re-fan
    the source, and at K15 the teeth's moment changed sign at round 1
    (+373 -> -156) and mirrored the bench against itself mid-loop; the
    balls are the same on every realized board. A pair whose balls
    balance exactly is +1 by convention. Read off the pair alone so a
    board with three or more arrays gives every pair its own frame."""
    sc = ((src_box[0] + src_box[2]) / 2, (src_box[1] + src_box[3]) / 2)
    dc = ((dst_box[0] + dst_box[2]) / 2, (dst_box[1] + dst_box[3]) / 2)
    ax, ay = dc[0] - sc[0], dc[1] - sc[1]
    pts = list(src_pads.values()) + list(dst_pads.values())
    mom = sum(ax * (y - sc[1]) - ay * (x - sc[0]) for x, y in pts)
    return -1 if mom < 0 else 1


_DIRS_ORDER = ('left', 'right', 'up', 'down')
_KIND_ORDER = {'surface': 0, 'via_in_pad': 1, 'dogbone': 2}


def ms_sites(moves, fr=None):
    """The centre of a net's dog-bone sites (its ball): the sign of a
    site about it is the generator's site order."""
    pts = [(fr.pt(m.site) if fr else m.site) for m in moves
           if m.kind == 'dogbone' and m.site is not None]
    if not pts:
        return (0.0, 0.0)
    return (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))


def menu_order(m: Move, ball: Pt, layers=('F.Cu', 'B.Cu')) -> tuple:
    """escape_moves.menu's own order, read off a move's geometry:
    surface moves by face then by the gap's coordinate along it, via-
    in-pad by layer then face, dog-bones by the site's signs about the
    ball then face then layer. Verified equal to the generated order on
    every net of the bench (K15..K51), the origin board and the zynq
    article, so a mirrored menu re-sorted by it is the front's menu."""
    d = _DIRS_ORDER.index(m.direction) if m.direction in _DIRS_ORDER else 9
    along = m.exit_pt[1] if m.direction in ('left', 'right') else m.exit_pt[0]
    L = layers.index(m.layer) if m.layer in layers else 9
    k = _KIND_ORDER.get(m.kind, 3)
    if k == 0:
        return (0, d, round(along, 6))
    # a climb (escape_moves climb=) shares its kind's key with the plain
    # move from the same site: the exit's coordinate along the face
    # splits the tie (a plain move is alone in its group, so its order
    # is unchanged)
    if k == 1:
        return (1, L, d, round(along, 6))

    def sgn(v):
        return -1 if v < -1e-6 else (1 if v > 1e-6 else 0)
    sx = sgn(m.site[0] - ball[0]) if m.site else 0
    sy = sgn(m.site[1] - ball[1]) if m.site else 0
    return (2, sx, sy, d, L, round(along, 6))


def frame_line(launch, keep_out, pads=None) -> float:
    """The y of the pair's mirror line: the middle of the pair's own
    points' extent (the run's launches, berth pads and the destination
    box), on the 0.0005 mm lattice so twice it has three decimals and a
    mirrored coordinate keeps its decimals -- the selector rounds
    absolute coordinates in places, and a line that translated the
    mirrored geometry off the front's decimal grid flipped a few of
    those roundings (4 of 28 choices)."""
    ys = [p[1] for p in launch.values()]
    for bx in _boxes(keep_out):
        ys += [bx[1], bx[3]]
    if pads:
        ys += [p[1] for p in pads.values()]
    c = (min(ys) + max(ys)) / 2
    return round(c * 2000.0) / 2000.0


class PairFrame:
    """The pair's canonical handed frame: +1 is the identity; -1 mirrors
    every point about the horizontal line y = CY and swaps up and down
    (layers stay: the selector only ever compares them). Moves are
    mirrored into new Move objects and mapped back by identity."""

    def __init__(self, chi: int, CY: float):
        self.chi, self.CY = chi, CY
        self._back: Dict[int, Move] = {}
        self._fwd: Dict[int, Move] = {}

    def pt(self, p: Pt) -> Pt:
        return p if self.chi > 0 else (p[0], 2 * self.CY - p[1])

    def box(self, b):
        if self.chi > 0 or b is None:
            return b
        if b and isinstance(b[0], (tuple, list)):
            return [self.box(bb) for bb in b]
        return (b[0], 2 * self.CY - b[3], b[2], 2 * self.CY - b[1])

    @staticmethod
    def layer(L):
        """F.* <-> B.*: the turn-over swaps faces, and the selector sorts
        slots by layer NAME, so the names must swap with the geometry."""
        if isinstance(L, str) and len(L) > 1 and L[1] == '.' and L[0] in 'FB':
            return ('B' if L[0] == 'F' else 'F') + L[1:]
        return L

    def layers(self, d):
        return d if self.chi > 0 or d is None else {n: self.layer(L) for n, L in d.items()}

    def move(self, m: Move) -> Move:
        if self.chi > 0:
            return m
        mm = self._fwd.get(id(m))
        if mm is None:
            mm = Move(net=m.net, kind=m.kind,
                      direction=_FLIP_FACE.get(m.direction, m.direction),
                      layer=self.layer(m.layer), exit_pt=self.pt(m.exit_pt), vias=m.vias,
                      legs=[(self.pt(a), self.pt(b), self.layer(L)) for (a, b, L) in m.legs],
                      site=None if m.site is None else self.pt(m.site),
                      # walk and off_array travel with the move too. Losing
                      # them made a mirrored board a DIFFERENT PROBLEM:
                      # _lane_spans took the single-span path (the walk leg
                      # and the elbow's crossing spans vanished from the
                      # conflict test), _conflict's SEL_XING gate never
                      # fired, and _select's SEL_EXT seeding -- which
                      # detects a walked menu with any(m.walk) -- silently
                      # switched off. A board and its mirror ran different
                      # selection algorithms, against a class whose whole
                      # purpose is that they must not.
                      walk=getattr(m, 'walk', 0),
                      off_array=getattr(m, 'off_array', False),
                      climb=getattr(m, 'climb', 0))
            self._fwd[id(m)] = mm
            self._back[id(mm)] = m
        return mm

    def back(self, mm: Move) -> Move:
        return mm if self.chi > 0 else self._back[id(mm)]

    def menu(self, menu):
        """The menus mirrored AND re-ordered as the generator orders
        them (menu_order): the selector breaks ties by list order, and a
        mirror alone leaves the mirror's own order."""
        if self.chi > 0:
            return menu
        return {n: sorted((self.move(m) for m in ms), key=lambda mm: menu_order(mm, ms_sites(ms, self)))
                for n, ms in menu.items()}

    def points(self, d):
        return d if self.chi > 0 or d is None else {n: self.pt(p) for n, p in d.items()}

    def choice(self, ch):
        return ch if self.chi > 0 or ch is None else {n: self.move(m) for n, m in ch.items()}

    def choice_back(self, ch):
        return ch if self.chi > 0 or ch is None else {n: self.back(m) for n, m in ch.items()}


def select(menu: Dict[str, List[Move]],
           launch: Dict[str, Pt],
           via_weight: float = 3.0,
           channel_weight: float = 2.0,
           keep_out=None,
           buses: Optional[Sequence[Sequence[str]]] = None,
           side_weight: float = 6.0,
           tooth_layer: Optional[Dict[str, str]] = None,
           mismatch_weight: float = 4.0,
           cross_weight: float = 6.0,
           align_rounds: int = 4,
           log=None,
           pads: Optional[Dict[str, Pt]] = None,
           chi: int = 1,
           ) -> Tuple[Dict[str, Move], List[str]]:
    """`_select` in the pair's canonical frame (pair_chirality): a -1
    pair has its menus, launches, box and pads mirrored in, the handed
    selection run unchanged, and the chosen moves mapped back."""
    if chi > 0 or not keep_out:
        return _select(menu, launch, via_weight, channel_weight, keep_out, buses,
                       side_weight, tooth_layer, mismatch_weight, cross_weight,
                       align_rounds, log, pads)
    fr = PairFrame(-1, frame_line(launch, keep_out, pads))
    ch, un = _select(fr.menu(menu), fr.points(launch), via_weight, channel_weight,
                     fr.box(keep_out), buses, side_weight, fr.layers(tooth_layer),
                     mismatch_weight, cross_weight, align_rounds, log, fr.points(pads))
    return fr.choice_back(ch), un


def _select(menu: Dict[str, List[Move]],
           launch: Dict[str, Pt],
           via_weight: float = 3.0,
           channel_weight: float = 2.0,
           keep_out=None,
           buses: Optional[Sequence[Sequence[str]]] = None,
           side_weight: float = 6.0,
           tooth_layer: Optional[Dict[str, str]] = None,
           mismatch_weight: float = 4.0,
           # 6.0 measured over the whole coherent ladder: it is the
           # only value tried that improves the WHOLE-PLAN floor at
           # every checkpoint (K11 12->10, K21 18->16, K32 46->38,
           # K47 70->62, K51 72->64). 12.0 is better at K32 and a
           # wash at K11; 24.0 regresses at K51.
           cross_weight: float = 6.0,
           align_rounds: int = 4,
           log=None,
           pads: Optional[Dict[str, Pt]] = None,
           ) -> Tuple[Dict[str, Move], List[str]]:
    """Pick one move per net. `launch[n]` is where the net enters the
    corridor, used to price how far the corridor must carry it to reach
    a move's exit point. `geo`: the order model to use for the floor,
    the LIS refinement and the layer alignment (the retired plan_order.BraidOrder,
    the braid's own rules) instead of this module's projection.
    Returns (choice, unplaced)."""
    if SEL_EXT and any(getattr(m, 'walk', 0) for ms in menu.values() for m in ms):
        menu = {n: ([m for m in ms if not getattr(m, 'walk', 0)] or list(ms))
                for n, ms in menu.items()}
    cand = {n: list(ms) for n, ms in menu.items()}
    geo = Corridor(keep_out, launch) if keep_out else None

    contend_all = (em.site_contention(menu, VIA_NEED_SITE)
                   if SEL_CONTEND else {})

    def cost(n: str, m: Move) -> float:
        contend = contend_all.get(n)
        lx, ly = launch[n]
        if keep_out is None:
            reach = math.hypot(m.exit_pt[0] - lx, m.exit_pt[1] - ly)
        else:
            # the corridor cannot cross the array: measure the reach it
            # would actually have to travel
            reach = around_box((lx, ly), m.exit_pt, keep_out)
        # the move's own run occupies a channel INSIDE the array, which
        # is scarcer than corridor length -- weight it above `reach`.
        # A BAND exit's lane runs the band from its mouth to the exit's
        # column: that run is a channel inside the array too (the band
        # holds a few lanes a layer), priced the same way -- unpriced,
        # the band read as the cheapest move on the menu for every ball
        # that faces it, and the greedy filled it to capacity with the
        # deepest balls (K28: 7 band berths, 9 in-band refusals, 50 vias)
        chan = _length(m)
        if BAND_CHAN and geo is not None and geo.bands:
            bi = band_of(m.exit_pt, geo.bands)
            if bi is not None:
                leg = band_leg((lx, ly), m.exit_pt, geo.bands[bi])
                chan += math.hypot(leg[2][0] - leg[1][0], leg[2][1] - leg[1][1])
        c_ = via_weight * m.vias + channel_weight * chan + reach
        # ...and the room the BARREL takes (SEL_CONTEND). The run above is
        # charged for the channel it occupies; the via was free wherever it
        # sat, so a dog-bone deep in the ball field -- whose site 6 other
        # escapes wanted -- cost the same as one outside the array that 2
        # wanted. Priced in VIAS per contender and converted at via_weight,
        # so it is on the same scale as the rest of this sum.
        if SEL_CONTEND and contend and m.site:
            c_ += via_weight * SEL_CONTEND * contend.get(
                (round(m.site[0], 3), round(m.site[1], 3)), 0)
        return c_

    # a bus enters the destination on ONE side, certified for capacity
    # before it is committed there; deviating from it means crossing
    # your own bundle, which nothing else in this cost sees
    side = (bus_sides(menu, launch, buses, cost, geo=geo,
                      cross_weight=cross_weight, log=log)
            if buses else {})
    # filled by the alignment loop below; empty on the first pass, so
    # the penalty is inert until there is a diver set to align with
    want_layer: Dict[str, str] = {}

    # legs of the nets chosen so far, so a candidate can be charged for
    # the corridors it would cut through. Pricing this only at the BUS
    # level is not enough: bus_sides stopped sending anyone `up`, and
    # three nets then deviated there one at a time inside the greedy
    # pass -- each deviation cheap on its own, 38 inter-corridor
    # crossings between them.
    placed_legs: List[List[Pt]] = []
    placed_nets: List[Tuple[str, Move]] = []      # in step with placed_legs

    def must_share(n: str, m: Move, o: str, om: Move) -> bool:
        """Do these two lanes have to share one layer end to end?"""
        tl = (tooth_layer or {})
        return (tl.get(n, 'F.Cu') == m.layer and tl.get(o, 'F.Cu') == om.layer
                and m.layer == om.layer)

    def total(n: str, m: Move) -> float:
        c = cost(n, m)
        if side.get(n) and m.direction != side[n]:
            c += side_weight
        # prefer an escape that starts on the layer the corridor will
        # actually hand this net over on; a mismatch costs a via
        if want_layer.get(n) and m.layer != want_layer[n]:
            c += mismatch_weight
        if geo is not None and placed_legs and cross_weight:
            leg = geo.leg(n, m)
            if SEL_XLAYER:
                for (o, om), ol in zip(placed_nets, placed_legs):
                    if geo.paths_cross(leg, ol):
                        c += cross_weight * (1.0 if must_share(n, m, o, om) else XLAYER_FREE)
            else:
                c += cross_weight * sum(1 for o in placed_legs
                                        if geo.paths_cross(leg, o))
        return c

    taken: List[Move] = []          # the moves laid so far this pass
    bands = geo.bands if geo is not None else []

    def lane_free(m: Move) -> bool:
        if any(_conflict(m, om, strict=False) for om in taken):
            return False
        return band_room(m, taken, bands)

    choice: Dict[str, Move] = {}
    unplaced: List[str] = []
    # most constrained first: a net with few options must choose before
    # a net with many takes its only lane
    order = sorted(cand, key=lambda n: len(cand[n]))
    for _retry in range(SEL_RETRY + 1):
      if _retry:
        # SEL_RETRY (2026-09-10): the nets left without a free lane go
        # FIRST and the greedy runs again -- the count of candidates is a
        # poor proxy for how constrained a net is, and a net the pass
        # left unplanned reaches the engine with no ask (K35: SA0 and
        # SA9 unplaced in every pass, each pass's first refusals)
        if not unplaced:
            break
        if log:
            log(f'  retry {_retry}: {unplaced} first')
        order = list(unplaced) + [n for n in order if n not in unplaced]
        choice, unplaced, taken[:], placed_legs[:], placed_nets[:] = {}, [], [], [], []
      for n in order:
        best = None
        for m in sorted(cand[n], key=lambda m: total(n, m)):
            if not lane_free(m):
                continue
            best = m
            break
        if best is None and SEL_FORCE and cand[n]:
            # SEL_FORCE (2026-09-10): a net with no free lane is PLANNED all
            # the same, on the candidate that conflicts with the fewest
            # moves taken (then the cheapest), and marked. Left unplanned
            # it reached the engine with no ask, which escaped it in its
            # generic phase BEFORE the planned balls and took a planned gap
            # (K35: SA0's engine-chosen stub down 143.13, SBA1's asked lane,
            # 'infeasible even alone' -- the first refusal of every pass)
            best = min(cand[n], key=lambda m: (sum(1 for om in taken if _conflict(m, om, strict=False)),
                                               total(n, m)))
            best.forced = True
            if log:
                log(f'  {n}: no free lane -- forced onto {best} '
                    f'({sum(1 for om in taken if _conflict(best, om, strict=False))} conflict(s))')
        if best is None:
            unplaced.append(n)
            continue
        if log and side.get(n) and best.direction != side[n]:
            log(f'  {n}: leaves its bus side {side[n]} for '
                f'{best.direction} (no room on the bus side)')
        choice[n] = best
        if geo is not None:
            placed_legs.append(geo.leg(n, best))
            placed_nets.append((n, best))
        taken.append(best)
        if log:
            log(f'  {n}: {best}  (of {len(cand[n])} candidates)')

    def _free(m: Move, sel: Dict[str, Move], me: str) -> bool:
        """Is this move's channel free, ignoring the net's OWN current
        claim? The LIS pass swaps one net at a time -- without
        excluding `me`, a net is blocked from changing gap by the gap
        it is already sitting in."""
        return lanes_free(m, sel, me, strict=False, bands=bands)

    # From here on the grouping is the CORRIDOR -- every net leaving on
    # one side -- not the taut-path cluster. The cluster chose the side
    # (above); the permutation, the via floor and the diver set all
    # belong to whatever ends up sharing that one channel, and are
    # recomputed from `choice` because the greedy pass is allowed to
    # send a net off its bus side when the side is full.
    if buses and geo:
        refine_lis(choice, corridor_groups(choice), menu, geo, _free, log=log)

    # --- fixed point: exits decide the divers, divers decide the
    # preferred escape layer, which decides the exits
    if buses and tooth_layer and geo:
        best = dict(choice)
        best_s = score(best, corridor_groups(best), geo, tooth_layer)
        if log:
            log(f'  align round 0: vias {best_s[0]}, floor {best_s[1]}, '
                f'mismatch {best_s[2]}')
        for r in range(align_rounds):
            want_layer.clear()
            want_layer.update(delivered_layers(choice, corridor_groups(choice),
                                               geo, tooth_layer))
            taken.clear()
            placed_legs.clear()
            trial: Dict[str, Move] = {}
            for n in order:
                pick = None
                for m in sorted(cand[n], key=lambda m: total(n, m)):
                    if not lane_free(m):
                        continue
                    pick = m
                    break
                if pick is None:
                    continue
                trial[n] = pick
                if geo is not None:
                    placed_legs.append(geo.leg(n, pick))
                taken.append(pick)
            if len(trial) < len(choice):
                break                      # lost a net: reject the round
            refine_lis(trial, corridor_groups(trial), menu, geo, _free,
                       prefer_layer=want_layer)
            s = score(trial, corridor_groups(trial), geo, tooth_layer)
            if log:
                log(f'  align round {r + 1}: vias {s[0]}, floor {s[1]}, '
                    f'mismatch {s[2]}')
            if s[0] < best_s[0]:        # judge on the true via count
                best, best_s = dict(trial), s
            if trial == choice:
                break
            choice = trial
        choice = best
        if log:
            log(f'  aligned: vias {best_s[0]}, floor {best_s[1]}, '
                f'mismatch {best_s[2]}')
    return choice, unplaced


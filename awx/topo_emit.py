#!/usr/bin/env python3
"""Braid-schedule emitter (v2) for the #622 topo-string experiment.

Pipeline: endpoints from the stub census; target order = BALL y; divers =
complement of the LIS of the launch->ball permutation; adjacent swaps
scheduled by bubbling divers one at a time in ascending target rank (each
diver's swaps form an exclusive x-band, so diver-diver swaps are
automatically exactly-one-on-B); trunk drawn as morphing lane slots.

Field entry, two modes:
 - F street: fanout-style run along the empty inter-row street, half-pitch
   jog to the ball. F entries must be strictly increasing in target order
   (tails cannot cross).
 - B dogbone: B.Cu under the field is EMPTY (all pads F-only SMD), so a
   net that cannot get a street enters on B and surfaces at a via in the
   diagonal cell next to its ball. For a diver this costs no extra vias
   (its surface via moves to the dogbone site; east-extending a window is
   safe: all swaps involving a diver live in its own or earlier bands).
   A promoted non-diver pays +2 vias. B entries keep their own monotone
   chain; in the trunk they ride a PHANTOM monotone lane and fan to the
   true entry on B east of the splice.

Emits copper (segments + vias) into a board copy for check_connected /
check_drc grading, plus an Eco2 overlay of the same geometry.
"""
import argparse
import math
import re
import os
import shutil
import sys
from collections import Counter

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402
import topo_strings as ts  # noqa: E402
import flow_frame as ff  # noqa: E402
import escape_moves as em  # noqa: E402

TRACK = 0.127
CLEAR = 0.105            # 0.1 spec + 5um so hugs don't sit exactly at 0.1
VIA_SIZE = 0.25
VIA_DRILL = 0.15


def build_obstacles(pcb, nid, kids, layer):
    obs = ts.Obstacles()
    m = CLEAR + TRACK / 2
    for ref, fp in pcb.footprints.items():
        for p in fp.pads:
            if p.net_id == nid:
                continue
            on_layer = any(L == layer or '*' in L for L in p.layers)
            if p.drill and p.drill > 0:
                on_layer = True
            if not on_layer:
                continue
            if p.pad_type == 'np_thru_hole' and p.drill:
                r0 = p.drill / 2
            elif p.shape in ('circle', 'oval'):
                r0 = max(p.size_x, p.size_y) / 2
            else:
                r0 = math.hypot(p.size_x, p.size_y) / 2
            obs.add_disc(p.global_x, p.global_y, r0 + m,
                         f'{ref}.{p.pad_number}')
    for s in pcb.segments:
        if s.net_id == nid or s.net_id in kids or s.layer != layer:
            continue
        obs.add_cap((s.start_x, s.start_y), (s.end_x, s.end_y),
                    s.width / 2 + m, f'seg:{s.net_id}')
    for v in pcb.vias:
        if v.net_id == nid:
            continue
        obs.add_disc(v.x, v.y, v.size / 2 + m, f'via:{v.net_id}')
    obs.build()
    return obs


def _owner(pt, segs, pads, pcb):
    """The component whose pad the net's copper connects `pt` to.

    Segment endpoints are graph nodes (vias join layers at one point,
    so a 2-D node graph is enough); a pad is reached when a node lies
    within its copper. Falls back to nearest-pad only when the walk
    reaches no pad at all (an isolated fragment)."""
    def key(x, y):
        return (round(x, 3), round(y, 3))
    adj = {}
    for s in segs:
        a, b = key(s.start_x, s.start_y), key(s.end_x, s.end_y)
        adj.setdefault(a, set()).add(b)
        adj.setdefault(b, set()).add(a)
    start = key(*pt)
    seen = {start}
    stack = [start]
    while stack:
        u = stack.pop()
        for v in adj.get(u, ()):
            if v not in seen:
                seen.add(v)
                stack.append(v)
    for p in pads:
        rx, ry = p.size_x / 2 + 0.02, p.size_y / 2 + 0.02
        for (x, y) in seen:
            if abs(x - p.global_x) <= rx and abs(y - p.global_y) <= ry:
                return p.component_ref
    p = min(pads, key=lambda q: ts.d2((q.global_x, q.global_y), pt))
    return p.component_ref


def endpoints(pcb, names, byname, dest_ref=None):
    """Where each net's braid must start and finish.

    Default (dest_ref None): the source stub's free end, to the BALL --
    the bench shape, where only the source is fanned out and the braid
    has to enter the destination's ball field itself.

    With dest_ref: the destination is fanned out too, so each net has a
    free end at BOTH ends and the braid's job is only the corridor
    between them. This is not a refinement of the default, it is the
    opposite of what it would do -- asked for a board fanned out at both
    ends, the default picks the DESTINATION stub end as `src` and a
    SOURCE pad as `tgt` (measured: 10 of 11 nets at K11), so the braid
    would run backwards, into the source's own 21x21 field. Silently:
    both are legal points and nothing downstream can tell.
    """
    ends = {}
    for nm in names:
        nid, net = byname[nm]
        segs = [s for s in pcb.segments if s.net_id == nid]
        cnt = Counter()
        for s in segs:
            cnt[(round(s.start_x, 3), round(s.start_y, 3))] += 1
            cnt[(round(s.end_x, 3), round(s.end_y, 3))] += 1
        # an endpoint inside a via barrel or a pad's copper is joined,
        # not free: the fanout starts a stub 25 um off the via centre
        # (K21: SRAS's via-in-pad at (140.329, 66.161), stub from
        # (140.329, 66.136)), and a 0.02 point tolerance read that as
        # a free end -- the braid then aimed at the ball, not the stub
        anchors = [(p.global_x, p.global_y, max(p.size_x, p.size_y) / 2)
                   for p in net.pads] + \
            [(v.x, v.y, v.size / 2) for v in pcb.vias if v.net_id == nid]
        free = [pt for pt, c in cnt.items() if c == 1 and
                all(math.hypot(pt[0] - ax, pt[1] - ay) > max(0.02, ar)
                    for (ax, ay, ar) in anchors)]
        assert free, (nm, 'no free stub end')
        if dest_ref is not None:
            # Which component does this free end BELONG to? Not the
            # nearest pad: a stub the fanout dragged across the whole
            # destination field (K15 chain: SA7, 11.6 mm from its
            # east-column ball to the west exit line) ends nearer the
            # SOURCE's pad than its own, so nearest-pad called it a
            # source tooth, x0 landed east of x1 and the trunk was
            # drawn in a corridor of width -0.5 mm. Walk the net's own
            # copper from the free end to the pad it reaches instead.
            def _at(pt):
                return _owner(pt, segs, net.pads, pcb)
            at_dest = [pt for pt in free if _at(pt) == dest_ref]
            at_src = [pt for pt in free if _at(pt) != dest_ref]
            if at_dest and at_src:
                # farthest-apart pair, so a net with several stubs at one
                # end still spans the whole corridor
                src = max(at_src, key=lambda p: max(ts.d2(p, q)
                                                    for q in at_dest))
                tgt = max(at_dest, key=lambda q: ts.d2(src, q))
                ends[nm] = (src, tgt, dest_ref)
                continue
            # only one end fanned out: fall through to the ball, which
            # is the honest answer rather than inventing a stub
            free = at_src or free
        if len(free) > 1:
            # soft joints / branches: the escape's exit is the end
            # farthest from every pad
            free.sort(key=lambda pt: -min(ts.d2(pt, a) for a in anchors))
        src = free[0]
        tgt = max(net.pads, key=lambda p: ts.d2((p.global_x, p.global_y),
                                                src))
        ends[nm] = (src, (tgt.global_x, tgt.global_y), tgt.component_ref)
    return ends


def lis_keep(ranks):
    n = len(ranks)
    best = [1] * n
    prev = [-1] * n
    for i in range(n):
        for j in range(i):
            if ranks[j] < ranks[i] and best[j] + 1 > best[i]:
                best[i] = best[j] + 1
                prev[i] = j
    i = max(range(n), key=lambda k: best[k])
    keep = set()
    while i >= 0:
        keep.add(i)
        i = prev[i]
    return keep


def lis_keep_weighted(ranks, weight):
    """A maximum-length increasing subsequence, choosing among the
    equally-long ones the one of greatest total weight.

    The LIS is NOT unique, and which one is picked decides which nets
    become DIVERS (the complement). That matters: a diver is delivered
    on the opposite layer, so it pairs for free with a dive escape
    (their vias merge) and costs an extra via with a surface escape.
    Weighting the nets whose escape starts on the tooth layer -- the
    ones that WANT to be keepers -- picks the LIS that aligns the two,
    at no cost in length.

    Length dominates: the weight bonus is scaled below 1 so it can only
    break ties between equally long subsequences.
    """
    n = len(ranks)
    if n == 0:
        return set()
    W = [1.0 + 0.5 * float(weight[i]) for i in range(n)]
    best = [(1, W[i]) for i in range(n)]
    prev = [-1] * n
    for i in range(n):
        for j in range(i):
            if ranks[j] < ranks[i]:
                cand = (best[j][0] + 1, best[j][1] + W[i])
                if cand > best[i]:
                    best[i] = cand
                    prev[i] = j
    i = max(range(n), key=lambda k: best[k])
    keep = set()
    while i >= 0:
        keep.add(i)
        i = prev[i]
    return keep


def rdp(pts, eps=0.02):
    """Ramer-Douglas-Peucker polyline simplification."""
    if len(pts) < 3:
        return list(pts)
    a, b = pts[0], pts[-1]
    dmax, idx = 0.0, 0
    for i in range(1, len(pts) - 1):
        d = ts.seg_pt_dist(a, b, pts[i])
        if d > dmax:
            dmax, idx = d, i
    if dmax <= eps:
        return [a, b]
    left = rdp(pts[:idx + 1], eps)
    return left[:-1] + rdp(pts[idx:], eps)


def octify_seg(p, q, eps=0.05):
    """Decompose one arbitrary-angle segment into octilinear pieces that
    stay within ~eps of the original: K interleaved blocks, each
    half-axis / 45 / half-axis (centered look). Steep segments become
    45/vertical staircases, shallow ones long axis runs with gentle 45
    nudges."""
    dx, dy = q[0] - p[0], q[1] - p[1]
    adx, ady = abs(dx), abs(dy)
    if adx < 1e-9 or ady < 1e-9 or abs(adx - ady) < 1e-9:
        return [p, q]
    m = min(adx, ady)               # total 45 travel (per axis)
    r = abs(adx - ady)              # dominant-axis remainder
    K = max(1, math.ceil(r * m / ((adx + ady) * 2 * eps)))
    sx = 1 if dx > 0 else -1
    sy = 1 if dy > 0 else -1
    dom_x = adx >= ady
    pts = [p]
    x, y = p
    for _k in range(K):
        for frac in (0.5, None, 0.5):
            if frac is None:
                x += sx * m / K
                y += sy * m / K
            elif dom_x:
                x += sx * r / K * frac
            else:
                y += sy * r / K * frac
            pts.append((x, y))
    pts[-1] = q
    return pts


def octi45(p, q):
    """Waypoints from p to q using one 45 segment + one axis segment
    (p, q included)."""
    dx, dy = q[0] - p[0], q[1] - p[1]
    if abs(dx) < 1e-9 or abs(dy) < 1e-9 or abs(abs(dx) - abs(dy)) < 1e-9:
        return [p, q]
    sx = math.copysign(1, dx)
    sy = math.copysign(1, dy)
    if abs(dx) > abs(dy):        # 45 first, then horizontal
        mid = (p[0] + sx * abs(dy), q[1])
    else:                        # 45 first, then vertical
        mid = (q[0], p[1] + sy * abs(dx))
    return [p, mid, q]


def astar_f(ok, start, goal, bound, h=0.1, x_lo=None, x_hi=None,
            y_hi=None, bound_hi=None):
    """Octilinear grid A* on one layer from `start` to `goal`.

    Eight moves of `h`, a small turn penalty, every step validated by
    `ok(p, q)` (static copper + copper already placed by the caller),
    and a floor `bound(x)` the path may not rise above (y smaller
    than it): the trunk's lanes live there. The grid is anchored on
    the goal so the last waypoint is exact; the first segment jumps
    from the exact start onto the grid. Returns waypoints with
    collinear runs merged, or None.

    This is the "A* only for the connection" piece: the trunk decides
    the order and the layers, the A* only threads one tail through
    whatever clutter sits between the trunk's end and a stub end
    (K15: C5 exactly where a straight descent to the south port must
    pass -- both its pads, at 45 and at the morph's slope alike).
    """
    import heapq
    gx, gy = goal

    def node(p):
        return (round((p[0] - gx) / h), round((p[1] - gy) / h))

    def pt(n):
        return (gx + n[0] * h, gy + n[1] * h)

    s0 = node(start)
    if not ok(start, pt(s0)):
        return None
    x_lo = start[0] - 0.3 if x_lo is None else x_lo
    x_hi = goal[0] + 0.6 if x_hi is None else x_hi
    y_hi = goal[1] + 3.0 if y_hi is None else y_hi
    DIRS = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1),
            (1, -1)]
    R2 = math.sqrt(2)

    def hcost(n):
        dx, dy = abs(n[0]), abs(n[1])
        return h * (max(dx, dy) + (R2 - 1) * min(dx, dy))

    best = {(s0, None): 0.0}
    came = {}
    heap = [(hcost(s0), 0.0, s0, None)]
    goal_state = None
    while heap:
        f, g, n, d = heapq.heappop(heap)
        if best.get((n, d), 1e9) < g - 1e-12:
            continue
        if n == (0, 0):
            goal_state = (n, d)
            break
        p = pt(n)
        for k, (dx, dy) in enumerate(DIRS):
            m = (n[0] + dx, n[1] + dy)
            q = pt(m)
            if not (x_lo <= q[0] <= x_hi) or q[1] > y_hi or \
                    q[1] < bound(q[0]):
                continue
            if bound_hi is not None and q[1] > bound_hi(q[0]):
                continue
            if not ok(p, q):
                continue
            g2 = g + h * (R2 if dx and dy else 1.0) + \
                (0.0 if d is None or d == k else 0.06)
            if g2 < best.get((m, k), 1e9):
                best[(m, k)] = g2
                came[(m, k)] = (n, d)
                heapq.heappush(heap, (g2 + hcost(m), g2, m, k))
    if goal_state is None:
        return None
    path = []
    st = goal_state
    while st is not None:
        path.append(pt(st[0]))
        st = came.get(st)
    path.reverse()
    return merge_collinear([start] + path)


def merge_collinear(pts):
    out = [pts[0]]
    for p in pts[1:]:
        if abs(p[0] - out[-1][0]) + abs(p[1] - out[-1][1]) < 1e-9:
            continue
        if len(out) >= 2:
            ax, ay = out[-2]
            bx, by = out[-1]
            d1 = (bx - ax, by - ay)
            d2 = (p[0] - bx, p[1] - by)
            if abs(d1[0] * d2[1] - d1[1] * d2[0]) < 1e-9 and \
                    d1[0] * d2[0] + d1[1] * d2[1] > 0:
                out[-1] = p
                continue
        out.append(p)
    return out


def octify_segs(segs, eps=0.05, fine=None):
    """Octilinearize a net's layered segment chain: per constant-layer
    run, RDP-simplify (the morphing-lane points are collinear between
    swaps and merge away), then octify each remaining segment. `fine`
    = (windows, fine_eps): segments overlapping a window x-range use
    the tighter tube (verifier-driven local refinement)."""
    out = []
    i = 0
    while i < len(segs):
        layer = segs[i][2]
        pts = [segs[i][0], segs[i][1]]
        j = i
        while j + 1 < len(segs) and segs[j + 1][2] == layer and \
                segs[j + 1][0] == segs[j][1]:
            j += 1
            pts.append(segs[j][1])
        run = rdp(pts, 0.02)
        opts = [run[0]]
        for p, q in zip(run, run[1:]):
            e = eps
            if fine:
                wlist, fe = fine
                xlo, xhi = min(p[0], q[0]), max(p[0], q[0])
                if any(xlo <= b and a <= xhi for (a, b) in wlist):
                    e = fe
            opts.extend(octify_seg(p, q, e)[1:])
        opts = merge_collinear(opts)
        out.extend((p, q, layer) for p, q in zip(opts, opts[1:]))
        i = j + 1
    return out


def strip_net_segments(txt, net_ids, net_names=()):
    """Remove every (segment ...) block whose net ref matches, in
    EITHER dialect: numeric (net N) or quoted name (net "/path/NAME")
    (#749 lore: boards carry both). Paren-balanced (KiCad multi-line
    and our one-line forms)."""
    out = []
    i = 0
    while True:
        j = txt.find('(segment', i)
        if j < 0:
            out.append(txt[i:])
            break
        k, depth = j, 0
        while True:
            c = txt[k]
            if c == '(':
                depth += 1
            elif c == ')':
                depth -= 1
                if depth == 0:
                    break
            k += 1
        block = txt[j:k + 1]
        m = re.search(r'\(net (\d+)\)', block)
        m2 = re.search(r'\(net "([^"]+)"\)', block)
        if (m and int(m.group(1)) in net_ids) or \
                (m2 and m2.group(1) in net_names):
            out.append(txt[i:j].rstrip(' \t'))
            e = k + 1
            if e < len(txt) and txt[e] == '\n':
                e += 1
            i = e
        else:
            out.append(txt[i:k + 1])
            i = k + 1
    return ''.join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--board', default=os.path.join(HERE,
                    'fb_t2q_base.kicad_pcb'))
    ap.add_argument('--nets', default='SDQ15,SDQ14,SDQ13,SDQ11')
    ap.add_argument('--out', default=os.path.join(HERE, 'topo_k4_emit'))
    # The destination is already fanned out (a fanout step ran first),
    # so the braid delivers to its STUB ENDS and the field-entry
    # machinery is not its problem. Omitted = the bench shape, ball
    # targets, exactly as before.
    ap.add_argument('--dest-stubs', metavar='REF', default=None,
                    help='destination component whose stub ends are '
                         'the braid targets')
    ap.add_argument('--pitch', type=float, default=0.8)
    ap.add_argument('--dump-segs', default='')
    ap.add_argument('--no-smooth', action='store_true',
                    help='skip the repo #536 octolinear smoothing pass')
    ap.add_argument('--moves', default='',
                    help='homotopy entry moves, e.g. SDQ0=S,SDQ11=N')
    ap.add_argument('--no-vip', action='store_true',
                    help='disable via-in-pad berths (task 4)')
    ap.add_argument('--plan-out', default='',
                    help='export per-net layer/entry plan JSON (task 5)')
    ap.add_argument('--no-octi', action='store_true',
                    help='emit the raw arbitrary-angle braid geometry')
    a = ap.parse_args()
    names = [n.strip() for n in a.nets.split(',') if n.strip()]

    pcb = parse_kicad_pcb(a.board)
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    # FLOW FRAME: everything below assumes the source is due west of the
    # destination. Rotate the board so that it is, then map the emitted
    # copper back at write time. Snapped to 90 degrees, so the frame is
    # a relabelling of the axes: octilinear stays octilinear and the
    # destination's rows stay rows.
    # dest_ref matters HERE too, not only at the real call below: on a
    # board fanned out at both ends the default picks the destination
    # stub as the source, so the probe concluded the source was EAST and
    # rotated the whole problem 180 degrees -- correctly, for the
    # endpoints it was given, and uselessly for the ones actually routed.
    _probe = endpoints(pcb, [n.strip() for n in a.nets.split(',')
                             if n.strip()], byname,
                       dest_ref=a.dest_stubs)
    _theta = ff.flow_angle([_probe[n][0] for n in _probe],
                           [_probe[n][1] for n in _probe])
    if os.environ.get('KICAD_FLOW_FRAME') == '0':
        _theta = 0.0      # negative control: prove the frame is needed
    _cx = sum(_probe[n][1][0] for n in _probe) / max(len(_probe), 1)
    _cy = sum(_probe[n][1][1] for n in _probe) / max(len(_probe), 1)
    pcb, back_xy = ff.rotate_pcb(pcb, _theta, _cx, _cy)
    if _theta:
        print(f'flow frame: source is not due west -- rotating the '
              f'problem by {_theta:.0f} deg about '
              f'({_cx:.2f},{_cy:.2f})')
        byname = {n.name.split('/')[-1]: (i, n)
                  for i, n in pcb.nets.items()}
    kids = {byname[nm][0] for nm in names}
    ends = endpoints(pcb, names, byname, dest_ref=a.dest_stubs)

    comps = {ends[nm][2] for nm in names}
    comps_pads0 = [p for c in comps for p in pcb.footprints[c].pads]
    rows0 = sorted({round(p.global_y, 3) for p in comps_pads0})
    # SOUTH RIVER split (take-3 task 3, K15+): nets whose fanout stubs
    # exit U1's south flank (teeth below the field) form a second river
    # routed by a dedicated builder; the west braid never sees them
    all_names = list(names)
    moves0 = {}
    for tok in a.moves.split(','):
        if '=' in tok:
            k, v = tok.split('=')
            moves0[k.strip()] = v.strip().upper()
    # The split into a separate hand-written flow is audit item 1. The
    # braid's lane morph does not inherently need a tooth inside the
    # destination's y-span -- it maps ANY launch order onto ANY entry
    # order -- so KICAD_NO_RIVER=1 routes these nets as ordinary
    # corridor nets and measures whether the separate builder is
    # earning its 87 lines.
    river2 = [] if os.environ.get('KICAD_NO_RIVER') else [
        nm for nm in names
        if ends[nm][0][1] > rows0[-1] + 0.8 and nm not in moves0]
    if river2:
        names = [nm for nm in names if nm not in river2]
        print(f'south river ({len(river2)}): {river2}')
    fx0 = min(p.global_x for c in comps for p in pcb.footprints[c].pads)
    x1 = fx0 - a.pitch * 0.8
    if a.dest_stubs:
        # the splice line must sit WEST of every stub end the braid is
        # delivering to; the field-derived x1 is only right when the
        # stub ends are east of it
        # ...and 0.6 mm west of them, so the tails have room for a 45
        # degree jog: the lanes leave the splice at a pitch a surface
        # via fits between (0.38), the stub ends are packed at 0.25.
        x1 = min(x1, min(ends[nm][1][0] for nm in names) - 0.6)
    x0 = max(ends[nm][0][0] for nm in names) + 0.3

    launch = sorted(names, key=lambda nm: ends[nm][0][1])
    print(f'launch order: {launch}')

    obsF = {nm: build_obstacles(pcb, byname[nm][0], kids, 'F.Cu')
            for nm in names}
    half = a.pitch / 2
    seg_min = TRACK + CLEAR                   # placed-vs-placed center min
    via_seg_min = VIA_SIZE / 2 + TRACK / 2 + CLEAR

    # entry assignment (ball-y order). F candidates: DIRECT (column-A
    # ball: enter at its own row y, no street consumed), north street,
    # south street. Leftovers get a B dogbone site. Candidates must be
    # static-clear and clear of already-placed tails (layer-aware).
    entry = {}          # nm -> ('F', entry_y) | ('B', (sx, sy))
    placed_f, placed_vias = [], []

    def f_ok(nm, segs):
        for (p, q) in segs:
            if p == q:
                continue
            if not obsF[nm].seg_clear(p, q):
                return False
            for (c, e) in placed_f:
                if ts.seg_seg_dist(p, q, c, e) < seg_min:
                    return False
            for v in placed_vias:
                if ts.seg_pt_dist(p, q, v) < via_seg_min:
                    return False
        return True

    moves = {}
    for tok in a.moves.split(','):
        if '=' in tok:
            k, v = tok.split('=')
            moves[k.strip()] = v.strip().upper()
    comps_pads = [p for c in comps for p in pcb.footprints[c].pads]
    rows_all = sorted({round(p.global_y, 3) for p in comps_pads})

    ball_order = sorted(names, key=lambda nm: (ends[nm][1][1],
                                               ends[nm][1][0]))

    # ---- the escape-move MENU for every berth pad (audit item 1).
    # The array's pitch and extent are measured from the footprint here,
    # not taken from --pitch.
    _dest_fp = {}
    for nm in names:
        _dest_fp.setdefault(ends[nm][2], pcb.footprints[ends[nm][2]])
    _grids = {ref: em.grid_of(fp) for ref, fp in _dest_fp.items()}
    _obs_cache = {}

    def _obs(nid, layer):
        if (nid, layer) not in _obs_cache:
            _obs_cache[(nid, layer)] = build_obstacles(pcb, nid, kids,
                                                       layer)
        return _obs_cache[(nid, layer)]

    LAYERS = ('F.Cu', 'B.Cu')
    menu = {}
    for nm in ([] if a.dest_stubs else names):
        nid_m = byname[nm][0]
        ref = ends[nm][2]
        bxm, bym = ends[nm][1]
        pad_m = min(pcb.footprints[ref].pads,
                    key=lambda p: (p.global_x - bxm) ** 2
                    + (p.global_y - bym) ** 2)

        def _clear(pp, qq, lay, _n=nid_m):
            return _obs(_n, lay).seg_clear(pp, qq)

        def _vclear(pp, lay, _n=nid_m):
            vv = _obs(_n, lay).point_violation(
                pp, pad=(VIA_SIZE - TRACK) / 2)
            return not (vv and vv[0] > 0)

        menu[nm] = em.enumerate_moves(pad_m, _grids[ref], LAYERS,
                                      _clear, _vclear)
    if menu:
        print('escape menu: ' + ' '.join(
            f'{nm}:{len(menu[nm])}' for nm in ball_order))

    # In --dest-stubs mode the entry is decided in one line below;
    # these loops exist to get INTO a ball field that is already
    # escaped, and the B loop asserts when it cannot.
    for nm in ([] if a.dest_stubs else ball_order):
        if nm in moves:
            continue
        bx, by = ends[nm][1][0], ends[nm][1][1]
        cands = []
        if bx <= fx0 + 0.01:
            cands.append(('direct', by, [((x1, by), (bx, by))]))
        for sy in (by - half, by + half):
            cands.append(('street', sy, [((x1, sy), (bx, sy)),
                                         ((bx, sy), (bx, by))]))
        for kind, ey, segs in cands:
            if any(abs(ey - e[1]) < 0.3 for e in entry.values()
                   if e[0] == 'F'):
                continue
            if f_ok(nm, segs):
                entry[nm] = ('F', ey)
                placed_f.extend(s for s in segs if s[0] != s[1])
                break

    # flank entries (#622 take-3 task 2): trunk lane just outside the W
    # block (clears the corridor cap band), vertical drop at the splice
    # column to the outer N/S run along the field edge, then the
    # vertical inter-column street west of the ball, half-pitch jog.
    f_ys = [e[1] for e in entry.values() if e[0] == 'F']
    flank_prev_run = {'N': rows_all[0] - 0.25, 'S': rows_all[-1] + 0.25}
    for flank in ('N', 'S'):
        movers = sorted((nm for nm, f in moves.items() if f == flank),
                        key=lambda nm: ends[nm][1][0])
        for i, nm in enumerate(movers):
            bx, by = ends[nm][1][0], ends[nm][1][1]
            sgn = -1 if flank == 'N' else 1
            # street: nearest side whose descent+jog is static-clear
            sx = None
            for sx_ in (bx - half, bx + half):
                edge = rows_all[0] - 0.3 if flank == 'N'                     else rows_all[-1] + 0.3
                if f_ok(nm, [((sx_, edge), (sx_, by)),
                             ((sx_, by), (bx, by))]):
                    sx = sx_
                    break
            assert sx is not None, f'no clear {flank} street for {nm}'
            # run y: step OUTWARD until the run is static-clear (C9/C7
            # class obstacles guard the field's N approach)
            run_y = flank_prev_run[flank] + sgn * 0.30
            while not f_ok(nm, [((x1 - 0.4, run_y), (sx, run_y))]):
                run_y += sgn * 0.15
                assert abs(run_y - rows_all[0]) < 8, 'no clear flank run'
            flank_prev_run[flank] = run_y
            if flank == 'N':
                ly = (min(f_ys) if f_ys else rows_all[0]) - 0.35 - i * 0.3
            else:
                ly = (max(f_ys) if f_ys else rows_all[-1]) + 0.35 + i * 0.3
            dx_ = x1 - 0.35 * i        # staggered drop columns
            entry[nm] = (flank, (sx, run_y, ly, dx_))
            placed_f.append(((dx_, ly), (dx_, run_y)))
            placed_f.append(((dx_, run_y), (sx, run_y)))
            placed_f.append(((sx, run_y), (sx, by)))
            placed_f.append(((sx, by), (bx, by)))

    obsB = {}
    placed_b = []           # B approaches: horizontal runs at site y
    vip_nets = []           # via-in-pad berths (IPC-4761 fab note)

    def b_ok(nm, sx, sy):
        """Dogbone site + its westward B approach (a street-line run
        east of x1) must clear B.Cu copper, earlier B approaches and
        sites, and the via barrel must clear both layers."""
        nid = byname[nm][0]
        if nm not in obsB:
            obsB[nm] = build_obstacles(pcb, nid, kids, 'B.Cu')
        run = ((x1, sy), (sx, sy))
        if not obsB[nm].seg_clear(*run):
            return False
        vpad_ = (VIA_SIZE - TRACK) / 2
        for o in (obsB[nm], obsF[nm]):
            vv = o.point_violation((sx, sy), pad=vpad_)
            if vv and vv[0] > 0:
                return False
        for (c, e) in placed_b:
            if ts.seg_seg_dist(run[0], run[1], c, e) < seg_min:
                return False
            if ts.seg_pt_dist(c, e, (sx, sy)) < via_seg_min:
                return False
        return True

    for nm in ([] if a.dest_stubs else ball_order):
        if nm in entry or nm in moves:
            continue
        bx, by = ends[nm][1][0], ends[nm][1][1]
        site = None
        if not a.no_vip:
            # via-in-pad berth (task 4): the surface via IS the ball --
            # no F stub, no diagonal cell consumed (IPC-4761 Type VII)
            ok = all(math.hypot(bx - v[0], by - v[1]) >
                     VIA_SIZE + CLEAR for v in placed_vias)
            if ok and b_ok(nm, bx, by):
                site = (bx, by)
                vip_nets.append(nm)
        if site is None:
            sx = bx - half
            for sy in (by - half, by + half):
                ok = all(ts.seg_pt_dist(p, q, (sx, sy)) > via_seg_min
                         for (p, q) in placed_f)
                ok = ok and all(math.hypot(sx - v[0], sy - v[1]) >
                                VIA_SIZE + CLEAR for v in placed_vias)
                if ok and f_ok(nm, [((sx, sy), (bx, by))]) and \
                        b_ok(nm, sx, sy):
                    site = (sx, sy)
                    placed_f.append(((sx, sy), (bx, by)))
                    break
        assert site is not None, f'no street/dogbone/VIP for {nm}'
        entry[nm] = ('B', site)
        placed_vias.append(site)
        placed_b.append(((x1, site[1]), site))

    port_placed = {}      # nm -> the straight port segs placed for it
    if a.dest_stubs:
        # The destination is already fanned out, so there is no field to
        # enter: a net whose stub exits the WEST face is an F entry on
        # the stub end's own y, and the 'F' path below (trunk ->
        # (x1, sy) -> (bx, sy) -> (bx, by)) collapses to the direct run
        # when sy == by. This is the whole point of chaining a fanout in
        # front of the braid -- the machinery above is what fails at
        # K32 and beyond, and here it is simply not needed.
        entry.clear()
        fy1 = rows0[-1]
        south = sorted((nm for nm in names if ends[nm][1][1] > fy1 + 0.2),
                       key=lambda nm: ends[nm][1][0])
        for nm in names:
            if nm not in south:
                entry[nm] = ('F', ends[nm][1][1])
        if south:
            # SOUTH PORT: a west-toothed net whose stub the plan sent out
            # the SOUTH face (K15: SDQ0, which was the deepest diver of
            # the west-only plan and is monotone this way). It rides the
            # trunk's bottom lanes, runs east under the field on a port
            # line below the plane-drop vias, and turns up into its own
            # stub. Nearest stub takes the uppermost lane, so no run
            # crosses an earlier net's turn-up. The port line steps
            # south until every run and turn is clear of static copper.
            y_s = fy1 + 0.55
            while True:
                trial = []
                for i, nm in enumerate(south):
                    bx_, by_ = ends[nm][1]
                    # 0.35 pitch: the octify tube deviates each lane by
                    # up to 0.05, and 0.27 measured 0.22 between two
                    # port lanes at K21 (0.227 is the floor)
                    ly_ = y_s + i * 0.35
                    segs_ = [((x1, ly_), (bx_, ly_)), ((bx_, ly_), (bx_, by_))]
                    if not f_ok(nm, segs_):
                        break
                    trial.append((nm, ly_, segs_))
                if len(trial) == len(south):
                    break
                y_s += 0.15
                assert y_s < fy1 + 6, 'no clear south port line'
            for nm, ly_, segs_ in trial:
                entry[nm] = ('P', (ly_, ends[nm][1][0]))
                placed_f.extend(segs_)
                port_placed[nm] = list(segs_)
            print(f'south port ({len(south)}) at y={y_s:.2f}: '
                  + ', '.join(f'{nm}@{ends[nm][1][0]:.2f}' for nm in south))
        _ys = sorted((e[1] if e[0] == 'F' else e[1][0], nm)
                     for nm, e in entry.items())
        _tight = [(b, aa) for (ya, aa), (yb, b) in zip(_ys, _ys[1:])
                  if abs(yb - ya) < 0.25]
        if _tight:
            # Not fatal, but not silent either: two stub ends this close
            # share a lane and the trunk cannot separate them.
            print(f'WARNING: {len(_tight)} stub-end pair(s) within '
                  f'0.25mm in y -- they share a lane: '
                  + ', '.join(f'{x}/{y}' for x, y in _tight[:6]))

    # lane ys: F nets pin their entry; B nets slot strictly between the
    # neighbouring F entries, ordered by site y
    f_sorted = sorted([(e[1], nm) for nm, e in entry.items()
                       if e[0] == 'F']
                      + [(e[1][2], nm) for nm, e in entry.items()
                         if e[0] in ('N', 'S')]
                      + [(e[1][0], nm) for nm, e in entry.items()
                         if e[0] == 'P'])
    b_nets = sorted(((e[1][1], nm) for nm, e in entry.items()
                     if e[0] == 'B'))
    lane_y = {nm: ey for ey, nm in f_sorted}
    for sy, nm in b_nets:
        fy = [v[0] for v in f_sorted]
        lo = max([y for y in fy if y <= sy], default=sy - 0.4)
        hi = min([y for y in fy if y > sy], default=sy + 0.4)
        cand = (lo + hi) / 2
        while any(abs(cand - v) < 0.25 for v in lane_y.values()):
            cand += 0.27
        lane_y[nm] = cand

    target = sorted(names, key=lambda nm: lane_y[nm])
    trank = {nm: i for i, nm in enumerate(target)}
    ranks = [trank[nm] for nm in launch]
    keep = lis_keep(ranks)
    divers = {launch[i] for i in range(len(launch)) if i not in keep}
    promoted = [nm for nm in names
                if entry[nm][0] == 'B' and nm not in divers]
    divers |= set(promoted)
    print(f'entry order (lane y): {target}  ranks: {ranks}')
    print(f'divers ({len(divers)}): {sorted(divers, key=lambda n: trank[n])}')
    for nm in target:
        mode, v = entry[nm]
        print(f'{nm}: {mode}-entry {v} lane_y={lane_y[nm]:.3f}'
              + (' (PROMOTED +2 vias)' if nm in promoted else ''))

    # swap schedule (wave-pipelined, task 3): divers that are pairwise
    # NON-inverted form a wave and bubble concurrently, offset one
    # column -- column count ~ longest chain + wave size instead of
    # total inversions (the serial form ran out of corridor at K11:
    # 37 columns x 0.149mm stages violated raw clearances). Waves run
    # serially, so every swap involving a diver outside its own flight
    # happens while it is F, keeping windows consistent.
    lidx = {nm: i for i, nm in enumerate(launch)}

    def inverted(anm, bnm):
        return (lidx[anm] < lidx[bnm]) != (trank[anm] < trank[bnm])

    ups = sorted((d for d in divers if trank[d] < lidx[d]),
                 key=lambda nm: trank[nm])
    downs = sorted((d for d in divers if trank[d] >= lidx[d]),
                   key=lambda nm: -trank[nm])
    dirs = {d: (-1 if trank[d] < lidx[d] else 1) for d in divers}
    # task 5: a diver whose fanout stub already sits on B.Cu is BORN
    # diving -- no dive via. It must do all its diver-diver crossings
    # as the MOVER (it can never be F while passed), so B-birth divers
    # go to the FRONT of priority; mutually-inverted B-birth pairs are
    # illegal (the plan/refanout must not create them).
    wtooth_layer = {}
    for nm in names:
        nid2 = byname[nm][0]
        tp = ends[nm][0]
        wtooth_layer[nm] = next(
            (s.layer for s in pcb.segments if s.net_id == nid2
             and (abs(s.start_x - tp[0]) + abs(s.start_y - tp[1]) < 0.005
                  or abs(s.end_x - tp[0]) + abs(s.end_y - tp[1])
                  < 0.005)), 'F.Cu')
    birth_b = {d for d in divers if wtooth_layer[d] == 'B.Cu'}
    for da in birth_b:
        for db in birth_b:
            if da < db:
                assert not inverted(da, db), \
                    f'mutually inverted B-birth divers {da}/{db}'
    if birth_b:
        print(f'B-birth divers (no dive via): {sorted(birth_b)}')
        ups = [d for d in birth_b if d in ups] + \
            [d for d in ups if d not in birth_b]
        downs = [d for d in birth_b if d in downs] + \
            [d for d in downs if d not in birth_b]
    priority = ups + downs

    # serial pass (the proven west-asc / east-desc remove-insert form)
    # fixes WHO passes WHOM: mover_set[p] = the partners p itself passes
    mover_set = {d: [] for d in priority}
    sseq = list(launch)
    for d in priority:
        i = sseq.index(d)
        rest = sseq[:i] + sseq[i + 1:]
        want = 0
        for j, e in enumerate(rest):
            if trank[e] < trank[d]:
                want = j + 1
        passed_ser = rest[want:i] if want <= i else rest[i:want]
        mover_set[d] = list(passed_ser)
        sseq = rest[:want] + [d] + rest[want:]
    assert sseq == target, (sseq, target)

    # gated rounds: d performs only its OWN mover-set swaps; before
    # flying past an earlier inverted diver p it waits for p's pass if
    # p will do the passing, else for p to FINISH (window closed) --
    # so every diver-diver crossing has exactly one side on B.
    def schedule(gap):
        """The gated wave schedule. `gap` = spacer columns a diver
        waits after being passed before its own first swap, so its
        dive via has room between the passer's lane and its own first
        crossing."""
        done_moves = {d: set() for d in priority}
        passed = set()
        last_passed_round = {}
        seq = list(launch)
        cols = []            # cols[c] = [(mover, partner), ...]
        guard = 0

        def finished(p):
            return len(done_moves[p]) == len(mover_set[p])

        while seq != target:
            used = set()
            col = []
            delay_stall = False
            for d in priority:
                if d in used or finished(d):
                    continue
                gated = False
                for p in priority[:priority.index(d)]:
                    if not inverted(p, d):
                        continue
                    if d in mover_set[p]:
                        if (p, d) not in passed:
                            gated = True
                            break
                    elif not finished(p):
                        gated = True
                        break
                if gated:
                    continue
                # clear column(s) after being passed, so the dive via
                # has room west of this diver's own first crossing
                if not done_moves[d] and d in last_passed_round and \
                        len(cols) <= last_passed_round[d] + gap:
                    delay_stall = True
                    continue
                i = seq.index(d)
                j = i + dirs[d]
                if not (0 <= j < len(seq)):
                    continue
                p = seq[j]
                if p in used or p not in mover_set[d] or \
                        p in done_moves[d]:
                    continue                          # stalled this round
                seq[i], seq[j] = seq[j], seq[i]
                col.append((d, p))
                used.add(d)
                used.add(p)
                done_moves[d].add(p)
                if p in dirs:
                    passed.add((d, p))
                    last_passed_round[p] = len(cols)
            if not col and delay_stall:
                cols.append([])          # spacer column -- the delay's point
                guard += 1
                continue
            assert col, ('wave schedule deadlocked', seq, target)
            cols.append(col)
            guard += 1
            assert guard < 20 * len(names) + 20, 'wave schedule runaway'
        return cols

    # The bench's rule was ONE spacer column: 1.5W of room, which at
    # its W (~0.5) is a via's worth. In dest-stubs mode the corridor is
    # shorter (tail room + surfacing reserve) and W drops to ~0.37, so
    # the passed diver's dive via -- 0.36 clear of the passer's lane
    # kink, 0.45 west of its own first crossing -- needs (gap+0.5)W >=
    # 0.81. Iterate: more spacers shrink W, which may need more spacers.
    # Measured at K21: SDQ13 had a 0.12 mm window at gap 1 and the last
    # resort branch put its dive via west of the pass (B on B, 343 DRC).
    gap = 1
    reserve = 0.5 if a.dest_stubs else 0.0
    for _ in range(4):
        cols = schedule(gap)
        if not a.dest_stubs:
            break
        W_est = (x1 - x0 - reserve) / (len(cols) + 1)
        need = max(1, math.ceil(0.81 / W_est - 0.5))
        if need <= gap:
            break
        gap = need
    if gap > 1:
        print(f'pass-to-swap gap {gap} columns (W ~{W_est:.3f})')
    swaps = [sw for col in cols for sw in col]
    seen_pairs = set()
    for (d, e) in swaps:
        pair = frozenset((d, e))
        assert pair not in seen_pairs, f'pair {d}/{e} swapped twice'
        seen_pairs.add(pair)
        assert inverted(d, e), f'phantom swap {d}/{e} (not an inversion)'
    n = len(cols)
    print(f'{len(swaps)} swaps in {n} columns: {cols}')

    # dest-stubs: RESERVE trunk after the last swap column. A diver
    # that swaps in the last column needs ~0.45 mm of trunk past it to
    # surface, and the exit line has no room for a via at all (stubs
    # at 0.25 pitch): the bench's via-in-pad fallback put SDQM1's
    # surface via ON its stub end, 0.25 from SDQ14's.
    W = (x1 - x0 - reserve) / (n + 1)
    if W < 0.24:
        print(f'WARNING: column pitch {W:.3f} < 0.24mm -- expect raw '
              f'clearance violations')
    first, last = {}, {}
    invol = {}
    for s, col in enumerate(cols):
        for (d, p) in col:
            first.setdefault(d, s)
            last[d] = s
            invol.setdefault(d, []).append(s)
            invol.setdefault(p, []).append(s)

    # windows: dive/surface vias want >= 0.45mm clear of the nearest swap
    # column, but must stay strictly inside the diver's own band region:
    # a swap involving d recorded during ANOTHER diver's move needs d on
    # F there, so the window is clamped away from those stages.
    window = {}
    wa_lo_map = {}
    # B-birth divers: window opens at the tooth (their stub IS the B
    # copper); assert nobody passes them west of their own swaps
    
    wb_hi_map = {}
    for d in divers:
        if d in birth_b:
            s1 = first.get(d)
            if s1 is not None:
                fw_chk = [s for s in invol[d] if s < s1]
                assert not fw_chk, f'B-birth {d} passed before its band'
            wa = ends[d][0][0] - 0.05
            sk = last.get(d)
            fe = min((s for s in invol[d] if sk is None or s > sk),
                     default=None)
            wb_hi = x0 + (fe + 0.5) * W - 0.30 if fe is not None \
                else x1 - 0.15
            if sk is not None:
                wb = min(wb_hi, max(x0 + (sk + 1.6) * W,
                                    x0 + (sk + 1.5) * W + 0.45))
                if fe is None and wb < x0 + (sk + 1.5) * W + 0.28:
                    if not a.no_vip:
                        wb = None
                        vip_nets.append(d)
                    else:
                        wb = min(x1 + 0.31, ends[d][1][0] - 0.35)
            else:
                wb = None if not a.no_vip else x1 - 0.15
                if wb is None:
                    vip_nets.append(d)
            if entry[d][0] == 'B':
                window[d] = (wa, None)
            else:
                window[d] = (wa, wb)
            continue
        fe = None
        if d in first:
            s1, sk = first[d], last[d]
            fw = max((s for s in invol[d] if s < s1), default=None)
            fe = min((s for s in invol[d] if s > sk), default=None)
            wa_lo = x0 + (fw + 1.5) * W + 0.10 if fw is not None \
                else x0 - 0.05
            wa_lo_map[d] = wa_lo
            wa = max(wa_lo, min(x0 + (s1 + 0.4) * W,
                                x0 + (s1 + 0.5) * W - 0.45))
            wb_hi = x0 + (fe + 0.5) * W - 0.30 if fe is not None \
                else x1 - 0.15
            wb_hi_map[d] = wb_hi
            wb = min(wb_hi, max(x0 + (sk + 1.6) * W,
                                x0 + (sk + 1.5) * W + 0.45))
            if fe is None and wb < x0 + (sk + 1.5) * W + 0.28:
                # no trunk room after the last own swap: via-in-pad tail
                # (B all the way to the ball; safe: fe is None means no
                # foreign crossing needs this diver on F east of here),
                # else surface on the street run
                if a.dest_stubs:
                    # there is no pad to put a via in -- the target is a
                    # stub end with a neighbour 0.25 away. The reserve
                    # above is meant to make this unreachable.
                    print(f'WARNING: {d}: no trunk room to surface after '
                          f'its last swap (column {sk}) -- via forced '
                          f'onto the tail')
                if not a.no_vip and not a.dest_stubs:
                    wb = None
                    vip_nets.append(d)
                else:
                    wb = min(x1 + 0.31, ends[d][1][0] - 0.35)
        else:
            wa = x0 + (n + 0.75) * W
            if invol.get(d):
                wa = max(wa, x0 + (max(invol[d]) + 1.5) * W + 0.10)
            wa_lo_map[d] = wa            # west-slide floor (swapless)
            wb = x1 - 0.15
        if entry[d][0] == 'B':
            if d in first and fe is not None:
                # trunk-crossing B-entry: normal trunk window for its
                # own swaps, then a second dive near the splice for the
                # B approach to the dogbone site (4 vias total).
                # ONLY when a foreign crossing east of this diver's own
                # band actually needs it on F (fe is not None). With
                # fe None it may simply STAY on B from its dive through
                # to the dogbone -- the same reasoning the no-trunk-room
                # branch above already states -- which is 2 vias, not 4.
                # Measured at K11: SDQ0 surfaced at 131.21, ran 1.6mm on
                # F and re-dived at 132.84 for nothing; dropping that
                # excursion takes the rung from 16 vias to the human's 14.
                x_ad = max(wb + 0.35, x1 - 0.45)
                if invol.get(d):
                    x_ad = max(x_ad,
                               x0 + (max(invol[d]) + 1.5) * W + 0.10)
                x_ad = min(x_ad, x1 - 0.12)
                window[d] = (wa, wb, x_ad)
            else:
                window[d] = (wa, None)  # B from the dive to the site
        else:
            window[d] = (wa, wb)

    py = [lane_y[nm] for nm in target]
    assert all(b > a for a, b in zip(py, py[1:])), ('lane ys not strictly '
                                                    'increasing', py)
    if a.dest_stubs:
        # minimum lane pitch at the splice: the fanout packs stub ends
        # at 0.25 (track + clearance), a surface via next to a lane
        # needs 0.2885 and via_clear asks 0.36. Relax the tight pairs
        # apart symmetrically (port lanes pinned -- they are >1 mm from
        # anything) and let the 0.6 mm tail jog the difference.
        MINP = 0.38
        pinned = {i for i, nm in enumerate(target) if entry[nm][0] == 'P'}
        for _ in range(40):
            moved = False
            for i in range(len(py) - 1):
                gap = py[i + 1] - py[i]
                if gap < MINP - 1e-9:
                    push = (MINP - gap) / 2
                    if i not in pinned:
                        py[i] -= push if i + 1 not in pinned else 2 * push
                    if i + 1 not in pinned:
                        py[i + 1] += push if i not in pinned else 2 * push
                    moved = True
            if not moved:
                break
        jog = max(abs(py[trank[nm]] - lane_y[nm]) for nm in names)
        if jog > 1e-6:
            print(f'lane pitch floor {MINP}: max tail jog {jog:.3f} mm')
        assert jog < 0.55, ('tail jog exceeds the 0.6 mm tail', jog)
        for nm in names:
            lane_y[nm] = py[trank[nm]]
    Ly = sorted(ends[nm][0][1] for nm in names)
    # enforce a minimum launch-slot pitch: refanned B teeth can land
    # arbitrarily close to existing teeth; the tooth legs jog the
    # difference (the slot grid must never degenerate)
    _ly2 = []
    for _v in Ly:
        _ly2.append(_v if not _ly2 else max(_v, _ly2[-1] + 0.28))
    Ly = _ly2

    def slot(t, i):
        return (1 - t) * Ly[i] + t * py[i]

    def morph_t(xm):
        # launch legs stay FLAT for the first column (dive-via room at
        # exact teeth spacing; a flat leg is octilinear and never
        # deviates), then the lane morph begins
        xms = x0 + W
        return max(0.0, (xm - xms) / (x1 - xms))

    orders = [list(launch)]
    for col in cols:
        o = list(orders[-1])
        for (d, p) in col:
            i, j = o.index(d), o.index(p)
            o[i], o[j] = o[j], o[i]
        orders.append(o)

    port_path = {}        # nm -> exact drawn pts, replaces the morph

    def _pts_of(nm):
        if nm in port_path:
            return port_path[nm]
        pts = [ends[nm][0]]
        for s in range(n + 1):
            xm = x0 + (s + 0.5) * W
            pts.append((xm, slot(morph_t(xm), orders[s].index(nm))))
        pts.append((x1, py[trank[nm]]))
        return pts

    def y_at(nm, x):
        """The net's exact drawn trunk y at x (piecewise-linear through
        the column midpoints; tooth west of the first, entry lane east
        of the last)."""
        pts = _pts_of(nm)
        if x <= pts[0][0]:
            return pts[0][1]
        for a_, b_ in zip(pts, pts[1:]):
            if a_[0] <= x <= b_[0]:
                tt = (x - a_[0]) / max(b_[0] - a_[0], 1e-9)
                return a_[1] + tt * (b_[1] - a_[1])
        return pts[-1][1]

    # SOUTH PORT approaches (dest-stubs): a port net leaves the trunk
    # right after the flat launch column and is threaded to its stub
    # end by A*, kept 0.3 below every other lane. Done BEFORE the via
    # placement below, which prices every other net's ACTUAL lane.
    def _lane_y(om, x):
        """y_at, but None east of a port path's end -- that net's copper
        stops at its stub; a bound that kept reading its last y there
        made the port BELOW it unreachable (K21: SDQ0 under SDQM0)."""
        if om in port_path and x > port_path[om][-1][0] + 0.05:
            return None
        return y_at(om, x)

    for nm in target:
        if entry[nm][0] != 'P':
            continue
        # leave the trunk after the last swap column that involves this
        # net (K21: SDQM0 is passed by two divers first), at the
        # column midpoint so the trunk's own pts lead exactly there
        smax = max(invol.get(nm, [-1]))
        xs_ = x0 + W if smax < 0 else x0 + (smax + 1.5) * W
        ys_ = y_at(nm, xs_)
        bx_, by_ = ends[nm][1]
        above = [om for om in names if trank[om] < trank[nm]]
        below = [om for om in names if trank[om] > trank[nm]]

        # band: below every lane that ends above it, above every lane
        # that ends below it (final order -- valid east of xs_)
        def _bound(x, _ab=above):
            ys = [v for v in (_lane_y(om, x) for om in _ab)
                  if v is not None]
            return max(ys) + 0.26 if ys else -1e9

        def _hi(x, _be=below):
            ys = [v for v in (_lane_y(om, x) for om in _be)
                  if v is not None]
            return min(ys) - 0.26 if ys else 1e9

        # the straight port run placed at entry time was this net's own
        # copper; the A* replaces it (and must not be blocked by it)
        for s_ in port_placed.get(nm, ()):
            placed_f.remove(s_)
        pts_ = astar_f(lambda p_, q_, _n=nm: f_ok(_n, [(p_, q_)]),
                       (xs_, ys_), (bx_, by_), _bound, x_lo=xs_ - 0.01,
                       bound_hi=_hi)
        if pts_ is None:
            print(f'WARNING: {nm}: no clear port approach -- linear morph')
            placed_f.extend(port_placed.get(nm, ()))
            continue
        port_path[nm] = [p_ for p_ in _pts_of(nm) if p_[0] < xs_ - 1e-6] \
            + pts_
        placed_f.extend(zip(pts_, pts_[1:]))
        print(f'{nm}: port approach {len(pts_)} pts from '
              f'({xs_:.2f},{ys_:.2f}) to ({bx_:.2f},{by_:.2f})')

    # geometry-aware via placement: slide each window edge east until
    # the via clears static copper (both layers, via-pad inflated),
    # already-placed vias, and every other net's ACTUAL lane at that x
    vpad2 = (VIA_SIZE - TRACK) / 2
    placed_wv = []
    _trunk_cache = {}

    def trunk_pts(nm):
        if nm not in _trunk_cache:
            _trunk_cache[nm] = _pts_of(nm)
        return _trunk_cache[nm]

    _via_dbg = os.environ.get('VIA_DEBUG', '')

    def via_clear(d, x, force_margin=None):
        vy = y_at(d, x)
        nid = byname[d][0]
        dbg = (d == _via_dbg)
        for layer in ('F.Cu', 'B.Cu'):
            o = obs_cache.get((nid, layer))
            if o is None:
                o = build_obstacles(pcb, nid, kids, layer)
                obs_cache[(nid, layer)] = o
            vv = o.point_violation((x, vy), pad=vpad2)
            if vv and vv[0] > 0:
                if dbg:
                    print(f'  via_dbg {d} x={x:.2f} y={vy:.2f}: static '
                          f'{layer} {sorted(o.hugs([(x, vy)], slack=vpad2))}')
                return False
        for (px, pyy) in placed_wv:
            if math.hypot(x - px, vy - pyy) < VIA_SIZE + 0.12:
                if dbg:
                    print(f'  via_dbg {d} x={x:.2f} y={vy:.2f}: via at '
                          f'({px:.2f},{pyy:.2f})')
                return False
        # tighter margin on the FLAT launch legs (no octify deviation)
        margin = force_margin or (0.31 if x < x0 + W else 0.36)
        for om in names:
            if om == d:
                continue
            pts = trunk_pts(om)
            for a_, b_ in zip(pts, pts[1:]):
                if max(a_[0], b_[0]) < x - 1.0 or \
                        min(a_[0], b_[0]) > x + 1.0:
                    continue
                dd = ts.seg_pt_dist(a_, b_, (x, vy))
                if dd < margin:
                    if dbg:
                        print(f'  via_dbg {d} x={x:.2f} y={vy:.2f}: lane '
                              f'{om} d={dd:.3f} < {margin}')
                    return False
        return True

    obs_cache = {}
    for d in sorted(window, key=lambda d: window[d][0]):
        if d in birth_b:
            wa, wb = window[d][0], window[d][1]
            print(f'{d}: window ({wa:.2f}, '
                  f'{wb if wb is None else round(wb, 2)}) [B-birth]')
            continue
        win = window[d]
        wa, wb = win[0], win[1]
        x_ad = win[2] if len(win) == 3 else None
        s1 = first.get(d)
        wa_max = x0 + (s1 + 0.5) * W - 0.15 if s1 is not None else \
            x1 - 0.3
        x_ = wa
        while not via_clear(d, x_) and x_ < wa_max:
            x_ += 0.10
        if not via_clear(d, x_):
            # no room east of the formula spot: back WEST onto the flat
            # launch leg -- but never west of the foreign-pass clamp
            # (a passed diver must be F while it is being passed)
            x_ = min(wa, wa_max)
            wa_min = max(ends[d][0][0] + 0.05,
                         wa_lo_map.get(d, x0 - 0.05))
            while not via_clear(d, x_) and x_ > wa_min:
                x_ -= 0.10
        if not via_clear(d, x_):
            # desperate: re-sweep at the hard margin, never past the
            # own-first-crossing bound
            x_ = max(min(wa, wa_max), wa_lo_map.get(d, x0 - 0.05))
            while not via_clear(d, x_, force_margin=0.30) and x_ < wa_max:
                x_ += 0.08
        if not via_clear(d, x_, force_margin=0.30):
            # last resort: the FLAT launch leg at the true clearance
            # floor (0.2885 + 0.5um) -- flat legs never octify-deviate.
            # NEVER west of the foreign-pass clamp: a diver that is
            # passed must be on F there, and putting its dive via on
            # the launch leg anyway is a B-on-B crossing (K21 SDQ13).
            wa_min = max(ends[d][0][0] + 0.05,
                         wa_lo_map.get(d, x0 - 0.05))
            if wa_min <= x0 + W - 0.05:
                x_ = min(wa_max, x0 + W - 0.05)
                while not via_clear(d, x_, force_margin=0.289) and \
                        x_ > wa_min:
                    x_ -= 0.06
            else:
                print(f'WARNING: {d}: no legal dive-via spot in '
                      f'[{wa_min:.2f}, {wa_max:.2f}] -- keeping '
                      f'{x_:.2f} at reduced margin')
        placed_wv.append((x_, y_at(d, x_)))
        wa = x_
        if wb is not None and (len(win) > 2 or win[1] is not None):
            sk = last.get(d)
            wb_min = x0 + (sk + 1.5) * W + 0.15 if sk is not None else wa
            x_ = max(wb, wb_min)
            wb_max = (ends[d][1][0] - 0.35) if wb > x1 else x1 - 0.15
            wb_max = min(wb_max, wb_hi_map.get(d, wb_max))
            while not via_clear(d, x_) and x_ < wb_max:
                x_ += 0.10
            placed_wv.append((x_, y_at(d, x_)))
            wb = x_
        window[d] = (wa, wb) if x_ad is None else (wa, wb, x_ad)
        print(f'{d}: window {tuple(round(v, 2) for v in window[d] if v is not None)}'
              f'{" +dogbone" if entry[d][0] == "B" else ""}')

    # per-net geometry + layered segments + vias
    out_segs, out_vias = {}, {}
    for nm in names:
        mode, v = entry[nm]
        trunk = [ends[nm][0]]
        for s in range(n + 1):
            xm = x0 + (s + 0.5) * W
            trunk.append((xm, slot(morph_t(xm), orders[s].index(nm))))
        bx, by = ends[nm][1][0], ends[nm][1][1]
        if mode == 'F' and a.dest_stubs:
            # trunk lane (pitch-floored) to the splice, 45 degree jog
            # onto the stub end
            ly_ = py[trank[nm]]
            jog_ = abs(ly_ - by)
            path = trunk + [(x1, ly_)] + \
                ([(bx - jog_, ly_)] if jog_ > 1e-6 else []) + [(bx, by)]
        elif mode == 'F':
            sy = v
            path = trunk + [(x1, sy), (bx, sy), (bx, by)]
        elif mode == 'P':
            ly_, _bx = v
            if nm in port_path:
                path = list(port_path[nm])
            else:
                path = trunk + [(x1, ly_), (bx, ly_), (bx, by)]
        elif mode in ('N', 'S'):
            sx, ry, ly, dx_ = v
            path = [p_ for p_ in trunk if p_[0] < dx_ - 0.05] + \
                [(dx_, ly), (dx_, ry), (sx, ry), (sx, by), (bx, by)]
        else:
            sx, sy = v
            ly_ = lane_y[nm]
            path = trunk + [(x1, ly_)] + \
                octi45((x1, ly_), (x1 + abs(sy - ly_) + 0.05, sy))[1:] + \
                [(sx, sy)]
        path = [p for i, p in enumerate(path)
                if i == 0 or p != path[i - 1]]
        if nm not in window:
            out_segs[nm] = [(p, q, 'F.Cu') for p, q in zip(path, path[1:])]
            out_vias[nm] = []
            continue
        win = window[nm]
        wa, wb = win[0], win[1]
        x_ad = win[2] if len(win) == 3 else None
        aug, vias = [path[0]], []
        cuts = [xv for xv in (wa, wb, x_ad) if xv is not None]
        for q in path[1:]:
            p = aug[-1]
            for xv in cuts:
                if p[0] < xv < q[0]:
                    tt = (xv - p[0]) / (q[0] - p[0])
                    vp = (xv, p[1] + tt * (q[1] - p[1]))
                    aug.append(vp)
                    vias.append(vp)
            aug.append(q)
        segs = []
        for p, q in zip(aug, aug[1:]):
            mx = (p[0] + q[0]) / 2
            if wb is None:
                on_b = wa < mx
            elif x_ad is not None:
                on_b = (wa < mx < wb) or (mx > x_ad)
            else:
                on_b = wa < mx < wb
            segs.append((p, q, 'B.Cu' if on_b else 'F.Cu'))
        if mode == 'B':
            vias.append((v[0], v[1]))    # surface via (dogbone or VIP)
            if (v[0], v[1]) != (bx, by):
                segs.append(((v[0], v[1]), (bx, by), 'F.Cu'))
        elif nm in vip_nets:
            vias.append((bx, by))        # via-in-pad tail surface
        out_segs[nm] = segs
        out_vias[nm] = vias

    obs_cache = {}

    def obs_for(nid, layer):
        if (nid, layer) not in obs_cache:
            obs_cache[(nid, layer)] = build_obstacles(pcb, nid, kids, layer)
        return obs_cache[(nid, layer)]

    def run_verify(quiet=False):
        hits = []
        bad = 0

        def note(msg, x, *nets):
            nonlocal bad
            bad += 1
            hits.append((nets, x))
            if not quiet:
                print(msg)

        for nm in names:
            nid, _ = byname[nm]
            for (p, q, layer) in out_segs[nm]:
                o = obs_for(nid, layer)
                if not o.seg_clear(p, q):
                    note(f'  STATIC HIT {nm} {layer} '
                         f'({p[0]:.2f},{p[1]:.2f})->'
                         f'({q[0]:.2f},{q[1]:.2f}) '
                         f'{sorted(o.hugs([p, q], slack=0.0))}',
                         (p[0] + q[0]) / 2, nm)
        # pairwise same-layer proximity via a coarse grid hash (was
        # O(total_segs^2) -- the wall-clock hog at K11+)
        min_cc = TRACK + 0.1
        cell = 1.0

        def cells_of(p, q, r=0.25):
            for gx in range(int((min(p[0], q[0]) - r) // cell),
                            int((max(p[0], q[0]) + r) // cell) + 1):
                for gy in range(int((min(p[1], q[1]) - r) // cell),
                                int((max(p[1], q[1]) + r) // cell) + 1):
                    yield (gx, gy)

        gidx = {}
        for nm in names:
            for k, (p, q, la) in enumerate(out_segs[nm]):
                for c_ in cells_of(p, q):
                    gidx.setdefault((la, c_), []).append((nm, k, p, q))
        checked = set()
        for nm in names:
            for k, (p, q, la) in enumerate(out_segs[nm]):
                for c_ in cells_of(p, q):
                    for (om, k2, c1, e1) in gidx.get((la, c_), ()):
                        if om == nm:
                            continue
                        key = ((nm, k, om, k2) if nm < om
                               else (om, k2, nm, k))
                        if key in checked:
                            continue
                        checked.add(key)
                        d = ts.seg_seg_dist(p, q, c1, e1)
                        if d < min_cc:
                            note(f'  PAIR HIT {nm}/{om} {la} '
                                 f'd={d:.3f} near ({p[0]:.2f},{p[1]:.2f})',
                                 (p[0] + q[0]) / 2, nm, om)
        extra = (VIA_SIZE - TRACK) / 2
        for nm in names:
            nid, _ = byname[nm]
            for (vx, vy) in out_vias[nm]:
                for layer in ('F.Cu', 'B.Cu'):
                    vv = obs_for(nid, layer).point_violation((vx, vy),
                                                             pad=extra)
                    if vv and vv[0] > 0:
                        note(f'  VIA HIT {nm} {layer} '
                             f'({vx:.2f},{vy:.2f}) depth={vv[0]:.3f}',
                             vx, nm)
                for om in names:
                    if om == nm:
                        continue
                    for (p, q, _l) in out_segs[om]:
                        if ts.seg_pt_dist(p, q, (vx, vy)) < \
                                VIA_SIZE / 2 + TRACK / 2 + 0.1:
                            note(f'  VIA/TRACK HIT {nm} via '
                                 f'({vx:.2f},{vy:.2f}) vs {om}',
                                 vx, nm, om)
                    for (ox, oy) in out_vias[om]:
                        if math.hypot(vx - ox, vy - oy) < VIA_SIZE + 0.1:
                            if nm < om:
                                note(f'  VIA/VIA HIT {nm}/{om} '
                                     f'({vx:.2f},{vy:.2f})', vx, nm, om)
        return bad, hits

    # octilinearize with verifier-driven LOCAL refinement: only the
    # x-windows around a violation get a tighter deviation tube (the
    # octified path converges to the raw geometry there, which verified
    # clean); the rest of the drawing keeps the pretty 0.05 tube
    # SOUTH RIVER builder: nested B.Cu runs below the teeth line
    # (deepest run = first turn, so descents nest and never cross),
    # one via at each turn, then an F.Cu street climb through the
    # field -- legally crossing the other B runs -- and a half-pitch
    # jog onto the ball. Zero same-layer crossings, one via per net.
    if river2:
        # tooth layer + descent x (F-toothed nets jog east before their
        # tooth via when a B-tooth descent sits within 0.28)
        tooth_layer = {}
        for nm in river2:
            nid2 = byname[nm][0]
            tp = ends[nm][0]
            tooth_layer[nm] = next(
                (s.layer for s in pcb.segments if s.net_id == nid2
                 and (abs(s.start_x - tp[0]) + abs(s.start_y - tp[1])
                      < 0.005 or
                      abs(s.end_x - tp[0]) + abs(s.end_y - tp[1])
                      < 0.005)), 'B.Cu')
        base = max(ends[nm][0][1] for nm in river2) + 0.80
        _deep = base + (len(river2) - 1) * 0.3

        def _desc_ok(nm, dx0):
            """dest-stubs: an F tooth's jog, tooth via and B descent
            must clear static copper. K21: SCKE1's tooth sits on the
            west launch line below the field, and the formula via at
            (tooth x, tooth y + 0.3) landed ON SA6's B tooth."""
            if not a.dest_stubs or tooth_layer[nm] != 'F.Cu':
                return True
            nid_ = byname[nm][0]
            if nm not in obsF:
                obsF[nm] = build_obstacles(pcb, nid_, kids, 'F.Cu')
            if nm not in obsB:
                obsB[nm] = build_obstacles(pcb, nid_, kids, 'B.Cu')
            tp = ends[nm][0]
            vp = (dx0, tp[1] + 0.30)
            fj = octi45(tp, vp)
            if not all(obsF[nm].seg_clear(p_, q_)
                       for p_, q_ in zip(fj, fj[1:]) if p_ != q_):
                return False
            for o in (obsF[nm], obsB[nm]):
                vv = o.point_violation(vp, pad=(VIA_SIZE - TRACK) / 2)
                if vv and vv[0] > 0:
                    return False
            return obsB[nm].seg_clear(vp, (dx0, _deep))

        desc_x = {}
        for nm in sorted(river2, key=lambda n: (tooth_layer[n] != 'B.Cu',
                                                ends[n][0][0])):
            dx0 = ends[nm][0][0]
            while any(abs(dx0 - v) < 0.28 for v in desc_x.values()) or \
                    not _desc_ok(nm, dx0):
                dx0 += 0.30
                assert dx0 < ends[nm][0][0] + 3.0, f'no descent for {nm}'
            desc_x[nm] = dx0
        r2 = sorted(river2, key=lambda nm: desc_x[nm])
        # dest-stubs: a stub end the fanout left on B.Cu (via-in-pad
        # escape) is joined from the F climb through a second via 0.3
        # below it -- the end itself may share x-y with a neighbour's
        # F stub end (K21: SRAS under SCKE1 in one street)
        stub_layer = {}
        for nm in r2:
            nid2 = byname[nm][0]
            tp = ends[nm][1]
            stub_layer[nm] = next(
                (s.layer for s in pcb.segments if s.net_id == nid2
                 and (abs(s.start_x - tp[0]) + abs(s.start_y - tp[1])
                      < 0.005 or
                      abs(s.end_x - tp[0]) + abs(s.end_y - tp[1])
                      < 0.005)), 'F.Cu') if a.dest_stubs else 'F.Cu'
        # order-free street assignment (climbs are on F.Cu, so any turn
        # permutation is legal; only spacing matters). Contested columns
        # overflow to FAR streets (+-1.2/+-2.0) with a row-street run
        # leg on the ball row's south or north side.
        turns = {}
        street_runs = []       # (y, xlo, xhi) of assigned run legs

        def run_free(y_, a_, b_):
            lo, hi = min(a_, b_) - 0.25, max(a_, b_) + 0.25
            return all(abs(y_ - ry) > 0.25 or hi < rl or lo > rh
                       for (ry, rl, rh) in street_runs)

        deep = base + (len(r2) - 1) * 0.3
        for nm in sorted(r2, key=lambda n: ends[n][1][0]):
            bx, by = ends[nm][1][0], ends[nm][1][1]
            cands = [(bx - half, None), (bx + half, None)]
            if a.dest_stubs:
                # the target is a stub end already OUTSIDE the field
                # (south face): climb straight up its own column
                cands.insert(0, (bx, None))
            for off in (1.5 * a.pitch, 2.5 * a.pitch):
                for side in (by + half, by - half):
                    cands.append((bx - off, side))
                    cands.append((bx + off, side))
            # turn-via x spacing: 0.45 on the bench (balls a pitch
            # apart, so it never binds); with stub ends the fanout
            # packs south exits at 0.38-0.42, and the turn vias sit on
            # DIFFERENT run ys (0.3 apart), so 0.30 in x is already
            # 0.42 centre-to-centre (via-via needs 0.35) and 0.30 to
            # the neighbour's climb (via-track needs 0.2885).
            tsp = 0.30 if a.dest_stubs else 0.45
            for tx_, ry_ in cands:
                if any(abs(tx_ - v[0]) < tsp for v in turns.values()):
                    continue
                if ry_ is not None and not run_free(ry_, tx_, bx):
                    continue
                if a.dest_stubs:
                    # the climb must clear static copper and the west
                    # braid's south-port runs (bench path unchanged: it
                    # never checked, and its K4..K19 rungs are pinned)
                    if ry_ is None:
                        climb = [((tx_, deep), (tx_, by))]
                    else:
                        climb = [((tx_, deep), (tx_, ry_)),
                                 ((tx_, ry_), (bx, ry_)), ((bx, ry_), (bx, by))]
                    if nm not in obsF:
                        obsF[nm] = build_obstacles(pcb, byname[nm][0],
                                                   kids, 'F.Cu')
                    if not f_ok(nm, climb):
                        continue
                    placed_f.extend(climb)
                turns[nm] = (tx_, ry_)
                if ry_ is not None:
                    street_runs.append((ry_, min(tx_, bx), max(tx_, bx)))
                break
            assert nm in turns, f'no south street for {nm}'
        for i, nm in enumerate(r2):
            run_y = base + (len(r2) - 1 - i) * 0.3
            tooth2 = ends[nm][0]
            bx, by = ends[nm][1][0], ends[nm][1][1]
            tx_, ry_ = turns[nm]
            dx0 = desc_x[nm]
            segs2 = []
            vias2 = []
            if tooth_layer[nm] == 'B.Cu':
                start = tooth2
            else:
                # F tooth: short F jog to the descent x, tooth via to B
                vp = (dx0, tooth2[1] + 0.30)
                fj = octi45(tooth2, vp)
                segs2 += [(p_, q_, 'F.Cu')
                          for p_, q_ in zip(fj, fj[1:]) if p_ != q_]
                vias2.append(vp)
                start = vp
            pts_b = [start, (dx0, run_y - 0.15), (dx0 + 0.15, run_y),
                     (tx_, run_y)]
            tail_b = []
            if stub_layer[nm] == 'B.Cu' and ry_ is None and tx_ == bx:
                vp2 = (bx, by + 0.30)
                pts_f = [(tx_, run_y), vp2]
                vias2.append(vp2)
                tail_b = [(vp2, (bx, by))]
            elif ry_ is None:
                pts_f = [(tx_, run_y), (tx_, by), (bx, by)]
            else:
                pts_f = [(tx_, run_y), (tx_, ry_), (bx, ry_), (bx, by)]
            vias2.append((tx_, run_y))
            segs2 += [(p_, q_, 'B.Cu')
                      for p_, q_ in zip(pts_b, pts_b[1:]) if p_ != q_]
            segs2 += [(p_, q_, 'F.Cu')
                      for p_, q_ in zip(pts_f, pts_f[1:]) if p_ != q_]
            segs2 += [(p_, q_, 'B.Cu') for p_, q_ in tail_b]
            out_segs[nm] = segs2
            out_vias[nm] = vias2
        names = all_names

    if not a.no_octi:
        raw_segs = {nm: list(out_segs[nm]) for nm in names}
        windows = {nm: [] for nm in names}
        fine_eps = 0.02
        for it in range(6):
            for nm in names:
                f = (windows[nm], fine_eps) if windows[nm] else None
                out_segs[nm] = octify_segs(raw_segs[nm], 0.05, f)
            bad, hits = run_verify(quiet=True)
            if not bad:
                break
            for nets_, x in hits:
                for nm in nets_:
                    windows[nm].append((x - 0.5, x + 0.5))
            print(f'  octify pass {it}: {bad} violations, refining '
                  f'{sorted(set(n for ns, _ in hits for n in ns))} '
                  f'at fine eps {fine_eps}')
            fine_eps = max(fine_eps / 2.5, 0.003)
        nw = sum(len(w) for w in windows.values())
        print(f'octilinearized: '
              f'{sum(len(s) for s in out_segs.values())} segments '
              f'({nw} fine windows)')
    if a.dump_segs:
        import json
        with open(a.dump_segs, 'w') as f:
            json.dump({nm: [[p, q, l] for (p, q, l) in out_segs[nm]]
                       for nm in names}, f)

    print('\nverification:')
    bad, _off = run_verify()
    print(f'  {"CLEAN" if not bad else str(bad) + " violations"}')

    # repo octolinear smoothing (#536): collapse the distributed 45
    # nudges into single elbows and merge our launch legs with the
    # fanout stubs. Every collapse is clearance-validated against ALL
    # copper and connectivity-guarded per net; it splices
    # pcb_data.segments in place, so the final geometry is read back
    # from there.
    smoothed = False
    if not a.no_smooth:
        from kicad_parser import Segment, Via
        from pcb_modification import smooth_octolinear_chains
        pre_len = {nm: sum(math.hypot(q[0] - p[0], q[1] - p[1])
                           for (p, q, _l) in out_segs[nm]) for nm in names}
        for nm in names:
            nid, _ = byname[nm]
            for (p, q, layer) in out_segs[nm]:
                pcb.segments.append(Segment(p[0], p[1], q[0], q[1],
                                            TRACK, layer, nid))
            for (vx, vy) in out_vias[nm]:
                pcb.vias.append(Via(vx, vy, VIA_SIZE, VIA_DRILL,
                                    ['F.Cu', 'B.Cu'], nid))
        _n, _nets, _rm, _addl, st = smooth_octolinear_chains(
            [], pcb, kids, clearance=0.1)
        final_segs = {}
        for nm in names:
            nid, _ = byname[nm]
            final_segs[nm] = [s for s in pcb.segments if s.net_id == nid]
        post_len = {nm: sum(math.hypot(s.end_x - s.start_x,
                                       s.end_y - s.start_y)
                            for s in final_segs[nm]) for nm in names}
        print(f'\nsmooth_octolinear_chains (#536): '
              f'{st.get("spans", 0)} spans on {_nets} nets, '
              f'-{st.get("saved_mm", 0):.2f} mm; segments '
              f'{sum(len(s) for s in out_segs.values())} -> '
              f'{sum(len(s) for s in final_segs.values())}; length '
              f'{sum(pre_len.values()):.2f} -> '
              f'{sum(post_len.values()):.2f} mm')
        smoothed = True

    if vip_nets:
        print(f'VIA-IN-PAD berths (IPC-4761 Type VII filled+capped '
              f'required): {sorted(set(vip_nets))}')
    if a.plan_out:
        import json as _json
        plan = {}
        for nm in names:
            if nm in river2:
                plan[nm] = {'river': 'south'}
                continue
            plan[nm] = {
                'river': 'west',
                'home': 'B' if nm in divers else 'F',
                'tooth_layer': wtooth_layer.get(nm, 'F.Cu'),
                'diver': nm in divers,
                'birth_b': nm in birth_b,
                'vip': nm in vip_nets,
                'entry': entry.get(nm, ('?',))[0],
            }
        # fanout wishlist: divers whose stubs are still F and who are
        # not mutually inverted with an existing B-birth diver
        wish = []
        for d in sorted(divers, key=lambda n: trank.get(n, 99)):
            if wtooth_layer.get(d) != 'F.Cu':
                continue
            if all(not inverted(d, e) for e in
                   list(birth_b) + wish if e != d):
                wish.append(d)
        for d in wish:
            plan[d]['fanout_wish'] = 'B'
        with open(a.plan_out, 'w') as f:
            _json.dump(plan, f, indent=1)
        print(f'plan exported: {a.plan_out} '
              f'(fanout B wishlist: {wish})')

    # write board
    txt = open(a.board, encoding='utf-8').read()
    add = []
    if smoothed:
        kid_names = {pcb.nets[i].name for i in kids if i in pcb.nets}
        txt = strip_net_segments(txt, kids, kid_names)
        # The octify / #536 smoothing passes leave a few DEGENERATE
        # segments -- measured at K11 and K21: three of 0.4 um and one
        # of 2 um. They are far below any manufacturing resolution, and
        # a 0.4 um segment between two 127 um tracks is an overlap, not
        # a link -- but they are junk in the output, they inflate the
        # segment count, and a structural audit reads the pair of them
        # meeting at one point as a BRANCH in an otherwise clean chain.
        # Drop anything under 1 um; the neighbours already overlap.
        # ...and a few DUPLICATES: the same 2 um segment emitted twice,
        # which reads as a degree-4 vertex -- a BRANCH -- in a chain that
        # is actually clean. Dedup is strictly safe (dropping a copy
        # cannot disconnect anything); the length drop is not, so it
        # stays at 1 um where the neighbours already overlap.
        n_deg = n_dup = 0
        for nm in names:
            keep = []
            seen = set()
            for s in final_segs[nm]:
                if math.hypot(s.end_x - s.start_x,
                              s.end_y - s.start_y) < 0.001:
                    n_deg += 1
                    continue
                k = (round(s.start_x, 4), round(s.start_y, 4),
                     round(s.end_x, 4), round(s.end_y, 4), s.layer)
                kr = (k[2], k[3], k[0], k[1], k[4])
                if k in seen or kr in seen:
                    n_dup += 1
                    continue
                seen.add(k)
                keep.append(s)
            final_segs[nm] = keep
        if n_deg or n_dup:
            print(f'dropped {n_deg} degenerate (< 1 um) and {n_dup} '
                  f'duplicate segment(s)')
        for nm in names:
            nid, _ = byname[nm]
            for s in final_segs[nm]:
                (ax, ay), (bx_, by_) = back_xy(s.start_x, s.start_y), \
                    back_xy(s.end_x, s.end_y)
                add.append(
                    f'  (segment (start {ax:.4f} {ay:.4f}) '
                    f'(end {bx_:.4f} {by_:.4f}) '
                    f'(width {s.width}) '
                    f'(layer "{s.layer}") (net {nid}))\n')
            for (vx, vy) in out_vias[nm]:
                vx, vy = back_xy(vx, vy)
                add.append(
                    f'  (via (at {vx:.4f} {vy:.4f}) (size {VIA_SIZE}) '
                    f'(drill {VIA_DRILL}) (layers "F.Cu" "B.Cu") '
                    f'(net {nid}))\n')
            for s in final_segs[nm]:
                (ax, ay), (bx_, by_) = back_xy(s.start_x, s.start_y), \
                    back_xy(s.end_x, s.end_y)
                add.append(
                    f'  (gr_line (start {ax:.4f} {ay:.4f}) '
                    f'(end {bx_:.4f} {by_:.4f}) '
                    f'(stroke (width 0.05) (type solid)) '
                    f'(layer "Eco2.User"))\n')
    else:
        for nm in names:
            nid, _ = byname[nm]
            for (p, q, layer) in out_segs[nm]:
                (ax, ay), (bx_, by_) = back_xy(*p), back_xy(*q)
                add.append(
                    f'  (segment (start {ax:.4f} {ay:.4f}) '
                    f'(end {bx_:.4f} {by_:.4f}) (width {TRACK}) '
                    f'(layer "{layer}") (net {nid}))\n')
            for (vx, vy) in out_vias[nm]:
                vx, vy = back_xy(vx, vy)
                add.append(
                    f'  (via (at {vx:.4f} {vy:.4f}) (size {VIA_SIZE}) '
                    f'(drill {VIA_DRILL}) (layers "F.Cu" "B.Cu") '
                    f'(net {nid}))\n')
            for (p, q, _l) in out_segs[nm]:
                (ax, ay), (bx_, by_) = back_xy(*p), back_xy(*q)
                add.append(
                    f'  (gr_line (start {ax:.4f} {ay:.4f}) '
                    f'(end {bx_:.4f} {by_:.4f}) '
                    f'(stroke (width 0.05) (type solid)) '
                    f'(layer "Eco2.User"))\n')
    k = txt.rstrip().rfind(')')
    out_board = a.out + '.kicad_pcb'
    with open(out_board, 'w') as f:
        f.write(txt[:k] + ''.join(add) + txt[k:])
    pro = os.path.splitext(a.board)[0] + '.kicad_pro'
    if os.path.exists(pro):
        shutil.copy(pro, a.out + '.kicad_pro')
    nv = sum(len(v) for v in out_vias.values())
    nseg = sum(len(final_segs[nm]) for nm in names) if smoothed else \
        sum(len(s) for s in out_segs.values())
    print(f'\nwrote {out_board}: {nseg} segments, {nv} vias')


if __name__ == '__main__':
    main()

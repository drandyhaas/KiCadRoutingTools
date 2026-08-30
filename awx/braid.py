#!/usr/bin/env python3
"""braid.py -- the general braid emitter for a fanned-out bus corridor.

Inputs: a board whose SOURCE and DESTINATION arrays are both fanned out,
the nets to route, and the destination's reference. Everything else is
read off the board's geometry in the FLOW FRAME (source due west).

The braid decides ORDER and LAYERS: the launch order is the source's
tooth order, the target order is the stub-end order at the destination,
the divers are the complement of the longest increasing subsequence of
that permutation, and the wave schedule turns the inversions into
adjacent swaps in a trunk of morphing lanes -- each swap with exactly
one side on the back layer. That part is pure combinatorics plus the
lane geometry it implies.

Every join between the trunk and a stub end -- a west lane onto its
stub, a port net onto a stub the plan sent out the flank face, and BOTH
ends of the flank corridor (the "river") -- is a CONNECTION made by the
real router (connect.py): routed against the production obstacle model,
with the via placed by the search where a layer changes. There are no
hand tails and no fallbacks: a refused connection is reported as such.

Two declared capability limits, not board facts: the trunk delivers to
the destination face that faces the source, and the flank corridor is
the SOUTH one in the flow frame (a plan that sends a net out the north
or east face has no corridor here). The corridor-per-face-pair
refactor lifts both.

The remaining hand mechanism is the trunk's own dive/surface via
placement (a window formula plus clearance sweeps). It is next: the
scheduler needs to hear back from via placement, and the placement
belongs to the router.
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
import connect as cn  # noqa: E402

TRACK = 0.127
CLEAR = 0.105            # 0.1 spec + 5um so hugs don't sit exactly at 0.1
VIA_SIZE = 0.25
VIA_DRILL = 0.15


def build_obstacles(pcb, nid, kids, layer):
    """The braid's own static-copper model for one net on one layer:
    every foreign pad as a disc, every foreign segment as a capsule,
    every foreign via as a disc, all inflated by clearance + half a
    track. Used for the trunk's via placement and the diagnostic
    verifier; connections are routed against the router's model."""
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


def _owner(pt, segs, pads):
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
    """Where each net's corridor must start and finish.

    Default (dest_ref None): the source stub's free end, to the far
    PAD -- what the plan works from before the destination is fanned
    out. With dest_ref: both ends are fanned out and each net has a
    free end at both; the pair is attributed by walking the net's own
    copper (nearest-pad is wrong for a stub dragged across a field).
    A free end is a segment endpoint used once that lies inside no pad
    and no via barrel of the net."""
    ends = {}
    for nm in names:
        nid, net = byname[nm]
        segs = [s for s in pcb.segments if s.net_id == nid]
        cnt = Counter()
        for s in segs:
            cnt[(round(s.start_x, 3), round(s.start_y, 3))] += 1
            cnt[(round(s.end_x, 3), round(s.end_y, 3))] += 1
        anchors = [(p.global_x, p.global_y, max(p.size_x, p.size_y) / 2)
                   for p in net.pads] + \
            [(v.x, v.y, v.size / 2) for v in pcb.vias if v.net_id == nid]
        free = [pt for pt, c in cnt.items() if c == 1 and
                all(math.hypot(pt[0] - ax, pt[1] - ay) > max(0.02, ar)
                    for (ax, ay, ar) in anchors)]
        assert free, (nm, 'no free stub end')
        if dest_ref is not None:
            at_dest = [pt for pt in free
                       if _owner(pt, segs, net.pads) == dest_ref]
            at_src = [pt for pt in free if pt not in at_dest]
            if at_dest and at_src:
                src = max(at_src, key=lambda p: max(ts.d2(p, q)
                                                    for q in at_dest))
                tgt = max(at_dest, key=lambda q: ts.d2(src, q))
                ends[nm] = (src, tgt, dest_ref)
                continue
            free = at_src or free
        if len(free) > 1:
            free.sort(key=lambda pt: -min(ts.d2(pt, (a[0], a[1]))
                                          for a in anchors))
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
    equally-long ones the one of greatest total weight (the plan uses
    it to prefer keepers whose escape starts on the tooth layer)."""
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
    stay within ~eps of the original."""
    dx, dy = q[0] - p[0], q[1] - p[1]
    adx, ady = abs(dx), abs(dy)
    if adx < 1e-9 or ady < 1e-9 or abs(adx - ady) < 1e-9:
        return [p, q]
    m = min(adx, ady)
    r = abs(adx - ady)
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
    run, RDP-simplify, then octify each remaining segment. `fine` =
    (windows, fine_eps): segments overlapping a window x-range use the
    tighter tube (verifier-driven local refinement)."""
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
    EITHER dialect: numeric (net N) or quoted name (#749 lore: boards
    carry both). Paren-balanced."""
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


def _layer_at(pcb, nid, pt, default):
    return next(
        (s.layer for s in pcb.segments if s.net_id == nid
         and (abs(s.start_x - pt[0]) + abs(s.start_y - pt[1]) < 0.005
              or abs(s.end_x - pt[0]) + abs(s.end_y - pt[1]) < 0.005)),
        default)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--board', required=True,
                    help='board with BOTH arrays fanned out')
    ap.add_argument('--nets', required=True, help='comma-separated names')
    ap.add_argument('--out', required=True, help='output stem')
    ap.add_argument('--dest', required=True, metavar='REF',
                    help='destination component (its stub ends are '
                         'the targets)')
    ap.add_argument('--no-smooth', action='store_true',
                    help='skip the repo #536 octolinear smoothing pass')
    ap.add_argument('--no-octi', action='store_true',
                    help='emit the raw arbitrary-angle trunk geometry')
    a = ap.parse_args()
    names = [n.strip() for n in a.nets.split(',') if n.strip()]

    pcb = parse_kicad_pcb(a.board)
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    # FLOW FRAME: everything below assumes the source is due west of the
    # destination. Rotate the board so that it is, then map the emitted
    # copper back at write time. KICAD_FLOW_FRAME=0 is the negative
    # control that proves the frame is needed.
    _probe = endpoints(pcb, names, byname, dest_ref=a.dest)
    _theta = ff.flow_angle([_probe[n][0] for n in _probe],
                           [_probe[n][1] for n in _probe])
    if os.environ.get('KICAD_FLOW_FRAME') == '0':
        _theta = 0.0
    _cx = sum(_probe[n][1][0] for n in _probe) / max(len(_probe), 1)
    _cy = sum(_probe[n][1][1] for n in _probe) / max(len(_probe), 1)
    pcb, back_xy = ff.rotate_pcb(pcb, _theta, _cx, _cy)
    if _theta:
        print(f'flow frame: source is not due west -- rotating the '
              f'problem by {_theta:.0f} deg about ({_cx:.2f},{_cy:.2f})')
        byname = {n.name.split('/')[-1]: (i, n)
                  for i, n in pcb.nets.items()}
    kids = {byname[nm][0] for nm in names}
    ends = endpoints(pcb, names, byname, dest_ref=a.dest)

    # the two arrays, from the endpoints
    dest_pads = pcb.footprints[a.dest].pads
    fy1 = max(p.global_y for p in dest_pads)          # destination's south row
    src_refs = set()
    for nm in names:
        nid, net = byname[nm]
        src_refs.add(min(net.pads, key=lambda p: ts.d2(
            (p.global_x, p.global_y), ends[nm][0])).component_ref)
    src_y_max = max(p.global_y for r in src_refs
                    for p in pcb.footprints[r].pads)
    tooth_layer = {nm: _layer_at(pcb, byname[nm][0], ends[nm][0], 'F.Cu')
                   for nm in names}
    dest_layer = {nm: _layer_at(pcb, byname[nm][0], ends[nm][1], 'F.Cu')
                  for nm in names}

    # FLANK CORRIDOR (the river): a tooth south of the source's own pad
    # field is on its flank, not on the launch line, and rides the
    # flank corridor -- a B.Cu run under everything, both ends made by
    # connections.
    river = [nm for nm in names if ends[nm][0][1] > src_y_max + 0.1]
    west = [nm for nm in names if nm not in river]
    if river:
        print(f'flank corridor ({len(river)}): {river}')
    assert west, 'no net on the launch line'

    # the trunk: from the launch line to the splice line, 0.6 mm west
    # of the nearest stub end so the tails have room for a 45 jog
    x0 = max(ends[nm][0][0] for nm in west) + 0.3
    x1 = min(ends[nm][1][0] for nm in west) - 0.6
    assert x1 - x0 > 1.0, ('no corridor between the teeth and the stubs',
                           x0, x1)
    launch = sorted(west, key=lambda nm: ends[nm][0][1])
    print(f'launch order: {launch}')

    # ---- entries: a west-face stub pins its lane at its own y; a
    # stub the plan sent out the SOUTH face is a PORT net, whose lane is
    # a port line below the field, nearest stub uppermost, so no port
    # run crosses an earlier net's turn-up.
    south = sorted((nm for nm in west if ends[nm][1][1] > fy1 + 0.2),
                   key=lambda nm: ends[nm][1][0])
    lane_y = {}
    is_port = set()
    for nm in west:
        if nm in south:
            is_port.add(nm)
            lane_y[nm] = fy1 + 0.55 + 0.35 * south.index(nm)
        else:
            lane_y[nm] = ends[nm][1][1]
    if south:
        print(f'south port ({len(south)}): '
              + ', '.join(f'{nm}@{ends[nm][1][0]:.2f}' for nm in south))
    target = sorted(west, key=lambda nm: lane_y[nm])
    trank = {nm: i for i, nm in enumerate(target)}
    ranks = [trank[nm] for nm in launch]
    keep = lis_keep(ranks)
    divers = {launch[i] for i in range(len(launch)) if i not in keep}
    print(f'entry order (lane y): {target}  ranks: {ranks}')
    print(f'divers ({len(divers)}): '
          f'{sorted(divers, key=lambda n: trank[n])}')

    # ---- swap schedule (wave-pipelined): divers that are pairwise
    # NON-inverted form a wave and bubble concurrently; every diver-
    # diver crossing has exactly one side on B.
    lidx = {nm: i for i, nm in enumerate(launch)}

    def inverted(anm, bnm):
        return (lidx[anm] < lidx[bnm]) != (trank[anm] < trank[bnm])

    ups = sorted((d for d in divers if trank[d] < lidx[d]),
                 key=lambda nm: trank[nm])
    downs = sorted((d for d in divers if trank[d] >= lidx[d]),
                   key=lambda nm: -trank[nm])
    dirs = {d: (-1 if trank[d] < lidx[d] else 1) for d in divers}
    # a diver whose tooth already sits on B.Cu is BORN diving -- no dive
    # via -- and must do all its diver-diver crossings as the mover, so
    # it goes to the front; mutually inverted B-born pairs are illegal
    birth_b = {d for d in divers if tooth_layer[d] == 'B.Cu'}
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

    # serial pass fixes WHO passes WHOM -- and in which DIRECTION. The
    # target-vs-launch index says where a diver ends up overall, but
    # the other divers' passes shift it on the way: rot90 K15's SDQ12
    # ends 3 slots below its launch (a "down" diver) yet must pass
    # SDQ11 going UP, once the up-divers have carried SDQ11 below it.
    # The serial pass sees the sequence as it stands when the diver
    # moves, so its direction is the one the wave must use; the global
    # one deadlocked the wave (mover facing away from its partner).
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
        if want != i:
            dirs[d] = -1 if want < i else 1
        sseq = rest[:want] + [d] + rest[want:]
    assert sseq == target, (sseq, target)

    def schedule(gap):
        """The gated wave schedule. `gap` = spacer columns a diver
        waits after being passed before its own first swap."""
        done_moves = {d: set() for d in priority}
        passed = set()
        last_passed_round = {}
        seq = list(launch)
        cols = []
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
                    continue
                seq[i], seq[j] = seq[j], seq[i]
                col.append((d, p))
                used.add(d)
                used.add(p)
                done_moves[d].add(p)
                if p in dirs:
                    passed.add((d, p))
                    last_passed_round[p] = len(cols)
            if not col and delay_stall:
                cols.append([])
                guard += 1
                continue
            assert col, ('wave schedule deadlocked', seq, target)
            cols.append(col)
            guard += 1
            assert guard < 20 * len(west) + 20, 'wave schedule runaway'
        return cols

    # the passed diver's dive via needs (gap + 0.5) W >= 0.81 of trunk
    # between the pass and its own first crossing; more spacers shrink
    # W, so iterate. RESERVE: 0.5 mm after the last column so divers
    # can surface in the trunk.
    reserve = 0.5
    gap = 1
    W_est = None
    for _ in range(4):
        cols = schedule(gap)
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

    # ---- via windows: dive/surface vias want >= 0.45 clear of the
    # nearest swap column, inside the diver's own band region. A diver
    # with no trunk room to surface after its last own swap -- and no
    # foreign crossing east that needs it on F -- stays on B to the
    # splice and the CONNECTION places the via.
    window = {}
    wa_lo_map = {}
    wb_hi_map = {}
    fe_map = {}
    for d in divers:
        if d in birth_b:
            s1 = first.get(d)
            if s1 is not None:
                assert not [s for s in invol[d] if s < s1], \
                    f'B-birth {d} passed before its band'
            wa = ends[d][0][0] - 0.05
            sk = last.get(d)
            fe = min((s for s in invol[d] if sk is None or s > sk),
                     default=None)
            fe_map[d] = fe
            wb_hi = x0 + (fe + 0.5) * W - 0.30 if fe is not None \
                else x1 - 0.15
            if sk is not None:
                wb = min(wb_hi, max(x0 + (sk + 1.6) * W,
                                    x0 + (sk + 1.5) * W + 0.45))
                if fe is None and wb < x0 + (sk + 1.5) * W + 0.28:
                    wb = None
            else:
                wb = None
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
                print(f'{d}: no trunk room to surface after its last '
                      f'swap (column {sk}) -- B to the splice, the '
                      f'connection places the via')
                wb = None
        else:
            wa = x0 + (n + 0.75) * W
            if invol.get(d):
                wa = max(wa, x0 + (max(invol[d]) + 1.5) * W + 0.10)
            wa_lo_map[d] = wa
            wb = x1 - 0.15
        fe_map[d] = fe
        window[d] = (wa, wb)

    # ---- lane geometry
    py = [lane_y[nm] for nm in target]
    assert all(b > a for a, b in zip(py, py[1:])), ('lane ys not strictly '
                                                    'increasing', py)
    # lane pitch floor at the splice: stub ends are packed at 0.25, a
    # surface via next to a lane needs 0.2885 and via_clear asks 0.36;
    # relax tight pairs apart (port lanes pinned), the tail jogs it
    MINP = 0.38
    pinned = {i for i, nm in enumerate(target) if nm in is_port}
    for _ in range(40):
        moved = False
        for i in range(len(py) - 1):
            g_ = py[i + 1] - py[i]
            if g_ < MINP - 1e-9:
                push = (MINP - g_) / 2
                if i not in pinned:
                    py[i] -= push if i + 1 not in pinned else 2 * push
                if i + 1 not in pinned:
                    py[i + 1] += push if i not in pinned else 2 * push
                moved = True
        if not moved:
            break
    jog = max(abs(py[trank[nm]] - lane_y[nm]) for nm in west)
    if jog > 1e-6:
        print(f'lane pitch floor {MINP}: max tail jog {jog:.3f} mm')
    assert jog < 0.55, ('tail jog exceeds the 0.6 mm tail', jog)
    Ly = sorted(ends[nm][0][1] for nm in west)
    _ly2 = []
    for _v in Ly:
        _ly2.append(_v if not _ly2 else max(_v, _ly2[-1] + 0.28))
    Ly = _ly2

    def slot(t, i):
        return (1 - t) * Ly[i] + t * py[i]

    def morph_t(xm):
        # launch legs stay FLAT for the first column, then the morph
        xms = x0 + W
        return max(0.0, (xm - xms) / (x1 - xms))

    orders = [list(launch)]
    for col in cols:
        o = list(orders[-1])
        for (d, p) in col:
            i, j = o.index(d), o.index(p)
            o[i], o[j] = o[j], o[i]
        orders.append(o)

    _trunk_cache = {}

    def trunk_pts(nm):
        """The net's trunk polyline: tooth, column midpoints, splice."""
        if nm not in _trunk_cache:
            pts = [ends[nm][0]]
            for s in range(n + 1):
                xm = x0 + (s + 0.5) * W
                pts.append((xm, slot(morph_t(xm), orders[s].index(nm))))
            pts.append((x1, py[trank[nm]]))
            _trunk_cache[nm] = pts
        return _trunk_cache[nm]

    def y_at(nm, x):
        pts = trunk_pts(nm)
        if x <= pts[0][0]:
            return pts[0][1]
        for a_, b_ in zip(pts, pts[1:]):
            if a_[0] <= x <= b_[0]:
                tt = (x - a_[0]) / max(b_[0] - a_[0], 1e-9)
                return a_[1] + tt * (b_[1] - a_[1])
        return pts[-1][1]

    # ---- via placement: slide each window edge east until the via
    # clears static copper (both layers), placed vias and every other
    # net's lane at that x. (The remaining hand mechanism.)
    vpad2 = (VIA_SIZE - TRACK) / 2
    placed_wv = []
    obs_cache = {}
    _via_dbg = os.environ.get('VIA_DEBUG', '')

    def obs_for(nid, layer):
        if (nid, layer) not in obs_cache:
            obs_cache[(nid, layer)] = build_obstacles(pcb, nid, kids, layer)
        return obs_cache[(nid, layer)]

    def via_clear(d, x, force_margin=None):
        vy = y_at(d, x)
        nid = byname[d][0]
        dbg = (d == _via_dbg)
        for layer in ('F.Cu', 'B.Cu'):
            vv = obs_for(nid, layer).point_violation((x, vy), pad=vpad2)
            if vv and vv[0] > 0:
                if dbg:
                    print(f'  via_dbg {d} x={x:.2f} y={vy:.2f}: static '
                          f'{layer}')
                return False
        for (px, pyy) in placed_wv:
            if math.hypot(x - px, vy - pyy) < VIA_SIZE + 0.12:
                if dbg:
                    print(f'  via_dbg {d} x={x:.2f} y={vy:.2f}: via at '
                          f'({px:.2f},{pyy:.2f})')
                return False
        margin = force_margin or (0.31 if x < x0 + W else 0.36)
        for om in west:
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

    for d in sorted(window, key=lambda d: window[d][0]):
        wa, wb = window[d]
        if d in birth_b:
            print(f'{d}: window ({wa:.2f}, '
                  f'{wb if wb is None else round(wb, 2)}) [B-birth]')
            continue
        s1 = first.get(d)
        wa_max = x0 + (s1 + 0.5) * W - 0.15 if s1 is not None else \
            x1 - 0.3
        x_ = wa
        while not via_clear(d, x_) and x_ < wa_max:
            x_ += 0.10
        if not via_clear(d, x_):
            x_ = min(wa, wa_max)
            wa_min = max(ends[d][0][0] + 0.05,
                         wa_lo_map.get(d, x0 - 0.05))
            while not via_clear(d, x_) and x_ > wa_min:
                x_ -= 0.10
        if not via_clear(d, x_):
            x_ = max(min(wa, wa_max), wa_lo_map.get(d, x0 - 0.05))
            while not via_clear(d, x_, force_margin=0.30) and x_ < wa_max:
                x_ += 0.08
        if not via_clear(d, x_, force_margin=0.30):
            # last resort: the FLAT launch leg at the clearance floor --
            # never west of the foreign-pass clamp (a passed diver must
            # be on F there)
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
        if wb is not None:
            sk = last.get(d)
            wb_min = x0 + (sk + 1.5) * W + 0.15 if sk is not None else wa
            x_ = max(wb, wb_min)
            wb_max = min(x1 - 0.15, wb_hi_map.get(d, x1 - 0.15))
            while not via_clear(d, x_) and x_ < wb_max:
                x_ += 0.10
            if fe_map.get(d) is None and \
                    not via_clear(d, x_, force_margin=0.30):
                print(f'{d}: no clear surface-via spot in the trunk -- '
                      f'B to the splice, the connection places the via')
                wb = None
            else:
                placed_wv.append((x_, y_at(d, x_)))
                wb = x_
        window[d] = (wa, wb)
        print(f'{d}: window ({wa:.2f}, '
              f'{wb if wb is None else round(wb, 2)})')

    # ---- trunk copper per net, up to its EXIT: a west lane to the
    # splice, a port net to where it leaves the trunk (after its last
    # swap column, at the column midpoint)
    out_segs, out_vias = {}, {}
    exits, exit_layer = {}, {}
    port_band = {}
    for nm in west:
        trunk = trunk_pts(nm)
        if nm in is_port:
            smax = max(invol.get(nm, [-1]))
            xs_ = x0 + W if smax < 0 else x0 + (smax + 1.5) * W
            ys_ = y_at(nm, xs_)
            path = [p_ for p_ in trunk if p_[0] < xs_ - 1e-6] + [(xs_, ys_)]
            exits[nm] = (xs_, ys_)
            above = [om for om in west if trank[om] < trank[nm]]
            below = [om for om in west if trank[om] > trank[nm]]

            # band: below every trunk lane that ends above it, above
            # every one that ends below it -- lanes only (x <= x1);
            # everything east of the splice is copper by the time a
            # port net is connected (it comes last in target order)
            def _lo(x, _ab=above):
                if x > x1 or not _ab:
                    return -1e9
                return max(y_at(om, x) for om in _ab) + 0.26

            def _hi(x, _be=below):
                if x > x1 or not _be:
                    return 1e9
                return min(y_at(om, x) for om in _be) - 0.26
            port_band[nm] = (_lo, _hi)
        else:
            path = trunk
            exits[nm] = (x1, py[trank[nm]])
        path = [p for i, p in enumerate(path)
                if i == 0 or p != path[i - 1]]
        if nm not in window:
            out_segs[nm] = [(p, q, 'F.Cu') for p, q in zip(path, path[1:])]
            out_vias[nm] = []
            exit_layer[nm] = 'F.Cu'
            continue
        wa, wb = window[nm]
        aug, vias = [path[0]], []
        cuts = [xv for xv in (wa, wb) if xv is not None]
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
            on_b = (wa < mx) if wb is None else (wa < mx < wb)
            segs.append((p, q, 'B.Cu' if on_b else 'F.Cu'))
        if nm in birth_b:
            # born on B: the copper from the tooth is B until the
            # surface via; the dive "via" at wa is not a via
            vias = [v for v in vias if abs(v[0] - wa) > 1e-9]
            segs = [(p, q, 'B.Cu' if (wb is None or (p[0] + q[0]) / 2 < wb)
                     else 'F.Cu') for (p, q, _l) in segs]
        exit_layer[nm] = segs[-1][2] if segs else 'F.Cu'
        out_segs[nm] = segs
        out_vias[nm] = vias

    # ---- the flank corridor: nested B.Cu runs under everything
    # (deepest = west-most descent, so descents never cross a run),
    # each net's run from its descent x to under its stub. Both ends
    # are connections. Run pitch 0.35 on the routing grid, the base
    # too, so every run end is a representable via site.
    run_of = {}
    if river:
        base = max(ends[nm][0][1] for nm in river) + 0.80
        base = math.ceil(base / 0.05) * 0.05
        rpitch = 0.35
        desc_x = {}
        for nm in sorted(river, key=lambda n: (tooth_layer[n] != 'B.Cu',
                                               ends[n][0][0])):
            dx0 = ends[nm][0][0]
            while any(abs(dx0 - v) < 0.28 for v in desc_x.values()):
                dx0 += 0.30
            desc_x[nm] = dx0
        r2 = sorted(river, key=lambda nm: desc_x[nm])
        for i, nm in enumerate(r2):
            run_y = base + (len(r2) - 1 - i) * rpitch
            bx = ends[nm][1][0]
            run_of[nm] = ((desc_x[nm], run_y), (bx, run_y))
            out_segs[nm] = [((desc_x[nm], run_y), (bx, run_y), 'B.Cu')]
            out_vias[nm] = []

    # ---- octilinearize the trunk with verifier-driven local refinement
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

    # build the static maps now, before our own copper enters pcb
    for nm in names:
        for layer in ('F.Cu', 'B.Cu'):
            obs_for(byname[nm][0], layer)

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

    print('\nverification (trunk):')
    bad, _off = run_verify()
    print(f'  {"CLEAN" if not bad else str(bad) + " violations"}')

    # ---- CONNECTIONS: trunk copper into the board, then every join
    # routed by the real router in an order where each result is an
    # obstacle for the next: west lanes and ports in target order, then
    # the flank corridor's descents (tooth -> run start) and climbs
    # (run end -> stub end).
    from kicad_parser import Segment, Via
    for nm in names:
        nid, _ = byname[nm]
        for (p, q, layer) in out_segs[nm]:
            pcb.segments.append(Segment(p[0], p[1], q[0], q[1],
                                        TRACK, layer, nid))
        for (vx, vy) in out_vias[nm]:
            pcb.vias.append(Via(vx, vy, VIA_SIZE, VIA_DRILL,
                                ['F.Cu', 'B.Cu'], nid))
    cfg_c = cn.make_config(pcb, TRACK, CLEAR, VIA_SIZE, VIA_DRILL)
    jobs = []
    for nm in target:
        jobs.append((nm, exits[nm], exit_layer[nm], ends[nm][1],
                     dest_layer[nm], port_band.get(nm)))
    for nm in sorted(river, key=lambda n: ends[n][0][0]):
        jobs.append((nm, ends[nm][0], tooth_layer[nm], run_of[nm][0],
                     'B.Cu', None))
    for nm in sorted(river, key=lambda n: ends[n][1][0]):
        jobs.append((nm, run_of[nm][1], 'B.Cu', ends[nm][1],
                     dest_layer[nm], None))
    n_ok = 0
    refused = []
    c_len = 0.0
    c_vias = 0
    for (nm, ex, exl, tgt, tl, band) in jobs:
        nid, _ = byname[nm]
        res = cn.connect(pcb, nid, ex, exl, tgt, tl, cfg_c, band=band,
                         margin=1.0)
        if res is None:
            refused.append(nm)
            print(f'REFUSED: {nm}: no route ({ex[0]:.2f},{ex[1]:.2f}) '
                  f'{exl} -> ({tgt[0]:.2f},{tgt[1]:.2f}) {tl}')
            continue
        n_ok += 1
        segs_o, vias_o = res
        segs_t = [((s.start_x, s.start_y), (s.end_x, s.end_y), s.layer)
                  for s in segs_o]
        vias_t = [(v.x, v.y) for v in vias_o]
        out_segs[nm].extend(segs_t)
        out_vias[nm].extend(vias_t)
        pcb.segments.extend(segs_o)
        pcb.vias.extend(vias_o)
        c_len += sum(math.hypot(q[0] - p[0], q[1] - p[1])
                     for (p, q, _l) in segs_t)
        c_vias += len(vias_t)
    print(f'\nconnections: {n_ok}/{len(jobs)} routed by the router, '
          f'{len(refused)} refused; {c_len:.2f} mm, {c_vias} via(s)')
    if refused:
        print(f'  REFUSED nets (left open): {sorted(set(refused))}')
    # The braid's own verifier models a rect pad as its circumscribed
    # DISC; a router tail that legally clips a pad's corner region reads
    # as a hit here. For connection copper the router's model is the
    # authority; this pass is informational.
    print('verification (with connections; disc model -- check_drc is '
          'the authority for router copper):')
    bad, _off = run_verify()
    print(f'  {"CLEAN" if not bad else str(bad) + " violations"}')

    # ---- repo octolinear smoothing (#536): collapse the distributed 45
    # nudges into single elbows, clearance-validated against ALL copper
    smoothed = False
    final_segs = {}
    if not a.no_smooth:
        from pcb_modification import smooth_octolinear_chains
        pre_len = {nm: sum(math.hypot(q[0] - p[0], q[1] - p[1])
                           for (p, q, _l) in out_segs[nm]) for nm in names}
        _n, _nets, _rm, _addl, st = smooth_octolinear_chains(
            [], pcb, kids, clearance=0.1)
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

    # ---- write board
    txt = open(a.board, encoding='utf-8').read()
    add = []
    if smoothed:
        kid_names = {pcb.nets[i].name for i in kids if i in pcb.nets}
        txt = strip_net_segments(txt, kids, kid_names)
        # the passes leave a few DEGENERATE (< 1 um) and DUPLICATE
        # segments; drop them (the neighbours already overlap)
        n_deg = n_dup = 0
        for nm in names:
            keep_ = []
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
                keep_.append(s)
            final_segs[nm] = keep_
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
    print(f'\nwrote {out_board}: {nseg} segments, {nv} vias'
          + (f' -- {len(refused)} net(s) REFUSED' if refused else ''))
    return 1 if refused else 0


if __name__ == '__main__':
    sys.exit(main())

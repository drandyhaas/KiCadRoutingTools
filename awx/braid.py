#!/usr/bin/env python3
"""braid.py -- the general braid emitter for a fanned-out bus corridor.

Inputs: a board whose SOURCE and DESTINATION arrays are both fanned out,
the nets to route, and the destination's reference. Everything else is
read off the board's geometry in the FLOW FRAME (source due west).

The braid decides ORDER and LAYERS: the launch order is the source's
tooth order, the target order is the stub-end order at the destination,
the divers are the complement of the longest increasing subsequence of
that permutation, and the wave schedule turns the inversions into
adjacent swaps -- at each crossing the mover is on the back layer and
the passed net on the front. That part is pure combinatorics, and it
implies the lane geometry: a corridor per net, morphing from the launch
order to the target order.

The COPPER is all the real router's (connect.py). Every lane is routed
from its tooth to its stub end inside its corridor, with the other
layer closed wherever the schedule requires one and the lanes not yet
routed stamped as virtual copper -- so the router places the dive and
surface vias where they actually fit and never where a later lane
must pass. A refused lane feeds back to the schedule: that diver waits
one more spacer column after being passed, the launch pitch widens a
step, and the schedule reruns. The flank corridor (the "river") is
nested back-layer runs with both ends routed the same way. There are
no hand tails, no via formulas and no fallbacks: a refused route is
reported as such and the net left open.

Two declared capability limits, not board facts: the trunk delivers to
the destination face that faces the source, and the flank corridor is
the SOUTH one in the flow frame (a plan that sends a net out the north
or east face has no corridor here). The corridor-per-face-pair
refactor lifts both.
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
    """A static-copper model for one net on one layer: every foreign
    pad as a disc, every foreign segment as a capsule, every foreign
    via as a disc, all inflated by clearance + half a track. The PLAN
    prices its candidate moves against it (fanout_from_plan, the
    probes); the braid's copper is routed against the router's own
    model and never consults this one."""
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
    # a diver whose tooth already sits on B.Cu is BORN diving: it needs
    # no dive via if it reaches its first own crossing on B, and the
    # router surfaces it (and dives it again) wherever the schedule
    # requires F. It takes no special place in the priority: moving
    # B-born divers to the front made the serial pass place them
    # relative to divers not yet moved (K32: the pass did not reach
    # the target), and the old rule that two B-born divers may not be
    # mutually inverted is gone with it.
    birth_b = {d for d in divers if tooth_layer[d] == 'B.Cu'}
    if birth_b:
        print(f'B-birth divers: {sorted(birth_b)}')
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
    placed = set(nm for nm in launch if nm not in divers)
    for d in priority:
        i = sseq.index(d)
        rest = sseq[:i] + sseq[i + 1:]
        # the slot: after the last PLACED element (a keeper, or a diver
        # already moved) of smaller rank. A diver not yet moved is no
        # anchor -- K32: SCKE1 was placed after SDQ7, an unmoved diver
        # then sitting to the right of SA0, and so never passed SA0.
        want = 0
        for j, e in enumerate(rest):
            if e in placed and trank[e] < trank[d]:
                want = j + 1
        passed_ser = rest[want:i] if want <= i else rest[i:want]
        mover_set[d] = list(passed_ser)
        if want != i:
            dirs[d] = -1 if want < i else 1
        sseq = rest[:want] + [d] + rest[want:]
        placed.add(d)
    assert sseq == target, (sseq, target)

    def schedule(gaps, lead):
        """The gated wave schedule. `gaps[d]` = spacer columns diver d
        waits after being passed before its own first swap; `lead[d]`
        = columns it waits from the launch before its first swap (room
        for its dive via when nobody passes it first). The router's
        refusals raise them."""
        done_moves = {d: set() for d in priority}
        passed = set()
        last_passed_round = {}
        last_move_round = {}
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
                # a layer change needs a column of its own: after being
                # passed (F) a diver waits gaps[d] columns before it
                # moves (B) -- whenever, not only before its first move
                if d in last_passed_round and \
                        len(cols) <= last_passed_round[d] + gaps[d]:
                    delay_stall = True
                    continue
                if not done_moves[d] and len(cols) < lead[d]:
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
                # ...and the other way: a diver that just moved (B)
                # must surface before it can be passed (F)
                if p in dirs and p in last_move_round and \
                        len(cols) <= last_move_round[p] + gaps[p]:
                    delay_stall = True
                    continue
                seq[i], seq[j] = seq[j], seq[i]
                col.append((d, p))
                used.add(d)
                used.add(p)
                done_moves[d].add(p)
                last_move_round[d] = len(cols)
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

    # ---- ROUTE THE TRUNK. The schedule says, per crossing, which net
    # is on B and which on F; the lane geometry says where each net's
    # corridor runs. Every lane is then ROUTED by the real router, from
    # its tooth to its stub end, inside its corridor: the other layer is
    # closed wherever the schedule requires one, and the lanes not yet
    # routed are stamped as virtual copper -- so the router places the
    # dive and surface vias where they actually fit, and never where a
    # later lane must pass. A refused lane feeds back: that diver waits
    # one more spacer column after being passed, the launch pitch widens
    # a step, and the schedule reruns.
    reserve = 0.3
    MINP = 0.38                    # lane pitch floor at the splice
    HALF_SEP = (TRACK + 0.1) / 2   # two lanes at their band edges clear
    gaps = {d: 1 for d in divers}
    lead = {d: 0 for d in divers}
    # the launch pitch floor: a lane must be able to carry a via next
    # to its neighbours ANYWHERE, so the floor is the via's need
    # (0.2885 to a neighbour's centreline) plus a routing-grid step
    ly_floor = 0.35
    base_segments = list(pcb.segments)
    base_vias = list(pcb.vias)
    # 0.025 grid: the fanout packs stub ends at 0.25, which is the legal
    # minimum (track + clearance = 0.227) plus 23 um. On a 0.05 grid the
    # cell nearest a lane's centreline can sit 25 um off it -- outside
    # the 23 um that clear the neighbours -- and the approach to a stub
    # end has no cell at all (rot180 K19: SDQ14 refused, backward
    # search boxed at its stub). At 0.025 the nearest cell is within
    # 12.5 um, always inside.
    cfg_c = cn.make_config(pcb, TRACK, CLEAR, VIA_SIZE, VIA_DRILL,
                           grid_step=0.025)
    out_segs, out_vias = {}, {}
    refused = []
    for attempt in range(6):
        cols = schedule(gaps, lead)
        swaps = [sw for col in cols for sw in col]
        seen_pairs = set()
        for (d, e) in swaps:
            pair = frozenset((d, e))
            assert pair not in seen_pairs, f'pair {d}/{e} swapped twice'
            seen_pairs.add(pair)
            assert inverted(d, e), f'phantom swap {d}/{e}'
        n = len(cols)
        W = (x1 - x0 - reserve) / (n + 1)
        print(f'attempt {attempt}: {len(swaps)} swaps in {n} columns, '
              f'W={W:.3f}, launch pitch >= {ly_floor:.2f}'
              + (f', gaps {dict((d, g) for d, g in gaps.items() if g > 1)}'
                 if any(g > 1 for g in gaps.values()) else '')
              + (f', lead {dict((d, g) for d, g in lead.items() if g)}'
                 if any(lead.values()) else ''))
        if attempt == 0:
            print(f'  {cols}')
        invol, first, last = {}, {}, {}
        for s, col in enumerate(cols):
            for (d, p) in col:
                invol.setdefault(d, []).append(s)
                invol.setdefault(p, []).append(s)
                first.setdefault(d, s)
                last[d] = s

        # REQUIRED layers: at the crossing of column s (between the
        # midpoints of s and s+1) the mover is on B and the passed net
        # on F, over the part of the column where the two lanes are too
        # close for one layer -- and NOT the whole column: a net passed
        # at s and moving at s+2 must change layer between the two, and
        # a requirement that spans each whole column leaves it no cell
        # to put the via in (K19: SDQ9 refused at every pitch).
        req = {nm: [] for nm in west}
        hw = 0.4 * W + 0.05
        for s, col in enumerate(cols):
            xc = x0 + (s + 1) * W
            for (d, p) in col:
                req[d].append((xc - hw, xc + hw, 'B.Cu'))
                req[p].append((xc - hw, xc + hw, 'F.Cu'))

        # POSSIBLE back-layer intervals: a diver may be on B only from
        # after the last foreign pass before its first own swap until
        # before the first foreign crossing after its last -- it must
        # be on F while passed -- and the router picks the via spots
        # inside that. A non-diver is on F in the trunk (B only in the
        # tail, for a stub the fanout left on B); one born on B may
        # stay there until it is first passed. Everything else on B is
        # closed, so the virtual copper of a lane that will never be
        # on B there does not box in a neighbour's via.
        # ...derived from the SAME required intervals, so the two can
        # never disagree (the old window formula ended a diver's B 0.17
        # mm before its own last crossing's requirement ended: both
        # layers closed, rot90 K19's SDQ15 refused at every pitch)
        bwin = {}
        for nm in west:
            wins = [(x1 - 0.1, 1e9)]
            b_iv = sorted((xa, xb) for (xa, xb, L) in req[nm] if L == 'B.Cu')
            f_iv = sorted((xa, xb) for (xa, xb, L) in req[nm] if L == 'F.Cu')
            if b_iv:
                b0, b1 = b_iv[0][0], b_iv[-1][1]
                lo = max((xb for (xa, xb) in f_iv if xb <= b0),
                         default=-1e9)
                hi = min((xa for (xa, xb) in f_iv if xa >= b1),
                         default=1e9)
                wins.append((lo, hi))
            elif tooth_layer[nm] == 'B.Cu':
                hi = min((xa for (xa, xb) in f_iv), default=1e9)
                wins.append((-1e9, hi))
            bwin[nm] = wins

        def allowed(nm, x, L):
            if any(xa <= x <= xb and RL != L
                   for (xa, xb, RL) in req.get(nm, ())):
                return False
            if L == 'B.Cu':
                return any(lo <= x <= hi for (lo, hi) in bwin[nm])
            return True

        # lane centrelines: launch slots (tooth ys at a pitch floor),
        # splice lanes (stub ys at the pitch floor, ports pinned), the
        # morph between, in the order the schedule gives per column
        py = [lane_y[nm] for nm in target]
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
        Ly = sorted(ends[nm][0][1] for nm in west)
        _ly2 = []
        for _v in Ly:
            _ly2.append(_v if not _ly2 else max(_v, _ly2[-1] + ly_floor))
        Ly = _ly2

        def slot(t, i):
            return (1 - t) * Ly[i] + t * py[i]

        def morph_t(xm):
            xms = x0 + W
            return max(0.0, (xm - xms) / (x1 - xms))

        orders = [list(launch)]
        for col in cols:
            o = list(orders[-1])
            for (d, p) in col:
                i, j = o.index(d), o.index(p)
                o[i], o[j] = o[j], o[i]
            orders.append(o)

        leave = {}
        for nm in west:
            if nm in is_port:
                smax = max(invol.get(nm, [-1]))
                leave[nm] = x0 + W if smax < 0 else x0 + (smax + 1.5) * W

        _line_cache = {}

        def line_pts(nm):
            """The net's centreline: tooth, column midpoints, splice,
            and the straight tail to the stub end -- or, for a port net,
            nothing past where it leaves the trunk."""
            if nm not in _line_cache:
                pts = [ends[nm][0]]
                for s in range(n + 1):
                    xm = x0 + (s + 0.5) * W
                    pts.append((xm, slot(morph_t(xm), orders[s].index(nm))))
                if nm in is_port:
                    xs_ = leave[nm]
                    pts = [p_ for p_ in pts if p_[0] < xs_ - 1e-6]
                    # the slot y at the leave point
                    ys_ = None
                    full = [ends[nm][0]] + [
                        (x0 + (s + 0.5) * W,
                         slot(morph_t(x0 + (s + 0.5) * W),
                              orders[s].index(nm))) for s in range(n + 1)]
                    for a_, b_ in zip(full, full[1:]):
                        if a_[0] <= xs_ <= b_[0]:
                            tt = (xs_ - a_[0]) / max(b_[0] - a_[0], 1e-9)
                            ys_ = a_[1] + tt * (b_[1] - a_[1])
                    if ys_ is None:
                        ys_ = full[-1][1]
                    pts.append((xs_, ys_))
                else:
                    pts.append((x1, py[trank[nm]]))
                    pts.append(ends[nm][1])
                _line_cache[nm] = pts
            return _line_cache[nm]

        def line_y(nm, x):
            pts = line_pts(nm)
            if x < pts[0][0] - 1e-9 or x > pts[-1][0] + 1e-9:
                return None
            for a_, b_ in zip(pts, pts[1:]):
                if a_[0] - 1e-9 <= x <= b_[0] + 1e-9:
                    tt = (x - a_[0]) / max(b_[0] - a_[0], 1e-9)
                    return a_[1] + tt * (b_[1] - a_[1])
            return pts[-1][1]

        def band_of(nm):
            """Per-layer corridor: between the centrelines of the
            neighbouring lanes present on that layer at x (a lane is
            present wherever the schedule does not require it on the
            other layer), pulled in by half the track spacing so two
            lanes at their band edges still clear. Closed where the
            schedule requires the other layer. A port net past its
            leave point rides the port band: below every trunk lane
            that ends above it, above every one below."""
            def fn_for(L):
                def fn(x):
                    if not allowed(nm, x, L):
                        return (1e9, -1e9)
                    ym = line_y(nm, x)
                    if ym is None:
                        # port region
                        if nm in is_port and x >= leave[nm] - 1e-9:
                            above = [line_y(om, x) for om in west
                                     if trank[om] < trank[nm] and x <= x1]
                            below = [line_y(om, x) for om in west
                                     if trank[om] > trank[nm] and x <= x1]
                            above = [v for v in above if v is not None]
                            below = [v for v in below if v is not None]
                            lo = max(above) + 0.26 if above else -1e9
                            hi = min(below) - 0.26 if below else 1e9
                            return (lo, hi)
                        return (-1e9, 1e9)
                    ys = []
                    for om in west:
                        if om == nm or not allowed(om, x, L):
                            continue
                        v = line_y(om, x)
                        if v is not None:
                            ys.append(v)
                    prev = max((v for v in ys if v < ym), default=None)
                    nxt = min((v for v in ys if v > ym), default=None)
                    lo = (prev + ym) / 2 + HALF_SEP if prev is not None \
                        else -1e9
                    hi = (nxt + ym) / 2 - HALF_SEP if nxt is not None \
                        else 1e9
                    # never narrower than a grid cell: at the stub ends
                    # the lanes are 0.25 apart and the corridor formula
                    # gives 0.0115 -- a band that, on an unlucky grid
                    # alignment, holds no cell at all (rot180 K19:
                    # SDQ14's stub end unreachable). Clearance to the
                    # neighbours' copper is the obstacle map's job.
                    lo = min(lo, ym - 0.03)
                    hi = max(hi, ym + 0.03)
                    return (lo, hi)
                return fn
            return {L: fn_for(L) for L in ('F.Cu', 'B.Cu')}

        def virtual_of(unrouted):
            """Centrelines of lanes not routed yet, on every layer the
            schedule lets them occupy, as copper the router must clear."""
            segs = []
            for om in unrouted:
                pts = line_pts(om)
                for a_, b_ in zip(pts, pts[1:]):
                    if a_ == b_:
                        continue
                    xm = (a_[0] + b_[0]) / 2
                    for L in ('F.Cu', 'B.Cu'):
                        if allowed(om, xm, L):
                            segs.append((a_, b_, L))
            return segs

        # route: divers first (they carry the vias), then the rest in
        # target order; each result is copper for the next
        pcb.segments = list(base_segments)
        pcb.vias = list(base_vias)
        out_segs, out_vias = {}, {}
        order = list(priority) + [nm for nm in target if nm not in divers]
        routed = set()
        refused = []
        for nm in order:
            nid, _ = byname[nm]
            virt = virtual_of([om for om in west
                               if om != nm and om not in routed])
            res = cn.connect(pcb, nid, ends[nm][0], tooth_layer[nm],
                             ends[nm][1], dest_layer[nm], cfg_c,
                             band=band_of(nm), virtual=virt, margin=0.6)
            if res is None:
                refused.append(nm)
                print(f'  refused: {nm}')
                if os.environ.get('LANE_DEBUG') == nm:
                    bnd = band_of(nm)
                    tx0 = ends[nm][0][0]
                    print(f'    tooth {ends[nm][0]} {tooth_layer[nm]}  '
                          f'stub {ends[nm][1]} {dest_layer[nm]}  x0={x0:.2f}')
                    print(f'    req: {req[nm]}')
                    print(f'    bwin: {bwin[nm]}')
                    for k in range(0, 16):
                        x = tx0 + 0.1 * k
                        ym = line_y(nm, x)
                        fb = bnd['F.Cu'](x)
                        bb = bnd['B.Cu'](x)
                        near = [(om, round(line_y(om, x), 2)) for om in west
                                if om != nm and line_y(om, x) is not None
                                and ym is not None
                                and abs(line_y(om, x) - ym) < 0.6]
                        print(f'    x={x:.2f} y={ym if ym is None else round(ym, 3)} '
                              f'F=({fb[0]:.2f},{fb[1]:.2f}) '
                              f'B=({bb[0]:.2f},{bb[1]:.2f}) near={near}')
                    vs = [(p_, q_, L) for (p_, q_, L) in virt
                          if min(p_[0], q_[0]) < tx0 + 1.6]
                    print(f'    virtual near start: {len(vs)}')
                    for (p_, q_, L) in vs[:12]:
                        print(f'      {L} ({p_[0]:.2f},{p_[1]:.2f})->'
                              f'({q_[0]:.2f},{q_[1]:.2f})')
                continue
            routed.add(nm)
            segs_o, vias_o = res
            out_segs[nm] = [((s.start_x, s.start_y), (s.end_x, s.end_y),
                             s.layer) for s in segs_o]
            out_vias[nm] = [(v.x, v.y) for v in vias_o]
            pcb.segments.extend(segs_o)
            pcb.vias.extend(vias_o)
        nv = sum(len(v) for v in out_vias.values())
        print(f'  lanes: {len(routed)}/{len(west)} routed, {nv} via(s)')
        if not refused:
            break
        # FEEDBACK. Room in y first -- the launch pitch is the cheap
        # dimension, every lane gets it, and a via beside a neighbour
        # is what usually fails. Then room in x for a refused diver's
        # dive via: before its first swap if nobody passes it first (a
        # lead column -- rot90 K19's SDQ15 swaps in column 0, so its
        # via had to sit on the launch leg), else after the pass (a
        # spacer column). Spacer columns shrink W for every lane, so
        # they come last.
        if ly_floor < 0.40 - 1e-9:
            ly_floor = min(0.40, ly_floor + 0.03)
            continue
        for nm in refused:
            if nm in divers:
                s1 = first.get(nm)
                fw = max((s for s in invol.get(nm, []) if s1 is None
                          or s < s1), default=None)
                if fw is None:
                    lead[nm] += 1
                else:
                    gaps[nm] += 1
    for nm in refused:
        out_segs[nm] = []
        out_vias[nm] = []
    if refused:
        print(f'REFUSED lanes (left open): {sorted(refused)}')

    # ---- the flank corridor: nested B.Cu runs under everything
    # (deepest = west-most descent, so descents never cross a run),
    # each net's run from its descent x to under its stub. Both ends
    # are connections. Run pitch 0.35 on the routing grid, the base
    # too, so every run end is a representable via site.
    from kicad_parser import Segment
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
            nid, _ = byname[nm]
            pcb.segments.append(Segment(desc_x[nm], run_y, bx, run_y,
                                        TRACK, 'B.Cu', nid))
        jobs = []
        for nm in sorted(river, key=lambda n: ends[n][0][0]):
            jobs.append((nm, ends[nm][0], tooth_layer[nm], run_of[nm][0],
                         'B.Cu'))
        for nm in sorted(river, key=lambda n: ends[n][1][0]):
            jobs.append((nm, run_of[nm][1], 'B.Cu', ends[nm][1],
                         dest_layer[nm]))
        n_ok = 0
        c_vias = 0
        for (nm, ex, exl, tgt, tl) in jobs:
            nid, _ = byname[nm]
            res = cn.connect(pcb, nid, ex, exl, tgt, tl, cfg_c, margin=1.0)
            if res is None:
                refused.append(nm)
                print(f'REFUSED: {nm}: no route ({ex[0]:.2f},{ex[1]:.2f}) '
                      f'{exl} -> ({tgt[0]:.2f},{tgt[1]:.2f}) {tl}')
                continue
            n_ok += 1
            segs_o, vias_o = res
            out_segs[nm].extend(((s.start_x, s.start_y),
                                 (s.end_x, s.end_y), s.layer)
                                for s in segs_o)
            out_vias[nm].extend((v.x, v.y) for v in vias_o)
            pcb.segments.extend(segs_o)
            pcb.vias.extend(vias_o)
            c_vias += len(vias_o)
        print(f'flank corridor: {n_ok}/{len(jobs)} connections routed, '
              f'{c_vias} via(s)')
    if refused:
        print(f'  REFUSED nets (left open): {sorted(set(refused))}')

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

    # ---- Eco overlay: the PLAN, drawn where the copper is, so a render
    # (render_eco.py) shows plan against copper.
    #   Eco1.User (white)   every lane's planned centreline -- tooth,
    #                       column midpoints, splice, stub -- and the
    #                       flank corridor's runs;
    #   Cmts.User (orange)  where the schedule REQUIRES the back layer:
    #                       the planned under-passes, on the centreline;
    #   Eco2.User (yellow)  the connection ends as crosses -- source
    #                       teeth, stub ends, port leave points, run ends.
    def gl(p, q, layer, w=0.05):
        (ax, ay), (bx_, by_) = back_xy(*p), back_xy(*q)
        return (f'  (gr_line (start {ax:.4f} {ay:.4f}) '
                f'(end {bx_:.4f} {by_:.4f}) '
                f'(stroke (width {w}) (type solid)) (layer "{layer}"))\n')

    def cross(p, layer='Eco2.User', r=0.12):
        return gl((p[0] - r, p[1] - r), (p[0] + r, p[1] + r), layer) + \
            gl((p[0] - r, p[1] + r), (p[0] + r, p[1] - r), layer)

    def sub_line(nm, xa, xb):
        """The centreline's polyline between x = xa and xb."""
        pts = line_pts(nm)
        ya, yb = line_y(nm, xa), line_y(nm, xb)
        out = []
        if ya is not None:
            out.append((xa, ya))
        out += [p_ for p_ in pts if xa < p_[0] < xb]
        if yb is not None:
            out.append((xb, yb))
        return out

    n_eco = 0
    for nm in west:
        pts = line_pts(nm)
        for p_, q_ in zip(pts, pts[1:]):
            add.append(gl(p_, q_, 'Eco1.User'))
            n_eco += 1
        for (xa, xb, L) in req[nm]:
            if L != 'B.Cu':
                continue
            sl = sub_line(nm, xa, xb)
            for p_, q_ in zip(sl, sl[1:]):
                add.append(gl(p_, q_, 'Cmts.User', 0.08))
        add.append(cross(ends[nm][0]))
        add.append(cross(ends[nm][1]))
        if nm in leave:
            yl = line_y(nm, leave[nm])
            if yl is not None:
                add.append(cross((leave[nm], yl)))
    for nm in river:
        a_, b_ = run_of[nm]
        add.append(gl(a_, b_, 'Eco1.User'))
        n_eco += 1
        for p_ in (ends[nm][0], ends[nm][1], a_, b_):
            add.append(cross(p_))
    print(f'eco overlay: {n_eco} planned centreline segments, '
          f'{sum(len([r for r in req[nm] if r[2] == "B.Cu"]) for nm in west)} '
          f'planned under-passes, {2 * len(west) + 4 * len(river)} ends')
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

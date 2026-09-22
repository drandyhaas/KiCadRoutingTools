"""The bus's DIFFERENTIAL PAIRS as members of the topo routing (#622, 2026-09-20).

A pair is two nets the board names as a pair (a trailing P/N, _P/_N, +/-)
whose two balls sit side by side at both arrays. Routed as two singles
the braid treats them as unrelated lanes: measured on the H3 bench at
K34, SDQS0's P leg dived twice and its N leg never, 7 percent of their
copper coupled; SDQS1 ran a millimetre from its partner. The human routes
each pair as ONE lane at one pitch, diving with two barrels side by side.

So the braid gets a PAIR MEMBER: one corridor lane whose ends are the
midpoints of its two teeth and its two berths, whose copper is laid by the
production pair router (connect.connect_pair) between the real ends, and
whose reservation as an unrouted lane is two lines a pitch apart. Nothing
here knows a board or a net name: the pairing is the suffix rule, the
pitch is the rules' track plus the pair gap.

    BRAID_PAIRS=1     the pair member (default off: the braid byte-identical)
"""
import math
import os
import re
from typing import Dict, List, Sequence, Tuple

import rules as _rules

Pt = Tuple[float, float]

# the pair gap: P-to-N edge gap. It must be at least the clearance the
# lanes are ROUTED at -- the braid's hug (clearance + 5 um), not the spec:
# the pair router rejects a pair whose legs sit closer than the config's
# clearance as a short (measured: gap 0.100 against clearance 0.105,
# SDQS0 refused after a found route). Overridable per invocation.
GAP = float(os.environ.get('BRAID_PAIR_GAP', '0') or 0) or round(_rules.DEFAULT.hug + 0.04, 6)
# (the hug plus 40 um: the pose router's short test is `gap < clearance`
# on the legs it GENERATES, and at a corner the inner leg's gap shrinks --
# measured 0.102, 0.1035 and 0.092 against 0.105 at turns on three pairs, and
# 0.1050 against 0.1050 was rejected on float rounding alone; the human's
# pairs run at 0.32 mm pitch, this is 0.272)
# the farthest apart a pair's two teeth (or two berths) may stand for the
# pair to be routed as ONE lane: beyond it the legs are singles, and the
# log says so (the plan chose the ends apart -- the joint menu's job)
MAX_SEP = float(os.environ.get('BRAID_PAIR_SEP', '0') or 0) or 2.0 * _rules.DEFAULT.lane_pitch

_SUFFIX = re.compile(r'^(.*?)(_P|_N|P|N|\+|-|_p|_n)$')

# BRAID_PAIR_ONLY (2026-09-21): a comma list of pair BASE names (SCK,SDQS0).
# A pair not named has its legs planned and routed as SINGLES everywhere --
# the plan's pair clauses, harmonise, the judge's penalty, the braid's
# member -- which is the instrument for "one pair at a time": the same 51
# nets, one pair coupled, the rest as they were before pairs existed. Unset
# = every pair the suffix rule finds. The ladder's ADMISSION (coherent_nets)
# ignores it (admit_all), so every arm routes the same net list.
ONLY = {s.strip() for s in os.environ.get('BRAID_PAIR_ONLY', '').split(',') if s.strip()}


def pitch(track: float = None) -> float:
    """Centre-to-centre pitch of the two legs."""
    return (_rules.TRACK if track is None else track) + GAP


def pair_names(names: Sequence[str], admit_all: bool = False) -> Dict[str, Tuple[str, str]]:
    """{base: (P name, N name)} over the given net names, by suffix.
    BRAID_PAIR_ONLY narrows it to the named pairs unless admit_all."""
    by: Dict[str, Dict[str, str]] = {}
    for nm in names:
        m = _SUFFIX.match(nm)
        if not m or not m.group(1):
            continue
        base, suf = m.group(1), m.group(2)
        pol = 'P' if suf in ('_P', 'P', '+', '_p') else 'N'
        by.setdefault(base, {})[pol] = nm
    out = {b: (d['P'], d['N']) for b, d in by.items() if 'P' in d and 'N' in d}
    if ONLY and not admit_all:
        out = {b: v for b, v in out.items() if b in ONLY}
    return out


def members(names: Sequence[str]) -> Tuple[List[str], Dict[str, Tuple[str, str]]]:
    """The run's member list with each pair's two legs replaced by ONE
    member named after the pair (at the P leg's position), and the pairs
    found. A base name that is also a net of the run is not a pair (it
    would shadow a real net)."""
    prs = {b: pn for b, pn in pair_names(names).items() if b not in names}
    leg_of = {leg: b for b, (p, n) in prs.items() for leg in (p, n)}
    out, seen = [], set()
    for nm in names:
        b = leg_of.get(nm)
        if b is None:
            out.append(nm)
        elif b not in seen:
            out.append(b)
            seen.add(b)
    return out, prs


def mid(a: Pt, b: Pt) -> Pt:
    return ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)


def _unit(a: Pt, b: Pt) -> Pt:
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = math.hypot(dx, dy) or 1.0
    return (dx / L, dy / L)


def _left(d: Pt) -> Pt:
    return (-d[1], d[0])


def _cross(a: Pt, b: Pt) -> float:
    return a[0] * b[1] - a[1] * b[0]


def offset_polyline(pts: Sequence[Pt], h: float) -> List[Pt]:
    """The polyline offset by `h` to its LEFT (mitred at the vertices; a
    near-reversal falls back to the plain offset of the incoming edge)."""
    if len(pts) < 2:
        return list(pts)
    out = []
    for i, p in enumerate(pts):
        if i == 0:
            d = _unit(pts[0], pts[1])
            n = _left(d)
            out.append((p[0] + n[0] * h, p[1] + n[1] * h))
        elif i == len(pts) - 1:
            d = _unit(pts[-2], pts[-1])
            n = _left(d)
            out.append((p[0] + n[0] * h, p[1] + n[1] * h))
        else:
            d1, d2 = _unit(pts[i - 1], p), _unit(p, pts[i + 1])
            n1, n2 = _left(d1), _left(d2)
            bx, by = n1[0] + n2[0], n1[1] + n2[1]
            bl = math.hypot(bx, by)
            cos_half = bl / 2.0
            if bl < 1e-6 or cos_half < 0.3:
                out.append((p[0] + n1[0] * h, p[1] + n1[1] * h))
                continue
            m = h / cos_half
            out.append((p[0] + bx / bl * m, p[1] + by / bl * m))
    return out


def _chain(segs, start: Pt, tol: float = 0.01):
    """Order the envelope's segments from `start` into runs per layer:
    [(layer, [pts...]), ...] with a via between consecutive runs."""
    key = lambda x, y: (round(x, 3), round(y, 3))
    left = list(segs)
    runs: List[Tuple[str, List[Pt]]] = []
    cur = start
    while left:
        nxt = None
        for s in left:
            for a, b in (((s.start_x, s.start_y), (s.end_x, s.end_y)),
                         ((s.end_x, s.end_y), (s.start_x, s.start_y))):
                if math.hypot(a[0] - cur[0], a[1] - cur[1]) <= tol:
                    nxt = (s, a, b)
                    break
            if nxt:
                break
        if nxt is None:
            break
        s, a, b = nxt
        left.remove(s)
        if not runs or runs[-1][0] != s.layer:
            runs.append((s.layer, [a]))
        runs[-1][1].append(b)
        cur = b
    return runs, left


def split_envelope(segs, vias, p_tip: Pt, n_tip: Pt, p_end: Pt, n_end: Pt,
                   a_pt: Pt, b_pt: Pt, half: float, via_half: float,
                   track: float, via_size: float, via_drill: float,
                   p_id: int, n_id: int, layers, jog: float = 0.15,
                   cross: bool = False, tip_layers=None):
    """The ENVELOPE lane (one wide track from `a_pt` to `b_pt`, vias as
    wide as two barrels) split into the pair's two legs: P and N offset
    `half` either side of the centreline, mitred at the corners; at each
    dive the two barrels stand `via_half` either side of the envelope's
    via, and each leg jogs out to its barrel over `jog` mm and back; the
    legs converge onto the real tips (`p_tip`, `n_tip`) and ends. P takes
    the side its tip lies on at the start; when its berth lies on the
    other side the pair must CROSS once: refused (None) unless `cross`,
    in which case P takes its berth's side and its tooth lead is left
    to the caller (returned as `p_start`, the first point of P's offset
    line). Returns (segments, vias, p_start) or None."""
    from kicad_parser import Segment, Via
    runs, left = _chain(segs, a_pt)
    if left or not runs:
        return None
    runs = [(L, _simplify(pts)) for (L, pts) in runs]
    runs = [(L, pts) for (L, pts) in runs if len(pts) >= 2]
    if not runs:
        return None
    # P's side: left (+) or right (-) of the path at the start
    d0 = _unit(runs[0][1][0], runs[0][1][1])
    s_p = 1.0 if _cross(d0, (p_tip[0] - a_pt[0], p_tip[1] - a_pt[1])) >= 0 else -1.0
    dl = _unit(runs[-1][1][-2], runs[-1][1][-1])
    s_p_end = 1.0 if _cross(dl, (p_end[0] - b_pt[0], p_end[1] - b_pt[1])) >= 0 else -1.0
    crossing = s_p_end != s_p
    if crossing and not cross:
        return None
    cross_end = crossing and cross == 'end'
    if crossing and not cross_end:
        # P takes the side its BERTH is on; its tooth is joined by the
        # caller, who routes that lead with the real router (it dives
        # under N: the crossing costs P two vias)
        s_p = s_p_end
    # (cross == 'end': P keeps its tooth's side and its BERTH lead is the
    # one left to the caller -- returned as p_start too, the last point)
    p_start = None
    out_segs, out_vias = [], []
    # the normal at each dive, ONE per via, from the longer of the two
    # segments that meet there (a dive right at the lead's start leaves a
    # 30 um first run whose direction is grid noise -- measured: the two
    # barrels landed diagonal and the jogs ran backwards into each other)
    # ...and it is the INCOMING direction: the barrels stand across the
    # lane as it arrives, and the legs leave them straight on for a jog
    # before bending onto the next run. Across the outgoing direction a
    # 45-degree turn at the dive put a leaving leg 0.25 mm from the other
    # barrel (needs via radius + clearance + half a track, 0.29). The
    # approach piece the caller prepends makes the first run long enough
    # for its direction to be the escape's, never grid noise.
    n_via, d_via = [], []
    for k in range(len(runs) - 1):
        pa, pb = runs[k][1][-2], runs[k][1][-1]
        d_in = _unit(pa, pb)
        n_via.append(_left(d_in))
        d_via.append(d_in)

    def emit(pts, layer, nid):
        for a, b in zip(pts, pts[1:]):
            if math.hypot(b[0] - a[0], b[1] - a[1]) < 1e-6:
                continue
            out_segs.append(Segment(a[0], a[1], b[0], b[1], track, layer, nid))

    # a leg whose tip or end lies on the other layer from the envelope's:
    # the converge leg is laid on the envelope's layer and a barrel stands
    # AT the tip (a berth laid as a surface stub beside a partner's via)
    tl = tip_layers or (runs[0][0], runs[0][0], runs[-1][0], runs[-1][0])
    tip_layer = {p_id: (tl[0], tl[2]), n_id: (tl[1], tl[3])}
    for sign, nid, tip, end in ((s_p, p_id, p_tip, p_end), (-s_p, n_id, n_tip, n_end)):
        if tip_layer[nid][0] != runs[0][0] and not (crossing and nid == p_id):
            out_vias.append(Via(tip[0], tip[1], via_size, via_drill, list(layers), nid))
        if tip_layer[nid][1] != runs[-1][0]:
            out_vias.append(Via(end[0], end[1], via_size, via_drill, list(layers), nid))
        for k, (layer, pts) in enumerate(runs):
            run_len = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
            jog_k = min(jog, run_len / 4.0)
            core = offset_polyline(pts, sign * half)
            if run_len <= 3.0 * jog_k + 1e-9 or run_len < 0.06:
                core = []                      # too short for a run between its barrels
            else:
                if k > 0:
                    core = _cut_front(core, 2.0 * jog_k)
                if k < len(runs) - 1:
                    core = _cut_back(core, jog_k)
            poly = []
            if k > 0:
                V, n, d_in = pts[0], n_via[k - 1], d_via[k - 1]
                vp = (V[0] + sign * n[0] * via_half, V[1] + sign * n[1] * via_half)
                poly.append(vp)
                if core:
                    # the lane may TURN at the dive. The barrel on the inside
                    # of the turn goes straight to its offset line; the outer
                    # one carries on for a jog first and bends after -- an
                    # inner leg carried straight on ran into the outer leg's
                    # bend (measured 0.075 mm short at a 45-degree dive)
                    d_out = _unit(pts[0], pts[1])
                    turn = _cross(d_in, d_out)
                    inner = (sign * turn) > 1e-9
                    if not inner:
                        poly.append((vp[0] + d_in[0] * jog_k, vp[1] + d_in[1] * jog_k))
            poly += core
            if k < len(runs) - 1:
                V, n = pts[-1], n_via[k]
                vp = (V[0] + sign * n[0] * via_half, V[1] + sign * n[1] * via_half)
                poly.append(vp)
                out_vias.append(Via(vp[0], vp[1], via_size, via_drill, list(layers), nid))
            if k == 0:
                if crossing and not cross_end and nid == p_id:
                    p_start = poly[0]
                else:
                    poly = [tip] + poly
            if k == len(runs) - 1:
                if cross_end and nid == p_id:
                    p_start = poly[-1]
                else:
                    poly = poly + [end]
            emit(poly, layer, nid)
    # the envelope's vias are the pair's barrels only once (P's loop added
    # them; the N loop's barrels are its own) -- dedupe by position
    seen = set()
    vias_out = []
    for v in out_vias:
        kk = (round(v.x, 4), round(v.y, 4), v.net_id)
        if kk in seen:
            continue
        seen.add(kk)
        vias_out.append(v)
    return out_segs, vias_out, p_start


def _simplify(pts: Sequence[Pt], tiny: float = 0.02) -> List[Pt]:
    """The polyline without its sub-`tiny` segments (their vertex dropped,
    the ends kept) and without collinear interior vertices -- an offset
    of a path with a 13 um tail turned its end into a loop."""
    pts = list(pts)
    if len(pts) < 3:
        return pts
    out = [pts[0]]
    for i in range(1, len(pts)):
        p = pts[i]
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) < tiny:
            if i == len(pts) - 1:
                out[-1] = p if len(out) == 1 else out[-1]
            continue
        out.append(p)
    if math.hypot(pts[-1][0] - out[-1][0], pts[-1][1] - out[-1][1]) > 1e-9:
        out[-1] = pts[-1]
    # collinear vertices
    res = [out[0]]
    for i in range(1, len(out) - 1):
        d1, d2 = _unit(res[-1], out[i]), _unit(out[i], out[i + 1])
        if abs(_cross(d1, d2)) < 1e-6 and (d1[0] * d2[0] + d1[1] * d2[1]) > 0:
            continue
        res.append(out[i])
    res.append(out[-1])
    return res


def _cut_front(poly: List[Pt], L: float) -> List[Pt]:
    """The polyline with its first `L` mm removed."""
    pts = list(poly)
    while len(pts) >= 2 and L > 0:
        d = math.hypot(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1])
        if d <= L:
            L -= d
            pts.pop(0)
        else:
            u = _unit(pts[0], pts[1])
            pts[0] = (pts[0][0] + u[0] * L, pts[0][1] + u[1] * L)
            L = 0
    return pts


def _cut_back(poly: List[Pt], L: float) -> List[Pt]:
    return list(reversed(_cut_front(list(reversed(poly)), L)))


def offset_line(p: Pt, q: Pt, h: float) -> List[Tuple[Pt, Pt]]:
    """The two lines parallel to p->q at +h and -h across it (the pair's
    legs about a centreline), as (p', q') pairs."""
    dx, dy = q[0] - p[0], q[1] - p[1]
    L = math.hypot(dx, dy)
    if L < 1e-9:
        return [(p, q)]
    nx, ny = -dy / L * h, dx / L * h
    return [((p[0] + nx, p[1] + ny), (q[0] + nx, q[1] + ny)),
            ((p[0] - nx, p[1] - ny), (q[0] - nx, q[1] - ny))]


def describe(m) -> str:
    return (f'{m.direction}/{m.layer}/{m.kind} @({m.exit_pt[0]:.2f},{m.exit_pt[1]:.2f}) '
            f'{m.vias}v')


_DIRS = {'left': (-1, 0), 'right': (1, 0), 'up': (0, -1), 'down': (0, 1)}


def hand(direction, p_pt, n_pt, arriving: bool = False) -> int:
    """THE HANDEDNESS of a pair end: +1 / -1 for which side of the travel
    direction P lies on, 0 when undecided. `direction` is an escape name
    ('up'...) or a unit vector; at a berth the pair ARRIVES against the
    stub's escape direction (arriving=True). A planar pair route keeps P
    on one side of travel from end to end, so a pair whose two ends
    disagree cannot be routed coupled without a crossover (the production
    router refuses it: "polarity mismatch cannot be resolved", K36 SCK:
    teeth P west of N leaving south, berths P west of N entered from the
    south)."""
    d = _DIRS.get(direction, direction) if isinstance(direction, str) else direction
    if d is None:
        return 0
    dx, dy = (-d[0], -d[1]) if arriving else (d[0], d[1])
    vx, vy = p_pt[0] - n_pt[0], p_pt[1] - n_pt[1]
    c = dx * vy - dy * vx
    return 0 if abs(c) < 1e-6 else (1 if c > 0 else -1)


def pair_waypoints(pcb, p_id: int, n_id: int, src_ref: str, dst_ref: str):
    """The two-pad parts a pair PASSES THROUGH between its arrays: a part
    with one pad on P and the other on N (a differential termination
    resistor), neither array. Returns [(pad_p, pad_n)] ordered from the
    source array outward."""
    out = []
    arrays = [pcb.footprints.get(r) for r in (src_ref, dst_ref)]
    array_pads = [q for fp in arrays if fp is not None for q in fp.pads if q.net_id in (p_id, n_id)]
    for fp in pcb.footprints.values():
        if fp.reference in (src_ref, dst_ref) or len(fp.pads) != 2:
            continue
        nets = {q.net_id for q in fp.pads}
        if nets == {p_id, n_id}:
            pp = [q for q in fp.pads if q.net_id == p_id][0]
            pn = [q for q in fp.pads if q.net_id == n_id][0]
            # a part UNDER the array's balls is served by a tie via at the
            # ball (fanout_from_plan.tie_vias_under), not passed through
            if any(under_pad(b, q, 0.25) for q in (pp, pn) for b in array_pads):
                continue
            out.append((pp, pn))
    src = pcb.footprints.get(src_ref)
    if src is not None and len(out) > 1:
        cx = sum(q.global_x for q in src.pads) / len(src.pads)
        cy = sum(q.global_y for q in src.pads) / len(src.pads)
        out.sort(key=lambda w: math.hypot(w[0].global_x - cx, w[0].global_y - cy))
    return out


def harmonise(choice: dict, menus: dict, names: Sequence[str], pitch: float,
              conflict, log=print) -> List[str]:
    """THE PAIR'S TWO BERTHS AS ONE MOVE, after a plan chose them one by one
    (2026-09-20, Andy: "at the berth the two tracks of the pair should
    stay on the same layer as much as possible"). For each pair whose two
    chosen moves differ in face, layer or kind, or whose exits are not
    neighbours (within 1.3 array pitches), one leg is moved onto the
    other's class: the candidate from its menu with the partner's face,
    layer and kind, the nearest neighbouring exit, in conflict with
    neither the partner nor any other chosen move. Both directions are
    tried and the cheaper stands (fewer vias: a surface pair over a
    via-in-pad pair), so a pair leaves the array together and on one
    layer. `menus` are the destination menus (net -> [Move]); `conflict`
    is the selector's own pairwise test. Returns the log lines."""
    out = []
    reach = 1.3 * pitch

    def same_class(a, b):
        # the same FACE on the same LAYER is what makes two legs one lane;
        # the kind (a via-in-pad beside a dog-bone) may differ
        return (a.direction, a.layer) == (b.direction, b.layer)

    def adjacent(a, b, others=()):
        d = math.hypot(a.exit_pt[0] - b.exit_pt[0], a.exit_pt[1] - b.exit_pt[1])
        if not (0.05 < d <= reach):
            return False
        # ...and no other chosen exit of the same face and layer between them
        ax = 0 if a.direction in ('up', 'down') else 1
        lo_, hi_ = sorted((a.exit_pt[ax], b.exit_pt[ax]))
        return not any((o.direction, o.layer) == (a.direction, a.layer)
                       and lo_ + 0.02 < o.exit_pt[ax] < hi_ - 0.02 for o in others)

    for base, (pn, nn) in pair_names(names).items():
        if pn not in choice or nn not in choice:
            continue
        mp, mn = choice[pn], choice[nn]
        others = [m for k, m in choice.items() if k not in (pn, nn)]
        if same_class(mp, mn) and adjacent(mp, mn, others):
            continue

        def fit(leg, ref):
            cands = [m for m in menus.get(leg, ()) if same_class(m, ref) and adjacent(m, ref, others)
                     and not conflict(m, ref, strict=False)
                     and not any(conflict(m, o, strict=False) for o in others)]
            if not cands:
                return None
            return min(cands, key=lambda m: math.hypot(m.exit_pt[0] - ref.exit_pt[0],
                                                       m.exit_pt[1] - ref.exit_pt[1]))
        opts = []
        c = fit(nn, mp)
        if c is not None:
            opts.append((mp.vias + c.vias, nn, c, mp))
        c = fit(pn, mn)
        if c is not None:
            opts.append((mn.vias + c.vias, pn, c, mn))
        if not opts:
            # neither leg can join the other's class: the JOINT search --
            # every (P move, N move) of one face and layer, exits
            # neighbouring, in conflict with neither each other nor any
            # other chosen move; the fewest vias, then the fewest legs
            # moved, then the nearest exits
            best = None
            for a in menus.get(pn, ()):
                for b in menus.get(nn, ()):
                    if not (same_class(a, b) and adjacent(a, b, others)):
                        continue
                    if conflict(a, b, strict=False):
                        continue
                    if any(conflict(a, o, strict=False) or conflict(b, o, strict=False) for o in others):
                        continue
                    moved = (a is not mp) + (b is not mn)
                    d = math.hypot(a.exit_pt[0] - b.exit_pt[0], a.exit_pt[1] - b.exit_pt[1])
                    key = (a.vias + b.vias, moved, d)
                    if best is None or key < best[0]:
                        best = (key, a, b)
            if best is None:
                line = (f'  pair {base}: berths differ ({describe(mp)} / {describe(mn)}) '
                        f'and no joint move of one face and layer fits -- left as chosen')
                out.append(line)
                log(line)
                continue
            (v, moved, d), a, b = best
            choice[pn], choice[nn] = a, b
            line = (f'  pair {base}: berths harmonised JOINTLY -- {pn} {describe(mp)} -> {describe(a)}, '
                    f'{nn} {describe(mn)} -> {describe(b)}; {v} via(s) for the pair, exits {d:.2f} mm apart')
            out.append(line)
            log(line)
            continue
        opts.sort(key=lambda t: t[0])
        v, leg, m, ref = opts[0]
        was = choice[leg]
        choice[leg] = m
        line = (f'  pair {base}: berths harmonised -- {leg} {describe(was)} -> {describe(m)} '
                f'beside {describe(ref)}; {v} via(s) for the pair')
        out.append(line)
        log(line)
    return out


def under_pad(ball, pad, via_size: float) -> bool:
    """`pad` is SERVED BY A VIA-IN-PAD at `ball`: it lies on another copper
    layer and the barrel dropped at the ball's centre lands inside its
    copper (a termination resistor placed on the back side under a DDR
    clock ball). Such a net's escape from the ball must be a via-in-pad,
    and the pad counts as the ball for the ladder's two-pad rule."""
    if pad is ball:
        return False
    # the served pad is the LARGER copper (a passive's pad under a ball):
    # the test is one-way, or a ball and the pad under it would each
    # exclude the other
    if pad.size_x * pad.size_y <= ball.size_x * ball.size_y:
        return False
    la = {L for L in ball.layers if L.endswith('.Cu')}
    lb = {L for L in pad.layers if L.endswith('.Cu')}
    if not la or not lb or la & lb:
        return False
    # a barrel at the ball's centre OVERLAPS the pad's copper by at least
    # 50 um in each axis -- a partial overlap connects (the clock pair's
    # second resistor pad stands 0.28 mm off its ball and the human's
    # via-in-pad still reaches it); containment was too strict
    r = via_size / 2.0
    return (abs(pad.global_x - ball.global_x) <= pad.size_x / 2.0 + r - 0.05
            and abs(pad.global_y - ball.global_y) <= pad.size_y / 2.0 + r - 0.05)


def _seg_seg_dist(a, b) -> float:
    """Shortest distance between two segments (their centrelines)."""
    def d_ps(px, py, ax, ay, bx, by):
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        if L2 <= 1e-12:
            return math.hypot(px - ax, py - ay)
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
        return math.hypot(px - (ax + t * dx), py - (ay + t * dy))
    # crossing?
    def orient(px, py, qx, qy, rx, ry):
        return (qx - px) * (ry - py) - (qy - py) * (rx - px)
    o1 = orient(a.start_x, a.start_y, a.end_x, a.end_y, b.start_x, b.start_y)
    o2 = orient(a.start_x, a.start_y, a.end_x, a.end_y, b.end_x, b.end_y)
    o3 = orient(b.start_x, b.start_y, b.end_x, b.end_y, a.start_x, a.start_y)
    o4 = orient(b.start_x, b.start_y, b.end_x, b.end_y, a.end_x, a.end_y)
    if (o1 * o2 < 0) and (o3 * o4 < 0):
        return 0.0
    return min(d_ps(a.start_x, a.start_y, b.start_x, b.start_y, b.end_x, b.end_y),
               d_ps(a.end_x, a.end_y, b.start_x, b.start_y, b.end_x, b.end_y),
               d_ps(b.start_x, b.start_y, a.start_x, a.start_y, a.end_x, a.end_y),
               d_ps(b.end_x, b.end_y, a.start_x, a.start_y, a.end_x, a.end_y))


def intra_ok(segs, vias, p_id: int, n_id: int, track: float, via_size: float,
             clearance: float, tol: float = 0.002) -> Optional[str]:
    """The pair's own two legs keep the clearance from each other: segment
    to segment on one layer at track + clearance, a barrel to the other
    leg's segments (any layer) at via radius + clearance + half a track,
    barrel to barrel at via size + clearance. None when clean, else a
    line naming the first shortfall (the split's geometry at a hard turn
    was the source of every intra-pair DRC the K36 chain shipped)."""
    P = [s for s in segs if s.net_id == p_id]
    N = [s for s in segs if s.net_id == n_id]
    need_ss = track + clearance - tol
    for a in P:
        for b in N:
            if a.layer != b.layer:
                continue
            d = _seg_seg_dist(a, b)
            if d < need_ss:
                return (f'legs {d:.3f} < {need_ss + tol:.3f} mm apart on {a.layer} at '
                        f'({(a.start_x + a.end_x) / 2:.2f},{(a.start_y + a.end_y) / 2:.2f})')
    need_vs = via_size / 2.0 + clearance + track / 2.0 - tol
    for v in vias:
        other = N if v.net_id == p_id else P
        for b in other:
            from kicad_parser import Segment
            d = _seg_seg_dist(Segment(v.x, v.y, v.x, v.y, 0.0, b.layer, 0), b)
            if d < need_vs:
                return f'a barrel {d:.3f} < {need_vs + tol:.3f} mm from the other leg at ({v.x:.2f},{v.y:.2f})'
    need_vv = via_size + clearance - tol
    for v in vias:
        for w in vias:
            if v is not w and v.net_id != w.net_id and math.hypot(v.x - w.x, v.y - w.y) < need_vv:
                return f'barrels {math.hypot(v.x - w.x, v.y - w.y):.3f} < {need_vv + tol:.3f} mm apart at ({v.x:.2f},{v.y:.2f})'
    return None

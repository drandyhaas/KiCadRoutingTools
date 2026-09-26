"""plan_audit.py -- audits of a braid PLAN, before anything is routed.

Every corridor is planned exactly as braid.run plans it (braid.plan_corridors,
under the caller's environment: BRAID_BRANCH, ...), and the
plan is checked against the rules the router will apply to it. A plan that
fails one of these hands the router a lane it cannot route in band, so each
failure here is a refusal found without paying for the route.

usage: plan_audit.py CHECK --board B --nets N1,N2,..|@FILE [--dest REF] [--png OUT]

  pitch   two lanes' reservations (virtual_of: exactly what the router stamps
          for a lane not routed yet) nearer than the router's bar on one layer:
          track + clearance, plus half a grid step for each of the two pieces
          that is OFF the router's grid. A piece ON the grid (its ends grid
          points, running at 0, 45 or 90 degrees) is laid where it is planned;
          one off it lands up to half a step from its line. Also lists lanes
          with a reserved piece off the board's bounds.
  dives   every planned via site (virtual_vias_of: exit corners, split legs, a
          swimmer's reserved diamonds) against static copper of other
          nets (pads, teeth, berth stubs), other lanes' planned lines and other
          lanes' via sites, each bar plus half a grid step per participant off
          the grid (a pair's two barrels always are).
          A PAIR's site is its two barrels, pairs.dive_offset either side of the
          centreline across the arriving direction, each checked; via to via is
          the copper and the drill (hole-to-hole) rule, whichever binds.
The bars are the ROUTER's: the DRC clearance, plus half the router's grid step
(ctx.cfg.grid_step) for every participant that is off its grid -- the router
places a centre on a grid point, so an item planned off the grid may land half a
step away. Static copper stays where it is.
  bands   each lane's band -- the one route_lane hands the router -- against
          its planned line: the planned length no layer's band covers, and
          whether the band connects tooth to berth at all (a flood over band
          cells, no obstacles). --only N1,N2 restricts; --png DIR renders.
  swim    every SWIMMER's planned line (a lane with no page, which the plan
          lets weave, so no other lane's reservation is checked
          against it)
          against the lines that cross it: the crossings in order along it,
          by layer, and each F crossing and B crossing nearer each other than
          one layer change fits between (two via rooms, plus the pair pitch
          for a pair) -- a place the swimmer cannot weave. --only restricts.
  static  every lane's reservation against other nets' static copper on its
          layer (pads, base segments -- stubs and teeth --, base vias) nearer
          than track/2 + clearance + grid/2 to that copper's edge: a line through
          a pad, or one the router cannot keep on its grid.
  shape   every lane's reserved lines (virtual_of, chained per layer) for a
          shape no router lays: a FOLD (a vertex turning back more than
          100 degrees) or a NOTCH (two opposite turns of 60+ degrees nearer
          than a track width and a grid step -- a step or a dip). A turn is ALSO measured at the
          lane's own scale -- the directions into and out of a vertex taken
          track + clearance back and ahead along the line -- so a fold made in
          two or three steps over segments shorter than that is one fold. The
          pitch and dives audits measure one lane against another and cannot
          see either.
  near NET X,Y [R]   every other lane's reservation within R (two lane pitches) of (X, Y),
          and the plan facts of NET and of those lanes (page, layers, slots,
          layer profile).

--png OUT (pitch: a file; bands: a directory) draws the finding over the plan.
"""
import argparse
import collections
import contextlib
import io
import bisect
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
import braid as bd  # noqa: E402
import pairs as _pairs  # noqa: E402


def read_nets(arg):
    """A comma list, or @FILE holding one."""
    if arg.startswith('@'):
        arg = open(arg[1:]).read()
    return [n.strip() for n in arg.replace('\n', ',').split(',') if n.strip()]


def plan(board, nets, dest, quiet=True):
    """(ctx, corridors, log lines): the braid's plan of `nets`, every corridor
    planned as braid.run plans it, the board's copper reset to the base."""
    logs = []
    cm = contextlib.redirect_stdout(io.StringIO()) if quiet else contextlib.nullcontext()
    with cm:
        ctx, corridors = bd.plan_corridors(board, nets, dest, logs.append)
    return ctx, corridors, logs


def dseg(x, y, p, q):
    dx, dy = q[0] - p[0], q[1] - p[1]
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 < 1e-12 else max(0.0, min(1.0, ((x - p[0]) * dx + (y - p[1]) * dy) / l2))
    return math.hypot(x - p[0] - t * dx, y - p[1] - t * dy)


def _samples(segs, L, step):
    out = []
    for p, q, L_ in segs:
        if L_ != L:
            continue
        n = max(1, int(np.hypot(*(q - p)) / step))
        t = np.linspace(0, 1, n + 1)[:, None]
        out.append(p + (q - p) * t)
    return np.concatenate(out) if out else np.zeros((0, 2))


def _on_grid(p, q, g):
    """A reservation piece the router lays EXACTLY: both ends on grid points and its direction 0, 45 or 90
    degrees -- a planned line that IS a grid row (or diagonal) lands on itself, where an off-grid one lands up to
    half a step off. Every bar adds half a grid step per participant that is NOT on the grid."""
    if any(abs(x / g - round(x / g)) > 1e-3 for x in (p[0], p[1], q[0], q[1])):
        return False
    dx, dy = abs(q[0] - p[0]), abs(q[1] - p[1])
    return dx < 1e-9 or dy < 1e-9 or abs(dx - dy) < 1e-6


def _pt_on_grid(x, y, g):
    return abs(x / g - round(x / g)) <= 1e-3 and abs(y / g - round(y / g)) <= 1e-3


def _router_terminals(v, ends, g):
    """A single lane's pieces [(p, q, L)], each with the most the router's copper may stand off it at either end (its
    ALLOWANCE): none where it lies on the grid or is fixed, half a grid step at a free end off the grid, linear
    between. At an off-grid END the router lays a short stub exactly to the grid point the end rounds to
    (single_ended_routing: the start cell is the end rounded to the grid; the stub from the exact end, or the terminal
    vertex moved onto it), then moves onto the plan's grid: a snapped lane's terminal join is graded as THAT, exactly
    (a planned join to another grid point is not what gets laid); a smooth lane's first piece runs from none at its
    fixed end to half a step at its free one -- copper cannot stand further off its plan than it has run from where
    it is fixed. -> [(p, q, L, allowance at p, at q)]"""
    g2 = g / 2
    out = []
    near = lambda a, b: abs(a[0] - b[0]) < 1e-6 and abs(a[1] - b[1]) < 1e-6
    for (p, q, L) in v:
        p, q = np.asarray(p, float), np.asarray(q, float)
        done = False
        for E in ends:
            E = np.asarray(E, float)
            for a_, b_, rev in ((p, q, False), (q, p, True)):
                if done or not near(a_, E) or _pt_on_grid(E[0], E[1], g):
                    continue
                if _pt_on_grid(b_[0], b_[1], g):
                    r = np.round(E / g) * g
                    pcs = [(E, r, 0.0, 0.0)] + ([] if near(r, b_) else [(r, b_, 0.0, 0.0 if _on_grid(r, b_, g) else g2)])
                else:
                    pcs = [(E, b_, 0.0, g2)]
                if rev:
                    pcs = [(y, x, ay, ax) for (x, y, ax, ay) in reversed(pcs)]
                out += [(x, y, L, ax, ay) for (x, y, ax, ay) in pcs]
                done = True
        if not done:
            a0 = 0.0 if _on_grid(p, q, g) else g2
            out.append((p, q, L, a0, a0))
    return out


def _samples_allow(segs, L, step):
    """points every `step` along the pieces [(p, q, L, a0, a1)] on L, each with its allowance there"""
    P, A = [], []
    for p, q, L_, a0, a1 in segs:
        if L_ != L:
            continue
        n = max(1, int(np.hypot(*(q - p)) / step))
        t = np.linspace(0, 1, n + 1)
        P.append(p + (q - p) * t[:, None])
        A.append(a0 + (a1 - a0) * t)
    return (np.concatenate(P), np.concatenate(A)) if P else (np.zeros((0, 2)), np.zeros(0))


def _dist_allow(P, segs, L):
    """per point of P: (the distance to the pieces [(p, q, L, a0, a1)] on L less the nearest piece's allowance there,
    that distance, that allowance) -- the piece that binds, not merely the nearest"""
    S = [(p, q, a0, a1) for p, q, L_, a0, a1 in segs if L_ == L]
    best = np.full(len(P), 1e9)
    dd, aa = np.full(len(P), 1e9), np.zeros(len(P))
    if not S or not len(P):
        return best, dd, aa
    A_ = np.array([s_[0] for s_ in S]); B_ = np.array([s_[1] for s_ in S])
    a0_ = np.array([s_[2] for s_ in S]); a1_ = np.array([s_[3] for s_ in S])
    for i in range(0, len(A_), 256):
        a, b = A_[i:i + 256], B_[i:i + 256]
        d = b - a
        l2 = np.maximum((d ** 2).sum(1), 1e-12)
        t = np.clip(((P[:, None, :] - a[None]) * d[None]).sum(2) / l2[None], 0, 1)
        dist = np.hypot(*(P[:, None, :] - (a[None] + t[..., None] * d[None])).transpose(2, 0, 1))
        al = a0_[i:i + 256][None] + (a1_[i:i + 256] - a0_[i:i + 256])[None] * t
        m = dist - al
        k = m.argmin(1)
        r_ = np.arange(len(P))
        better = m[r_, k] < best
        best[better] = m[r_, k][better]
        dd[better] = dist[r_, k][better]
        aa[better] = al[r_, k][better]
    return best, dd, aa


def _dist_to_segs(P, segs, L):
    S = [(p, q) for p, q, L_ in segs if L_ == L]
    if not S or not len(P):
        return np.full(len(P), 1e9)
    A_ = np.array([p for p, q in S])
    B_ = np.array([q for p, q in S])
    best = np.full(len(P), 1e9)
    for i in range(0, len(A_), 256):
        a, b = A_[i:i + 256], B_[i:i + 256]
        d = b - a
        l2 = np.maximum((d ** 2).sum(1), 1e-12)
        t = np.clip(((P[:, None, :] - a[None]) * d[None]).sum(2) / l2[None], 0, 1)
        proj = a[None] + t[..., None] * d[None]
        best = np.minimum(best, np.hypot(*(P[:, None, :] - proj).transpose(2, 0, 1)).min(1))
    return best


def check_pitch(ctx, corridors, png=None):
    TW, CL, g = ctx.cfg.track_width, ctx.cfg.clearance, ctx.cfg.grid_step
    # the router's threading bar, either lane first: track + clearance, plus half a grid step for each of the two
    # pieces that is off the grid (it lands up to half a step off its plan; one ON the grid lands on itself)
    bar = lambda off_a, off_b: TW + CL + g / 2 * (off_a + off_b)
    block = bar(1, 1)
    bx0, by0, bx1, by1 = ctx.pcb.board_info.board_bounds
    hits, V_all = [], {}
    for c in corridors:
        s_h = getattr(c, 'handoff_s', None)
        V = {nm: [(np.array(p, float), np.array(q, float), L) for (p, q, L) in c.virtual_of([nm])]
             for nm in c.members}
        V_all.update(V)
        off = collections.Counter()
        for nm, v in V.items():
            for p, q, _L in v:
                if any(not (bx0 <= x <= bx1 and by0 <= y <= by1)
                       for x, y in (p + (q - p) * t for t in np.linspace(0, 1, 11))):
                    off[nm] += 1
        if off:
            print(f'PITCH corridor {c.idx}: lanes with a reserved piece off the board bounds: {dict(off)}')
        mem = list(c.members)
        # each piece with its ALLOWANCE (_router_terminals): none on the grid or at a fixed end, half a grid step at a
        # free end off it; a pair's pieces on the grid or off it
        prs_ = getattr(ctx, 'pairs', {}) or {}
        # (a pair's END LEGS -- whole_snap's connectors, laid by the pair step where they are drawn -- exact)
        exact = {nm: {(tuple(map(float, p)), tuple(map(float, q)), L) for (p, q, L) in
                      (c.end_legs_of(nm) if hasattr(c, 'end_legs_of') else [])} for nm in V}
        R = {nm: (_router_terminals(v, (c.teeth[nm], c.stubs[nm]), g) if nm not in prs_
                  else [(p, q, L, *((0.0, 0.0) if (_on_grid(p, q, g) or (tuple(map(float, p)), tuple(map(float, q)), L)
                                                   in exact[nm]) else (g / 2, g / 2))) for (p, q, L) in v])
             for nm, v in V.items()}
        for i, nm in enumerate(mem):
            for L in ctx.cfg.layers:
                Pa, Aa = _samples_allow(R[nm], L, g)
                if not len(Pa):
                    continue
                for om in mem[i + 1:]:
                    _m, d, ab = _dist_allow(Pa, R[om], L)
                    need = TW + CL + Aa + ab
                    bad = d < need - 1e-6
                    if bad.any():
                        k = int(np.argmin(d - need))
                        d_, need_, (x, y), ln_ = float(d[k]), float(need[k]), Pa[k], float(bad.sum()) * g
                        S, _O = c.spine.project(np.array([x]), np.array([y]))
                        S = float(np.asarray(S).ravel()[0])
                        where = 'branch' if s_h is not None and S >= s_h else 'trunk'
                        hits.append((d_, nm, om, L, (float(x), float(y)), S, where, ln_, c.idx, need_))
    hits.sort()
    for (d, a, b, L, xy, S, where, ln, ci, need) in hits:
        print(f'PITCH {a:7s} {b:7s} {L[0]} min {d:.3f}  over {ln:4.2f} mm  at ({xy[0]:.2f},{xy[1]:.2f})  '
              f'corridor {ci} s {S:6.2f} {where}  (bar {need:.4f})')
    print(f'PITCH {len(hits)} pair(s) short of their bar on one layer (track + clearance {TW + CL:.3f}, + half a '
          f'grid step per piece off the grid): {dict(collections.Counter(h[6] for h in hits))}')
    if png:
        from route_render import BoardRenderer
        from kicad_parser import Segment as _S
        allp = [p for v in V_all.values() for (p, q, L) in v] + [q for v in V_all.values() for (p, q, L) in v]
        view = (min(p[0] for p in allp) - 1, min(p[1] for p in allp) - 1,
                max(p[0] for p in allp) + 1, max(p[1] for p in allp) + 1)
        r = BoardRenderer(ctx.pcb, size=2000, supersample=2, show_zones=False, view=view, layer_alpha=70)

        def ov(dr, rr):
            for v in V_all.values():
                rr._draw_segments(dr, [_S(p[0], p[1], q[0], q[1], bd.TRACK / 3, L, 0) for p, q, L in v if L == 'F.Cu'],
                                  color=(230, 200, 60))
                rr._draw_segments(dr, [_S(p[0], p[1], q[0], q[1], bd.TRACK / 3, L, 0) for p, q, L in v if L != 'F.Cu'],
                                  color=(80, 160, 255))
            for h in hits:
                xy = h[4]
                rr._draw_segments(dr, [_S(xy[0] - bd.TRACK, xy[1], xy[0] + bd.TRACK, xy[1], 2 * bd.TRACK, 'F.Cu', 0)],
                                  color=(255, 0, 0))
        r.frame(segments=[], vias=[], overlays=[ov],
                label=f'reservations F yellow, B blue | red: two lanes within {block:.3f} mm on one layer '
                      f'({len(hits)} pairs)').save(png)
        print('wrote', png)
    return hits


def _cross_t(a, b, p, q):
    """The parameter along a->b where it crosses p->q, or None."""
    rx, ry = b[0] - a[0], b[1] - a[1]
    sx, sy = q[0] - p[0], q[1] - p[1]
    den = rx * sy - ry * sx
    if abs(den) < 1e-12:
        return None
    t = ((p[0] - a[0]) * sy - (p[1] - a[1]) * sx) / den
    u = ((p[0] - a[0]) * ry - (p[1] - a[1]) * rx) / den
    return t if (0.0 <= t <= 1.0 and 0.0 <= u <= 1.0) else None


def check_swim(ctx, corridors, only=None):
    """Each swimmer's crossings along its planned line, and the F/B crossings
    too close together for it to change layer between them."""
    prs = getattr(ctx, 'pairs', {}) or {}
    bad_all = []
    for c in corridors:
        sc = getattr(c, 'sched_cur', None)
        if sc is None:
            continue
        for nm in c.members:
            if sc.page.get(nm) is not None or (only and nm not in only):
                continue          # a page lane does not weave
            xy = list((getattr(c, 'lane_xy', {}) or {}).get(nm) or [])
            if len(xy) < 2:
                continue
            need = 2 * bd.VIA_NEED + (_pairs.pitch(bd.TRACK) if nm in prs else 0.0)
            cum = [0.0]
            for a, b in zip(xy, xy[1:]):
                cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
            hits = []
            for om in c.members:
                if om == nm:
                    continue
                for (p, q, L) in c.virtual_of([om]):
                    for k, (a, b) in enumerate(zip(xy, xy[1:])):
                        t = _cross_t(a, b, p, q)
                        if t is not None:
                            hits.append((cum[k] + t * (cum[k + 1] - cum[k]), L, om))
            # a pair's two legs cross twice: one crossing per lane and layer within one and a half pair pitches
            hits.sort()
            merged = []
            leg_span = 1.5 * _pairs.pitch(bd.TRACK)
            for h in hits:
                if merged and merged[-1][2] == h[2] and merged[-1][1] == h[1] and h[0] - merged[-1][0] < leg_span:
                    continue
                merged.append(h)
            tri = [(a_, b_) for a_, b_ in zip(merged, merged[1:]) if a_[1] != b_[1] and b_[0] - a_[0] < need]
            kind = 'pair' if nm in prs else 'single'
            seq = ' | '.join(f'{d:.2f} {L[0]} {om}' for d, L, om in merged) or 'none'
            print(f'SWIM {nm:7s} ({kind}) corridor {c.idx} line {cum[-1]:.2f} mm, {len(merged)} crossing(s): {seq}')
            for a_, b_ in tri:
                print(f'  CANNOT WEAVE: {a_[2]} {a_[1][0]} at {a_[0]:.2f} / {b_[2]} {b_[1][0]} at {b_[0]:.2f} '
                      f'-- {b_[0] - a_[0]:.2f} mm apart, a layer change needs {need:.2f}')
                bad_all.append((nm, a_, b_))
    print(f'SWIM {len(bad_all)} place(s) a swimmer cannot weave')
    return bad_all


def _pad_edge(x, y, pd):
    """signed distance to a pad's copper as KiCad draws it: rounded corners, a stadium for an oval"""
    return float(_pairs.pad_distance(x - pd.global_x, y - pd.global_y, pd.size_x / 2, pd.size_y / 2,
                                     _pairs.pad_corner_radius(pd)))


def _straight_len(pieces, s, u, tol=1e-6):
    """how far a centreline [(p, q, layer)] runs straight from s along the unit heading u, its pieces on that line
    joined end to end in any order and across layers (the pair router's straight runs on through a dive): 0 when no
    piece leaves s along u"""
    iv = []
    for (p, q, _L) in pieces:
        tp = (p[0] - s[0]) * u[0] + (p[1] - s[1]) * u[1]
        tq = (q[0] - s[0]) * u[0] + (q[1] - s[1]) * u[1]
        if abs((p[0] - s[0]) * u[1] - (p[1] - s[1]) * u[0]) < tol and abs((q[0] - s[0]) * u[1] - (q[1] - s[1]) * u[0]) < tol:
            iv.append((min(tp, tq), max(tp, tq)))
    reach, grew = 0.0, True
    while grew:
        grew = False
        for t0, t1 in iv:
            if t0 <= reach + tol and t1 > reach + tol:
                reach, grew = t1, True
    return reach


def _dive_straight(pieces, s, tol=1e-4):
    """((straight length arriving at the dive site s, its heading), (leaving it, its heading)) along a centreline
    [(p, q, layer)] in any order: the pieces chained per layer, the run that ends at s on one layer and the run
    that starts at s on the other, headings as unit vectors along travel. None when s is not where two layers'
    runs meet."""
    at = lambda pt: math.hypot(pt[0] - s[0], pt[1] - s[1]) < tol
    ends = []
    for L in sorted({pc[2] for pc in pieces}):
        for run in bd._chain_runs(pieces, L):
            if at(run[-1]):
                ends.append(list(run))
            elif at(run[0]):
                ends.append(list(reversed(run)))
    if len(ends) != 2:
        return None

    def straight(run):
        """(length, heading) of the collinear stretch ending at run[-1]"""
        pts = [run[-1]]
        for q in reversed(run[:-1]):
            if math.hypot(q[0] - pts[-1][0], q[1] - pts[-1][1]) > 1e-9:
                pts.append(q)
        if len(pts) < 2:
            return 0.0, (1.0, 0.0)
        d0 = math.hypot(pts[0][0] - pts[1][0], pts[0][1] - pts[1][1])
        u = ((pts[0][0] - pts[1][0]) / d0, (pts[0][1] - pts[1][1]) / d0)       # toward s
        L = 0.0
        for a_, b_ in zip(pts, pts[1:]):
            dx, dy = a_[0] - b_[0], a_[1] - b_[1]
            dd = math.hypot(dx, dy)
            if (dx * u[0] + dy * u[1]) / dd < 1 - 1e-6:
                break
            L += dd
        return L, u
    (la, ua), (lb, ub) = straight(ends[0]), straight(ends[1])
    # one run arrives at s (heading toward it), the other leaves (heading away): travel through the dive
    return (la, ua), (lb, (-ub[0], -ub[1]))


def check_dives(ctx, corridors, show_all=False):
    cfg = ctx.cfg
    VR, CL, TW = cfg.via_size / 2, cfg.clearance, cfg.track_width
    g = cfg.grid_step
    g2 = g / 2                                   # the router puts a via on its grid
    h2h = getattr(cfg, 'hole_to_hole_clearance', 0.0) or 0.0
    from fab_tiers import min_via_center_distance
    # every bar: the clearance, plus half a grid step for each participant OFF the grid (static copper stays put;
    # a via on a grid point, or a lane piece on a grid line, lands on itself)
    c2c = min_via_center_distance(cfg.via_size, CL, cfg.via_drill, h2h)
    pairs = getattr(ctx, 'pairs', {}) or {}
    off = _pairs.dive_offset(cfg, _pairs.pitch(TW) / 2) if pairs else 0.0
    name = lambda i: (ctx.pcb.nets[i].name.split('/')[-1] if i in ctx.pcb.nets else str(i))
    pads = [(fp.reference, pd) for fp in ctx.pcb.footprints.values() for pd in fp.pads
            if pd.pad_type != 'np_thru_hole' and any(L.endswith('.Cu') for L in pd.layers)]
    fail = collections.Counter()
    n_sites = 0
    for c in corridors:
        M = list(c.members)
        own = {nm: {ctx.byname[nm][0]} | {ctx.byname[leg][0] for leg in pairs.get(nm, ()) if leg in ctx.byname}
               for nm in M}
        sites = {nm: [tuple(p) for p in c.virtual_vias_of([nm])] for nm in M}
        lines = {nm: c.virtual_of([nm]) for nm in M}
        centre = getattr(c, '_virtual_of_plain', c.virtual_of)
        # a crossed pair's crossover barrels (whole_snap: laid where they are drawn, one leg each)
        cross = {nm: (c.cross_of(nm) if hasattr(c, 'cross_of') else None) for nm in M}
        exact_b = {nm: ({(float(v_[0]), float(v_[1])) for v_ in cross[nm]['vias']} if cross[nm] else set()) for nm in M}
        # the copper each site stands for: one barrel, or a pair's two
        barrels = {nm: {s: (_pairs.dive_barrels(s, centre([nm]), off) if nm in pairs and s not in exact_b[nm] else [s])
                        for s in sites[nm]} for nm in M}
        # a barrel off the grid (one laid where it is drawn lands on itself)
        boff = lambda nm, x, y: 0 if ((x, y) in exact_b[nm] or (nm not in pairs and _pt_on_grid(x, y, g))) else 1
        half_ = _pairs.pitch(TW) / 2
        ring = {om: _pairs.via_ring(cfg, half_ if om in pairs else 0.0) for om in M}
        ring_lines = {om: (centre([om]) if om in pairs else lines[om]) for om in M}
        for nm in M:
            for s in sites[nm]:
                n_sites += 1
                hits = []
                for (x, y) in barrels[nm][s]:
                    near = lambda px, py: abs(px - x) < 2 and abs(py - y) < 2
                    vo = boff(nm, x, y)
                    need_static = VR + CL + g2 * vo
                    # (distance, its bar, what): a via's bar is the via-to-via
                    # rule (copper and drill), everything else copper's
                    st = min([(_pad_edge(x, y, pd), need_static, f'pad {ref}.{pd.pad_number} {name(pd.net_id)}')
                              for ref, pd in pads if pd.net_id not in own[nm] and near(pd.global_x, pd.global_y)]
                             + [(dseg(x, y, (s_.start_x, s_.start_y), (s_.end_x, s_.end_y)) - s_.width / 2,
                                 need_static, f'{s_.layer[0]} copper {name(s_.net_id)}')
                                for s_ in ctx.base_segments if s_.net_id not in own[nm] and near(s_.start_x, s_.start_y)]
                             + [(math.hypot(v.x - x, v.y - y),
                                 max(VR + v.size / 2 + CL, cfg.via_drill / 2 + v.drill / 2 + h2h) + g2 * vo,
                                 f'via {name(v.net_id)}')
                                for v in ctx.base_vias if v.net_id not in own[nm] and near(v.x, v.y)],
                             key=lambda r: r[0] - r[1], default=(9, 0, ''))
                    if st[0] < st[1] - 1e-6:
                        hits.append(f'static {st[0]:+.3f}/{st[1]:.3f} ({st[2]})')
                        fail['static'] += 1
                    # a via keeps a track's grid cells out of its RING (pairs.via_ring: the clearance rounded up to
                    # whole cells plus a quarter, and as far again as the via stands off its grid point); a pair's
                    # centreline by the pair's ring -- the router's pair map is its centreline's
                    # (a site ON the grid is where the router puts it: its barrels' own offsets; a smooth plan's
                    # site is not yet placed -- half a step, the allowance the polish leaves it)
                    voff = (math.hypot(x - round(x / g) * g, y - round(y / g) * g)
                            if _pt_on_grid(s[0], s[1], g) or s in exact_b[nm] else g2)
                    ln = min(((dseg(x, y, p, q), ring[om] + voff + g2 * (0 if _on_grid(p, q, g) else 1), om, L)
                              for om in M if om != nm for (p, q, L) in ring_lines[om]),
                             key=lambda r: r[0] - r[1], default=(9, 0, '', ''))
                    if ln[0] < ln[1] - 1e-6:
                        hits.append(f'lane {ln[0]:.3f}/{ln[1]:.3f} ({ln[2]} {ln[3][0]})')
                        fail['lane'] += 1
                    vv = min(((math.hypot(px - x, py - y), c2c + g2 * (vo + boff(om, px, py)), om) for om in M if om != nm
                              for bs in barrels[om].values() for (px, py) in bs),
                             key=lambda r: r[0] - r[1], default=(9, 0, ''))
                    if vv[0] < vv[1] - 1e-6:
                        hits.append(f'via {vv[0]:.3f}/{vv[1]:.3f} ({vv[2]})')
                        fail['via'] += 1
                if nm in pairs:
                    # the pair router dives straight: one heading through the via, and that many grid steps of it on
                    # each side (pairs.via_straight_steps) -- a plan that turns at its dive is one it cannot follow
                    st_ = _dive_straight(centre([nm]), s)
                    if st_ is not None:
                        (lb, ub), (la, ua) = st_
                        need_b, need_a = _pairs.via_straight(cfg, ub), _pairs.via_straight(cfg, ua)
                        turn_ = ub[0] * ua[0] + ub[1] * ua[1] < 1 - 1e-6
                        if turn_ or lb < need_b - 1e-6 or la < need_a - 1e-6:
                            hits.append(f'straight {lb:.3f}/{need_b:.3f} before, {la:.3f}/{need_a:.3f} after'
                                        + (f', turning {math.degrees(math.acos(max(-1.0, min(1.0, ub[0] * ua[0] + ub[1] * ua[1])))):.0f} deg' if turn_ else ''))
                            fail['straight'] += 1
                    # ...and no nearer either end than its DIVE ROOM (pairs.dive_room: its end connector from the tips
                    # to the pose where the pair router takes over, then the router's straight from the pose into the
                    # via): a dive inside it is one the router cannot reach (SDQS0 once dived 0.53 mm from its berth)
                    pcs_ = centre([nm])
                    k_ = next((k for k in range(1, len(pcs_)) if pcs_[k][2] != pcs_[k - 1][2]
                               and math.hypot(pcs_[k][0][0] - s[0], pcs_[k][0][1] - s[1]) < 1e-6), None)
                    if k_ is not None and nm in ctx.pair_ends:
                        plen = [math.hypot(q[0] - p[0], q[1] - p[1]) for (p, q, _L) in pcs_]
                        a0, a1 = sum(plen[:k_]), sum(plen[k_:])
                        ub, ua = (st_[0][1], st_[1][1]) if st_ is not None else ((1.0, 0.0), (1.0, 0.0))
                        need0 = _pairs.dive_room(cfg, ctx.pair_ends[nm][0], ub)
                        need1 = _pairs.dive_room(cfg, ctx.pair_ends[nm][1], ua)
                        if a0 < need0 - 1e-6 or a1 < need1 - 1e-6:
                            hits.append(f'end {a0:.3f}/{need0:.3f} from its tooth, {a1:.3f}/{need1:.3f} from its berth')
                            fail['end'] += 1
                if hits or show_all:
                    tag = ' pair' if nm in pairs and s not in exact_b[nm] else (' crossover' if nm in pairs else '')
                    print(f'DIVE {nm:7s} ({s[0]:7.2f},{s[1]:6.2f}){tag}  ' + ('; '.join(hits) if hits else 'ok'))
            # a pair with END CONNECTORS: the pair router runs pose to pose, and from a pose it looks
            # pairs.pose_probe_steps straight ahead before it accepts it -- that many grid steps straight along the
            # heading out of the pose it starts from and into the pose it ends at (whole_snap's search owes the same);
            # a crossed pair's crossover is two such poses round its centre, its half-span and those steps each side
            ends_ = c.ends_of(nm) if hasattr(c, 'ends_of') else None
            if nm in pairs and ends_:
                pcs_ = centre([nm])[1:-1]
                PR = _pairs.pose_probe_steps(cfg)
                step_ = lambda u: g / max(abs(u[0]), abs(u[1]))
                hits = []
                for k_, e_ in enumerate(ends_):
                    u_ = tuple(map(float, e_['heading']))
                    got, need_ = _straight_len(pcs_, tuple(e_['pose']), u_), PR * step_(u_)
                    if got < need_ - 1e-6:
                        hits.append(f'pose {got:.3f}/{need_:.3f} straight from its {("tooth", "berth")[k_]} pose')
                        fail['pose'] += 1
                xo = cross[nm]
                if xo is not None:
                    u_ = tuple(map(float, xo['heading']))
                    at_ = tuple(map(float, xo['at']))
                    lb = _straight_len(pcs_, at_, (-u_[0], -u_[1]))
                    la = _straight_len(pcs_, at_, u_)
                    nb, na = -xo['span'][0] + PR * step_(u_), xo['span'][1] + PR * step_(u_)
                    if lb < nb - 1e-6 or la < na - 1e-6:
                        hits.append(f'crossover {lb:.3f}/{nb:.3f} straight before, {la:.3f}/{na:.3f} after')
                        fail['crossover'] += 1
                if hits or show_all:
                    print(f'DIVE {nm:7s} poses  ' + ('; '.join(hits) if hits else 'ok'))
    print(f'DIVE {n_sites} planned via sites'
          + (f' (a pair\'s dive = two barrels {off:.3f} either side of its centreline)' if pairs else '')
          + f'; failing: {dict(fail)}')
    return fail


def check_static(ctx, corridors, only=None):
    """Each lane's reservation (virtual_of) against the static copper of OTHER
    nets on its layer -- pads (a drilled pad on both layers, an unplated hole by
    its drill), the base segments (other nets' and other lanes' stubs and teeth)
    and base vias -- nearer than half a track plus the clearance to that
    copper's edge: a planned line the router cannot sit on. The pitch audit
    measures lanes against lanes and the dives audit vias against static
    copper; neither sees a line through a pad."""
    cfg = ctx.cfg
    TW, CL, g = cfg.track_width, cfg.clearance, cfg.grid_step
    need = TW / 2 + CL + g / 2                   # the bar for a lane piece off the grid (it lands up to half a step off)
    pairs = getattr(ctx, 'pairs', {}) or {}
    pads = []
    for fp in ctx.pcb.footprints.values():
        for pd in fp.pads:
            drilled = bool(pd.drill and pd.drill > 0)
            if pd.pad_type == 'np_thru_hole':
                pads.append((fp.reference, pd, {'F.Cu', 'B.Cu'}, 'hole'))
            elif drilled or any(L.startswith('*') for L in pd.layers):
                pads.append((fp.reference, pd, {'F.Cu', 'B.Cu'}, 'pad'))
            else:
                Ls = {L for L in pd.layers if L in ('F.Cu', 'B.Cu')}
                if Ls:
                    pads.append((fp.reference, pd, Ls, 'pad'))
    name = lambda i: (ctx.pcb.nets[i].name.split('/')[-1] if i in ctx.pcb.nets else str(i))
    hits = {}
    for c in corridors:
        for nm in c.members:
            if only and nm not in only:
                continue
            own = {ctx.byname[nm][0]} | {ctx.byname[leg][0] for leg in pairs.get(nm, ()) if leg in ctx.byname}
            R = c.virtual_of([nm])
            exact_ = {(tuple(map(float, p)), tuple(map(float, q)), L_) for (p, q, L_) in
                      (c.end_legs_of(nm) if hasattr(c, 'end_legs_of') else [])}
            is_on = lambda s_: _on_grid(s_[0], s_[1], g) or (tuple(map(float, s_[0])), tuple(map(float, s_[1])), s_[2]) in exact_
            for L in ('F.Cu', 'B.Cu'):
                RR = [(np.array(p, float), np.array(q, float), L_) for p, q, L_ in R]
                on_ = [s for s in RR if is_on(s)]
                off_ = [s for s in RR if not is_on(s)]
                P_on, P_off = _samples(on_, L, g), _samples(off_, L, g)
                P = np.concatenate([P_on, P_off]) if len(P_on) and len(P_off) else (P_on if len(P_on) else P_off)
                # each sample's bar: track/2 + clearance, plus half a grid step where its piece is off the grid
                NEED = np.concatenate([np.full(len(P_on), TW / 2 + CL), np.full(len(P_off), need)])[:len(P)]
                if not len(P):
                    continue
                x0, y0 = P.min(0) - 4 * bd.LANE_MIN       # the static copper near the lane: a few lane pitches round it
                x1, y1 = P.max(0) + 4 * bd.LANE_MIN
                inb = lambda x, y: x0 <= x <= x1 and y0 <= y <= y1
                cand = []
                for ref, pd, Ls, kind in pads:
                    if L not in Ls or pd.net_id in own or not inb(pd.global_x, pd.global_y):
                        continue
                    if kind == 'hole':
                        d = np.hypot(P[:, 0] - pd.global_x, P[:, 1] - pd.global_y) - (pd.drill or 0) / 2
                    else:       # the copper as KiCad draws it: rounded corners, a stadium for an oval
                        d = _pairs.pad_distance(P[:, 0] - pd.global_x, P[:, 1] - pd.global_y, pd.size_x / 2,
                                                pd.size_y / 2, _pairs.pad_corner_radius(pd))
                    cand.append((d, f'{kind} {ref}.{pd.pad_number} {name(pd.net_id)}'))
                for s in ctx.base_segments:
                    if s.layer != L or s.net_id in own or not (inb(s.start_x, s.start_y) or inb(s.end_x, s.end_y)):
                        continue
                    d = _dist_to_segs(P, [(np.array([s.start_x, s.start_y]), np.array([s.end_x, s.end_y]), L)], L)
                    cand.append((d - s.width / 2, f'copper {name(s.net_id)}'))
                for v in ctx.base_vias:
                    if v.net_id in own or not inb(v.x, v.y):
                        continue
                    cand.append((np.hypot(P[:, 0] - v.x, P[:, 1] - v.y) - v.size / 2, f'via {name(v.net_id)}'))
                for d, what in cand:
                    k = int(np.argmin(d - NEED))
                    if d[k] < NEED[k] - 1e-6:
                        key = (nm, L, what)
                        if key not in hits or d[k] - NEED[k] < hits[key][0] - hits[key][2]:
                            hits[key] = (float(d[k]), tuple(P[k]), float(NEED[k]))
    rows = sorted((v[0], k[0], k[1], k[2], v[1], v[2]) for k, v in hits.items())
    for d, nm, L, what, xy, nd in rows:
        print(f'STATIC {nm:7s} {L[0]} {d:+.3f}/{nd:.3f} {what} at ({xy[0]:.2f},{xy[1]:.2f})')
    print(f'STATIC {len(rows)} lane/object pair(s) short of their bar (track/2 + clearance {TW / 2 + CL:.3f}, + half a '
          f'grid step off the grid): {dict(collections.Counter(r[3].split()[0] for r in rows))}')
    return rows


def router_band(c, nm):
    """(kind, band): the band route_lane hands the router for `nm`'s search
    (a swimmer is searched free: None)."""
    if c.sched_cur.page.get(nm) is None:
        return 'free', None
    return 'page', c.band_of(nm)


def _flood(ok, xs, ys, layers, a, aL, b, bL, G):
    ij = lambda p: (int(round((p[0] - xs[0]) / G)), int(round((p[1] - ys[0]) / G)))
    li = {L: k for k, L in enumerate(layers)}
    open_at = lambda c_: 0 <= c_[0] < len(xs) and 0 <= c_[1] < len(ys) and ok[layers[c_[2]]][c_[0], c_[1]]

    def seed(c_):
        if open_at(c_):
            return c_
        r = int(math.ceil(bd.LANE_MIN / G))
        best = None
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                cc = (c_[0] + di, c_[1] + dj, c_[2])
                if open_at(cc) and (best is None or di * di + dj * dj < best[0]):
                    best = (di * di + dj * dj, cc)
        return best[1] if best else None
    s, t = seed(ij(a) + (li[aL],)), seed(ij(b) + (li[bL],))
    if s is None:
        return 'TOOTH-OUT'
    if t is None:
        return 'BERTH-OUT'
    seen = {s}
    q = collections.deque([s])
    while q:
        c_ = q.popleft()
        if c_ == t:
            return 'connected'
        i, j, L = c_
        for n in [(i + di, j + dj, L) for di in (-1, 0, 1) for dj in (-1, 0, 1) if di or dj] + [(i, j, 1 - L)]:
            if n not in seen and open_at(n):
                seen.add(n)
                q.append(n)
    return 'BROKEN'


def check_bands(ctx, corridors, only=None, png_dir=None):
    """sampled on the router's own grid (a band may be a single grid line wide: a snapped lane's), a few lane pitches
    round the plan"""
    layers = list(ctx.cfg.layers)
    G = ctx.cfg.grid_step
    pad_ = 4 * bd.LANE_MIN
    tot_out = 0.0
    for c in corridors:
        for nm in c.members:
            if only and nm not in only:
                continue
            if nm not in c.lane_xy:
                print(f'BAND {nm:7s} no planned line')
                continue
            kind, band = router_band(c, nm)
            if band is None:
                print(f'BAND {nm:7s} {kind:5s} no band (free)')
                continue
            # the plan's own line: a whole-route plan's exact geometry (its lane_xy is simplified for the search window)
            geo_ = getattr(c, '_geo', None)
            w = [tuple(p) for p in geo_['lanes'][nm]['xy']] if geo_ is not None and nm in geo_['lanes'] else list(c.lane_xy[nm])
            a, b = c.teeth[nm], c.stubs[nm]
            ends_ = geo_['lanes'][nm].get('ends') if geo_ is not None and nm in geo_['lanes'] else None
            if ends_:
                # a pair with END CONNECTORS: the router's band runs pose to pose (the legs are laid as drawn)
                pcs_ = geo_['lanes'][nm]['pieces'][1:-1]
                w = [tuple(pcs_[0][:2])] + [tuple(p_[2:4]) for p_ in pcs_]
                a, b = tuple(ends_[0]['pose']), tuple(ends_[1]['pose'])
            P = w + [a, b]
            x0, x1 = min(p[0] for p in P) - pad_, max(p[0] for p in P) + pad_
            y0, y1 = min(p[1] for p in P) - pad_, max(p[1] for p in P) + pad_
            xs = np.arange(math.floor(x0 / G), math.ceil(x1 / G) + 1) * G
            ys = np.arange(math.floor(y0 / G), math.ceil(y1 / G) + 1) * G
            ok = {L: np.asarray(band(xs, ys, L), dtype=bool) for L in layers}
            pts = []
            for p, q in zip(w, w[1:]):
                # a step per grid step along the leading axis: an octilinear piece on the grid is sampled at its own
                # grid points
                n = max(1, int(math.ceil(max(abs(q[0] - p[0]), abs(q[1] - p[1])) / G - 1e-6)))
                pts += [(p[0] + (q[0] - p[0]) * k / n, p[1] + (q[1] - p[1]) * k / n) for k in range(n)]
            runs, cur, s, out, prev = [], None, 0.0, 0.0, pts[0]
            for (x, y) in pts:
                ds = math.hypot(x - prev[0], y - prev[1])
                s += ds
                prev = (x, y)
                i, j = int(round((x - xs[0]) / G)), int(round((y - ys[0]) / G))
                st = ''.join(L[0] if 0 <= i < len(xs) and 0 <= j < len(ys) and ok[L][i, j] else '-'
                             for L in layers)
                if st == '-' * len(layers):
                    out += ds
                if cur is None or cur[0] != st:
                    if cur:
                        runs.append(cur)
                    cur = [st, s, s]
                else:
                    cur[2] = s
            runs.append(cur)
            tot_out += out
            fl = _flood(ok, xs, ys, layers, a, ctx.tooth_layer[nm], b, ctx.dest_layer[nm], G)
            gaps = [r for r in runs if r[0] == '-' * len(layers) and r[2] - r[1] > 2 * G]
            print(f'BAND {nm:7s} {kind:5s} plan {s:5.1f} mm  outside {out:4.1f} mm  {fl:9s} '
                  + ' '.join(f'out:{r[1]:.1f}-{r[2]:.1f}' for r in gaps))
            if png_dir:
                from route_render import BoardRenderer
                from kicad_parser import Segment as _S
                os.makedirs(png_dir, exist_ok=True)
                r = BoardRenderer(ctx.pcb, size=1400, supersample=2, show_zones=False,
                                  view=(x0, y0, x1, y1), layer_alpha=110)

                def ov(d, rr, ok=ok, xs=xs, ys=ys, w=w, a=a, b=b):
                    both = ok[layers[0]] & ok[layers[1]]
                    for L, col in zip(layers, ((0, 150, 60), (170, 40, 170))):
                        bi, bj = np.nonzero(ok[L] & ~both)
                        rr._draw_segments(d, [_S(xs[i], ys[j], xs[i] + bd.TRACK / 100, ys[j], bd.TRACK / 4, 'F.Cu', 0)
                                              for i, j in zip(bi, bj)], color=col)
                    bi, bj = np.nonzero(both)
                    rr._draw_segments(d, [_S(xs[i], ys[j], xs[i] + bd.TRACK / 100, ys[j], bd.TRACK / 4, 'F.Cu', 0)
                                          for i, j in zip(bi, bj)], color=(90, 110, 160))
                    rr._draw_segments(d, [_S(p[0], p[1], q[0], q[1], 0.4 * bd.TRACK, 'F.Cu', 0) for p, q in zip(w, w[1:])],
                                      color=(255, 255, 255))
                    for p, col in ((a, (255, 80, 80)), (b, (80, 160, 255))):
                        rr._draw_segments(d, [_S(p[0] - 0.6 * bd.TRACK, p[1], p[0] + 0.6 * bd.TRACK, p[1], 1.2 * bd.TRACK, 'F.Cu', 0)], color=col)
                r.frame(segments=[], vias=[], overlays=[ov],
                        label=f'{nm} {kind} band: F only green, B only magenta, both blue | plan white | '
                              f'tooth red, berth blue | {fl}').save(os.path.join(png_dir, f'{nm}.png'))
    print(f'BAND total planned length outside its band: {tot_out:.1f} mm')
    return tot_out


TINY = bd.TRACK / 25         # a piece shorter than this (5 um) is the writers' rounding, not a piece


def _polyline(run):
    """a polyline as the plan's writers leave it: its sub-5 um segments dropped and its collinear runs merged (the
    polish writes its pieces so) -- a shape measured on a dense vertex list must see the turns the written plan
    has: a collinear vertex between two sharp turns hid SDQ13's notch from the polish, then the audit found it"""
    pts = [run[0]]
    for q in run[1:]:
        if math.hypot(q[0] - pts[-1][0], q[1] - pts[-1][1]) > TINY:
            pts.append(q)
    out = [pts[0]]
    start = pts[0]
    for q in pts[1:]:
        if len(out) >= 2:
            e = out[-1]
            if abs((e[0] - start[0]) * (q[1] - start[1]) - (e[1] - start[1]) * (q[0] - start[0])) < 1e-7:
                out[-1] = q
                continue
            start = out[-1]
        out.append(q)
    return out


def _turns(run):
    """(vertex, signed turn in degrees, length of the segment after it) along
    a polyline as its writers leave it (_polyline)."""
    pts = _polyline(run)
    out = []
    for a_, b_, c_ in zip(pts, pts[1:], pts[2:]):
        h1 = math.atan2(b_[1] - a_[1], b_[0] - a_[0])
        h2 = math.atan2(c_[1] - b_[1], c_[0] - b_[0])
        t = math.degrees((h2 - h1 + math.pi) % (2 * math.pi) - math.pi)
        out.append((b_, t, math.hypot(c_[0] - b_[0], c_[1] - b_[1])))
    return out


def _scale_turns(run, w):
    """(vertex, signed turn in degrees) at every inner vertex of a polyline,
    the directions measured over `w` either side along it: a turn made over
    segments shorter than w is ONE turn at this scale."""
    pts = _polyline(run)
    if len(pts) < 3:
        return []
    s = [0.0]
    for a_, b_ in zip(pts, pts[1:]):
        s.append(s[-1] + math.hypot(b_[0] - a_[0], b_[1] - a_[1]))

    def at(u):
        u = min(max(u, 0.0), s[-1])
        k = max(1, min(len(s) - 1, bisect.bisect_left(s, u)))
        f = 0.0 if s[k] - s[k - 1] < 1e-12 else (u - s[k - 1]) / (s[k] - s[k - 1])
        return (pts[k - 1][0] + (pts[k][0] - pts[k - 1][0]) * f, pts[k - 1][1] + (pts[k][1] - pts[k - 1][1]) * f)
    out = []
    for i in range(1, len(pts) - 1):
        a_, b_, c_ = at(s[i] - w), pts[i], at(s[i] + w)
        if math.hypot(b_[0] - a_[0], b_[1] - a_[1]) < 1e-3 or math.hypot(c_[0] - b_[0], c_[1] - b_[1]) < 1e-3:
            continue
        h1 = math.atan2(b_[1] - a_[1], b_[0] - a_[0])
        h2 = math.atan2(c_[1] - b_[1], c_[0] - b_[0])
        out.append((b_, math.degrees((h2 - h1 + math.pi) % (2 * math.pi) - math.pi)))
    return out


def check_shape(ctx, corridors, only=None):
    hits = []
    W = ctx.cfg.track_width + ctx.cfg.clearance          # the lane's own scale
    NOTCH = ctx.cfg.track_width + ctx.cfg.grid_step      # a step or a dip shorter than a track and a grid step
    for c in corridors:
        for nm in c.members:
            if only and nm not in only:
                continue
            R = c.virtual_of([nm])
            for L in ('F.Cu', 'B.Cu'):
                for run in bd._chain_runs(R, L):
                    tr = _turns(run)
                    for i, (v, t, _l) in enumerate(tr):
                        if abs(t) > 100:
                            hits.append((nm, L, 'fold', v, f'{t:+.0f}'))
                    for v, t in _scale_turns(run, W):
                        if abs(t) > 100:
                            hits.append((nm, L, 'fold', v, f'{t:+.0f} over {W:.3f}'))
                    for (v1, t1, l1), (v2, t2, _l2) in zip(tr, tr[1:]):
                        if abs(t1) > 60 and abs(t2) > 60 and t1 * t2 < 0 and l1 < NOTCH:
                            hits.append((nm, L, 'notch', v1, f'{t1:+.0f}/{t2:+.0f} over {l1:.3f}'))
    seen = set()
    for nm, L, kind, v, how in hits:
        key = (nm, L, kind, round(v[0] / W), round(v[1] / W))     # one report per place, at the lane's scale
        if key in seen:
            continue
        seen.add(key)
        print(f'SHAPE {nm:7s} {L[0]} {kind:5s} at ({v[0]:.2f},{v[1]:.2f}) {how}')
    print(f'SHAPE {len(seen)} place(s): {dict(collections.Counter(k[2] for k in seen))} '
          f'in {len({k[0] for k in seen})} lane(s)')
    return hits


def show_near(ctx, corridors, nm, P, R=None):
    R = 2 * bd.LANE_MIN if R is None else R
    c = next((c for c in corridors if nm in c.members), None)
    if c is None:
        print(f'{nm} is not planned')
        return
    rows, seen = [], set()
    for d, om, L in sorted((round(dseg(P[0], P[1], p, q), 3), om, L[0]) for om in c.members if om != nm
                           for p, q, L in c.virtual_of([om]) if dseg(P[0], P[1], p, q) < R):
        if (om, L) not in seen:
            seen.add((om, L))
            rows.append((d, om, L))
    print(f'near ({P[0]}, {P[1]}) within {R}: ' + ', '.join(f'{om} {L} {d}' for d, om, L in rows))
    sc = c.sched_cur
    for om in [nm] + sorted({r[1] for r in rows}):
        try:
            prof = [(round(s, 2), L[0]) for s, L in c.layer_profile(om)]
        except Exception:
            prof = None
        print(f'  {om:7s} page {sc.page.get(om)} tooth {ctx.tooth_layer[om][0]} dest {ctx.dest_layer[om][0]} '
              f'st {tuple(round(v, 2) for v in c.st[om])} launch {c.launch_o[om]:.2f} '
              f'target {c.target_o[om]:.2f} profile {prof}')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('check', choices=('pitch', 'dives', 'static', 'bands', 'swim', 'shape', 'near'))
    ap.add_argument('args', nargs='*', help='near: NET X,Y [R]')
    ap.add_argument('--board', required=True)
    ap.add_argument('--nets', required=True, help='N1,N2,.. or @FILE')
    ap.add_argument('--dest', default='DU1')
    ap.add_argument('--only', default='', help='bands, swim, shape: these lanes only')
    ap.add_argument('--png', default='')
    ap.add_argument('--all', action='store_true', help='dives: print every site, not only the failing ones')
    a = ap.parse_args(argv)
    ctx, corridors, _logs = plan(a.board, read_nets(a.nets), a.dest)
    if a.check == 'pitch':
        check_pitch(ctx, corridors, a.png or None)
    elif a.check == 'dives':
        check_dives(ctx, corridors, a.all)
    elif a.check == 'static':
        check_static(ctx, corridors, set(a.only.split(',')) - {''} or None)
    elif a.check == 'bands':
        check_bands(ctx, corridors, set(a.only.split(',')) - {''} or None, a.png or None)
    elif a.check == 'shape':
        check_shape(ctx, corridors, set(a.only.split(',')) - {''} or None)
    elif a.check == 'swim':
        check_swim(ctx, corridors, set(a.only.split(',')) - {''} or None)
    else:
        if len(a.args) < 2:
            ap.error('near: NET X,Y [R]')
        show_near(ctx, corridors, a.args[0], tuple(map(float, a.args[1].split(','))),
                  float(a.args[2]) if len(a.args) > 2 else None)


if __name__ == '__main__':
    main()

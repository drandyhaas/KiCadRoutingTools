"""whole_passages.py -- the PASSAGES the fixed copper leaves a bus's lanes, layer by layer, across the whole route's
frames (whole_frame: the trunk and each ring), and the check of a plan against them.

What it is for: the geometry lays a plan only when, on every routing layer, each passage between two fixed things --
a part's pads on that layer, a hole's copper on every layer, another net's via or track, an array's pad box grown to
its stubs, the board's edge -- carries no more of the plan's lanes on that layer than fit there at the lane pitch, and
each change (a via, on every layer at once) stands where a via fits. The solve reads the same census (whole_solve's
PASSAGES rows) and the geometry the same sides (whole_geo's static sides), so the three agree on what is in the way.

A COLUMN is a line square to a frame's spine at s; on a layer its free RUNS are the stretches of offset at least a
lane's bar (half its track, the clearance, half a grid step) off every obstacle on that layer and inside the board.
A lane stands in the run its REFERENCE route (references: the frame's taut path, a ring lane's carried to its handoff
and round its ring) crosses the column in -- or the nearest run -- and passes the obstacles on the sides that run
lies; a run's lanes on its layer fit when their pitches sum within its room.

  python3 whole_passages.py WORK ROUND_DIR --sizes ... --dest D --layers ... --plan SOLVE.json [--geo GEO.json]

reports each run a plan overfills (by the references, or with --geo by where that geometry laid each lane) and each
change standing where no via fits."""
KRT_TOOL = {'scope': [], 'kind': 'actor'}   # a research tool (awx), catalogued, shown at no door

import collections
import math

import numpy as np


class Census:
    """the fixed copper a bus's lanes go round, per routing layer (its index in LAYN), read across a frame's columns:
    the arrays' boxes (the source's on every layer; the destination's where whole_solve's dest_boxes puts them), each
    island of other parts' pads (whole_ctx.part_islands: pads no lane passes between, one box per island and layer set
    -- a header's row of holes one wall on every layer), other nets' vias (every layer) and tracks (their own) outside
    the arrays' boxes on their layers, and the board's edge. A GROUP's part stands on its pads' layers alone, as the
    ends model and the solve have it (whole_frame.Group.open; whole_solve's dest_boxes): closed on every layer here, the
    2-D geometry pushed every lane the plan laid under the part -- 2 to 3 mm deep under a PHY, under a connector -- out
    onto the box's edge, into the strip between the two parts, where they crossed and stood short of a pitch"""

    def __init__(self, ctx, Fr, LAYN, trk, clr, via, pitch, bus_nets=(), dest_boxes=None):
        import whole_ctx
        self.ctx, self.Fr, self.LAYN, self.NL = ctx, Fr, list(LAYN), len(LAYN)
        g2 = ctx.cfg.grid_step / 2
        self.BAR = trk / 2 + clr + g2                    # a lane's centreline off copper (whole_geo's LANE_ST)
        self.PMIN = max(pitch, trk + clr + 2 * g2)       # two lanes' centrelines apart (whole_geo's P_MIN)
        self.VBAR = via / 2 + clr + g2                   # a via's centre off copper
        e_ = float(getattr(ctx.cfg, 'board_edge_clearance', 0.0) or 0.0) or clr
        self.EDGE, self.VEDGE = e_ + trk / 2, e_ + via / 2
        self.BB = ctx.pcb.board_info.board_bounds
        ALL = frozenset(range(self.NL))
        LI = {L: i for i, L in enumerate(self.LAYN)}
        obs = []                                          # (x0, y0, x1, y1, layer indices, label)
        obs.append(tuple(Fr.SB) + (ALL, 'src ' + str(Fr.src)))
        grp = getattr(Fr, 'group', None)
        if dest_boxes is None and grp is not None:
            def dest_boxes(i):
                return [B_ for r_, B_ in Fr.DBs.items() if self.LAYN[i] not in grp.open[r_]]
        on_ = []                                          # each layer's boxes: the source's, the destination's there
        for i in range(self.NL):
            on_.append([tuple(Fr.SB)])
            for r_, B_ in (Fr.DBs.items() if dest_boxes is None else
                           [(r_, B_) for r_, B_ in Fr.DBs.items() if B_ in dest_boxes(i)]):
                obs.append(tuple(B_) + (frozenset([i]), 'dest ' + r_))
                on_[i].append(tuple(B_))
        isl = {}
        import pairs as _pairs
        import board_rules
        for (ref_, i_), lab_ in whole_ctx.part_islands(ctx, skip=(Fr.src, *Fr.dests)).items():
            p_ = ctx.pcb.footprints[ref_].pads[i_]
            if p_.pad_type == 'np_thru_hole':
                # (a hole with no copper grown by what the router keeps a track off its wall past the clearance --
                # board_rules.hole_track_clear -- as the geometry grows it)
                hx_ = hy_ = (p_.drill or 0) / 2 + board_rules.hole_track_clear(ctx.pcb, clr) - clr
                ls_ = ALL
            else:
                # (a tilted pad by the box round its copper turned, as the geometry boxes it: pairs.pad_extent)
                hx_, hy_ = _pairs.pad_extent(p_.size_x / 2, p_.size_y / 2, _pairs.pad_tilt(p_),
                                             _pairs.pad_corner_radius(p_))
                ls_ = ALL if ((p_.drill or 0) > 0 or '*.Cu' in p_.layers) else \
                    frozenset(LI[L_] for L_ in p_.layers if L_ in LI)
            if not ls_:
                continue
            b_ = isl.setdefault((lab_, ls_), [math.inf, math.inf, -math.inf, -math.inf])
            b_[0], b_[1] = min(b_[0], p_.global_x - hx_), min(b_[1], p_.global_y - hy_)
            b_[2], b_[3] = max(b_[2], p_.global_x + hx_), max(b_[3], p_.global_y + hy_)
        for (lab_, ls_), b_ in sorted(isl.items(), key=lambda kv: (kv[0][0], sorted(kv[0][1]))):
            obs.append(tuple(b_) + (ls_, lab_))
        self.islands = isl
        # (what a box covers is left out: a via inside one that stands on EVERY layer, a track inside one on its own --
        # a group's part open on a layer leaves the vias and tracks under it there in the way)
        inbox = lambda x, y, bs: any(B_[0] <= x <= B_[2] and B_[1] <= y <= B_[3] for B_ in bs)   # noqa: E731
        every = [B_ for B_ in on_[0] if all(B_ in on_[i] for i in range(self.NL))]
        bus = set(bus_nets)
        for v in ctx.base_vias:
            if v.net_id in bus or inbox(v.x, v.y, every):
                continue
            r_ = (v.size or via) / 2
            obs.append((v.x - r_, v.y - r_, v.x + r_, v.y + r_, ALL, 'via ' + whole_ctx.copper_name(ctx, v)))
        for s_ in ctx.base_segments:
            if s_.net_id in bus or s_.layer not in LI:
                continue
            ln_ = math.hypot(s_.end_x - s_.start_x, s_.end_y - s_.start_y)
            k_ = max(1, int(math.ceil(ln_ / 0.2)))
            h_ = s_.width / 2
            for j_ in range(k_):
                ax_ = s_.start_x + (s_.end_x - s_.start_x) * j_ / k_
                ay_ = s_.start_y + (s_.end_y - s_.start_y) * j_ / k_
                bx_ = s_.start_x + (s_.end_x - s_.start_x) * (j_ + 1) / k_
                by_ = s_.start_y + (s_.end_y - s_.start_y) * (j_ + 1) / k_
                if inbox(ax_, ay_, on_[LI[s_.layer]]) and inbox(bx_, by_, on_[LI[s_.layer]]):
                    continue
                obs.append((min(ax_, bx_) - h_, min(ay_, by_) - h_, max(ax_, bx_) + h_, max(ay_, by_) + h_,
                            frozenset([LI[s_.layer]]), 'track ' + whole_ctx.copper_name(ctx, s_)))
        self.obs = obs
        A = np.array([o[:4] for o in obs], dtype=float)
        self._box = A
        self._cx, self._cy = (A[:, 0] + A[:, 2]) / 2, (A[:, 1] + A[:, 3]) / 2
        self._hd = np.hypot((A[:, 2] - A[:, 0]) / 2, (A[:, 3] - A[:, 1]) / 2)
        self._on = np.array([[i in o[4] for i in range(self.NL)] for o in obs], dtype=bool)
        self.OS = np.arange(-15.0, 15.0 + 1e-9, self.PMIN / 8)
        self._cache = {}
        dc_ = Fr.dpads
        self.DCEN = (sum(x_ for x_, _y in dc_) / len(dc_), sum(y_ for _x, y_ in dc_) / len(dc_))

    def _line(self, sp, s):
        k_ = sp.seg_of(s)
        t_ = s - sp.S[k_]
        px_, py_ = float(sp.P[k_, 0] + t_ * sp.d[k_, 0]), float(sp.P[k_, 1] + t_ * sp.d[k_, 1])
        return px_, py_, float(sp.nrm[k_, 0]), float(sp.nrm[k_, 1])

    def column(self, fk, sp, s, ring=False):
        """{layer: [(lo, hi, label below, label above)]}: the free runs of frame `fk`'s column at s, each layer's, as
        the offsets lanes' centrelines may take (a ring's: only on its spine's side of the destination); and, under
        'via', the runs where a via's centre fits on every layer at once"""
        key = (fk, round(s, 4))
        if key in self._cache:
            return self._cache[key]
        px_, py_, nx_, ny_ = self._line(sp, s)
        OS = self.OS
        X_, Y_ = px_ + OS * nx_, py_ + OS * ny_
        bx0, by0, bx1, by1 = self.BB
        near_ = np.abs((self._cx - px_) * ny_ - (self._cy - py_) * nx_) <= self._hd + self.VBAR + 1e-6
        idx_ = np.flatnonzero(near_)
        oc_ = float(sp.project_pt(self.DCEN)[1]) if ring else None
        out = {}
        vok_ = ((X_ >= bx0 + self.VEDGE) & (X_ <= bx1 - self.VEDGE) & (Y_ >= by0 + self.VEDGE) & (Y_ <= by1 - self.VEDGE))
        for L in range(self.NL):
            ok_ = ((X_ >= bx0 + self.EDGE) & (X_ <= bx1 - self.EDGE) & (Y_ >= by0 + self.EDGE) & (Y_ <= by1 - self.EDGE))
            who_ = np.full(len(OS), -1, dtype=int)
            for i_ in idx_:
                if not self._on[i_, L]:
                    continue
                b_ = self._box[i_]
                hit_ = (X_ > b_[0] - self.BAR) & (X_ < b_[2] + self.BAR) & (Y_ > b_[1] - self.BAR) & (Y_ < b_[3] + self.BAR)
                ok_ &= ~hit_
                who_[hit_] = i_
                vok_ &= ~((X_ > b_[0] - self.VBAR) & (X_ < b_[2] + self.VBAR) & (Y_ > b_[1] - self.VBAR)
                          & (Y_ < b_[3] + self.VBAR))
            out[L] = self._runs(ok_, who_, oc_)
        out['via'] = self._runs(vok_, np.full(len(OS), -1, dtype=int), oc_)
        self._cache[key] = out
        return out

    def _runs(self, ok_, who_, oc_):
        OS = self.OS
        lab = lambda i: 'edge' if i < 0 or i >= len(OS) or who_[i] < 0 else self.obs[who_[i]][5]
        idx_ = np.flatnonzero(ok_)
        runs = []
        if idx_.size:
            cut_ = np.flatnonzero(np.diff(idx_) > 1)
            for i_, j_ in zip(np.concatenate((idx_[:1], idx_[cut_ + 1])).tolist(),
                              np.concatenate((idx_[cut_], idx_[-1:])).tolist()):
                a_, b_ = float(OS[i_]), float(OS[j_])
                if oc_ is not None and not ((a_ + b_) / 2 - oc_) * (0.0 - oc_) > 0:
                    continue                                # (a ring's room is on its spine's side of the part)
                runs.append((a_, b_, lab(i_ - 1), lab(j_ + 1)))
        return runs

    def room(self, run):
        """a run's room as lane pitches summed: centrelines a pitch apart from its low end to its high"""
        return run[1] - run[0] + self.PMIN

    @staticmethod
    def run_of(runs, o):
        """the index of the run in `runs` offset o stands in, else the nearest; None with no run"""
        best = None
        for i, r in enumerate(runs):
            d = 0.0 if r[0] <= o <= r[1] else min(abs(o - r[0]), abs(o - r[1]))
            if best is None or d < best[0]:
                best = (d, i)
        return None if best is None else best[1]

    def via_fits(self, x, y):
        """a via's centre at (x, y) clears every obstacle on every layer and the board's edge"""
        bx0, by0, bx1, by1 = self.BB
        if not (bx0 + self.VEDGE <= x <= bx1 - self.VEDGE and by0 + self.VEDGE <= y <= by1 - self.VEDGE):
            return False
        b_ = self._box
        return not bool(np.any((x > b_[:, 0] - self.VBAR) & (x < b_[:, 2] + self.VBAR)
                               & (y > b_[:, 1] - self.VBAR) & (y < b_[:, 3] + self.VBAR)))


def route_coords(Fr, G):
    """(Hn, rs, entry, end, tend, u_ring) as whole_solve reads a frame: each ring lane's ring start, each ring's route
    origin (a geometry column ahead of every handoff), each lane's entry, landing and where it leaves the trunk, and a
    ring lane's route u at ring coordinate s_b"""
    spine, bname, ring_of, Hk = Fr.spine, Fr.cls, Fr.rings, Fr.Hk
    Hn = {n: Hk[bname[n]] for n in bname}
    GG = 2 * G
    rs = {}
    for k_, b_ in ring_of.items():
        sbs = [b_.project_pt(spine.xy(Hk[k_], Fr.o_h[n]))[0] for n in Fr.M if bname.get(n) == k_]
        rs[k_] = math.ceil(max(sbs) / GG - 1e-9) * GG
    entry = {n: Fr.st[n][0] for n in Fr.M}

    def u_ring(n, s_b):
        return Hn[n] + (s_b - rs[bname[n]])
    end = {}
    for n in Fr.M:
        if n in bname:
            end[n] = u_ring(n, ring_of[bname[n]].project_pt(Fr.land[n])[0])
        else:
            end[n] = spine.project_pt(Fr.land[n])[0]
    tend = {n: (Hn[n] if n in bname else end[n]) for n in Fr.M}
    return Hn, rs, entry, end, tend, u_ring


def references(ctx, Fr, G, rs, end, pitch, trk, clr, ring_stack=False):
    """{lane: [(u, x, y)]}: each lane's REFERENCE route on the board in its route coordinate, every G of u -- the
    frame's mid path on the trunk; a ring lane's ramped at 45 degrees onto its handoff offset, then out to its ring's
    standoff (a pair: its landing's offset), held there and in to its landing (whole_geo's references). `ring_stack`
    (whole_solve's probe SOLVE_RING_STACK_REF): along the ring at its place in the ring's stack instead"""
    import pairs as _pairs
    prs = getattr(ctx, 'pairs', {}) or {}
    M, spine, bname, ring_of, Hk = list(Fr.M), Fr.spine, Fr.cls, Fr.rings, Fr.Hk
    Hn = {n: Hk[bname[n]] for n in bname}
    u_ring = lambda n, s_b: Hn[n] + (s_b - rs[bname[n]])
    dc_ = Fr.dpads
    DCEN_ = (sum(x_ for x_, _y in dc_) / len(dc_), sum(y_ for _x, y_ in dc_) / len(dc_))
    deps_ = [abs(ring_of[bname[n]].project_pt(Fr.land[n])[1]
                 - ring_of[bname[n]].project_pt(_pairs.mid(*ctx.pair_ends[n][1]))[1])
             for n in M if n in bname and n in prs]
    STAND_ = max(deps_) if deps_ else _pairs.turn_straight_steps(ctx.cfg) * ctx.cfg.grid_step + trk + clr

    def ref_route(n):
        ms_ = np.array([p_[0] for p_ in Fr.mid[n]])
        mo_ = np.array([p_[1] for p_ in Fr.mid[n]])
        r_ = lambda s_: float(np.interp(s_, ms_, mo_))
        s0_ = Fr.st[n][0]
        pts_ = []
        if n not in bname:
            return [(float(s_),) + spine.xy(float(s_), r_(s_)) for s_ in np.arange(s0_, end[n] + 1e-9, G)]
        k_ = bname[n]
        hk_, oh_ = Hk[k_], Fr.o_h[n]
        a_ = max(hk_ - abs(oh_ - r_(hk_)), s0_)
        for s_ in np.arange(s0_, hk_ + 1e-9, G):
            o_ = r_(s_) if s_ <= a_ else r_(a_) + (oh_ - r_(a_)) * (s_ - a_) / max(hk_ - a_, 1e-9)
            pts_.append((float(s_),) + spine.xy(float(s_), o_))
        sp_ = ring_of[k_]
        sb0_, ob0_ = (float(v_) for v_ in sp_.project_pt(spine.xy(hk_, oh_)))
        sb1_, ob1_ = (float(v_) for v_ in sp_.project_pt(Fr.land[n]))
        out_ = 1.0 if ob1_ >= sp_.project_pt(DCEN_)[1] else -1.0
        orun_ = ob1_ if n in prs else ob1_ + out_ * STAND_
        srun_ = sb1_ - (STAND_ + trk + clr)
        if ring_stack:
            mem_ = [m_ for m_ in M if bname.get(m_) == k_]
            sbe_ = {m_: end[m_] - Hn[m_] + rs[k_] for m_ in mem_}
            inner_ = [(sbe_[m_], pitch + (_pairs.pitch(trk) if m_ in prs else 0.0))
                      for m_ in mem_ if m_ != n and sbe_[m_] < sbe_[n]]
            o_st = lambda sb_: ob0_ - out_ * sum(w_ for e_, w_ in inner_ if e_ <= sb_)
            for sb_ in np.arange(sb0_, sb1_ + 1e-9, G):
                o_ = (o_st(sb_) if sb_ <= srun_ else
                      float(np.interp(sb_, [srun_, sb1_], [o_st(srun_), ob1_])))
                pts_.append((u_ring(n, float(sb_)),) + sp_.xy(float(sb_), o_))
            return pts_
        sout_ = sb0_ + abs(orun_ - ob0_)
        rp_ = ([(sb0_, ob0_), (sout_, orun_), (srun_, orun_), (sb1_, ob1_)] if sout_ < srun_ else
               [(sb0_, ob0_), (srun_, orun_), (sb1_, ob1_)] if srun_ > sb0_ + 1e-9 else [(sb0_, ob0_), (sb1_, ob1_)])
        for sb_ in np.arange(sb0_, sb1_ + 1e-9, G):
            o_ = float(np.interp(sb_, [q_[0] for q_ in rp_], [q_[1] for q_ in rp_]))
            pts_.append((u_ring(n, float(sb_)),) + sp_.xy(float(sb_), o_))
        return pts_
    return {n: np.array(ref_route(n)) for n in M}


def ref_at(R, u):
    """(x, y) of a reference route R (rows u, x, y) at route u, interpolated"""
    return float(np.interp(u, R[:, 0], R[:, 1])), float(np.interp(u, R[:, 0], R[:, 2]))


def frames(Fr):
    """[(key, spine, ring?)]: the trunk and each ring"""
    return [('T', Fr.spine, False)] + [(k_, sp_, True) for k_, sp_ in sorted(Fr.rings.items())]


def present(Fr, fk, s, entry, tend, end, Hn, u_ring):
    """{lane: its route u} at frame fk's coordinate s: a trunk lane between its entry and where it leaves the trunk, a
    ring lane between its handoff and its landing"""
    if fk == 'T':
        return {n: s for n in Fr.M if entry[n] < s < tend[n]}
    return {n: u_ring(n, s) for n in Fr.M if Fr.cls.get(n) == fk and Hn[n] < u_ring(n, s) < end[n]}


def survey(ctx, Fr, LAYN, plan, geo=None, step=0.25):
    """the census of the bench and a plan's lanes in it: SimpleNamespace(C, REF, records, vias, ...) -- `records` every
    run of every column every `step` along each frame where lanes are, (frame, s, layer name, run, its lanes on that
    layer, their pitches summed, its room, the run's two ends and its middle in frame xy); a lane stands where its
    reference crosses the column (a solve's plan, its layers there), or with `geo` (a geometry's lanes) where that
    geometry laid it. `vias` [(lane, u or None, x, y, why)] each change -- at its reference, or the geometry's via --
    with no room: 'fixed' (no place in its run where a via clears the fixed copper on every layer), 'crowd L' (the via's
    room and the lanes beside it on layer L overfill that layer's run there)"""
    import types
    import braid as bd
    import pairs as _pairs
    prs = getattr(ctx, 'pairs', {}) or {}
    TRK, CLR, VIA, PITCH = bd.TRACK, bd.CLEAR, bd.VIA_SIZE, bd.LANE_MIN
    G = PITCH / 5
    bus = {ctx.byname[n][0] for n in Fr.M} | {ctx.byname[leg][0] for n in Fr.M for leg in prs.get(n, ())
                                             if leg in ctx.byname}
    C = Census(ctx, Fr, LAYN, TRK, CLR, VIA, PITCH, bus_nets=bus)
    Hn, rs, entry, end, tend, u_ring = route_coords(Fr, G)
    if plan is not None and 'rs' in plan:
        rs = {k_: float(v_) for k_, v_ in plan['rs'].items()}
        end = {n: float(plan['end'].get(n, end[n])) for n in Fr.M}
        tend = {n: (Hn[n] if n in Fr.cls else end[n]) for n in Fr.M}
        u_ring = lambda n, s_b: Hn[n] + (s_b - rs[Fr.cls[n]])
    REF = references(ctx, Fr, G, rs, end, PITCH, TRK, CLR)
    wid = {n: C.PMIN + (_pairs.pitch(TRK) if n in prs else 0.0) for n in Fr.M}
    # a change's room across the lanes (whole_geo's VS and VX: the router's ring round the via, a pair's two barrels)
    VIA_R = max(bd.VIA_NEED, _pairs.via_ring(ctx.cfg) + ctx.cfg.grid_step)
    PP = _pairs.pitch(TRK)
    vw = {n: 2 * (VIA_R + (PP / 2 + _pairs.dive_offset(ctx.cfg, PP / 2) if n in prs else 0.0)) for n in Fr.M}
    chg = {n: sorted((plan or {}).get('changes', {}).get(n, [])) for n in Fr.M}
    # (no plan: no lane stands on any layer -- the runs' rooms alone)
    lay = (lambda n, u: plan['layers'][n][sum(1 for c_ in chg[n] if c_ <= u)]) if plan is not None else \
        (lambda n, u: None)
    gpos = {n: v_['pieces'] for n, v_ in geo['lanes'].items()} if geo is not None else None
    FRS = {fk: (sp, ring) for fk, sp, ring in frames(Fr)}

    def place(fk, sp, s, pres, L, runs):
        """{run index: [lanes]} of the lanes present on layer L at frame fk's column s"""
        dem = collections.defaultdict(list)
        if gpos is None:
            for n, u_ in pres.items():
                if lay(n, u_) != LAYN[L]:
                    continue
                j_ = Census.run_of(runs, float(sp.project_pt(ref_at(REF[n], u_))[1]))
                if j_ is not None:
                    dem[j_].append(n)
            return dem
        for n in pres:
            for x0, y0, x1, y1, L_ in gpos.get(n, ()):
                if L_ != LAYN[L]:
                    continue
                s0_, o0_ = sp.project_pt((x0, y0))
                s1_, o1_ = sp.project_pt((x1, y1))
                if (s0_ - s) * (s1_ - s) > 0 or abs(s1_ - s0_) < 1e-9:
                    continue
                o_ = o0_ + (o1_ - o0_) * (s - s0_) / (s1_ - s0_)
                if abs(o_) > 15:
                    continue
                j_ = Census.run_of(runs, o_)
                if j_ is not None:
                    dem[j_].append(n)
                break
        return dem
    # every via: (lane, its frame, s there, x, y) -- a change at its reference, or the geometry's
    def frame_at(n, u):
        if n in Fr.cls and u > Hn[n]:
            return Fr.cls[n], u - Hn[n] + rs[Fr.cls[n]]
        return 'T', u
    VS = []
    if geo is None:
        for n in Fr.M:
            for cu in chg[n]:
                fk, s = frame_at(n, cu)
                VS.append((n, fk, s) + ref_at(REF[n], cu) + (cu,))
    else:
        for (n, x_, y_) in geo.get('vias', []):
            s_t = float(Fr.spine.project_pt((x_, y_))[0])
            if n in Fr.cls and s_t > Hn[n]:
                fk = Fr.cls[n]
                s = float(FRS[fk][0].project_pt((x_, y_))[0])
            else:
                fk, s = 'T', s_t
            VS.append((n, fk, s, x_, y_, None))
    VREACH = VIA_R                                  # a via stands in the columns within its room along the frame
    records = []
    for fk, sp, ring in frames(Fr):
        s = 0.0
        while s <= sp.L:
            pres = present(Fr, fk, s, entry, tend, end, Hn, u_ring)
            if pres:
                col = C.column(fk, sp, s, ring)
                vnear = [v_ for v_ in VS if v_[1] == fk and abs(v_[2] - s) <= VREACH]
                for L in range(len(LAYN)):
                    runs = col[L]
                    dem = place(fk, sp, s, pres, L, runs)
                    vd = collections.defaultdict(list)          # a run's vias: each takes its room on every layer
                    for v_ in vnear:
                        j_ = Census.run_of(runs, float(sp.project_pt((v_[3], v_[4]))[1]))
                        if j_ is not None:
                            vd[j_].append(v_[0])
                    for j_, r_ in enumerate(runs):
                        ls_ = sorted(dem.get(j_, ()))
                        vs_ = sorted(vd.get(j_, ()))
                        need_ = sum(wid[n] for n in ls_) + sum(vw[m_] - (wid[m_] if m_ in ls_ else 0.0) for m_ in vs_)
                        records.append((fk, s, LAYN[L], r_, ls_, need_, C.room(r_),
                                        sp.xy(s, r_[0]), sp.xy(s, r_[1]), sp.xy(s, (r_[0] + r_[1]) / 2), vs_))
            s += step

    def via_room(n, fk, s, x_, y_):
        """why a via of lane n at (x_, y_), frame fk's column s, has no room -- None when it has"""
        sp, ring = FRS[fk]
        col = C.column(fk, sp, s, ring)
        o_ = float(sp.project_pt((x_, y_))[1])
        pres = present(Fr, fk, s, entry, tend, end, Hn, u_ring)
        pres.pop(n, None)
        # (the lane's own run on each layer -- the via's place must lie in every one -- and a via's place in them)
        span = [-math.inf, math.inf]
        for L in range(len(LAYN)):
            j_ = Census.run_of(col[L], o_)
            if j_ is None:
                return 'fixed'
            span = [max(span[0], col[L][j_][0]), min(span[1], col[L][j_][1])]
        if not any(v_[0] <= span[1] and v_[1] >= span[0] for v_ in col['via']):
            return 'fixed'
        for L in range(len(LAYN)):
            runs = col[L]
            j_ = Census.run_of(runs, o_)
            ls_ = place(fk, sp, s, pres, L, runs).get(j_, [])
            if sum(wid[m_] for m_ in ls_) + vw[n] > C.room(runs[j_]) + 1e-9:
                return f'crowd {LAYN[L]} ({len(ls_)} lanes + via, room {C.room(runs[j_]) / C.PMIN:.1f})'
        return None

    bad = []
    if geo is None:
        for n in Fr.M:
            for cu in chg[n]:
                x_, y_ = ref_at(REF[n], cu)
                fk, s = frame_at(n, cu)
                why = via_room(n, fk, s, x_, y_)
                if why:
                    bad.append((n, round(cu, 2), x_, y_, why))
    else:
        for (n, x_, y_) in geo.get('vias', []):
            s_t = float(Fr.spine.project_pt((x_, y_))[0])
            if n in Fr.cls and s_t > Hn[n]:
                fk = Fr.cls[n]
                s = float(FRS[fk][0].project_pt((x_, y_))[0])
            else:
                fk, s = 'T', s_t
            why = 'fixed' if not C.via_fits(x_, y_) else via_room(n, fk, s, x_, y_)
            if why:
                bad.append((n, None, x_, y_, why))
    return types.SimpleNamespace(C=C, REF=REF, records=records, vias=bad, wid=wid, vw=vw, Hn=Hn, rs=rs, end=end,
                                 frame_at=frame_at)


def main():
    import argparse
    import contextlib
    import io
    import json
    import os
    import sys
    HERE = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, HERE)
    sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
    import replay_stage
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    replay_stage.add_round_args(ap)
    ap.add_argument('--plan', required=True, help='a whole_solve plan (its layers and changes)')
    ap.add_argument('--geo', default=None, help="a geometry (whole_geo's): where it laid each lane, in place of "
                                                "the references")
    ap.add_argument('--step', type=float, default=0.25, help='the columns apart along each frame, mm')
    ap.add_argument('--bench', default=None, help='the bench (default ROUND_DIR/fo.kicad_pcb)')
    ap.add_argument('--show', default=None, metavar='X,Y', help='every run of the columns nearest this board point')
    a = ap.parse_args()
    env = replay_stage.round_env(a.work, a.round_dir, a.sizes, a.dest, a.layers, a.set)
    if a.bench:
        env['BENCH'] = os.path.abspath(a.bench)
    plan = json.load(open(os.path.abspath(a.plan)))
    geo = json.load(open(os.path.abspath(a.geo))) if a.geo else None
    os.chdir(HERE)
    os.environ.update(env)
    import awx_settings
    with awx_settings.given({**awx_settings.environ(), **env}):
        import braid as bd
        import pairs as _pairs
        import route_layers
        import whole_ctx
        import whole_frame as wf
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            ctx, _cs = whole_ctx.plan()
            Fr = wf.build(ctx, env['DEST'])
        LAYN = route_layers.layers()
        side = json.load(open(os.path.splitext(env['BENCH'])[0] + '.plan.json'))
        CY = None
        if int(side.get('chi', 1)) < 0:
            from bga_fanout.flip_frame import mirror_axis
            CY = mirror_axis(ctx.pcb)
        board = lambda x, y: (round(x, 2), round(2 * CY - y if CY is not None else y, 2))
        bl = lambda L: ({'F.Cu': 'B.Cu', 'B.Cu': 'F.Cu'}.get(L, L) if CY is not None else L)   # (the board's name)
        S = survey(ctx, Fr, LAYN, plan, geo, a.step)
        C = S.C
        print(f'census: {len(C.obs)} obstacles ({len(C.islands)} island-layer boxes), BAR {C.BAR:.4f} PMIN '
              f'{C.PMIN:.4f} VBAR {C.VBAR:.4f}; positions from ' + ('the geometry' if geo else 'the references')
              + (' [board coordinates and layer names: the frame mirrored back]' if CY is not None else ''))
        over = [(fk, s, L, r_, need_, room_, ls_ + ['via:' + v_ for v_ in vs_], board(*mid_))
                for fk, s, L, r_, ls_, need_, room_, _a, _b, mid_, vs_ in S.records if need_ > room_ + 1e-9]
        # consecutive columns with one run's lanes and bounds, as one passage
        rows = []
        for o_ in over:
            fk, s, L, r_, need_, room_, ls_, xy_ = o_
            if rows and rows[-1][0] == fk and rows[-1][2] == L and rows[-1][3] == (r_[2], r_[3]) \
                    and rows[-1][6] == ls_ and s - rows[-1][1][1] <= a.step + 1e-6:
                rows[-1][1][1] = s
                rows[-1][4] = max(rows[-1][4], need_)
                rows[-1][5] = min(rows[-1][5], room_)
                continue
            rows.append([fk, [s, s], L, (r_[2], r_[3]), need_, room_, ls_, xy_])
        print(f'PASSAGES over: {len(rows)} (frame, s from..to, layer, between, lanes there / room in lanes, at)')
        for fk, (s0, s1), L, (la, lb), need_, room_, ls_, xy_ in rows:
            print(f'  {fk} s {s0:6.2f}..{s1:6.2f} {bl(L):6s} {la[:22]:>22s} / {lb[:22]:22s} '
                  f'{len(ls_)} lanes need {need_:.3f} room {room_:.3f} ({room_ / C.PMIN:.1f} lanes) at {xy_}: '
                  + ' '.join(n.replace('RAM_', '') for n in ls_))
        # each change where no via fits
        bad = [(n, cu, board(x_, y_), why) for n, cu, x_, y_, why in S.vias]
        if a.show:
            qx_, qy_ = (float(v_) for v_ in a.show.split(','))
            qy_ = 2 * CY - qy_ if CY is not None else qy_
            for fk_ in sorted({r_[0] for r_ in S.records}):
                rr_ = [r_ for r_ in S.records if r_[0] == fk_]
                near_ = min(rr_, key=lambda r_: math.hypot(r_[9][0] - qx_, r_[9][1] - qy_) if
                            min(r_[7][0], r_[8][0]) - 0.3 <= qx_ <= max(r_[7][0], r_[8][0]) + 0.3 and
                            min(r_[7][1], r_[8][1]) - 0.3 <= qy_ <= max(r_[7][1], r_[8][1]) + 0.3 else 1e9)
                sq_ = near_[1]
                print(f'COLUMN {fk_} s {sq_:.2f}:')
                for r_ in rr_:
                    if abs(r_[1] - sq_) < 1e-9:
                        print(f'   {bl(r_[2]):6s} {board(*r_[7])}..{board(*r_[8])} {r_[3][2][:18]:>18s} / {r_[3][3][:18]:18s}'
                              f' room {r_[6] / C.PMIN:4.1f} need {r_[5] / C.PMIN:4.1f} lanes {len(r_[4])}: '
                              + ' '.join(n.replace('RAM_', '') for n in r_[4])
                              + (' vias: ' + ' '.join(n.replace('RAM_', '') for n in r_[10]) if r_[10] else ''))
        print(f'CHANGES with no via room ({"the geometry\'s vias" if geo else "at the reference"}): {len(bad)}')
        for b_ in bad:
            print('  ', b_)


if __name__ == '__main__':
    main()

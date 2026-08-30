#!/usr/bin/env python3
"""plan_order.py -- the braid's ORDER MODEL, for the plan.

The plan's crossing floor was read off a transverse projection of the
launch and exit points (select_moves.Corridor). The braid does not lay
lanes in that order: a flank tooth joins from the side and its join
block puts it OUTERMOST, in join order; a side exit takes a slot in an
exit block -- ports first, by exit s, then the joined in reverse join
order. The projection put K28's SWE and SCAS (joiners) next to SCKE0 in
the middle of the bundle, so a left-face exit among the data lanes
looked cheap; the braid then had them cross ten and twelve lanes each
(22 of 71 swaps), which forced the schedule's gate off and cost ~30
vias over the human's 46. And the projection prices a diver at two
vias whether it passes one lane or twelve, while the lanes it passes
are what the corridor is short of (columns).

This model scores a candidate destination choice exactly as the braid
will lay it: the spine as braid.Corridor.build_spine builds it, the
launch order from its classification and offsets rules (fixed once
the teeth are), the target order from the candidate exits under its
exit rules, and then schedule.Schedule for the floor (2 x divers) AND
the columns, against the corridor's column capacity. It subclasses
select_moves.Corridor so the plan's LIS refinement, floor, and layer
alignment run on it unchanged; only the ORDER changes -- the leg
crossing counts (bus_sides, the greedy pass) keep the drawn legs.

Ranks are scalar keys (BIG = 1000 beyond any offset):
  launch: head-on  o_t;  joiner on side sg  sg*(BIG + S_max - s_t)
          (first joiner outermost)
  exit:   head-on  o_e;  port on side sg    sg*(BIG + s_e)
          joined on side sg  sg*(2 BIG + S_max - s_t)
          (ports by exit s, innermost first; joined beyond them in
          join order)
"""
from __future__ import annotations

import math
import os
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

import braid as te
import corridor as cr
import select_moves as sm
import topo_strings as ts
from schedule import Schedule, lis_keep, lis_keep_weighted

Pt = Tuple[float, float]
BIG = 1000.0
_DIRV = {'right': (1, 0), 'left': (-1, 0), 'up': (0, -1), 'down': (0, 1)}


def _unit(v):
    h = math.hypot(v[0], v[1]) or 1.0
    return (v[0] / h, v[1] / h)


class BraidOrder(sm.Corridor):
    def __init__(self, box, launch: Dict[str, Pt], names: Sequence[str],
                 ends, paths, tooth_layer: Dict[str, str], tooth_dirs: Dict[str, Pt],
                 obs_for, base_obs, src_centre: Pt, dst_centre: Pt,
                 pad_obs=None, pad: float = 0.3, cache=None, log=None):
        super().__init__(box, launch, pad=pad, cache=cache)
        self.names = list(names)
        self.ends = ends
        self.paths = paths
        self.tooth_layer = tooth_layer
        self.obs_for = obs_for
        self.pad_obs = pad_obs
        self.log = log
        self.dst_centre = dst_centre
        self.src_centre = src_centre
        n = len(self.names)
        H = te.LPITCH * (n - 1) / 2 + te.LPITCH
        teeth = [launch[nm] for nm in self.names]
        stubs = [ends[nm][1] for nm in self.names]
        # the arrival flow at a ball not yet fanned out: the taut path's
        # own direction as it reaches the ball
        sdirs = []
        for nm in self.names:
            p = paths[nm]
            q = cr.point_before_end(p, 1.5) if cr.polyline_len(p) > 3.0 else p[0]
            sdirs.append(_unit((p[-1][0] - q[0], p[-1][1] - q[1])))
        spine = cr.build_spine([paths[nm] for nm in self.names], base_obs, H,
                               teeth=teeth, stubs=stubs,
                               tooth_dirs=[tooth_dirs[nm] for nm in self.names],
                               stub_dirs=sdirs, log=log)
        P0, d0 = spine.P[0], spine.d[0]
        Pn, dn = spine.P[-1], spine.d[-1]
        back = max([0.3] + [-((t[0] - P0[0]) * d0[0] + (t[1] - P0[1]) * d0[1]) + 0.3
                            for t in teeth])
        fwd = max([0.3] + [((t[0] - Pn[0]) * dn[0] + (t[1] - Pn[1]) * dn[1]) + 0.3
                           for t in stubs])
        self.spine = spine.extend(back, fwd)
        self._x_cache: Dict = {}
        self._rebase(launch)

    # ------------------------------------------------------------ launches
    def _rebase(self, launch: Dict[str, Pt]):
        """Launch classification and keys for these teeth (the source
        refinement moves them)."""
        sp = self.spine
        self.launch = launch
        st = {nm: sp.project_pt(launch[nm]) for nm in self.names}
        self.st = st
        self.s0 = max(s for s, _o in st.values()) + 0.3
        o_src = sp.project_pt(self.src_centre)[1]
        o_dst = sp.project_pt(self.dst_centre)[1]
        self.o_src, self.o_dst = o_src, o_dst

        def head_launch(nm):
            s_i, o_i = st[nm]
            for om in self.names:
                if om == nm:
                    continue
                s_j, o_j = st[om]
                if s_j > s_i + te.TOL_S and abs(o_j - o_i) < te.DIST_O:
                    return False
            run_end = sp.xy(self.s0, o_i)
            return self.obs_for(nm, self.tooth_layer.get(nm, 'F.Cu')).seg_clear(
                launch[nm], run_end)
        self.heads_l = {nm for nm in self.names if head_launch(nm)}
        self.joiners = [nm for nm in self.names if nm not in self.heads_l]
        self.join_side = {nm: (1 if st[nm][1] >= o_src else -1) for nm in self.joiners}
        s_max = max([s for s, _o in st.values()] + [0.0])
        self._lkey = {}
        N = len(self.names)
        for i, nm in enumerate(self.names):
            if nm in self.heads_l:
                self._lkey[nm] = st[nm][1]
            else:
                # first joiner outermost; two teeth at one s: the braid
                # places the legs in member order and jogs the second
                # inward, so the EARLIER member is the outer one
                sg = self.join_side[nm]
                self._lkey[nm] = sg * (BIG + s_max - st[nm][0] + 1e-6 * (N - i))
        self.s_max = s_max
        self.dn = (float(sp.d[-1, 0]), float(sp.d[-1, 1]))

    def rebased(self, launch: Dict[str, Pt]) -> 'BraidOrder':
        """The same spine with the teeth moved (a source-side search)."""
        other = BraidOrder.__new__(BraidOrder)
        other.__dict__.update(self.__dict__)
        other._rebase(launch)
        return other

    # ------------------------------------------------------------ exits
    def exit_info(self, n: str, m) -> Tuple[str, int, float, float]:
        """(kind, side, s_e, o_e) of a move's exit: 'head' when it
        leaves against the corridor's arrival flow, else 'side' on the
        side of the destination centre it lies."""
        pt = m if isinstance(m, tuple) else m.exit_pt
        d = None if isinstance(m, tuple) else m.direction
        key = (n, round(pt[0], 4), round(pt[1], 4), d)
        hit = self._x_cache.get(key)
        if hit is None:
            s_e, o_e = self.spine.project_pt(pt)
            if d is None:
                # a bare point: the face of the box it sits on
                x0, y0, x1, y1 = self.box
                dd = {'left': abs(pt[0] - x0), 'right': abs(pt[0] - x1),
                      'up': abs(pt[1] - y0), 'down': abs(pt[1] - y1)}
                d = min(dd, key=dd.get)
            v = _DIRV[d]
            dot = v[0] * self.dn[0] + v[1] * self.dn[1]
            kind = 'head' if dot < -0.5 else 'side'
            side = 1 if o_e >= self.o_dst else -1
            hit = (kind, side, s_e, o_e)
            self._x_cache[key] = hit
        return hit

    def _key_of(self, n: str, kind: str, sg: int, s_e: float, o_e: float) -> float:
        if kind == 'head':
            return o_e
        if n in self.heads_l:
            return sg * (BIG + s_e)
        return sg * (2 * BIG + self.s_max - self.st[n][0])

    def exit_key(self, n: str, m, t: Optional[Pt] = None) -> float:
        """A per-move key (the LIS refinement's slot DP compares one
        net's moves with no other choice in view): the face rule
        alone. Whether the FIRST stub along a flank counts as head-on
        (the braid's rule: nothing upstream at its offset) needs the
        other exits, and target_order applies it."""
        return self._key_of(n, *self.exit_info(n, m))

    def exit_kinds(self, grp: Sequence[str], sel) -> Dict[str, Tuple[str, int, float, float]]:
        """(kind, side, s_e, o_e) per net under the braid's own exit
        rule: head-on unless another member's stub lies clearly
        UPSTREAM at (nearly) its offset -- so a lane can run straight
        to it. That makes the first stub along a flank head-on and the
        ones behind it side exits, whatever face they are on."""
        info = {n: self.exit_info(n, sel[n]) for n in grp}
        out = {}
        for n in grp:
            kind, sg, s_e, o_e = info[n]
            head = not any(s_j < s_e - te.TOL_S and abs(o_j - o_e) < te.DIST_O
                           for m, (_k, _s, s_j, o_j) in info.items() if m != n)
            # and the straight run-in must be clear of static copper
            # (braid.classify's head_exit): a flank stub's is its row
            if head and not self.run_clear(n, sel[n]):
                head = False
            out[n] = ('head' if head else 'side', sg, s_e, o_e)
        return out

    def run_clear(self, n: str, m) -> bool:
        """Is a head-on run-in at the move's offset clear of static
        copper (cached per move)?"""
        if self.pad_obs is None:
            return True
        pt = m if isinstance(m, tuple) else m.exit_pt
        key = ('run', n, round(pt[0], 4), round(pt[1], 4))
        hit = self._x_cache.get(key)
        if hit is None:
            s_e, o_e = self.spine.project_pt(pt)
            hit = self.pad_obs.seg_clear(self.spine.xy(s_e - te.HEAD_RUN, o_e), pt)
            self._x_cache[key] = hit
        return hit

    def launch_key(self, n: str, t: Optional[Pt] = None) -> float:
        return self._lkey[n]

    def axis(self, grp, sel):
        return (0.0, 0.0)

    def order(self, grp: Sequence[str], sel: Dict[str, sm.Move],
              t: Optional[Pt] = None) -> List[str]:
        return sorted(grp, key=lambda n: self._lkey[n])

    def target_order(self, grp: Sequence[str], sel) -> List[str]:
        lo = self.order(grp, sel)
        li = {n: i for i, n in enumerate(lo)}
        kinds = self.exit_kinds(grp, sel)
        return sorted(grp, key=lambda n: (round(self._key_of(n, *kinds[n]), 6), li[n]))

    def keep(self, grp: Sequence[str], sel, weight=None) -> List[str]:
        """The LIS of the launch -> target permutation, as the braid's
        schedule will find it (no crossing pruning: the keys ARE the
        order the lanes are laid in)."""
        if len(grp) < 2:
            return list(grp)
        lo = self.order(grp, sel)
        tgt = self.target_order(grp, sel)
        tr = {n: i for i, n in enumerate(tgt)}
        ranks = [tr[n] for n in lo]
        if weight:
            idx = lis_keep_weighted(ranks, [weight.get(n, 0.0) for n in lo])
        else:
            idx = lis_keep(ranks)
        return [lo[i] for i in sorted(idx)]

    # ------------------------------------------------------------ columns
    def capacity(self, sel) -> Tuple[float, int]:
        """(free length, columns) the corridor has for swap columns:
        s0 .. the first exit (head-on - 0.6, side - 0.3), less the
        corner wedges, at W_GATE per column."""
        s1 = []
        omax = 0.0
        for n, (kind, sg, s_e, o_e) in self.exit_kinds(list(sel), sel).items():
            s1.append(s_e - (0.6 if kind == 'head' else 0.3))
            omax = max(omax, abs(o_e))
        omax = max([omax] + [abs(self._lkey[n]) for n in sel if n in self.heads_l]
                   + [te.LPITCH * len(self.joiners)])
        s1 = min(s1) if s1 else self.s0
        ivs = []
        for (_i, s_c, turn) in self.spine.corners():
            half = omax * math.tan(math.radians(min(abs(turn), 170.0) / 2)) + 0.3
            ivs.append((max(s_c - half, self.s0), min(s_c + half, s1)))
        reserved = sum(max(0.0, b - a) for a, b in te._intervals_union(ivs))
        L_free = max(0.0, s1 - self.s0 - reserved)
        return L_free, max(0, int((L_free - 0.3) / te.W_GATE) - 1)

    def leg_crossed(self, grp: Sequence[str], sel) -> List[str]:
        """The lanes an exit leg crosses: in a side-exit block the
        lanes leave one by one, each leg running across every lane
        still present between it and the flank, and a crossed lane
        pays two vias (the braid's exit-leg layer rule: on the other
        layer from the first leg that crosses it to its own corner).
        A lane is crossed when some lane OUTER to it in the block exits
        before it -- which is exactly what the human avoids by exiting
        the outer lanes last (SWE at the far end of the face)."""
        kinds = self.exit_kinds(grp, sel)
        out = []
        for sg in (-1, 1):
            block = [n for n in grp if kinds[n][0] == 'side' and kinds[n][1] == sg]
            if len(block) < 2:
                continue
            # inner -> outer by the braid's block key
            block.sort(key=lambda n: sg * self._key_of(n, *kinds[n]))
            for i, b in enumerate(block):
                s_b = kinds[b][2]
                if any(kinds[a][2] < s_b - 0.1 for a in block[i + 1:]):
                    out.append(b)
        return out

    def schedule(self, sel) -> Tuple[int, int, int, int, int]:
        """(vias, floor, columns, capacity, gated columns) of this choice
        under the braid's own schedule: the gated form ('last') while
        its columns fit the corridor, else ungated -- as the braid
        chooses -- with the vias that costs (a diver passed in flight
        dives again). `vias` is the estimate of what the braid spends
        in the corridor: two per diver, two per lane an exit leg
        crosses, two per in-flight surfacing."""
        grp = [n for n in self.names if n in sel]
        if len(grp) < 2:
            return 0, 0, 0, 0, 0
        lo = self.order(grp, sel)
        tgt = self.target_order(grp, sel)
        sched = Schedule(lo, tgt, self.tooth_layer)
        gaps = {d: 1 for d in sched.divers}
        lead = {d: 0 for d in sched.divers}
        cols = sched.columns(gaps, lead, gate='last')
        _L, cap = self.capacity(sel)
        n_gated = len(cols)
        if n_gated > cap:
            cols = sched.columns(gaps, lead, gate='off')
        floor = 2 * len(sched.divers)
        vias = floor + 2 * len(sched.surfaced) + 2 * len(self.leg_crossed(grp, sel))
        return vias, floor, len(cols), cap, n_gated

    def corridors(self, sel, D: float = 6.0) -> int:
        """How many corridors the braid would make of this choice --
        corridor.cluster_corridors' linkage: two stubs share a corridor
        when they are within D and their arrivals (seen from the
        destination's centre, 2 mm before the stub) are within 60
        degrees. The arrival is taken along the straight run from the
        tooth, the taut path to a stub that does not exist yet.
        Crossings between corridors are v1 in the braid: a second
        corridor is laid against the first one's copper and refused
        where it collides (K4 under this model without the count: two
        stubs sent down, two left, 7.6 mm apart -- two corridors, one
        lane open where one corridor of four routes clean)."""
        nets = [n for n in self.names if n in sel]
        parent = list(range(len(nets)))

        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i
        c = self.dst_centre
        arr, pts = {}, {}
        for n in nets:
            m = sel[n]
            pt = m if isinstance(m, tuple) else m.exit_pt
            pts[n] = pt
            u = _unit((pt[0] - self.launch[n][0], pt[1] - self.launch[n][1]))
            appr = (pt[0] - 2.0 * u[0], pt[1] - 2.0 * u[1])
            arr[n] = _unit((appr[0] - c[0], appr[1] - c[1]))
        for i, a in enumerate(nets):
            for j in range(i + 1, len(nets)):
                b = nets[j]
                if math.hypot(pts[a][0] - pts[b][0], pts[a][1] - pts[b][1]) > D:
                    continue
                dot = max(-1.0, min(1.0, arr[a][0] * arr[b][0] + arr[a][1] * arr[b][1]))
                if math.degrees(math.acos(dot)) <= 60.0:
                    ra, rb = find(i), find(j)
                    if ra != rb:
                        parent[ra] = rb
        return len({find(i) for i in range(len(nets))})

    def cost(self, sel) -> Tuple[int, int, int, int]:
        """What the plan minimises: (extra corridors, corridor vias,
        gated columns over capacity, columns). One corridor first (the
        braid lays a second against the first's copper), then the vias
        the braid will spend in the corridor (divers, exit-leg
        crossings, in-flight surfacings), then the columns the gated
        schedule cannot fit, then columns."""
        vias, _fl, cols, cap, n_gated = self.schedule(sel)
        return (self.corridors(sel) - 1, vias, max(0, n_gated - cap), cols)


def build_model(pcb, names, ends, byname, obs, paths, launch, tooth0,
                dref, sref, box, log=None) -> BraidOrder:
    """The model as fanout_from_plan needs it: `obs(nid, layer)` is the
    plan's obstacle builder (every planned net's copper excluded),
    `paths` its taut pre-routes, `launch` the teeth, `tooth0` their
    layers, `box` the destination grid's bbox."""
    tdirs = {nm: te._end_dir(pcb, byname[nm][0], launch[nm], byname[nm][1].pads)
             for nm in names}

    def centre(ref):
        ps = pcb.footprints[ref].pads
        return (sum(p.global_x for p in ps) / len(ps),
                sum(p.global_y for p in ps) / len(ps))
    kids = {byname[n][0] for n in names}
    big = {ref for ref, fp in pcb.footprints.items()
           if len(fp.pads) >= 10 and ref not in (sref, dref)}
    spine_obs = te.build_obstacles(pcb, -1, kids, 'F.Cu')
    base_obs = ts.Obstacles()
    for (x, y, r, name) in spine_obs.discs:
        if name.split('.')[0] in big:
            base_obs.add_disc(x, y, r, name)
    base_obs.build()
    return BraidOrder(box, launch, names, ends, paths, tooth0, tdirs,
                      lambda nm, L: obs(byname[nm][0], L), base_obs,
                      centre(sref), centre(dref),
                      pad_obs=te.array_pad_obstacles(pcb, {sref, dref}), log=log)


def refine_faces(choice: Dict[str, sm.Move], menu: Dict[str, List[sm.Move]],
                 model: BraidOrder, free, rounds: int = 6, log=None,
                 only: Optional[Sequence[str]] = None) -> Dict[str, sm.Move]:
    """Move nets between faces (and gaps) one at a time while the
    braid's schedule cost falls. select() chooses faces by reach,
    channel length and leg crossings, and only its LIS pass knows the
    floor -- within a face. This is the pass that can send a joiner to
    the exit block where its join order already puts it. `only`
    restricts the nets that may move (the joiners: the rest of the
    plan's decisions are left to select(), whose choices for the
    head-on nets the braid was tuned on -- letting the model move every
    net cost K11..K21 two open lanes and more vias each)."""
    cur = model.cost(choice)
    debug = os.environ.get('PLAN_DEBUG', '')
    allowed = set(only) if only is not None else None
    for r in range(rounds):
        moved = 0
        for n in model.order(list(choice), choice):
            if allowed is not None and n not in allowed:
                continue
            best_m, best_c = None, cur
            for m in menu.get(n, ()):
                if m is choice[n]:
                    continue
                if not free(m, choice, n):
                    if n == debug and log:
                        log(f'      {n} {m}: lane taken')
                    continue
                trial = dict(choice)
                trial[n] = m
                c = model.cost(trial)
                if n == debug and log:
                    log(f'      {n} {m}: {c}  (now {cur})')
                if c < best_c:
                    best_m, best_c = m, c
            if best_m is not None:
                if log:
                    log(f'    face refine: {n} {choice[n].direction}/{choice[n].layer[0]}'
                        f' -> {best_m.direction}/{best_m.layer[0]}  '
                        f'(floor, over, cols) {cur} -> {best_c}')
                choice[n] = best_m
                cur = best_c
                moved += 1
        if not moved:
            break
    return choice

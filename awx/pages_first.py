"""pages_first.py -- the PAGES-FIRST planner (#622, 2026-09-13; opt-in
PLAN_PAGES=1 in the fanout stage, the old planner byte-identical at 0).

The directive it serves: no net may need more than two vias. Under the
braid's two-page model a page lane costs at most two (one at each end whose
layer is not its page), and a lane that fits NEITHER page -- a swimmer --
is the only thing that costs more. So the plan must hand the braid launch
and target orders that two crossing-free chains cover completely, and
that is a property of the ENDS the plan chooses, not of the router.
Measured before this existed: on our recorded K41 / K51 plans the best any
two-page schedule can page is 28 of 41 / 28 of 47 lanes (13 / 19 must
swim); on the human's ends 35 / 41 (6 / 6).

Why the human's ends are orderable: per net the vias are 0 for an F tooth
on page F to an F berth, and exactly 2 for every other combination of
(tooth, page, berth) layers but one, so once a net has a B end its page is
free at no cost -- and a B end makes the net's RANK free, because under a
BGA the back layer has no pads and a dog-bone can climb along a gap and
leave the face at a chosen row (escape_moves climb=), or enter the field
from a chosen column. An F end pins the rank to the ball's own gap. The
recorded chain could not use that freedom: the source was frozen, the
destination menu had no climb class, and every chooser priced a swimmer at
two vias and accepted it.

This planner chooses BOTH ends and the page of every net in ONE CP-SAT:
  * per net, one destination move from its menu (climbed ones when
    DST_CLIMB enumerates them) and one source move from {the tooth as it
    stands} + its source menu (climbs under SRC_CLIMB);
  * a page per net; the cost of a net is its escape vias plus one per end
    whose layer differs from its page, plus the greedy's channel and reach
    weights as tie-breaks (the greedy's units, VIA_W / CHAN_W below);
  * HARD: two nets on one page are never inverted between the launch order
    and the target order -- keys from the braid's own slots (braid_slots),
    the Frame below at the destination and its mirror at the source for a
    net the braid did not key (the braid's nesting: front face by position
    across the bundle, side faces nested first-exiter innermost, the far
    face outermost). A net may still be left to swim, at a price
    of PLAN_PAGES_SWIM vias, so an infeasible instance answers with the
    fewest swimmers instead of nothing;
  * two moves that cannot both be laid (select_moves._conflict) exclude
    each other; berths a caller holds `fixed` are one-move menus; `learned`
    pairs the engine refused together exclude each other.
The braid then verifies the plan with its own planner (judge_by_braid): a
net it still swims is a disagreement between these keys and its order,
printed by name. Deterministic: CP-SAT under a deterministic-time budget
with interleaved search (PLAN_PAGES_DET), never wall-clock.
"""
from __future__ import annotations

import math
import os
import time
from typing import Dict, List, Optional, Tuple

import select_moves as sm
import source_realize as sr
from escape_moves import Move, DIRS
import solve_memo as _smemo  # noqa: E402  a solve read back instead of run

Pt = Tuple[float, float]

# the greedy's units: a via 3, a mm of channel 2, a mm of reach 1
VIA_W = 3.0
CHAN_W = 2.0


class Frame:
    """The bundle's frame at the destination: u along the bundle (launch
    centroid to the box centre), t across it (u's left normal); the
    box's half extents along and across. The key of a net the braid could
    not slot (braid_slots): the front face by its position across, a side
    face beyond the front keys, the far face outermost of its side."""

    def __init__(self, launch: Dict[str, Pt], box):
        x0, y0, x1, y1 = box
        self.box = box
        self.c = ((x0 + x1) / 2, (y0 + y1) / 2)
        lx = sum(p[0] for p in launch.values()) / len(launch)
        ly = sum(p[1] for p in launch.values()) / len(launch)
        ux, uy = self.c[0] - lx, self.c[1] - ly
        h = math.hypot(ux, uy) or 1.0
        self.u = (ux / h, uy / h)
        self.t = (-self.u[1], self.u[0])
        hx, hy = (x1 - x0) / 2, (y1 - y0) / 2
        self.Hu = abs(hx * self.u[0]) + abs(hy * self.u[1])
        self.Ht = abs(hx * self.t[0]) + abs(hy * self.t[1])

    def along(self, p: Pt) -> float:
        return (p[0] - self.c[0]) * self.u[0] + (p[1] - self.c[1]) * self.u[1]

    def across(self, p: Pt) -> float:
        return (p[0] - self.c[0]) * self.t[0] + (p[1] - self.c[1]) * self.t[1]

    def face(self, m: Move) -> Tuple[str, int]:
        """('front' | 'far' | 'side', sign across) of a move's face, from
        its direction's outward normal against the bundle."""
        n = DIRS[m.direction]
        d = n[0] * self.u[0] + n[1] * self.u[1]
        if d < -0.5:
            return 'front', 0
        if d > 0.5:
            return 'far', (1 if self.across(m.exit_pt) >= 0 else -1)
        s = n[0] * self.t[0] + n[1] * self.t[1]
        return 'side', (1 if s >= 0 else -1)

    def key(self, m: Move) -> float:
        """The move's place in the braid's target order (ascending across
        the bundle): the front face by its position across; a side face
        beyond the front keys, the exit nearest the front innermost; the
        far face outermost of its side, the exit nearest its corner
        innermost."""
        kind, sg = self.face(m)
        a, x = self.along(m.exit_pt), self.across(m.exit_pt)
        if kind == 'front':
            return x
        if kind == 'side':
            return sg * (self.Ht + 0.5 + (a + self.Hu))
        return sg * (self.Ht + 0.5 + 2 * self.Hu + 1.0 + (self.Ht - sg * x))

PAGES_DET = float(os.environ.get('PLAN_PAGES_DET', '40'))     # CP-SAT deterministic time. 40 (2026-09-14): at 20 the K41 solve stops FEASIBLE with 4 swimmers, at 40 with 2-3, 80 adds nothing; on the ladder 20 -> 40 took K41 87 -> 79/81 and K51 125 -> 115 complete, within the time budget (K41 92 s)
PAGES_WORKERS = int(os.environ.get('PLAN_PAGES_WORKERS', '4'))
PAGES_SWIM = float(os.environ.get('PLAN_PAGES_SWIM', '100'))  # vias: the price of a net left to swim
PAIR_SWIM = float(os.environ.get('PLAN_PAIR_SWIM', '3') or 3)   # a pair leg's swim price, times PAGES_SWIM (2026-09-20: SDQS1 planned as a swimmer weaved 6 page crossings)
PAGES_LOG = int(os.environ.get('PLAN_PAGES_LOG', '0'))        # 1 = per-net choice printed
PAGES_ITERS = int(os.environ.get('PLAN_PAGES_ITERS', '3'))    # re-key on the chosen plan, at most this often
PAGES_SIDEKEY = int(os.environ.get('PLAN_PAGES_SIDEKEY', os.environ.get('PLAN_PAGES_SIDERS', '1')))
PAGES_SIDERS_MODE = int(os.environ.get('PLAN_PAGES_SIDERS', '1'))
PAGES_STRICT = int(os.environ.get('PLAN_PAGES_STRICT', '1'))   # destination exclusions by the STRICT conflict test (the engine lays the geometry asked; non-strict let 14 verbatim berths collide at K41)
PAGES_HINT = int(os.environ.get('PLAN_PAGES_HINT', '1'))      # 1 = the seed plan (the greedy's berths, the teeth as they stand) as the CP-SAT's solution hint. At DET 20 it measured WORSE standalone (K41 obj 2359.8 / 4 swimmers -> 2659.4 / 5); at DET 40 on the LADDER it is worth 2 vias at K41 (81 -> 79) and completion at K51 (106 / SA2 open -> 115 complete), 2026-09-14 pg2 vs pg3. On, with DET 40.
PAGES_JOINKEY = int(os.environ.get('PLAN_PAGES_JOINKEY', '1'))  # 1 = a side exit's slot depends on its SOURCE class: lanes whose tooth is a joiner sit outermost of the block, by tooth position (the braid's exit-block rule)
# PLAN_PAGES_MISMATCH: the price of an end whose layer is not its page, as
# a multiple of a via (1 = as it was).
PAGES_MISMATCH = float(os.environ.get('PLAN_PAGES_MISMATCH', '1') or 1)
# THE INSTANCE, WRITTEN OUT (2026-09-15, session 9): PLAN_PAGES_DUMP=<dir>
# writes every CP-SAT model this module solves, as built and hinted, to
# <dir>/<board>_solve<n>.pb (binary CpModelProto) beside a .json naming the
# nets and the parameters -- so the CHAIN'S OWN instance (in-process plan()
# takes a different greedy seed) can be re-solved offline against
# deterministic time, and the objective / bound curve read off it. Inert
# when unset: nothing in the solve changes.
PAGES_DUMP = os.environ.get('PLAN_PAGES_DUMP', '')
import schedule as _schedule
import braid as te
# a row-line run and a column-line run on ONE layer that cross are a
# conflict whatever the moves' kinds -- select_moves tests it only for
# climbed moves at its default SEL_XING=1 (K41 pages-first pass 0:
# 17 same-layer crossings of plain via-in-pad runs, laid as asked, DRC).
# The planner's exclusions use the full test; the standard planner's
# default is unchanged (its own SEL_XING=2 is the opt-in to measure).
sm.SEL_XING = max(sm.SEL_XING, 2)
# the two-page schedule assigned EXACTLY (BRAID_EXACT_PAGES): a plan two
# chains cover must be paged as two chains, which the greedy pager (the LIS
# of one page first) can miss. In-process for the judge; the plan sidecar
# carries `pages_first` so the braid stage does the same.
#
# HONOUR THE FLAG (review, 2026-09-17). This used to be a bare `= 1`,
# consulting no environment at all -- so `BRAID_EXACT_PAGES=0` could not
# turn it off ON THE PLANNER SIDE even though braid.py had been fixed to
# honour it. That matters because `judge_by_braid` calls the braid
# IN-PROCESS, and importing this module pins the judge's schedule to
# EXACT_PAGES=1 for the whole run: the planner then scores every
# candidate as ARM A of the braid portfolio while the chain frequently
# ships ARM B (the K51 log's "the marker-OFF arm won"). The marker is
# worth up to 14 vias, so the judge was systematically modelling the
# wrong regime on the rung where the deficit is worst.
# The braid SUBPROCESS was never affected -- braid.py does not import
# this module, so every ladder number measured through chain_k.sh stands.
_schedule.EXACT_PAGES = 0 if os.environ.get('BRAID_EXACT_PAGES') == '0' else 1
SCALE = 100                                                   # cost units -> ints


class SrcFrame(Frame):
    """The destination frame's mirror at the SOURCE: the same `t` axis (so
    `across` agrees at both ends), `u` reversed (the source's front face is
    the one facing the destination), half extents of the source box."""

    def __init__(self, fr: Frame, box):
        x0, y0, x1, y1 = box
        self.box = box
        self.c = ((x0 + x1) / 2, (y0 + y1) / 2)
        self.u = (-fr.u[0], -fr.u[1])
        self.t = fr.t
        hx, hy = (x1 - x0) / 2, (y1 - y0) / 2
        self.Hu = abs(hx * self.u[0]) + abs(hy * self.u[1])
        self.Ht = abs(hx * self.t[0]) + abs(hy * self.t[1])


def current_tooth(st, nm) -> Optional[Move]:
    """The tooth AS IT STANDS on the board as a Move: what the engine laid,
    read off the copper (source_realize.measure_tooth). None for a net
    without a source pad on the source array or without copper there."""
    p = st['src_pad'].get(nm)
    if p is None or p.component_ref != st['sref']:
        return None
    g = sr.measure_tooth(st['pcb'], nm, p, st['byname'])
    if not g or g.get('direction') not in DIRS or g.get('layer') not in ('F.Cu', 'B.Cu'):
        return None
    return Move(net=nm, kind=g['kind'], direction=g['direction'], layer=g['layer'],
                exit_pt=tuple(g['tooth']), vias=g['vias'], legs=[],
                site=(tuple(g['site']) if g.get('site') else None))


def _conflicts(cands: Dict[str, List[Move]], strict: bool) -> List[Tuple[str, int, str, int]]:
    """Every (net a, index, net b, index) whose two moves cannot both be
    laid, tested only between moves sharing a lane, a site or an exit
    (bucketed: the all-pairs test is 10^6-10^7 calls at K41)."""
    buckets: Dict[tuple, List[Tuple[str, int]]] = {}
    for nm, ms in cands.items():
        for i, m in enumerate(ms):
            keys = set()

            def q(v, tol=sm._EXIT_TOL):
                # two values within `tol` of each other share at least one
                # of these two cells (a single rounding can split them)
                c = int(math.floor(v / tol))
                return (c, c + 1)
            for key, a_, b_ in sm._lane_spans(m):
                cells = q(key[1]) if strict else (round(key[1], 3),)
                for c in cells:
                    keys.add(('lane', key[0], c, key[2]))
                # a row run and a column run on ONE layer that CROSS share no
                # lane, site or exit key, so bucket them by the 1 mm cells the
                # row run spans and the cell the column run stands in
                # (2026-09-14: 14 such pairs at K41 pass 0 were never tested)
                if key[0] == 'row':
                    for cx in range(int(math.floor(a_)), int(math.floor(b_)) + 1):
                        keys.add(('xc', key[2], cx))
                else:
                    keys.add(('xc', key[2], int(math.floor(key[1]))))
            if m.site is not None:
                # a site in another move's lane: bucket the lane through the
                # site by its line, on both layers (a via spans them all)
                keys.add(('site', round(m.site[0], 2), round(m.site[1], 2)))
                for axis, v in (('row', m.site[1]), ('col', m.site[0])):
                    cells = q(v) if strict else (round(v, 3),)
                    for c in cells:
                        for L in ('F.Cu', 'B.Cu'):
                            keys.add(('lane', axis, c, L))
            for cx in q(m.exit_pt[0]):
                for cy in q(m.exit_pt[1]):
                    keys.add(('exit', cx, cy))
            # SORTED: `keys` is a set of string-keyed tuples, whose iteration
            # order follows the process's hash seed -- and the ORDER the
            # exclusions reach the CP-SAT decides which FEASIBLE answer a
            # deterministic-time solve lands on (measured 2026-09-14, K41:
            # one process gave obj 1784, the next 1727, the next 1809 on
            # one model; in one process three solves agreed). The model
            # must be built in a canonical order.
            for k in sorted(keys, key=repr):
                buckets.setdefault(k, []).append((nm, i))
    seen = set()
    out = []
    for members in buckets.values():
        if len(members) < 2:
            continue
        for x in range(len(members)):
            a, i = members[x]
            for y in range(x + 1, len(members)):
                b, j = members[y]
                if a == b or (a, i, b, j) in seen or (b, j, a, i) in seen:
                    continue
                seen.add((a, i, b, j))
                if sm._conflict(cands[a][i], cands[b][j], strict=strict):
                    out.append((a, i, b, j))
    return out


def braid_slots(st, board, names, seed, src_seed, D, S, log):
    """Every candidate's place in the BRAID'S OWN order, the other lanes
    held at the seed plan: the corridors built on the seed (setup +
    Corridor.run(plan_only), 0.4 s), then each destination candidate's slot
    by `_alt_geo` + `_alt_slot` (a head-on exit's own offset, else inserted
    into its side's exit comb by the comb's rule) and each tooth candidate's
    by `_alt_src_slot` (its own offset, else into its side's join block);
    the tooth as it stands keeps `launch_o`. Returns (tkeys, lkeys,
    corridor index per net); a net the braid did not plan gets None."""
    import fanout_from_plan as F
    st_seed = st
    for nm, mv in (src_seed or {}).items():
        st_seed = F._st_with_src(st_seed, nm, mv)
    plan = F.braid_plan_of(st_seed, seed, board)
    plan['pages_first'] = True          # the braid's rules for a pages-first plan (siders, exact pages)
    quiet = lambda m='': None
    ctx, groups = te.setup(board, list(seed), st['dref'], quiet, plan=plan)
    corridors = [te.Corridor(ci, M, ctx, quiet) for ci, M in enumerate(groups)]
    ctx.corridors = corridors
    for c in corridors:
        try:
            c.run(plan_only=True)
            ctx.laid.extend(c.lane_xy[nm] for nm in c.members if nm in getattr(c, 'lane_xy', {}))
            if getattr(c, 'spine_core', None) is not None:
                ctx.laid_tubes.append((c.spine_core.pts, c.H))
        except Exception as e:
            log(f'  pages-first: corridor {c.idx} not planned ({e})')
    tkeys, lkeys, corr = {}, {}, {}
    jkeys, joiner = {}, {}
    errs: Dict[str, list] = {}       # net -> [(end, candidate, exception)] for the candidates no key could be built for          # PAGES_JOINKEY: the joined variant of each dest key; joiner flag per source candidate
    for c in corridors:
        if not hasattr(c, 'launch_o') or not hasattr(c, 'target_o'):
            continue
        # the braid's orders ARE its offsets sorted: say so if not
        lo = [c.launch_o[n] for n in c.launch if n in c.launch_o]
        to = [c.target_o[n] for n in c.target if n in c.target_o]
        if lo != sorted(lo) or to != sorted(to):
            log(f'  pages-first: WARNING corridor {c.idx} orders are not its offsets sorted')
        for nm in c.members:
            if nm not in D:
                continue
            corr[nm] = c.idx
            tk = []
            jk = []
            sp = c.spine
            dn = sp.d[-1]                      # the spine's direction at the destination
            ref = c.ctx.ends[nm][2]
            # the joined lanes sit beyond EVERY port of their side: the shift
            # is the largest port term any candidate of this corridor can
            # take (half the farthest stub's distance past s1) plus a pitch --
            # a fixed number would be a scale read off one board
            s_far = max([sp.project_pt(tuple(mv.exit_pt))[0] for om in c.members if om in D
                         for mv in D[om]] + [c.s1])
            join_shift = 0.5 * max(0.0, s_far - c.s1) + te.LPITCH
            for m in D[nm]:
                try:
                    g = c._alt_geo(nm, {'exit': list(m.exit_pt), 'layer': m.layer, 'dir': list(DIRS[m.direction])})
                    d_ = DIRS[m.direction]
                    along = d_[0] * dn[0] + d_[1] * dn[1]
                    # mode 2: a side-face candidate is keyed by the comb whether
                    # or not it tests head-on on the seed -- among a face's
                    # stubs the order is by position either way, and the one
                    # the braid makes head-on (the most upstream) is innermost
                    if PAGES_SIDEKEY and (not g.get('head') or (PAGES_SIDERS_MODE == 2 and abs(along) < 0.5)):
                        # a SIDE-FACE stub (its direction across the spine).
                        # The braid's relative head-on test makes the most
                        # upstream stub of the face head-on and every other
                        # one a side exit, so a candidate's class -- and its
                        # slot -- flips with which neighbours are chosen;
                        # with no stub of the face in the seed EVERY candidate
                        # tested head-on at the face line, one key for the
                        # whole face. The ORDER the braid ends up with is the
                        # comb's whatever the class: beyond the front lanes on
                        # that side, the first exiter innermost. Key it so.
                        # ONE formula for every side exit of a side, the far
                        # face included: beyond that side's head-on lanes, then
                        # nested by position along the spine (the comb's rule,
                        # first exiter innermost, the far face -- the largest s
                        # -- outermost). Two scales (the insertion slot for the
                        # far face, this for the side faces) could not be
                        # compared: every remaining K28 disagreement was a
                        # far-face berth against a side-face comb.
                        s_e, o_e = g['se']
                        sg = g['side'] if g.get('side') else c._side_of(o_e, ref)
                        ext = max([sg * c.target_o[om] for om in getattr(c, 'heads_e', ()) if om != nm]
                                  + [sg * abs(o_e) * 0 + 0.0])
                        tk.append(float(sg * (ext + te.BLOCK_GAP) + sg * 0.5 * max(0.0, s_e - c.s1)))
                        # the JOINED variant: a lane whose tooth is a joiner
                        # sits outermost of its exit block, the joiners
                        # ordered by their tooth's s (the nearest the
                        # corridor innermost) -- offsets(): order = ports + joined
                        s_t = c.st[nm][0] if nm in c.st else c.s0
                        jk.append(float(sg * (ext + te.BLOCK_GAP + join_shift + 0.5 * max(0.0, c.s0 - s_t))))
                    else:
                        v = float(c._alt_slot(nm, g))
                        tk.append(v); jk.append(v)
                except Exception as e:
                    tk.append(None); jk.append(None)
                    errs.setdefault(nm, []).append(('dst', sr.fmt_ask(m), repr(e)))
            tkeys[nm] = tk
            jkeys[nm] = jk
            # the source class per candidate: the tooth as it stands by the
            # corridor's own classification, a candidate by _alt_geo
            jf = []
            for i, s in enumerate(S[nm]):
                if i == 0 and not s.legs:
                    jf.append(nm in getattr(c, 'join_block', {}))
                    continue
                try:
                    g = c._alt_geo(nm, {'end': 'src', 'exit': list(s.exit_pt), 'layer': s.layer})
                    jf.append(not g['head'])
                except Exception:
                    jf.append(False)
            joiner[nm] = jf
            lk = []
            for i, s in enumerate(S[nm]):
                if i == 0 and not s.legs:
                    lk.append(float(c.launch_o.get(nm, 0.0)))
                    continue
                try:
                    g = c._alt_geo(nm, {'end': 'src', 'exit': list(s.exit_pt), 'layer': s.layer})
                    lk.append(float(c._alt_src_slot(nm, g)))
                except Exception as e:
                    lk.append(None)
                    errs.setdefault(nm, []).append(('src', sr.fmt_ask(s), repr(e)))
            lkeys[nm] = lk
    braid_slots.extra = (jkeys, joiner)
    braid_slots.errors = errs
    if errs:
        log(f'  pages-first: {len(errs)} net(s) with candidates the braid could not key: '
            + '; '.join(f'{nm} {len(v)} ({v[0][0]} {v[0][1]}: {v[0][2][:80]})' for nm, v in sorted(errs.items())))
    return tkeys, lkeys, corr


def verify(st, board, names, dst_choice, src_choice):
    """The braid's own planner on the chosen plan (exact pages): the nets it
    still swims. This is what the chain will do, so it is the verdict."""
    import fanout_from_plan as F
    st2 = st
    for nm, mv in src_choice.items():
        st2 = F._st_with_src(st2, nm, mv)
    plan = F.braid_plan_of(st2, dst_choice, board)
    plan['pages_first'] = True
    bp = te.plan_braid(board, list(dst_choice), st['dref'], plan)
    swim = [nm for nm in dst_choice if bp.get(nm, {}).get('page') is None]
    # PLAN_JUDGE: the braid's plan-implied count of this plan (the teeth
    # as chosen, priced on the same planner answer); None as recorded
    cost = F.judge_by_braid(st2, dst_choice, board, bp=bp)[0] if F.PLAN_JUDGE else None
    return swim, bp, cost


def _pairs_off(names, bp):
    """The model-vs-braid KEY disagreement on a verified plan (section E's
    instrument): over the pairs the braid put in one corridor, how many
    the last solve's keys (L, T) order differently from the braid's slots
    (launch_idx, target_idx). Returns (off, pairs) or None."""
    keys = getattr(_solve, 'last_keys', None)
    if not keys:
        return None
    L, T = keys['L'], keys['T']
    ns = [n for n in names if n in L and bp.get(n, {}).get('launch_idx') is not None
          and bp[n].get('target_idx') is not None]
    off = tot = 0
    for i, a in enumerate(ns):
        for b in ns[i + 1:]:
            if bp[a]['corridor'] != bp[b]['corridor']:
                continue
            tot += 1
            # does the MODEL think the pair crosses where the braid does not,
            # or the reverse? (orientation-invariant: a mirrored frame flips
            # every order and no crossing)
            x_model = (L[a] < L[b]) != (T[a] < T[b])
            x_braid = (bp[a]['launch_idx'] < bp[b]['launch_idx']) != (bp[a]['target_idx'] < bp[b]['target_idx'])
            if x_model != x_braid:
                off += 1
    return off, tot


def choose(st, board, log=print, fixed=None, learned=None, src_free=True, seed=None):
    """The pages-first choice, re-keyed on its own answer until the braid's
    planner agrees (at most PAGES_ITERS solves). `src_free` False: the tooth
    as it stands is the only source candidate (the destination re-plan
    loop, which realizes no source move). `seed`: the plan the corridors
    are first built on (the greedy's choice)."""
    rep = []
    best = None
    seed_d = dict(seed or {})
    seed_s: Dict[str, Move] = {}
    hold_d, hold_s, avoid = None, None, None
    for it in range(max(1, PAGES_ITERS)):
        fx = dict(fixed or {})
        if hold_d:
            fx.update(hold_d)
        dst_choice, src_choice, lines, model = _solve(st, board, log, fx, learned, src_free,
                                                       seed_d, seed_s, hold_s, avoid, hard_fixed=fixed)
        rep += lines
        if not dst_choice:
            break
        swim, bp, cost = verify(st, board, list(dst_choice), dst_choice, src_choice)
        rep.append(f'  pages-first: iteration {it}: the braid\'s planner swims {len(swim)} '
                   f'{swim if swim else ""} on this plan (model {model["swim"]})'
                   + (f'; the braid\'s count {cost:.0f}' if cost is not None else ''))
        import fanout_from_plan as F
        # the key: (residue, the model's vias) as recorded; under
        # PLAN_JUDGE (the braid's count, residue) -- fanout_from_plan.pf_key
        key = F.pf_key(dst_choice, bp, cost, model['vias'])
        if best is None or F.pf_better(key, best[0]):
            best = (key, dst_choice, src_choice, dict(model, swim_braid=len(swim), count=cost), swim, bp)
        elif it > 0:
            break               # no better
        if not best[4]:
            break
        # DAMPED re-key: the braid's verdict is exact for the moves it just
        # planned, and the insertion rule is exact only with the OTHER lanes
        # held -- so the next solve frees the SWIMMERS, each barred from the
        # berth that swam, and holds every other net at the best plan (its
        # berth as a one-move menu, its tooth as chosen). Free to move
        # everything, the loop oscillated (K28: 6 -> 5 -> 6 swimmers).
        base_d, base_s, base_swim = best[1], best[2], best[4]
        free = set(base_swim)
        hold_d = {n: sr.move_sig(mv) for n, mv in base_d.items() if n not in free}
        hold_s = {n: base_s.get(n) for n in base_d if n not in free}   # None = the tooth as it stands
        avoid = {w: {sr.move_sig(base_d[w])} for w in base_swim}
        rep.append(f'  pages-first: re-solving {len(free)} net(s) {sorted(free)} with {len(hold_d)} held'
                   ' (the swimmers, each barred from its berth)')
        seed_d, seed_s = base_d, base_s
    if best is None:
        choose.last = {}
        return {}, {}, rep
    choose.last = best[3]
    return best[1], best[2], rep


def _solve(st, board, log, fixed, learned, src_free, seed, src_seed, hold_s=None, avoid=None,
           hard_fixed=None):
    """The pages-first choice on a plan state. Returns (dst_choice,
    src_choice, report): dst_choice {net: Move} for every net with a
    destination menu, src_choice {net: Move} for the nets whose tooth
    should MOVE (the source menu's move; a net keeping its tooth is not
    in it), and the report lines."""
    from ortools.sat.python import cp_model
    t0 = time.time()
    fixed = dict(fixed or {})
    learned = learned or set()
    launch: Dict[str, Pt] = st['launch']
    dbox = st['dgrid'].bbox
    names = [n for n in launch if st['dmenu'].get(n)]
    fr = Frame({n: launch[n] for n in names}, dbox)
    sfr = SrcFrame(fr, st['sgrid'].bbox)

    # ---- candidates
    D: Dict[str, List[Move]] = {}
    for n in names:
        ms = list(st['dmenu'][n])
        if n in fixed:
            hit = [m for m in ms if sr.move_sig(m) == fixed[n]]
            if hit:
                ms = hit[:1]
        if avoid and n in avoid:
            ms2 = [m for m in ms if sr.move_sig(m) not in avoid[n]]
            ms = ms2 or ms
        D[n] = ms
    # PAIRS UNHOLD (2026-09-20): a pair's two berths must be neighbours with
    # nothing between them. When every neighbouring combination in the two
    # legs' menus has a HELD berth (a one-move menu) between it, the holders
    # of the cheapest combination get their full menus back, so the solve
    # can move them and the pair constraint binds them by its clause.
    # Without this the re-solve is INFEASIBLE and the greedy choice, which
    # knows no pairs, stands (K36 pf8: SODT0 held between SCK's dogbones,
    # SDQ11/SDQ8 standing between SDQS1's teeth).
    if fixed and int(os.environ.get('PLAN_PAIRS', os.environ.get('BRAID_PAIRS', '0')) or 0):
        import pairs as _pairs
        _dg = st['dgrid']
        _reach_b = 1.3 * max(_dg.pitch_x, _dg.pitch_y)
        hard = set(hard_fixed or {})
        freed_p = []
        for _base, (_pn, _nn) in _pairs.pair_names(names).items():
            if _pn not in D or _nn not in D:
                continue
            best = None
            for a in D[_pn]:
                for b in D[_nn]:
                    ea, eb = getattr(a, 'exit_pt', None), getattr(b, 'exit_pt', None)
                    if ea is None or eb is None or (a.direction, a.layer) != (b.direction, b.layer):
                        continue
                    d = math.hypot(ea[0] - eb[0], ea[1] - eb[1])
                    if not (0.05 < d <= _reach_b):
                        continue
                    ax = 0 if a.direction in ('up', 'down') else 1
                    lo_, hi_ = sorted((ea[ax], eb[ax]))
                    held_between = set()
                    for o in names:
                        if o in (_pn, _nn) or len(D.get(o, [])) != 1:
                            continue
                        mo = D[o][0]
                        eo = getattr(mo, 'exit_pt', None)
                        if eo is not None and (mo.direction, mo.layer) == (a.direction, a.layer) \
                                and lo_ + 0.02 < eo[ax] < hi_ - 0.02:
                            held_between.add(o)
                    key = (len(held_between), a.vias + b.vias, d)
                    if best is None or key < best[0]:
                        best = (key, held_between)
            if best is None or not best[1]:
                continue            # a free combination exists, or none at all (the constraint says so)
            if best[1] & hard:
                continue            # the berth between is laid copper: nothing to free
            for o in sorted(best[1]):
                D[o] = list(st['dmenu'][o])
                freed_p.append(o)
        if freed_p:
            log(f'  pages-first: pairs: {len(freed_p)} held berth(s) between a pair freed {sorted(set(freed_p))}')
    S: Dict[str, List[Move]] = {}
    cur: Dict[str, Optional[Move]] = {}
    for n in names:
        c = current_tooth(st, n)
        cur[n] = c
        opts = [c] if c is not None else []
        if hold_s is not None and n in hold_s:
            # held at the verified plan: its chosen tooth move, or as it stands
            if hold_s[n] is not None:
                opts = [c, hold_s[n]] if c is not None else [hold_s[n]]
        elif src_free and c is not None:
            opts += list(st['smenu'].get(n, []))
        S[n] = opts
    tkey = {n: [int(round(fr.key(m) * 1000)) for m in D[n]] for n in names}
    lkey = {}
    for n in names:
        if S[n]:
            lkey[n] = [int(round(sfr.key(m) * 1000)) for m in S[n]]
        else:
            lkey[n] = [int(round(fr.across(launch[n]) * 1000))]
    corr = {n: 0 for n in names}
    jkey, jflag = {}, {}
    if seed is not None:
        # THE BRAID'S OWN ORDER as the keys, the other lanes held at the seed.
        # A net the greedy left UNPLACED is not in the seed, so the braid's
        # plan phase never sees it: no corridor, frame keys, corr -1 -- and
        # therefore OUTSIDE every planarity pair, free to swim in the model
        # at no cost (K41: 6 of 41 nets, and they were the braid's swimmers).
        # For KEYING ONLY, such a net is seeded at its cheapest berth (the
        # greedy's own ranking, conflicts ignored); the solve still chooses
        # among all of its candidates.
        seed = dict(seed)
        unplaced = [n for n in names if n not in seed and D[n]]
        for n in unplaced:
            seed[n] = min(D[n], key=lambda mv: VIA_W * mv.vias + CHAN_W * sm._length(mv)
                          + sm.around_box(launch[n], mv.exit_pt, dbox))
        if unplaced:
            log(f'  pages-first: {len(unplaced)} net(s) the seed left unplaced, keyed at their cheapest berth: {unplaced}')
        tk_b, lk_b, corr_b = braid_slots(st, board, names, seed, src_seed, D, S, log)
        n_keyed = 0
        for n in names:
            if n in tk_b and all(v is not None for v in tk_b[n]) and all(v is not None for v in lk_b.get(n, [])):
                tkey[n] = [int(round(v * 1000)) for v in tk_b[n]]
                if lk_b.get(n):
                    lkey[n] = [int(round(v * 1000)) for v in lk_b[n]]
                corr[n] = corr_b[n]
                n_keyed += 1
            else:
                corr[n] = -1      # not planned by the braid: Frame keys, its own corridor
        missing = [n for n in names if corr[n] == -1]
        if missing:
            log(f'  pages-first: {len(missing)} net(s) keyed by the frame, not the braid: {missing}')
        jkeys_b, joiner_b = getattr(braid_slots, 'extra', ({}, {}))
        if PAGES_JOINKEY:
            for n in names:
                if corr[n] != -1 and n in jkeys_b and all(v is not None for v in jkeys_b[n]):
                    jkey[n] = [int(round(v * 1000)) for v in jkeys_b[n]]
                    jflag[n] = list(joiner_b.get(n, []))

    # ---- the model
    m = cp_model.CpModel()
    xd = {n: [m.NewBoolVar(f'd_{n}_{j}') for j in range(len(D[n]))] for n in names}
    xs = {n: [m.NewBoolVar(f's_{n}_{i}') for i in range(max(1, len(S[n])))] for n in names}
    pg = {n: m.NewBoolVar(f'p_{n}') for n in names}          # 1 = page B
    sw = {n: m.NewBoolVar(f'w_{n}') for n in names}          # left to swim
    # THE PAIRS AS ONE MOVE (pairs.py, 2026-09-20): a differential pair's two
    # nets take moves of ONE face and ONE layer with neighbouring exits, at
    # BOTH ends -- a constraint of the plan, not a repair after it. The
    # post-fix (pairs.harmonise) could only move one leg where the other's
    # class had room: SDQS1's teeth ended 13.7 mm apart, SCK's berths 7.4.
    # A pair with no compatible combination in its menus at an end is left
    # unconstrained there and named. Gated with the braid's pair member.
    import pairs as _pairs
    n_pair_c = 0
    _pair_legs = set()        # a pair's legs swim at PAIR_SWIM times the price: a swimming pair weaves both legs
    if int(os.environ.get('PLAN_PAIRS', os.environ.get('BRAID_PAIRS', '0')) or 0):
        _pair_legs = {leg for pr in _pairs.pair_names(names).values() for leg in pr}
        _dg, _sg = st['dgrid'], st['sgrid']
        _reach = {'berths': 1.3 * max(_dg.pitch_x, _dg.pitch_y),
                  'teeth': 1.3 * max(_sg.pitch_x, _sg.pitch_y)}

        def _compat(a, b, reach):
            ea, eb = getattr(a, 'exit_pt', None), getattr(b, 'exit_pt', None)
            if ea is None or eb is None:
                return True
            if (a.direction, a.layer) != (b.direction, b.layer):
                return False
            d = math.hypot(ea[0] - eb[0], ea[1] - eb[1])
            return 0.05 < d <= reach
        n_hand = 0
        for _base, (_pn, _nn) in _pairs.pair_names(names).items():
            if _pn not in names or _nn not in names:
                continue
            combos = {'berths': [], 'teeth': []}      # (j, k, hand) per end, variables live
            fixed_hand = {}                           # an end with no variables: its hand as a constant
            for X, C, end in ((xd, D, 'berths'), (xs, S, 'teeth')):
                A, B = C.get(_pn, []), C.get(_nn, [])
                if len(A) <= 1 and len(B) <= 1:
                    if len(A) == 1 and len(B) == 1:
                        fixed_hand[end] = _pairs.hand(A[0].direction, A[0].exit_pt, B[0].exit_pt,
                                                      arriving=(end == 'berths'))
                    continue
                if len(X[_pn]) != len(A) or len(X[_nn]) != len(B):
                    continue          # an empty source menu's dummy variable
                okj = {j: [k for k, b in enumerate(B) if _compat(a, b, _reach[end])] for j, a in enumerate(A)}
                relaxed = False
                if not any(okj.values()):
                    # no neighbouring combination: ONE FACE AND ONE LAYER will
                    # do (the pair's legs leave through the comb one by one
                    # and are converged by the router's approach after it --
                    # zynq DQS0's P ball, an outer-column ball with a cap
                    # behind it, has no neighbouring tooth at all)
                    okj = {j: [k for k, b in enumerate(B) if _compat(a, b, 4.0 * _reach[end])]
                           for j, a in enumerate(A)}
                    relaxed = True
                    if not any(okj.values()):
                        log(f'  pages-first: pair {_base}: no {end} of one face and layer '
                            f'in its menus -- unconstrained there')
                        continue
                    log(f'  pages-first: pair {_base}: no neighbouring {end} in its menus -- '
                        f'one face and layer accepted, the approach converges the legs')
                for j, ks in okj.items():
                    for k in ks:
                        combos[end].append((j, k, _pairs.hand(A[j].direction, A[j].exit_pt, B[k].exit_pt,
                                                                 arriving=(end == 'berths'))))
                for j, ks in okj.items():
                    if ks:
                        m.AddBoolOr([X[_nn][k] for k in ks]).OnlyEnforceIf(X[_pn][j])
                    else:
                        m.Add(X[_pn][j] == 0)
                # ...and NOTHING BETWEEN them: a third net's exit of the same
                # face and layer strictly between the two is barred while
                # both stand (K36 pf7: SDQS1's teeth 0.96 mm apart passed the
                # reach, with SDQ11 and SDQ8 between them on a 0.32 mm comb;
                # the pair could not be launched coupled)
                # ...but NOT in the relaxed case (tried 2026-09-20 for zynq
                # K47 DQS0, whose teeth straddle DQ6's and DQ0's: a third
                # tooth with a one-move menu made the model INFEASIBLE, the
                # greedy fallback moved DQS0_N to another face and split
                # DQS1's berths across layers -- K44 100/0 open -> 116/2
                # open). The layer a straddled tooth LEAVES on is the braid
                # schedule's page, and that is where such a rule belongs.
                n_between = 0
                for j, ks in ([] if relaxed else okj.items()):
                    a = A[j]
                    for k in ks:
                        b = B[k]
                        ax = 0 if a.direction in ('up', 'down') else 1
                        lo_, hi_ = sorted((a.exit_pt[ax], b.exit_pt[ax]))
                        for o in names:
                            if o in (_pn, _nn) or len(X[o]) != len(C.get(o, [])):
                                continue
                            for i, mo in enumerate(C[o]):
                                eo = getattr(mo, 'exit_pt', None)
                                if eo is None or (mo.direction, mo.layer) != (a.direction, a.layer):
                                    continue
                                if lo_ + 0.02 < eo[ax] < hi_ - 0.02:
                                    m.AddBoolOr([X[_pn][j].Not(), X[_nn][k].Not(), X[o][i].Not()])
                                    n_between += 1
                if n_between:
                    log(f'  pages-first: pair {_base} {end}: {n_between} third-exit-between clause(s)')
                okk = {k: [j for j, a in enumerate(A) if _compat(a, b, (4.0 if relaxed else 1.0) * _reach[end])]
                       for k, b in enumerate(B)}
                for k, js in okk.items():
                    if js:
                        m.AddBoolOr([X[_pn][j] for j in js]).OnlyEnforceIf(X[_nn][k])
                    else:
                        m.Add(X[_nn][k] == 0)
                n_pair_c += 1
            # ...and ONE HANDEDNESS at both ends (pairs.hand): a teeth
            # combination and a berth combination that disagree cannot both
            # stand; an end without variables contributes its hand as a fact
            hb = combos['berths']
            ht = combos['teeth']
            if hb and ht:
                for (jb, kb, sb) in hb:
                    for (jt, kt, st_) in ht:
                        if sb and st_ and sb != st_:
                            m.AddBoolOr([xd[_pn][jb].Not(), xd[_nn][kb].Not(), xs[_pn][jt].Not(), xs[_nn][kt].Not()])
                            n_hand += 1
            elif hb and fixed_hand.get('teeth'):
                for (jb, kb, sb) in hb:
                    if sb and sb != fixed_hand['teeth']:
                        m.AddBoolOr([xd[_pn][jb].Not(), xd[_nn][kb].Not()])
                        n_hand += 1
            elif ht and fixed_hand.get('berths'):
                for (jt, kt, st_) in ht:
                    if st_ and st_ != fixed_hand['berths']:
                        m.AddBoolOr([xs[_pn][jt].Not(), xs[_nn][kt].Not()])
                        n_hand += 1
        if n_pair_c:
            log(f'  pages-first: {n_pair_c} pair constraint(s): each pair one face, one layer, neighbouring exits, at each end'
                + (f'; {n_hand} handedness clause(s)' if n_hand else ''))
    T = {n: m.NewIntVar(min(tkey[n] + jkey.get(n, [])), max(tkey[n] + jkey.get(n, [])), f'T_{n}') for n in names}
    L = {n: m.NewIntVar(min(lkey[n]), max(lkey[n]), f'L_{n}') for n in names}
    cost_terms = []
    for n in names:
        m.AddExactlyOne(xd[n])
        m.AddExactlyOne(xs[n])
        if hold_s is not None and hold_s.get(n) is not None and len(S[n]) == 2:
            m.Add(xs[n][1] == 1)
        if n in jkey and n in jflag and len(jflag[n]) == len(S[n]) and any(jflag[n]) \
                and any(a != b for a, b in zip(tkey[n], jkey[n])):
            # T = the port key, or the joined key when the chosen tooth is a
            # joiner: w_j = xd_j AND jn, jn = the joiner flag of the chosen tooth
            jn = sum(v for i, v in enumerate(xs[n]) if jflag[n][i])
            terms = []
            for j, v in enumerate(xd[n]):
                dlt = jkey[n][j] - tkey[n][j]
                terms.append(tkey[n][j] * v)
                if dlt:
                    w = m.NewBoolVar(f'w_{n}_{j}')
                    m.Add(w <= v); m.Add(w <= jn); m.Add(w >= v + jn - 1)
                    terms.append(dlt * w)
            m.Add(T[n] == sum(terms))
        else:
            jd = m.NewIntVar(0, len(D[n]) - 1, f'jd_{n}')
            m.Add(jd == sum(j * v for j, v in enumerate(xd[n])))
            m.AddElement(jd, tkey[n], T[n])
        if len(lkey[n]) > 1:
            js = m.NewIntVar(0, len(S[n]) - 1, f'js_{n}')
            m.Add(js == sum(i * v for i, v in enumerate(xs[n])))
            m.AddElement(js, lkey[n], L[n])
        else:
            m.Add(L[n] == lkey[n][0])
        # layers at the two ends, as linear expressions of the choice
        dB = sum(v for j, v in enumerate(xd[n]) if D[n][j].layer == 'B.Cu')
        if S[n]:
            tB = sum(v for i, v in enumerate(xs[n]) if S[n][i].layer == 'B.Cu')
        else:
            tB = 1 if st['tooth0'].get(n, 'F.Cu') == 'B.Cu' else 0
        # mismatch of each end with the page (xor), as bools
        mt = m.NewBoolVar(f'mt_{n}')
        md = m.NewBoolVar(f'md_{n}')
        for (b, e) in ((mt, tB), (md, dB)):
            m.Add(b >= e - pg[n]); m.Add(b >= pg[n] - e)
            m.Add(b <= e + pg[n]); m.Add(b <= 2 - e - pg[n])
        # the greedy's units, scaled: vias 3, channel 2, reach 1
        for j, mv in enumerate(D[n]):
            c = VIA_W * mv.vias + CHAN_W * sm._length(mv) + sm.around_box(launch[n], mv.exit_pt, dbox)
            cost_terms.append(int(round(c * SCALE)) * xd[n][j])
        if S[n]:
            for i, mv in enumerate(S[n]):
                c = VIA_W * mv.vias + (CHAN_W * sm._length(mv) if mv.legs else 0.0)
                cost_terms.append(int(round(c * SCALE)) * xs[n][i])
        else:
            cost_terms.append(int(round(VIA_W * st['tooth_vias'].get(n, 0) * SCALE)))
        cost_terms.append(int(round(VIA_W * PAGES_MISMATCH * SCALE)) * mt)
        cost_terms.append(int(round(VIA_W * PAGES_MISMATCH * SCALE)) * md)
        cost_terms.append(int(round(PAGES_SWIM * VIA_W * SCALE * (PAIR_SWIM if n in _pair_legs else 1.0))) * sw[n])
    # ---- the pages: no inversion inside a page
    npairs = 0
    for a_i in range(len(names)):
        a = names[a_i]
        for b in names[a_i + 1:]:
            if corr[a] != corr[b] or corr[a] == -1:
                continue                # different corridors never cross
            ltL = m.NewBoolVar(f'lL_{a}_{b}')
            m.Add(L[a] < L[b]).OnlyEnforceIf(ltL)
            m.Add(L[a] >= L[b]).OnlyEnforceIf(ltL.Not())
            ltT = m.NewBoolVar(f'lT_{a}_{b}')
            m.Add(T[a] < T[b]).OnlyEnforceIf(ltT)
            m.Add(T[a] >= T[b]).OnlyEnforceIf(ltT.Not())
            same = m.NewBoolVar(f'sm_{a}_{b}')
            m.Add(pg[a] == pg[b]).OnlyEnforceIf(same)
            m.Add(pg[a] != pg[b]).OnlyEnforceIf(same.Not())
            m.Add(ltL == ltT).OnlyEnforceIf([same, sw[a].Not(), sw[b].Not()])
            npairs += 1
    # ---- moves that cannot both be laid
    nconf = 0
    excl_d = []
    Sreal = {n: [mv for mv in S[n] if mv.legs] for n in names}
    idx_real = {n: [i for i, mv in enumerate(S[n]) if mv.legs] for n in names}
    excl_d = _conflicts(D, strict=bool(PAGES_STRICT))
    for (a, i, b, j) in excl_d:
        m.AddBoolOr([xd[a][i].Not(), xd[b][j].Not()]); nconf += 1
    for (a, i, b, j) in _conflicts(Sreal, strict=True):
        m.AddBoolOr([xs[a][idx_real[a][i]].Not(), xs[b][idx_real[b][j]].Not()]); nconf += 1
    if learned:
        sig_d = {n: [sr.move_sig(mv) for mv in D[n]] for n in names}
        # CANONICAL, and `key=repr` was NOT (review, 2026-09-16). `_conflicts`
        # sorts a set of TUPLES, whose repr is deterministic; `learned` is a
        # set of FROZENSETS, and a frozenset renders its elements in internal
        # hash order -- which for strings depends on PYTHONHASHSEED, pinned
        # NOWHERE in this chain. Measured: one two-element `learned` sorted
        # into two different orders across seeds 1..6. The exclusions below
        # are added to the model in this order, so two identical re-plan
        # passes could stop at different feasible points and lay different
        # copper from identical inputs -- the defect `_conflicts` documents
        # (obj 1784 / 1727 / 1809 on one model).
        for pair in sorted(learned, key=lambda p: sorted(p)):
            pair = list(pair)
            if len(pair) != 2:
                continue
            for a in names:
                for b in names:
                    if a >= b:
                        continue
                    for i, sa in enumerate(sig_d[a]):
                        for j, sb in enumerate(sig_d[b]):
                            if {sa, sb} == set(pair):
                                m.AddBoolOr([xd[a][i].Not(), xd[b][j].Not()]); nconf += 1
    if PAGES_HINT and seed:
        # the seed as a hint: its berth per net (the greedy's, or the cheapest
        # for a net it left unplaced) and the tooth as it stands; the pages
        # and the rest are the solver's to complete
        sig_seed = {n: sr.move_sig(seed[n]) for n in names if n in seed}
        for n in names:
            js_ = [j for j, mv in enumerate(D[n]) if sr.move_sig(mv) == sig_seed.get(n)]
            if js_:
                for j, v in enumerate(xd[n]):
                    m.AddHint(v, 1 if j == js_[0] else 0)
            if S[n] and cur.get(n) is not None:
                for i, v in enumerate(xs[n]):
                    m.AddHint(v, 1 if i == 0 else 0)
    m.Minimize(sum(cost_terms))
    if PAGES_DUMP:
        import json as _json
        os.makedirs(PAGES_DUMP, exist_ok=True)
        _solve.n_dump = getattr(_solve, 'n_dump', 0) + 1
        _stem = os.path.join(PAGES_DUMP, f'{os.path.splitext(os.path.basename(board))[0]}_solve{_solve.n_dump}')
        m.export_to_file(_stem + '.pb')
        with open(_stem + '.json', 'w', encoding='utf-8') as _f:
            _json.dump({'board': board, 'names': names, 'det': PAGES_DET, 'workers': PAGES_WORKERS,
                        'hint': bool(PAGES_HINT and seed), 'held': sorted(hold_s) if hold_s else [],
                        'n_berth': sum(len(v) for v in D.values()), 'n_tooth': sum(len(v) for v in S.values()),
                        'pairs': npairs, 'exclusions': nconf,
                        'via_w': VIA_W, 'chan_w': CHAN_W, 'swim': PAGES_SWIM, 'mismatch': PAGES_MISMATCH, 'scale': SCALE,
                        # per candidate (vias, channel length mm, reach mm): the offline
                        # reader splits any solution's objective into its terms
                        'berth_terms': {n: [(mv.vias, round(sm._length(mv), 4),
                                             round(sm.around_box(launch[n], mv.exit_pt, dbox), 4)) for mv in D[n]]
                                        for n in names},
                        'tooth_terms': {n: [(mv.vias, round(sm._length(mv), 4) if mv.legs else 0.0, 0.0) for mv in S[n]]
                                        for n in names},
                        'tooth0_vias': {n: st['tooth_vias'].get(n, 0) for n in names if not S[n]}}, _f, indent=1)
        log(f'  pages-first: instance written to {_stem}.pb')
    solver = cp_model.CpSolver()
    solver.parameters.num_workers = PAGES_WORKERS
    solver.parameters.interleave_search = True
    solver.parameters.max_deterministic_time = PAGES_DET
    rep = []
    status = _smemo.solve(m, solver, log=rep.append, label='pages-first')
    if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        rep.append(f'  pages-first: NO SOLUTION ({solver.StatusName(status)}) -- the greedy choice stands')
        if status == cp_model.INFEASIBLE and excl_d:
            # WHY (2026-09-15, session 9; log only, changes nothing): the
            # damped re-solve holds every non-swimmer at a one-move menu and
            # bars each swimmer from the berth that swam -- and a swimmer
            # whose every remaining candidate is excluded by some held berth
            # has no move at all, so the whole solve dies (K41: 132 of 251
            # re-solves across the session's logs). Name them.
            one = {n for n in names if len(D[n]) == 1}
            cut: Dict[str, set] = {}
            for (a, i, b, j) in excl_d:
                if b in one and a not in one:
                    cut.setdefault(a, set()).add(i)
                if a in one and b not in one:
                    cut.setdefault(b, set()).add(j)
            dead = [f'{n} {len(cut[n])}/{len(D[n])}' for n in sorted(cut) if len(cut[n]) >= len(D[n])]
            rep.append(f'  pages-first: infeasible re-solve: {len(one)} one-move menus, {len(dead)} freed net(s) with '
                       f'EVERY candidate excluded by a held berth: {dead}'
                       + ('' if dead else ' (the cause is elsewhere: a source exclusion, a learned pair or a key bound)'))
        return {}, {}, rep, {}
    dst_choice: Dict[str, Move] = {}
    src_choice: Dict[str, Move] = {}
    swim = []
    pages = {}
    vias = 0
    ff = 0
    for n in names:
        j = next(j for j, v in enumerate(xd[n]) if solver.Value(v))
        dst_choice[n] = D[n][j]
        pages[n] = 'B.Cu' if solver.Value(pg[n]) else 'F.Cu'
        if S[n]:
            i = next(i for i, v in enumerate(xs[n]) if solver.Value(v))
            if i > 0:
                src_choice[n] = S[n][i]
            tl, tv = S[n][i].layer, S[n][i].vias
        else:
            tl, tv = st['tooth0'].get(n, 'F.Cu'), st['tooth_vias'].get(n, 0)
        v = tv + D[n][j].vias + (tl != pages[n]) + (D[n][j].layer != pages[n])
        vias += v
        ff += (v == 0)
        if solver.Value(sw[n]):
            swim.append(n)
    # THE PLANNER'S VALUE OF EACH SOURCE MOVE it chose (read by
    # fanout_from_plan): the model cost the move saves for its own
    # net (tooth vias, the tooth/page mismatch) plus the inversions the
    # STANDING key would have with the net's same-page, non-swimming
    # partners under the chosen keys, each priced as a swimmer. Read off
    # the solution; changes nothing in it.
    value: Dict[str, float] = {}
    if src_choice:
        Lv = {n: solver.Value(L[n]) for n in names}
        Tv = {n: solver.Value(T[n]) for n in names}
        pgv = {n: solver.Value(pg[n]) for n in names}
        swv = {n: solver.Value(sw[n]) for n in names}
        for n in src_choice:
            i = next(k for k, mv in enumerate(S[n]) if mv is src_choice[n])
            c0 = VIA_W * S[n][0].vias + VIA_W * ((S[n][0].layer == 'B.Cu') != bool(pgv[n]))
            ci = (VIA_W * S[n][i].vias + CHAN_W * sm._length(S[n][i])
                  + VIA_W * ((S[n][i].layer == 'B.Cu') != bool(pgv[n])))
            inv = 0
            if corr[n] != -1 and not swv[n]:
                l0 = lkey[n][0]
                for b in names:
                    if b == n or corr[b] != corr[n] or pgv[b] != pgv[n] or swv[b]:
                        continue
                    if (l0 < Lv[b]) != (Tv[n] < Tv[b]):
                        inv += 1
            value[n] = round((c0 - ci) + PAGES_SWIM * VIA_W * inv, 3)
    import ortools as _ortools
    rep.append(f'  pages-first: {len(names)} nets, {sum(len(v) for v in D.values())} berth + '
               f'{sum(len(v) for v in S.values())} tooth candidates, {npairs} pairs, {nconf} exclusions; '
               f'{solver.StatusName(status)} obj {solver.ObjectiveValue() / SCALE:.1f} '
               f'bound {solver.BestObjectiveBound() / SCALE:.1f} in {time.time() - t0:.1f} s'
               f' (det {PAGES_DET:g}, {PAGES_WORKERS} workers, ortools {_ortools.__version__})')
    rep.append(f'  pages-first: model vias {vias} ({ff} nets at 0), pages F '
               f'{sum(1 for n in names if pages[n] == "F.Cu")} / B {sum(1 for n in names if pages[n] == "B.Cu")}, '
               f'swimmers {len(swim)} {swim if swim else ""}, teeth to move {len(src_choice)} {sorted(src_choice) if src_choice else ""}')
    if PAGES_LOG:
        for n in sorted(names, key=lambda n: solver.Value(L[n])):
            rep.append(f'     {n:6s} page {pages[n][0]}  L {solver.Value(L[n]) / 1000:7.2f}  T {solver.Value(T[n]) / 1000:7.2f}'
                       f'  tooth {sr.fmt_ask(src_choice[n]) if n in src_choice else "(as is)"}  berth {sr.fmt_ask(dst_choice[n])}')
    _solve.last_keys = {'L': {n: solver.Value(L[n]) for n in names}, 'T': {n: solver.Value(T[n]) for n in names},
                        'corr': dict(corr)}
    return dst_choice, src_choice, rep, {'vias': vias, 'swim': len(swim), 'ff': ff, 'pages': pages, 'value': value,
                                         'moved': [], 'status': solver.StatusName(status),
                                         'obj': solver.ObjectiveValue() / SCALE, 'cap': None}

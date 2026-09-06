"""Choose the escape at BOTH ends of the corridor, not just the berth.

The destination end has been a real decision for a while: a menu of
escape moves per ball, chosen against channel conflicts, side capacity,
the corridor's permutation and the crossings between corridors. The
source end was never a decision at all -- the launch point was read off
whatever stub the source fanout happened to lay, and the corridor's via
floor is the inversion count of the launch->exit permutation, so half
of it was an input nobody chose.

It is worth a lot. Holding the destination fixed and re-picking source
exits to minimise crossings, the whole-plan floor falls 16 -> 6 at K21,
38 -> 28 at K32 and 66 -> 56 at K51.

The two ends are the same problem with the roles swapped, so this needs
no new machinery: a corridor leg runs between two array boundaries, and
whichever end is being chosen, the other end's points are the fixed
side. select() is called twice per round -- destination against the
current source points, then source against the destination points it
just produced -- and iterated to a fixed point, keeping the best round.

The layer falls out rather than being read off the board: a chosen
source escape STARTS the net on a layer, so that escape's layer is the
tooth layer the destination end aligns against.
"""
from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

import select_moves as sm
from escape_moves import Move

Pt = Tuple[float, float]




SWIM_VIAS = 2      # a swimmer's dive and surface; its mid-corridor changes are the braid's


def plan_pages(dst_choice, launch, dst_box, cache, tooth_layer, buses):
    """The PAGES the braid will route on, decided here with the braid's
    own schedule code (schedule.Schedule) on the plan's launch and exit
    orders per corridor, with both ends' layers -- so there is one
    planner: the braid only verifies these in its own orders. Returns
    ({net: 'F.Cu' | 'B.Cu' | None}, {net: predicted vias}), the
    prediction per net = tooth-layer mismatch with the page + berth-layer
    mismatch + the berth escape's vias (a swimmer: SWIM_VIAS + berth vias)."""
    import schedule as sch
    geo = sm.Corridor(dst_box, launch, cache=cache)
    pages = {}
    pred = {}
    for grp in buses:
        grp = [n for n in grp if n in dst_choice]
        if not grp:
            continue
        if len(grp) == 1:
            n = grp[0]
            pages[n] = tooth_layer.get(n, 'F.Cu')
            pred[n] = ((1 if dst_choice[n].layer != pages[n] else 0)
                       + dst_choice[n].vias)
            continue
        t = geo.axis(grp, dst_choice)
        lo = geo.order(grp, dst_choice, t)
        li = {n: i for i, n in enumerate(lo)}
        tgt = sorted(grp, key=lambda n: (round(geo.exit_key(n, dst_choice[n], t), 6),
                                         li[n]))
        sc = sch.Schedule(lo, tgt, {n: tooth_layer.get(n, 'F.Cu') for n in grp},
                          dest_layer={n: dst_choice[n].layer for n in grp})
        for n in grp:
            pg = sc.page.get(n)
            pages[n] = pg
            if pg is None:
                pred[n] = SWIM_VIAS + dst_choice[n].vias
            else:
                pred[n] = ((1 if tooth_layer.get(n, 'F.Cu') != pg else 0)
                           + (1 if dst_choice[n].layer != pg else 0)
                           + dst_choice[n].vias)
    return pages, pred


def vias_from_pages(dst_choice, tooth_layer, tooth_vias, pages, leg_layer=None,
                    changes=None, cross=None):
    """Per-net vias implied by a page assignment ({net: layer | None}):
    tooth vias + the lane's layer CHANGES + berth vias. `changes` is the
    braid planner's count over the lane's whole profile (tooth, page,
    the tail stretches under same-layer exit legs, the exit leg, the
    berth; braid.Corridor.layer_profile); `cross` its dives under
    earlier corridors' lanes (braid.cross_corridor_vias), which no page
    sees. Without `changes` the profile is tooth -> page -> (leg) ->
    berth: tooth/page mismatch + the arrival -- straight into the berth:
    page/berth mismatch; through a side-exit leg on `leg_layer`: page/leg
    + leg/berth mismatch. A swimmer: tooth vias + SWIM_VIAS + berth vias."""
    pred = {}
    for n, m in dst_choice.items():
        pg = pages.get(n)
        tv = tooth_vias.get(n, 0) + (cross or {}).get(n, 0)
        if pg is None:
            pred[n] = tv + SWIM_VIAS + m.vias
            continue
        ch = (changes or {}).get(n)
        if ch is not None:
            pred[n] = tv + ch + m.vias
            continue
        leg = (leg_layer or {}).get(n)
        if leg:
            arrive = (1 if leg != pg else 0) + (1 if m.layer != leg else 0)
        else:
            arrive = 1 if m.layer != pg else 0
        pred[n] = (tv + (1 if tooth_layer.get(n, 'F.Cu') != pg else 0)
                   + arrive + m.vias)
    return pred


def judged_cost(dst_choice, launch, dst_box, cache, src_box,
                tooth_layer, tooth_vias, buses=None):
    """What a plan is judged on: the VIAS it implies, in the plan's own
    model -- per net a dive if the corridor cannot keep it on its tooth
    layer, a via where the delivered layer is not the berth escape's, the
    berth escape's own vias (select_moves.true_vias) -- PLUS the source
    escape's own vias (a dog-bone or via-in-pad tooth is a via the
    crossing floor never saw: at K4 the floor-only objective sent SDQ15
    out the north face and SDQ11 out the WEST face on dog-bones to buy
    a crossing-free order, 4 -> 6 realized vias), plus the ride round
    both arrays at VIA_MM per via."""
    # the vias per net come from the PAGES the braid's own schedule code
    # assigns on the plan's orders (plan_pages), judged within the
    # corridors the braid will form (buses = planned_buses); the source
    # escape's vias and the ride round both arrays are added
    groups = list(buses) if buses else sm.corridor_groups(dst_choice)
    _pages, pred = plan_pages(dst_choice, launch, dst_box, cache, tooth_layer, groups)
    return (sum(pred.values())
            + sum(tooth_vias.get(n, 0) for n in dst_choice)
            + sm.ride_mm(dst_choice, launch, dst_box, src_box) / sm.VIA_MM)


def refine_source(src_choice: Dict[str, Move],
                  src_menu: Dict[str, List[Move]],
                  dst_choice: Dict[str, Move],
                  dst_box, launch0: Dict[str, Pt],
                  rounds: int = 5, cache=None, log=None,
                  src_box=None, tooth_layer0=None, tooth_vias0=None,
                  buses=None):
    """Move source exits to cut the WHOLE-PLAN cost, one net at a time.

    select() at the source end optimises select()'s cost -- reach,
    vias, channel length, crossings among the legs placed so far -- and
    that is not the corridor's floor. Run alone it made things worse
    (16 -> 18 -> 20 at K21), because its `reach` term pulls every source
    exit toward its own destination, which is exactly the greedy that
    tangles the permutation.

    So the cost is optimised against DIRECTLY here, with the channel
    check kept, which is the difference between this and the bound the
    probe measured: a move is only taken if its lane and via site are
    still free.

    The cost is what the braid SPENDS (select_moves.true_vias, with the
    source escape's own vias added), not the crossing floor alone: a
    tooth the fanout put on B that the corridor delivers on F costs the
    braid a via the floor never saw (K15 SRAS). And a move keeps its
    tooth on the side the fanout already used: optimising the floor
    alone sent four of K15's E-face teeth out the north flank, which the
    braid then paid for as side joiners of a second corridor (2 open
    lanes where the unmoved teeth gave none). The layer -- a surface
    escape or a dogbone in the same direction -- is the degree of
    freedom here; the side is the corridor plan's, not this pass's.
    """
    cache = cache if cache is not None else {}
    launch = dict(launch0)
    tl = dict(tooth_layer0 or {})
    tv = dict(tooth_vias0 or {})
    for n, m in src_choice.items():
        launch[n] = m.exit_pt
        tl[n] = m.layer
        tv[n] = m.vias

    def cost(launch_c, tl_c, tv_c):
        return judged_cost(dst_choice, launch_c, dst_box, cache, src_box,
                           tl_c, tv_c, buses)
    cur = cost(launch, tl, tv)
    best = (cur, dict(launch), dict(src_choice))
    for r in range(rounds):
        moved = 0
        for n in sorted(src_menu, key=lambda n: -len(src_menu[n])):
            if n not in dst_choice:
                continue
            for m in src_menu[n]:
                if m.exit_pt == launch.get(n) and m.layer == tl.get(n):
                    continue
                if not sm.lanes_free(m, src_choice, n, strict=False):
                    continue
                cand = dict(launch)
                cand[n] = m.exit_pt
                ctl = dict(tl)
                ctl[n] = m.layer
                ctv = dict(tv)
                ctv[n] = m.vias
                f = cost(cand, ctl, ctv)
                if f <= cur:
                    if f < cur:
                        moved += 1
                    cur, launch, tl, tv = f, cand, ctl, ctv
                    src_choice[n] = m
                    if cur < best[0]:
                        best = (cur, dict(launch), dict(src_choice))
        if log:
            log(f'    source refine round {r}: floor {cur} (best {best[0]})')
        if not moved:
            break
    return best[2], best[1], best[0]


def plan_ends(src_menu: Dict[str, List[Move]],
              dst_menu: Dict[str, List[Move]],
              launch0: Dict[str, Pt],
              src_box, dst_box,
              buses: Optional[Sequence[Sequence[str]]] = None,
              tooth_layer0: Optional[Dict[str, str]] = None,
              rounds: int = 4,
              log=None,
              src_seed: Optional[Dict[str, Move]] = None,
              pads=None):
    """Returns (src_choice, dst_choice, launch, report). `objective`:
    'floor' (the crossing floor, the plan's original) or 'spend' (what
    the braid spends, for a chain that APPLIES the source moves --
    SOURCE=1); see refine_source.

    (An order model once sat here -- plan_order.BraidOrder -- and was retired. With
    it the floor, the LIS refinement and the alignment run on the
    order the braid will lay, a face-refinement pass moves nets
    between faces while the schedule's (floor, columns over capacity,
    columns) falls, and the source refinement is skipped unless the
    source moves are applied (SOURCE=1): its launch points were never
    on the board, and the destination was being chosen against them.

    `launch0` is where the source stubs end today -- the seed, and the
    fallback for any net whose source pad is boxed in. A net with no
    source move keeps its existing tooth rather than being dropped:
    the source end is an improvement, not a precondition. `src_seed`
    describes those existing stubs AS MOVES (kind, gap, site), so the
    lane checks see the copper of every net the refinement leaves
    alone -- without it a new escape was planned through the gap an
    unmoved neighbour's stub still occupies (K15 SDQ15 over SDQM0).
    """
    launch = dict(launch0)
    tooth = dict(tooth_layer0 or {})
    cache: Dict[str, Dict] = {}
    best = None
    report: List[str] = []
    src_choice: Dict[str, Move] = {}

    def total(dst_c, launch_c):
        """What the plan is judged on: the crossing floor plus the ride
        length at VIA_MM per via."""
        geo = sm.Corridor(dst_box, launch_c, cache=cache)
        return (sm.plan_floor(dst_c, geo)
                + sm.ride_mm(dst_c, launch_c, dst_box) / sm.VIA_MM)
    for r in range(rounds):
        dst_choice, un = sm.select(dst_menu, launch, keep_out=dst_box,
                                   buses=buses, tooth_layer=tooth, log=log,
                                   pads=pads)
        if not dst_choice:
            break
        f = total(dst_choice, launch)
        line = (f'  round {r}: floor {f}, {len(dst_choice)} placed'
                + (f', {len(un)} unplaced' if un else ''))
        if best is None or f < best[0]:
            best = (f, dict(src_choice), dict(dst_choice), dict(launch),
                    dict(tooth))
            line += '   <- best'
        report.append(line)
        if log:
            log(line)

        dst_pts = {n: m.exit_pt for n, m in dst_choice.items()}
        sub = {n: ms for n, ms in src_menu.items() if n in dst_pts and ms}
        if not sub:
            break
        src_choice, nxt, sf = refine_source(
            {}, sub, dst_choice, dst_box, launch0, cache=cache, log=log)
        tooth_r = dict(tooth_layer0 or {})
        for n, m in src_choice.items():
            tooth_r[n] = m.layer
        if sf < best[0]:
            best = (sf, dict(src_choice), dict(dst_choice), dict(nxt),
                    dict(tooth_r))
            report.append(f'  round {r}: source refine -> floor {sf}   <- best')
            if log:
                log(report[-1])
        tooth = tooth_r
        if nxt == launch:
            break
        launch = nxt
    if best is None:
        return {}, {}, launch, report
    f, sc, dc, lp, tl = best
    report.append(f'  kept floor {f}')
    return sc, dc, lp, report

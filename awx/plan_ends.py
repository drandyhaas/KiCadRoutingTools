"""The plan's own via model on a destination choice: the PAGES the braid's
schedule gives the plan's launch and exit orders per corridor
(plan_pages), and the vias a plan implies in that model plus the ride
round both arrays (judged_cost: fanout_from_plan's fast proxy on two
routing layers).
"""
from __future__ import annotations

import select_moves as sm


# SWIM_VIAS: what the judge charges for a lane the schedule gives no page.
# It was 2 -- a dive and a surface -- which is LESS than a scheduled lane
# really costs, so the search was rewarded for accepting swimmers. What a
# swimmer actually costs, measured per net on routed boards: K35 2.50,
# K41 3.08, K51 3.74 against scheduled 1.90 / 2.13 / 2.29, and against
# the plan's own prediction the flat 2 misses by +2.92 (K41) and +1.55
# (K51) per swimmer. Two other prices were tried and are WORSE: the
# braid's swim_changes line (over by 3.31 / 1.73, larger spread) and the
# DP floor of the planned lane (no better, sd worse) -- a swimmer has no
# planned path, so its cost is not predictable from the plan's geometry.
# A calibrated CONSTANT is therefore the honest model, and it makes the
# planner prefer orders that need fewer swimmers instead of orders that
# hide them.
import prices as _pr
SWIM_VIAS = _pr.JUDGE          # ONE source: prices.py


def plan_pages(dst_choice, launch, dst_box, cache, tooth_layer, buses, chi: int = 1):
    """`_plan_pages` in the pair's canonical frame (select_moves.PairFrame):
    a -1 pair's choice, launches and box are mirrored in; the pages and
    the prediction are per net and come back as they are."""
    if chi > 0:
        return _plan_pages(dst_choice, launch, dst_box, cache, tooth_layer, buses)
    fr = sm.PairFrame(-1, sm.frame_line(launch, dst_box))
    mc = cache.setdefault('_mirror', {}) if cache is not None else None
    return _plan_pages(fr.choice(dst_choice), fr.points(launch), fr.box(dst_box), mc,
                       fr.layers(tooth_layer), buses)


def _plan_pages(dst_choice, launch, dst_box, cache, tooth_layer, buses):
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


def judged_cost(dst_choice, launch, dst_box, cache, src_box,
                tooth_layer, tooth_vias, buses=None, chi: int = 1):
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
    _pages, pred = plan_pages(dst_choice, launch, dst_box, cache, tooth_layer, groups, chi=chi)
    return (sum(pred.values())
            + sum(tooth_vias.get(n, 0) for n in dst_choice)
            + sm.ride_mm(dst_choice, launch, dst_box, src_box) / sm.VIA_MM)



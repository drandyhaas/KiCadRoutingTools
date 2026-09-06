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


def _snap_dir(v: Pt) -> str:
    dirs = {'right': (1, 0), 'left': (-1, 0), 'up': (0, -1), 'down': (0, 1)}
    h = (v[0] ** 2 + v[1] ** 2) ** 0.5 or 1.0
    return min(dirs, key=lambda k: (dirs[k][0] - v[0] / h) ** 2
               + (dirs[k][1] - v[1] / h) ** 2)


def refine_source(src_choice: Dict[str, Move],
                  src_menu: Dict[str, List[Move]],
                  dst_choice: Dict[str, Move],
                  dst_box, launch0: Dict[str, Pt],
                  rounds: int = 5, cache=None, log=None,
                  ):
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
    for n, m in src_choice.items():
        launch[n] = m.exit_pt

    def cost(launch_c):
        geo = sm.Corridor(dst_box, launch_c, cache=cache)
        return (sm.plan_floor(dst_choice, geo)
                + sm.ride_mm(dst_choice, launch_c, dst_box) / sm.VIA_MM)
    cur = cost(launch)
    best = (cur, dict(launch), dict(src_choice))
    for r in range(rounds):
        moved = 0
        for n in sorted(src_menu, key=lambda n: -len(src_menu[n])):
            if n not in dst_choice:
                continue
            for m in src_menu[n]:
                if m.exit_pt == launch.get(n):
                    continue
                if not sm.lanes_free(m, src_choice, n, strict=False):
                    continue
                cand = dict(launch)
                cand[n] = m.exit_pt
                f = cost(cand)
                if f <= cur:
                    if f < cur:
                        moved += 1
                    cur, launch = f, cand
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

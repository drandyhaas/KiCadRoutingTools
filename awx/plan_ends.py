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
                  tooth_layer: Optional[Dict[str, str]] = None):
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
    tooth = dict(tooth_layer or {})
    launch = dict(launch0)
    for n, m in src_choice.items():
        launch[n] = m.exit_pt
        tooth[n] = m.layer
    # the seed's own vias are the board's, not a cost of this pass:
    # count only vias a CHANGED move adds over the seed's
    seed_vias = {n: m.vias for n, m in src_choice.items()}
    # the side each tooth already escapes on: pad -> tooth, snapped
    side0 = {}
    for n, ms in src_menu.items():
        if ms and n in launch0:
            pad = ms[0].legs[0][0]
            side0[n] = _snap_dir((launch0[n][0] - pad[0], launch0[n][1] - pad[1]))

    def cost(launch_c, tooth_c, src_c):
        geo = sm.Corridor(dst_box, launch_c, cache=cache)
        tv = sm.true_vias(dst_choice, sm.corridors(dst_choice), geo, tooth_c)
        return tv + sum(m.vias - seed_vias.get(n, 0) for n, m in src_c.items())
    cur = cost(launch, tooth, src_choice)
    best = (cur, dict(launch), dict(src_choice))
    for r in range(rounds):
        moved = 0
        for n in sorted(src_menu, key=lambda n: -len(src_menu[n])):
            if n not in dst_choice:
                continue
            for m in src_menu[n]:
                if m.exit_pt == launch.get(n) and m.layer == tooth.get(n):
                    continue
                if n in side0 and m.direction != side0[n]:
                    continue
                if not sm.lanes_free(m, src_choice, n):
                    continue
                cand = dict(launch)
                cand[n] = m.exit_pt
                tooth_c = dict(tooth)
                tooth_c[n] = m.layer
                src_c = dict(src_choice)
                src_c[n] = m
                f = cost(cand, tooth_c, src_c)
                # ties are TAKEN, not just strict improvements. Reaching
                # a better source order needs several nets to move
                # together, so a strict hill climb sits on the first
                # plateau it meets -- the single-pass version of this
                # search reported 0 teeth worth moving where a
                # plateau-walking one moved 19 and halved the floor.
                if f <= cur:
                    if f < cur:
                        moved += 1
                    cur, launch, tooth = f, cand, tooth_c
                    src_choice[n] = m
                    if cur < best[0]:
                        best = (cur, dict(launch), dict(src_choice))
                    # deliberately NO break: keep walking this net's
                    # remaining moves. Stopping at the first acceptable
                    # one pins the net to wherever the plateau was first
                    # entered, and the plateau is the whole point --
                    # breaking here found 0 improvements at K21 where
                    # walking on finds the floor halved.
        if log:
            log(f'    source refine round {r}: true vias {cur} '
                f'(best {best[0]})')
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
              src_seed: Optional[Dict[str, Move]] = None):
    """Returns (src_choice, dst_choice, launch, report).

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

    def total(dst_c, launch_c, tooth_c, src_c):
        """What the braid spends: true vias for the destination choice
        given the teeth's layers, plus the source escapes' own vias.
        Both ends are judged on this one number."""
        geo = sm.Corridor(dst_box, launch_c, cache=cache)
        return (sm.true_vias(dst_c, sm.corridors(dst_c), geo, tooth_c)
                + sum(m.vias for m in src_c.values()))
    for r in range(rounds):
        dst_choice, un = sm.select(dst_menu, launch, keep_out=dst_box,
                                   buses=buses, tooth_layer=tooth)
        if not dst_choice:
            break
        geo = sm.Corridor(dst_box, launch, cache=cache)
        fl = sm.plan_floor(dst_choice, geo)
        f = total(dst_choice, launch, tooth, src_choice)
        line = (f'  round {r}: true vias {f} (floor {fl}), '
                f'{len(dst_choice)} placed'
                + (f', {len(un)} unplaced' if un else ''))
        if best is None or f < best[0]:
            best = (f, dict(src_choice), dict(dst_choice), dict(launch),
                    dict(tooth))
            line += '   <- best'
        report.append(line)
        if log:
            log(line)

        # now the other end: the destination exits are the fixed side,
        # and the source escapes are what is being chosen
        dst_pts = {n: m.exit_pt for n, m in dst_choice.items()}
        sub = {n: ms for n, ms in src_menu.items() if n in dst_pts and ms}
        if not sub:
            break
        # Start from the fanout ALREADY ON THE BOARD and change a tooth
        # only when moving it helps. Two seeds were tried first and both
        # are worse than doing nothing:
        #
        #   a fresh select() at the source     16 -> 18 -> 20 at K21.
        #     select() minimises reach, which pulls every source exit
        #     toward its own destination -- precisely the greedy that
        #     tangles the permutation this is trying to untangle.
        #   the menu move NEAREST each existing tooth    seeds at 20.
        #     The teeth sit a median 0.09 mm from an offered exit, so
        #     this looks like a no-op and is not: the nets whose nearest
        #     move loses its lane fall back to the old tooth while the
        #     rest move, and the mixture is worse than either.
        #
        # An empty start means launch == launch0 exactly, so the search
        # begins at the board's own floor and can only leave it for
        # something measured better.
        src_choice, nxt, sf = refine_source(dict(src_seed or {}), sub,
                                            dst_choice, dst_box, launch0,
                                            cache=cache, log=log,
                                            tooth_layer=tooth_layer0)
        # Record the REFINED pair, not just the state the round opened
        # with. The source refinement optimises against this round's
        # destination choice, and re-selecting the destination next
        # round gives some of it back -- so the best pair seen is
        # routinely mid-round, and scoring only at the top of the loop
        # threw it away (floor 12 reached and reported as 14).
        tooth_r = dict(tooth_layer0 or {})
        for n, m in src_choice.items():
            tooth_r[n] = m.layer
        if sf < best[0]:
            best = (sf, dict(src_choice), dict(dst_choice), dict(nxt),
                    dict(tooth_r))
            report.append(f'  round {r}: source refine -> true vias {sf}'
                          f'   <- best')
            if log:
                log(report[-1])
        tooth = tooth_r
        if nxt == launch:
            break
        launch = nxt
    if best is None:
        return {}, {}, launch, report
    f, sc, dc, lp, tl = best
    geo = sm.Corridor(dst_box, lp, cache=cache)
    report.append(f'  kept true vias {f} (floor {sm.plan_floor(dc, geo)})')
    return sc, dc, lp, report

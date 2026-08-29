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


def refine_source(src_choice: Dict[str, Move],
                  src_menu: Dict[str, List[Move]],
                  dst_choice: Dict[str, Move],
                  dst_box, launch0: Dict[str, Pt],
                  rounds: int = 5, cache=None, log=None):
    """Move source exits to cut the WHOLE-PLAN floor, one net at a time.

    select() at the source end optimises select()'s cost -- reach,
    vias, channel length, crossings among the legs placed so far -- and
    that is not the corridor's floor. Run alone it made things worse
    (16 -> 18 -> 20 at K21), because its `reach` term pulls every source
    exit toward its own destination, which is exactly the greedy that
    tangles the permutation.

    So the floor is optimised against DIRECTLY here, with the channel
    check kept, which is the difference between this and the bound the
    probe measured: a move is only taken if its lane and via site are
    still free.
    """
    cache = cache if cache is not None else {}
    launch = dict(launch0)
    for n, m in src_choice.items():
        launch[n] = m.exit_pt
    cur = sm.plan_floor(dst_choice,
                        sm.Corridor(dst_box, launch, cache=cache))
    best = (cur, dict(launch), dict(src_choice))
    for r in range(rounds):
        moved = 0
        for n in sorted(src_menu, key=lambda n: -len(src_menu[n])):
            if n not in dst_choice:
                continue
            for m in src_menu[n]:
                if m.exit_pt == launch.get(n):
                    continue
                if not sm.lanes_free(m, src_choice, n):
                    continue
                cand = dict(launch)
                cand[n] = m.exit_pt
                f = sm.plan_floor(dst_choice,
                                  sm.Corridor(dst_box, cand, cache=cache))
                # ties are TAKEN, not just strict improvements. Reaching
                # a better source order needs several nets to move
                # together, so a strict hill climb sits on the first
                # plateau it meets -- the single-pass version of this
                # search reported 0 teeth worth moving where a
                # plateau-walking one moved 19 and halved the floor.
                if f <= cur:
                    if f < cur:
                        moved += 1
                    cur, launch = f, cand
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
            log(f'    source refine round {r}: floor {cur} '
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
              log=None):
    """Returns (src_choice, dst_choice, launch, report).

    `launch0` is where the source stubs end today -- the seed, and the
    fallback for any net whose source pad is boxed in. A net with no
    source move keeps its existing tooth rather than being dropped:
    the source end is an improvement, not a precondition.
    """
    launch = dict(launch0)
    tooth = dict(tooth_layer0 or {})
    cache: Dict[str, Dict] = {}
    best = None
    report: List[str] = []
    src_choice: Dict[str, Move] = {}
    for r in range(rounds):
        dst_choice, un = sm.select(dst_menu, launch, keep_out=dst_box,
                                   buses=buses, tooth_layer=tooth)
        if not dst_choice:
            break
        geo = sm.Corridor(dst_box, launch, cache=cache)
        f = sm.plan_floor(dst_choice, geo)
        line = (f'  round {r}: floor {f}, {len(dst_choice)} placed'
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
        src_choice, nxt, sf = refine_source({}, sub, dst_choice,
                                            dst_box, launch0,
                                            cache=cache, log=log)
        # Record the REFINED pair, not just the state the round opened
        # with. The source refinement optimises against this round's
        # destination choice, and re-selecting the destination next
        # round gives some of it back -- so the best pair seen is
        # routinely mid-round, and scoring only at the top of the loop
        # threw it away (floor 12 reached and reported as 14).
        if sf < best[0]:
            best = (sf, dict(src_choice), dict(dst_choice), dict(nxt),
                    dict(tooth))
            report.append(f'  round {r}: source refine -> floor {sf}'
                          f'   <- best')
            if log:
                log(report[-1])
        for n, m in src_choice.items():
            tooth[n] = m.layer
        if nxt == launch:
            break
        launch = nxt
    if best is None:
        return {}, {}, launch, report
    f, sc, dc, lp, tl = best
    report.append(f'  kept floor {f}')
    return sc, dc, lp, report

#!/usr/bin/env python3
"""CUT-CAPACITY LEDGER for a corridor: is there provably enough space?

The classical realizability fact (Maley/Cole-Siegel single-layer
routing): a set of homotopy classes is geometrically routable iff every
CUT has flow <= capacity -- the tracks crossing it, widths plus
clearances, fit in its free length. This tool measures exactly that for
the braid's plan, BEFORE any lane is routed: at each station s along
the spine, on each layer, the free cross-section width (static copper
only, the members' own stubs excluded) versus the width the scheduled
lanes need there (track + clearance each; a swimmer loads BOTH layers
-- it must be able to weave; page lanes load their page).

A cut whose load exceeds its capacity names the nets that must change
something structural THERE (another layer, another face, a wider
corridor); a plan whose every cut fits cannot be refused for space --
only for search -- which is the guarantee worth building toward.

usage: cut_ledger.py FANOUT_BOARD K [--dest DU1] [--step 0.2]
       [--net NET: corridor containing NET; default the biggest]
"""
import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
import numpy as np  # noqa: E402
import braid as te  # noqa: E402
from schedule import Schedule  # noqa: E402

TRACK, CLEAR = te.TRACK, te.CLEAR
NEED = TRACK + CLEAR           # one lane's slice of a cut


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('k', type=int)
    ap.add_argument('--dest', default='DU1')
    ap.add_argument('--step', type=float, default=0.2)
    ap.add_argument('--net', default=None)
    ap.add_argument('--o-span', type=float, default=None,
                    help='half-width of the cut in o (default: from the lanes)')
    a = ap.parse_args()
    nets = subprocess.run([sys.executable,
                           os.path.join(HERE, 'coherent_nets.py'), str(a.k)],
                          capture_output=True, text=True).stdout.strip()
    names = [n for n in nets.split(',') if n]
    quiet = lambda *x: None  # noqa: E731
    ctx, groups = te.setup(a.board, names, a.dest, quiet)
    grp = (next(g for g in groups if a.net in g) if a.net
           else max(groups, key=len))
    c = te.Corridor(0, grp, ctx, quiet)
    c.build_spine()
    c.classify()
    c.offsets(0.35)
    c.reserve_intervals()
    sched = Schedule(c.launch, c.target, ctx.tooth_layer,
                     dest_layer=ctx.dest_layer,
                     pages=getattr(ctx, 'pages', None))
    cols, _gate = c.plan_columns(sched, {d: 1 for d in sched.divers},
                                 {d: 0 for d in sched.divers})
    c.lay_lanes(cols)
    sp = c.spine
    two = getattr(sched, 'two_page', False)
    print(f'corridor [{len(grp)}] {",".join(grp)}')
    if two:
        print(f'  pages F {sum(1 for n in grp if sched.page.get(n) == "F.Cu")}'
              f' / B {len(sched.b_page)} / swimmers {len(sched.swimmers)}')

    # static copper per layer, every member's own segments excluded
    obs = {L: ctx.obs_but(grp[0], grp, L) for L in ('F.Cu', 'B.Cu')}

    # o range: from the lanes themselves, with a margin
    o_all = [o for nm in grp for (_s, o) in c.mid[nm]]
    o_lo = min(o_all) - 0.8 if a.o_span is None else -a.o_span
    o_hi = max(o_all) + 0.8 if a.o_span is None else a.o_span

    def cap_at(s, L):
        """Free width of the cut at station s on layer L: sum of clear
        o-intervals, sampled at 0.05 mm, minus a clearance at each free
        interval's obstacle-touching ends (a track centreline must stay
        CLEAR + TRACK/2 off copper -- the obstacle model is inflated by
        exactly that, so a clear sample IS a legal centreline; the sum
        of clear samples x step approximates legal centreline room, and
        each maximal run holds floor(run / NEED) + 1 lanes' centres)."""
        step_o = 0.05
        os_ = np.arange(o_lo, o_hi + step_o, step_o)
        free = 0
        run = 0
        lanes_fit = 0
        for o in os_:
            x, y = sp.xy(s, o)
            if obs[L].point_violation((x, y)) is None:
                run += 1
                free += 1
            else:
                if run:
                    lanes_fit += int((run - 1) * step_o / NEED) + 1
                run = 0
        if run:
            lanes_fit += int((run - 1) * step_o / NEED) + 1
        return free * step_o, lanes_fit

    def load_at(s, L):
        """Lanes the schedule puts through the cut on layer L."""
        out = []
        for nm in grp:
            ms = [p[0] for p in c.mid[nm]]
            if not (ms[0] - 1e-9 <= s <= ms[-1] + 1e-9):
                continue
            if two:
                P = sched.page.get(nm)
                if P is not None and P != L:
                    continue
                if P is None and not c.allowed(nm, s, L):
                    continue
            elif not c.allowed(nm, s, L):
                continue
            out.append(nm)
        return out

    # tooth to stub, NOT s0..s1: the schedule region between the two
    # arrays is open field (first run: capacity 64 lanes, load 20,
    # everywhere) -- the binding cuts are in the ESCAPE FIELDS the
    # lanes thread past s1 and before s0, which is where every refusal
    # this campaign has ever seen actually stands
    s0 = min(c.mid[nm][0][0] for nm in grp)
    s1 = max(c.mid[nm][-1][0] for nm in grp)
    print(f'  s {s0:.2f}..{s1:.2f} (schedule region {c.s0:.2f}..{c.s1:.2f}),'
          f' cut span o {o_lo:.2f}..{o_hi:.2f}')
    print(f'  {"s":>6} {"xy":>16}  {"capF":>6} {"fitF":>4} {"loadF":>5} '
          f'{"capB":>6} {"fitB":>4} {"loadB":>5}  over')
    worst = []
    for s in np.arange(s0, s1 + a.step, a.step):
        capF, fitF = cap_at(s, 'F.Cu')
        capB, fitB = cap_at(s, 'B.Cu')
        lF = load_at(s, 'F.Cu')
        lB = load_at(s, 'B.Cu')
        overF = len(lF) > fitF
        overB = len(lB) > fitB
        x, y = sp.xy(s, 0.0)
        mark = ('F!' if overF else '') + ('B!' if overB else '')
        print(f'  {s:6.2f} ({x:6.2f},{y:6.2f})  {capF:6.2f} {fitF:4d} '
              f'{len(lF):5d} {capB:6.2f} {fitB:4d} {len(lB):5d}  {mark}')
        if overF:
            worst.append((s, 'F.Cu', len(lF) - fitF, lF))
        if overB:
            worst.append((s, 'B.Cu', len(lB) - fitB, lB))
    if worst:
        print('\nOVER-CAPACITY cuts (s, layer, excess, nets):')
        for s, L, ex, lanes in sorted(worst, key=lambda w: -w[2])[:12]:
            print(f'  s={s:.2f} {L} over by {ex}: {",".join(lanes)}')
    else:
        print('\nevery cut fits: the plan is not refusable for cross-'
              'section space on the spine stations sampled')


if __name__ == '__main__':
    main()

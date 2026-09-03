#!/usr/bin/env python3
"""#622 JOINT ride assignment over a crossing group (brief item 6, the
first concrete step).

Where K35's +8 to the human lives (0902): SDQ7 pays 4 layer changes
over 13 crossings where the human pays 0 over 5, and six more nets
pay +2 -- the human rides the data group on a constant layer with
every partner opposite. No one- or two-net move reaches that (measured
flat: stub moves, pairs, surgical page flips), because with the
partners frozen a net's ride is dictated by where they sit at each
crossing. The move is JOINT: the pages of a whole crossing group.

Model (consistent, milliseconds): every group net rides ONE page at
all its crossings; its cost = (start pad != page) + (end pad != page)
+ 2 per crossing whose partner presents the same layer (a dive under
it). A non-group net keeps the judge's DP with its partners' layers
(group partners at their assigned page, others as routed). Enumerate
2^|G| assignments, rank by total floor, compare with the CURRENT
assignment scored the same way, realize the best few: strip the
group's lanes from the record (stubs from the fanout board), braid
the group TOGETHER with its pages pinned through the sidecar, grade,
judge, keep lexicographically better (open, drc, vias).

usage: group_pages.py TAG BEST FO K NET [--max-group 12] [--try 4]
"""
import argparse
import itertools
import os
import shutil
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.chdir(HERE)
from ledger_cal import Judge, _dp  # noqa: E402
import surgical as sg              # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('tag')
ap.add_argument('best')
ap.add_argument('fo')
ap.add_argument('k')
ap.add_argument('net')
ap.add_argument('--max-group', type=int, default=12)
ap.add_argument('--try', dest='ntry', type=int, default=4)
ap.add_argument('--src', default='U1')
ap.add_argument('--dst', default='DU1')
a = ap.parse_args()
NETS = sg.k_nets(a.k)
nets = NETS.split(',')
T0 = time.time()


def say(msg):
    print(f'[{time.time() - T0:6.0f}s] {msg}', flush=True)


J = Judge(a.best, nets, a.src, a.dst)
g0 = sg.grade(a.best, NETS)
say(f'start: grade {g0} floor {J.floor_total} changes {J.act_total}')

# ---- the group: the net plus its crossing partners, most-crossed first
seed = a.net
partners = {}
for (_t, o, _pl) in J.seqs[seed]:
    partners[o] = partners.get(o, 0) + 1
G = [seed] + sorted(partners, key=lambda o: -partners[o])
G = G[:a.max_group]
say(f'group ({len(G)}): {G}')


def ride_layer(m):
    """The layer the net presents at most of its crossings today."""
    P = J.paths[m]
    cnt = {}
    for (t, _o, _pl) in J.seqs[m]:
        L = P.layer_at(t)
        cnt[L] = cnt.get(L, 0) + 1
    if not cnt:
        return P.lay[len(P.lay) // 2]
    return max(cnt, key=cnt.get)


current = {}          # all free = the board as routed


def score(assign):
    """Total floor under the joint assignment (group nets on their
    pages, others by DP)."""
    total = 0
    for m in J.paths:
        P = J.paths[m]
        if m in assign:
            pg = assign[m]
            c = (P.start_pad_lay != pg) + (P.end_pad_lay != pg)
            for (t, o, pl) in J.seqs[m]:
                opl = assign.get(o, pl)
                if opl == pg:
                    c += 2
            total += c
        else:
            seq = [(t, assign.get(o, pl)) for (t, o, pl) in J.seqs[m]]
            total += _dp(P, seq)
    return total


base = score(current)
say(f'current assignment scores {base} (judge floor {J.floor_total})')
# three states per group net: pinned F, pinned B, or FREE (as routed:
# the judge's DP against its partners' assigned layers, presenting its
# actual layers) -- all-free reproduces the judge floor exactly, so a
# score under it is a real predicted drop, not a model offset
ranked = []
for bits in itertools.product(('F.Cu', 'B.Cu', None), repeat=len(G)):
    assign = {m: b for m, b in zip(G, bits) if b is not None}
    ranked.append((score(assign), assign))
ranked.sort(key=lambda r: r[0])
say(f'{len(ranked)} assignments scored in {time.time() - T0:.1f}s; '
    f'best {ranked[0][0]}, current {base}')
for s, asg in ranked[:5]:
    flips = [f'{m}:{asg[m][0]}' for m in G if m in asg]
    say(f'  {s}: flips {flips}')

# ---- realize the best few that beat the current score
best, gb, Jb = a.best, g0, J
tried = 0
for s, asg in ranked:
    if s >= base or tried >= a.ntry:
        break
    tried += 1
    flips = [m for m in G if m in asg]
    label = f'{a.tag}_{seed}_t{tried}'
    say(f'-- trial {tried}: score {s}, flipping {flips}')
    cur = best
    for i, m in enumerate(G):
        nxt = f'tmp/{label}_scr{i}.kicad_pcb'
        sg.swap_stub(cur, a.fo, m, nxt)
        cur = nxt
    ok = sg.braid_one(cur, ','.join(G), f'tmp/{label}', dst=a.dst,
                      pages={m: asg[m] for m in asg})   # absent = braid decides; None would mean SWIMMER
    board = f'tmp/{label}.kicad_pcb'
    if not os.path.exists(board):
        say('   braid wrote nothing')
        continue
    gc, opens = sg.grade_full(board, NETS)
    if gc[0] > gb[0] and gc[1] <= gb[1]:
        _g, base_open = sg.grade_full(best, NETS)
        board, gc, opens = sg.rescue_close(board, a.fo, base_open, NETS,
                                           label, a.dst)
    Jc = Judge(board, nets, a.src, a.dst)
    say(f'   realized: grade {gc} floor {Jc.floor_total} changes '
        f'{Jc.act_total} (refused={not ok})')
    if gc < gb:
        keep = f'tmp/{label}_acc.kicad_pcb'
        shutil.copy(board, keep)
        say(f'   ACCEPT: {gb} -> {gc}')
        best, gb, Jb = keep, gc, Jc
say(f'FINAL {best}: grade {gb} floor {Jb.floor_total} changes {Jb.act_total}')

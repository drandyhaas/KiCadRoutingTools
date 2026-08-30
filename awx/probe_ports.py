#!/usr/bin/env python3
"""PORT homotopy probe: every DU1 ball is within 3 rows of the north or
south edge, so each net may enter from the WEST port (ball-y order),
the NORTH flank (a B river wrapping the NW corner, nested: topmost comb
lane -> easternmost column) or the SOUTH flank (mirror). For a
candidate split, report each port's permutation structure (inversions,
LIS, LDS) and the west bundle's two-river cover."""
import os
import sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import braid as te  # noqa: E402

pcb = parse_kicad_pcb(os.path.join(HERE, 'fb_t2q_base.kicad_pcb'))
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
W = ('SDQM1,SDQ9,SDQ10,SDQ11,SDQS1N,SDQ8,SDQS1P,SDQ12,SDQ13,SDQM0,SDQ14,'
     'SA14,SDQ15,SDQ0,SA10,SA11,SDQ2,SDQ1,SA15,SA12,SDQS0P,SDQS0N,SDQ4,SA0,'
     'SDQ5,SBA1,SDQ6,SDQ3,SA3,SDQ7,SA1,SA4,SCS0,SCS1,SCKE1,SA6').split(',')
ends = te.endpoints(pcb, W, byname)
rows = sorted({round(p.global_y, 2) for p in pcb.footprints['DU1'].pads})
cols = sorted({round(p.global_x, 2) for p in pcb.footprints['DU1'].pads})
row_i = {n: rows.index(round(ends[n][1][1], 2)) for n in W}
col_i = {n: cols.index(round(ends[n][1][0], 2)) for n in W}
comb = {n: ends[n][0][1] for n in W}


def perm_stats(nets, key):
    """nets in comb-y order; key = the port order value; returns
    (inversions, LIS, LDS)."""
    seq = [key(n) for n in sorted(nets, key=lambda n: comb[n])]
    inv = sum(1 for i in range(len(seq)) for j in range(i + 1, len(seq))
              if seq[i] > seq[j])
    ranks = [sorted(seq).index(v) for v in seq]
    lis = len(te.lis_keep(ranks)) if ranks else 0
    lds = len(te.lis_keep([-r for r in ranks])) if ranks else 0
    return inv, lis, lds


def two_cover(nets):
    seq_nets = sorted(nets, key=lambda n: comb[n])
    seq = [sorted(nets, key=lambda n: (ends[n][1][1], ends[n][1][0]))
           .index(n) for n in seq_nets]
    best = {(-1, -1): 0}
    for i in range(len(seq)):
        new = {}
        for (a, b), v in best.items():
            new[(a, b)] = max(new.get((a, b), 0), v)
            if a == -1 or seq[a] < seq[i]:
                new[(i, b)] = max(new.get((i, b), 0), v + 1)
            if b == -1 or seq[b] < seq[i]:
                new[(a, i)] = max(new.get((a, i), 0), v + 1)
        best = new
    return max(best.values()) if best else 0


def report(name, west, north, south):
    print(f'== {name}: west {len(west)} north {len(north)} south {len(south)}')
    inv, lis, lds = perm_stats(west, lambda n: (ends[n][1][1],
                                                ends[n][1][0]))
    print(f'   west port: inv {inv} LIS {lis} LDS {lds} '
          f'2-river cover {two_cover(west)} crossers {len(west) - two_cover(west)}')
    # north nesting: comb-y increasing <-> column decreasing
    inv, lis, lds = perm_stats(north, lambda n: -col_i[n])
    print(f'   north flank: inv {inv} LIS {lis} LDS {lds}  '
          + ','.join(sorted(north, key=lambda n: comb[n])))
    inv, lis, lds = perm_stats(south, lambda n: col_i[n])
    print(f'   south flank: inv {inv} LIS {lis} LDS {lds}  '
          + ','.join(sorted(south, key=lambda n: comb[n])))


for n in sorted(W, key=lambda n: comb[n]):
    print(f'{n:7s} comb y {comb[n]:.2f} row {row_i[n]} col {col_i[n]}')
report('all west', W, [], [])
# split A: columns >= 6 (deep in the field) go around; rows 0-2 north,
# rows 3-5 south
north = [n for n in W if col_i[n] >= 6 and row_i[n] <= 2]
south = [n for n in W if col_i[n] >= 6 and row_i[n] >= 3]
west = [n for n in W if n not in north and n not in south]
report('deep columns around', west, north, south)
north = [n for n in W if col_i[n] >= 4 and row_i[n] <= 2]
south = [n for n in W if col_i[n] >= 4 and row_i[n] >= 3]
west = [n for n in W if n not in north and n not in south]
report('cols>=4 around', west, north, south)
north = [n for n in W if row_i[n] <= 2 and col_i[n] >= 1]
south = [n for n in W if row_i[n] >= 3 and col_i[n] >= 1]
west = [n for n in W if n not in north and n not in south]
report('everything around', west, north, south)

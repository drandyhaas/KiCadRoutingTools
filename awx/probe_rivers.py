#!/usr/bin/env python3
"""TWO-RIVER decomposition probe: how many of the west nets can ride
two order-preserving rivers (one per layer), i.e. the largest subset
whose launch->entry permutation splits into 2 increasing subsequences?
The rest are CROSSERS. Reports sizes and the born-layer fit."""
import os
import sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import topo_emit as te  # noqa: E402

pcb = parse_kicad_pcb(os.path.join(HERE, 'fb_t2q_base.kicad_pcb'))
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
W = ('SDQM1,SDQ9,SDQ10,SDQ11,SDQS1N,SDQ8,SDQS1P,SDQ12,SDQ13,SDQM0,SDQ14,'
     'SA14,SDQ15,SDQ0,SA10,SA11,SDQ2,SDQ1,SA15,SA12,SDQS0P,SDQS0N,SDQ4,SA0,'
     'SDQ5,SBA1,SDQ6,SDQ3,SA3,SDQ7,SA1,SA4,SCS0,SCS1,SCKE1,SA6').split(',')
K = int(sys.argv[1]) if len(sys.argv) > 1 else 36
W = W[:K]
ends = te.endpoints(pcb, W, byname)
born = {}
for n in W:
    nid = byname[n][0]
    tp = ends[n][0]
    lay = next((s.layer for s in pcb.segments if s.net_id == nid
                and (abs(s.start_x - tp[0]) + abs(s.start_y - tp[1]) < 0.005
                     or abs(s.end_x - tp[0]) + abs(s.end_y - tp[1])
                     < 0.005)), 'F.Cu')
    born[n] = lay[0]
launch = sorted(W, key=lambda n: ends[n][0][1])
target = sorted(W, key=lambda n: (ends[n][1][1], ends[n][1][0]))
rank = {n: i for i, n in enumerate(target)}
seq = [rank[n] for n in launch]
K = len(seq)


def lis(seq):
    return len(te.lis_keep(seq))


# largest subset splittable into 2 increasing subsequences: DP over
# (i, j) = last elements of the two chains; O(K^3) is fine at K <= 300
INF = -1
# f[a][b]: max size using elements up to index max(a,b) with chain1
# ending at a and chain2 ending at b (a, b in -1..K-1; -1 = empty)
best = {}
best[(-1, -1)] = 0
order = []
for i in range(K):
    new = {}
    for (a, b), v in best.items():
        # skip i
        new[(a, b)] = max(new.get((a, b), 0), v)
        # append to chain 1
        if a == -1 or seq[a] < seq[i]:
            new[(i, b)] = max(new.get((i, b), 0), v + 1)
        if b == -1 or seq[b] < seq[i]:
            new[(a, i)] = max(new.get((a, i), 0), v + 1)
    best = new
two = max(best.values())
# longest decreasing = min chains
lds = lis([-r for r in seq])
print(f'K={K} inversions='
      f'{sum(1 for i in range(K) for j in range(i + 1, K) if seq[i] > seq[j])}'
      f' LIS={lis(seq)} LDS(min rivers)={lds} 2-river cover={two} '
      f'crossers={K - two}')
# born layers along the launch order
print('born layers (launch order):',
      ''.join(born[n] for n in launch))

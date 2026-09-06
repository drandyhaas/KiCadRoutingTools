#!/usr/bin/env python3
"""Print the first K nets of the COHERENT K-ladder (k_ladder_coherent.txt).

That file's own header is the point: a prefix K must never split a
river. Taking "the first K nets by launch y" instead mixes singletons
into small K and measures a harder problem than the campaign's -- at
K=8 the braid emits 593 DRC violations on the launch-y prefix and 0 on
this one."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.path.join(HERE, 'fb_t2q_fresh.kicad_pcb')


def _rivers():
    rivers = []
    for line in open(os.path.join(HERE, 'k_ladder_coherent.txt')):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        rivers.append(line.split())
    return rivers


def coherent_nets(K, board=BENCH):
    """The first K routable nets of the coherent ladder: two-pad nets
    between two components that are fanned out on `board` (a free
    stub end exists at the source)."""
    flat = [n for r in _rivers() for n in r]
    sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
    sys.path.insert(0, HERE)
    from kicad_parser import parse_kicad_pcb
    import braid as te
    pcb = parse_kicad_pcb(board)
    by = {n.name.split('/')[-1]: n for n in pcb.nets.values()}
    flat = [n for n in flat
            if n in by and len(by[n].pads) == 2
            and len({p.component_ref for p in by[n].pads}) == 2]
    bn = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    ok = []
    for n in flat:
        try:
            te.endpoints(pcb, [n], bn)
            ok.append(n)
        except AssertionError:
            pass
    return ok[:K]


if __name__ == '__main__':
    rivers = _rivers()
    flat = [n for r in rivers for n in r]
    if '--checkpoints' in sys.argv:
        tot = 0
        out = []
        for r in rivers:
            tot += len(r)
            out.append(str(tot))
        print(' '.join(out))
        sys.exit(0)
    K = int(sys.argv[1]) if len(sys.argv) > 1 else 51
    print(','.join(flat[:K] if '--raw' in sys.argv else coherent_nets(K)))

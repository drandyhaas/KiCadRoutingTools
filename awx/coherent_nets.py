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


def ladder_of(board=BENCH):
    """The ladder file for a bench board: `<board stem>.ladder.txt` beside
    it when there is one (a second array pair carries its own rivers),
    else the bench's `k_ladder_coherent.txt`."""
    side = os.path.splitext(board)[0] + '.ladder.txt'
    return side if os.path.exists(side) else os.path.join(HERE, 'k_ladder_coherent.txt')


def _rivers(board=BENCH):
    rivers = []
    for line in open(ladder_of(board)):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        rivers.append(line.split())
    return rivers


def coherent_nets(K, board=BENCH):
    """The first K routable nets of the coherent ladder: two-pad nets
    between two components that are fanned out on `board` (a free
    stub end exists at the source)."""
    flat = [n for r in _rivers(board) for n in r]
    sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
    sys.path.insert(0, HERE)
    import contextlib
    from kicad_parser import parse_kicad_pcb
    import braid as te
    # the parser prints board warnings (net-tagged graphics, duplicate
    # references) on STDOUT; this CLI's stdout is the net list a chain
    # captures, so those go to stderr here
    with contextlib.redirect_stdout(sys.stderr):
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
    # A SHORT COUNT IS A DIFFERENT PROBLEM, and it was silent: "K51" is
    # 48 nets here, so the human's 81 (48 nets) and 85 (51) are both
    # right and both were in circulation for one label. Pointing --board
    # at a ROUTED board is worse: a routed net has no free stub end, so
    # K41 returned the 7 UNROUTED nets instead of the 41.
    if len(ok) < K:
        print(f'  coherent_nets: asked K={K}, {len(ok)} net(s) qualify on '
              f'{os.path.basename(board)} -- this is a {len(ok)}-net problem',
              file=sys.stderr)
    return ok[:K]


if __name__ == '__main__':
    board = next((a.split('=', 1)[1] for a in sys.argv if a.startswith('--board=')), BENCH)
    rivers = _rivers(board)
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
    print(','.join(flat[:K] if '--raw' in sys.argv else coherent_nets(K, board)))

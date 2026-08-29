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
rivers = []
for line in open(os.path.join(HERE, 'k_ladder_coherent.txt')):
    line = line.strip()
    if not line or line.startswith('#'):
        continue
    rivers.append(line.split())
flat = [n for r in rivers for n in r]
# The ladder's tail carries names that are not point-to-point bus nets
# at all (SVREF, SZQ, DU1_ZQ...): no fanout stub, or not a two-pad net
# between the two components. They are not something a corridor can
# route, and they crash endpoints() with 'no free stub end'. Keep only
# nets the bus problem is actually about.
if '--raw' not in sys.argv:
    import os as _os
    _here = _os.path.dirname(_os.path.abspath(__file__))
    try:
        sys.path.insert(0, _os.path.join(_here, '..', 'py_router'))
        from kicad_parser import parse_kicad_pcb as _p
        _pcb = _p(_os.path.join(_here, 'fb_t2q_base.kicad_pcb'))
        _by = {n.name.split('/')[-1]: n for n in _pcb.nets.values()}
        flat = [n for n in flat
                if n in _by and len(_by[n].pads) == 2
                and len({p.component_ref for p in _by[n].pads}) == 2]
        # ...and it must actually be FANNED OUT. DU1_ZQ is a two-pad
        # net across two components and passes the test above, but has
        # no free stub end, so endpoints() rejects it. Run the real
        # test rather than a proxy for it.
        sys.path.insert(0, _here)
        import topo_emit as _te
        _bn = {n.name.split('/')[-1]: (i, n)
               for i, n in _pcb.nets.items()}
        _ok = []
        for _n in flat:
            try:
                _te.endpoints(_pcb, [_n], _bn)
                _ok.append(_n)
            except AssertionError:
                pass
        flat = _ok
    except Exception:
        pass
if '--checkpoints' in sys.argv:
    tot = 0
    out = []
    for r in rivers:
        tot += len(r)
        out.append(str(tot))
    print(' '.join(out))
else:
    K = int(sys.argv[1]) if len(sys.argv) > 1 else 51
    print(','.join(flat[:K]))

#!/usr/bin/env python3
"""A DERIVED obstacle model answers as a model built from scratch without the items it leaves out.

  python3 tests/test_622_obstacle_overlay.py

The models are topo_strings.Obstacles; braid.build_obstacles memoises one per (board, layer, excluded nets) and
derives each net's from it. On kicad_files/routed_output.kicad_pcb, on every copper layer:

1. Obstacles.exclude({net}) -- the rewritten cells over the base's (an overlay) -- against a full build of the same
   items less the net's: every cell's candidates, every cell's geometry pack, point_violation (with and without a
   pad) and seg_clear at fixed random points and segments, and signature(). The derived model holds the rewritten
   cells alone: a copy of the base's cells per derived model held 800 MB over zynq U1's joint plan.
2. A model derived from a derived one: without both nets' items.
3. build_obstacles with an excluded set larger than the net (pair_exit_clear's {leg, partner}; the braid's group):
   its base DERIVED from the base without any (less the set's segments) against the base built in full without them,
   then the net taken out of each. A full build per excluded set was a whole board per diff pair per layer.
4. The taut strings (taut_fast.relax_many) of a run's nets in one batch, each against its model DERIVED as in 3 and
   against the model built directly: the same paths. The strings' model is drawn from the models' shared lists, which a
   derived base shares with the full board's -- its excluded segments must stay out of it (they once stood in as
   obstacles to every other string: synth `mix`, three taut paths ~1 um off and two nets left open).
"""
import contextlib
import io
import os
import random
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
with contextlib.redirect_stdout(io.StringIO()):
    from kicad_parser import parse_kicad_pcb  # noqa: E402
    import braid  # noqa: E402
    import taut_fast as tf  # noqa: E402
    import topo_strings as ts  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
BAD = []


def check(ok, what):
    print(('ok    ' if ok else 'FAIL  ') + what)
    if not ok:
        BAD.append(what)


def full_without(base, drop, where=None):
    """The model built from scratch with `drop`'s items (those `where` names) left out, the rest in their order."""
    o = ts.Obstacles()
    for i, d in enumerate(base.discs):
        if not (base.dnets[i] in drop and (where is None or where(d))):
            o.add_disc(d[0], d[1], d[2], d[3], net=base.dnets[i])
    for i, c in enumerate(base.caps):
        if not (base.cnets[i] in drop and (where is None or where(c))):
            o.add_cap(c[0], c[1], c[2], c[3], net=base.cnets[i])
    o.build()
    return o


def answers(o, pts, segs):
    """Every answer a consumer reads, by content (a derived model indexes the base's items, a full build its own)."""
    cells_d = {k: [o.discs[i] for i in v] for k, v in o._near_d.items()}
    cells_c = {k: [o.caps[i] for i in v] for k, v in o._near_c.items()}
    return (cells_d, cells_c, dict(o._pack.items()), [o.point_violation(p) for p in pts],
            [o.point_violation(p, pad=0.1) for p in pts], [o.seg_clear(a, b) for a, b in segs], o.signature())


def main():
    if not os.path.isfile(BOARD):
        print(f'BROKEN TEST: no board at {BOARD}')
        return 2
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(BOARD)
    if not getattr(pcb, 'source_path', None):
        print('BROKEN TEST: the parsed board has no source_path, so build_obstacles memoises (and derives) nothing')
        return 2
    seg_nets = sorted({s.net_id for s in pcb.segments if s.net_id})
    pad_nets = sorted({p.net_id for f in pcb.footprints.values() for p in f.pads if p.net_id})
    if len(seg_nets) < 20:
        print(f'BROKEN TEST: {len(seg_nets)} nets with segments on the board, too few to exercise an excluded set')
        return 2
    b = pcb.board_info.board_bounds
    rnd = random.Random(5)
    pts = [(rnd.uniform(b[0], b[2]), rnd.uniform(b[1], b[3])) for _ in range(300)]
    segs = [(p, (p[0] + rnd.uniform(-3, 3), p[1] + rnd.uniform(-3, 3))) for p in pts[:150]]
    nets = pad_nets[::max(1, len(pad_nets) // 12)] + seg_nets[::max(1, len(seg_nets) // 6)]
    n4 = 0
    for L in pcb.board_info.copper_layers:
        base = braid._build_obstacles(pcb, frozenset(), L, margin=0.15)
        bad1 = [n for n in nets if answers(base.exclude({n}), pts, segs) != answers(full_without(base, {n}), pts, segs)]
        check(not bad1, f'{L}: {len(nets)} nets, the derived model answers as a full build without the net '
                        f'{"" if not bad1 else f"(differs: {bad1[:5]})"}')
        big = max(nets, key=lambda n: sum(1 for x in base.dnets + base.cnets if x == n))
        der = base.exclude({big})
        check(isinstance(der._pack, ts._Overlay) and len(der._pack.over) < len(base._pack) / 4,
              f'{L}: a derived model holds its rewritten cells alone ({len(getattr(der._pack, "over", base._pack))} '
              f'of the base\'s {len(base._pack)})')
        a_, b_ = seg_nets[0], seg_nets[len(seg_nets) // 2]
        check(answers(base.exclude({a_}).exclude({b_}), pts, segs) == answers(full_without(base, {a_, b_}), pts, segs),
              f'{L}: derived from a derived model, without both nets')
        asks = [(seg_nets[i], frozenset(seg_nets[i:i + 2])) for i in range(0, len(seg_nets) - 2, max(1, len(seg_nets) // 5))]
        asks += [(seg_nets[3], frozenset(seg_nets[:8])), (-1, frozenset(seg_nets[:12]))]
        bad3 = []
        for nid, kids in asks:
            base_kids = kids if (nid in kids and len(kids) > 1) else kids - {nid}
            new = braid.build_obstacles(pcb, nid, kids, L)
            old = braid._build_obstacles(pcb, base_kids, L).exclude({nid})
            if answers(new, pts, segs) != answers(old, pts, segs):
                bad3.append((nid, sorted(kids)[:3]))
        check(not bad3, f'{L}: {len(asks)} excluded sets, the base derived without their segments answers as the base '
                        f'built without them {"" if not bad3 else f"(differs: {bad3[:3]})"}')
        # (a run's nets with copper on the layer, each string from its own segment to the next net's: strings that pass
        # the other nets' copper, which the derived base excludes -- from random points the strings met none of it, and
        # this check passed with the bug in)
        first = {}
        for sg in pcb.segments:
            if sg.layer == L and sg.net_id and sg.net_id not in first:
                first[sg.net_id] = (sg.start_x, sg.start_y)
        run = sorted(first)[:8]
        if len(run) < 2:
            continue
        n4 += 1
        ends_ = [(first[n_], first[run[(k + 1) % len(run)]]) for k, n_ in enumerate(run)]
        new_ = tf.relax_many([(s_, e_, braid.build_obstacles(pcb, n_, frozenset(run), L)) for n_, (s_, e_) in zip(run, ends_)])
        dir_ = braid._build_obstacles(pcb, frozenset(run), L)
        old_ = tf.relax_many([(s_, e_, dir_.exclude({n_})) for n_, (s_, e_) in zip(run, ends_)])
        bad4 = [n_ for n_, a_, b_ in zip(run, new_, old_) if a_[0] != b_[0]]
        check(not bad4, f'{L}: {len(run)} taut strings in one batch, the derived models\' paths the direct ones\' '
                        f'{"" if not bad4 else f"(differ: {bad4[:5]})"}')
    check(n4 > 0, f'the taut strings were checked on {n4} layer(s) (a layer with two nets\' copper or more)')
    print('PASS: 0 failure(s)' if not BAD else f'FAIL: {len(BAD)} failure(s): ' + '; '.join(BAD))
    return 1 if BAD else 0


if __name__ == '__main__':
    sys.exit(main())

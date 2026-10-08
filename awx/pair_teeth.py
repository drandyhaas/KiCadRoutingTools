#!/usr/bin/env python3
"""pair_teeth.py BOARD NETS REF.. -- at each array REF, each differential pair of the bus (NETS: a file of net names,
one a line -- a round's nets.lines -- or a comma list; pairs.pair_names) and what stands between its two TEETH: the
outer ends of the two legs' copper past the array's ball box. Two legs leaving by one face on one layer, side by side,
close to their pair's gap just past their teeth and go on as one lane; another net's track on that layer, or a via of
any net, in the quad from half a pitch inside the two teeth to REACH mm out splits the pair there, and is walled in by
it. Two legs by two faces or two layers are reported apart.

The zynq DDR's joint fanout: U1's NetR3_2 escaped on B.Cu between DDR3_DQS1's B teeth, the pair closed past its end,
A14 ran over it on F.Cu, and the whole route's audit found the pair short at the teeth in every pass.

    python3 pair_teeth.py RUN/r1/fo.kicad_pcb RUN/r1/nets.lines U1 U2      # in awx/
"""
KRT_TOOL = {'scope': [], 'kind': 'instrument'}   # a research tool (awx), catalogued, shown at no door

import argparse
import contextlib
import io
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))

REACH = 1.6         # mm past the teeth: where a pair has closed to its gap (joint_escape.LANE_REACH)
GROW = 1.5          # mm round the ball box a tooth may stand


def teeth(segments, name_of, legs, bbox, grow=GROW):
    """{leg: (tooth, outward unit, layer)}: each leg's copper end furthest past the ball box `bbox` (x0, y0, x1, y1)
    and inside it grown by `grow`; `segments` [(a, b, layer, net)], `name_of` {net: short name}"""
    x0, y0, x1, y1 = bbox
    best = {}
    for a, b, layer, net in segments:
        nm = name_of.get(net)
        if nm not in legs:
            continue
        for q in (a, b):
            if not (x0 - grow <= q[0] <= x1 + grow and y0 - grow <= q[1] <= y1 + grow):
                continue
            d, u = max((q[0] - x1, (1, 0)), (x0 - q[0], (-1, 0)), (q[1] - y1, (0, 1)), (y0 - q[1], (0, -1)))
            if d > 0 and (nm not in best or d > best[nm][0]):
                best[nm] = (d, q, u, layer)
    return {nm: v[1:] for nm, v in best.items()}


def _cross(a, b, c, d):
    def o(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    return o(a, b, c) * o(a, b, d) < 0 and o(c, d, a) * o(c, d, b) < 0


def _inside(q, quad):
    s = [(quad[(i + 1) % 4][0] - quad[i][0]) * (q[1] - quad[i][1])
         - (quad[(i + 1) % 4][1] - quad[i][1]) * (q[0] - quad[i][0]) for i in range(4)]
    return all(x >= 0 for x in s) or all(x <= 0 for x in s)


def pockets(pairs, tooth, half_pitch, reach=REACH):
    """{pair: (layer, face, quad) or 'apart'}: `pairs` {base: (P leg, N leg)}, `tooth` teeth(); each pair's POCKET,
    the quad from `half_pitch` inside its two teeth to `reach` out -- where its legs close past their teeth, so a
    track that enters it is walled in. A pair with a leg that has no tooth is left out"""
    out = {}
    for base, (pn, nn) in sorted(pairs.items()):
        if pn not in tooth or nn not in tooth:
            continue
        (qp, u, lp), (qn, un, ln_) = tooth[pn], tooth[nn]
        if u != un or lp != ln_:
            out[base] = 'apart'
            continue
        out[base] = (lp, u, [(qp[0] - u[0] * half_pitch, qp[1] - u[1] * half_pitch),
                             (qn[0] - u[0] * half_pitch, qn[1] - u[1] * half_pitch),
                             (qn[0] + u[0] * reach, qn[1] + u[1] * reach),
                             (qp[0] + u[0] * reach, qp[1] + u[1] * reach)])
    return out


def in_pocket(quad, a, b=None):
    """a point `a`, or the segment a-b, inside the quad `quad` or crossing its edge"""
    if b is None:
        return _inside(a, quad)
    return _inside(a, quad) or _inside(b, quad) or any(_cross(a, b, quad[i], quad[(i + 1) % 4]) for i in range(4))


def between(pairs, tooth, segments, vias, name_of, half_pitch, reach=REACH):
    """{pair: (layer, face, [the other nets between its teeth]) or 'apart'}: `pairs` {base: (P leg, N leg)}, `tooth`
    teeth(); `segments` [(a, b, layer, net)], `vias` [(at, net)]; another net's track on the pair's layer, or via,
    in its pocket (pockets). A pair with a leg that has no tooth is left out"""
    out = {}
    for base, pk in pockets(pairs, tooth, half_pitch, reach).items():
        if pk == 'apart':
            out[base] = pk
            continue
        lp, u, quad = pk
        pn, nn = pairs[base]
        hit = {name_of[net] for a, b, layer, net in segments
               if layer == lp and name_of.get(net) not in (None, pn, nn) and in_pocket(quad, a, b)}
        hit |= {name_of[net] for at, net in vias if name_of.get(net) not in (None, pn, nn) and in_pocket(quad, at)}
        out[base] = (lp, u, sorted(hit))
    return out


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument('board', metavar='BOARD')
    ap.add_argument('nets', metavar='NETS', help="the bus's nets: a file of names, one a line, or a comma list")
    ap.add_argument('refs', metavar='REF', nargs='+', help='the arrays')
    args = ap.parse_args(argv)
    from kicad_parser import parse_kicad_pcb
    import escape_moves as em
    import pairs as _pairs
    board, nets_arg, refs = args.board, args.nets, args.refs
    names = ([ln.strip() for ln in open(nets_arg) if ln.strip()] if os.path.isfile(nets_arg)
             else [n for n in nets_arg.split(',') if n])
    bus = {n.split('/')[-1] for n in names}
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    name_of = {i: n.name.split('/')[-1] for i, n in pcb.nets.items() if n.name}
    prs = _pairs.pair_names(sorted(bus), admit_all=True)
    legs = {leg for pn, nn in prs.values() for leg in (pn, nn)}
    segs = [((s.start_x, s.start_y), (s.end_x, s.end_y), s.layer, s.net_id) for s in pcb.segments]
    vias = [((v.x, v.y), v.net_id) for v in pcb.vias]
    split = 0
    for ref in refs:
        foot = pcb.footprints[ref]
        grid = em.grid_of(foot)
        t = teeth(segs, name_of, legs, grid.bbox)
        for base, r in between(prs, t, segs, vias, name_of, max(grid.pitch_x, grid.pitch_y) / 2.0).items():
            if r == 'apart':
                print(f'{ref} {base}: its legs leave by two faces or two layers')
                continue
            layer, u, hit = r
            split += bool(hit)
            print(f'{ref} {base}: teeth {tuple(round(c, 2) for c in t[prs[base][0]][0])} '
                  f'{tuple(round(c, 2) for c in t[prs[base][1]][0])} on {layer}, face {u}; between them: '
                  + (', '.join(hit) if hit else 'nothing'))
    print(f'{split} pair(s) split at their teeth')
    return 1 if split else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))

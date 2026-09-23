#!/usr/bin/env python3
"""human_ends_bench.py -- a bench whose ENDS are the human's: the braid alone,
measured on the teeth and berths a human fanned out.

usage: human_ends_bench.py HUMAN.kicad_pcb OUT.kicad_pcb NETS|@FILE
           [--ends both|src|dst] [--others LIST|bench:BOARD|none] [--margin 1.0]
           [--src U1] [--dst DU1] [--add BOARD]... [--ladder FILE] [--pro FILE]
           [--sidecar] [--marker]

NETS      the run's nets: their copper is clipped at the ends asked for.
--others  the other nets that keep copper on the bench ('bench:BOARD' = every
          net with copper on BOARD that is not in NETS): clipped at the SOURCE
          only (their teeth are obstacles, as on the bench); 'none' drops them.
--add     every segment and via of BOARD appended, nets mapped by NAME (a comb
          transplant: a bench's source fanout onto the human's placement, when
          the source part's pose is the same on both).
--sidecar also write OUT's plan sidecar (<OUT>.plan.json): the braid's own
          reading of the ends, in the keys fanout_from_plan writes; --marker
          adds the pages_first marker (the chain's marker-ON regime).

A net's stub at an end is its copper inside the array's pad box grown by the
margin (segments cut at the box line, vias inside kept) that touches one of
its pads there; anything else inside the box is dropped. Exactly one piece per
asked end, or the run refuses and names the net. Every other copper item of
the human board is dropped. Three things about a human's copper the braid
cannot read as drawn, all handled here:
  * track ARCS (which kicad_parser does not read) become two chords each;
  * copper joined by OVERLAP (a track ending inside a via's annulus, a track
    ending on another's interior) is joined explicitly -- a bridge to the via
    centre, a split at the T -- or the braid reads the overlap as a free end;
  * a via that lands in a pad is stamped Type VII as the chain's own are.
The bench gets the chain's DRC floor in its .kicad_pro. Never give a chain TAG
that collides with the bench's name up to case (macOS paths are case-blind,
and chain_k.sh removes its outputs before it writes them).
"""
import argparse
import collections
import contextlib
import io
import json
import math
import os
import re
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb, Segment  # noqa: E402
from kicad_writer import generate_segment_sexpr, generate_via_sexpr  # noqa: E402


def read_arcs(pcb, path):
    """The board's track arcs appended to pcb.segments as two chords each."""
    n = 0
    for m in re.finditer(r'\n  \(arc \(start ([-\d.]+) ([-\d.]+)\) \(mid ([-\d.]+) ([-\d.]+)\) \(end ([-\d.]+) '
                         r'([-\d.]+)\) \(width ([\d.]+)\) \(layer "([^"]+)"\) \(net (\d+)\)',
                         open(path, encoding='utf-8').read()):
        x0, y0, xm, ym, x1, y1, w = (float(m.group(i)) for i in range(1, 8))
        for (p0, q0, p1, q1) in ((x0, y0, xm, ym), (xm, ym, x1, y1)):
            pcb.segments.append(Segment(start_x=p0, start_y=q0, end_x=p1, end_y=q1, width=w,
                                        layer=m.group(8), net_id=int(m.group(9))))
        n += 1
    return n


def strip_blocks(txt, names=('segment', 'via', 'arc')):
    """The board text with every top-level block of those kinds removed."""
    out, i, n = [], 0, 0
    pat = re.compile(r'\n  \((' + '|'.join(names) + r')[\s(]')
    while True:
        m = pat.search(txt, i)
        if not m:
            out.append(txt[i:])
            break
        out.append(txt[i:m.start()])
        j, depth, instr = m.start() + 3, 0, False
        while j < len(txt):
            c = txt[j]
            if instr:
                if c == '\\':
                    j += 1
                elif c == '"':
                    instr = False
            elif c == '"':
                instr = True
            elif c == '(':
                depth += 1
            elif c == ')':
                depth -= 1
                if depth == 0:
                    j += 1
                    break
            j += 1
        i = j
        n += 1
    return ''.join(out), n


def clip_seg(x0, y0, x1, y1, box):
    bx0, by0, bx1, by1 = box
    dx, dy = x1 - x0, y1 - y0
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, x0 - bx0), (dx, bx1 - x0), (-dy, y0 - by0), (dy, by1 - y0)):
        if abs(p) < 1e-12:
            if q < 0:
                return None
        else:
            t = q / p
            if p < 0:
                t0 = max(t0, t)
            else:
                t1 = min(t1, t)
    if t0 >= t1 - 1e-9:
        return None
    return (x0 + dx * t0, y0 + dy * t0, x0 + dx * t1, y0 + dy * t1)


def pad_layers(p):
    if (p.drill and p.drill > 0) or any('*' in L for L in p.layers):
        return {'F.Cu', 'B.Cu'}
    return {L for L in p.layers if L.endswith('.Cu')}


def pt_seg_d(x, y, s):
    x0, y0, x1, y1 = s
    dx, dy = x1 - x0, y1 - y0
    L2 = dx * dx + dy * dy
    t = 0 if L2 == 0 else max(0, min(1, ((x - x0) * dx + (y - y0) * dy) / L2))
    return math.hypot(x - (x0 + t * dx), y - (y0 + t * dy))


def pieces(segs, vias, pads, tol=0.02):
    """The connected pieces of copper (segs: (x0, y0, x1, y1, width, layer);
    vias: Via) that touch one of `pads`, each [(kind, item)], every join
    spelled out; and the number of pieces there were. Returns (pieces, n, joins)."""
    items = [('s', s, [(s[5], s[0], s[1]), (s[5], s[2], s[3])]) for s in segs]
    items += [('v', v, [(L, v.x, v.y) for L in ('F.Cu', 'B.Cu')]) for v in vias]
    parent = list(range(len(items)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    radius = lambda it: it[1].size / 2 if it[0] == 'v' else it[1][4] / 2
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            hit = any(La == Lb and abs(xa - xb) < tol and abs(ya - yb) < tol
                      for (La, xa, ya) in items[i][2] for (Lb, xb, yb) in items[j][2])
            if not hit:
                # copper OVERLAPPING copper: a T, a via on a segment's interior,
                # a via whose annulus reaches a track's end
                for (k1, k2) in ((i, j), (j, i)):
                    if items[k2][0] == 's':
                        s = items[k2][1]
                        hit = hit or any(L == s[5] and pt_seg_d(x, y, s[:4]) <= radius(items[k1]) + s[4] / 2 + 0.005
                                         for (L, x, y) in items[k1][2])
            if hit:
                parent[find(i)] = find(j)
    comps = collections.defaultdict(list)
    for i in range(len(items)):
        comps[find(i)].append(i)

    def touches(i):
        r = radius(items[i])
        return any(L in pad_layers(p) and math.hypot(x - p.global_x, y - p.global_y)
                   <= max(p.size_x, p.size_y) / 2 + r + tol for (L, x, y) in items[i][2] for p in pads)
    attached = [[items[i][:2] for i in c] for c in comps.values() if any(touches(i) for i in c)]
    # SPELL THE JOINS OUT: the braid reads a stub's end as a segment endpoint
    # used once that lies in no pad and no barrel (braid.endpoints), so copper
    # joined by overlap alone reads as a free end. Only an endpoint used ONCE
    # can, so only those are joined (a chain's interior nodes are left alone).
    joins = 0
    for piece in attached:
        segs_p = [o for k, o in piece if k == 's']
        vias_p = [o for k, o in piece if k == 'v']
        extra, drop, add = [], [], []
        use = collections.Counter((s[5], round(x, 2), round(y, 2)) for s in segs_p
                                  for (x, y) in ((s[0], s[1]), (s[2], s[3])))
        seen = set()
        for s in segs_p:
            for (x, y) in ((s[0], s[1]), (s[2], s[3])):
                if use[(s[5], round(x, 2), round(y, 2))] != 1 or (s[5], round(x, 3), round(y, 3)) in seen:
                    continue
                seen.add((s[5], round(x, 3), round(y, 3)))
                for v in vias_p:
                    if 0.02 < math.hypot(x - v.x, y - v.y) <= v.size / 2 + s[4] / 2 + 0.005:
                        extra.append((x, y, v.x, v.y, s[4], s[5]))
                for s2 in segs_p:
                    if s2 is s or s2[5] != s[5]:
                        continue
                    if min(math.hypot(x - s2[0], y - s2[1]), math.hypot(x - s2[2], y - s2[3])) <= 0.02:
                        continue
                    # a T: the endpoint projects onto s2's INTERIOR within the two
                    # half-widths -- never onto an end, the chain's next segment
                    x0, y0, x1, y1 = s2[:4]
                    dx, dy = x1 - x0, y1 - y0
                    L2 = dx * dx + dy * dy
                    if L2 <= 0:
                        continue
                    tt = ((x - x0) * dx + (y - y0) * dy) / L2
                    L = math.sqrt(L2)
                    if tt * L < 0.02 or (1 - tt) * L < 0.02:
                        continue
                    px, py = x0 + tt * dx, y0 + tt * dy
                    if math.hypot(px - x, py - y) <= s[4] / 2 + s2[4] / 2 + 0.005:
                        if s2 not in drop:
                            drop.append(s2)
                            add += [(x0, y0, px, py, s2[4], s2[5]), (px, py, x1, y1, s2[4], s2[5])]
                        if math.hypot(px - x, py - y) > 0.001:
                            extra.append((x, y, px, py, s[4], s[5]))
        for s2 in drop:
            piece.remove(('s', s2))
        piece += [('s', o) for o in add + extra if math.hypot(o[2] - o[0], o[3] - o[1]) > 1e-4]
        joins += len(extra) + len(drop)
    return attached, len(comps), joins


def write_sidecar(board, nets, dest, out, marker):
    """The braid's own reading of the board's ends as a plan sidecar."""
    import braid as te
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
        ctx, groups = te.setup(board, nets, dest, lambda m='': None, plan=None)
    if ctx.M is not None:
        raise SystemExit('the pair reads mirrored (chirality -1): turn it with flow_frame.py first')
    keep = [nm for nm in nets if nm in ctx.ends]
    d = {'chi': int(ctx.chi),
         'ends': {nm: [list(map(float, ctx.ends[nm][0])), list(map(float, ctx.ends[nm][1]))] for nm in keep},
         'tooth_layer': {nm: ctx.tooth_layer[nm] for nm in keep},
         'dest_layer': {nm: ctx.dest_layer[nm] for nm in keep},
         'tooth_dir': {nm: [float(v) for v in ctx.tooth_dir[nm]] for nm in keep},
         'stub_dir': {nm: [float(v) for v in ctx.stub_dir[nm]] for nm in keep}}
    if marker:
        d['pages_first'] = True
    json.dump(d, open(out, 'w'), indent=1)
    print(f'wrote {out}: {len(d["ends"])} ends; tooth layers {dict(collections.Counter(d["tooth_layer"].values()))}; '
          f'berth layers {dict(collections.Counter(d["dest_layer"].values()))}; groups {[len(g) for g in groups]}')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('human')
    ap.add_argument('out')
    ap.add_argument('nets')
    ap.add_argument('--ends', default='both', choices=('both', 'src', 'dst'))
    ap.add_argument('--others', default='none')
    ap.add_argument('--margin', type=float, default=1.0)
    ap.add_argument('--src', default='U1')
    ap.add_argument('--dst', default='DU1')
    ap.add_argument('--add', action='append', default=[])
    ap.add_argument('--ladder')
    ap.add_argument('--pro')
    ap.add_argument('--sidecar', action='store_true')
    ap.add_argument('--marker', action='store_true')
    a = ap.parse_args(argv)
    nets_arg = open(a.nets[1:]).read() if a.nets.startswith('@') else a.nets
    nets = [n.strip() for n in nets_arg.replace('\n', ',').split(',') if n.strip()]

    pcb = parse_kicad_pcb(a.human)
    print(f'  {read_arcs(pcb, a.human)} arcs read as chords')
    short = {i: n.name.split('/')[-1] for i, n in pcb.nets.items()}
    byname = {v: k for k, v in short.items()}
    if a.others.startswith('bench:'):
        b = parse_kicad_pcb(a.others[6:])
        bs = {i: n.name.split('/')[-1] for i, n in b.nets.items()}
        others = sorted({bs[s.net_id] for s in b.segments} | {bs[v.net_id] for v in b.vias})
        others = [n for n in others if n not in nets]
    elif a.others == 'none':
        others = []
    else:
        others = [n for n in a.others.split(',') if n]

    def box_of(ref):
        fp = pcb.footprints[ref]
        xs = [p.global_x for p in fp.pads]
        ys = [p.global_y for p in fp.pads]
        return (min(xs) - a.margin, min(ys) - a.margin, max(xs) + a.margin, max(ys) + a.margin)

    joins = 0

    def stubs_of(nm, ends):
        nonlocal joins
        nid = byname[nm]
        segs_n = [(s.start_x, s.start_y, s.end_x, s.end_y, s.width, s.layer) for s in pcb.segments if s.net_id == nid]
        vias_n = [v for v in pcb.vias if v.net_id == nid]
        keep, report = [], []
        for end, ref in (('src', a.src), ('dst', a.dst)):
            if end not in ends:
                continue
            box = box_of(ref)
            cs = []
            for s in segs_n:
                c = clip_seg(*s[:4], box)
                if c and math.hypot(c[2] - c[0], c[3] - c[1]) > 1e-3:
                    cs.append((c[0], c[1], c[2], c[3], s[4], s[5]))
            cv = [v for v in vias_n if box[0] <= v.x <= box[2] and box[1] <= v.y <= box[3]]
            pads = [p for p in pcb.footprints[ref].pads if p.net_id == nid]
            if not pads:
                report.append(f'{end}: no pad of {nm} on {ref}')
                continue
            att, ncomp, nj = pieces(cs, cv, pads)
            joins += nj
            flat = sum(att, [])
            report.append(f'{end}: {len(att)} piece(s) of {ncomp} ({sum(1 for k, _ in flat if k == "s")} seg, '
                          f'{sum(1 for k, _ in flat if k == "v")} via)')
            if len(att) != 1:
                raise SystemExit(f'REFUSED {nm} at {end}: {len(att)} attached pieces of {ncomp} inside the box')
            keep += att[0]
        return keep, '; '.join(report)

    txt, n_gone = strip_blocks(open(a.human, encoding='utf-8').read())
    emit, tot_s, tot_v = [], 0, 0
    for nm in nets + others:
        if nm not in byname:
            print(f'  {nm}: not a net of the human board -- skipped')
            continue
        ends = {'both': ('src', 'dst'), 'src': ('src',), 'dst': ('dst',)}[a.ends] if nm in nets else ('src',)
        keep, rep = stubs_of(nm, ends)
        nid = byname[nm]
        for kind, o in keep:
            if kind == 's':
                emit.append(generate_segment_sexpr((o[0], o[1]), (o[2], o[3]), o[4], o[5], nid))
                tot_s += 1
            else:
                emit.append(generate_via_sexpr(o.x, o.y, o.size, o.drill, list(o.layers), nid,
                                               tenting_attrs=o.tenting_attrs, inherit_when_unspecified=True))
                tot_v += 1
        print(f'  {nm:7s} {rep}')
    for bpath in a.add:
        b = parse_kicad_pcb(bpath)
        bs = {i: n.name.split('/')[-1] for i, n in b.nets.items()}
        na = nv = 0
        for s in b.segments:
            nid = byname.get(bs[s.net_id])
            if nid is not None:
                emit.append(generate_segment_sexpr((s.start_x, s.start_y), (s.end_x, s.end_y), s.width, s.layer, nid))
                na += 1
        for v in b.vias:
            nid = byname.get(bs[v.net_id])
            if nid is not None:
                emit.append(generate_via_sexpr(v.x, v.y, v.size, v.drill, list(v.layers), nid,
                                               tenting_attrs=v.tenting_attrs, inherit_when_unspecified=True))
                nv += 1
        print(f'  added from {os.path.basename(bpath)}: {na} segments, {nv} vias')
        tot_s += na
        tot_v += nv
    body = txt.rstrip()
    assert body.endswith(')')
    body = body[:-1].rstrip() + '\n' + '\n'.join(emit) + '\n)\n'
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    open(a.out, 'w', encoding='utf-8').write(body)
    stem = a.out[:-len('.kicad_pcb')]
    pro = a.pro or (a.human[:-len('.kicad_pcb')] + '.kicad_pro')
    if os.path.exists(pro):
        shutil.copy(pro, stem + '.kicad_pro')
    import braid as te
    import source_realize as sr
    import ship_vias
    from fix_kicad_drc_settings import fix_project_for_output
    with contextlib.redirect_stdout(io.StringIO()):
        fix_project_for_output(a.out, clearance=te.SPEC_CLEARANCE, track_width=sr.FAN_TRACK,
                               via_diameter=te.VIA_SIZE, via_drill=te.VIA_DRILL, verbose=False)
    rec = ship_vias.stamp(a.out, context='human_ends_bench')
    if a.ladder:
        shutil.copy(a.ladder, stem + '.ladder.txt')
    chk = parse_kicad_pcb(a.out)
    print(f'joins spelled out: {joins}')
    print(f'wrote {a.out}: {n_gone} copper items stripped; {tot_s} segments + {tot_v} vias kept '
          f'(re-read {len(chk.segments)}/{len(chk.vias)}); Type VII stamped {rec.get("stamped") if rec else None}; '
          f'floor {te.SPEC_CLEARANCE}')
    if a.sidecar:
        write_sidecar(a.out, nets, a.dst, stem + '.plan.json', a.marker)


if __name__ == '__main__':
    main()

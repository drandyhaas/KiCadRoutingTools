#!/usr/bin/env python3
"""pack_board.py BOARD.kicad_pcb [--out OUT] -- pack a braided board again on
its own. Reads the braid's sidecar (BOARD.pack.json: each lane's copper
as the pack receives it, the corridors' target orders and planned
lines, every lane's tooth) and the board, runs pack.pack_corridor per
corridor exactly as braid.write_out does, and writes OUT.kicad_pcb (+
.kicad_pro) with the routed nets' copper replaced. Seconds where the
braid took a minute: the way to iterate on the pack.

    BRAID_PACK_DEBUG=1 python3 pack_board.py tmp/pk0_k41.kicad_pcb --out tmp/x_k41
"""
import argparse
import json
import os
import shutil
import sys
import time
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)

from kicad_parser import parse_kicad_pcb  # noqa: E402
import braid as br  # noqa: E402
import pack as pk  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('--out', default=None, help='output stem (default: <board stem>_packed)')
    ap.add_argument('--sidecar', default=None, help='the pack sidecar (default: <board stem>.pack.json)')
    a = ap.parse_args()
    stem = os.path.splitext(a.board)[0]
    out = a.out or stem + '_packed'
    side_path = a.sidecar or stem + '.pack.json'
    if not os.path.exists(side_path):
        print(f'no sidecar {side_path}: run the braid on this board first (it writes one)')
        return 2
    side = json.load(open(side_path))
    t0 = time.time()
    pcb = parse_kicad_pcb(a.board)
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    names = list(side['lanes'])
    kids = {byname[nm][0] for nm in names}
    # the lanes' copper: the sidecar's pieces matched to the board's own
    # segment and via OBJECTS (the pack tells copper apart by identity)
    def key(x, y):
        return (round(x, 3), round(y, 3))
    seg_index = {}
    for s in pcb.segments:
        if s.net_id in kids:
            seg_index.setdefault((key(s.start_x, s.start_y), key(s.end_x, s.end_y), s.layer), []).append(s)
            seg_index.setdefault((key(s.end_x, s.end_y), key(s.start_x, s.start_y), s.layer), []).append(s)
    via_index = {}
    for v in pcb.vias:
        if v.net_id in kids:
            via_index.setdefault(key(v.x, v.y), []).append(v)
    out_segs, out_vias = {}, {}
    missing = 0
    for nm in names:
        segs = []
        for sx, sy, ex, ey, L, w in side['lanes'][nm]['segs']:
            hit = seg_index.get((key(sx, sy), key(ex, ey), L))
            if hit:
                segs.append(hit[0])
            else:
                missing += 1
        vias = []
        for x, y in side['lanes'][nm]['vias']:
            hit = via_index.get(key(x, y))
            if hit:
                vias.append(hit[0])
            else:
                missing += 1
        out_segs[nm], out_vias[nm] = segs, vias
    if missing:
        print(f'WARNING: {missing} sidecar piece(s) not found on the board (a board the braid did not write?)')
    cfg = SimpleNamespace(layers=list(side['layers']), board_edge_clearance=side['board_edge_clearance'])
    # the destination stub chain per lane, tip-side first, matched to the
    # board's segments so a join at a vertex can drop the bypassed stub
    dest_chain = {}
    for nm, ch in (side.get('dest_chain') or {}).items():
        lst = []
        for sx, sy, ex, ey, L in ch:
            hit = seg_index.get((key(sx, sy), key(ex, ey), L))
            lst.append((hit[0] if hit else None, (round(sx, 3), round(sy, 3)), (round(ex, 3), round(ey, 3))))
        dest_chain[nm] = lst
    ctx = SimpleNamespace(pcb=pcb, byname=byname, kids=kids, cfg=cfg,
                          ends={nm: (tuple(e[0]), tuple(e[1])) for nm, e in side['ends'].items()},
                          base_segments=list(pcb.segments), base_vias=list(pcb.vias),
                          dest_chain=dest_chain, trim_spans={})
    corridors = []
    for cd in side['corridors']:
        c = SimpleNamespace(members=list(cd['members']), target=list(cd['target']),
                            lane_xy={nm: [tuple(p) for p in poly] for nm, poly in cd['lane_xy'].items()},
                            out_segs={nm: out_segs[nm] for nm in cd['members'] if nm in out_segs},
                            out_vias={nm: out_vias[nm] for nm in cd['members'] if nm in out_vias},
                            ctx=ctx)
        corridors.append(c)
    log = print
    log(f'pack_board: {os.path.basename(a.board)}, {len(names)} lanes in {len(corridors)} corridor(s), '
        f'{sum(len(v) for v in out_segs.values())} lane pieces  (read {time.time() - t0:.1f} s)')
    for c in corridors:
        pk.pack_corridor(c, log)
    # the stub trim, as braid.write_out does it: the stub tip-side of the
    # vertex the packed lane ends at is bypassed copper
    n_trim = 0
    for c in corridors:
        for nm in c.members:
            if c.out_segs.get(nm):
                br.note_joint(ctx, nm, c.out_segs[nm])
                n_trim += len(ctx.trim_spans.get(nm, []))
    if n_trim:
        log(f'berth stub trim: {n_trim} bypassed segment(s) removed')
    # write: the routed nets' copper replaced, every other line of the file as it was
    txt = open(a.board, encoding='utf-8').read()
    kid_names = {pcb.nets[i].name for i in kids if i in pcb.nets}
    txt = br.strip_net_segments(txt, kids, kid_names)
    txt = br.strip_net_items(txt, 'via', kids, kid_names)
    add = []
    seen = set()
    for s in pcb.segments:
        if s.net_id not in kids:
            continue
        if abs(s.end_x - s.start_x) < 1e-3 and abs(s.end_y - s.start_y) < 1e-3:
            continue
        k = (round(s.start_x, 4), round(s.start_y, 4), round(s.end_x, 4), round(s.end_y, 4), s.layer)
        if k in seen or (k[2], k[3], k[0], k[1], k[4]) in seen:
            continue
        seen.add(k)
        add.append(f'  (segment (start {s.start_x:.4f} {s.start_y:.4f}) (end {s.end_x:.4f} {s.end_y:.4f}) '
                   f'(width {s.width}) (layer "{s.layer}") (net {s.net_id}))\n')
    for v in pcb.vias:
        if v.net_id not in kids:
            continue
        layers = v.layers if getattr(v, 'layers', None) else ['F.Cu', 'B.Cu']
        add.append(f'  (via (at {v.x:.4f} {v.y:.4f}) (size {v.size}) (drill {v.drill}) '
                   f'(layers "{layers[0]}" "{layers[-1]}") (net {v.net_id}))\n')
    k = txt.rstrip().rfind(')')
    with open(out + '.kicad_pcb', 'w') as f:
        f.write(txt[:k] + ''.join(add) + txt[k:])
    pro = stem + '.kicad_pro'
    if os.path.exists(pro):
        shutil.copy(pro, out + '.kicad_pro')
    log(f'wrote {out}.kicad_pcb: {len(add)} items  ({time.time() - t0:.1f} s)')
    return 0


if __name__ == '__main__':
    sys.exit(main())

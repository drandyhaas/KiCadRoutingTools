#!/usr/bin/env python3
"""Plan-driven re-fanout (task 5 prototype): rewrite the U1-side escape
stubs of plan-wished nets onto B.Cu -- ball, 45 F diagonal to the
inter-ball cell, dogbone via, B street run east to the comb line. The
emitter then sees a B tooth and the net is BORN diving (no dive via).

Production integration would put this inside bga_fanout's layer
assignment (per-net layer constraints from the plan); this prototype
proves the interface on the bench without touching the engine.
"""
import argparse
import json
import math
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import braid as te  # noqa: E402
import topo_strings as ts  # noqa: E402

VIA_SIZE, VIA_DRILL, TRACK = 0.25, 0.15, 0.127


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--board', default=os.path.join(HERE,
                    'fb_t2q_base.kicad_pcb'))
    ap.add_argument('--plan', required=True)
    ap.add_argument('--out', default=os.path.join(HERE,
                    'fb_t2q_refan.kicad_pcb'))
    a = ap.parse_args()

    plan = json.load(open(a.plan))
    wished = [nm for nm, spec in plan.items()
              if spec.get('fanout_wish') == 'B']
    print(f'refanout wishlist: {wished}')
    pcb = parse_kicad_pcb(a.board)
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    kids = {byname[nm][0] for nm in plan}

    # comb exit x = just past the existing east teeth
    exit_x = max(max(s.start_x, s.end_x) for s in pcb.segments
                 if s.net_id in kids) + 0.0
    # U1 ball pitch from the wished nets' source component
    add = []
    strip_ids = set()
    placed = []
    for nm in wished:
        nid, net = byname[nm]
        ball = min(net.pads, key=lambda p: p.global_x)   # U1 side (west)
        fp = pcb.footprints[ball.component_ref]
        xs = sorted({round(p.global_x, 3) for p in fp.pads})
        pitch = min(b - a2 for a2, b in zip(xs, xs[1:]) if b - a2 > 0.1)
        hp = pitch / 2
        ux, uy = ball.global_x, ball.global_y
        obsF = te.build_obstacles(pcb, nid, kids, 'F.Cu')
        obsB = te.build_obstacles(pcb, nid, kids, 'B.Cu')
        got = None
        for sy in (uy + hp, uy - hp):        # street above/below the row
            vx, vy = ux + hp, sy             # east diagonal cell
            ok = True
            vpad = (VIA_SIZE - TRACK) / 2
            for o in (obsF, obsB):
                vv = o.point_violation((vx, vy), pad=vpad)
                if vv and vv[0] > 0:
                    ok = False
            ok = ok and obsF.seg_clear((ux, uy), (vx, vy))
            ok = ok and obsB.seg_clear((vx, vy), (exit_x, vy))
            ok = ok and all(abs(vy - py_) > 0.25 or
                            math.hypot(vx - px_, vy - py_) > 0.4
                            for (px_, py_) in placed)
            ok = ok and all(abs(vy - py_) > 0.25 for (_p, py_) in placed)
            if ok:
                got = (vx, vy)
                break
        if got is None:
            print(f'{nm}: NO clear B street (existing B escapes crowd '
                  f'its row) -- left on F')
            continue
        vx, vy = got
        placed.append((vx, vy))
        strip_ids.add(nid)
        add.append(f'  (segment (start {ux:.4f} {uy:.4f}) '
                   f'(end {vx:.4f} {vy:.4f}) (width {TRACK}) '
                   f'(layer "F.Cu") (net {nid}))\n')
        add.append(f'  (via (at {vx:.4f} {vy:.4f}) (size {VIA_SIZE}) '
                   f'(drill {VIA_DRILL}) (layers "F.Cu" "B.Cu") '
                   f'(net {nid}))\n')
        add.append(f'  (segment (start {vx:.4f} {vy:.4f}) '
                   f'(end {exit_x:.4f} {vy:.4f}) (width {TRACK}) '
                   f'(layer "B.Cu") (net {nid}))\n')
        print(f'{nm}: B escape via ({vx:.3f},{vy:.3f}), '
              f'tooth ({exit_x:.3f},{vy:.3f})')

    txt = open(a.board, encoding='utf-8').read()
    strip_names = {pcb.nets[i].name for i in strip_ids if i in pcb.nets}
    txt = te.strip_net_segments(txt, strip_ids, strip_names)
    k = txt.rstrip().rfind(')')
    with open(a.out, 'w') as f:
        f.write(txt[:k] + ''.join(add) + txt[k:])
    pro = os.path.splitext(a.board)[0] + '.kicad_pro'
    if os.path.exists(pro):
        shutil.copy(pro, os.path.splitext(a.out)[0] + '.kicad_pro')
    print(f'wrote {a.out} ({len(wished)} nets re-fanned to B)')


if __name__ == '__main__':
    main()

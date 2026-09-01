#!/usr/bin/env python3
"""#622 single-net relay: rip 1-2 nets' escape stubs at ONE array and
re-lay them to revised asks, everything else frozen. The improve
loop's second channel -- the pages sidecar moves where a net RIDES;
this moves where it is BORN (tooth layer/slot at the source, or the
berth stub at the destination with --ref DU1), which is the only way
to harvest end-layer intrinsic waste (a tooth on B feeding an F ride
pays vias no schedule flip can remove). Only the named nets' copper
changes, so the experiment stays surgical -- no whole-comb re-pack.

usage: relay_net.py BOARD NET[,NET2] --out OUT --layer L[,L2]
         [--keep-pos | --side S --coord C] [--swap]
         [--ref U1] [--slot-w 0.4]

--keep-pos derives each net's ask from its EXISTING stub end (proven
position, new layer). --swap (2 nets, keep-pos) exchanges the two
derived positions -- the safest reorder move, both poses proven by
copper. --layer 'keep' keeps that net's current stub-end layer.
--ref DU1 relays BERTH stubs instead (same strip window and engine,
other array).
"""
import argparse
import os
import re
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb          # noqa: E402
from kicad_writer import add_tracks_and_vias_to_pcb  # noqa: E402
import bga_fanout as bf                           # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('board')
ap.add_argument('nets')
ap.add_argument('--out', required=True)
ap.add_argument('--ref', default='U1')
ap.add_argument('--layer', required=True)
ap.add_argument('--keep-pos', action='store_true')
ap.add_argument('--swap', action='store_true')
ap.add_argument('--side', default=None)
ap.add_argument('--coord', type=float, default=None)
ap.add_argument('--slot-w', type=float, default=0.4)
a = ap.parse_args()

nets = a.nets.split(',')
layers = a.layer.split(',')
if len(layers) < len(nets):
    layers += [layers[-1]] * (len(nets) - len(layers))
if a.swap and (len(nets) != 2 or not a.keep_pos):
    sys.exit('--swap needs exactly 2 nets and --keep-pos')

base = parse_kicad_pcb(a.board)
bby = {n.name.split('/')[-1]: (i, n) for i, n in base.nets.items()}
for nm in nets:
    if nm not in bby:
        sys.exit(f'no net {nm}')
fp = base.footprints[a.ref]
xs = [p.global_x for p in fp.pads]
ys = [p.global_y for p in fp.pads]
BB = (min(xs), min(ys), max(xs), max(ys))
W = (BB[0] - 0.25, BB[1] - 0.25, BB[2] + 0.25, BB[3] + 0.25)
RW = (BB[0] - 1.6, BB[1] - 1.6, BB[2] + 1.6, BB[3] + 1.6)


def in_rip(x, y):
    return RW[0] <= x <= RW[2] and RW[1] <= y <= RW[3]


def stub_end(nid):
    """The net's degree-1 stub end inside the rip window, farthest
    from the array centre, with the layer it ends on."""
    deg, lay = {}, {}
    for s in base.segments:
        if s.net_id != nid or not in_rip(s.start_x, s.start_y):
            continue
        for pt in ((round(s.start_x, 3), round(s.start_y, 3)),
                   (round(s.end_x, 3), round(s.end_y, 3))):
            deg[pt] = deg.get(pt, 0) + 1
            lay[pt] = s.layer
    free = [p for p, c in deg.items() if c == 1]
    if not free:
        return None
    cx, cy = (BB[0] + BB[2]) / 2, (BB[1] + BB[3]) / 2
    p = max(free, key=lambda q: (q[0] - cx) ** 2 + (q[1] - cy) ** 2)
    return p, lay[p]


def side_coord(pt):
    d = {'up': abs(pt[1] - W[1]), 'right': abs(pt[0] - W[2]),
         'down': abs(pt[1] - W[3]), 'left': abs(pt[0] - W[0])}
    side = min(d, key=d.get)
    return side, pt[1] if side in ('left', 'right') else pt[0]


# ---- the asks
asks = {}
poses = {}
for nm in nets:
    nid, _ = bby[nm]
    if a.keep_pos:
        se = stub_end(nid)
        if se is None:
            sys.exit(f'{nm}: no stub end found in the rip window')
        poses[nm] = se
    else:
        if a.side is None or a.coord is None:
            sys.exit('--side/--coord required without --keep-pos')
if a.keep_pos:
    order = list(nets)
    src = {nm: poses[nm] for nm in nets}
    if a.swap:
        src = {nets[0]: poses[nets[1]], nets[1]: poses[nets[0]]}
    for nm, lay_arg in zip(order, layers):
        pt, cur_lay = src[nm]
        side, coord = side_coord(pt)
        lay = cur_lay if lay_arg == 'keep' else lay_arg
        asks[nm] = (side, coord, lay)
else:
    asks[nets[0]] = (a.side, a.coord, layers[0])
for nm, (side, coord, lay) in asks.items():
    print(f'{nm}: ask {side}/{lay}@{coord:.2f}')

# ---- rip only these nets inside the window (plan_fanout's stripper)
def strip_region(txt, token, nid_, full_name):
    out, i = [], 0
    while True:
        j = txt.find('(' + token, i)
        if j < 0:
            out.append(txt[i:])
            break
        k, depth = j, 0
        while True:
            ch = txt[k]
            if ch == '(':
                depth += 1
            elif ch == ')':
                depth -= 1
                if depth == 0:
                    break
            k += 1
        block = txt[j:k + 1]
        m = re.search(r'\(net (\d+)\)', block)
        m2 = re.search(r'\(net "([^"]+)"\)', block)
        hit = (m and int(m.group(1)) == nid_) or \
              (m2 and m2.group(1) == full_name)
        pts = re.findall(r'\((?:start|end|at) ([-\d.]+) ([-\d.]+)',
                         block)
        if hit and pts and all(in_rip(float(x), float(y))
                               for x, y in pts):
            out.append(txt[i:j].rstrip(' \t'))
            e = k + 1
            if e < len(txt) and txt[e] == '\n':
                e += 1
            i = e
        else:
            out.append(txt[i:k + 1])
            i = k + 1
    return ''.join(out)


txt = open(a.board).read()
for nm in nets:
    nid, net = bby[nm]
    for token in ('segment', 'via'):
        txt = strip_region(txt, token, nid, net.name)
stripped = a.out + '.bare.tmp'
open(stripped, 'w').write(txt)
pcb = parse_kicad_pcb(stripped)

# ---- re-lay through the production engine, hints per ball
dir_h, lay_h, slot_h, keys = {}, {}, {}, {}
for nm in nets:
    nid, _ = bby[nm]
    ball = next(p for p in pcb.footprints[a.ref].pads
                if p.net_id == nid)
    k = (round(ball.global_x, 3), round(ball.global_y, 3))
    side, coord, lay = asks[nm]
    keys[nm] = k
    dir_h[k] = side
    lay_h[k] = lay
    slot_h[k] = (side, coord - a.slot_w / 2, coord + a.slot_w / 2)
tracks, vias_add, vias_rm, failed = bf.generate_bga_fanout(
    pcb.footprints[a.ref], pcb, net_filter=nets,
    layers=['F.Cu', 'B.Cu'], track_width=0.1, clearance=0.1,
    via_size=0.45, via_drill=0.25, exit_margin=0.5,
    escape_method='underpad', plane_drop='off',
    escape_dir_hints=dir_h, escape_layer_hints=lay_h,
    escape_slot_hints=slot_h)
if failed:
    os.remove(stripped)
    sys.exit(f'RELAY FAILED: engine could not escape {failed}')
add_tracks_and_vias_to_pcb(
    stripped, a.out, tracks, vias_add, vias_rm,
    net_id_to_name={i: n.name for i, n in pcb.nets.items()})
os.remove(stripped)
pro = os.path.splitext(a.board)[0] + '.kicad_pro'
if os.path.exists(pro):
    shutil.copy(pro, os.path.splitext(a.out)[0] + '.kicad_pro')
for nm in nets:
    rep = bf.LAST_LAYER_HINT_REPORT.get(keys[nm])
    print(f'{nm}: layer hint '
          + (('OBEYED' if rep['obeyed']
              else f"MISSED -> {rep['got']}") if rep else 'unaudited'))

# ---- DELIVERED-face audit (audit #4): the layer report above says
# nothing about the FACE/slot -- the engine may slide along the face
# or fall back to another gap and still read OBEYED. Verify delivery
# GEOMETRICALLY from the written board: recompute each net's stub end
# and name the face it actually landed on, so a caller (face_sweep)
# can tell an undelivered face from a slid slot.
out_pcb = parse_kicad_pcb(a.out)
oby = {n.name.split('/')[-1]: i for i, n in out_pcb.nets.items()}
for nm in nets:
    deg2, lay2 = {}, {}
    for s in out_pcb.segments:
        if s.net_id != oby.get(nm) or not in_rip(s.start_x, s.start_y):
            continue
        for pt in ((round(s.start_x, 3), round(s.start_y, 3)),
                   (round(s.end_x, 3), round(s.end_y, 3))):
            deg2[pt] = deg2.get(pt, 0) + 1
            lay2[pt] = s.layer
    free2 = [p for p, c in deg2.items() if c == 1]
    if not free2:
        print(f'{nm}: DELIVERED unknown (no stub end)')
        continue
    cx, cy = (BB[0] + BB[2]) / 2, (BB[1] + BB[3]) / 2
    p2 = max(free2, key=lambda q: (q[0] - cx) ** 2 + (q[1] - cy) ** 2)
    dside, dcoord = side_coord(p2)
    ask_side = asks[nm][0]
    # deliberately NOT the string 'MISSED' -- improve_k's refusal
    # gate keys on that substring and an off-target delivery is a
    # real board, not a refusal
    print(f'{nm}: DELIVERED {dside}/{lay2[p2]}@{dcoord:.2f}'
          + ('' if dside == ask_side
             else f'  face OFFTARGET (asked {ask_side})'))
print(f'wrote {a.out}: {len(tracks)} track(s), {len(vias_add)} via(s)')

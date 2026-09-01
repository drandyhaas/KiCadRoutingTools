#!/usr/bin/env python3
"""#622 via-pair collapse: eliminate short dives on a ROUTED board.

A short dive = two same-net vias joined by a brief single-layer
bridge -- the A*'s zigzag escapes (three-via chains at U1's rim, a
lane that surfaces for 0.85 mm and dives again). Each pair is tried
SERIALLY, accept-and-build: rip the two vias + the bridge, ask the
real router (connect, band-free) for a path between the cut ends,
keep the collapse ONLY if it adds no via (2 saved per accept; 1 when
the cut ends sit on different layers). Every accept is verified by
re-walking the net's endpoint degrees; the final board is graded by
the caller (grade_k / check_drc / check_connected as ever).

usage: collapse_dives.py BOARD --out OUT [--nets CSV] [--max-bridge MM]
"""
import argparse
import math
import os
import re
import shutil
import sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402
from kicad_writer import add_tracks_and_vias_to_pcb  # noqa: E402
import connect as cn  # noqa: E402

TRACK, CLEAR, VIA_SIZE, VIA_DRILL = 0.1, 0.1, 0.45, 0.25

ap = argparse.ArgumentParser()
ap.add_argument('board')
ap.add_argument('--out', required=True)
ap.add_argument('--nets', default=None,
                help='short names CSV; default = every net with vias')
ap.add_argument('--max-bridge', type=float, default=3.0)
a = ap.parse_args()

pcb = parse_kicad_pcb(a.board)
id2nm = {i: n.name.split('/')[-1] for i, n in pcb.nets.items()}
want = set(a.nets.split(',')) if a.nets else None
cfg = cn.make_config(pcb, TRACK, CLEAR, VIA_SIZE, VIA_DRILL,
                     grid_step=0.025)
allpads = [p for n in pcb.nets.values() for p in n.pads]


def key(x, y):
    return (round(x, 2), round(y, 2))


def inpad(v):
    return any(abs(p.global_x - v.x) <= p.size_x / 2 + 1e-3
               and abs(p.global_y - v.y) <= p.size_y / 2 + 1e-3
               and p.net_id == v.net_id for p in allpads)


def find_dives():
    """(net_id, v1, v2, bridge segs, bridge layer) short dives."""
    segs = defaultdict(list)
    for s in pcb.segments:
        segs[s.net_id].append(s)
    out = []
    seen = set()
    for v1 in pcb.vias:
        nm = id2nm.get(v1.net_id)
        if not nm or (want and nm not in want) or inpad(v1):
            continue
        for lay in ('F.Cu', 'B.Cu'):
            sn = [s for s in segs[v1.net_id] if s.layer == lay]
            cur, prev, ln, bridge = key(v1.x, v1.y), None, 0.0, []
            for _hop in range(30):
                nxt = [s for s in sn if id(s) != id(prev)
                       and (key(s.start_x, s.start_y) == cur
                            or key(s.end_x, s.end_y) == cur)]
                if len(nxt) != 1:
                    break
                s = nxt[0]
                other = key(s.end_x, s.end_y) \
                    if key(s.start_x, s.start_y) == cur \
                    else key(s.start_x, s.start_y)
                ln += math.hypot(s.end_x - s.start_x,
                                 s.end_y - s.start_y)
                if ln > a.max_bridge:
                    break
                bridge.append(s)
                v2 = next((v for v in pcb.vias
                           if v.net_id == v1.net_id
                           and id(v) != id(v1)
                           and key(v.x, v.y) == other), None)
                if v2 is not None:
                    pk = frozenset((id(v1), id(v2)))
                    if pk not in seen and not inpad(v2):
                        seen.add(pk)
                        out.append((v1.net_id, v1, v2,
                                    list(bridge), lay))
                    break
                cur, prev = other, s
    return out


def seg_pt_d(px, py, s):
    ax, ay, bx, by = s.start_x, s.start_y, s.end_x, s.end_y
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(
        1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def bridge_has_tjoint(bridge, segs_net):
    """A same-net segment endpoint landing MID-SPAN on the bridge is
    a T-joint the endpoint walk cannot see; removing the bridge would
    sever that branch (measured: K15 SDQ10, K32 3 opens)."""
    bids = {id(s) for s in bridge}
    verts = set()
    for s in bridge:
        verts.add(key(s.start_x, s.start_y))
        verts.add(key(s.end_x, s.end_y))
    for s in segs_net:
        if id(s) in bids:
            continue
        for (px, py) in ((s.start_x, s.start_y), (s.end_x, s.end_y)):
            if key(px, py) in verts:
                continue
            if any(b.layer == s.layer and seg_pt_d(px, py, b) < 0.09
                   for b in bridge):
                return True
    return False


def net_connected(nid):
    """Union-find over this net's copper + pads + vias: every pad in
    one component. The per-accept guard -- a collapse that breaks the
    net is ROLLED BACK, not shipped."""
    net = pcb.nets[nid]
    parent = {}

    def find(x):
        while parent.setdefault(x, x) != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(x, y):
        parent[find(x)] = find(y)

    anchors = []
    for i2, p in enumerate(net.pads):
        anchors.append((('P', i2), p.global_x, p.global_y,
                        max(p.size_x, p.size_y) / 2 + 0.02, None))
    for i2, v in enumerate(pcb.vias):
        if v.net_id == nid:
            anchors.append((('V', i2), v.x, v.y, v.size / 2 + 0.02,
                            None))
    pts = []
    for i2, s in enumerate(pcb.segments):
        if s.net_id != nid:
            continue
        a2, b2 = ('S', i2, 0), ('S', i2, 1)
        union(a2, b2)
        pts.append((a2, s.start_x, s.start_y, s.layer))
        pts.append((b2, s.end_x, s.end_y, s.layer))
    for (ka, xa, ya, la) in pts:
        for (kb, xb, yb, ra, _l) in anchors:
            if math.hypot(xa - xb, ya - yb) <= ra:
                union(ka, kb)
    for i2 in range(len(pts)):
        for j2 in range(i2 + 1, len(pts)):
            ka, xa, ya, la = pts[i2]
            kb, xb, yb, lb = pts[j2]
            if la == lb and math.hypot(xa - xb, ya - yb) <= 0.06:
                union(ka, kb)
    roots = {find(k) for (k, _x, _y, _r, _l) in anchors
             if k[0] == 'P'}
    return len(roots) <= 1


def cut_end(v, bridge_layer, segs_net):
    """The outer copper end at via v: the (pt, layer) of the segment
    touching v on the NON-bridge layer. None if ambiguous."""
    touch = [s for s in segs_net if s.layer != bridge_layer
             and (key(s.start_x, s.start_y) == key(v.x, v.y)
                  or key(s.end_x, s.end_y) == key(v.x, v.y))]
    if len(touch) != 1:
        return None
    return ((v.x, v.y), touch[0].layer)


collapsed = 0
saved = 0
removed_spans = []      # ((x1,y1),(x2,y2), layer, net_id)
removed_vias = []       # (x, y, net_id)
added_tracks = []
for (nid, v1, v2, bridge, blay) in find_dives():
    nm = id2nm[nid]
    segs_net = [s for s in pcb.segments if s.net_id == nid]
    ea = cut_end(v1, blay, segs_net)
    eb = cut_end(v2, blay, segs_net)
    if ea is None or eb is None:
        print(f'  {nm}: dive at ({v1.x:.2f},{v1.y:.2f}) skipped '
              '(junction at via)')
        continue
    if ea[1] != eb[1]:
        print(f'  {nm}: dive spans layers ({ea[1]}/{eb[1]}) -- '
              'skipped (would still need 1 via)')
        continue
    if bridge_has_tjoint(bridge, segs_net):
        print(f'  {nm}: dive at ({v1.x:.2f},{v1.y:.2f}) skipped '
              '(bridge carries a T-joint)')
        continue
    ids_b = {id(s) for s in bridge}
    seg0, via0 = list(pcb.segments), list(pcb.vias)
    pcb.segments = [s for s in pcb.segments if id(s) not in ids_b]
    pcb.vias = [v for v in pcb.vias
                if id(v) not in (id(v1), id(v2))]
    res = None
    for mg in (1.5, 3.0):
        res = cn.connect(pcb, nid, ea[0], ea[1], eb[0], eb[1], cfg,
                         band=None, margin=mg)
        if res is not None:
            break
    if res is None or res[1]:
        pcb.segments, pcb.vias = seg0, via0
        why = 'no route' if res is None else f'{len(res[1])} via(s)'
        print(f'  {nm}: dive at ({v1.x:.2f},{v1.y:.2f}) kept '
              f'({why})')
        continue
    new_segs, _nv = res
    pcb.segments.extend(new_segs)
    if not net_connected(nid):
        pcb.segments, pcb.vias = seg0, via0
        print(f'  {nm}: dive at ({v1.x:.2f},{v1.y:.2f}) ROLLED BACK '
              '(net no longer connected after collapse)')
        continue
    collapsed += 1
    saved += 2
    removed_vias += [(v1.x, v1.y, nid), (v2.x, v2.y, nid)]
    removed_spans += [((s.start_x, s.start_y), (s.end_x, s.end_y),
                       s.layer, nid) for s in bridge]
    added_tracks += [{'start': (s.start_x, s.start_y),
                      'end': (s.end_x, s.end_y), 'width': s.width,
                      'layer': s.layer, 'net_id': nid}
                     for s in new_segs]
    print(f'  {nm}: COLLAPSED ({v1.x:.2f},{v1.y:.2f})<->'
          f'({v2.x:.2f},{v2.y:.2f}) -2 vias, '
          f'{len(new_segs)} new seg(s) on {ea[1]}')

if not collapsed:
    shutil.copy(a.board, a.out)
    pro = os.path.splitext(a.board)[0] + '.kicad_pro'
    if os.path.exists(pro):
        shutil.copy(pro, os.path.splitext(a.out)[0] + '.kicad_pro')
    print('0 collapses; board copied unchanged')
    sys.exit(0)

# textual write: strip the removed vias/spans, append the new tracks
txt = open(a.board, encoding='utf-8').read()


def strip_items(txt, token, match):
    out, i = [], 0
    pat = '(' + token
    while True:
        j = txt.find(pat, i)
        # the next char must be a delimiter: braid emits '(via (at',
        # the fanout writer '(via\n  (at' -- a space-only pattern
        # missed every fanout via (the strip-mismatch refusals)
        while j >= 0 and j + len(pat) < len(txt) \
                and txt[j + len(pat)] not in ' \n\t(':
            j = txt.find(pat, j + 1)
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
        if match(block):
            out.append(txt[i:j].rstrip(' \t'))
            e = k + 1
            if e < len(txt) and txt[e] == '\n':
                e += 1
            i = e
        else:
            out.append(txt[i:k + 1])
            i = k + 1
    return ''.join(out)


def net_of(block):
    m = re.search(r'\(net (\d+)\)', block)
    if m:
        return int(m.group(1))
    m = re.search(r'\(net "([^"]+)"\)', block)
    if m:
        for i2, n2 in pcb.nets.items():
            if n2.name == m.group(1):
                return i2
    return None


TOL = 0.005


def near(x, y, x2, y2):
    return abs(x - x2) < TOL and abs(y - y2) < TOL


n_via_hit = n_seg_hit = 0


def kill_via(block):
    global n_via_hit
    m = re.search(r'\(at ([-\d.]+) ([-\d.]+)\)', block)
    if not m:
        return False
    bx, by = float(m.group(1)), float(m.group(2))
    bn = net_of(block)
    hit = any(near(bx, by, x, y) and n == bn
              for (x, y, n) in removed_vias)
    n_via_hit += hit
    return hit


def kill_seg(block):
    global n_seg_hit
    pts = re.findall(r'\((?:start|end) ([-\d.]+) ([-\d.]+)', block)
    ml = re.search(r'\(layer "([^"]+)"\)', block)
    if len(pts) != 2 or not ml:
        return False
    bn = net_of(block)
    (x1, y1), (x2, y2) = ((float(x), float(y)) for x, y in pts)
    hit = any(lay == ml.group(1) and n == bn
              and ((near(x1, y1, *p) and near(x2, y2, *q))
                   or (near(x1, y1, *q) and near(x2, y2, *p)))
              for (p, q, lay, n) in removed_spans)
    n_seg_hit += hit
    return hit


txt = strip_items(txt, 'via', kill_via)
txt = strip_items(txt, 'segment', kill_seg)
if n_via_hit != len(removed_vias) or n_seg_hit != len(removed_spans):
    # a silent partial strip ships stale copper (measured: one via of
    # four missed by exact-rounding match, board graded +1 via) --
    # refuse rather than write a board that lies
    print(f'STRIP MISMATCH: vias {n_via_hit}/{len(removed_vias)}, '
          f'segs {n_seg_hit}/{len(removed_spans)} -- refusing to '
          'write; board unchanged')
    shutil.copy(a.board, a.out)
    sys.exit(2)
tmp0 = a.out + '.cd.tmp'
open(tmp0, 'w').write(txt)
add_tracks_and_vias_to_pcb(
    tmp0, a.out, added_tracks, [], [],
    net_id_to_name={i: n.name for i, n in pcb.nets.items()})
os.remove(tmp0)
pro = os.path.splitext(a.board)[0] + '.kicad_pro'
if os.path.exists(pro):
    shutil.copy(pro, os.path.splitext(a.out)[0] + '.kicad_pro')
print(f'wrote {a.out}: {collapsed} dive(s) collapsed, '
      f'{saved} via(s) saved')

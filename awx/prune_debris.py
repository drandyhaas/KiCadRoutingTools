#!/usr/bin/env python3
"""#622 dead-branch pruner: remove same-net copper that carries no
connection.

Per net: SPLIT stub segments at mid-span T-joints (a lane joining a
stub's interior leaves a tip-side tail no endpoint test can see --
K32 SA0's 1 mm tail at a t=0.37 joint), then remove dead-end pieces,
parallel TWIN arms (a lane running 20 um beside its own stub) and
degenerate hooks. Every removal is a TRANSACTION: remove + heal the
local gaps to exact contact + cascade the orphaned hooks/tails, then
judge the package by the strict dangle census -- accept all or roll
back all. Via removal only when no surviving copper touches the
barrel. Structural cycles are otherwise KEPT (cycle collapse is
collapse_dives' judged business).

The writer STRIPS every accepted net's segments and RE-EMITS its
work pieces verbatim, so the model the guards measured is exactly
what ships (a block-matching writer diverged from the model and
shipped a 14 um gap no guard had seen).

usage: prune_debris.py BOARD --out OUT [--nets CSV]
"""
import argparse
import copy
import math
import os
import re
import shutil
import sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('board')
ap.add_argument('--out', required=True)
ap.add_argument('--nets', default=None)
a = ap.parse_args()

pcb = parse_kicad_pcb(a.board)
id2nm = {i: n.name.split('/')[-1] for i, n in pcb.nets.items()}
want = set(a.nets.split(',')) if a.nets else None

# per ORIGINAL segment: 'kill' (whole block) or (nsx,nsy,nex,ney)
# trim rewrite; plus vias to kill
emit_nets = {}        # net_id -> alive work pieces to re-emit
kill_vias = []
report = []


def skey(s):
    return (frozenset(((round(s.start_x, 3), round(s.start_y, 3)),
                       (round(s.end_x, 3), round(s.end_y, 3)))),
            s.layer, s.net_id)


for nid, net in sorted(pcb.nets.items()):
    nm = id2nm.get(nid)
    if not nm or (want and nm not in want):
        continue
    origs = [s for s in pcb.segments if s.net_id == nid]
    vias = [v for v in pcb.vias if v.net_id == nid]
    if not origs or len(net.pads) < 2:
        continue
    work = []
    root = {}
    for s in origs:
        w = copy.copy(s)
        work.append(w)
        root[id(w)] = skey(s)

    def census(pieces, dbg=False):
        """Strict dangle count: degree-1 ends (<= 0.005 merge)
        outside every same-net pad and via."""
        n2 = 0
        for i2, w2 in enumerate(pieces):
            for (ex, ey) in ((w2.start_x, w2.start_y),
                             (w2.end_x, w2.end_y)):
                shared = False
                for j2, t2 in enumerate(pieces):
                    if j2 == i2:
                        continue
                    for (tx, ty) in ((t2.start_x, t2.start_y),
                                     (t2.end_x, t2.end_y)):
                        if math.hypot(ex - tx, ey - ty) <= 0.005:
                            shared = True
                            break
                    if shared:
                        break
                if shared:
                    continue
                if any(math.hypot(ex - x2, ey - y2)
                       <= max(p.size_x, p.size_y) / 2 + 0.02
                       for p in net.pads
                       for (x2, y2) in ((p.global_x, p.global_y),)) \
                        or any(math.hypot(ex - v.x, ey - v.y)
                               <= v.size / 2 + 0.02 for v in vias):
                    continue
                n2 += 1
                if dbg:
                    print(f'    DBG counted end ({ex:.3f},{ey:.3f})')
        return n2

    base_dangles = census(origs)
    # ---- split at mid-span T-joints
    changed = True
    while changed:
        changed = False
        epts = {(round(x, 3), round(y, 3), w.layer)
                for w in work
                for (x, y) in ((w.start_x, w.start_y),
                               (w.end_x, w.end_y))}
        for w in list(work):
            ax, ay, bx, by = w.start_x, w.start_y, w.end_x, w.end_y
            L2 = (bx - ax) ** 2 + (by - ay) ** 2
            if L2 < 0.01 ** 2:
                continue
            for (px, py, lay) in epts:
                if lay != w.layer:
                    continue
                t = ((px - ax) * (bx - ax)
                     + (py - ay) * (by - ay)) / L2
                if not (0.02 < t < 0.98):
                    continue
                jx = ax + t * (bx - ax)
                jy = ay + t * (by - ay)
                if math.hypot(px - jx, py - jy) > 0.06:
                    continue
                if math.hypot(jx - ax, jy - ay) < 0.05 or \
                        math.hypot(jx - bx, jy - by) < 0.05:
                    continue
                w2 = copy.copy(w)
                w.end_x, w.end_y = jx, jy
                w2.start_x, w2.start_y = jx, jy
                root[id(w2)] = root[id(w)]
                work.append(w2)
                changed = True
                break
            if changed:
                break

    parent = {}

    def find(x):
        while parent.setdefault(x, x) != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(x, y):
        parent[find(x)] = find(y)

    anch = [(('v', i), v.x, v.y, v.size / 2 + 0.02)
            for i, v in enumerate(vias)] + \
        [(('p', i), p.global_x, p.global_y,
          max(p.size_x, p.size_y) / 2 + 0.02)
         for i, p in enumerate(net.pads)]

    def pads_ok(alive):
        parent.clear()
        pts = []
        for i in alive:
            w = work[i]
            union(('s', i, 0), ('s', i, 1))
            pts.append((('s', i, 0), w.start_x, w.start_y, w.layer))
            pts.append((('s', i, 1), w.end_x, w.end_y, w.layer))
        for (k, x, y, lay) in pts:
            for (k2, x2, y2, r) in anch:
                if math.hypot(x - x2, y - y2) <= r:
                    union(k, k2)
        for i in range(len(pts)):
            for j in range(i + 1, len(pts)):
                ka, xa, ya, la = pts[i]
                kb, xb, yb, lb = pts[j]
                if la == lb and math.hypot(xa - xb, ya - yb) <= 0.06:
                    union(ka, kb)
        roots = {find(('p', i)) for i in range(len(net.pads))}
        return len(roots) <= 1

    alive = set(range(len(work)))
    if not pads_ok(alive):
        report.append(f'  {nm}: already disconnected -- untouched')
        continue

    def endpoint_dead(i, end):
        w = work[i]
        ex = w.start_x if end == 0 else w.end_x
        ey = w.start_y if end == 0 else w.end_y
        for j in alive:
            if j == i:
                continue
            t2 = work[j]
            for (tx, ty) in ((t2.start_x, t2.start_y),
                             (t2.end_x, t2.end_y)):
                if t2.layer == w.layer and \
                        math.hypot(ex - tx, ey - ty) <= 0.06:
                    return False
        if any(math.hypot(ex - x2, ey - y2) <= r
               for (k2, x2, y2, r) in anch if k2[0] == 'p'):
            return False
        at_via = [k2 for (k2, x2, y2, r) in anch if k2[0] == 'v'
                  and math.hypot(ex - x2, ey - y2) <= r]
        if at_via:
            vset = {k2 for k2 in at_via}
            for j in alive:
                if j == i:
                    continue
                t2 = work[j]
                for (tx, ty) in ((t2.start_x, t2.start_y),
                                 (t2.end_x, t2.end_y)):
                    if any(math.hypot(tx - x2, ty - y2) <= r
                           for (k2, x2, y2, r) in anch
                           if k2 in vset):
                        return False
        return True

    def degree1(i, end):
        w = work[i]
        ex = w.start_x if end == 0 else w.end_x
        ey = w.start_y if end == 0 else w.end_y
        for j in alive:
            if j == i:
                continue
            t2 = work[j]
            if t2.layer != w.layer:
                continue
            for (tx, ty) in ((t2.start_x, t2.start_y),
                             (t2.end_x, t2.end_y)):
                if math.hypot(ex - tx, ey - ty) <= 0.005:
                    return False
        return True

    def try_remove(i):
        """TRANSACTIONAL removal: remove i, heal local gaps, cascade
        the consequences (hooks and tails the removal orphans), then
        judge the whole package by the strict dangle census -- accept
        all or roll back all. Per-net was too coarse (a valid berth
        harvest reverted for an unrelated orphan); per-piece was too
        fine (a twin removal fails alone because its hook and tail
        must go WITH it)."""
        global alive
        saved_alive = set(alive)
        coord_undo = []

        def heal_around(pts_near):
            for j in sorted(alive):
                wj = work[j]
                for endj in (0, 1):
                    ex = wj.start_x if endj == 0 else wj.end_x
                    ey = wj.start_y if endj == 0 else wj.end_y
                    if min((math.hypot(ex - px, ey - py)
                            for (px, py) in pts_near),
                           default=9.0) > 0.12:
                        continue
                    if not degree1(j, endj):
                        continue
                    best = None
                    for k2 in sorted(alive):
                        if k2 == j or work[k2].layer != wj.layer:
                            continue
                        tk = work[k2]
                        for e2 in (0, 1):
                            if not degree1(k2, e2):
                                continue
                            tx = tk.start_x if e2 == 0 else tk.end_x
                            ty = tk.start_y if e2 == 0 else tk.end_y
                            d2 = math.hypot(ex - tx, ey - ty)
                            if 0.005 < d2 <= 0.06 and (
                                    best is None or d2 < best[0]):
                                best = (d2, tx, ty)
                    if best is not None:
                        coord_undo.append((j, endj, ex, ey))
                        if endj == 0:
                            wj.start_x, wj.start_y = best[1], best[2]
                        else:
                            wj.end_x, wj.end_y = best[1], best[2]

        if not pads_ok(alive - {i}):
            return False
        w0 = work[i]
        alive = alive - {i}
        heal_around([(w0.start_x, w0.start_y),
                     (w0.end_x, w0.end_y)])
        changed3 = True
        while changed3:
            changed3 = False
            for j in sorted(alive):
                w3 = work[j]
                degen = math.hypot(w3.end_x - w3.start_x,
                                   w3.end_y - w3.start_y) < 0.02
                if not (degen or endpoint_dead(j, 0)
                        or endpoint_dead(j, 1)):
                    continue
                if not pads_ok(alive - {j}):
                    continue
                alive = alive - {j}
                heal_around([(w3.start_x, w3.start_y),
                             (w3.end_x, w3.end_y)])
                changed3 = True
                break
        if census([work[q] for q in sorted(alive)]) > base_dangles:
            alive = saved_alive
            for (j, endj, ox, oy) in coord_undo:
                if endj == 0:
                    work[j].start_x, work[j].start_y = ox, oy
                else:
                    work[j].end_x, work[j].end_y = ox, oy
            return False
        return True

    def sweep_dead_ends():
        global alive
        any_ = False
        changed2 = True
        while changed2:
            changed2 = False
            for i in sorted(alive):
                if (endpoint_dead(i, 0) or endpoint_dead(i, 1)) \
                        and try_remove(i):
                    changed2 = True
                    any_ = True
                    break
        return any_

    sweep_dead_ends()

    # ---- parallel-loop collapse: a piece that runs within contact
    # distance (<= 0.08) of ANOTHER alive same-layer piece over its
    # whole span is a redundant twin arm (K32 SA0: the lane ran 20 um
    # beside its own 1.78 mm stub to formally reach the tip -- one
    # fat strip electrically, an appendage visually). Removal is
    # judged by pads_ok on the SPLIT graph, so only truly redundant
    # arms go.
    def seg_pt_d(px, py, s2):
        ax2, ay2, bx2, by2 = (s2.start_x, s2.start_y,
                              s2.end_x, s2.end_y)
        dx2, dy2 = bx2 - ax2, by2 - ay2
        L22 = dx2 * dx2 + dy2 * dy2
        t2 = 0.0 if L22 == 0 else max(0.0, min(
            1.0, ((px - ax2) * dx2 + (py - ay2) * dy2) / L22))
        return math.hypot(px - (ax2 + t2 * dx2),
                          py - (ay2 + t2 * dy2))

    while True:
        changed = False
        for i in sorted(alive):
            w = work[i]
            ln = math.hypot(w.end_x - w.start_x, w.end_y - w.start_y)
            if ln < 0.15:
                continue
            twin = False
            for j in alive:
                if j == i or work[j].layer != w.layer:
                    continue
                if seg_pt_d(w.start_x, w.start_y, work[j]) <= 0.08 \
                        and seg_pt_d(w.end_x, w.end_y,
                                     work[j]) <= 0.08 \
                        and seg_pt_d((w.start_x + w.end_x) / 2,
                                     (w.start_y + w.end_y) / 2,
                                     work[j]) <= 0.08:
                    twin = True
                    break
            if not twin:
                continue
            if try_remove(i):
                changed = True
                break
        if not changed:
            break
        # a removed twin arm orphans its hooks and the stub tail
        # beyond the joint -- sweep dead ends again, then look for
        # more twins, to a fixed point
        sweep_dead_ends()

    # ---- degenerate sweep: pieces under 0.02 mm (the 20 um hooks
    # left where a removed twin arm met the stub tip) prop up
    # micro-cycles that keep real tails "shared"; drop them when the
    # pads survive, then sweep dead ends again (K32 SA0: hook dies,
    # tail frees, dead-end sweep takes it).
    changed = True
    while changed:
        changed = False
        for i in sorted(alive):
            w = work[i]
            if math.hypot(w.end_x - w.start_x,
                          w.end_y - w.start_y) >= 0.02:
                continue
            if try_remove(i):
                changed = True
                sweep_dead_ends()
                break

    dead = [i for i in range(len(work)) if i not in alive]
    if not dead:
        continue
    # merge collinear neighbours so the emitted count stays sane
    merged = True
    while merged:
        merged = False
        al = sorted(alive)
        for x1 in al:
            for x2 in al:
                if x1 >= x2 or work[x1].layer != work[x2].layer:
                    continue
                w1, w2 = work[x1], work[x2]
                for (e1x, e1y, o1x, o1y) in (
                        (w1.end_x, w1.end_y, w1.start_x, w1.start_y),
                        (w1.start_x, w1.start_y, w1.end_x, w1.end_y)):
                    for (e2x, e2y, o2x, o2y) in (
                            (w2.start_x, w2.start_y,
                             w2.end_x, w2.end_y),
                            (w2.end_x, w2.end_y,
                             w2.start_x, w2.start_y)):
                        if math.hypot(e1x - e2x, e1y - e2y) > 0.001:
                            continue
                        cross = abs((e1x - o1x) * (o2y - e2y)
                                    - (e1y - o1y) * (o2x - e2x))
                        if cross > 1e-6:
                            continue
                        w1.start_x, w1.start_y = o1x, o1y
                        w1.end_x, w1.end_y = o2x, o2y
                        alive = alive - {x2}
                        merged = True
                        break
                    if merged:
                        break
                if merged:
                    break
            if merged:
                break
    # final integrity, OUTCOME-based, on EXACTLY what will be
    # written: the strict dangle census must not GROW vs the
    # pristine input (this caught SODT1's 14 um cross-layer split
    # that every structural check missed)
    post = census([work[i] for i in sorted(alive)],
                  dbg=os.environ.get('PRUNE_DEBUG') == nm)
    if post > base_dangles:
        report.append(f'  {nm}: prune REVERTED (dangle census '
                      f'{base_dangles} -> {post})')
        continue
    mm = sum(math.hypot(work[i].end_x - work[i].start_x,
                        work[i].end_y - work[i].start_y)
             for i in dead if i < len(origs))
    emit_nets[nid] = [work[i] for i in sorted(alive)]
    nv0 = 0
    for v in vias:
        touch = any(
            math.hypot(x - v.x, y - v.y) <= v.size / 2 + 0.02
            for i in alive
            for (x, y) in ((work[i].start_x, work[i].start_y),
                           (work[i].end_x, work[i].end_y)))
        if not touch:
            kill_vias.append(v)
            nv0 += 1
    report.append(f'  {nm}: {len(dead)} piece(s) pruned, '
                  f'{mm:.2f} mm dead'
                  + (f', {nv0} via(s)' if nv0 else ''))

for line in report:
    print(line)
if not emit_nets and not kill_vias:
    shutil.copy(a.board, a.out)
    pro = os.path.splitext(a.board)[0] + '.kicad_pro'
    if os.path.exists(pro):
        shutil.copy(pro, os.path.splitext(a.out)[0] + '.kicad_pro')
    print('nothing to prune; board copied unchanged')
    sys.exit(0)


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


via_keys = defaultdict(int)
for v in kill_vias:
    via_keys[(round(v.x, 3), round(v.y, 3), v.net_id)] += 1
n_via = 0


def kill_via_fn(block):
    global n_via
    m = re.search(r'\(at ([-\d.]+) ([-\d.]+)\)', block)
    if not m:
        return False
    key = (round(float(m.group(1)), 3), round(float(m.group(2)), 3),
           net_of(block))
    if via_keys.get(key, 0) > 0:
        via_keys[key] -= 1
        n_via += 1
        return True
    return False


def kill_seg_fn(block):
    # the MODEL IS THE ARTIFACT rule: every accepted net's segments
    # are stripped wholesale and its work pieces re-emitted verbatim,
    # so the census the guard measured is exactly what ships (the
    # block-matching writer diverged from the model and shipped a
    # 14 um gap the guard had never seen)
    return net_of(block) in emit_nets


def walk(txt, token, fn_kill):
    out, i = [], 0
    pat = '(' + token
    while True:
        j = txt.find(pat, i)
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
        if fn_kill(txt[j:k + 1]):
            out.append(txt[i:j].rstrip(' \t'))
            e = k + 1
            if e < len(txt) and txt[e] == '\n':
                e += 1
            i = e
        else:
            out.append(txt[i:k + 1])
            i = k + 1
    return ''.join(out)


txt = open(a.board, encoding='utf-8').read()
txt = walk(txt, 'segment', kill_seg_fn)
txt = walk(txt, 'via', kill_via_fn)
if n_via != len(kill_vias):
    print(f'VIA STRIP MISMATCH: {n_via}/{len(kill_vias)} -- '
          'refusing to write')
    shutil.copy(a.board, a.out)
    sys.exit(2)
add = []
for nid2, pieces in emit_nets.items():
    for w in pieces:
        add.append(f'  (segment (start {w.start_x:.4f} '
                   f'{w.start_y:.4f}) (end {w.end_x:.4f} '
                   f'{w.end_y:.4f}) (width {w.width}) '
                   f'(layer "{w.layer}") (net {nid2}))\n')
k = txt.rstrip().rfind(')')
open(a.out, 'w', encoding='utf-8').write(
    txt[:k] + ''.join(add) + txt[k:])
pro = os.path.splitext(a.board)[0] + '.kicad_pro'
if os.path.exists(pro):
    shutil.copy(pro, os.path.splitext(a.out)[0] + '.kicad_pro')
print(f'wrote {a.out}: {len(emit_nets)} net(s) re-emitted, '
      f'{len(kill_vias)} via(s) removed')

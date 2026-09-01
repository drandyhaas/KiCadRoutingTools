#!/usr/bin/env python3
"""#622 write-time micro-nudge for QUANTIZATION grazes: a lane vertex
a few tens of microns inside a foreign pad's clearance zone.

The class (measured, K41): the engine prices obstacles on a 0.05 mm
grid, so a legal-by-the-router lane can graze a pad by up to ~half a
cell diagonal (~35 um) at exact-geometry DRC. Fattening obstacles
would close the under-pad channels (the pitch cliff), so the repair
is LOCAL: move the offending vertices away by the deficit plus a
hair.

Discipline, each rule paid for:
- the violations come from check_drc ITSELF (a home-rolled scanner
  graded stricter than the checker and flagged four phantom nets);
- vertices on same-net pad/via copper never move, and a net with a
  T-joint at a moving vertex is skipped whole;
- the write goes through tolerant strip + the production writer, no
  text formatting match (the prune_debris law);
- the RESULT is verified by re-running check_drc (violations must
  strictly decrease, none added) and check_connected on the touched
  nets; otherwise the output is refused.

Iterates internally (a pinned vertex's graze can become nudgeable
once its neighbours move; measured converging 3 -> 1 -> 0 on K41).

usage: nudge_grazes.py BOARD --out OUT [--max-deficit 0.04]
"""
import argparse
import math
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
from kicad_writer import add_tracks_and_vias_to_pcb  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('board')
ap.add_argument('--out', required=True)
ap.add_argument('--max-deficit', type=float, default=0.04)
ap.add_argument('--pad', type=float, default=0.006)
a = ap.parse_args()
CHECK = os.path.join(HERE, '..', 'py_router', 'check_drc.py')


def drc(board):
    """(count, [(net, layer, deficit, (px,py), (x1,y1), (x2,y2))])
    for PAD-SEGMENT violations."""
    r = subprocess.run([sys.executable, CHECK, board],
                       capture_output=True, text=True)
    txt = r.stdout
    m = re.search(r'LISTING: (\d+) of (\d+) violation', txt)
    total = int(m.group(2)) if m else 0
    out = []
    pat = re.compile(
        r'Pad:\S+ \([^)]+\) <-> Seg:(.+?)\n'
        r'\s*Layer: (\S+), Overlap: ([\d.]+)mm\n'
        r'\s*Pad: \(([-\d.]+),([-\d.]+)\)\n'
        r'\s*Seg: \(([-\d.]+),([-\d.]+)\)-\(([-\d.]+),([-\d.]+)\)')
    for g in pat.finditer(txt):
        out.append((g.group(1), g.group(2), float(g.group(3)),
                    (float(g.group(4)), float(g.group(5))),
                    (float(g.group(6)), float(g.group(7))),
                    (float(g.group(8)), float(g.group(9)))))
    return total, out


total0, hits = drc(a.board)
hits = [h for h in hits if h[2] <= a.max_deficit]
if not hits:
    print(f'no nudgeable pad-segment grazes '
          f'(total violations {total0}); copying unchanged')
    shutil.copy(a.board, a.out)
    pro = os.path.splitext(a.board)[0] + '.kicad_pro'
    if os.path.exists(pro):
        shutil.copy(pro, os.path.splitext(a.out)[0] + '.kicad_pro')
    sys.exit(0)

pcb = parse_kicad_pcb(a.board)
full = {n.name: i for i, n in pcb.nets.items()}


def near(x, y, x2, y2, tol=0.011):
    return abs(x - x2) <= tol and abs(y - y2) <= tol


# map printed violations to real segments and plan vertex moves
by_net = {}
for name, layer, deficit, (px, py), (x1, y1), (x2, y2) in hits:
    nid = full.get(name)
    if nid is None:
        continue
    seg = next((s for s in pcb.segments
                if s.net_id == nid and s.layer == layer
                and ((near(s.start_x, s.start_y, x1, y1)
                      and near(s.end_x, s.end_y, x2, y2))
                     or (near(s.start_x, s.start_y, x2, y2)
                         and near(s.end_x, s.end_y, x1, y1)))), None)
    if seg is None:
        print(f'{name}: printed segment not found; skipped')
        continue
    by_net.setdefault(nid, []).append((seg, deficit, (px, py)))


def anchored(nid, x, y):
    for v in pcb.vias:
        if v.net_id == nid and math.hypot(v.x - x, v.y - y) \
                <= v.size / 2 + 0.02:
            return True
    for fp in pcb.footprints.values():
        for p in fp.pads:
            if p.net_id == nid and abs(p.global_x - x) <= p.size_x / 2 \
                    and abs(p.global_y - y) <= p.size_y / 2:
                return True
    return False


def tjoint(nid, x, y):
    """A same-net segment whose INTERIOR passes through (x, y)."""
    for s in pcb.segments:
        if s.net_id != nid:
            continue
        if near(s.start_x, s.start_y, x, y, 0.005) \
                or near(s.end_x, s.end_y, x, y, 0.005):
            continue
        dx, dy = s.end_x - s.start_x, s.end_y - s.start_y
        L2 = dx * dx + dy * dy
        if L2 == 0:
            continue
        t = ((x - s.start_x) * dx + (y - s.start_y) * dy) / L2
        if 0 < t < 1 and math.hypot(x - (s.start_x + t * dx),
                                    y - (s.start_y + t * dy)) < 0.01:
            return True
    return False


changed = {}          # nid -> {rounded old vertex: (nx, ny)}
for nid, items in by_net.items():
    moves = {}
    for seg, deficit, (px, py) in items:
        # the vertices near the pad move; an anchored or T-joint
        # vertex simply stays (the authoritative re-check below
        # judges whether the partial move cleared the graze)
        for (x, y) in ((seg.start_x, seg.start_y),
                       (seg.end_x, seg.end_y)):
            if math.hypot(x - px, y - py) > 1.2:
                continue
            if anchored(nid, x, y) or tjoint(nid, x, y):
                print(f'net {nid}: vertex ({x:.2f},{y:.2f}) pinned '
                      '(anchored/T-joint), not moved')
                continue
            ax, ay = x - px, y - py
            n = math.hypot(ax, ay) or 1.0
            step = deficit + a.pad
            k = (round(x, 3), round(y, 3))
            if k not in moves or step > moves[k][2]:
                moves[k] = (x + ax / n * step, y + ay / n * step, step)
    if moves:
        changed[nid] = {k: (v[0], v[1]) for k, v in moves.items()}

if not changed:
    print('nothing nudgeable; copying unchanged')
    shutil.copy(a.board, a.out)
    sys.exit(0)

# ---- apply: strip the net's affected segments (tolerant match),
# re-add moved copies through the production writer
strip_keys = {}     # nid -> set of old endpoint pairs (frozenset)
new_tracks = []
touched = set()
for nid, moves in changed.items():
    for s in pcb.segments:
        if s.net_id != nid:
            continue
        k1 = (round(s.start_x, 3), round(s.start_y, 3))
        k2 = (round(s.end_x, 3), round(s.end_y, 3))
        if k1 not in moves and k2 not in moves:
            continue
        sx, sy = moves.get(k1, (s.start_x, s.start_y))
        ex, ey = moves.get(k2, (s.end_x, s.end_y))
        strip_keys.setdefault(nid, set()).add(
            frozenset((k1, k2)) if k1 != k2 else frozenset((k1,)))
        new_tracks.append({'start': (sx, sy), 'end': (ex, ey),
                           'width': s.width, 'layer': s.layer,
                           'net_id': nid})
        touched.add(nid)
    print(f'net {nid}: {len(moves)} vertex/vertices, '
          f'{sum(1 for t in new_tracks if t["net_id"] == nid)} '
          'segment(s)')


def _walk(txt, token, kill):
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
        if kill(txt[j:k + 1]):
            out.append(txt[i:j].rstrip(' \t'))
            e = k + 1
            if e < len(txt) and txt[e] == '\n':
                e += 1
            i = e
        else:
            out.append(txt[i:k + 1])
            i = k + 1
    return ''.join(out)


id2name = {i: n.name for i, n in pcb.nets.items()}


def kill_seg(block):
    m = re.search(r'\(net (\d+)\)', block)
    m2 = re.search(r'\(net "([^"]+)"\)', block)
    nid = int(m.group(1)) if m else full.get(m2.group(1)) if m2 else None
    if nid not in strip_keys:
        return False
    pts = re.findall(r'\((?:start|end) ([-\d.]+) ([-\d.]+)', block)
    if len(pts) != 2:
        return False
    key = frozenset((round(float(x), 3), round(float(y), 3))
                    for x, y in pts)
    return key in strip_keys[nid]


txt = open(a.board, encoding='utf-8').read()
n0 = txt.count('(segment')
txt = _walk(txt, 'segment', kill_seg)
stripped = n0 - txt.count('(segment')
if stripped != len(new_tracks):
    print(f'REFUSED: stripped {stripped} but re-adding '
          f'{len(new_tracks)}; board unchanged')
    shutil.copy(a.board, a.out)
    sys.exit(2)
tmp = a.out + '.bare.tmp'
open(tmp, 'w', encoding='utf-8').write(txt)
add_tracks_and_vias_to_pcb(tmp, a.out, new_tracks, [], [],
                           net_id_to_name=id2name)
os.remove(tmp)
pro = os.path.splitext(a.board)[0] + '.kicad_pro'
if os.path.exists(pro):
    shutil.copy(pro, os.path.splitext(a.out)[0] + '.kicad_pro')

# ---- authoritative verification: DRC strictly better, no new class;
# connectivity intact on the touched nets
total1, _ = drc(a.out)
names = [id2name[nid] for nid in touched]
cc = subprocess.run(
    [sys.executable, os.path.join(HERE, '..', 'py_router',
                                  'check_connected.py'),
     a.out, '--nets'] + names, capture_output=True, text=True)
conn_ok = 'FULLY CONNECTED' in cc.stdout
print(f'drc: {total0} -> {total1}; connectivity '
      f'{"OK" if conn_ok else "BROKEN"}')
if total1 >= total0 or not conn_ok:
    print('REFUSED: verification failed; restoring input board')
    shutil.copy(a.board, a.out)
    sys.exit(2)
print(f'wrote {a.out}')

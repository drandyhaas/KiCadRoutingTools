#!/usr/bin/env python3
"""Per-net audit of an emitted board: every track and via of every net,
checked against what the plan said and against what is legal.

Per net it CHECKS:
  * the copper walks tooth -> ball as ONE chain: no gap, no branch, no
    stray fragment, every layer change sitting on a via
  * every EMITTED segment is octilinear (the board's own fanout stubs
    are at arbitrary angles and are excluded -- they are input, not our
    output)
  * the via count against the homotopy floor 2*(K - LIS)
  * emitted length against the octilinear ideal for its own lane, so
    staircase residue from the octify pass shows up as a number

Endpoint matching uses a 5 um tolerance: the emitter and the #536
smoother legitimately disagree in the last micron (64.909 vs 64.908),
which is a 1 um offset on a 127 um track -- an overlap, not a gap.
Exact rounding reported those as broken chains.

A via-in-pad via joins B copper to the PAD, so "two layers of segments"
is the wrong test for it; pads are counted as layer contacts."""
import math
import os
import subprocess
import sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import braid as te  # noqa: E402

SNAP = 0.005
JOIN = 0.020        # points within 20 um are the same node


def key(p):
    return (round(p[0] / SNAP), round(p[1] / SNAP))


def cluster(points, tol=JOIN):
    """Union points within `tol` into nodes. Bucketing on a fixed grid
    splits a pair that straddles a bucket edge -- measured on SA9, where
    the segment end, the via and the pad sat 8 um apart across three
    5 um buckets and the chain read as broken. Distance clustering has
    no boundary."""
    pts = sorted(set(points))
    parent = list(range(len(pts)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for a in range(len(pts)):
        for b in range(a + 1, len(pts)):
            if pts[b][0] - pts[a][0] > tol:
                break
            if math.hypot(pts[a][0] - pts[b][0],
                          pts[a][1] - pts[b][1]) <= tol:
                ra, rb = find(a), find(b)
                if ra != rb:
                    parent[ra] = rb
    return {pts[i]: find(i) for i in range(len(pts))}


board = sys.argv[1]
spec = sys.argv[2] if len(sys.argv) > 2 else None
if spec and not spec.startswith('S'):
    names = subprocess.run([sys.executable,
                            os.path.join(HERE, 'coherent_nets.py'), spec],
                           capture_output=True, text=True).stdout.strip()
    names = [n for n in names.split(',') if n]
else:
    names = [n for n in (spec or '').split(',') if n]

pcb = parse_kicad_pcb(board)
base = parse_kicad_pcb(os.path.join(HERE, 'fb_t2q_base.kicad_pcb'))
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
ends = te.endpoints(base, names, byname)

print(f'== {os.path.basename(board)}  K={len(names)}')
launch = sorted(names, key=lambda n: ends[n][0][1])
target = sorted(names, key=lambda n: (ends[n][1][1], ends[n][1][0]))
trank = {n: i for i, n in enumerate(target)}
ranks = [trank[n] for n in launch]
lis = len(te.lis_keep(ranks))
floor = 2 * (len(names) - lis)
print(f'   ranks {ranks}  LIS {lis}  via floor 2*(K-LIS) = {floor}')

tot_v = 0
bad_nets = []
rows = []
for nm in names:
    nid = byname[nm][0]
    bsegs = {(key((s.start_x, s.start_y)), key((s.end_x, s.end_y)),
              s.layer) for s in base.segments if s.net_id == nid}
    segs = [s for s in pcb.segments if s.net_id == nid]
    vias = [v for v in pcb.vias if v.net_id == nid]
    tot_v += len(vias)
    problems = []

    def is_base(s):
        a, b = key((s.start_x, s.start_y)), key((s.end_x, s.end_y))
        return (a, b, s.layer) in bsegs or (b, a, s.layer) in bsegs

    for s in segs:
        if is_base(s):
            continue
        dx, dy = s.end_x - s.start_x, s.end_y - s.start_y
        if math.hypot(dx, dy) < 1e-9:
            problems.append('zero-length segment')
            continue
        ang = math.degrees(math.atan2(dy, dx)) % 45.0
        if min(ang, 45.0 - ang) > 0.6:
            problems.append(f'non-octilinear at ({s.start_x:.2f},'
                            f'{s.start_y:.2f})')
    allpts = []
    for s in segs:
        allpts += [(s.start_x, s.start_y), (s.end_x, s.end_y)]
    allpts += [(v.x, v.y) for v in vias]
    allpts += [(p.global_x, p.global_y) for p in pcb.nets[nid].pads]
    node = cluster(allpts)
    deg = defaultdict(int)
    for s in segs:
        a = node[(s.start_x, s.start_y)]
        b = node[(s.end_x, s.end_y)]
        if a == b:
            continue        # shorter than the join tolerance: a point
        deg[a] += 1
        deg[b] += 1
    vkeys = {node[(v.x, v.y)] for v in vias}
    padk = {node[(p.global_x, p.global_y)] for p in pcb.nets[nid].pads}
    free = [p for p, d in deg.items() if d == 1]
    stray = [p for p in free if p not in padk and p not in vkeys]
    if len(stray) > 0:
        problems.append(f'{len(stray)} dangling end(s) not on a pad/via')
    branch = [p for p, d in deg.items() if d > 2]
    if branch:
        problems.append(f'{len(branch)} branch point(s)')
    bylayer = defaultdict(set)
    for s in segs:
        bylayer[s.layer].add(node[(s.start_x, s.start_y)])
        bylayer[s.layer].add(node[(s.end_x, s.end_y)])
    if not deg:
        problems.append('no copper')
    layers = sorted(bylayer)
    if len(layers) > 1:
        for p in set.intersection(*(bylayer[L] for L in layers)):
            if p not in vkeys:
                problems.append('layer change with no via')
                break
    for v in vias:
        vk = node[(v.x, v.y)]
        touch = sum(1 for L in layers if vk in bylayer[L])
        if vk in padk:
            touch += 1              # via-in-pad: the pad is the contact
        if touch < 2:
            problems.append(f'via at ({v.x:.2f},{v.y:.2f}) joins nothing')
    lf = sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
             for s in segs if s.layer == 'F.Cu')
    lb = sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
             for s in segs if s.layer == 'B.Cu')
    # octilinear ideal from the tooth to the ball (a 45 leg + a straight
    # leg is the shortest octilinear path between two points)
    (tx, ty), (bx, by) = ends[nm][0], ends[nm][1]
    dx, dy = abs(bx - tx), abs(by - ty)
    ideal = max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)
    if problems:
        bad_nets.append(nm)
    rows.append((nm, len(vias), len(segs), lf + lb, ideal, problems))

print(f'{"net":9s} vias segs  len_mm  ideal  over   issues')
for nm, nv, ns, ln, ideal, problems in rows:
    over = 100.0 * (ln / ideal - 1.0) if ideal > 0 else 0.0
    print(f'{nm:9s} {nv:4d} {ns:4d} {ln:7.2f} {ideal:6.2f} {over:+5.0f}%  '
          + ('OK' if not problems else '; '.join(problems[:2])))
print(f'TOTAL vias {tot_v} (floor {floor}, '
      f'{"AT FLOOR" if tot_v == floor else f"+{tot_v - floor}"})'
      f'   nets with structural issues: {len(bad_nets)}'
      + (f' -> {bad_nets}' if bad_nets else ''))

#!/usr/bin/env python3
"""Which entry kind is exhausted? Replays the emitter's own entry
assignment (F street -> via-in-pad -> B dogbone, in ball order) and
reports the mix, plus WHY each candidate was refused for the first net
that fails. Read-only: it imports topo_emit's helpers rather than
re-deriving them, so it cannot drift from the real rule."""
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import topo_emit as te  # noqa: E402
import topo_strings as ts  # noqa: E402

K = sys.argv[1] if len(sys.argv) > 1 else '32'
names = subprocess.run([sys.executable,
                        os.path.join(HERE, 'coherent_nets.py'), K],
                       capture_output=True, text=True).stdout.strip()
names = [n for n in names.split(',') if n]
pcb = parse_kicad_pcb(os.path.join(HERE, 'fb_t2q_base.kicad_pcb'))
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
ends = te.endpoints(pcb, names, byname)
kids = {byname[n][0] for n in names}
comps = {ends[n][2] for n in names}
pads = [p for c in comps for p in pcb.footprints[c].pads]
fx0 = min(p.global_x for p in pads)
PITCH = 0.8
half = PITCH / 2
x1 = fx0 - PITCH * 0.8
obsF = {n: te.build_obstacles(pcb, byname[n][0], kids, 'F.Cu')
        for n in names}
obsB = {}
seg_min = te.TRACK + te.CLEAR
via_seg_min = te.VIA_SIZE / 2 + te.TRACK / 2 + te.CLEAR
entry, placed_f, placed_vias, placed_b = {}, [], [], []


def f_ok(nm, segs):
    for (p, q) in segs:
        if p == q:
            continue
        if not obsF[nm].seg_clear(p, q):
            return False
        for (c, e) in placed_f:
            if ts.seg_seg_dist(p, q, c, e) < seg_min:
                return False
        for v in placed_vias:
            if ts.seg_pt_dist(p, q, v) < via_seg_min:
                return False
    return True


def b_ok(nm, sx, sy):
    nid = byname[nm][0]
    if nm not in obsB:
        obsB[nm] = te.build_obstacles(pcb, nid, kids, 'B.Cu')
    run = ((x1, sy), (sx, sy))
    if not obsB[nm].seg_clear(*run):
        return 'B run hits static copper'
    vpad = (te.VIA_SIZE - te.TRACK) / 2
    for o in (obsB[nm], obsF[nm]):
        vv = o.point_violation((sx, sy), pad=vpad)
        if vv and vv[0] > 0:
            return 'via barrel hits static copper'
    for (c, e) in placed_b:
        if ts.seg_seg_dist(run[0], run[1], c, e) < seg_min:
            return 'B run too close to an earlier B run'
        if ts.seg_pt_dist(c, e, (sx, sy)) < via_seg_min:
            return 'site too close to an earlier B run'
    return None


ball_order = sorted(names, key=lambda nm: (ends[nm][1][1], ends[nm][1][0]))
# pass 1: F streets, exactly as the emitter does
for nm in ball_order:
    bx, by = ends[nm][1]
    cands = []
    if bx <= fx0 + 0.01:
        cands.append(('direct', by, [((x1, by), (bx, by))]))
    for sy in (by - half, by + half):
        cands.append(('street', sy, [((x1, sy), (bx, sy)),
                                     ((bx, sy), (bx, by))]))
    for kind, ey, segs in cands:
        if any(abs(ey - e[1]) < 0.3 for e in entry.values()
               if e[0] == 'F'):
            continue
        if f_ok(nm, segs):
            entry[nm] = ('F', ey)
            placed_f.extend(s for s in segs if s[0] != s[1])
            break
n_f = len(entry)
# pass 2: via-in-pad then B dogbone
n_vip = n_dog = 0
failed = None
for nm in ball_order:
    if nm in entry:
        continue
    bx, by = ends[nm][1]
    site = None
    why = []
    ok = all(math.hypot(bx - v[0], by - v[1]) > te.VIA_SIZE + te.CLEAR
             for v in placed_vias)
    if ok:
        r = b_ok(nm, bx, by)
        if r is None:
            site = (bx, by)
            n_vip += 1
        else:
            why.append(f'VIP: {r}')
    else:
        why.append('VIP: too close to an earlier via')
    if site is None:
        sx = bx - half
        for sy in (by - half, by + half):
            ok = all(ts.seg_pt_dist(p, q, (sx, sy)) > via_seg_min
                     for (p, q) in placed_f)
            if not ok:
                why.append(f'dogbone y={sy:.2f}: site hits an F stub')
                continue
            if not all(math.hypot(sx - v[0], sy - v[1]) >
                       te.VIA_SIZE + te.CLEAR for v in placed_vias):
                why.append(f'dogbone y={sy:.2f}: site hits an earlier via')
                continue
            if not f_ok(nm, [((sx, sy), (bx, by))]):
                why.append(f'dogbone y={sy:.2f}: F stub blocked')
                continue
            r = b_ok(nm, sx, sy)
            if r is not None:
                why.append(f'dogbone y={sy:.2f}: {r}')
                continue
            site = (sx, sy)
            n_dog += 1
            placed_f.append(((sx, sy), (bx, by)))
            break
    if site is None:
        failed = (nm, why)
        break
    entry[nm] = ('B', site)
    placed_vias.append(site)
    placed_b.append(((x1, site[1]), site))

print(f'K={K}: {len(names)} nets -> F street {n_f}, via-in-pad {n_vip}, '
      f'B dogbone {n_dog}, placed {len(entry)}')
if failed:
    nm, why = failed
    print(f'FIRST FAILURE: {nm} (ball {ends[nm][1][0]:.2f},'
          f'{ends[nm][1][1]:.2f})')
    for w in why:
        print('   refused ->', w)
else:
    print('all nets placed')

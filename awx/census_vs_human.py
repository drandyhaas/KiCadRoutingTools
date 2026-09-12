#!/usr/bin/env python3
"""#622 per-net census: our board vs the human, scoped to a K-set.

For each net: via count, copper mm, layer mm split, and via position
class (SRC = within src_r of U1 center, DST = within dst_r of DU1,
MID = the field between). The human's smarter planning should show up
as WHERE the vias are (2/net constant, at the ends) and which nets
ride which layer.

usage: census_vs_human.py OURS.kicad_pcb HUMAN.kicad_pcb K
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(
    os.path.abspath(__file__)), '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402
import subprocess  # noqa: E402

ours_p, human_p, K = sys.argv[1], sys.argv[2], sys.argv[3]
nets = subprocess.run(
    [sys.executable, os.path.join(os.path.dirname(
        os.path.abspath(__file__)), 'coherent_nets.py'), K],
    capture_output=True, text=True).stdout.strip().split(',')


SRC_R = DST_R = None      # set from the arrays' own reach (see _cls)


def _cls(du, dd):
    """SRC / DST / MID, the three classes the docstring promises.

    It used to be `'S' if du < dd else 'D'` -- nearest-of-two, no radii,
    no MID at all -- so a via in the dead centre of the field was
    reported as a destination-end via and the claim this tool exists to
    test ("the human's vias are at the ENDS") could not be falsified on
    any board. A via is at an end when it is within that array's reach;
    anything else is mid-field.
    """
    if SRC_R and du <= SRC_R:
        return 'S'
    if DST_R and dd <= DST_R:
        return 'D'
    if not SRC_R and not DST_R:
        return 'S' if du < dd else 'D'
    return 'M'


def short(name):
    return name.rsplit('/', 1)[-1]


def census(path):
    pcb = parse_kicad_pcb(path)
    u1 = pcb.footprints.get('U1')
    du1 = pcb.footprints.get('DU1')
    by = {}
    id2short = {nid: short(n.name) for nid, n in pcb.nets.items()}
    want = set(nets)
    for s in pcb.segments:
        nm = id2short.get(s.net_id)
        if nm not in want:
            continue
        d = by.setdefault(nm, {'vias': 0, 'mm': 0.0, 'lay': {},
                               'vpos': []})
        L = math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
        d['mm'] += L
        d['lay'][s.layer] = d['lay'].get(s.layer, 0.0) + L
    for v in pcb.vias:
        nm = id2short.get(v.net_id)
        if nm not in want:
            continue
        d = by.setdefault(nm, {'vias': 0, 'mm': 0.0, 'lay': {},
                               'vpos': []})
        d['vias'] += 1
        du = math.hypot(v.x - u1.x, v.y - u1.y) if u1 else 999
        dd = math.hypot(v.x - du1.x, v.y - du1.y) if du1 else 999
        d['vpos'].append(_cls(du, dd))
        d.setdefault('vxy', []).append((round(v.x, 2), round(v.y, 2),
                                        _cls(du, dd)))
    if u1 and du1:
        # an "end" via is one inside its own array's reach: a third of the
        # centre-to-centre span, so the middle third of the field is MID
        global SRC_R, DST_R
        span = math.hypot(du1.x - u1.x, du1.y - u1.y)
        SRC_R = DST_R = span / 3.0
    return by, (u1.x, u1.y) if u1 else None, \
        (du1.x, du1.y) if du1 else None


ours, u1o, duo = census(ours_p)
hum, u1h, duh = census(human_p)
print(f'U1 ours={u1o} human={u1h}  DU1 ours={duo} human={duh}')
rows = []
for n in nets:
    o = ours.get(n, {'vias': 0, 'mm': 0, 'lay': {}, 'vpos': []})
    h = hum.get(n, {'vias': 0, 'mm': 0, 'lay': {}, 'vpos': []})
    rows.append((o['vias'] - h['vias'], n, o, h))
rows.sort(reverse=True)
to = th = 0


def laystr(lay):
    tot = sum(lay.values()) or 1
    parts = []
    for k in sorted(lay, key=lambda x: -lay[x]):
        parts.append(f'{k[0]}{lay[k] / tot * 100:.0f}')
    return '/'.join(parts)


print(f'{"net":8s} {"dv":>3s} | {"ours":>4s} {"mm":>6s} '
      f'{"pos":8s} {"layers":12s} | {"hum":>3s} {"mm":>6s} '
      f'{"pos":8s} {"layers":12s}')
for dv, n, o, h in rows:
    to += o['vias']
    th += h['vias']
    print(f'{n:8s} {dv:+3d} | {o["vias"]:4d} {o["mm"]:6.1f} '
          f'{"".join(sorted(o["vpos"])):8s} {laystr(o["lay"]):12s} | '
          f'{h["vias"]:3d} {h["mm"]:6.1f} '
          f'{"".join(sorted(h["vpos"])):8s} {laystr(h["lay"]):12s}')
print(f'TOTAL ours={to} human={th}  diff={to - th}')

#!/usr/bin/env python3
"""Will the braid actually hand off cleanly to the chosen escapes?

The selector picks, per ball, an exit POINT and the LAYER the net is
travelling on when it gets there. The braid delivers each net to the
splice line on the layer its own schedule gives it -- F for a keeper,
B for a diver. Three ways that handoff can fail, all counted here:

  WRONG PLACE   the exit is not on the side the corridor arrives from.
                The braid has no mechanism to deliver anywhere else, so
                these nets have no corridor at all yet.
  WRONG LAYER   the corridor arrives on the other layer from the one the
                escape starts on. Recoverable, but it costs a via at the
                exit that nothing has counted.
  WRONG ORDER   the exit order is not one the corridor can realise
                without extra crossings -- measured as the via floor.

usage: probe_handoff.py [K]
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import topo_emit as te  # noqa: E402
import escape_moves as em  # noqa: E402
import select_moves as sm  # noqa: E402
import detect_buses as db  # noqa: E402

K = sys.argv[1] if len(sys.argv) > 1 else '21'
names = subprocess.run([sys.executable,
                        os.path.join(HERE, 'coherent_nets.py'), K],
                       capture_output=True, text=True).stdout.strip()
names = [n for n in names.split(',') if n]
pcb = parse_kicad_pcb(os.path.join(HERE, 'fb_t2q_base.kicad_pcb'))
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
ends = te.endpoints(pcb, names, byname)
kids = {byname[n][0] for n in names}
cache = {}


def obs(nid, layer):
    if (nid, layer) not in cache:
        cache[(nid, layer)] = te.build_obstacles(pcb, nid, kids, layer)
    return cache[(nid, layer)]


LAYERS = ('F.Cu', 'B.Cu')
menu, launch = {}, {}
for nm in names:
    nid = byname[nm][0]
    fp = pcb.footprints[ends[nm][2]]
    bx, by = ends[nm][1]
    pad = min(fp.pads, key=lambda p: (p.global_x - bx) ** 2
              + (p.global_y - by) ** 2)
    menu[nm] = em.enumerate_moves(
        pad, em.grid_of(fp), LAYERS,
        lambda p, q, L, _n=nid: obs(_n, L).seg_clear(p, q),
        lambda p, L, _n=nid: not (obs(_n, L).point_violation(
            p, pad=(te.VIA_SIZE - te.TRACK) / 2) or [0])[0])
    launch[nm] = ends[nm][0]
grid0 = em.grid_of(pcb.footprints[ends[names[0]][2]])
paths = db.taut_paths(names, ends, lambda nm: obs(byname[nm][0], 'F.Cu'))
buses = db.cluster(names, paths)
# the tooth layer is what the corridor STARTS the net on
tooth_layer = {}
for nm in names:
    nid = byname[nm][0]
    tp = ends[nm][0]
    tooth_layer[nm] = next(
        (s.layer for s in pcb.segments if s.net_id == nid
         and (abs(s.start_x - tp[0]) + abs(s.start_y - tp[1]) < 0.005
              or abs(s.end_x - tp[0]) + abs(s.end_y - tp[1]) < 0.005)),
        'F.Cu')

choice, unplaced = sm.select(menu, launch, keep_out=grid0.bbox,
                             buses=buses, tooth_layer=tooth_layer,
                             log=(print if '-v' in sys.argv else None))

print(f'K={K}: {len(names)} nets, {len(buses)} buses\n')
tot_mismatch = tot_extra = 0
for bus in sorted(buses, key=len, reverse=True):
    if not all(n in choice for n in bus):
        continue
    side = choice[bus[0]].direction
    axis = 1 if side in ('left', 'right') else 0
    lo = sorted(bus, key=lambda n: launch[n][1])
    li = {n: i for i, n in enumerate(lo)}
    tgt = sorted(bus, key=lambda n: (round(choice[n].exit_pt[axis], 3),
                                     li[n]))
    tr = {n: i for i, n in enumerate(tgt)}
    ranks = [tr[n] for n in lo]
    # ask the SELECTOR which layer it hands each net over on, rather
    # than recomputing it here: this probe used its own unweighted LIS
    # and so reported a diver set the selector was not using
    dl = sm.delivered_layers(choice, buses, launch, tooth_layer)
    divers = {n for n in bus if dl[n] != tooth_layer[n]}
    mism = [n for n in bus if dl[n] != choice[n].layer]
    floor = 2 * (len(bus) - len(te.lis_keep(ranks)))
    tot_mismatch += len(mism)
    tot_extra += len(mism)
    print(f'bus[{len(bus):2d}] exits {side:5s}: '
          f'{len(divers)} divers, corridor via floor {floor}')
    print(f'    layer handoff: {len(bus) - len(mism)}/{len(bus)} arrive '
          f'on the layer their escape starts on'
          + (f'; MISMATCH on {",".join(sorted(mism)[:6])}' if mism else ''))
print(f'\nWRONG PLACE: the braid only delivers to the corridor side.')
by_side = {}
for n, m in choice.items():
    by_side.setdefault(m.direction, []).append(n)
for d in sorted(by_side):
    print(f'    {d:5s}: {len(by_side[d]):2d} nets'
          + ('   <- the existing corridor reaches these'
             if d == 'left' else
             '   <- NO corridor mechanism yet'))
print(f'\nWRONG LAYER: {tot_mismatch} nets would need an extra via at '
      f'their exit ({tot_extra} vias not counted in any total so far)')

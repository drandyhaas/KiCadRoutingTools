#!/usr/bin/env python3
"""Render the whole plan: U1 escape -> corridor -> berth escape -> ball.

  Eco1  white   the CORRIDOR LEG the braid must draw, source exit to
                berth exit, routed AROUND the destination array.
  Eco2  yellow  the ESCAPES at both ends, plus a small box at each exit
                point. A DIAMOND at the source end marks a tooth this
                plan MOVED from where the fanout put it.
  Cmts  orange  what the plan still has to pay for. A cross on every
                net outside the crossing-free set -- those are the nets
                that must dive and come back, 2 vias each, and they are
                the whole via floor. A box where the corridor hands a
                net over on the opposite layer from the one its berth
                escape starts on.

There is deliberately no "the corridor is on the left" here any more.
That was a fact about one board, and it was still in this file marking
every exit on the other three sides as a failure -- while the selector
had long since been choosing all four.

usage: make_handoff.py OUT.kicad_pcb [K]
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
import plan_ends as pe  # noqa: E402

out_path = sys.argv[1]
rest = [a for a in sys.argv[2:] if not a.startswith('-')]
K = rest[0] if rest else '21'
base = os.path.join(HERE, 'fb_t2q_base.kicad_pcb')
# --on BOARD draws the overlay onto ANOTHER board -- normally the one the
# braid was emitted to -- while still PLANNING from the bare bench. That
# is the comparison worth looking at: the braid copper that exists next
# to the corridor the plan says it should have drawn. Planning from the
# emitted board instead would plan around the braid's own copper and
# answer a different question.
on_path = next((a.split('=', 1)[1] for a in sys.argv
                if a.startswith('--on=')), None)
names = subprocess.run([sys.executable,
                        os.path.join(HERE, 'coherent_nets.py'), K],
                       capture_output=True, text=True).stdout.strip()
names = [n for n in names.split(',') if n]
pcb = parse_kicad_pcb(base)
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
ends = te.endpoints(pcb, names, byname)
kids = {byname[n][0] for n in names}
cache = {}


def obs(nid, layer):
    if (nid, layer) not in cache:
        cache[(nid, layer)] = te.build_obstacles(pcb, nid, kids, layer)
    return cache[(nid, layer)]


LAYERS = ('F.Cu', 'B.Cu')
dmenu, launch, src_pad = {}, {}, {}
for nm in names:
    nid, net = byname[nm]
    fp = pcb.footprints[ends[nm][2]]
    bx, by = ends[nm][1]
    pad = min(fp.pads, key=lambda p: (p.global_x - bx) ** 2
              + (p.global_y - by) ** 2)
    dmenu[nm] = em.enumerate_moves(
        pad, em.grid_of(fp), LAYERS,
        lambda p, q, L, _n=nid: obs(_n, L).seg_clear(p, q),
        lambda p, L, _n=nid: not (obs(_n, L).point_violation(
            p, pad=(te.VIA_SIZE - te.TRACK) / 2) or [0])[0])
    launch[nm] = ends[nm][0]
    others = [p for p in net.pads if p.component_ref != ends[nm][2]]
    src_pad[nm] = others[0] if others else None

dgrid = em.grid_of(pcb.footprints[ends[names[0]][2]])
refs = {}
for nm in names:
    if src_pad[nm] is not None:
        refs[src_pad[nm].component_ref] = refs.get(
            src_pad[nm].component_ref, 0) + 1
sref = max(refs, key=refs.get)
sgrid = em.grid_of(pcb.footprints[sref])
smenu = {}
for nm in names:
    p = src_pad[nm]
    if p is None or p.component_ref != sref:
        continue
    nid = byname[nm][0]
    smenu[nm] = em.enumerate_moves(
        p, sgrid, LAYERS,
        lambda a, b, L, _n=nid: obs(_n, L).seg_clear(a, b),
        lambda a, L, _n=nid: not (obs(_n, L).point_violation(
            a, pad=(te.VIA_SIZE - te.TRACK) / 2) or [0])[0])

print('taut pre-routes...')
paths = db.taut_paths(names, ends, lambda nm: obs(byname[nm][0], 'F.Cu'))
buses = db.cluster(names, paths)
tooth0 = {}
for nm in names:
    nid = byname[nm][0]
    tp = ends[nm][0]
    tooth0[nm] = next(
        (s.layer for s in pcb.segments if s.net_id == nid
         and (abs(s.start_x - tp[0]) + abs(s.start_y - tp[1]) < 0.005
              or abs(s.end_x - tp[0]) + abs(s.end_y - tp[1]) < 0.005)),
        'F.Cu')

print('planning both ends...')
schoice, choice, lp, report = pe.plan_ends(
    smenu, dmenu, launch, sgrid.bbox, dgrid.bbox, buses=buses,
    tooth_layer0=tooth0)
for line in report:
    print(line)
geo = sm.Corridor(dgrid.bbox, lp)
corr = sm.corridors(choice)
tooth = dict(tooth0)
for n, m in schoice.items():
    tooth[n] = m.layer
delivered = sm.delivered_layers(choice, corr, geo, tooth)
# the crossing-free set over the WHOLE plan, not the union of the
# per-corridor ones. Taking the union ignores every crossing between
# corridors, so it marks fewer nets than must actually dive and the
# picture disagrees with the floor the plan reports (30 drawn against
# 48 planned at K51).
kept = set(geo.keep(list(choice), choice))

lines = []


def gr(p, q, layer):
    lines.append(f'  (gr_line (start {p[0]:.4f} {p[1]:.4f}) '
                 f'(end {q[0]:.4f} {q[1]:.4f}) '
                 f'(stroke (width 0.05) (type solid)) (layer "{layer}"))\n')


def cross(c, r, layer):
    gr((c[0] - r, c[1] - r), (c[0] + r, c[1] + r), layer)
    gr((c[0] - r, c[1] + r), (c[0] + r, c[1] - r), layer)


def box(c, r, layer):
    p = [(c[0] - r, c[1] - r), (c[0] + r, c[1] - r),
         (c[0] + r, c[1] + r), (c[0] - r, c[1] + r)]
    for a, b in zip(p, p[1:] + p[:1]):
        gr(a, b, layer)


def diamond(c, r, layer):
    p = [(c[0] - r, c[1]), (c[0], c[1] - r),
         (c[0] + r, c[1]), (c[0], c[1] + r)]
    for a, b in zip(p, p[1:] + p[:1]):
        gr(a, b, layer)


n_dive = n_layer = n_moved = 0
for nm in names:
    m = choice.get(nm)
    if m is None:
        continue
    for a, b in zip(geo.leg(nm, m), geo.leg(nm, m)[1:]):
        gr(a, b, 'Eco1.User')
    for (a, b, _L) in m.legs:
        gr(a, b, 'Eco2.User')
    box(m.exit_pt, 0.16, 'Eco2.User')
    sm_ = schoice.get(nm)
    if sm_ is not None:
        for (a, b, _L) in sm_.legs:
            gr(a, b, 'Eco2.User')
        if lp.get(nm) != launch.get(nm):
            diamond(sm_.exit_pt, 0.22, 'Eco2.User')
            n_moved += 1
    if nm not in kept:
        cross(m.exit_pt, 0.42, 'Cmts.User')
        n_dive += 1
    if delivered.get(nm) and delivered[nm] != m.layer:
        box(m.exit_pt, 0.30, 'Cmts.User')
        n_layer += 1

host = on_path or base
txt = open(host, encoding='utf-8').read()
k = txt.rstrip().rfind(')')
open(out_path, 'w').write(txt[:k] + ''.join(lines) + txt[k:])
pro = os.path.splitext(host)[0] + '.kicad_pro'
if not os.path.exists(pro):
    pro = os.path.splitext(base)[0] + '.kicad_pro'
if os.path.exists(pro):
    import shutil
    shutil.copy(pro, os.path.splitext(out_path)[0] + '.kicad_pro')
print(f'wrote {out_path}: {len(lines)} lines; '
      f'floor {2 * n_dive} ({n_dive} nets must dive, orange X), '
      f'{n_layer} layer mismatches (orange box), '
      f'{n_moved} teeth moved (yellow diamond)')

#!/usr/bin/env python3
"""Fan out the berth array in the directions the PLAN chose.

The plan picks, per ball, an escape direction and a travel layer. Until
now it picked them for nobody -- the production fanout was never told,
and chose its own with `preferred_escape_dirs`, which guesses "toward
this net's nearest off-footprint pad" one net at a time, blind to the
other nets and to the corridor they share.

This hands the plan's directions to `generate_bga_fanout` through
escape_dir_hints and then CHECKS THE COPPER, because a hint accepted is
not a hint obeyed: the fanout still has to find a free channel, and
when it cannot it silently takes another direction (or drops the ball).
The direction of each emitted escape is measured from the track that
actually leaves the pad, and compared with what was asked for.

The negative control is the same run with the hints withheld
(--no-hints), which is what the production fanout does today.

usage: fanout_from_plan.py OUT.kicad_pcb [K] [--no-hints] [--both-ends]
"""
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
from kicad_writer import add_tracks_and_vias_to_pcb  # noqa: E402
from bga_fanout import generate_bga_fanout  # noqa: E402
import topo_emit as te  # noqa: E402
import escape_moves as em  # noqa: E402
import select_moves as sm  # noqa: E402
import detect_buses as db  # noqa: E402
import plan_ends as pe  # noqa: E402

out_path = sys.argv[1]
rest = [a for a in sys.argv[2:] if not a.startswith('-')]
K = rest[0] if rest else '21'
NO_HINTS = '--no-hints' in sys.argv
BOTH = '--both-ends' in sys.argv
METHOD = next((a.split('=', 1)[1] for a in sys.argv
               if a.startswith('--escape-method=')), 'auto')
# Restrict the plan to one exit side. The braid delivers from the WEST
# splice line and has no mechanism to reach any other face, so a chain
# that means to hand off to it has to plan within that limit; without
# this the plan spreads over all four sides and most nets have no
# corridor at all. Not a fact about this board -- a fact about what the
# braid can currently do.
ONLY = next((set(a.split('=', 1)[1].split(',')) for a in sys.argv
             if a.startswith('--dirs=')), None)
# The plane-drop pass collides with the decoupling caps under the array
# on this bench -- 4 pad-via violations present with hints, without
# hints, and with no braid at all. It is a defect of that pass, not of
# anything here, so it gets a switch: with it off, what the DRC count
# reports is the escapes and the braid alone.
NO_DROP = '--no-plane-drop' in sys.argv
# Negative control for the exit-line hint specifically: directions
# still applied, gaps left to the fanout.
NO_LINES = '--no-lines' in sys.argv
base = os.path.join(HERE, 'fb_t2q_base.kicad_pcb')

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
dmenu, launch, src_pad, dst_pad = {}, {}, {}, {}
for nm in names:
    nid, net = byname[nm]
    fp = pcb.footprints[ends[nm][2]]
    bx, by = ends[nm][1]
    pad = min(fp.pads, key=lambda p: (p.global_x - bx) ** 2
              + (p.global_y - by) ** 2)
    dst_pad[nm] = pad
    dmenu[nm] = em.enumerate_moves(
        pad, em.grid_of(fp), LAYERS,
        lambda p, q, L, _n=nid: obs(_n, L).seg_clear(p, q),
        lambda p, L, _n=nid: not (obs(_n, L).point_violation(
            p, pad=(te.VIA_SIZE - te.TRACK) / 2) or [0])[0])
    launch[nm] = ends[nm][0]
    others = [p for p in net.pads if p.component_ref != ends[nm][2]]
    src_pad[nm] = others[0] if others else None

dref = ends[names[0]][2]
dgrid = em.grid_of(pcb.footprints[dref])
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

print('planning...')
if ONLY:
    dmenu = {n: [m for m in ms if m.direction in ONLY]
             for n, ms in dmenu.items()}
    _empty = [n for n, ms in dmenu.items() if not ms]
    if _empty:
        print(f'  {len(_empty)} net(s) have NO move on {",".join(ONLY)}: '
              + ','.join(_empty[:8]))
schoice, choice, lp, report = pe.plan_ends(
    smenu, dmenu, launch, sgrid.bbox, dgrid.bbox, buses=buses,
    tooth_layer0=tooth0)
for line in report:
    print(line)

hints, lines = {}, {}
for nm, m in choice.items():
    p = dst_pad[nm]
    k = (round(p.global_x, 3), round(p.global_y, 3))
    hints[k] = m.direction
    # the exit LINE: which row gap (left/right) or column gap (up/down)
    # the plan put this net in. The side alone leaves the fanout free to
    # pick the gap, and the gap is what the launch->exit permutation --
    # and so the via floor -- is actually made of.
    lines[k] = m.exit_pt[1] if m.direction in ('left', 'right') \
        else m.exit_pt[0]
print(f'\nplan: {len(hints)} berth escape directions '
      + ', '.join(f'{d}:{sum(1 for v in hints.values() if v == d)}'
                  for d in sorted(set(hints.values()))))

fp = pcb.footprints[dref]
print(f'\nfanning out {dref} ({len(fp.pads)} pads), method {METHOD}'
      + ('  WITHOUT hints (negative control)' if NO_HINTS else
         '  with the plan\'s directions'))
tracks, vias_add, vias_rm, failed = generate_bga_fanout(
    fp, pcb,
    net_filter=names,
    layers=['F.Cu', 'B.Cu'],
    track_width=0.1, clearance=0.1,
    via_size=0.45, via_drill=0.25,
    exit_margin=0.5,
    escape_method=METHOD,
    plane_drop=('off' if NO_DROP else 'auto'),
    escape_dir_hints=(None if NO_HINTS else hints),
    escape_line_hints=(None if (NO_HINTS or NO_LINES) else lines),
)

net_names = {nid: n.name for nid, n in pcb.nets.items()}
if tracks:
    add_tracks_and_vias_to_pcb(base, out_path, tracks, vias_add, vias_rm,
                               net_id_to_name=net_names)
else:
    import shutil
    shutil.copy(base, out_path)
pro = os.path.splitext(base)[0] + '.kicad_pro'
if os.path.exists(pro):
    import shutil
    shutil.copy(pro, os.path.splitext(out_path)[0] + '.kicad_pro')

# --- did the fanout OBEY? measure the direction of the copper that
# actually leaves each ball, rather than trusting that the hint landed.
id2name = {byname[n][0]: n for n in names}
by_net = {}
for t in tracks:
    by_net.setdefault(t['net_id'], []).append(t)
DIRS = {'right': (1, 0), 'left': (-1, 0), 'up': (0, -1), 'down': (0, 1)}
agree = disagree = absent = 0
bad = []
for nm in names:
    m = choice.get(nm)
    if m is None:
        continue
    p = dst_pad[nm]
    ts = by_net.get(byname[nm][0], [])
    if not ts:
        absent += 1
        continue
    # the escape's overall direction: pad centre to the far end of the
    # emitted copper, snapped to the axis it mostly runs along
    far = max((pt for t in ts for pt in (t['start'], t['end'])),
              key=lambda q: (q[0] - p.global_x) ** 2
              + (q[1] - p.global_y) ** 2)
    dx, dy = far[0] - p.global_x, far[1] - p.global_y
    got = min(DIRS, key=lambda k: (DIRS[k][0] - dx / (math.hypot(dx, dy) or 1))
              ** 2 + (DIRS[k][1] - dy / (math.hypot(dx, dy) or 1)) ** 2)
    if got == m.direction:
        agree += 1
    else:
        disagree += 1
        bad.append(f'{nm}({m.direction}->{got})')

print(f'\nwrote {out_path}: {len(tracks)} tracks, {len(vias_add)} vias, '
      f'{len(failed)} failed nets')
print(f'plan obeyed: {agree}/{agree + disagree + absent} balls escaped in '
      f'the planned direction, {disagree} took another, {absent} no copper')
if bad:
    print('  differed: ' + ', '.join(bad[:12])
          + (f' (+{len(bad) - 12} more)' if len(bad) > 12 else ''))
if failed:
    print(f'  failed nets: {", ".join(sorted(failed)[:10])}')

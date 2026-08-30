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
import braid as te  # noqa: E402
import escape_moves as em  # noqa: E402
import detect_buses as db  # noqa: E402
import plan_ends as pe  # noqa: E402
import flow_frame as ff  # noqa: E402

# Face names are FLOW-FRAME names: 'left' is the face toward the source,
# 'down' the face on the side of the source's flank teeth, whatever the
# board's own axes say. They are mapped to board directions through the
# same flow angle the braid rotates by, so a rotated bench gets the same
# plan -- which the rotation gate (chain_rot.sh) checks.
_DIRV = {'right': (1, 0), 'left': (-1, 0), 'up': (0, -1), 'down': (0, 1)}


def board_dirs(flow_names, theta):
    """Flow-frame direction names -> board direction names, for a flow
    frame that rotates the board by `theta` degrees (flow_frame.rotate_pcb's
    convention: flow = R(theta) . board, so board = R(-theta) . flow)."""
    r = math.radians(-theta)
    c, s = math.cos(r), math.sin(r)
    out = set()
    for nm in flow_names:
        vx, vy = _DIRV[nm]
        bx, by = c * vx - s * vy, s * vx + c * vy
        out.add(min(_DIRV, key=lambda k: (_DIRV[k][0] - bx) ** 2
                    + (_DIRV[k][1] - by) ** 2))
    return out

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
# Apply the plan's SOURCE choices too: the bench's source array comes
# fanned out, and the plan's source refinement (plan_ends.refine_source)
# only ever chose better teeth on paper. With --source the chosen nets'
# source copper is stripped and re-fanned in the planned direction and
# KIND (surface -> channel escape on the pad's layer, dogbone -> dogbone,
# via-in-pad -> underpad), so a tooth arrives on the layer the corridor
# delivers on -- a B tooth the corridor must bring back to F costs the
# braid a via the plan already knew it did not want (K15 SRAS).
SOURCE = '--source' in sys.argv
# --order-model: the braid's own launch and exit rules
# (plan_order.BraidOrder) score the JOINERS' face choice by the
# schedule the braid will lay -- corridor vias (divers, exit-leg
# crossings, in-flight surfacings) and columns against the corridor's
# capacity -- after select() has placed everything by its projection.
# Opt-in: it moves SWE down at K21 and K28 as the human does (vias
# 34 -> 32, 75 -> 68) and the braid then refuses two exit-block lanes
# behind it each time (SA7/SA9, SA1/SA7) -- a braid defect to chase
# before this becomes the default. Off, the chain is bit-identical.
ORDER_MODEL = '--order-model' in sys.argv
base = next((a.split('=', 1)[1] for a in sys.argv
             if a.startswith('--board=')),
            os.path.join(HERE, 'fb_t2q_base.kicad_pcb'))

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

# the source stubs already on the board, as MOVES: the plan's lane
# checks must see the gap and via site of every net it leaves alone
src_seed = {}
for nm in smenu:
    p = src_pad[nm]
    nid = byname[nm][0]
    tooth = launch[nm]
    near_vias = [v for v in pcb.vias if v.net_id == nid
                 and math.hypot(v.x - p.global_x, v.y - p.global_y) < 2.5]
    d = (tooth[0] - p.global_x, tooth[1] - p.global_y)
    direction = min(_DIRV, key=lambda k: (_DIRV[k][0] * math.hypot(*d) - d[0]) ** 2
                    + (_DIRV[k][1] * math.hypot(*d) - d[1]) ** 2)
    site = (near_vias[0].x, near_vias[0].y) if near_vias else None
    src_seed[nm] = em.Move(nm, 'dogbone' if site else 'surface', direction,
                           tooth0[nm], tooth, len(near_vias),
                           [((p.global_x, p.global_y), tooth, tooth0[nm])],
                           site=site)

print('planning...')
if ONLY:
    _theta = ff.flow_angle([ends[n][0] for n in names],
                           [ends[n][1] for n in names])
    _only_board = board_dirs(ONLY, _theta)
    if _theta:
        print(f'  flow frame {_theta:.0f} deg: faces {sorted(ONLY)} '
              f'(flow) -> {sorted(_only_board)} (board)')
    ONLY = _only_board
    dmenu = {n: [m for m in ms if m.direction in ONLY]
             for n, ms in dmenu.items()}
    _empty = [n for n, ms in dmenu.items() if not ms]
    if _empty:
        print(f'  {len(_empty)} net(s) have NO move on {",".join(ONLY)}: '
              + ','.join(_empty[:8]))
model = None
if ORDER_MODEL:
    import plan_order as po
    model = po.build_model(pcb, names, ends, byname, obs, paths, launch, tooth0,
                           dref, sref, dgrid.bbox)
    print(f'  order model: {len(model.joiners)} joiner(s) '
          f'{sorted(model.joiners)}, s0 {model.s0:.2f}')
schoice, choice, lp, report = pe.plan_ends(
    smenu, dmenu, launch, sgrid.bbox, dgrid.bbox, buses=buses,
    tooth_layer0=tooth0, src_seed=src_seed,
    # the 'spend' objective only when the source moves will be APPLIED
    objective='spend' if SOURCE else 'floor', model=model)
# only the nets the plan actually MOVED are re-fanned: a move of the
# same kind, side, layer and gap as the stub already on the board IS
# that stub (the menu's exit x differs from the tooth's by the array
# margin, and identity let every such net be re-fanned for nothing)


def _same_as_seed(nm, m):
    s = src_seed.get(nm)
    if s is None or m is s:
        return m is s
    if (m.kind, m.direction, m.layer) != (s.kind, s.direction, s.layer):
        return False
    ax = 1 if m.direction in ('left', 'right') else 0
    if abs(m.exit_pt[ax] - s.exit_pt[ax]) > 0.16:
        return False
    if m.site is not None and s.site is not None and \
            math.hypot(m.site[0] - s.site[0], m.site[1] - s.site[1]) > 0.16:
        return False
    return True


schoice = {nm: m for nm, m in schoice.items() if not _same_as_seed(nm, m)}
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

DIRS = {'right': (1, 0), 'left': (-1, 0), 'up': (0, -1), 'down': (0, 1)}


def obeyed(tracks, chosen, pads, label):
    """Did the fanout OBEY? Measure the direction of the copper that
    actually leaves each ball, rather than trusting that the hint
    landed: pad centre to the far end of the emitted copper, snapped to
    the axis it mostly runs along."""
    by_net = {}
    for t in tracks:
        by_net.setdefault(t['net_id'], []).append(t)
    agree = disagree = absent = 0
    bad = []
    for nm, m in chosen.items():
        p = pads[nm]
        ts = by_net.get(byname[nm][0], [])
        if not ts:
            absent += 1
            continue
        far = max((pt for t in ts for pt in (t['start'], t['end'])),
                  key=lambda q: (q[0] - p.global_x) ** 2
                  + (q[1] - p.global_y) ** 2)
        dx, dy = far[0] - p.global_x, far[1] - p.global_y
        h = math.hypot(dx, dy) or 1
        got = min(DIRS, key=lambda k: (DIRS[k][0] - dx / h) ** 2
                  + (DIRS[k][1] - dy / h) ** 2)
        if got == m.direction:
            agree += 1
        else:
            disagree += 1
            bad.append(f'{nm}({m.direction}->{got})')
    print(f'{label} plan obeyed: {agree}/{agree + disagree + absent} balls '
          f'escaped in the planned direction, {disagree} took another, '
          f'{absent} no copper')
    if bad:
        print('  differed: ' + ', '.join(bad[:12])
              + (f' (+{len(bad) - 12} more)' if len(bad) > 12 else ''))


def pad_key(p):
    return (round(p.global_x, 3), round(p.global_y, 3))


def copy_pro(src_board, dst_board):
    import shutil
    pro = os.path.splitext(src_board)[0] + '.kicad_pro'
    if os.path.exists(pro):
        shutil.copy(pro, os.path.splitext(dst_board)[0] + '.kicad_pro')


board = base
if SOURCE and schoice:
    import statistics
    from collections import Counter
    src_nets = [nm for nm in schoice if nm in src_pad and src_pad[nm] is not None]
    src_ids = {byname[nm][0] for nm in src_nets}
    src_names_full = {byname[nm][1].name for nm in src_nets}
    # the re-fanout uses the bench's OWN source fanout geometry
    widths = [s.width for s in pcb.segments if s.net_id in src_ids]
    vsz = [(v.size, v.drill) for v in pcb.vias if v.net_id in src_ids]
    tw = statistics.median(widths) if widths else 0.1
    vs, vd = Counter(vsz).most_common(1)[0][0] if vsz else (0.25, 0.15)
    txt = open(base, encoding='utf-8').read()
    txt = te.strip_net_items(txt, 'segment', src_ids, src_names_full)
    txt = te.strip_net_items(txt, 'via', src_ids, src_names_full)
    stem = os.path.splitext(out_path)[0]
    board = stem + '_src0.kicad_pcb'
    with open(board, 'w', encoding='utf-8') as f:
        f.write(txt)
    copy_pro(base, board)
    kinds = Counter(schoice[nm].kind for nm in src_nets)
    print(f'\nsource: re-fanning {len(src_nets)} of {sref}\'s nets per the '
          f'plan ({", ".join(f"{k}:{v}" for k, v in sorted(kinds.items()))}), '
          f'track {tw} via {vs}/{vd}')
    for nm in src_nets:
        print(f'    {nm}: {src_seed[nm]} -> {schoice[nm]}')
    src_tracks = []
    # dogbones and via-in-pads FIRST: their stubs are short and sit
    # against the pad, and the dogbone engine does not keep its 45-degree
    # stub clear of an earlier pass's surface escape running through the
    # same row gap (K15: 9 grazes SDQM0/SDQ15). The channel escapes go
    # last and route round whatever copper is there.
    for kind, method in (('via_in_pad', 'underpad'), ('dogbone', 'dogbone'),
                         ('surface', 'channel')):
        nets_k = [nm for nm in src_nets if schoice[nm].kind == kind]
        if not nets_k:
            continue
        pcb_k = parse_kicad_pcb(board)
        fp_k = pcb_k.footprints[sref]
        hints_k = {pad_key(src_pad[nm]): schoice[nm].direction for nm in nets_k}
        # the plan's exit LINE too: it checked that exact gap against
        # the static copper (a foreign via in the next gap grazed the
        # engine's own pick of gap -- K15 SDQ8 vs SDQS1N)
        lines_k = {pad_key(src_pad[nm]): (schoice[nm].exit_pt[1]
                                          if schoice[nm].direction in ('left', 'right')
                                          else schoice[nm].exit_pt[0])
                   for nm in nets_k}
        tr, va, vr, fl = generate_bga_fanout(
            fp_k, pcb_k, net_filter=nets_k, layers=['F.Cu', 'B.Cu'],
            track_width=tw, clearance=0.1, via_size=vs, via_drill=vd,
            exit_margin=0.5, escape_method=method, plane_drop='off',
            escape_dir_hints=hints_k, escape_line_hints=lines_k)
        nxt = f'{stem}_src_{kind}.kicad_pcb'
        if tr:
            add_tracks_and_vias_to_pcb(
                board, nxt, tr, va, vr,
                net_id_to_name={i: n.name for i, n in pcb_k.nets.items()})
        else:
            import shutil
            shutil.copy(board, nxt)
        copy_pro(board, nxt)
        print(f'  {kind} -> {method}: {len(nets_k)} net(s), {len(tr)} tracks, '
              f'{len(va)} vias, {len(fl)} failed'
              + (f' ({", ".join(sorted(fl)[:6])})' if fl else ''))
        src_tracks.extend(tr)
        board = nxt
    obeyed(src_tracks, {nm: schoice[nm] for nm in src_nets}, src_pad, 'source')
    pcb = parse_kicad_pcb(board)

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
    add_tracks_and_vias_to_pcb(board, out_path, tracks, vias_add, vias_rm,
                               net_id_to_name=net_names)
else:
    import shutil
    shutil.copy(board, out_path)
copy_pro(board, out_path)

print(f'\nwrote {out_path}: {len(tracks)} tracks, {len(vias_add)} vias, '
      f'{len(failed)} failed nets')
obeyed(tracks, choice, dst_pad, 'berth')
if failed:
    print(f'  failed nets: {", ".join(sorted(failed)[:10])}')

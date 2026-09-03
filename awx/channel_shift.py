#!/usr/bin/env python3
"""#622 CHANNEL SHIFT at an array face -- the assignment-level lever
the back-wrap question turned out to be (0903).

At U1's east edge the human puts one F track through each channel
between the outer-column pads, every ring-2 ball taking the channel
next to its own row (SDQ7 68.15, SA3 68.84, SCS0 69.49). Our fanout
slid that group by one channel (SA3 in SDQ7's, SCS0 in SA3's, the
channel at 69.45 EMPTY) and the ball at the top of the group, SDQ7,
got no F channel and was dogboned to B -- 4 layer changes over 13
crossings where the human pays 0 over 5. Relaying the chain toward the
empty channel (SCS0 -> 69.45, SA3 -> 68.80, SDQ7 -> 68.15 on F, plus
SDQ7's berth onto F) and braiding the three against the record gave
K35 67 -> 66v 0/0 (project-clearance check_drc and kicad-cli clean).

General form, per face of each array: the outer ring's balls own their
row lines, the second ring's balls own the channels between outer-ring
pads; census which net's F track leaves through each channel; a
STARVED net (a ring-2 ball with no F exit on this face, its stub on B)
whose adjacent channel holds a neighbour, with an EMPTY channel within
--reach channels along the face, gets the chain shifted one channel
each toward the empty one. Each shift is relayed with relay_net,
realized surgically (the shifted nets + the starved net braided as one
group against the frozen record, blockers ripped), graded and judged;
kept when lexicographically better on (open, drc, vias).

usage: channel_shift.py TAG BEST FO K [--refs U1,DU1] [--reach 3]
"""
import argparse
import os
import shutil
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
os.chdir(HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
from ledger_cal import Judge              # noqa: E402
import surgical as sg                     # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('tag')
ap.add_argument('best')
ap.add_argument('fo')
ap.add_argument('k')
ap.add_argument('--refs', default='U1,DU1')
ap.add_argument('--reach', type=int, default=3)
ap.add_argument('--src', default='U1')
ap.add_argument('--dst', default='DU1')
a = ap.parse_args()
NETS = sg.k_nets(a.k)
nets = NETS.split(',')
knets = set(nets)
T0 = time.time()


def say(msg):
    print(f'[{time.time() - T0:6.0f}s] {msg}', flush=True)


FACES = {  # face -> (axis along the edge, edge selector, outward sign)
    'right': ('y', 'xmax', +1), 'left': ('y', 'xmin', -1),
    'down': ('x', 'ymax', +1), 'up': ('x', 'ymin', -1)}


def face_census(pcb, ref, face):
    """For one face: pitch, the outer ring's line coords, the channel
    coords, the ring-2 balls {net: line coord}, and the F tracks leaving
    the face {coord: net}."""
    short = {i: n.name.rsplit('/', 1)[-1] for i, n in pcb.nets.items()}
    fp = pcb.footprints[ref]
    xs = sorted(set(round(q.global_x, 3) for q in fp.pads))
    ys = sorted(set(round(q.global_y, 3) for q in fp.pads))
    axis, edge, sgn = FACES[face]
    pitch = min(b - c for b, c in zip(xs[1:], xs)) if len(xs) > 1 else 0.65
    if axis == 'y':
        outer = xs[-1] if edge == 'xmax' else xs[0]
        ring2 = outer - sgn * pitch
        lines = ys
        along = lambda q: q.global_y            # noqa: E731
        across = lambda q: q.global_x           # noqa: E731
    else:
        outer = ys[-1] if edge == 'ymax' else ys[0]
        ring2 = outer - sgn * pitch
        lines = xs
        along = lambda q: q.global_x            # noqa: E731
        across = lambda q: q.global_y           # noqa: E731
    channels = [(u + v) / 2 for u, v in zip(lines, lines[1:])]
    r2 = {}
    for q in fp.pads:
        nm = short.get(q.net_id)
        if nm in knets and abs(across(q) - ring2) < 1e-3:
            r2[nm] = along(q)
    # F tracks crossing a line just outside the outer ring
    probe = outer + sgn * (pitch / 2)
    exits = {}
    for s in pcb.segments:
        if s.layer != 'F.Cu':
            continue
        p0, p1 = ((s.start_x, s.start_y), (s.end_x, s.end_y))
        c0, c1 = (p0[0], p1[0]) if axis == 'y' else (p0[1], p1[1])
        if (c0 - probe) * (c1 - probe) >= 0:
            continue
        t = (probe - c0) / (c1 - c0)
        v = (p0[1] + t * (p1[1] - p0[1])) if axis == 'y' \
            else (p0[0] + t * (p1[0] - p0[0]))
        if lines[0] - pitch < v < lines[-1] + pitch:
            exits[round(v, 3)] = short.get(s.net_id, '?')
    return dict(pitch=pitch, lines=lines, channels=channels, r2=r2,
                exits=exits, axis=axis)


def nearest(vals, v):
    return min(range(len(vals)), key=lambda i: abs(vals[i] - v))


def plan_shifts(c, reach):
    """[(starved_net, [(net, new_coord), ...])] -- the chain relayed
    from the empty channel back toward the starved ball."""
    ch = c['channels']
    occ = {}
    for v, nm in c['exits'].items():
        i = nearest(ch, v)
        if abs(ch[i] - v) < c['pitch'] * 0.3:
            occ.setdefault(i, nm)
    plans = []
    for nm, line in c['r2'].items():
        if nm in c['exits'].values():
            continue                          # already exits this face on F
        li = nearest(c['lines'], line)
        for ci in (li - 1, li):               # the two adjacent channels
            if not (0 <= ci < len(ch)) or ci not in occ:
                continue
            # walk the displaced chain in the direction away from the
            # ball's line until an empty channel, within reach
            step = -1 if ci == li - 1 else +1
            chain = [(occ[ci], ci)]
            j = ci + step
            found = None
            while 0 <= j < len(ch) and abs(j - ci) <= reach:
                if j not in occ:
                    found = j
                    break
                chain.append((occ[j], j))
                j += step
            if found is None:
                continue
            # relay order: the net nearest the empty channel first
            moves = []
            dest = found
            for (onm, oi) in reversed(chain):
                moves.append((onm, ch[dest]))
                dest = oi
            moves.append((nm, ch[dest]))
            plans.append((nm, moves))
    return plans


best, fo = a.best, a.fo
g0 = sg.grade(best, NETS)
J = Judge(best, nets, a.src, a.dst)
say(f'start {os.path.basename(best)}: grade {g0} floor {J.floor_total}')
pcb = parse_kicad_pcb(fo)
applied = 0
for ref in a.refs.split(','):
    for face in FACES:
        c = face_census(pcb, ref, face)
        plans = plan_shifts(c, a.reach)
        if not plans:
            continue
        say(f'{ref} {face}: {len(c["exits"])} F exits, ring-2 balls '
            f'{len(c["r2"])}, shift plans {[(p[0], len(p[1])) for p in plans]}')
        for starved, moves in plans:
            label = f'{a.tag}_{ref}{face[0]}_{starved}'
            cur_fo = fo
            ok = True
            for i, (nm, coord) in enumerate(moves):
                out = f'tmp/{label}_{i}_{nm}_fo.kicad_pcb'
                largs = (['--ref', ref] if ref != a.src else []) + [
                    '--side', face, '--coord', f'{coord:.2f}',
                    '--layer', 'F.Cu']
                key = sg.relay(cur_fo, nm, largs, out)
                if key is None or key[1] != 'F.Cu':
                    say(f'  {label}: relay of {nm} to {face}/F@{coord:.2f} '
                        f'missed ({key})')
                    ok = False
                    break
                cur_fo = out
            if not ok:
                continue
            # the starved net's OTHER end onto F too (a straight F ride)
            other = a.dst if ref == a.src else a.src
            out = f'tmp/{label}_end_fo.kicad_pcb'
            key = sg.relay(cur_fo, starved, (
                ['--ref', other] if other != a.src else []) + [
                '--keep-pos', '--layer', 'F.Cu'], out)
            if key is not None and key[1] == 'F.Cu':
                cur_fo = out
            group = [nm for nm, _c in moves]
            cur = best
            for i, nm in enumerate(group):
                nxt = f'tmp/{label}_scr{i}.kicad_pcb'
                sg.swap_stub(cur, cur_fo, nm, nxt)
                cur = nxt
            blk = set()
            for nm in group:
                blk.update(sg.drc_partners(cur, nm))
            blk -= set(group)
            for i, b in enumerate(sorted(blk)):
                nxt = f'tmp/{label}_blk{i}.kicad_pcb'
                sg.swap_stub(cur, cur_fo, b, nxt)
                cur = nxt
            allg = group + sorted(blk)
            sg.braid_one(cur, ','.join(allg), f'tmp/{label}_b', dst=a.dst)
            board = f'tmp/{label}_b.kicad_pcb'
            if not os.path.exists(board):
                say(f'  {label}: braid wrote nothing')
                continue
            gc, opens = sg.grade_full(board, NETS)
            if gc[0] > g0[0] and gc[1] <= g0[1]:
                _g, base_open = sg.grade_full(best, NETS)
                board, gc, opens = sg.rescue_close(
                    board, cur_fo, base_open, NETS, label, a.dst)
            Jc = Judge(board, nets, a.src, a.dst)
            say(f'  {label}: moved {allg} -> grade {gc} floor '
                f'{Jc.floor_total} (was {g0} / {J.floor_total})')
            if gc < g0:
                keep = f'tmp/{label}_acc.kicad_pcb'
                shutil.copy(board, keep)
                pro = os.path.splitext(fo)[0] + '.kicad_pro'
                if os.path.exists(pro):
                    shutil.copy(pro, os.path.splitext(keep)[0] + '.kicad_pro')
                    shutil.copy(pro, os.path.splitext(cur_fo)[0] + '.kicad_pro')
                say(f'  ACCEPT {label}: {g0} -> {gc}')
                best, g0, J, fo = keep, gc, Jc, cur_fo
                pcb = parse_kicad_pcb(fo)
                applied += 1
say(f'FINAL {best}: grade {g0} floor {J.floor_total} ({applied} shift(s) '
    f'applied; fo {fo})')

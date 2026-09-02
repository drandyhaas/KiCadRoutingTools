#!/usr/bin/env python3
"""#622 FLOOR EVOLUTION: judge-guided surgical descent.

The measured division of labor (0902): the improve loop's moves
harvest SLACK against a flat floor; FLOOR moves are assignment
changes -- and applied SURGICALLY (replace one net's stub, braid
that net alone against everyone else's FROZEN copper) the judged
floor is realized exactly (SRST wrap: predicted 63->60, realized
60, board 73v -> 71v 0/0 in one move). Whole-board re-braids threw
the prediction away (measured: same move re-braided = floor 65,
1 open).

Loop: sweep assignment moves (alt faces x layers both ends +
in-place flips) -> realize each cheaply (relay + single-net braid on
the frozen world) -> judge by swap_floor -> apply the best candidate
whose REALIZED board grades 0 open / 0 drc and whose full-Judge
floor confirms the drop -> repeat until flat. Then SLACK HARVEST:
per net with act > dp, re-braid it alone in place, keep strict
improvements. Every accepted board is graded and floor-verified.

usage: floor_evolve.py TAG BEST_BOARD FO_BOARD K [--rounds 4]
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
os.chdir(HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
from ledger_cal import Judge              # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('tag')
ap.add_argument('best')
ap.add_argument('fo')
ap.add_argument('k')
ap.add_argument('--rounds', type=int, default=4)
ap.add_argument('--src', default='U1')
ap.add_argument('--dst', default='DU1')
a = ap.parse_args()
NETS = subprocess.run(
    [sys.executable, 'coherent_nets.py', a.k],
    capture_output=True, text=True).stdout.strip()
nets = NETS.split(',')
ENV = dict(os.environ, TWO_PAGE='1')


def _walk_strip(txt, token, match):
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
        if match(txt[j:k + 1]):
            out.append(txt[i:j].rstrip(' \t'))
            e = k + 1
            if e < len(txt) and txt[e] == '\n':
                e += 1
            i = e
        else:
            out.append(txt[i:k + 1])
            i = k + 1
    return ''.join(out)


def _collect(txt, token, match):
    out, i = [], 0
    pat = '(' + token
    while True:
        j = txt.find(pat, i)
        while j >= 0 and j + len(pat) < len(txt) \
                and txt[j + len(pat)] not in ' \n\t(':
            j = txt.find(pat, j + 1)
        if j < 0:
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
        if match(txt[j:k + 1]):
            out.append(txt[j:k + 1])
        i = k + 1
    return out


def _matcher(nid, name):
    def match(block):
        m = re.search(r'\(net (\d+)\)', block)
        if m:
            return int(m.group(1)) == nid
        m = re.search(r'\(net "([^"]+)"\)', block)
        return bool(m and m.group(1) == name)
    return match


def net_ids(board):
    p = parse_kicad_pcb(board)
    return {n.name.split('/')[-1]: (i, n.name)
            for i, n in p.nets.items()}


def swap_stub(best, src_board, n, out, strip_only_lane=False):
    """best with net n's copper replaced by src_board's copy of n.
    With strip_only_lane, n's copper in `best` is stripped and
    src_board must BE best (in-place re-braid harvest)."""
    ids = net_ids(src_board)
    nid, name = ids[n]
    match = _matcher(nid, name)
    blocks = []
    if not strip_only_lane:
        src_txt = open(src_board, encoding='utf-8').read()
        blocks = (_collect(src_txt, 'segment', match)
                  + _collect(src_txt, 'via', match))
    ids_b = net_ids(best)
    match_b = _matcher(ids_b[n][0], ids_b[n][1])
    txt = open(best, encoding='utf-8').read()
    txt = _walk_strip(txt, 'segment', match_b)
    txt = _walk_strip(txt, 'via', match_b)
    pos = txt.rfind(')')
    open(out, 'w', encoding='utf-8').write(
        txt[:pos] + '\n'.join(blocks) + '\n' + txt[pos:])


def grade(board):
    g = subprocess.run(
        [sys.executable, 'grade_k.py', board, NETS],
        capture_output=True, text=True).stdout
    m = re.search(r'open=(\d+) drc=(\d+) vias=(\d+)', g)
    return tuple(int(x) for x in m.groups()) if m else (99, 99, 999)


def braid_one(board, n, out):
    side = os.path.splitext(board)[0] + '.pages.json'
    if os.path.exists(side):
        os.remove(side)
    r = subprocess.run(
        [sys.executable, 'braid.py', '--board', board, '--dest',
         a.dst, '--nets', n, '--out', out],
        env=ENV, capture_output=True, text=True)
    return (os.path.exists(out + '.kicad_pcb')
            and 'REFUSED' not in r.stdout)


# ---- candidate menu (the sweep's geometry)
fo_pcb = parse_kicad_pcb(a.fo)


def bbox(ref):
    fp = fo_pcb.footprints[ref]
    xs = [p.global_x for p in fp.pads]
    ys = [p.global_y for p in fp.pads]
    return (min(xs) - 0.25, min(ys) - 0.25,
            max(xs) + 0.25, max(ys) + 0.25)


BBs = {a.src: bbox(a.src), a.dst: bbox(a.dst)}


def ball(ref, nm):
    fp = fo_pcb.footprints[ref]
    nid = next((i for i, n2 in fo_pcb.nets.items()
                if n2.name.rsplit('/', 1)[-1] == nm), None)
    p = next((p2 for p2 in fp.pads if p2.net_id == nid), None)
    return (p.global_x, p.global_y) if p else None


def cands(nm):
    out = []
    for ref in (a.src, a.dst):
        b = ball(ref, nm)
        o = ball(a.dst if ref == a.src else a.src, nm)
        if not b or not o:
            continue
        W = BBs[ref]
        dx, dy = o[0] - b[0], o[1] - b[1]
        for side in ('up', 'down', 'left', 'right'):
            if side in ('left', 'right'):
                edge = W[0] if side == 'left' else W[2]
                t = (edge - b[0]) / dx if abs(dx) > 1e-9 else -1
                raw = b[1] + t * dy if t > 1e-9 else o[1]
                span = (W[1], W[3])
            else:
                edge = W[1] if side == 'up' else W[3]
                t = (edge - b[1]) / dy if abs(dy) > 1e-9 else -1
                raw = b[0] + t * dx if t > 1e-9 else o[0]
                span = (W[0], W[2])
            coord = min(max(raw, span[0] + 0.3), span[1] - 0.3)
            for L in ('F.Cu', 'B.Cu'):
                out.append((f'{ref[0]}{side[0]}{L[0]}',
                            (['--ref', a.dst] if ref == a.dst else [])
                            + ['--side', side, '--coord',
                               f'{coord:.2f}', '--layer', L]))
        for L in ('F.Cu', 'B.Cu'):
            out.append((f'{ref[0]}kp{L[0]}',
                        (['--ref', a.dst] if ref == a.dst else [])
                        + ['--keep-pos', '--layer', L]))
    return out


best = a.best
fo = a.fo
g0 = grade(best)
J = Judge(best, nets)
print(f'start: grade {g0} floor {J.floor_total} '
      f'changes {J.act_total}')

for rnd in range(a.rounds):
    print(f'== round {rnd}: floor {J.floor_total}, sweeping')
    found = []
    for nm in sorted(J.paths):
        seen = set()
        for label, largs in cands(nm):
            out = f'tmp/{a.tag}_r{rnd}_{nm}_{label}.kicad_pcb'
            r = subprocess.run(
                [sys.executable, 'relay_net.py', fo, nm,
                 '--out', out] + largs,
                capture_output=True, text=True)
            if not os.path.exists(out) or 'MISSED' in r.stdout:
                continue
            md = re.search(r'DELIVERED (\w+)/([\w.]+)@([-\d.]+)',
                           r.stdout)
            if md:
                k2 = (md.group(1), md.group(2),
                      round(float(md.group(3)), 1))
                if k2 in seen:
                    os.remove(out)
                    continue
                seen.add(k2)
            scr = f'tmp/{a.tag}_scr.kicad_pcb'
            swap_stub(best, out, nm, scr)
            if not braid_one(scr, nm, f'tmp/{a.tag}_b1'):
                continue
            b1 = f'tmp/{a.tag}_b1.kicad_pcb'
            nf = J.swap_floor(b1, nm)
            if nf is not None and nf < J.floor_total:
                found.append((nf, nm, label, out))
                # keep the realized candidate board
                keep = f'tmp/{a.tag}_r{rnd}_{nm}_{label}_b1.kicad_pcb'
                shutil.copy(b1, keep)
    if not found:
        print(f'  round {rnd}: landscape flat, stopping')
        break
    found.sort()
    applied = False
    for nf, nm, label, rel_fo in found[:5]:
        cand = f'tmp/{a.tag}_r{rnd}_{nm}_{label}_b1.kicad_pcb'
        gc = grade(cand)
        Jc = Judge(cand, nets)
        print(f'  try {nm} {label}: predicted {nf}, realized '
              f'floor {Jc.floor_total}, grade {gc}')
        # lexicographic: completion/drc improvement outranks the
        # floor (closing an open ADDS a path and legitimately RAISES
        # the floor); at equal (open, drc) the floor must drop
        if (gc[0], gc[1]) < (g0[0], g0[1]) \
                or ((gc[0], gc[1]) == (g0[0], g0[1])
                    and Jc.floor_total < J.floor_total):
            best = cand
            fo = rel_fo
            J = Jc
            g0 = gc
            print(f'  APPLIED {nm} {label}: floor -> '
                  f'{J.floor_total}, grade {gc}')
            applied = True
            break
    if not applied:
        print(f'  round {rnd}: no candidate realized cleanly, '
              'stopping')
        break

# ---- surgical slack harvest: re-braid slack nets alone, in place
print(f'== harvest: floor {J.floor_total}, '
      f'changes {J.act_total}, grade {g0}')
for _pass in range(2):
    moved = False
    slack = sorted(((J.paths[m].changes - J.per[m], m)
                    for m in J.paths), reverse=True)
    for d, m in slack:
        if d <= 0:
            continue
        scr = f'tmp/{a.tag}_h_scr.kicad_pcb'
        # strip m's lane but KEEP its stubs: swap in the stubs from
        # the current fo (the stub source of record)
        swap_stub(best, fo, m, scr)
        if not braid_one(scr, m, f'tmp/{a.tag}_h1'):
            continue
        h1 = f'tmp/{a.tag}_h1.kicad_pcb'
        gh = grade(h1)
        if (gh[0], gh[1]) > (g0[0], g0[1]) or gh[2] >= g0[2]:
            continue
        Jh = Judge(h1, nets)
        if Jh.floor_total > J.floor_total:
            continue
        keep = f'tmp/{a.tag}_h_{m}.kicad_pcb'
        shutil.copy(h1, keep)
        best, J, g0 = keep, Jh, gh
        print(f'  harvested {m}: grade {gh}, '
              f'changes {J.act_total}')
        moved = True
    if not moved:
        break

out = f'tmp/{a.tag}_final.kicad_pcb'
shutil.copy(best, out)
pro = os.path.splitext(best)[0] + '.kicad_pro'
if os.path.exists(pro):
    shutil.copy(pro, os.path.splitext(out)[0] + '.kicad_pro')
print(f'FINAL {out}: grade {g0} floor {J.floor_total} '
      f'changes {J.act_total}')

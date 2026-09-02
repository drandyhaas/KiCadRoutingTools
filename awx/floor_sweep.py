#!/usr/bin/env python3
"""#622 FLOOR SWEEP: the calibrated judge in its earned seat.

The improve loop harvests SLACK (measured: the record path's accepts
moved actual changes 77->67 against a flat floor ~63); the FLOOR --
the assignment's intrinsic cost -- moves only with assignment-level
changes (wraps, faces, end layers). This driver maps the floor-drop
landscape of a routed best board: for EVERY net and every
assignment move (3 alt faces x 2 layers at each end + 2 in-place
layer flips), realize the move cheaply (relay the stub + braid the
single net against the frozen rest) and score it with
Judge.swap_floor. Output: every candidate's floor delta -- the
evidence that decides between greedy single-move application and
composed-only search.

usage: floor_sweep.py BEST_BOARD FO_BOARD K [--out csv]
"""
import argparse
import math
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
os.chdir(HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
from ledger_cal import Judge              # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('best')
ap.add_argument('fo')
ap.add_argument('k')
ap.add_argument('--tag', default='fs')
ap.add_argument('--src', default='U1')
ap.add_argument('--dst', default='DU1')
a = ap.parse_args()
NETS = subprocess.run(
    [sys.executable, 'coherent_nets.py', a.k],
    capture_output=True, text=True).stdout.strip()
nets = NETS.split(',')
ENV = dict(os.environ, TWO_PAGE='1')


# ---- the swapped-stub screen world (improve_k's proven recipe)
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


def swap_stub(best, relayed_fo, n, out):
    fo_p = parse_kicad_pcb(relayed_fo)
    by = {net.name.split('/')[-1]: (i, net)
          for i, net in fo_p.nets.items()}
    nid, netname = by[n][0], by[n][1].name
    match = _matcher(nid, netname)
    fo_txt = open(relayed_fo, encoding='utf-8').read()
    blocks = (_collect(fo_txt, 'segment', match)
              + _collect(fo_txt, 'via', match))
    txt = open(best, encoding='utf-8').read()
    txt = _walk_strip(txt, 'segment', match)
    txt = _walk_strip(txt, 'via', match)
    pos = txt.rfind(')')
    open(out, 'w', encoding='utf-8').write(
        txt[:pos] + '\n'.join(blocks) + '\n' + txt[pos:])


# ---- candidate menu per net (pure geometry, as the face channel)
pcb = parse_kicad_pcb(a.fo)
short = {i: n.name.rsplit('/', 1)[-1] for i, n in pcb.nets.items()}


def bbox(ref):
    fp = pcb.footprints[ref]
    xs = [p.global_x for p in fp.pads]
    ys = [p.global_y for p in fp.pads]
    return (min(xs) - 0.25, min(ys) - 0.25,
            max(xs) + 0.25, max(ys) + 0.25)


BBs = {a.src: bbox(a.src), a.dst: bbox(a.dst)}


def ball(ref, nm):
    fp = pcb.footprints[ref]
    nid = next((i for i, n2 in pcb.nets.items()
                if n2.name.rsplit('/', 1)[-1] == nm), None)
    p = next((p2 for p2 in fp.pads if p2.net_id == nid), None)
    return (p.global_x, p.global_y) if p else None


def cands(nm):
    """[(label, largs)] -- alt faces x layers at each end + in-place
    flips."""
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
                largs = ((['--ref', a.dst] if ref == a.dst else [])
                         + ['--side', side, '--coord',
                            f'{coord:.2f}', '--layer', L])
                out.append((f'{ref[0]}{side[0]}{L[0]}', largs))
        for L in ('F.Cu', 'B.Cu'):
            largs = ((['--ref', a.dst] if ref == a.dst else [])
                     + ['--keep-pos', '--layer', L])
            out.append((f'{ref[0]}kp{L[0]}', largs))
    return out


print(f'floor sweep on {os.path.basename(a.best)} '
      f'(fo {os.path.basename(a.fo)}) K={a.k}')
J = Judge(a.best, nets)
print(f'base floor {J.floor_total}, actual changes {J.act_total}')
rows = []
tried = refused = 0
for nm in sorted(J.paths):
    seen = set()
    for label, largs in cands(nm):
        tried += 1
        out = f'tmp/{a.tag}_{nm}_{label}.kicad_pcb'
        r = subprocess.run(
            [sys.executable, 'relay_net.py', a.fo, nm, '--out', out]
            + largs, capture_output=True, text=True)
        if not os.path.exists(out) or 'MISSED' in r.stdout:
            refused += 1
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
        scr = f'tmp/{a.tag}_{nm}_{label}_scr.kicad_pcb'
        swap_stub(a.best, out, nm, scr)
        side = os.path.splitext(scr)[0] + '.pages.json'
        if os.path.exists(side):
            os.remove(side)
        rb = subprocess.run(
            [sys.executable, 'braid.py', '--board', scr,
             '--dest', a.dst, '--nets', nm, '--out',
             f'tmp/{a.tag}_{nm}_{label}_b1'],
            env=ENV, capture_output=True, text=True)
        b1 = f'tmp/{a.tag}_{nm}_{label}_b1.kicad_pcb'
        if not os.path.exists(b1) or 'REFUSED' in rb.stdout:
            refused += 1
            continue
        nf = J.swap_floor(b1, nm)
        if nf is None:
            continue
        d = nf - J.floor_total
        rows.append((d, nm, label))
        if d < 0:
            g2 = subprocess.run(
                [sys.executable, 'grade_k.py', b1, NETS],
                capture_output=True, text=True).stdout
            mg = re.search(r'open=(\d+) drc=(\d+) vias=(\d+)', g2)
            gs = mg.groups() if mg else ('?', '?', '?')
            print(f'  DROPPER {nm} {label}: floor {J.floor_total} '
                  f'-> {nf}  realized grade open={gs[0]} '
                  f'drc={gs[1]} vias={gs[2]}')
print(f'\n{tried} candidates, {refused} refused, '
      f'{len(rows)} judged')
neg = sorted(r for r in rows if r[0] < 0)
zero = sum(1 for r in rows if r[0] == 0)
pos = sum(1 for r in rows if r[0] > 0)
print(f'floor droppers: {len(neg)}   flat: {zero}   raisers: {pos}')
for d, nm, label in neg[:15]:
    print(f'  {nm:7s} {label:6s} {d:+d}')

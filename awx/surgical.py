#!/usr/bin/env python3
"""#622 the SURGICAL toolkit: one net's copper, swapped or stripped,
braided ALONE against everyone else's frozen copper, graded, judged.

Factored out of floor_evolve.py (byte-for-byte the same text
surgery) so the floor descent and the slack harvest share one set of
primitives. Nothing here decides anything: it moves copper between
boards, runs the braid on one net (or a few), grades, and enumerates
the geometry of the assignment menu. The loops decide.
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

PY = sys.executable


def k_nets(k):
    return subprocess.run(
        [PY, os.path.join(HERE, 'coherent_nets.py'), str(k)],
        capture_output=True, text=True).stdout.strip()


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


def grade(board, nets_csv):
    """(open, drc, vias) among the run's nets; (99, 99, 999) when the
    grader could not read a board."""
    return grade_full(board, nets_csv)[0]


def grade_full(board, nets_csv):
    """((open, drc, vias), [open net short names])."""
    g = subprocess.run(
        [PY, os.path.join(HERE, 'grade_k.py'), board, nets_csv],
        capture_output=True, text=True).stdout
    m = re.search(r'open=(\d+) drc=(\d+) vias=(\d+)', g)
    if not m:
        return (99, 99, 999), []
    mo = re.search(r'open: (\S+)', g)
    opens = mo.group(1).split(',') if mo else []
    return tuple(int(x) for x in m.groups()), opens


def braid_one(board, nets_csv, out, dst='DU1', env=None):
    """Braid the named net(s) alone on `board` (everything else is
    frozen copper). True when the braid wrote out.kicad_pcb and
    refused nothing."""
    side = os.path.splitext(board)[0] + '.pages.json'
    if os.path.exists(side):
        os.remove(side)
    env = dict(env or os.environ, TWO_PAGE='1')
    r = subprocess.run(
        [PY, os.path.join(HERE, 'braid.py'), '--board', board,
         '--dest', dst, '--nets', nets_csv, '--out', out],
        env=env, capture_output=True, text=True, cwd=HERE)
    return (os.path.exists(out + '.kicad_pcb')
            and 'REFUSED' not in r.stdout)


def relay(fo, n, largs, out):
    """relay_net on the fanout board; returns the DELIVERED key
    (side, layer, coord) or None when the relay missed / wrote
    nothing."""
    r = subprocess.run(
        [PY, os.path.join(HERE, 'relay_net.py'), fo, n, '--out', out]
        + largs, capture_output=True, text=True, cwd=HERE)
    if not os.path.exists(out) or 'MISSED' in r.stdout:
        return None
    md = re.search(r'DELIVERED (\w+)/([\w.]+)@([-\d.]+)', r.stdout)
    if md:
        return (md.group(1), md.group(2), round(float(md.group(3)), 1))
    return ('?', '?', 0.0)


class Menu:
    """The assignment menu's geometry on a fanout board: for a net,
    every (label, relay_net args) -- alt faces x layers at each end,
    plus keep-position layer flips."""

    def __init__(self, fo, src='U1', dst='DU1'):
        self.src, self.dst = src, dst
        self.pcb = parse_kicad_pcb(fo)
        self.BBs = {src: self.bbox(src), dst: self.bbox(dst)}

    def bbox(self, ref):
        fp = self.pcb.footprints[ref]
        xs = [p.global_x for p in fp.pads]
        ys = [p.global_y for p in fp.pads]
        return (min(xs) - 0.25, min(ys) - 0.25,
                max(xs) + 0.25, max(ys) + 0.25)

    def ball(self, ref, nm):
        fp = self.pcb.footprints[ref]
        nid = next((i for i, n2 in self.pcb.nets.items()
                    if n2.name.rsplit('/', 1)[-1] == nm), None)
        p = next((p2 for p2 in fp.pads if p2.net_id == nid), None)
        return (p.global_x, p.global_y) if p else None

    def keep_pos(self, nm, ref, layer):
        return (f'{ref[0]}kp{layer[0]}',
                (['--ref', self.dst] if ref == self.dst else [])
                + ['--keep-pos', '--layer', layer])

    def faces(self, nm, ref):
        out = []
        b = self.ball(ref, nm)
        o = self.ball(self.dst if ref == self.src else self.src, nm)
        if not b or not o:
            return out
        W = self.BBs[ref]
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
                            (['--ref', self.dst] if ref == self.dst
                             else [])
                            + ['--side', side, '--coord',
                               f'{coord:.2f}', '--layer', L]))
        return out

    def cands(self, nm):
        """floor_evolve's sweep order: per end, faces x layers then
        the keep-position flips."""
        out = []
        for ref in (self.src, self.dst):
            if not self.ball(ref, nm):
                continue
            out.extend(self.faces(nm, ref))
            for L in ('F.Cu', 'B.Cu'):
                out.append(self.keep_pos(nm, ref, L))
        return out



def rescue_close(cand, fo, base_open, nets_csv, tag, dst='DU1'):
    """COMPLETION RESCUE (the user's rule, 0902: moves get vetoed on
    completion loss, so fixing completion frees the slack/floor moves
    behind the veto). For every K net open on `cand` but not on the
    base board, braid it ALONE against the candidate's frozen copper
    (stubs restored from fo). Returns (board, grade, opens) -- the
    rescued board when every stranded net closed, else the candidate
    as it was. Measured origin: SA2, K41's chronic open, closed this
    way at +5 vias / 0 drc on the 79v record."""
    g, opens = grade_full(cand, nets_csv)
    stranded = [m for m in opens if m not in base_open]
    if not stranded:
        return cand, g, opens
    cur = cand
    for i, m in enumerate(stranded):
        scr = f'tmp/{tag}_rsc_scr.kicad_pcb'
        swap_stub(cur, fo, m, scr)
        stem = f'tmp/{tag}_rsc{i}_{m}'
        if not braid_one(scr, m, stem, dst=dst):
            cur = None
            break
        cur = stem + '.kicad_pcb'
    if cur is not None:
        g2, opens2 = grade_full(cur, nets_csv)
        if len(opens2) < len(opens):
            return cur, g2, opens2
    # tier 2: the stranded nets as ONE group braid (schedule + last
    # call + rip-assist over all of them). Measured on the K35 batch
    # boards: sequential closes reach 2 of 4 before a refusal, the
    # group reaches 3 of 4 -- strictly further, one braid. A net the
    # batch's own wraps wall in (SA9 there) refuses either way.
    cur = cand
    for i, m in enumerate(stranded):
        nxt = f'tmp/{tag}_rscg_scr{i}.kicad_pcb'
        swap_stub(cur, fo, m, nxt)
        cur = nxt
    stem = f'tmp/{tag}_rscg'
    braid_one(cur, ','.join(stranded), stem, dst=dst)
    if os.path.exists(stem + '.kicad_pcb'):
        g3, opens3 = grade_full(stem + '.kicad_pcb', nets_csv)
        if len(opens3) < len(opens):
            return stem + '.kicad_pcb', g3, opens3
    return cand, g, opens

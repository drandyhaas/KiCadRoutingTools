#!/usr/bin/env python3
"""plan_loop.py -- the grading and board-finding helpers the evolution
(`evolve.py`) shares: `grade` / `better` / `fmt_g` (a board's (opens, drc,
vias) by grade_k and the order on it), `arm_boards` / `winner_pair` (the
portfolio arms a chain run left under a stem, and the pair it shipped).
Nothing here reads a face, a ref or a board name.
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dedupe_boards  # noqa: E402


def log(msg=''):
    print(msg, flush=True)

def grade(board, nets_csv):
    """(opens, drc, vias) by grade_k, or None when it did not grade."""
    r = subprocess.run([sys.executable, os.path.join(HERE, 'grade_k.py'), board, nets_csv],
                       capture_output=True, text=True, cwd=HERE)
    line = next((l for l in (r.stdout + r.stderr).splitlines() if l.startswith('GRADE')), '')
    m = re.search(r'open=(\d+) drc=(\d+) vias=(\d+)', line)
    if not m or 'BROKEN' in line:
        return None, line
    opens = sorted(line.split('open: ')[1].split(',')) if 'open: ' in line else []
    return [opens, int(m.group(2)), int(m.group(3))], line

def better(g, ref, margin=0):
    """Is grade g better than ref: fewer opens; no DRC; fewer vias by
    more than `margin` (opens equal)."""
    if g is None or ref is None:
        return g is not None and ref is None
    if g[1] != 0:
        return False
    if len(g[0]) != len(ref[0]):
        return len(g[0]) < len(ref[0])
    return g[2] < ref[2] - margin

def fmt_g(g):
    if g is None:
        return 'NO GRADE'
    return f'open {len(g[0])} drc {g[1]} vias {g[2]}' + (f' {g[0]}' if g[0] else '')

def arm_boards(stem, K):
    """The (fanout board, routed board) pairs a chain run left under
    `stem`: every portfolio arm STEM_fo_kK_J<j>_<A|B>, else the single-shot
    pair. Only pairs whose routed board exists."""
    pairs = []
    for j in (0, 1):
        fo = f'{stem}_fo_k{K}_J{j}.kicad_pcb'
        if not os.path.exists(fo):
            continue
        for arm in ('A', 'B'):
            rb = f'{stem}_fo_k{K}_J{j}_{arm}.kicad_pcb'
            if os.path.exists(rb):
                pairs.append((fo, rb))
    if not pairs:
        fo, rb = f'{stem}_fo_k{K}.kicad_pcb', f'{stem}_k{K}.kicad_pcb'
        if os.path.exists(fo) and os.path.exists(rb):
            pairs.append((fo, rb))
    return pairs

def winner_pair(stem, K, pairs):
    """Which arm pair the chain SHIPPED as STEM_kK: named in the chain's
    output when there is one, else the arm whose copper is the shipped
    board's (dedupe_boards' fingerprint), else the single pair."""
    shipped = f'{stem}_k{K}.kicad_pcb'
    out = stem + '.out'
    if os.path.exists(out):
        m = re.findall(r'braid A/B: keeping (\S+\.kicad_pcb)', open(out, encoding='utf-8').read())
        if m:
            for fo, rb in pairs:
                if os.path.basename(rb) == m[-1]:
                    return fo, rb
    if os.path.exists(shipped) and len(pairs) > 1:
        fpr = dedupe_boards.fingerprint(shipped)
        for fo, rb in pairs:
            if dedupe_boards.fingerprint(rb) == fpr:
                return fo, rb
    return pairs[0] if pairs else (None, None)

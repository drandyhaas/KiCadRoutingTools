#!/usr/bin/env python3
"""#622 CLOSE LADDER for one stranded net (0904): the single-net
rescue braids the net as-is against the frozen record; when that
refuses (K51 SA6, 1 open of 48), offer the net's whole assignment
menu -- every face x layer at either end and the keep-position flips
(surgical.Menu) -- each relayed onto the fanout board and realized
SURGICALLY with its blockers (realize_relay: the nets check_drc pairs
the new stub with are ripped and braided with it as one group).
Keeps the cheapest candidate that closes the net clean; the record is
untouched otherwise.

Measured 0904, K51 (48 nets, first ever complete): the record's last
stranded net SA6 (113v, 1 open) refused the single-net rescue; of 21
menu candidates the dest-face relay UdF with SCKE0 ripped and braided
alongside closed it at 121v 0/0 (project-clearance check_drc and
kicad-cli clean). Human K51 = 85.

usage: close_net.py TAG BEST FO K NET [--max-blockers 3]
"""
import argparse
import os
import shutil
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.chdir(HERE)
import surgical as sg  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('tag')
ap.add_argument('best')
ap.add_argument('fo')
ap.add_argument('k')
ap.add_argument('net')
ap.add_argument('--max-blockers', type=int, default=3)
ap.add_argument('--dst', default='DU1')
a = ap.parse_args()
NETS = sg.k_nets(a.k)
T0 = time.time()


def say(msg):
    print(f'[{time.time() - T0:6.0f}s] {msg}', flush=True)


g0, opens0 = sg.grade_full(a.best, NETS)
say(f'start {os.path.basename(a.best)}: {g0} open {opens0}')
menu = sg.Menu(a.fo)
cands = [('asis', None)] + list(menu.cands(a.net))
say(f'{len(cands)} candidate(s) for {a.net}')
best, gb = None, None
for label, largs in cands:
    stem = f'tmp/{a.tag}_{a.net}_{label}'
    if largs is None:
        relfo = a.fo
    else:
        relfo = stem + '_fo.kicad_pcb'
        key = sg.relay(a.fo, a.net, largs, relfo)
        if key is None:
            say(f'  {label}: relay missed')
            continue
    r = sg.realize_relay(a.best, a.fo, a.net, relfo, stem, dst=a.dst,
                         max_blockers=a.max_blockers)
    if r is None:
        say(f'  {label}: refused (too many blockers or braid refused)')
        continue
    board, group = r
    g, opens = sg.grade_full(board, NETS)
    say(f'  {label}: group {group} -> {g} open {opens}')
    if g[0] == 0 and g[1] == 0 and (gb is None or g < gb):
        best, gb = board, g
        keep = f'tmp/{a.tag}_{a.net}_closed.kicad_pcb'
        shutil.copy(board, keep)
        for ext in ('.kicad_pro', '.kicad_prl'):
            sib = os.path.splitext(a.best)[0] + ext
            if os.path.exists(sib):
                shutil.copy(sib, os.path.splitext(keep)[0] + ext)
        say(f'  CLOSED: {keep} {g}')
say(f'FINAL {best} {gb}' if best else f'FINAL: {a.net} not closed')

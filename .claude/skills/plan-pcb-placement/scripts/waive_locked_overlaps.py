#!/usr/bin/env python3
"""Waive, in the intent, the overlaps between parts that are both locked.

    python3 -X utf8 .claude/skills/plan-pcb-placement/scripts/waive_locked_overlaps.py BOARD INTENT

Two parts locked at the input's poses (§3.1) whose courtyards or drawn bodies
overlap are the designer's decision -- two footprints offered for one site,
a part drawn over another -- and nothing a placement does can clear them, as
neither part moves. check_assembly still gates them, so every seed of the
board comes out NOT BUILDABLE on a pair no move can change. Naming the pair
in the intent's `overlap_waivers` is how an overlap is accepted where it is
visible (docs/floorplan-intent.md).

Runs check_assembly on BOARD (the prepared board: fixed parts locked, the
rest piled) and, for every pair of LOCKED parts it reports as a courtyard or
drawn-body (fab) overlap, adds an `overlap_waivers` entry to INTENT unless
one names the pair already. A waiver covers courtyards and bodies only, never
pads or holes, so a pad or hole conflict between two locked parts is printed
and left alone: the grader keeps refusing it until a pose changes.

Exit 0, also when there is nothing to waive; 2 on a bad argument or a failed
check_assembly run.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile

KRT_TOOL = {'scope': ['placement', 'combined'], 'kind': 'instrument'}   # writes the intent, never the board

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))))
for _d in ('py_router', 'py_placer'):
    _p = os.path.join(ROOT, _d)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from kicad_parser import parse_kicad_pcb  # noqa: E402

WAIVABLE = ('courtyard', 'fab')
WHY = ("both parts are locked at the input's poses: the overlap is the "
       "designer's (two footprints for one site, or a part drawn over "
       "another), and no placement move can clear it")


def locked_pairs(board):
    """({(a, b): [kinds]} of waivable overlaps, {(a, b): [kinds]} of the
    rest) between parts that are both locked, as check_assembly grades the
    board."""
    pcb = parse_kicad_pcb(board)
    locked = {r for r, fp in pcb.footprints.items() if getattr(fp, 'locked', False)}
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, 'assembly.json')
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            os.path.join(ROOT, 'py_tools', 'check_assembly.py'),
                            board, '--json', out],
                           capture_output=True, text=True)
        if not os.path.isfile(out):
            sys.exit(f'waive_locked_overlaps: check_assembly wrote no report '
                     f'(exit {r.returncode}): {r.stderr.strip()[-400:]}')
        doc = json.load(open(out, encoding='utf-8'))
    waive, hard = {}, {}
    for p in doc.get('pairs') or ():
        a, b = p.get('a'), p.get('b')
        if a not in locked or b not in locked:
            continue
        key = tuple(sorted((a, b)))
        (waive if p.get('kind') in WAIVABLE else hard).setdefault(key, set()).add(p.get('kind'))
    return ({k: sorted(v) for k, v in waive.items()},
            {k: sorted(v) for k, v in hard.items()})


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('board', help='the prepared board (fixed parts locked)')
    ap.add_argument('intent', help='the floorplan intent JSON, updated in place')
    a = ap.parse_args(argv)
    for f in (a.board, a.intent):
        if not os.path.isfile(f):
            print(f'waive_locked_overlaps: no such file: {f}', file=sys.stderr)
            return 2
    waive, hard = locked_pairs(a.board)
    doc = json.load(open(a.intent, encoding='utf-8'))
    have = {tuple(sorted(w.get('pair') or ())) for w in doc.get('overlap_waivers') or ()}
    new = [k for k in sorted(waive) if k not in have]
    for k in new:
        doc.setdefault('overlap_waivers', []).append(
            {'pair': list(k), 'why': WHY,
             'context': {'overlaps': waive[k], 'by': 'waive_locked_overlaps.py'}})
    if new:
        with open(a.intent, 'w', encoding='utf-8') as f:
            json.dump(doc, f, indent=2)
            f.write('\n')
    print(f'waive_locked_overlaps: {len(waive)} overlapping pair(s) of locked parts, '
          f'{len(new)} newly waived in {a.intent}'
          + (': ' + ', '.join(f'{x}<->{y}' for x, y in new) if new else ''))
    for (x, y), kinds in sorted(hard.items()):
        print(f'  WARNING: {x}<->{y} also conflict by {", ".join(kinds)}: no waiver covers '
              f'pads or holes, so this pair stays refused until one of them moves')
    return 0


if __name__ == '__main__':
    sys.exit(main())

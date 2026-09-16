#!/usr/bin/env python3
"""dedupe_boards.py STEM [STEM...] -- the stems whose `.kicad_pcb` copper is
DISTINCT, in the order given, first occurrence kept.

The chain's fanout portfolio plans the board both ways and routes every
candidate twice; on the rungs where the two plans come out the same, that
is two braids of the same copper for nothing. Measured on the four rungs:
the joint re-fan changes the board at K35 and K51 and is copper-identical
at K28 and K41, so this check pays for itself on half of them.

Compares SEGMENTS and VIAS only -- uuids and the sibling project differ on
every write and say nothing about the copper (the repo rule: never hash or
whole-file-diff a .kicad_pcb to decide identity).
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402


def fingerprint(board):
    pcb = parse_kicad_pcb(board)
    segs = sorted((round(s.start_x, 4), round(s.start_y, 4), round(s.end_x, 4),
                   round(s.end_y, 4), s.layer, s.net_id, round(s.width, 4))
                  for s in pcb.segments)
    vias = sorted((round(v.x, 4), round(v.y, 4), round(v.size, 4),
                   round(v.drill, 4), v.net_id) for v in pcb.vias)
    return (tuple(segs), tuple(vias))


def main(argv):
    seen, out = [], []
    for stem in argv[1:]:
        board = stem + '.kicad_pcb'
        if not os.path.isfile(board):
            continue
        try:
            fp = fingerprint(board)
        except Exception as e:                       # noqa: BLE001
            # a board we cannot read is kept, not dropped: dropping it would
            # silently shrink the portfolio
            print(f'dedupe_boards: {board} unreadable ({e}) -- kept', file=sys.stderr)
            out.append(stem)
            continue
        if fp in seen:
            print(f'dedupe_boards: {os.path.basename(board)} is copper-identical '
                  f'to an earlier candidate -- dropped', file=sys.stderr)
            continue
        seen.append(fp)
        out.append(stem)
    print(' '.join(out))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))

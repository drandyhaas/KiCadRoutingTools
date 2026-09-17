#!/usr/bin/env python3
"""REPRODUCTION of an OPEN, MERGE-BLOCKING bug (#622 review, 2026-09-17).

NOT under tests/ on purpose: `run_all.py` globs `test_*.py`, and this
FAILS today. It is a reproduction, not a gate, until the semantics below
are decided by someone who owns them.

A back-side BGA must honour the layer NAMES its caller passed.

`to_front_frame` renames F.Cu<->B.Cu across the whole board, so every
argument that NAMES a layer has to travel with it. Forwarded unmapped, a
forbidding `layer_costs` entry landed on the MIRROR of the layer the
caller forbade -- so the forbidden face was exactly the one that got the
copper, and a declared plane was modelled on the wrong side.

This needs MORE THAN TWO copper layers to see at all: the default
['F.Cu','B.Cu'] is symmetric under the rename, which is why every
existing flip test passes either way. It reuses the working fixture from
test_fanout_flip_frame and only widens the stackup.
"""
import contextlib
import io
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..', '..')
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

from kicad_parser import BoardInfo            # noqa: E402
from bga_fanout import generate_bga_fanout    # noqa: E402
import test_fanout_flip_frame as base         # noqa: E402

# A caller fans a part out with ITS OWN layer first -- layers[0] is the
# "top escape layer" the engine places edge escapes on, and forbidding it
# is refused outright. So each side gets its own order, and the layer
# forbidden is the FAR face in both cases.
LAYERS = {'F': ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'],
          'B': ['B.Cu', 'In2.Cu', 'In1.Cu', 'F.Cu']}
STACK = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']


def _fixture4(side):
    """base's fixture, on a 4-layer stackup."""
    pcb = base._fixture(side)
    pcb.board_info = BoardInfo(
        layers={0: 'F.Cu', 1: 'In1.Cu', 2: 'In2.Cu', 31: 'B.Cu'},
        copper_layers=list(STACK), board_bounds=(0.0, 0.0, 20.0, 20.0))
    return pcb


def _fan(pcb, side, layer_costs):
    names = [n.name for n in pcb.nets.values() if n.name.startswith('N')]
    with contextlib.redirect_stdout(io.StringIO()):
        tracks, vias, _rm, _failed = generate_bga_fanout(
            pcb.footprints['U1'], pcb, net_filter=names, layers=list(LAYERS[side]),
            layer_costs=list(layer_costs), track_width=0.1, clearance=0.1,
            via_size=0.25, via_drill=0.15, exit_margin=0.5,
            escape_method='auto', plane_drop='off')
    by = {}
    for t in tracks:
        by[t['layer']] = by.get(t['layer'], 0) + 1
    return by


def main():
    bad = []
    for side, forbid in (('F', 'B.Cu'), ('B', 'F.Cu')):
        costs = [(-1.0 if L == forbid else 1.0) for L in LAYERS[side]]
        by = _fan(_fixture4(side), side, costs)
        n_forbidden = by.get(forbid, 0)
        # the run must lay copper at all, or the check is vacuous
        if sum(by.values()) == 0:
            bad.append(f'{side}-side laid NO copper -- the check would be vacuous')
            print(f'  {side}-side part, {forbid} forbidden -> NO COPPER (vacuous)')
            continue
        ok = n_forbidden == 0
        print(f'  {side}-side part, {forbid} forbidden -> '
              f'{dict(sorted(by.items()))}  '
              f'{"OK" if ok else "FAIL: copper on the FORBIDDEN layer"}')
        if not ok:
            bad.append(f'{side}-side: {n_forbidden} track(s) on the forbidden {forbid}')
    if bad:
        print('FAIL: ' + '; '.join(bad))
        return 1
    print('PASS: a forbidding layer cost is honoured on both faces')
    return 0


if __name__ == '__main__':
    sys.exit(main())

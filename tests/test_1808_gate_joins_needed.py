#!/usr/bin/env python3
"""#1808: the improvement gate counts the joins each net still needs.

The gate compared, per net, (connected, disconnected pads). Disconnected pads
are the pads outside a net's LARGEST piece, so cutting an already-open net's
other piece into more pieces moved neither number: an open net taken from 2
pieces to 6, the same 5 pads stranded, graded "broke 0, worsened 0 --
ACCEPTED" while board_score's `broken` rose by 4. The map now carries the
joins each net needs (pieces - 1), the comparison names a net cut into more
pieces (`fragmented`) and sums the joins, and the verdict rejects a run that
raises that sum.

Part A is the decision logic on hand-built maps; part B builds two boards
that differ only in one open net's tracks and grades them with the calls the
gate makes on its input and output files.
"""
import contextlib
import io
import os
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, HERE)

from improvement_gate import (net_connectivity_map, compare_connectivity,  # noqa: E402
                              gate_verdict, format_report)
from run_utils import evidence  # noqa: E402

_failures = []


def check(label, cond, detail=''):
    print(f"{'PASS' if cond else 'FAIL'}: {label}" + (f" -- {detail}" if not cond and detail else ''))
    if not cond:
        _failures.append(label)


def _cmp(before, after, names=None):
    names = names or {}
    return compare_connectivity(before, after,
                                lambda nid: names.get(nid, f"n{nid}"))


# ---------------------------------------------------------------- A. logic
# (connected, disconnected pads, joins needed) per net id.
print("A. decision logic")
c = _cmp({1: (False, 5, 1)}, {1: (False, 5, 5)}, {1: '+3V3'})
check("an open net cut from 2 pieces to 6, pads unchanged, is named",
      c['fragmented'] == [('+3V3', 2, 6)] and not c['worsened'], str(c))
check("...its joins rise 1 -> 5", (c['joins_needed_before'],
                                   c['joins_needed_after']) == (1, 5))
check("...and the run is rejected", gate_verdict(c) == 'reject')
_head = format_report(c, gate_verdict(c), 'x').splitlines()[0]
check("...on a head line that names it",
      'cut 1 open net(s) into more pieces [+3V3 2->6]' in _head
      and _head.endswith('REJECTED'), _head)

# Connects one open net, cuts another from 2 pieces to 3 with the same pads:
# the joins stay level (1 + 1 -> 0 + 2), so the run ships -- and the cut net
# is still named.
c = _cmp({1: (False, 1, 1), 2: (False, 3, 1)},
         {1: (True, 0, 0), 2: (False, 3, 2)}, {1: '/A', 2: '/B'})
check("one net connected, another cut 2 -> 3: joins level, accepted",
      c['gained'] == ['/A'] and c['fragmented'] == [('/B', 2, 3)]
      and (c['joins_needed_before'], c['joins_needed_after']) == (2, 2)
      and gate_verdict(c) == 'accept', str(c))

# Controls, as before #1808.
c = _cmp({1: (True, 0, 0), 2: (False, 2, 1)},
         {1: (False, 2, 1), 2: (True, 0, 0)})
check("control: an exact draw (one lost, one gained, same pads, same "
      "joins) still ships", gate_verdict(c) == 'accept' and not c['fragmented'])
c = _cmp({1: (True, 0, 0), 2: (False, 3, 2)},
         {1: (True, 0, 0), 2: (False, 3, 2)})
check("control: identical boards accept with empty lists",
      gate_verdict(c) == 'accept' and not c['lost'] and not c['gained']
      and not c['worsened'] and not c['fragmented'])
c = _cmp({1: (False, 5, 1)}, {1: (False, 6, 6)}, {1: '+3V3'})
_head = format_report(c, gate_verdict(c), 'x').splitlines()[0]
check("control: a stranded pad more still reads 'worsened 1 [+3V3 5->6]'",
      'worsened 1 [+3V3 5->6]' in _head and _head.endswith('REJECTED'), _head)
# A grade with no piece count (2-tuple: KiCad's fill on one side only) keeps
# the net out of the joins sum instead of guessing.
c = _cmp({1: (False, 5, 1), 2: (False, 1)}, {1: (False, 5, 1), 2: (False, 1, 4)})
check("an entry without a piece count stays out of the joins sum",
      c['joins_compared'] == 1 and gate_verdict(c) == 'accept', str(c))


# ------------------------------------------------------------ B. two boards
print("B. two boards that differ in one open net's tracks")
BOARD = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')
evidence(BOARD, 'the in-repo board the boards are built from')
from kicad_parser import parse_kicad_pcb  # noqa: E402


def _load(path):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(path)


pcb = _load(BOARD)
nid = next(i for i, n in pcb.nets.items() if n.name == '+3V3')
pads = sorted(pcb.pads_by_net[nid], key=lambda p: (p.global_x, p.global_y))


def _seg(a, b):
    return (f'\t(segment (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) '
            f'(width 0.2) (layer "F.Cu") (net "+3V3"))\n')


def _chain(ps):
    return ''.join(_seg((a.global_x, a.global_y), (b.global_x, b.global_y))
                   for a, b in zip(ps, ps[1:]))


main, island = pads[:-5], pads[-5:]
a = (island[0].global_x, island[0].global_y)
b = (island[1].global_x, island[1].global_y)
fragment = _seg((a[0] + (b[0] - a[0]) * 0.4, a[1] + (b[1] - a[1]) * 0.4),
                (a[0] + (b[0] - a[0]) * 0.6, a[1] + (b[1] - a[1]) * 0.6))
text = open(BOARD, encoding='utf-8').read()
k = text.rindex(')')
# before:  one 34-pad piece, and the last 5 pads joined to each other
# after:   the 5 pads' 4 tracks gone, a part of one left touching no pad
# control: as after, and one pad of the large piece cut off too
boards = {'before': _chain(main) + _chain(island),
          'after': _chain(main) + fragment,
          'control': _chain(main[:-1]) + fragment}
tmp = tempfile.mkdtemp(prefix='gate1808_')
try:
    maps = {}
    for name, tracks in boards.items():
        path = os.path.join(tmp, name + '.kicad_pcb')
        with open(path, 'w', encoding='utf-8') as f:
            f.write(text[:k] + tracks + text[k:])
        maps[name] = net_connectivity_map(_load(path))
finally:
    shutil.rmtree(tmp, ignore_errors=True)
check("the two boards' map entries for +3V3 differ (2 pieces vs 6)",
      maps['before'][nid] == (False, 5, 1) and maps['after'][nid] == (False, 5, 5),
      f"{maps['before'][nid]} / {maps['after'][nid]}")
nm = (lambda i: pcb.nets[i].name)
c = compare_connectivity(maps['before'], maps['after'], nm)
r = format_report(c, gate_verdict(c), 'x').splitlines()[0]
check("before -> after: +3V3 named 2 -> 6 pieces, verdict reject",
      c['fragmented'] == [('+3V3', 2, 6)] and gate_verdict(c) == 'reject'
      and r.endswith('REJECTED'), r)
check("...the joins rise by 4, board_score's `broken` delta",
      c['joins_needed_after'] - c['joins_needed_before'] == 4,
      f"{c['joins_needed_before']} -> {c['joins_needed_after']}")
c = compare_connectivity(maps['before'], maps['control'], nm)
r = format_report(c, gate_verdict(c), 'x').splitlines()[0]
check("control: one more stranded pad still reads worsened 1 [+3V3 5->6], "
      "REJECTED", 'worsened 1 [+3V3 5->6]' in r and r.endswith('REJECTED'), r)

print()
if _failures:
    print(f"{len(_failures)} FAILURE(S): " + "; ".join(_failures))
    sys.exit(1)
print("All #1808 gate joins checks passed")

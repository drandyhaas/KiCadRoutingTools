#!/usr/bin/env python3
"""The bus step hands its bus on: another net's copper the step laid goes back first.

  python3 tests/test_622_bus_handed_on.py

A bus step that left another net's copper in a violation refused the whole bus for it: on sdi_mipi_conv's SDI bus
14 nets laid clean were refused for one violation on no bus net. On kicad_files/flat_hierarchy, with tracks of two
nets written onto it, this pins:

1. route_bus.changed_nets: of the nets named, those whose tracks or vias differ from the board as it came -- the one
   given a new track, not the one left as it was;
2. route_bus.put_back of that net: its copper as the board had it, the step's taken out.
"""
import contextlib
import io
import os
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
import route_bus as rb  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402

fails = []


def check(cond, msg):
    print(('PASS ' if cond else 'FAIL ') + msg)
    if not cond:
        fails.append(msg)


SRC = os.path.join(ROOT, 'kicad_files', 'flat_hierarchy')


def with_tracks(src, dst, tracks):
    """`src` written to `dst` with the segments `tracks` [(net name, x0, y0, x1, y1)] on F.Cu"""
    txt = open(src, encoding='utf-8').read()
    seg = ''.join(f'  (segment (start {a} {b}) (end {c} {d}) (width 0.2) (layer "F.Cu") (net "{n}") '
                  f'(uuid "00000000-0000-0000-0000-{i:012d}"))\n' for i, (n, a, b, c, d) in enumerate(tracks))
    end = txt.rstrip().rfind(')')
    open(dst, 'w', encoding='utf-8').write(txt[:end] + seg + txt[end:])
    shutil.copy(src[:-len('.kicad_pcb')] + '.kicad_pro', dst[:-len('.kicad_pcb')] + '.kicad_pro')


with contextlib.redirect_stdout(io.StringIO()):
    pcb0 = parse_kicad_pcb(SRC + '.kicad_pcb')
nets = sorted(n.name for n in pcb0.nets.values() if n.name and not n.name.startswith('unconnected'))[:2]
if len(nets) < 2:
    raise SystemExit(f'BROKEN TEST: the board has no two nets to draw on: {nets}')
A, B = nets
short = lambda n: n.split('/')[-1]
with tempfile.TemporaryDirectory() as td:
    board, routed = os.path.join(td, 'in.kicad_pcb'), os.path.join(td, 'routed.kicad_pcb')
    with_tracks(SRC + '.kicad_pcb', board, [(A, 10, 10, 12, 10), (B, 10, 20, 12, 20)])
    # (the step's board: A's track as it was, B given another)
    with_tracks(board, routed, [(B, 30, 30, 32, 30)])
    # 1.
    got = rb.changed_nets(board, routed, {short(A), short(B)})
    check(got == {short(B)}, f'1: the net the step laid copper on is changed, the other not: {sorted(got)}')
    # 2.
    with contextlib.redirect_stdout(io.StringIO()):
        pcb_in = parse_kicad_pcb(board)
    rb.put_back(routed, board, pcb_in, sorted(rb.others_full(pcb_in, got)))
    check(rb.changed_nets(board, routed, {short(A), short(B)}) == set(),
          '2: put back, the net\'s copper is the board\'s as it came')
print('=' * 60)
print(f'{len(fails)} FAILED' if fails else 'ALL PASS')
sys.exit(1 if fails else 0)

#!/usr/bin/env python3
"""The bus step hands its bus on: another net's copper the step laid goes back first, and a part the shared array's
cap nudge cannot move is held where it stands for the field planned again round it.

  python3 tests/test_622_bus_handed_on.py

A shared array (route_bus --escape, ESCAPE_JOINT) is fanned whole each round, its other nets with the bus, and the
passives beneath it nudged off that copper after; on sdi_mipi_conv's SDI bus on four layers the nudge could not move
C118 off a DDR3 address via the field had planned under it, and the bus step refused the whole bus, laid 14 nets clean,
for that one violation on no bus net. On kicad_files/flat_hierarchy, with tracks of two nets written onto it, this
pins:

1. route_bus.changed_nets: of the nets named, those whose tracks or vias differ from the board as it came -- the one
   given a new track, not the one left as it was;
2. route_bus.put_back of that net: its copper as the board had it, the step's taken out;
3. escape_plan.lock_parts: a two-pad cap locked -- read as locked (placement.parser.extract_locked_refs), and so out of
   the cap step's movable parts and in the engine's immovable foreign pads -- and unlocked again, the board byte for
   byte as it was; a part already locked is left as it is.
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
import escape_plan as ep  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from placement.parser import extract_locked_refs  # noqa: E402

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
    # 3.
    caps = sorted(r for r, f in pcb0.footprints.items() if r.startswith('C') and len(f.pads) == 2)[:2]
    lk, un = os.path.join(td, 'lk.kicad_pcb'), os.path.join(td, 'un.kicad_pcb')
    base_locked = extract_locked_refs(board)
    got_lk = ep.lock_parts(board, lk, set(caps))
    check(sorted(got_lk) == caps and extract_locked_refs(lk) - base_locked == set(caps),
          f'3: {caps} locked, and read as locked: {sorted(extract_locked_refs(lk) - base_locked)}')
    ep.lock_parts(lk, lk, set(caps[:1]))
    check(open(lk).read().count('(locked yes)') == open(board).read().count('(locked yes)') + 2,
          '3: a part already locked is left as it is')
    ep.lock_parts(lk, un, set(caps), unlock=True)
    check(open(un, encoding='utf-8').read() == open(board, encoding='utf-8').read(),
          '3: unlocked, the board byte for byte as it was')
print('=' * 60)
print(f'{len(fails)} FAILED' if fails else 'ALL PASS')
sys.exit(1 if fails else 0)

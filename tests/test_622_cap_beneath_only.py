#!/usr/bin/env python3
"""The cap step's --beneath-only (placement.fanout_clearance, beneath_only): a passive is moved only when it is
BENEATH a BGA's package, and only to poses that keep it beneath.

  python3 tests/test_622_cap_beneath_only.py

awx's joint fanout runs the cap step once, after its first round's fanout, and the whole route then holds the parts
where it left them; nudged freely, the zynq's C160, C161 and R6 -- beside U2, in the channel -- went 1.2 mm under
U2's edge column, across the DDR bus's berths, and every round after met them there. On kicad_files/
orangecrab_ext_pll.kicad_pcb at --clearance 0.1 (the tracked board the step moves 18 parts on):

1. CONTROL, the liveness check: without the flag, some part the step may move is NOT beneath a BGA's package, or
   ends a move outside it. If none is, the arm below tests nothing, and the test says so rather than passing.
2. WITH IT: every part the step may move has its pads beneath a package, and every one it moved ends with its pads
   still within that package.
"""
import contextlib
import io
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path[:0] = [os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_placer')]
with contextlib.redirect_stdout(io.StringIO()):
    from kicad_parser import parse_kicad_pcb  # noqa: E402
    from placement import fanout_clearance as FC  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'orangecrab_ext_pll.kicad_pcb')
BAD = []


def check(ok, what):
    print(('ok    ' if ok else 'FAIL  ') + what)
    if not ok:
        BAD.append(what)


def run(beneath_only):
    """the step's final state (the on_move hook's: every frame is the same object) and its result"""
    seen = []
    with contextlib.redirect_stdout(io.StringIO()):
        res = FC.repair_fanout_clearance(parse_kicad_pcb(BOARD), BOARD, clearance=0.1, beneath_only=beneath_only,
                                         on_move=lambda st: seen.append(st))
    return (seen[0] if seen else None), res


if not os.path.isfile(BOARD):
    raise SystemExit(f'BROKEN TEST: {BOARD} is missing')
with contextlib.redirect_stdout(io.StringIO()):
    pcb = parse_kicad_pcb(BOARD)
courtyards = FC.extract_courtyard_bboxes(BOARD)
pkgs = [FC.package_box(fp, courtyards) for fp in FC.find_components_by_type(pcb, 'BGA')]
if not pkgs:
    raise SystemExit('BROKEN TEST: the board has no BGA')
moved = lambda c: (round(c.x, 4), round(c.y, 4), round(c.rot, 3)) != (round(c.seed_x, 4), round(c.seed_y, 4),
                                                                    round(c.seed_rot, 3))
inside_any = lambda c: any(FC._inside(c.pad_bbox(), b) for b in pkgs)

# 1. control
st0, res0 = run(False)
if st0 is None:
    raise SystemExit('BROKEN TEST: the step never reached its move loop (no on_move frame)')
outside = sorted(r for r, c in st0.caps.items() if FC.pads_beneath(pcb.footprints[r], pkgs) is None)
left = sorted(r for r, c in st0.caps.items() if moved(c) and not inside_any(c))
check(bool(outside or left), f'control: without the flag, {len(st0.caps)} movable parts -- {len(outside)} not beneath a '
                             f'package {outside[:6]}, {len(left)} moved out of one {left[:6]} (none: the arm below '
                             f'tests nothing)')

# 2. with the flag
st1, res1 = run(True)
if st1 is None:
    raise SystemExit('BROKEN TEST: the step never reached its move loop with --beneath-only')
not_beneath = sorted(r for r, c in st1.caps.items() if c.beneath is None)
check(not not_beneath, f'every movable part is beneath a package ({len(st1.caps)} parts; not: {not_beneath[:6]})')
strayed = sorted(r for r, c in st1.caps.items() if c.beneath is not None and not FC._inside(c.pad_bbox(), c.beneath))
n_moved = sum(1 for c in st1.caps.values() if moved(c))
check(not strayed, f'every moved part stays beneath its package ({n_moved} moved; strayed: {strayed[:6]})')
check(set(st1.caps) <= set(st0.caps), 'the flag only narrows which parts move')

print(f'\n{"PASS" if not BAD else "FAIL"}: {len(BAD)} failure(s)' + (': ' + '; '.join(BAD) if BAD else ''))
sys.exit(1 if BAD else 0)

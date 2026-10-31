#!/usr/bin/env python3
"""#1551: bga_fanout checks its escapes against the copper ALREADY on the board.

The channel engine's collision checks (layer assignment, collision detection
and resolution, rebalancing, the pad-aware repair) read the board's copper only
with check_for_previous -- the "skip balls already fanned" option, off by
default on both fronts -- and never read its vias at all. So a fanout run on a
board that already carried copper by the array validated its escapes against
each other only: a second fanout of one array laid a channel track straight
along an earlier call's escape, printed "Validated: No collisions" and exited 0.

Two inner balls of a 0.8 mm BGA that share one channel row (they take two
layers when fanned in one call) are the fixture:

  1. a SECOND call: the first ball fanned alone, then the second on its
     output. The second ball's channel copper must keep clearance from the
     first call's escape (it used to lie on it, on the same layer).
  2. a via already on the board (another net's) in the second ball's channel:
     its escape must keep clearance from that via (it used to run through it).

Each run's copper is graded edge to edge against the copper it was given, and
by the step's own `drc_grazes`.

    python3 tests/test_1551_bga_fanout_board_copper.py
"""
import contextlib
import io
import json
import os
import re
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

from copy_board import copy_board                                  # noqa: E402
from run_utils import check as run_check, evidence                 # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
REF = 'U30'
FIRST, SECOND = '/IO_Banks/DA4', '/IO_Banks/DA1'   # balls B8, C8: one channel row
CLEARANCE = 0.1
ARGS = ['--component', REF, '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu',
        '--track-width', '0.12', '--clearance', str(CLEARANCE),
        '--via-size', '0.3', '--via-drill', '0.15']
#: a through via of a net with no ball on the part, in SECOND's channel row,
#: between the outer ball column and the escape's exit
PLANTED_VIA = (88.1, 96.7, 0.3, 0.15, '/SCL')
TOL = 0.005

failures = []


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}" + (f' -- {detail}' if detail and not cond else ''))
    if not cond:
        failures.append(name)


def fanout(src, dst, nets):
    r = run_check([sys.executable, '-u', '-X', 'utf8',
                   os.path.join(ROOT, 'py_router', 'bga_fanout.py'), src,
                   '--output', dst, '--nets', *nets] + ARGS, accept=True)
    evidence(dst, 'fanout output')
    m = re.search(r'^JSON_SUMMARY: (\{.*\})\s*$', r.stdout, re.M)
    assert m, 'BROKEN TEST: no JSON_SUMMARY\n' + r.stdout[-1500:]
    return r.stdout, json.loads(m.group(1))


def new_copper(before, after, net_name):
    """The segments and vias of `net_name` in `after` that `before` lacks."""
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):   # the fixture's REF** note
        a, b = parse_kicad_pcb(before), parse_kicad_pcb(after)
    nid = next(n.net_id for n in b.nets.values() if n.name == net_name)
    seen = {(round(s.start_x, 4), round(s.start_y, 4), round(s.end_x, 4),
             round(s.end_y, 4), s.layer, s.net_id) for s in a.segments}
    segs = [s for s in b.segments if s.net_id == nid and (
        round(s.start_x, 4), round(s.start_y, 4), round(s.end_x, 4),
        round(s.end_y, 4), s.layer, s.net_id) not in seen]
    vseen = {(round(v.x, 4), round(v.y, 4), v.net_id) for v in a.vias}
    vias = [v for v in b.vias if v.net_id == nid
            and (round(v.x, 4), round(v.y, 4), v.net_id) not in vseen]
    return a, segs, vias


def min_gap(board, segs, vias):
    """Smallest edge-to-edge gap from the new copper to the given board's own
    copper of other nets (same layer; a via is on every layer)."""
    from geometry_utils import segment_to_segment_distance, point_to_segment_distance
    gap = float('inf')
    for s in segs:
        for o in board.segments:
            if o.layer == s.layer and o.net_id != s.net_id:
                d = segment_to_segment_distance(s.start_x, s.start_y, s.end_x, s.end_y,
                                                o.start_x, o.start_y, o.end_x, o.end_y)
                gap = min(gap, d - (s.width + o.width) / 2)
        for v in board.vias:
            if v.net_id != s.net_id:
                d = point_to_segment_distance(v.x, v.y, s.start_x, s.start_y,
                                              s.end_x, s.end_y)
                gap = min(gap, d - (s.width + v.size) / 2)
    for nv in vias:
        for o in board.segments:
            if o.net_id != nv.net_id:
                d = point_to_segment_distance(nv.x, nv.y, o.start_x, o.start_y,
                                              o.end_x, o.end_y)
                gap = min(gap, d - (nv.size + o.width) / 2)
    return gap


def plant_via(src, dst):
    copy_board(src, dst)
    x, y, size, drill, net = PLANTED_VIA
    with open(dst, encoding='utf-8', newline='') as fh:
        txt = fh.read()
    via = (f'\t(via\n\t\t(at {x} {y})\n\t\t(size {size})\n\t\t(drill {drill})\n'
           f'\t\t(layers "F.Cu" "B.Cu")\n\t\t(net "{net}")\n'
           f'\t\t(uuid "15510000-0000-4000-8000-000000000001")\n\t)\n')
    end = txt.rstrip().rfind(')')
    with open(dst, 'w', encoding='utf-8', newline='') as fh:
        fh.write(txt[:end] + via + txt[end:])


def main():
    with tempfile.TemporaryDirectory(prefix='t1551_') as tmp:
        src = os.path.join(tmp, 'in.kicad_pcb')
        copy_board(BOARD, src)

        print('1. a second fanout call, on the first call\'s escape')
        a = os.path.join(tmp, 'a.kicad_pcb')
        b = os.path.join(tmp, 'b.kicad_pcb')
        fanout(src, a, [FIRST])
        _, first_segs, _ = new_copper(src, a, FIRST)
        ok('the first call escapes its ball on the top layer (fixture shape)',
           any(s.layer == 'F.Cu' for s in first_segs), [s.layer for s in first_segs])
        out, js = fanout(a, b, [SECOND])
        board_a, segs, vias = new_copper(a, b, SECOND)
        ok('the second ball escapes', js.get('escaped') == 1, js.get('unescaped_nets'))
        g = min_gap(board_a, segs, vias)
        ok(f'its copper keeps clearance from the first call\'s escape '
           f'(gap {g:.3f} >= {CLEARANCE})', g >= CLEARANCE - TOL, f'{g:.4f}')
        ok('its own drc_grazes is empty', js['drc_grazes'].get('total') == 0,
           js['drc_grazes'])

        print('2. a via already on the board, in the channel')
        pv = os.path.join(tmp, 'via.kicad_pcb')
        c = os.path.join(tmp, 'c.kicad_pcb')
        plant_via(src, pv)
        out, js = fanout(pv, c, [SECOND])
        board_pv, segs, vias = new_copper(pv, c, SECOND)
        ok('the planted via is read back (fixture shape)',
           any(abs(v.x - PLANTED_VIA[0]) < 1e-6 and abs(v.y - PLANTED_VIA[1]) < 1e-6
               for v in board_pv.vias))
        g = min_gap(board_pv, segs, vias) if (segs or vias) else float('inf')
        ok(f'the escape keeps clearance from the via (gap {g:.3f} >= {CLEARANCE})',
           g >= CLEARANCE - TOL, f'{g:.4f}')
        ok('its own drc_grazes is empty', js['drc_grazes'].get('total') == 0,
           js['drc_grazes'])

    print(f"\n{'ALL PASS' if not failures else f'{len(failures)} FAILED'}")
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

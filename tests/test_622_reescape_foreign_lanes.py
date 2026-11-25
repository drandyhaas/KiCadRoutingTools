#!/usr/bin/env python3
"""A re-escape's run-out via stands clear of the OTHER buses' lanes (awx/ends_reescape.py, joint_lanes.foreign).

  python3 tests/test_622_reescape_foreign_lanes.py

Buses off one shared array are routed negotiated: each in a process of its own, on a bench that holds its own
copper alone. What the others have planned or laid reaches a stage through joint_lanes.foreign, which puts their
published lanes on the board the stage has parsed, as copper of their own nets. The re-escape measured its run-out
via -- the backup via of a lane the loop left open -- against the bench's file alone: the via was laid on a lane
another bus had laid on an inner layer, clear of everything the file held, and the two buses' merged board carried a
short between them; the group routed a whole round more to answer it.

1. joint_lanes.foreign puts another bus's laid lane on a parsed board as copper of its net -- under JOINT_FOREIGN
   alone, and never this bus's own;
2. the re-escape's own measure (its foreign_d, read from the script) of a point on that lane: clear before the
   lanes are read, on copper after;
3. ends_reescape reads them where JOINT_FOREIGN is set, on the board it measures against, before it measures.
"""
import ast
import json
import math
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('awx', 'py_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

import awx_settings  # noqa: E402
import joint_lanes  # noqa: E402
from synth import make_net, make_pad, make_pcb, make_seg  # noqa: E402

N, BAD = [0], []


def check(what, got, want):
    N[0] += 1
    ok = got == want
    if not ok:
        BAD.append(what)
    print(f'{"ok  " if ok else "FAIL"} {what}' + ('' if ok else f': got {got!r}, want {want!r}'))


def board():
    """a board of two nets: MINE (this bus's, a stub on F.Cu) and THEIRS (another bus's, no copper on the file)"""
    p1, p2 = make_pad(1, 0.0, 0.0, ref='U1', num='1', net_name='MINE'), make_pad(1, 9.0, 0.0, ref='U2', num='1',
                                                                                 net_name='MINE')
    q1, q2 = make_pad(2, 3.0, -4.0, ref='U1', num='2', net_name='THEIRS'), make_pad(2, 3.0, 4.0, ref='J1', num='1',
                                                                                    net_name='THEIRS')
    nets = {1: make_net(1, 'MINE', pads=[p1, p2]), 2: make_net(2, 'THEIRS', pads=[q1, q2])}
    return make_pcb(nets=nets, segments=[make_seg(0.0, 0.0, 1.0, 0.0, net_id=1, layer='F.Cu')], vias=[], zones=[])


SITE = (3.0, 0.0)           # where a run-out via of MINE would stand: on THEIRS' lane, laid on In3.Cu
with tempfile.TemporaryDirectory() as jd:
    os.makedirs(os.path.join(jd, 'lanes'))
    doc = {'bus': 'J1', 'stage': 'final', 'laid': True, 'vias': [],
           'pieces': [[3.0, -4.0, 3.0, 4.0, 'In3.Cu', 'THEIRS', 0.1]]}
    json.dump(doc, open(os.path.join(jd, 'lanes', 'J1.json'), 'w'))
    json.dump(dict(doc, bus='U2', pieces=[[0.0, 0.0, 9.0, 0.0, 'In3.Cu', 'MINE', 0.1]]),
              open(os.path.join(jd, 'lanes', 'U2.json'), 'w'))
    joint = {'JOINT_DIR': jd, 'JOINT_BUS': 'U2', 'JOINT_TURN': '0,0.0,0.0'}

    # ---- 1. the other bus's lane, copper of its net on the parsed board
    pcb = board()
    with awx_settings.given(joint):
        check('1. no JOINT_FOREIGN: no lane read', (joint_lanes.foreign(pcb), len(pcb.segments)), (0, 1))
    with awx_settings.given(dict(joint, JOINT_FOREIGN='all')):
        check('1. JOINT_FOREIGN: the other bus\'s lane read, this bus\'s own left out', joint_lanes.foreign(pcb), 1)
    theirs = [s for s in pcb.segments if s.net_id == 2]
    check('1. ...as copper of its own net, on its layer',
          [(s.layer, s.start_x, s.start_y, s.end_x, s.end_y, s.width) for s in theirs],
          [('In3.Cu', 3.0, -4.0, 3.0, 4.0, 0.1)])

    # ---- 2. the re-escape's own measure of the site, before the lanes are read and after
    src = open(os.path.join(ROOT, 'awx', 'ends_reescape.py')).read()
    tree = ast.parse(src)
    fns = {n.name: ast.get_source_segment(src, n) for n in tree.body if isinstance(n, ast.FunctionDef)}

    def measure(pcb_):
        ns = {'pcb': pcb_, 'math': math}
        exec(fns['seg_d'], ns)
        exec(fns['foreign_d'], ns)
        return min(ns['foreign_d'](SITE, L, 1) for L in ('F.Cu', 'In3.Cu'))
    check('2. the site against the bench\'s file alone: clear by millimetres', measure(board()) > 2.0, True)
    check('2. ...against the board with the other buses\' lanes read: on copper', measure(pcb) < 0.0, True)

# ---- 3. the script reads them on the board it measures against, before it measures
reads = [n for n in tree.body if isinstance(n, ast.If) and 'JOINT_FOREIGN' in ast.get_source_segment(src, n.test)
         and any(isinstance(c, ast.Call) and ast.get_source_segment(src, c) == 'joint_lanes.foreign(pcb)'
                 for c in ast.walk(n))]
check('3. ends_reescape reads the other buses\' lanes where JOINT_FOREIGN is set', len(reads), 1)
parsed = [n.lineno for n in tree.body if isinstance(n, ast.Assign)
          and ast.get_source_segment(src, n) == 'pcb = parse_kicad_pcb(bench)']
measured = [n.lineno for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'foreign_d']
check('3. ...on the board parsed from the bench, before its measure is defined or asked',
      bool(reads and parsed and measured and parsed[0] < reads[0].lineno < measured[0]), True)

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)

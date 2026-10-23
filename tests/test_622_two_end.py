#!/usr/bin/env python3
"""One source, several ends (route_bus --bus SRC:U4+U3, whole_frame's branches).

  python3 tests/test_622_two_end.py

On boards stood in for, this pins:

1. flow_frame.dest_refs: DEST names one part or several, the trunk's first; quarter_of reads every named part's pads
   as the destination and turns about the first's centre;
2. whole_frame.dest_ranks: the trunk's destination 0, a part north of the trunk -1 (the farther north lower), one south
   of it +1 -- the side its branch leaves the trunk on (branch_side);
3. whole_frame.dest_box_cut: a branch part's box grown by its berths, its far-face cut in that face's middle when its
   berths face the source;
4. the frame's final order (whole_frame.build: each part's rank, then its lanes round it) and the ends model's
   (whole_ends: the place round the part plus RANK_MM a rank) are ONE order -- the solve inverts the frame's, the ends
   model prices its own, and a disagreement is a crossing priced that is not laid or laid that is not priced;
5. braid.net_dest: a net's end at the part it reaches, the first named when it reaches none;
6. route_buses: --bus U1:U4+U3 parses, and a bus handed on through several parts records each as gone through -- a
   later escape bus on U3 plans it whole (ESCAPE_JOINT).
"""
import os
import shutil
import sys
import tempfile
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
import flow_frame  # noqa: E402
import whole_frame as wf  # noqa: E402
import whole_ends  # noqa: E402
import braid as bd  # noqa: E402
import route_bus as rb  # noqa: E402

fails = []


def check(cond, msg):
    if not cond:
        fails.append(msg)
        print('FAIL', msg)


def pad(ref, x, y, nid=0):
    return types.SimpleNamespace(component_ref=ref, global_x=x, global_y=y, net_id=nid)


def part(ref, cx, cy, half=1.0, nid_pads=()):
    """a part's pads: a 3 x 3 grid `half` either side of (cx, cy), and the given (x, y, net id) pads"""
    ps = [pad(ref, cx + dx * half, cy + dy * half) for dx in (-1, 0, 1) for dy in (-1, 0, 1)]
    ps += [pad(ref, x, y, nid) for x, y, nid in nid_pads]
    return types.SimpleNamespace(pads=ps, x=cx, y=cy)


# 1. dest_refs, quarter_of
check(flow_frame.dest_refs('U2') == ['U2'] and flow_frame.dest_refs('U4+U3') == ['U4', 'U3'],
      f'1: dest_refs: {flow_frame.dest_refs("U2")}, {flow_frame.dest_refs("U4+U3")}')
# U1 at the origin; U4 due east, U3 north-east (north is -y): the run's nets N1 to U4, N2 to U3
fps = {'U1': part('U1', 0.0, 0.0, nid_pads=[(1.0, 0.0, 1), (1.0, -1.0, 2)]),
       'U4': part('U4', 20.0, 0.0, nid_pads=[(19.0, 0.0, 1)]),
       'U3': part('U3', 19.0, -12.0, nid_pads=[(18.0, -11.0, 2)]),
       'U7': part('U7', 19.0, 10.0)}
nets = {1: types.SimpleNamespace(name='/N1', pads=[p for f in fps.values() for p in f.pads if p.net_id == 1]),
        2: types.SimpleNamespace(name='/N2', pads=[p for f in fps.values() for p in f.pads if p.net_id == 2])}
pcb = types.SimpleNamespace(nets=nets, footprints=fps)
k, cx, cy = flow_frame.quarter_of(pcb, 'U4+U3', {'N1', 'N2'})
check((k, cx, cy) == (0, 20.0, 0.0), f'1: quarter_of U4+U3: no turn, about U4 -- {(k, cx, cy)}')
k1, _x, _y = flow_frame.quarter_of(pcb, 'U4', {'N1', 'N2'})
check(k1 == 0, f'1: quarter_of U4 alone: {k1}')

# 2. dest_ranks, branch_side
r = wf.dest_ranks(pcb, 'U1', ['U4', 'U3', 'U7'])
check(r == {'U3': -1, 'U4': 0, 'U7': 1}, f'2: ranks north to south: {r}')
check(wf.dest_ranks(pcb, 'U1', ['U4', 'U3']) == {'U3': -1, 'U4': 0}, '2: two parts')
check(wf.dest_ranks(pcb, 'U1', ['U3', 'U4'])['U4'] == 1, '2: the trunk to U3: U4 south of it')
check(wf.branch_side(-1) == 'N' and wf.branch_side(1) == 'S', '2: branch_side')

# 3. dest_box_cut: U3's berths on its west and south faces (toward the source), stubs 0.5 out
berths = [(17.5, -12.5), (17.5, -11.5), (18.5, -10.5), (19.5, -10.5)]
B, c = wf.dest_box_cut(pcb, 'U3', berths)
check(abs(B[0] - 17.5) < 1e-9 and abs(B[3] - (-10.5)) < 1e-9, f'3: grown by the berths: {B}')
check(abs(c - (B[1] + B[3]) / 2) < 1e-9, f'3: the far-face cut its middle: {c} in {B}')

# 4. the frame's final order and the ends model's, one order
prim_box = wf.grown(wf.box_of(pcb, 'U4'), [(18.5, -0.5), (18.5, 0.5), (19.0, -1.5), (21.0, 1.5)])
prim_cut = wf.cut([], prim_box[1], prim_box[3])
lanes = {'a': ('U4', (18.5, -0.5)), 'b': ('U4', (18.5, 0.5)), 'c': ('U4', (19.0, -1.5)), 'd': ('U4', (21.0, 1.5)),
         'e': ('U3', berths[0]), 'f': ('U3', berths[1]), 'g': ('U3', berths[2]), 'h': ('U3', berths[3])}
P = {n: (wf.perim_c(p, prim_box, prim_cut) if d == 'U4' else wf.perim_c(p, B, c)) for n, (d, p) in lanes.items()}
frame_final = sorted(lanes, key=lambda n: (r[lanes[n][0]], P[n]))
ends_final = sorted(lanes, key=lambda n: P[n] + whole_ends.RANK_MM * r[lanes[n][0]])
check(frame_final == ends_final, f'4: the frame\'s final order and the ends model\'s: {frame_final} vs {ends_final}')
check(frame_final[:4] == ['e', 'f', 'g', 'h'], f'4: U3 (north) first, its west face north to south then its south '
                                               f'face west to east: {frame_final}')
check(max(P.values()) < whole_ends.RANK_MM, '4: RANK_MM past any part\'s perimeter')

# 5. net_dest
n3 = types.SimpleNamespace(pads=[pad('U1', 0, 0), pad('U3', 1, 1)])
n0 = types.SimpleNamespace(pads=[pad('U1', 0, 0), pad('R1', 1, 1)])
check(bd.net_dest(n3, 'U4+U3') == 'U3' and bd.net_dest(n0, 'U4+U3') == 'U4' and bd.net_dest(n3, 'U3') == 'U3',
      '5: net_dest')

# 6. route_buses, the step stood in for
check(rb.parse_bus('U1:U4+U3:plain:F.Cu,B.Cu') == ('U1', 'U4+U3', 'plain', 'F.Cu,B.Cu'), '6: parse_bus U4+U3')
calls = []


def step(board, out, src, dest, *a, joint_fanout=False, escape=False, settings=None, **kw):
    calls.append(dict(src=src, dest=dest, escape=escape, settings=dict(settings or {})))
    shutil.copy(board, out)
    shutil.copy(board[:-len('.kicad_pcb')] + '.kicad_pro', out[:-len('.kicad_pcb')] + '.kicad_pro')
    return 0, f'BUS {src}->{dest}'


real = rb.route_bus
rb.route_bus = step
try:
    with tempfile.TemporaryDirectory() as td:
        b0 = os.path.join(td, 'b.kicad_pcb')
        open(b0, 'w').write('(kicad_pcb)\n')
        open(os.path.join(td, 'b.kicad_pro'), 'w').write('{}\n')
        rb.route_buses(b0, os.path.join(td, 'o.kicad_pcb'), [('U1', 'U4+U3', 'plain', None), ('U3', 'U9', 'escape', None)],
                       log=lambda *a: None)
finally:
    rb.route_bus = real
check([(c_['src'], c_['dest']) for c_ in calls] == [('U1', 'U4+U3'), ('U3', 'U9')], f'6: the buses: {calls}')
check(calls[1]['settings'].get('ESCAPE_JOINT') == 'U3' and calls[1]['settings'].get('ESCAPE_JOINT_ENDS') == b0,
      f'6: U3, gone through by U1:U4+U3, planned whole by the escape bus on it: {calls[1]["settings"]}')

print('two_end: ' + ('FAIL' if fails else 'PASS: DEST names several parts, ranked across the trunk, the frame\'s and '
                     'the ends model\'s orders one'))
sys.exit(1 if fails else 0)

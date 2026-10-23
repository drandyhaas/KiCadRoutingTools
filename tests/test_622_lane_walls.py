#!/usr/bin/env python3
"""No escape of another ball goes on from between two laid lanes that close round it (pair_teeth.lane_pockets,
joint_escape.lane_walls / exit_walled).
  python3 tests/test_622_lane_walls.py
The zynq's U1: the DDR3 bus's A14 and A10 leave the east face 1.6 mm apart on F.Cu and run on north-east 0.78 mm
apart, no room for a via between them; the LVDS step's escape field laid OTG_DATA5 and SD_D1 on F.Cu east between
them, and the USB bus's OTG_DATA5 was open on every board after (its tooth's only option, the router's too). On a 4 mm
ball box, its east face at x = 4, lanes at the rule's sizes (close: a via and its clearances and a track, 0.79):
1. two lanes leaving 1.6 apart that close to 0.6 past the face: a pocket from half a pitch inside the face to where
   they close; an exit between them on their layer is walled, on another layer it is not;
2. two lanes that part past the face, and two that keep 0.9 apart (a via's room), close nothing;
3. teeth with no copper past the face (the bus's own round, its lanes not laid) close nothing;
4. a pair's own two legs are no lane pocket (pockets() is theirs);
5. exit_walled reads a move's exit and layer; one with none is never walled.
"""
import os
import sys
import types
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
import pair_teeth as pt  # noqa: E402
import joint_escape as je  # noqa: E402
fails = []


def check(cond, msg):
    print(('PASS ' if cond else 'FAIL ') + msg)
    if not cond:
        fails.append(msg)


BOX = (0.0, 0.0, 4.0, 4.0)
CLOSE, HP = 0.79, 0.4
name_of = {1: 'A', 2: 'B', 3: 'C', 4: 'D', 5: 'X_P', 6: 'X_N'}


def lane(net, pts, layer='F.Cu'):
    return [(a, b, layer, net) for a, b in zip(pts, pts[1:])]


def pockets(segs, nets):
    tooth = pt.teeth(segs, name_of, set(nets), BOX)
    return pt.lane_pockets(segs, name_of, tooth, BOX, HP, CLOSE, {'X': ('X_P', 'X_N')})


# 1. A and B leave at y 1.0 and 2.6, close to 0.6 apart by x 6
segs = lane(1, [(3.6, 1.0), (4.0, 1.0), (5.0, 1.0), (6.0, 1.5)]) + lane(2, [(3.6, 2.6), (4.0, 2.6), (5.0, 2.6), (6.0, 2.1)])
pk = pockets(segs, {'A', 'B'})
check(len(pk) == 1 and pk[0][0] == 'F.Cu', f'1. one pocket on F.Cu between the two -- {pk}')
walls = [(L, q) for L, _u, q in pk]
mv = lambda x, y, L: types.SimpleNamespace(exit_pt=(x, y), layer=L)
check(je.exit_walled(walls, mv(4.0, 1.8, 'F.Cu')), '1. an exit between them on F.Cu is walled')
check(not je.exit_walled(walls, mv(4.0, 1.8, 'B.Cu')), '1. an exit between them on B.Cu is not')
check(not je.exit_walled(walls, mv(4.0, 3.2, 'F.Cu')), '1. an exit beside them (past B) is not')
q = pk[0][2] if pk else []
check(bool(q) and min(p[0] for p in q) == 3.6 and 5.0 < max(p[0] for p in q) < 6.1,
      f'1. the pocket from half a pitch inside the face to where they close -- {q}')
# 2. parting, and keeping a via's room
segs = lane(1, [(4.0, 1.0), (6.0, 0.0)]) + lane(2, [(4.0, 2.6), (6.0, 3.6)])
check(pockets(segs, {'A', 'B'}) == [], '2. two lanes that part close nothing')
segs = lane(1, [(4.0, 1.0), (5.0, 1.0), (7.0, 1.5)]) + lane(2, [(4.0, 2.6), (5.0, 2.6), (7.0, 2.4)])
check(pockets(segs, {'A', 'B'}) == [], '2. two that keep 0.9 apart, a via\'s room, close nothing')
# 3. teeth alone (the copper ends at the face)
segs = lane(1, [(3.6, 1.0), (4.0, 1.0)]) + lane(2, [(3.6, 2.6), (4.0, 2.6)])
check(pockets(segs, {'A', 'B'}) == [], '3. teeth with no copper past the face close nothing')
# 4. a pair's own legs, 0.2 apart past the face
segs = lane(5, [(4.0, 1.0), (6.0, 1.0)]) + lane(6, [(4.0, 1.4), (6.0, 1.2)])
check(pockets(segs, {'X_P', 'X_N'}) == [], '4. a pair\'s own legs are no lane pocket')
# 5. a move with no exit
check(not je.exit_walled(walls, types.SimpleNamespace()), '5. a move with no exit is never walled')
if fails:
    print(f'FAILED {len(fails)}')
    sys.exit(1)
print('ALL PASS')

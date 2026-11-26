#!/usr/bin/env python3
"""The buses' STANDING ORDER in a negotiated group (negotiated_buses.standing_order): of two buses whose plans cross,
the later plans round the earlier one's lanes.

  python3 tests/test_622_standing_order.py

On parts stood in for (pads alone):

1. escape_moves.reaches: a row part a millimetre in front of a ball array -- nearer than the two exits' reaches and a
   channel -- has both exits give way (the source's margin under its whole one, the part's below 0); the same part
   five millimetres off, and a pin header close in front (which gives nothing, the source alone), read as they are;
2. negotiated_buses.close_in_front: the buses whose destination stands so, a bus to several parts by any of them, a
   part not on the board none;
3. negotiated_buses.ordered: the close buses first whatever their lanes, then the bus of fewer lanes, a tie by name --
   and with none close, by the lanes alone;
4. negotiated_buses.lane_room: the room a lane of a bus has -- the box round its lanes' exits (the group's state: each
   lane's tooth and berth options) over its lanes, a side never under ROOM_SIDE; no exit, none;
5. ...and ordered by it: the bus with the least room a lane first -- forty lanes between two arrays before eight to a
   row of pins twice as far, where the lanes alone put the eight first --, the close buses still before any, a bus
   with no room given (no state of the group's) before those with one.
"""
import os
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
import escape_moves as em  # noqa: E402
import negotiated_buses as nb  # noqa: E402

fails = []


def check(cond, msg):
    if not cond:
        fails.append(msg)
        print('FAIL', msg)


def pad(n, x, y, sx, sy, kind='smd', drill=0.0):
    return types.SimpleNamespace(pad_number=str(n), global_x=x, global_y=y, size_x=sx, size_y=sy, pad_type=kind,
                                 drill=drill, layers=['F.Cu', 'F.Mask'], net_name='N', net_id=1)


def part(ref, pads):
    return types.SimpleNamespace(reference=ref, pads=pads, x=pads[0].global_x, y=pads[0].global_y)


# a ball array, 8 x 8 at 0.5 mm, its balls from (0, 0) to (3.5, 3.5)
src = part('U1', [pad(f'{r}{c}', 0.5 * c, 0.5 * r, 0.25, 0.25) for r in range(8) for c in range(8)])


def so8(ref, y0):
    """a two-row pack below the array, four 1.5 x 0.6 pads a side at 1.27 mm, its top pads' copper at y0"""
    return part(ref, [pad(i + 1, 0.2, y0 + 0.3 + 1.27 * i, 1.5, 0.6) for i in range(4)]
                + [pad(8 - i, 3.3, y0 + 0.3 + 1.27 * i, 1.5, 0.6) for i in range(4)])


near, far = so8('U2', 3.5 + 1.3), so8('U3', 3.5 + 5.0)
hdr = part('J1', [pad(i + 1, 2.54 * i, 3.5 + 1.5, 1.7, 1.7, 'thru_hole', 1.0) for i in range(4)])

# 1. reaches
o_s, o_d = em.reaches(src, [near])
check(o_s < em.EXIT_MARGIN and o_d['U2'] < 0, f'1: a row part 1.3 mm in front: both exits give way ({o_s:.3f}, {o_d})')
o_s, o_d = em.reaches(src, [far])
check(o_s == em.EXIT_MARGIN and o_d['U3'] == 0, f'1: the same part 5 mm off: each exit its whole reach ({o_s}, {o_d})')
o_s, o_d = em.reaches(src, [hdr])
check(o_s < em.EXIT_MARGIN and o_d['J1'] == 0, f'1: a pin header close in front: the source alone gives ({o_s:.3f}, {o_d})')

# 2. close_in_front
pcb = types.SimpleNamespace(footprints={p.reference: p for p in (src, near, far, hdr)})
got = nb.close_in_front(pcb, 'U1', ['U2', 'U3', 'J1', 'U3+U2', 'U9'])
check(got == {'U2', 'J1', 'U3+U2'}, f'2: the close buses, a bus to several parts by any of them: {sorted(got)}')
check(nb.close_in_front(pcb, 'U8', ['U2']) == set(), '2: no such source: none')

# 3. ordered
lanes = {'U2': 9, 'U3': 4, 'J1': 6, 'J2': 4}
check(nb.ordered(['U3', 'U2', 'J2', 'J1'], lanes, {'U2', 'J1'}) == ['J1', 'U2', 'J2', 'U3'],
      '3: the close buses first (the fewer lanes of them first), then fewer lanes, a tie by name')
check(nb.ordered(['U3', 'U2', 'J2', 'J1'], lanes) == ['J2', 'U3', 'J1', 'U2'], '3: none close: by the lanes alone')

# 4. lane_room
opt = lambda x, y: ['dogbone', 'right', '*run*', x, y, [x, y], []]      # noqa: E731  (an option as group.json holds it)
mem = {f'D{i}': [[opt(10.0, 1.0 + 0.2 * i)], [opt(16.0, 1.0 + 0.2 * i)]] for i in range(40)}       # 6 x 7.8 mm, 40 lanes
hdr8 = {f'IO{i}': [[opt(10.0, 3.0 + 0.5 * i)], [opt(22.0, 2.54 * i), ['on', 2.5]]] for i in range(8)}  # 12 x 17.78, 8
one = {'CLK': [[opt(10.0, 5.0)], [opt(10.6, 5.2)]]}                                                # a lane 0.6 mm long
check(abs(nb.lane_room(mem) - 6.0 * 7.8 / 40) < 1e-9, f'4: the box round the exits over the lanes ({nb.lane_room(mem)})')
check(abs(nb.lane_room(hdr8) - 12.0 * 17.78 / 8) < 1e-9, f'4: ...a berth\'s note beside its option read past ({nb.lane_room(hdr8)})')
check(nb.lane_room(one) == nb.ROOM_SIDE ** 2, f'4: one short lane: a side never under ROOM_SIDE ({nb.lane_room(one)})')
check(nb.lane_room({}) is None and nb.lane_room(None) is None, '4: no exit: none')

# 5. ordered by the room a lane has
lanes = {'U2': 40, 'J1': 8, 'X1': 1, 'J9': 3}
room = {'U2': nb.lane_room(mem), 'J1': nb.lane_room(hdr8), 'X1': nb.lane_room(one)}
check(nb.ordered(['J1', 'U2'], lanes) == ['J1', 'U2'], '5: by the lanes alone the bus of eight stands first')
check(nb.ordered(['J1', 'U2'], lanes, room=room) == ['U2', 'J1'], '5: by the room a lane has: the forty between two arrays')
check(nb.ordered(['J1', 'U2', 'X1'], lanes, room=room) == ['X1', 'U2', 'J1'], '5: ...a short lane before both')
check(nb.ordered(['J1', 'U2', 'X1'], lanes, {'J1'}, room) == ['J1', 'X1', 'U2'], '5: the close bus still before any')
check(nb.ordered(['J1', 'U2', 'J9'], lanes, room=room) == ['J9', 'U2', 'J1'], '5: a bus with no room given: before those with one')

print('standing order: ' + ('FAIL' if fails else 'PASS: a destination close in front of its source found by the exits\' '
                                                 'own test, its bus first; the rest by the room a lane has, then their lanes'))
sys.exit(1 if fails else 0)

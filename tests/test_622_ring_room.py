#!/usr/bin/env python3
"""A part on a ring's handoff stack: inside it if the ring's lanes fit, round it if not (whole_frame.room_inside,
_seg_rect).

  python3 tests/test_622_ring_room.py

The whole route's frame stacks each ring's lanes across the trunk's handoff line, outside the destination's copper. A
part whose copper lies on that stack is passed INSIDE -- the lanes packed between the destination and it -- when they
fit, each lane's copper a clearance off its neighbours', the first's off the destination's, the last's off the part's;
else the ring goes round it. This pins, on numbers of its own:

1. room_inside: three single lanes (a lane's pitch each) fit in exactly (n - 1) pitches + a track + two clearances, and
   not in a hair less; the packed edge puts the first lane's copper a clearance off the destination's;
2. room_inside never packs the ring's lanes inside the facing face's lanes (their edge is the floor), and then they
   may not fit;
3. a pair's width (a pitch more) counts;
4. _seg_rect: a segment's distance to a rectangle, 0 where they meet or it lies inside.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import whole_frame as wf  # noqa: E402

TW, CL, LM = 0.127, 0.105, 0.252          # track, clearance, a lane's pitch (synth_handoff's rules)


def main():
    print('=' * 60)
    print("a part on a ring's stack: inside if the lanes fit, round if not")
    print('=' * 60)
    fails = []
    e = 5.68                              # the destination's copper's farthest offset on the ring's side
    need = 2 * LM + TW + 2 * CL           # three singles between the destination's copper and the part's
    # 1. exactly the room, and a hair less
    got = wf.room_inside(e, -9.0, 3 * LM, e + need, TW, CL, LM)
    if got is None or abs((got + LM) - (e + CL + TW / 2)) > 1e-9:
        fails.append(f'room_inside, exactly the room: {got} (want the first lane at {e + CL + TW / 2:.4f})')
    if wf.room_inside(e, -9.0, 3 * LM, e + need - 0.001, TW, CL, LM) is not None:
        fails.append('room_inside: packed three lanes into a gap 1 um short of their room')
    # 2. the facing face's lanes beyond the destination's copper: the stack starts there, and no longer fits
    near_e = e + 0.2
    got = wf.room_inside(e, near_e, 3 * LM, e + need + 0.1, TW, CL, LM)
    if got is not None:
        fails.append(f'room_inside packed the ring inside the facing face\'s lanes: {got}')
    if wf.room_inside(e, near_e, 3 * LM, near_e + 3 * LM + TW / 2 + CL, TW, CL, LM) != near_e:
        fails.append('room_inside: with room past the facing face\'s lanes, the stack did not start at their edge')
    # 3. a pair counts its pitch
    pp = 0.3
    if wf.room_inside(e, -9.0, 3 * LM + pp, e + need, TW, CL, LM) is not None:
        fails.append('room_inside: a pair\'s width not counted')
    # 4. _seg_rect
    cases = [(((0, 0), (0, 2), (1, 0.5, 2, 1)), 1.0), (((0, 0), (0, 2), (-1, 0.5, 2, 1)), 0.0),
             (((0, 0), (0, 2), (-0.1, -0.1, 0.1, 0.1)), 0.0), (((0, 0), (2, 2), (2, 0, 3, 0.5)), 2 ** 0.5 * 0.75)]
    for (a, b, r), want in cases:
        got = wf._seg_rect(a, b, r)
        if abs(got - want) > 1e-9:
            fails.append(f'_seg_rect({a}, {b}, {r}) = {got}, want {want}')

    for f in fails:
        print(f'  FAIL: {f}')
    if fails:
        return 1
    print('PASS: the lanes packed inside exactly when they fit, never inside the facing face\'s lanes, a pair\'s pitch '
          'counted; a segment\'s distance to a rectangle')
    return 0


if __name__ == '__main__':
    sys.exit(main())

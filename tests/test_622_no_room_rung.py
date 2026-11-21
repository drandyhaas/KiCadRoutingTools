#!/usr/bin/env python3
"""No room at these sizes, and room on a rung below: the group's ends model ends there, and the group is routed again
a rung down (awx/group_ends.py: room_on_a_rung, short_named, --rungs-below; awx/rules.py: pitches).

  python3 tests/test_622_no_room_rung.py

The group's ends model puts each bus's ends to that bus's whole solve, and answers a refusal with another state. A
bus whose lanes are too short between their two ends for a change of layer at the run's sizes -- a via and its room
at each end do not fit -- has no plan on any ends, and a model answering its refusals went on for twelve checks and
65 minutes without one. The fab escalation has rungs for it: a smaller via needs less room. The model now reads, off
the solve's own measure of each lane, whether a lane the refusal NAMES has no room for a change here and room on a
rung below; a lane so named AGAIN, the model's answer between, ends the model with "no room at these sizes", and the
group's escalation routes the group again a rung down from that end.

1. room_on_a_rung: a lane whose span between its entry and its end is short of its two ends' rooms here has room on
   a rung whose via needs enough less; none on a rung as coarse as the run, none where even the finest is short,
   none with no rung below;
2. short_named: of a refusal's lanes with no room for a change, those it NAMES that have room on a rung below -- not
   one it does not name, not one with no room on any rung, not one out of the state;
3. the rule as the model's loop applies it: a lane named so twice, the model's answer between, is the bus's "no
   room"; named once, or two lanes named once each, is not;
4. a run's pitches are its own sizes' (rules.pitches): the chain's two at the chain's own lane, to the bit; scaled as
   the lane's slice at any other -- finer for a narrower lane, wider for a wider one -- and never under the slice;
   Rules.from_router_config gives them, so a rung resolves the same pitches from its sizes as the ladder gave it.
"""
import os
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
os.environ.pop('AWX_RULES', None)
try:
    import ortools  # noqa: F401
except ImportError:
    print('SKIP: ortools is not installed (the group\'s ends model plans with CP-SAT)')
    sys.exit(77)

import rules as R  # noqa: E402

BAD, N = [], [0]


def check(what, ok, got=None):
    N[0] += 1
    if not ok:
        BAD.append(what)
    print(('ok   ' if ok else 'FAIL ') + what + ('' if ok or got is None else f' -- got {got}'))


def sizes(track, via, clearance=0.09, drill=0.15):
    return R.Rules.from_router_config(types.SimpleNamespace(clearance=clearance, track_width=track, via_size=via,
                                                            via_drill=drill), fan_track=track)


RUN = sizes(0.15, 0.45, drill=0.3)
R.install(RUN)
import group_ends as ge  # noqa: E402

BELOW = [(0.1254, 0.3), (0.1008, 0.25), (0.0762, 0.25)]
G = 0.025


def room(span, need):
    """a lane as the solve measures it: its entry and its end along the frame, the room each end keeps, ..., the step"""
    return [1.0, 1.0 + span, need / 2, need / 2, 0.0, 0, 0, G]


# 1. room_on_a_rung
NEED = 3.08
k1 = sizes(*BELOW[0]).via_need / RUN.via_need
check('1. the first rung\'s via needs less room than the run\'s', 0.7 < k1 < 0.9, k1)
check('1. a lane short here by a little: room on the first rung', ge.room_on_a_rung(room(2.86, NEED), BELOW))
check('1. ...one shorter: none on the first rung, room on a finer one',
      not ge.room_on_a_rung(room(2.27, NEED), BELOW[:1]) and ge.room_on_a_rung(room(2.27, NEED), BELOW))
check('1. ...one too short at the finest sizes: none', not ge.room_on_a_rung(room(1.45, NEED), BELOW))
check('1. no rung below: none', not ge.room_on_a_rung(room(2.86, NEED), []))
check('1. a rung as coarse as the run: none', not ge.room_on_a_rung(room(2.86, NEED), [(RUN.track, RUN.via_size)]))
at_ = NEED * k1 + G
check('1. ...the bar: the span its two ends\' rooms at the rung and one step of the changes\' places',
      ge.room_on_a_rung(room(at_ + 1e-6, NEED), BELOW[:1]) and not ge.room_on_a_rung(room(at_ - 1e-3, NEED), BELOW[:1]))

# 2. short_named
STATE = {n: (0, 0) for n in ('A', 'B', 'C', 'D')}
CHK = {'plan': False, 'lanes': ['A', 'C', 'E'], 'short': ['A', 'B', 'C', 'E'],
       'rooms': {'A': room(2.86, NEED), 'B': room(2.86, NEED), 'C': room(1.45, NEED), 'E': room(2.86, NEED)}}
check('2. the lanes named, short here, with room below', ge.short_named(CHK, STATE, BELOW) == {'A'},
      ge.short_named(CHK, STATE, BELOW))
check('2. ...none with no rung below', ge.short_named(CHK, STATE, []) == set())
check('2. ...none where the refusal names no lane', ge.short_named(dict(CHK, lanes=[]), STATE, BELOW) == set())
check('2. ...none where the solve gave no rooms', ge.short_named(dict(CHK, rooms={}), STATE, BELOW) == set())


# 3. named again
def refused_twice(refusals):
    """the lanes for which the bus has no room: the model's loop over one bus's refusals in turn"""
    tight = set()
    for chk in refusals:
        short = ge.short_named(chk, STATE, BELOW)
        again = sorted(short & tight)
        tight |= short
        if again:
            return again
    return []


one = dict(CHK)
other = {'plan': False, 'lanes': ['B'], 'short': ['A', 'B'], 'rooms': CHK['rooms']}
unnamed = {'plan': False, 'lanes': ['D'], 'short': ['A'], 'rooms': CHK['rooms']}
check('3. a lane named so twice: no room at these sizes', refused_twice([one, unnamed, one]) == ['A'])
check('3. ...named once: not', refused_twice([one]) == [] and refused_twice([one, unnamed, unnamed]) == [])
check('3. ...two lanes named once each: not', refused_twice([one, other]) == [])

# 4. the pitches
check('4. the chain\'s own lane: its own two pitches, to the bit',
      R.pitches(R.TRACK, R.SPEC_CLEARANCE) == (R.LANE_PITCH, R.EXIT_PITCH), R.pitches(R.TRACK, R.SPEC_CLEARANCE))
ref_slice = R.Rules().lane_slice
for trk, clr in ((0.0762, 0.09), (0.1, 0.1), (0.15, 0.1), (0.2, 0.15)):
    lp, ep = R.pitches(trk, clr)
    sl = round(trk + clr + R.HUG_OVER, 6)
    check(f'4. a {trk:g} lane at {clr:g}: the two pitches scaled as its slice, never under it',
          abs(lp - R.LANE_PITCH * sl / ref_slice) < 1e-6 and abs(ep - R.EXIT_PITCH * sl / ref_slice) < 1e-6
          and lp >= sl and ep >= sl, (lp, ep, sl))
check('4. ...finer for a narrower lane, wider for a wider one',
      R.pitches(0.0762, 0.09)[0] < R.LANE_PITCH < R.pitches(0.15, 0.1)[0])
rung = sizes(*BELOW[0])
check('4. a run\'s rules carry them', (rung.lane_pitch, rung.exit_pitch) == R.pitches(BELOW[0][0], 0.09),
      (rung.lane_pitch, rung.exit_pitch))

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)

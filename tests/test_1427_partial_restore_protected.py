#!/usr/bin/env python3
"""#1427: a rip victim put back PARTIALLY is protected from the cleanup's
subtractive passes, like any other unfinished net.

The #134 last resort restores the pieces of a refused net's pre-rip route that
do not collide and prints "net remains PARTIAL for a later reconnect pass".
That net had a result and no `failed_pads_info`, so the cleanup's protect set
(#473, the nets whose spurs are the landing sites the next step welds to) did
not hold it, the dead-end sweep read its restored BGA escape as a spur, took
it, and the ball shipped bare (glasgow U30.D11 on /IO_Banks/QB3, with the
restored net's route open: the escape is the only copper that lands on it).

Rows on `route._unfinished_net_ids`:

  * a result with failed pads is unfinished (as before)
  * a scoped net with no result is unfinished (as before)
  * a result stamped `partial_restore_134` is unfinished
  * a terminal restore of 'full_open' or 'stub' is unfinished; 'full' is not
  * a routed, finished net is not

The full-board repro (about 13 min; the glasgow U30 chain in issue #1427) was
run by hand: with the fix QB3's D11 escape ships, and the other nets the #134
last resort restored keep theirs.

    python3 tests/test_1427_partial_restore_protected.py
"""
import os
import sys

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from route import _unfinished_net_ids                           # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def main():
    routed = {
        1: {'failed_pads_info': [('U1', '3')]},
        2: {'partial_restore_134': True, 'new_segments': []},
        3: {'new_segments': []},                 # routed and finished
        4: {'new_segments': []},                 # terminal restore 'full_open'
        5: {'new_segments': []},                 # terminal restore 'full'
    }
    scoped = [('N6', 6), ('N3', 3)]              # 6 has no result at all
    got = _unfinished_net_ids(routed, scoped, {4: 'full_open', 5: 'full', 7: 'stub'})
    check('a result with failed pads is unfinished', 1 in got)
    check('a scoped net with no result is unfinished', 6 in got)
    check('a #134 partial restore is unfinished', 2 in got)
    check("a terminal restore of 'full_open' is unfinished", 4 in got)
    check("a terminal 'stub' restore with no result is unfinished", 7 in got)
    check("a terminal 'full' restore is finished", 5 not in got)
    check('a routed, finished net is not unfinished', 3 not in got)
    check('no terminal restores given: same answer as without',
          _unfinished_net_ids(routed, scoped) == {1, 2, 6})

    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())

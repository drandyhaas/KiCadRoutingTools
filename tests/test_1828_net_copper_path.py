#!/usr/bin/env python3
"""#1828: `length_matching._net_copper_path`, the test a late group member's
copper must pass before it is meandered: ONE path between its two pads, read
the way KiCad's connectivity reads copper.

Rows:
  1. a clean pad-to-pad chain across a via is one path, ordered pad to pad,
     with every segment oriented along it (a reversed piece is a copy);
  2. joins by overlap: a track end a few um off the next piece's end, a track
     end inside a via's pad, a via inside a pad;
  3. a stub with both ends inside one joint's copper is on no path and is
     left out of the list;
  4. not one path: a spur (a dead-end branch), a loop, a separate piece, a
     via no track reaches, a chain shorter than a track width is not folded
     into one node.

    python3 tests/test_1828_net_copper_path.py
"""
import os
import sys

TESTS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(os.path.dirname(TESTS), 'py_router'))

from synth import make_pad, make_seg, make_via          # noqa: E402
from length_matching import _net_copper_path            # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


PADS = [make_pad(1, 0, 0, num='1'), make_pad(1, 10, 0, num='2', ref='U2')]


def chained(path):
    """Each segment starts where the one before ends (or at a via there)."""
    return all(abs(a.end_x - b.start_x) < 0.3 and abs(a.end_y - b.start_y) < 0.3
               for (_i, a), (_j, b) in zip(path, path[1:]))


def main():
    print("1. a clean chain across a via")
    segs = [make_seg(0, 0, 4, 0), make_seg(4, 0, 4, 2, layer='B.Cu'),
            make_seg(6, 2, 4, 2, layer='B.Cu'), make_seg(10, 0, 6, 2)]
    p = _net_copper_path(segs, [make_via(4, 0), make_via(6, 2)], PADS)
    check("one path, every segment, pad to pad",
          p is not None and [i for i, _s in p] in ([0, 1, 2, 3], [3, 2, 1, 0]),
          str(p and [i for i, _s in p]))
    check("oriented along the path", p is not None and chained(p))
    check("a reversed piece is a copy, the others the objects given",
          p is not None and sum(s is segs[i] for i, s in p) == 2)

    print("2. joins by overlap")
    segs = [make_seg(0, 0, 3, 0), make_seg(3.005, 0, 4.2, 0),     # 5 um offset
            make_seg(4, 0.1, 6, 0.1, layer='B.Cu'),                # end in via pad
            make_seg(6, 0.1, 10, 0)]
    p = _net_copper_path(segs, [make_via(4.1, 0.05), make_via(6, 0.1)], PADS)
    check("an offset end and an end inside a via's pad join", p is not None, str(p))
    segs = [make_seg(0.4, 0.3, 10, 0.3, layer='B.Cu')]
    p = _net_copper_path(segs, [make_via(0.4, 0.3), make_via(10, 0.3)],
                         [make_pad(1, 0, 0, size_x=1, size_y=1),
                          make_pad(1, 10, 0, num='2', ref='U2', size_x=1, size_y=1)])
    check("a via inside a pad joins it", p is not None, str(p))

    print("3. a stub inside one joint's copper")
    segs = [make_seg(0, 0, 10, 0), make_seg(9.9, 0.05, 10.1, 0.05)]
    p = _net_copper_path(segs, [], PADS)
    check("is on no path, and left out of the list",
          p is not None and [i for i, _s in p] == [0], str(p))

    print("4. not one path")
    spur = [make_seg(0, 0, 5, 0), make_seg(5, 0, 10, 0), make_seg(5, 0, 5, 3)]
    check("a spur", _net_copper_path(spur, [], PADS) is None)
    loop = [make_seg(0, 0, 5, 0), make_seg(5, 0, 10, 0), make_seg(5, 0, 5, 3),
            make_seg(5, 3, 7, 3), make_seg(7, 3, 5, 0)]
    check("a loop", _net_copper_path(loop, [], PADS) is None)
    apart = [make_seg(0, 0, 10, 0), make_seg(3, 5, 6, 5)]
    check("a separate piece", _net_copper_path(apart, [], PADS) is None)
    check("a via no track reaches",
          _net_copper_path([make_seg(0, 0, 10, 0)], [make_via(5, 4)], PADS) is None)
    short = [make_seg(0, 0, 5, 0), make_seg(5, 0, 5.05, 0), make_seg(5.05, 0, 5.1, 0),
             make_seg(5.1, 0, 10, 0)]
    p = _net_copper_path(short, [], PADS)
    check("pieces shorter than a track width stay a chain",
          p is not None and len(p) == 4 and chained(p), str(p))
    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())

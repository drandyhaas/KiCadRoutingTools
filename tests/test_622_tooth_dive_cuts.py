#!/usr/bin/env python3
"""A pair whose tooth is a surface stub changes layer in its face band only where its DIVE stands clear of the copper
in front of the tooth (awx/whole_solve.py: dive_clear, blocked_spans; the solve's via cuts where a pair's dive meets
the copper in front of its tooth).

  python3 tests/test_622_tooth_dive_cuts.py

Such a pair may change layer in the face band (its tooth's via window), where the arrays' ends stand thick: its
neighbours' stubs and dog-bone vias, its own bus's among them, which no other cut reads. A memory bus's strobe pair
dived a dive room past its tooth between two neighbours' tooth vias. The embed laid the dive across its way to fit, the
pair router could not lay it, and the dive, moved on along the face, crowded a neighbour there.

1. dive_clear: nothing near the dive is clear; another net's via on a barrel is not, the pair's own net's via is;
2. ...another net's segment across a leg's run beside the dive is not, one well apart is;
3. ...another net's pad on a barrel is not, a non-plated hole's is ignored;
4. ...the dive turns with its line: a via beside the line's barrels on one heading is clear of them on the other;
5. _seg_seg: 0 where two segments cross, else their least distance;
6. blocked_spans merges blocked places a step apart into spans;
7. the solve tests every surface-tooth pair's window and cuts it plainly (a built-in cut is waived in that window).
"""
import os
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

try:
    import whole_solve as ws  # noqa: E402
except ImportError as exc:
    print('SKIP: %s' % exc)
    sys.exit(77)

BAD = []
N = [0]


def check(what, got, want):
    N[0] += 1
    if got != want:
        BAD.append(what)
    print(('ok   ' if got == want else 'FAIL ') + what + ('' if got == want else f': {got!r}, wanted {want!r}'))


def seg(x0, y0, x1, y1, net, w=0.12):
    return types.SimpleNamespace(start_x=x0, start_y=y0, end_x=x1, end_y=y1, width=w, net_id=net)


def via(x, y, net, size=0.25):
    return types.SimpleNamespace(x=x, y=y, size=size, net_id=net)


def pad(x, y, net, sx=0.3, sy=0.3, kind='smd'):
    return types.SimpleNamespace(global_x=x, global_y=y, size_x=sx, size_y=sy, net_id=net, pad_type=kind, shape='rect',
                                 rect_rotation=0.0)


OWN = {1, 2}                    # the pair's two legs' nets
OFF, HALF, LD = 0.3, 0.134, 0.4         # its barrels' offset, each leg's run off the line, the runs' half length
P, T = (0.0, 0.0), (1.0, 0.0)           # the dive at the origin, its line along x: barrels at (0, +-0.3)


def clear(segs=(), vias=(), pads=(), t=T):
    return ws.dive_clear(list(segs), list(vias), list(pads), OWN, P, t, OFF, HALF, LD)


check('1. a dive with no copper near it is clear', clear(), True)
check('1. another net\'s via on a barrel blocks it', clear(vias=[via(0.0, 0.3, 9)]), False)
check('1. ...the pair\'s own net\'s via there does not', clear(vias=[via(0.0, 0.3, 1)]), True)
check('1. another net\'s via 1 mm off the line is clear of it', clear(vias=[via(0.0, 1.3, 9)]), True)
check('2. another net\'s segment across a leg\'s run, clear of the barrels, blocks it',
      clear(segs=[seg(0.35, 0.05, 0.35, 0.25, 9)]), False)
check('2. ...a segment well clear of runs and barrels does not', clear(segs=[seg(-1.0, 1.0, 1.0, 1.0, 9)]), True)
check('2. ...nor the pair\'s own leg\'s segment there', clear(segs=[seg(0.35, 0.05, 0.35, 0.25, 2)]), True)
check('3. another net\'s pad on a barrel blocks it', clear(pads=[pad(0.0, -0.3, 9)]), False)
check('3. ...a non-plated hole there is ignored', clear(pads=[pad(0.0, -0.3, 0, kind='np_thru_hole')]), True)
check('4. along y its barrels stand at (+-0.3, 0): a pad there blocks it', clear(pads=[pad(-0.3, 0.0, 9)], t=(0.0, 1.0)),
      False)
check('4. ...along x that pad stands 0.3 behind the dive\'s middle, under its legs\' runs, and blocks it too',
      clear(pads=[pad(-0.3, 0.0, 9)]), False)
check('4. ...a pad 0.8 mm ahead of the dive on its line is clear of it, on either heading',
      (clear(pads=[pad(0.0, 0.8, 9)], t=(0.0, 1.0)), clear(pads=[pad(0.8, 0.0, 9)])), (True, True))
check('4. ...a pad 0.5 mm beside the line along x blocks a barrel, ahead of it along y it is clear of the barrels '
      'but meets a leg\'s run', (clear(pads=[pad(0.0, 0.5, 9)]), clear(pads=[pad(0.0, 0.5, 9)], t=(0.0, 1.0))),
      (False, False))
check('5. two crossing segments are 0 apart', ws._seg_seg((0, -1), (0, 1), (-1, 0), (1, 0)), 0.0)
check('5. two parallel segments 0.5 apart are 0.5 apart',
      round(ws._seg_seg((0, 0), (1, 0), (0, 0.5), (1, 0.5)), 9), 0.5)
check('6. blocked places a step apart merge into spans',
      ws.blocked_spans([1.0, 1.1, 1.2000001, 2.0, 2.1], 0.1), [[1.0, 1.2000001], [2.0, 2.1]])
check('6. ...no place blocked, no span', ws.blocked_spans([], 0.1), [])

src = open(os.path.join(ROOT, 'awx', 'whole_solve.py')).read()
body = src[src.index('def solve('):]
body = body[:body.index('\ndef ', 10)]
i0 = body.index('for n in sorted(TVIA):\n        if n not in prs')
blk = body[i0:body.index('dive meets the copper', i0)]
check('7. the solve tests each surface-tooth pair\'s window with dive_clear',
      'dive_clear(ctx.base_segments, ctx.base_vias, pads_, nids_, P,' in blk, True)
check('7. ...and cuts it plainly, never as a built-in cut', "blocked_spans(blocked, G)" in blk and "'built'" not in blk,
      True)

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)

#!/usr/bin/env python3
"""A restore that re-emits the input's own objects keeps them on the board (#1310).

route.py's #284 stale strip takes an input original out of the written file
when the writer re-emits a twin at its position, and mirrors that removal into
pcb_data so the board matches the file (the ottercast Net-(C46-Pad1) sliver:
a routed twin, a DIFFERENT object, stays and the original goes).

The #134 last-resort restore of a PRE-EXISTING net breaks that picture. Its
saved route is the net's custody registration, whose segments and vias are
the input file's own objects, and the restore puts them on the write-list. So
each original is its own re-emitted "twin": the file strips the input copy
and writes the emitted one, which is right, but the mirror removed the object
from pcb_data, which left the net with no copper in the engine's model while
the file shipped all of it. Every later pass routed through copper it could
not see: on glasgow (gap6_s2_c, the issue's route.py line) /RD was restored
whole, /D0's octolinear smoothing and a GND tap via landed on it, and the run
shipped 30 DRC violations on an input that grades clean.

Pinned here, on the shared helper route.py's batch_route calls
(`mirror_stale_strip`), with the stale set computed by the real
`compute_stale_input_segments` / `compute_stale_input_vias`:

  1. an original that is itself on the write-list stays on the board;
  2. an original whose re-emitted twin is a different object still leaves
     the board (the ottercast case, unchanged);
  3. board == file: the writer's model (input copy minus the strips, plus the
     write-list) carries exactly the copper pcb_data does;
  4. batch_route calls the helper.

    python3 tests/test_1310_restored_original_stays_on_board.py
"""
import ast
import os
import sys
from collections import Counter
from types import SimpleNamespace

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router')):
    if p not in sys.path:
        sys.path.insert(0, p)

from route import (compute_stale_input_segments,               # noqa: E402
                   compute_stale_input_vias, mirror_stale_strip)

FAILS = []
RD, D0 = 7, 8


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name
          + (f'  {detail}' if detail else ''))


def _seg(x1, y1, x2, y2, net, layer='B.Cu'):
    return SimpleNamespace(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                           width=0.1, layer=layer, net_id=net)


def _via(x, y, net):
    return SimpleNamespace(x=x, y=y, size=0.4, drill=0.2, net_id=net,
                           layers=['F.Cu', 'B.Cu'])


def _key(o):
    if hasattr(o, 'start_x'):
        a = (round(o.start_x, 3), round(o.start_y, 3))
        b = (round(o.end_x, 3), round(o.end_y, 3))
        return ('s', o.net_id, min(a, b), max(a, b), o.layer)
    return ('v', o.net_id, round(o.x, 3), round(o.y, 3))


def main():
    # /RD: pre-existing, ripped, then restored by #134's last resort -- the
    # restore's result carries the input file's OWN objects.
    rd1, rd2 = _seg(71.5, 96.53, 73.72, 96.53, RD), _seg(73.72, 96.53, 74.0, 96.8, RD)
    rd_via = _via(71.3, 91.75, RD)
    restore134 = {'net_id': RD, 'new_segments': [rd1, rd2],
                  'new_vias': [rd_via], 'is_existing_route': True,
                  'partial_restore_134': True}
    # /D0: ripped and rerouted onto a byte-identical span -- a DIFFERENT object.
    d0_orig = _seg(70.8, 91.6, 72.0, 91.6, D0)
    d0_twin = _seg(70.8, 91.6, 72.0, 91.6, D0)
    rerouted = {'net_id': D0, 'new_segments': [d0_twin], 'new_vias': []}

    input_segs, input_vias = [rd1, rd2, d0_orig], [rd_via]
    board = SimpleNamespace(segments=[rd1, rd2, d0_orig, d0_twin],
                            vias=[rd_via])
    results = [restore134, rerouted]
    emitted_s = [s for r in results for s in r['new_segments']]
    emitted_v = [v for r in results for v in r['new_vias']]

    frozen = list(board.segments) + list(board.vias)
    ids = {id(x) for x in frozen}
    stale_s = compute_stale_input_segments(
        {RD: [rd1, rd2], D0: [d0_orig]}, {RD, D0}, board.segments, emitted_s,
        final_ids=ids)
    stale_v = compute_stale_input_vias({RD: [rd_via]}, {RD, D0}, board.vias,
                                       emitted_v, final_ids=ids)
    check('the stale strip takes every re-emitted original out of the file',
          {id(x) for x in stale_s} == {id(rd1), id(rd2), id(d0_orig)}
          and [id(x) for x in stale_v] == [id(rd_via)])

    mirror_stale_strip(board, stale_s, stale_v, emitted_s, emitted_v)

    print('1. the restored originals stay on the board')
    check('/RD keeps both segments and its via',
          rd1 in board.segments and rd2 in board.segments
          and rd_via in board.vias)
    print('2. an original with a different-object twin still leaves')
    check('/D0 keeps its twin and loses the original',
          d0_twin in board.segments
          and not any(s is d0_orig for s in board.segments))

    print('3. board == file')
    stale_ids = {id(x) for x in stale_s + stale_v}
    file_copper = [x for x in input_segs + input_vias if id(x) not in stale_ids]
    file_copper += emitted_s + emitted_v
    want = Counter(_key(x) for x in file_copper)
    got = Counter(_key(x) for x in list(board.segments) + list(board.vias))
    check('the writer ships exactly the copper pcb_data holds', want == got,
          f'file-only {dict(want - got)}, board-only {dict(got - want)}')

    print('4. batch_route mirrors through the helper')
    tree = ast.parse(open(os.path.join(ROOT, 'py_router', 'route.py'),
                          encoding='utf-8').read())
    br = [f for f in tree.body if isinstance(f, ast.FunctionDef)
          and f.name == 'batch_route']
    calls = [n for n in ast.walk(br[0]) if isinstance(n, ast.Call)
             and getattr(n.func, 'id', None) == 'mirror_stale_strip'] if br else []
    check('batch_route calls mirror_stale_strip', len(calls) == 1,
          f'{len(calls)} call(s)')

    print(f"\n{'ALL PASS' if not FAILS else f'{len(FAILS)} FAILED: ' + ', '.join(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())

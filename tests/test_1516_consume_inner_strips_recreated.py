#!/usr/bin/env python3
"""Issue #1516: consume_inner_strips must not delete copper the inner run
re-created with the same coordinates as a piece it stripped.

An in-memory batch_route (a plane repair's reconnect or guaranteed-join
sub-run) reports the copper its cleanup removed in ``segments_to_remove``.
``merge_collinear_segments`` rebuilds a whole chain as NEW objects --
including pieces that did not change -- and strips every original. The
consumer matched the strip channel by coordinate VALUE, so an unchanged
piece's live re-creation (in the inner run's ``new_segments``, already in the
write list and pcb_data) was deleted along with its stripped original: a
45-degree piece vanished and the chain shipped a segment-endpoint-gap.

Value matching itself stays (write-list entries are dicts, never the inner
run's objects, and pcb_data duplicates re-added by partial restores must
still go). What changes: a value the inner run's own output still holds is
kept, exactly as many times as that output holds it.

  1. the issue's chain: the re-created piece survives in the write list and
     pcb_data, the merged original pieces are still consumed;
  2. a stale write-list emission of the stripped original (the same value,
     emitted before the sub-run) is dropped, so the value ships ONCE;
  3. a pcb_data duplicate of a re-created value that is not the inner run's
     own object is dropped, the inner run's object kept;
  4. control: a stripped value the inner run did NOT re-create is consumed
     from the write list and pcb_data as before;
  5. the same for a re-created via.

    python3 tests/test_1516_consume_inner_strips_recreated.py
"""
import contextlib
import io
import os
import sys
from types import SimpleNamespace as NS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import Segment, Via  # noqa: E402
from plane_write_reconcile import consume_inner_strips  # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    print(("  ok   " if cond else "  FAIL ") + name + (f"  {detail}" if detail else ''))
    if not cond:
        FAILS.append(name)


def seg(a, b, net=5):
    return Segment(start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1],
                   width=0.127, layer='F.Cu', net_id=net)


def sd(s):
    return {'start': (s.start_x, s.start_y), 'end': (s.end_x, s.end_y),
            'width': s.width, 'layer': s.layer, 'net_id': s.net_id}


def via(x, y, net=5):
    return Via(x=x, y=y, size=0.45, drill=0.2, layers=['F.Cu', 'B.Cu'],
               net_id=net)


def vd(v):
    return {'x': v.x, 'y': v.y, 'size': v.size, 'drill': v.drill,
            'layers': v.layers, 'net_id': v.net_id}


def ends(items):
    return sorted((x['start'], x['end']) if isinstance(x, dict)
                  else ((x.start_x, x.start_y), (x.end_x, x.end_y))
                  for x in items)


def run(rdata, write_segs, write_vias, pcb):
    fs, fv = [], []
    with contextlib.redirect_stdout(io.StringIO()):
        consume_inner_strips(rdata, write_segs, write_vias, pcb, fs, fv, 't')
    return fs, fv


A, B, C, E = (0, 0), (0.05, 0), (0.10, 0), (0.15, 0.05)


def main():
    print('1. the issue: merge_collinear re-creates an unchanged piece')
    orig = [seg(A, B), seg(B, C), seg(C, E)]
    merged, same = seg(A, C), seg(C, E)
    rdata = {'results': [{'new_segments': [merged, same]}],
             'segments_to_remove': orig}
    write = [sd(merged), sd(same)]
    pcb = NS(segments=[merged, same], vias=[])
    fs, _fv = run(rdata, write, [], pcb)
    check('the write list keeps both live pieces',
          ends(write) == ends([merged, same]), f'{ends(write)}')
    check('pcb_data keeps the inner run\'s re-created object',
          any(s is same for s in pcb.segments) and len(pcb.segments) == 2)
    check('every stripped original still reaches the file strip channel',
          fs == orig)

    print('2. a stale emission of the stripped original ships once, not twice')
    stale = sd(orig[2])            # the tap emitted C-E before the sub-run
    write = [sd(orig[0]), sd(orig[1]), stale, sd(merged), sd(same)]
    pcb = NS(segments=[merged, same], vias=[])
    run(rdata, write, [], pcb)
    check('the write list holds C-E exactly once',
          ends(write) == ends([merged, same]), f'{ends(write)}')
    check('the stale emission is the copy dropped',
          not any(d is stale for d in write))

    print('3. a pcb_data duplicate of a re-created value')
    dup = seg(C, E)                # a partial restore's copy, not the inner's
    pcb = NS(segments=[merged, dup, same], vias=[])
    write = [sd(merged), sd(same)]
    run(rdata, write, [], pcb)
    check('pcb_data keeps the inner run\'s object and drops the duplicate',
          any(s is same for s in pcb.segments)
          and not any(s is dup for s in pcb.segments)
          and len(pcb.segments) == 2)

    print('4. control: a stripped value the inner run did not re-create')
    gone = seg(C, E)
    rdata4 = {'results': [{'new_segments': [merged]}],
              'segments_to_remove': [seg(A, B), seg(B, C), gone]}
    write = [sd(merged), sd(gone)]
    pcb = NS(segments=[merged, seg(C, E)], vias=[])
    run(rdata4, write, [], pcb)
    check('the write list drops it', ends(write) == ends([merged]),
          f'{ends(write)}')
    check('pcb_data drops its remaining copies', ends(pcb.segments) == ends([merged]))

    print('5. a re-created via')
    v_old, v_new, v_gone = via(1, 1), via(1, 1), via(2, 2)
    rdata5 = {'results': [{'new_segments': [], 'new_vias': [v_new]}],
              'vias_to_remove': [v_old, v_gone]}
    wv = [vd(v_old), vd(v_gone), vd(v_new)]
    pcb = NS(segments=[], vias=[v_new, via(2, 2)])
    _fs, fv = run(rdata5, [], wv, pcb)
    check('the re-created via ships once, the removed one not at all',
          [(d['x'], d['y']) for d in wv] == [(1, 1)], f'{wv}')
    check('pcb_data keeps the re-created via object only',
          len(pcb.vias) == 1 and pcb.vias[0] is v_new)
    check('both stripped vias reach the file strip channel', fv == [v_old, v_gone])

    if FAILS:
        print(f'\nFAILED ({len(FAILS)})')
        sys.exit(1)
    print('\nPASS')


if __name__ == '__main__':
    main()

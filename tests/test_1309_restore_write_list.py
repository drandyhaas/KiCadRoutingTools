#!/usr/bin/env python3
"""#1309: a net a plane tap rips and then restores ships the copper it had.

In route.py's in-run GND finalize, StickHub's +3.3V was reconnected in place
(its copper then lives only in the repair's write lists, its input copper
stripped), ripped again for D3.1's tap, and restored when that tap failed.
The rip purged its write-list entries; the restore put the pieces back into
pcb_data alone, so a routed 17-pad rail shipped with no copper, and
pad_pairs_open (a snapshot taken before the final reconciliation) never
named it.

Checks:
  1. `_return_purged_copper` puts back exactly the purged entries whose
     pieces were restored, matched by geometry (either segment direction),
     and nothing for a piece that stayed off the board.
  2. The re-grade carries each broken net's pad-pair tally, and the merged
     summary's pad_pairs_open names a net no summary listed, its
     pad_pairs_connected counting the deficit.

    python3 tests/test_1309_restore_write_list.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

from kicad_parser import Segment, Via                          # noqa: E402
from repair_planes import _return_purged_copper                # noqa: E402
from route_summary import merge_summaries, regrade_record      # noqa: E402


def _seg(x0, y0, x1, y1, net=4, layer='F.Cu'):
    return Segment(start_x=x0, start_y=y0, end_x=x1, end_y=y1, width=0.2,
                   layer=layer, net_id=net)


def _sd(s):
    return {'start': (s.start_x, s.start_y), 'end': (s.end_x, s.end_y),
            'width': s.width, 'layer': s.layer, 'net_id': s.net_id}


def test_restored_pieces_return_to_the_write_list():
    a, b, c = _seg(0, 0, 1, 0), _seg(1, 0, 1, 1), _seg(1, 1, 2, 1, layer='B.Cu')
    v = Via(x=1.0, y=1.0, size=0.45, drill=0.25, layers=['F.Cu', 'B.Cu'],
            net_id=4)
    vd = {'x': 1.0, 'y': 1.0, 'size': 0.45, 'drill': 0.25,
          'layers': ['F.Cu', 'B.Cu'], 'net_id': 4}
    # b's entry was written end-first: the match must not care.
    bd = dict(_sd(b), start=(1, 1), end=(1, 0))
    purged = ([_sd(a), bd, _sd(c)], [vd])
    write_lists = ([{'net_id': 9}], [])
    # c stayed off the board (it collided): its entry must not come back.
    _return_purged_copper(write_lists, purged, [a, b], [v])
    segs, vias = write_lists
    assert [d.get('net_id') for d in segs] == [9, 4, 4], segs
    assert segs[1:] == [_sd(a), bd] and vias == [vd], (segs, vias)
    _return_purged_copper(write_lists, None, [a], [])   # nothing purged
    assert len(write_lists[0]) == 3
    print("  PASS: restored pieces return to the write list, the rest do not")


def _summary():
    return {'scope': 'run', 'successful': 2, 'failed': 0,
            'routed_single': ['+3.3V', 'X'], 'failed_single': [],
            'open_single': [], 'failed_multipoint': [],
            'multipoint_pads_total': 19, 'multipoint_pads_connected': 19,
            'pad_pairs_total': 210, 'pad_pairs_connected': 210,
            'total_iterations': 10, 'total_vias': 1, 'total_time': 1.0}


def test_pad_pairs_open_names_a_net_broken_after_the_summaries():
    sums = [_summary()]
    grades = {'+3.3V': {'pads': 17, 'broken': True, 'copper': False,
                        'pad_pairs': [16, 0],
                        'failed_pads': [{'x': 0, 'y': 0, 'component_ref': 'U1',
                                         'pad_number': '?'}] * 16},
              'X': {'pads': 2, 'broken': False, 'copper': True,
                    'pad_pairs': [1, 1], 'failed_pads': []}}
    rg = regrade_record(sums, grades, ['+3.3V', 'X'], board='file')
    assert rg['pad_pairs'] == {'+3.3V': {'total': 16, 'connected': 0,
                                         'copper': False}}, rg['pad_pairs']
    m = merge_summaries(sums, regrade=rg)
    assert m['pad_pairs_open'] == [
        {'net': '+3.3V', 'pairs_connected': 0, 'pairs_total': 16,
         'outcome': 'open', 'open_subtype': 'unrouted'}], m['pad_pairs_open']
    assert m['pad_pairs_connected'] == 210 - 16, m['pad_pairs_connected']
    print("  PASS: pad_pairs_open names the rail the summaries never saw")


def main():
    fns = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for fn in fns:
        fn()
    print(f"{len(fns)} test(s) passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())

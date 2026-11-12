#!/usr/bin/env python3
"""#581 follow-up: QFN underpad escapes honor same-net SMD pad clearance.

generate_qfn_fanout(..., escape_method='underpad', same_net_pad_clearance=0.1)
disables via-in-pad, but before this fix the underpad escape still dropped each
via just past its own pad's edge: 0.0000 mm from same-net SMD copper. An active
same-net pad clearance must keep every escape via at least that gap away from
same-net SMD pad copper; a zero gap keeps the old "just past the pad edge"
placement.

Run:
    python3 tests/test_qfn_underpad_same_net_pad_clearance.py
"""
import contextlib
import io
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

from kicad_parser import BoardInfo, Footprint, Net, Pad, PCBData  # noqa: E402
from qfn_fanout import generate_qfn_fanout                       # noqa: E402

VIA_SIZE = 0.4
VIA_DRILL = 0.2
GRID = 0.05
GAP = 0.1


def _pad(num, x, y, sx, sy):
    return Pad(component_ref='U1', pad_number=str(num),
               global_x=x, global_y=y, local_x=x, local_y=y,
               size_x=sx, size_y=sy, shape='rect', layers=['F.Cu'],
               net_id=1, net_name='N1', pad_type='smd')


def _fixture():
    # Four same-net perimeter pads make a tiny QFN-like footprint. The left and
    # right pads have their long axis pointed outward, so the first legacy
    # underpad candidate is exactly via_size/2 past the pad edge.
    pads = [
        _pad(1, -1.5, 0.0, 1.0, 0.2),
        _pad(2, 1.5, 0.0, 1.0, 0.2),
        _pad(3, 0.0, -1.5, 0.2, 1.0),
        _pad(4, 0.0, 1.5, 0.2, 1.0),
    ]
    fp = Footprint(reference='U1', footprint_name='Synthetic_QFN',
                   x=0.0, y=0.0, rotation=0.0, layer='F.Cu', pads=pads)
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                   copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=(-10.0, -10.0, 10.0, 10.0))
    net = Net(1, 'N1', pads=list(pads))
    pcb = PCBData(board_info=bi, nets={1: net}, footprints={'U1': fp},
                  vias=[], segments=[], pads_by_net={1: pads})
    return fp, pcb


def _run(snpc, allow_via_in_pad=False):
    fp, pcb = _fixture()
    with contextlib.redirect_stdout(io.StringIO()):
        return generate_qfn_fanout(
            fp, pcb, layer='B.Cu', track_width=0.1, clearance=0.0,
            grid_step=GRID, escape_method='underpad',
            via_size=VIA_SIZE, via_drill=VIA_DRILL,
            allow_via_in_pad=allow_via_in_pad,
            same_net_pad_clearance=snpc)


def _rect_clearance(via, pad):
    dx = max(abs(via['x'] - pad.global_x) - pad.size_x / 2.0, 0.0)
    dy = max(abs(via['y'] - pad.global_y) - pad.size_y / 2.0, 0.0)
    return math.hypot(dx, dy) - via['size'] / 2.0


def _min_same_net_smd_edge_gap(vias):
    _fp, pcb = _fixture()
    gaps = []
    for via in vias:
        for pad in pcb.pads_by_net[via['net_id']]:
            if pad.drill and pad.drill > 0:
                continue
            gaps.append(_rect_clearance(via, pad))
    return min(gaps) if gaps else float('inf')


def main():
    fails = []

    def check(name, cond, detail=''):
        print(('  PASS: ' if cond else '  FAIL: ') + name
              + (f' -- {detail}' if detail else ''))
        if not cond:
            fails.append(name)

    print('QFN underpad same-net pad clearance')

    tracks0, vias0, dropped0 = _run(0.0)
    gap0 = _min_same_net_smd_edge_gap(vias0)
    check('legacy zero clearance places vias just past same-net pad edges',
          vias0 and abs(gap0) <= 1e-6,
          f'min edge gap {gap0:.4f}, vias={len(vias0)}, dropped={dropped0}')

    tracks1, vias1, dropped1 = _run(GAP)
    gap1 = _min_same_net_smd_edge_gap(vias1)
    check('active clearance keeps every emitted via at least the requested gap away',
          (not vias1) or gap1 >= GAP - 1e-6,
          f'min edge gap {gap1:.4f}, vias={len(vias1)}, dropped={dropped1}')
    check('active clearance either escapes farther out or reports pins unescaped',
          bool(dropped1) or gap1 > gap0 + 0.05,
          f'legacy {gap0:.4f}, active {gap1:.4f}, dropped={dropped1}')

    _tracks2, vias2, dropped2 = _run(GAP, allow_via_in_pad=True)
    gap2 = _min_same_net_smd_edge_gap(vias2)
    check('allow_via_in_pad=True still honors an active clearance',
          (not vias2) or gap2 >= GAP - 1e-6,
          f'min edge gap {gap2:.4f}, vias={len(vias2)}, dropped={dropped2}')

    if fails:
        print('FAIL: ' + '; '.join(fails))
        return 1
    print('PASS: QFN underpad same-net pad clearance is enforced')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
"""
Regression test for issue #315 (stub layer swap relayers a BGA fanout stub
onto F.Cu straight through foreign pad copper).

On zynq_ad9364 the signal retry's single-ended swap pair moved DDR3_DQ0's
In1.Cu fanout stub onto F.Cu. The stub legally ran under the U1 BGA pad row on
the inner layer (SMD pads only block their own layer), but the identical XY
path on F.Cu passes dead through the copper of U1.A3 (VCC_1V5) and U1.B3
(DDR3_DQ1). `validate_single_swap`'s foreign-pad check (issue #123) only
looked at `get_stub_info().segments`, which for a source stub is just the
zero-length pad nub -- the long under-pad-row segment was never tested. The
foreign-TRACK check (issue #238) already expanded to the whole connected
source-side trace via `connected_stub_segments_on_layer`; the fix applies the
same expansion to the pad check, in both `validate_single_swap` (single-ended)
and `validate_swap` (diff pair).

This test builds the zynq geometry synthetically:
  - net STUB has a via-in-pad at (0,0) and an In1.Cu fanout stub running
    +x through a row of foreign 0.35mm F.Cu BGA pads at 0.8mm pitch
  - swapping that stub to F.Cu must be REJECTED (it would overlap the pads)
  - an otherwise identical stub with no foreign pads along it must still be
    ACCEPTED (no over-rejection)
  - same pair of assertions for the diff-pair validator `validate_swap`

Run:
    python3 tests/test_stub_swap_foreign_pad.py
"""

import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

from kicad_parser import PCBData, Segment, Via, Pad, Net
from routing_config import GridRouteConfig
from stub_layer_switching import get_stub_info, validate_single_swap, validate_swap

STUB_NET, STUB_N_NET, FOREIGN_NET = 1, 2, 99


def _cfg():
    c = GridRouteConfig()
    c.layers = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
    c.track_width = 0.1
    c.clearance = 0.15
    c.grid_step = 0.05
    c.via_size = 0.45
    c.via_drill = 0.3
    return c


def _pad(ref, num, x, y, net_id, net_name):
    return Pad(component_ref=ref, pad_number=num, global_x=x, global_y=y,
               local_x=0.0, local_y=0.0, size_x=0.35, size_y=0.35,
               shape='circle', layers=['F.Cu'], net_id=net_id,
               net_name=net_name)


def _seg(x1, y1, x2, y2, net_id, layer='In1.Cu'):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                   width=0.1, layer=layer, net_id=net_id)


def _pcb(y0=0.0, with_foreign_pads=True, n_net=False):
    """Net STUB: via-in-pad at (0,y0), In1.Cu stub to (2.1,y0) then a short
    jog -- mirrors the zynq DDR3_DQ0 fanout shape. Foreign F.Cu BGA pads sit
    ON the stub path at 0.8mm pitch (like U1.B3/A3)."""
    segments = [
        _seg(0.0, y0, 2.1, y0, STUB_NET),          # run under the pad row
        _seg(0.0, y0, 0.0, y0, STUB_NET),          # zero-length pad nub
        _seg(2.1, y0, 2.09, y0 + 0.02, STUB_NET),  # tail jog
    ]
    vias = [Via(x=0.0, y=y0, size=0.45, drill=0.3,
                layers=['F.Cu', 'B.Cu'], net_id=STUB_NET)]
    pads = {
        STUB_NET: [_pad('U1', 'C3', 0.0, y0, STUB_NET, 'STUB'),
                   _pad('U2', 'E3', 1.4, -23.2, STUB_NET, 'STUB')],
    }
    if n_net:
        segments += [
            _seg(0.0, y0 - 0.8, 2.1, y0 - 0.8, STUB_N_NET),
            _seg(0.0, y0 - 0.8, 0.0, y0 - 0.8, STUB_N_NET),
        ]
        vias.append(Via(x=0.0, y=y0 - 0.8, size=0.45, drill=0.3,
                        layers=['F.Cu', 'B.Cu'], net_id=STUB_N_NET))
        pads[STUB_N_NET] = [_pad('U1', 'C2', 0.0, y0 - 0.8, STUB_N_NET, 'STUB_N'),
                            _pad('U2', 'F3', 0.6, -23.2, STUB_N_NET, 'STUB_N')]
    if with_foreign_pads:
        pads[FOREIGN_NET] = [_pad('U1', 'B3', 0.8, y0, FOREIGN_NET, 'VCC_1V5'),
                             _pad('U1', 'A3', 1.6, y0, FOREIGN_NET, 'VCC_1V5')]
    return PCBData(
        footprints={}, segments=segments, vias=vias,
        nets={STUB_NET: Net(STUB_NET, 'STUB'),
              STUB_N_NET: Net(STUB_N_NET, 'STUB_N'),
              FOREIGN_NET: Net(FOREIGN_NET, 'VCC_1V5')},
        board_info=None, pads_by_net=pads,
    )


def main():
    cfg = _cfg()
    fails = []

    def check(name, cond):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}")
        if not cond:
            fails.append(name)

    # -- single-ended validator ------------------------------------------
    pcb = _pcb()
    stub = get_stub_info(pcb, STUB_NET, 0.0, 0.0, 'In1.Cu')
    ok, reason = validate_single_swap(stub, 'F.Cu', {}, pcb, cfg)
    check("single-ended swap through foreign pad row rejected", not ok)
    check("rejection names the grazed pad", 'A3' in reason or 'B3' in reason)

    pcb_clear = _pcb(with_foreign_pads=False)
    stub = get_stub_info(pcb_clear, STUB_NET, 0.0, 0.0, 'In1.Cu')
    ok, reason = validate_single_swap(stub, 'F.Cu', {}, pcb_clear, cfg)
    check(f"single-ended swap with clear path accepted ({reason or 'ok'})", ok)

    # -- diff-pair validator ---------------------------------------------
    pcb = _pcb(n_net=True)
    stub_p = get_stub_info(pcb, STUB_NET, 0.0, 0.0, 'In1.Cu')
    stub_n = get_stub_info(pcb, STUB_N_NET, 0.0, -0.8, 'In1.Cu')
    ok, reason = validate_swap(stub_p, stub_n, 'F.Cu', {}, pcb, cfg)
    check("diff-pair swap through foreign pad row rejected", not ok)

    pcb_clear = _pcb(n_net=True, with_foreign_pads=False)
    stub_p = get_stub_info(pcb_clear, STUB_NET, 0.0, 0.0, 'In1.Cu')
    stub_n = get_stub_info(pcb_clear, STUB_N_NET, 0.0, -0.8, 'In1.Cu')
    ok, reason = validate_swap(stub_p, stub_n, 'F.Cu', {}, pcb_clear, cfg)
    check(f"diff-pair swap with clear path accepted ({reason or 'ok'})", ok)

    if fails:
        print(f"\nFAIL ({len(fails)}): " + "; ".join(fails))
        return 1
    print("\nAll checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())

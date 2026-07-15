#!/usr/bin/env python3
"""
Regression test for issue #328: NPTH pads with a net must not be treated as
plated copper barrels.

Several sites gated "this pad connects every copper layer" on `pad.drill > 0`
alone. An NPTH pad (`pad_type == 'np_thru_hole'`) has a hole but NO copper
barrel, so a net-tied mounting hole (KiCad allows it) would be treated as an
all-layer connection point that doesn't exist -- phantom plane connectivity,
launch layers offered on nothing. `kicad_parser.pad_is_plated_through` is now
the shared gate.

Run:
    python3 tests/test_npth_not_plated.py
"""
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

from kicad_parser import PCBData, Net, pad_is_plated_through
from routing_config import GridRouteConfig
from synth import make_pad  # #382 E7: canonical builder


def _pad(drill, pad_type, layers=('*.Cu', '*.Mask'), x=5.0, y=5.0, net=7):
    return make_pad(net_id=net, x=x, y=y, ref='H1', num='1', net_name='GND',
                    size_x=3.0, size_y=3.0, shape='circle', layers=layers,
                    drill=drill, pad_type=pad_type)


def main():
    fails = []

    def check(name, cond):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}")
        if not cond:
            fails.append(name)

    # -- the helper itself -------------------------------------------------
    check("PTH pin is plated", pad_is_plated_through(_pad(0.8, 'thru_hole')))
    check("NPTH hole is NOT plated", not pad_is_plated_through(_pad(3.2, 'np_thru_hole')))
    check("SMD pad is not plated", not pad_is_plated_through(_pad(0.0, 'smd')))
    check("unknown pad_type with drill counts as plated (conservative)",
          pad_is_plated_through(_pad(0.8, '')))

    # -- _endpoint_launch_layer_indices: NPTH must not offer all layers -----
    from diff_pair_routing import _endpoint_launch_layer_indices
    config = GridRouteConfig(layers=['F.Cu', 'In1.Cu', 'B.Cu'])

    def _mini_pcb(pad):
        return PCBData(footprints={}, segments=[], vias=[],
                       nets={7: Net(7, 'GND')}, board_info=None,
                       pads_by_net={7: [pad]})

    pcb = _mini_pcb(_pad(3.2, 'np_thru_hole'))
    spanned = _endpoint_launch_layer_indices(pcb, 7, 5.0, 5.0, config)
    check("NPTH endpoint does not span all layers", spanned == set())

    pcb = _mini_pcb(_pad(0.8, 'thru_hole'))
    spanned = _endpoint_launch_layer_indices(pcb, 7, 5.0, 5.0, config)
    check("PTH endpoint spans all layers", spanned == {0, 1, 2})

    # -- get_stub_info: NPTH at the pad is NOT a pad via --------------------
    from kicad_parser import Segment
    from stub_layer_switching import get_stub_info, needs_pad_via_for_switch
    for ptype, expect_via_needed, label in (
            ('np_thru_hole', True, "NPTH pad does not stand in for a pad via"),
            ('thru_hole', False, "PTH pad counts as the pad via (#289)")):
        pad = _pad(1.0 if ptype == 'thru_hole' else 3.2, ptype, x=0.0, y=0.0)
        pcb = PCBData(footprints={}, segments=[
            Segment(start_x=0.0, start_y=0.0, end_x=2.0, end_y=0.0,
                    width=0.2, layer='F.Cu', net_id=7)],
            vias=[], nets={7: Net(7, 'GND')}, board_info=None,
            pads_by_net={7: [pad]})
        stub = get_stub_info(pcb, 7, 2.0, 0.0, 'F.Cu')
        check(label, stub is not None
              and needs_pad_via_for_switch(stub) == expect_via_needed)

    if fails:
        print(f"\nFAIL ({len(fails)})")
        return 1
    print("\nAll checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())

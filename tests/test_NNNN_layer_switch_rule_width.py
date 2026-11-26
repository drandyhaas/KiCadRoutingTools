#!/usr/bin/env python3
"""
A stub layer switch takes the DESTINATION layer's rule width.

Per-layer width rules (a .kicad_dru track_width rule scoped by layer, a
per-layer netclass or impedance width) give one net different widths on
different layers, e.g. min 0.18 mm on the outer layers and max 0.102 mm on the
inner ones. The layer-switch passes used to move a segment by relabelling its
layer only, so a 0.102 mm inner stub landed on B.Cu at 0.102 mm (breaking the
0.18 minimum) and a 0.18 mm outer stub landed inside at 0.18 mm (breaking the
0.102 maximum). The file writer replays the moves as text edits, so it has to
rewrite the width too.

Invariants gated here:
  1. apply_stub_layer_switch re-widths a moved segment to the destination
     layer's resolved width and records old/new width in the mod.
  2. revert_stub_layer_switch restores the old layer AND the old width.
  3. A net with the same width on both layers keeps the segment's own width
     (a neck-down is not widened).
  4. modify_segment_layers writes the new width into the file text, and a
     chained move (F -> In1 -> B) lands at the final layer's width.
  5. The keep-out/foreign-pad legality checks size the stub at the destination
     width (relayer_width).

Run:
    python3 tests/test_NNNN_layer_switch_rule_width.py
"""

import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_tools'))

from kicad_parser import PCBData, Segment, Pad, Net
from routing_config import GridRouteConfig
from stub_layer_switching import (StubInfo, apply_stub_layer_switch,
                                  revert_stub_layer_switch, relayer_width)
from kicad_writer import modify_segment_layers

LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']


def _seg(x1, y1, x2, y2, net_id=1, layer='F.Cu', width=0.18):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                   width=width, layer=layer, net_id=net_id)


def _pad(x, y, net_id=1):
    return Pad(pad_number='1', net_id=net_id, net_name=f'N{net_id}',
               global_x=x, global_y=y, local_x=0, local_y=0,
               size_x=1.6, size_y=1.6, shape='circle',
               layers=['*.Cu'], drill=0.8, pad_type='thru_hole',
               component_ref='A1')


def _pcb(segments, pads):
    return PCBData(board_info=None, nets={1: Net(1, 'ADDR0'), 2: Net(2, 'SIG')},
                   footprints={}, vias=[], segments=segments, pads_by_net=pads)


def _config():
    c = GridRouteConfig(layers=list(LAYERS), track_width=0.15)
    # net 1: an impedance class, 0.18 outer / 0.102 inner (per-layer rule)
    c.net_layer_widths = {1: {'F.Cu': 0.18, 'B.Cu': 0.18,
                              'In1.Cu': 0.102, 'In2.Cu': 0.102}}
    return c


FILE = """(kicad_pcb
\t(segment
\t\t(start 2 0)
\t\t(end 6 0)
\t\t(width 0.18)
\t\t(layer "F.Cu")
\t\t(net "ADDR0")
\t\t(uuid "a")
\t)
)
"""


def main():
    fails = []

    def check(name, cond):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}")
        if not cond:
            fails.append(name)

    config = _config()

    print("1: outer stub moved inside takes the inner width")
    track = _seg(2, 0, 6, 0)
    pcb = _pcb([track], {1: [_pad(2, 0)]})
    stub = StubInfo(net_id=1, x=6, y=0, layer='F.Cu', segments=[track],
                    pad_x=2, pad_y=0, has_pad_via=True)
    vias, mods = apply_stub_layer_switch(pcb, stub, 'In1.Cu', config, debug=False)
    check("moved to In1.Cu", track.layer == 'In1.Cu')
    check("width 0.102 on In1.Cu", abs(track.width - 0.102) < 1e-9)
    m = [x for x in mods if 'start' in x]
    check("mod records old/new width",
          m and m[0].get('old_width') == 0.18 and m[0].get('new_width') == 0.102)

    print("2: revert restores layer and width")
    revert_stub_layer_switch(pcb, mods, vias)
    check("back on F.Cu", track.layer == 'F.Cu')
    check("width back to 0.18", abs(track.width - 0.18) < 1e-9)

    print("3: same width on both layers keeps the segment's own width")
    neck = _seg(0, 5, 3, 5, net_id=2, width=0.1)
    check("neck-down not widened", relayer_width(neck, 'B.Cu', config) == 0.1)
    pcb3 = _pcb([neck], {2: [_pad(0, 5, net_id=2)]})
    stub3 = StubInfo(net_id=2, x=3, y=5, layer='F.Cu', segments=[neck],
                     pad_x=0, pad_y=5, has_pad_via=True)
    _, mods3 = apply_stub_layer_switch(pcb3, stub3, 'B.Cu', config, debug=False)
    check("neck keeps 0.1 on B.Cu", neck.width == 0.1 and neck.layer == 'B.Cu')
    check("no width in its mod", all('new_width' not in x for x in mods3))

    print("4: writer rewrites the width; a chain lands at the final width")
    out, n = modify_segment_layers(FILE, [m[0]])
    check("file reads In1.Cu at 0.102",
          '(layer "In1.Cu")' in out and '(width 0.102)' in out and n == 1)
    chain = [dict(m[0]),
             {'start': (2, 0), 'end': (6, 0), 'net_id': 1, 'net_name': 'ADDR0',
              'old_layer': 'In1.Cu', 'new_layer': 'B.Cu',
              'old_width': 0.102, 'new_width': 0.18}]
    out2, _ = modify_segment_layers(FILE, chain)
    check("chain F -> In1 -> B reads B.Cu at 0.18",
          '(layer "B.Cu")' in out2 and '(width 0.18)' in out2)
    plain = {'start': (2, 0), 'end': (6, 0), 'net_id': 1, 'net_name': 'ADDR0',
             'old_layer': 'F.Cu', 'new_layer': 'B.Cu'}
    out3, _ = modify_segment_layers(FILE, [plain])
    check("a mod without width keeps the file width", '(width 0.18)' in out3)

    print("5: legality checks size the stub at the destination width")
    probe = _seg(2, 0, 6, 0, width=0.18)
    check("relayer_width F.Cu -> In2.Cu = 0.102",
          abs(relayer_width(probe, 'In2.Cu', config) - 0.102) < 1e-9)

    print()
    if fails:
        print(f"FAILED: {len(fails)} check(s): {fails}")
        return 1
    print("All layer-switch rule-width checks passed.")
    return 0


if __name__ == '__main__':
    sys.exit(main())

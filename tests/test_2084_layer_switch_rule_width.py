#!/usr/bin/env python3
"""
A stub layer switch keeps the segment inside the DESTINATION layer's rule width.

A .kicad_dru track_width rule scoped by layer gives one net a different width
window per layer, e.g. 0.18 mm on the outer layers and 0.102 mm on the inner
ones. The layer-switch passes used to move a segment by relabelling its layer
only, so a 0.102 mm inner stub landed on B.Cu at 0.102 mm (breaking the 0.18
minimum) and a 0.18 mm outer stub landed inside at 0.18 mm (breaking the 0.102
maximum). The file writer replays the moves as text edits, so it has to
rewrite the width too.

Invariants gated here, on a synthetic four-layer board and its .kicad_dru:
  1. apply_stub_layer_switch clamps a moved segment into the destination
     layer's rule window and records old/new width in the mod.
  2. revert_stub_layer_switch restores the old layer AND the old width.
  3. A net no rule binds keeps the segment's own width (a neck-down is not
     widened), even when it has per-layer default widths.
  4. modify_segment_layers writes the new width into the file text, and a
     chained move (F -> In1 -> B) lands at the final layer's width.
  5. The keep-out/foreign-pad legality checks size the stub at the destination
     width (relayer_width).
  6. The rule bounds follow the live config: switching --escalation to fab or
     installing other rules on the same config changes the answer (no stale
     per-config cache).

Run:
    python3 tests/test_2084_layer_switch_rule_width.py
"""

import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_tools'))
sys.path.insert(0, TESTS_DIR)

from synth import RoutingBoard                                  # noqa: E402
from kicad_parser import PCBData, Segment, Pad, parse_kicad_pcb  # noqa: E402
from design_rules import DesignRules                            # noqa: E402
from routing_config import GridRouteConfig                      # noqa: E402
from fab_tiers import get_escalation_policy, set_escalation_policy  # noqa: E402
from stub_layer_switching import (StubInfo, apply_stub_layer_switch,  # noqa: E402
                                  revert_stub_layer_switch)
try:
    from stub_layer_switching import relayer_width
except ImportError:  # before the fix: no destination-width helper
    relayer_width = None
from kicad_writer import modify_segment_layers                  # noqa: E402

LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
DRU = """(version 1)
(rule "addr_outer" (layer outer) (condition "A.NetName == 'ADDR*'")
    (constraint track_width (min {outer}mm) (opt {outer}mm) (max {outer}mm)))
(rule "addr_inner" (layer inner) (condition "A.NetName == 'ADDR*'")
    (constraint track_width (min 0.102mm) (opt 0.102mm) (max 0.102mm)))
"""


def _rules(outer):
    """(pcb, DesignRules) for a board with ADDR0 / SIG and the DRU above."""
    with tempfile.TemporaryDirectory() as td:
        b = RoutingBoard(w=20, h=10, layers=tuple(LAYERS))
        b.pad('A1', 2, 5, 'ADDR0')
        b.pad('A2', 18, 5, 'ADDR0')
        b.pad('B1', 2, 8, 'SIG')
        b.pad('B2', 18, 8, 'SIG')
        path = os.path.join(td, 'b.kicad_pcb')
        with open(path, 'w', encoding='utf-8') as f:
            f.write(b.text())
        with open(os.path.join(td, 'b.kicad_dru'), 'w', encoding='utf-8') as f:
            f.write(DRU.format(outer=outer))
        pcb = parse_kicad_pcb(path)
        return pcb, DesignRules.from_project(pcb, path)


def _seg(x1, y1, x2, y2, net_id, layer='F.Cu', width=0.18):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                   width=width, layer=layer, net_id=net_id)


def _pad(x, y, net_id):
    return Pad(pad_number='1', net_id=net_id, net_name=f'N{net_id}',
               global_x=x, global_y=y, local_x=0, local_y=0,
               size_x=1.6, size_y=1.6, shape='circle',
               layers=['*.Cu'], drill=0.8, pad_type='thru_hole',
               component_ref='A1')


def _pcb(nets, segments, pads):
    return PCBData(board_info=None, nets=nets, footprints={}, vias=[],
                   segments=segments, pads_by_net=pads)


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

    board, rules = _rules('0.18')
    nid = {n.name: i for i, n in board.nets.items()}
    addr, sig = nid['ADDR0'], nid['SIG']
    nets = board.nets

    prev = get_escalation_policy()
    try:
        set_escalation_policy('board')
        config = GridRouteConfig(layers=list(LAYERS), track_width=0.15, rules=rules)

        print("1: outer stub moved inside is held to the inner maximum")
        track = _seg(2, 0, 6, 0, addr)
        pcb = _pcb(nets, [track], {addr: [_pad(2, 0, addr)]})
        stub = StubInfo(net_id=addr, x=6, y=0, layer='F.Cu', segments=[track],
                        pad_x=2, pad_y=0, has_pad_via=True)
        vias, mods = apply_stub_layer_switch(pcb, stub, 'In1.Cu', config, debug=False)
        check("moved to In1.Cu", track.layer == 'In1.Cu')
        check("width 0.102 on In1.Cu", abs(track.width - 0.102) < 1e-9)
        m = [x for x in mods if 'start' in x]
        check("mod records old/new width",
              m and m[0].get('old_width') == 0.18
              and abs(m[0].get('new_width', 0) - 0.102) < 1e-9)

        print("2: revert restores layer and width")
        revert_stub_layer_switch(pcb, mods, vias)
        check("back on F.Cu", track.layer == 'F.Cu')
        check("width back to 0.18", abs(track.width - 0.18) < 1e-9)

        print("3: a net no rule binds keeps the segment's own width")
        neck = _seg(0, 5, 3, 5, sig, width=0.1)
        check("neck-down not widened", relayer_width is not None
              and relayer_width(neck, 'B.Cu', config) == 0.1)
        pcb3 = _pcb(nets, [neck], {sig: [_pad(0, 5, sig)]})
        stub3 = StubInfo(net_id=sig, x=3, y=5, layer='F.Cu', segments=[neck],
                         pad_x=0, pad_y=5, has_pad_via=True)
        _, mods3 = apply_stub_layer_switch(pcb3, stub3, 'B.Cu', config, debug=False)
        check("neck keeps 0.1 on B.Cu", neck.width == 0.1 and neck.layer == 'B.Cu')
        check("no width in its mod", all('new_width' not in x for x in mods3))
        layered = GridRouteConfig(layers=list(LAYERS), track_width=0.15, rules=rules)
        layered.net_layer_widths = {sig: {'F.Cu': 0.2, 'B.Cu': 0.3,
                                          'In1.Cu': 0.2, 'In2.Cu': 0.2}}
        fan = _seg(0, 5, 3, 5, sig, width=0.12)
        check("per-layer default width without a rule: not resized",
              relayer_width is not None
              and relayer_width(fan, 'B.Cu', layered) == 0.12)

        print("4: writer rewrites the width; a chain lands at the final width")
        out, n = modify_segment_layers(FILE, [m[0]])
        check("file reads In1.Cu at 0.102",
              '(layer "In1.Cu")' in out and '(width 0.102)' in out and n == 1)
        chain = [dict(m[0]),
                 {'start': (2, 0), 'end': (6, 0), 'net_id': addr, 'net_name': 'ADDR0',
                  'old_layer': 'In1.Cu', 'new_layer': 'B.Cu',
                  'old_width': 0.102, 'new_width': 0.18}]
        out2, _ = modify_segment_layers(FILE, chain)
        check("chain F -> In1 -> B reads B.Cu at 0.18",
              '(layer "B.Cu")' in out2 and '(width 0.18)' in out2)
        plain = {'start': (2, 0), 'end': (6, 0), 'net_id': addr, 'net_name': 'ADDR0',
                 'old_layer': 'F.Cu', 'new_layer': 'B.Cu'}
        out3, _ = modify_segment_layers(FILE, [plain])
        check("a mod without width keeps the file width", '(width 0.18)' in out3)

        print("5: legality checks size the stub at the destination width")
        probe = _seg(2, 0, 6, 0, addr, width=0.18)
        check("relayer_width F.Cu -> In2.Cu = 0.102", relayer_width is not None
              and abs(relayer_width(probe, 'In2.Cu', config) - 0.102) < 1e-9)
        thin = _seg(2, 0, 6, 0, addr, layer='In1.Cu', width=0.102)
        check("relayer_width In1.Cu -> B.Cu = 0.18", relayer_width is not None
              and abs(relayer_width(thin, 'B.Cu', config) - 0.18) < 1e-9)

        print("6: the bounds follow the live policy and rules")
        live = GridRouteConfig(layers=list(LAYERS), track_width=0.15, rules=rules)
        check("board: 0.102 on B.Cu lifted to 0.18",
              abs(live.rule_width_bounds(addr, 'B.Cu', 0.102) - 0.18) < 1e-9)
        set_escalation_policy('fab')
        check("fab on the same config: minimum dropped",
              abs(live.rule_width_bounds(addr, 'B.Cu', 0.102) - 0.102) < 1e-9)
        set_escalation_policy('board')
        _, wider = _rules('0.25')
        live.rules = wider
        check("new rules on the same config: 0.25 outer",
              abs(live.rule_width_bounds(addr, 'B.Cu', 0.102) - 0.25) < 1e-9)
    finally:
        set_escalation_policy(*prev)

    print()
    if fails:
        print(f"FAILED: {len(fails)} check(s): {fails}")
        return 1
    print("All layer-switch rule-width checks passed.")
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""
Tests for issue #369 Tier A writer/parser fixes (A1, A4, A6, A7, A8, A9).

Each scenario is the minimal repro from the #368 review:
  A1: copper strip must fire on a name-net board with a pre-2025 header.
  A4: Pad.rotation is the file's ABSOLUTE angle; no footprint double-count
      (slot-drill capsule orientation depends on it).
  A6: a KiCad-10 net-tied copper graphic must NOT be moved to silkscreen.
  A7: locked ARCs parse; modify_segment_layers matches locked segments.
  A8: pad net swap works via fp_text reference (KiCad 6/7) and swaps EVERY
      same-numbered pad, not just the first.
  A9: via removal matches this tool's own %.6f output and respects the net.

Run:
    python3 tests/test_369_writer_parser_fixes.py
"""

import math
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

import tempfile

from kicad_parser import parse_kicad_pcb, pad_drill_capsule, Pad
from kicad_writer import (remove_segments_from_content, remove_vias_from_content,
                          move_copper_graphics_to_silkscreen,
                          modify_segment_layers, swap_pad_nets_in_content)


class _Seg:
    def __init__(self, sx, sy, ex, ey, layer, net_id):
        self.start_x, self.start_y = sx, sy
        self.end_x, self.end_y = ex, ey
        self.layer, self.net_id = layer, net_id


def main():
    fails = []

    def check(name, cond):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}")
        if not cond:
            fails.append(name)

    # -- A1: strip on name-net board with pre-2025 header --------------------
    print("A1: copper strip on name-net board with old version header")
    old_header_board = '''(kicad_pcb
\t(version 20240108)
\t(net 0 "")
\t(net 1 "SIG")
\t(segment
\t\t(start 1.000000 2.000000)
\t\t(end 3.000000 2.000000)
\t\t(width 0.2)
\t\t(layer "F.Cu")
\t\t(net "SIG")
\t\t(uuid "s1")
\t)
\t(via
\t\t(at 5.000000 5.000000)
\t\t(size 0.5)
\t\t(drill 0.3)
\t\t(layers "F.Cu" "B.Cu")
\t\t(net "SIG")
\t\t(uuid "v1")
\t)
)'''
    out, n = remove_segments_from_content(
        old_header_board, [_Seg(1.0, 2.0, 3.0, 2.0, 'F.Cu', 1)], {1: 'SIG'})
    check("segment stripped (was silent no-op)", n == 1 and '(segment' not in out)

    class _V:
        x, y, net_id = 5.0, 5.0, 1
    out, n = remove_vias_from_content(old_header_board, [_V()], {1: 'SIG'})
    check("via stripped (was silent no-op)", n == 1 and '(via' not in out)

    # -- A4: absolute pad rotation, no footprint double-count ---------------
    print("\nA4: Pad.rotation absolute (rotated footprint, slot drill)")
    rot_board = '''(kicad_pcb
\t(version 20241229)
\t(net 0 "")
\t(net 1 "SIG")
\t(footprint "test:CONN"
\t\t(layer "F.Cu")
\t\t(uuid "fp1")
\t\t(at 50 50 90)
\t\t(property "Reference" "J1"
\t\t\t(at 0 0 90)
\t\t)
\t\t(pad "1" thru_hole oval
\t\t\t(at 0 0 90)
\t\t\t(size 1.6 1.0)
\t\t\t(drill oval 1.2 0.6)
\t\t\t(layers "*.Cu" "*.Mask")
\t\t\t(net 1 "SIG")
\t\t\t(uuid "p1")
\t\t)
\t)
)'''
    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False) as f:
        f.write(rot_board)
    pcb = parse_kicad_pcb(f.name)
    pad = pcb.footprints['J1'].pads[0]
    check("rotation is the file's absolute 90 (not 90+90)",
          abs(pad.rotation % 360 - 90.0) < 1e-6)
    (x1, y1), (x2, y2), _d = pad_drill_capsule(pad)
    # pad angle 90 rotates the slot's long axis (local +x) onto board -y/+y:
    # the capsule must extend in Y, not X.
    check("slot drill capsule oriented along Y (was 90° off)",
          abs(x2 - x1) < 1e-6 and abs(y2 - y1) > 0.3)

    # -- A6: v10 net-tied copper graphic stays on copper ---------------------
    print("\nA6: KiCad-10 net-tied copper graphic not silk'd")
    g = '''(kicad_pcb
\t(gr_line
\t\t(start 0 0)
\t\t(end 5 0)
\t\t(stroke (width 0.3) (type solid))
\t\t(layer "F.Cu")
\t\t(net "GND")
\t\t(uuid "g1")
\t)
\t(gr_line
\t\t(start 0 2)
\t\t(end 5 2)
\t\t(stroke (width 0.3) (type solid))
\t\t(layer "F.Cu")
\t\t(uuid "g2")
\t)
)'''
    out = move_copper_graphics_to_silkscreen(g)
    check("net-tied v10 graphic kept on F.Cu",
          '(net "GND")' in out and out.count('(layer "F.Cu")') == 1)
    check("net-less decoration still moved to silk",
          out.count('(layer "F.SilkS")') == 1)

    # -- A7: locked arcs parse; locked segment layer-mod applies -------------
    print("\nA7: locked-token family")
    arc_board = '''(kicad_pcb
\t(version 20240108)
\t(net 0 "")
\t(net 1 "SIG")
\t(arc (start 10 10) (mid 12 12) (end 14 10) (width 0.25) (locked yes) (layer "F.Cu") (net 1) (uuid "a1")
)'''
    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False) as f:
        f.write(arc_board)
    pcb = parse_kicad_pcb(f.name)
    check("locked arc parsed into segments", len(pcb.segments) > 0
          and all(s.net_id == 1 for s in pcb.segments))

    locked_seg_board = '''(kicad_pcb
\t(segment
\t\t(start 1 2)
\t\t(end 3 2)
\t\t(width 0.2)
\t\t(locked yes)
\t\t(layer "F.Cu")
\t\t(net 1)
\t\t(uuid "s1")
\t)
)'''
    out, n = modify_segment_layers(locked_seg_board, [{
        'start': (1.0, 2.0), 'end': (3.0, 2.0), 'net_id': 1,
        'old_layer': 'F.Cu', 'new_layer': 'B.Cu'}])
    check("locked segment layer mod applied (was silent no-op)",
          n == 1 and '(layer "B.Cu")' in out)

    # -- A8: fp_text reference + duplicate pad numbers ------------------------
    print("\nA8: pad net swap on KiCad 6/7 refs and duplicate pad numbers")
    kicad7_board = '''(kicad_pcb
\t(footprint "test:USB"
\t\t(layer "F.Cu")
\t\t(fp_text reference "J1"
\t\t\t(at 0 0)
\t\t)
\t\t(pad "S" smd rect
\t\t\t(at -3 0)
\t\t\t(size 1 1)
\t\t\t(layers "F.Cu")
\t\t\t(net 1 "GND")
\t\t)
\t\t(pad "S" smd rect
\t\t\t(at 3 0)
\t\t\t(size 1 1)
\t\t\t(layers "F.Cu")
\t\t\t(net 1 "GND")
\t\t)
\t)
\t(footprint "test:R"
\t\t(layer "F.Cu")
\t\t(fp_text reference "R1"
\t\t\t(at 0 0)
\t\t)
\t\t(pad "1" smd rect
\t\t\t(at 0 0)
\t\t\t(size 1 1)
\t\t\t(layers "F.Cu")
\t\t\t(net 2 "SHIELD")
\t\t)
\t)
)'''
    p1 = Pad(component_ref='J1', pad_number='S', global_x=-3, global_y=0,
             local_x=0, local_y=0, size_x=1, size_y=1, shape='rect',
             layers=['F.Cu'], net_id=1, net_name='GND')
    p2 = Pad(component_ref='R1', pad_number='1', global_x=0, global_y=0,
             local_x=0, local_y=0, size_x=1, size_y=1, shape='rect',
             layers=['F.Cu'], net_id=2, net_name='SHIELD')
    out = swap_pad_nets_in_content(kicad7_board, p1, p2)
    check("fp_text-referenced footprint found (KiCad 6/7)",
          out != kicad7_board)
    check("BOTH duplicate-numbered pads swapped",
          out.count('(net 2 "SHIELD")') == 2 and out.count('(net 1 "GND")') == 1)

    # -- A9: via removal matches our own %.6f output; net-gated --------------
    print("\nA9: via removal format + net gate")
    via_board = '''(kicad_pcb
\t(via
\t\t(at 104.140000 55.500000)
\t\t(size 0.5)
\t\t(drill 0.3)
\t\t(layers "F.Cu" "B.Cu")
\t\t(net 1)
\t\t(uuid "v1")
\t)
\t(via
\t\t(at 104.140000 55.500000)
\t\t(size 0.5)
\t\t(drill 0.3)
\t\t(layers "F.Cu" "B.Cu")
\t\t(net 2)
\t\t(uuid "v2")
\t)
)'''
    from kicad_writer import add_tracks_and_vias_to_pcb
    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False) as f:
        f.write(via_board)
    outp = f.name + '.out'
    add_tracks_and_vias_to_pcb(f.name, outp, [], [],
                               remove_vias=[{'x': 104.14, 'y': 55.5,
                                             'net_id': 1}])
    out = open(outp).read()
    check("trailing-zero %.6f via matched and removed (was never matched)",
          '(net 1)' not in out)
    check("coincident via of the OTHER net survives (net gate)",
          '(net 2)' in out)

    if fails:
        print(f"\nFAIL ({len(fails)}): " + "; ".join(fails))
        return 1
    print("\nAll checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())

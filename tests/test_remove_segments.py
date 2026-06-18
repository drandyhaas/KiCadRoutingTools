#!/usr/bin/env python3
"""remove_segments_from_content must delete whole (segment ...) blocks, including
KiCad v10 auto-net names that contain parentheses, e.g. (net "Net-(R2-Pad1)")
(issue #84 original-copper stub removal).

    python3 tests/test_remove_segments.py
"""
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_writer import remove_segments_from_content


def _seg(x1, y1, x2, y2, layer='F.Cu', net=153):
    return SimpleNamespace(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                           layer=layer, net_id=net)


def run():
    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)

    # KiCad v10 content (name-only nets); one net name has parentheses.
    v10 = '''(kicad_pcb
\t(version 20250101)
\t(segment (start 1 1) (end 2 2) (width 0.127) (layer "F.Cu") (net "Net-(R2-Pad1)") (uuid "a"))
\t(segment (start 5 5) (end 6 6) (width 0.127) (layer "F.Cu") (net "EPHY_TX_N") (uuid "b"))
\t(segment (start 7 7) (end 8 8) (width 0.127) (layer "B.Cu") (net "Net-(R2-Pad1)") (uuid "c"))
)'''
    n2n = {153: 'Net-(R2-Pad1)', 200: 'EPHY_TX_N'}

    # Remove the paren-named segment "a" (endpoints order-independent, by layer+net).
    out, n = remove_segments_from_content(v10, [_seg(2, 2, 1, 1)], n2n)
    check("paren-name v10 segment removed", n == 1)
    check("target block 'a' gone", '"a"' not in out)
    check("other nets untouched", '"b"' in out and '"c"' in out)

    # A normal (non-paren) v10 net name still removes.
    out2, n2 = remove_segments_from_content(v10, [_seg(5, 5, 6, 6, net=200)], n2n)
    check("plain v10 name removed", n2 == 1 and '"b"' not in out2)

    # Wrong layer / net does not match (no accidental deletion).
    out3, n3 = remove_segments_from_content(v10, [_seg(1, 1, 2, 2, layer='B.Cu')], n2n)
    check("layer mismatch keeps segment", n3 == 0 and '"a"' in out3)

    # KiCad v9 content (numeric nets) still works.
    v9 = '''(kicad_pcb
\t(version 20221018)
\t(segment (start 1 1) (end 2 2) (width 0.127) (layer "F.Cu") (net 153) (uuid "a"))
\t(segment (start 5 5) (end 6 6) (width 0.127) (layer "F.Cu") (net 200) (uuid "b"))
)'''
    out4, n4 = remove_segments_from_content(v9, [_seg(1, 1, 2, 2, net=153)])
    check("v9 numeric-net removal", n4 == 1 and '"a"' not in out4 and '"b"' in out4)

    print("=" * 60)
    if fails:
        for f in fails:
            print(f"  FAIL  {f}")
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("  PASS  v10 paren-name + plain-name removal, layer guard, v9 numeric")
    print("\n5/5 checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(run())

#!/usr/bin/env python3
"""check_drc flags same-net SOFT JOINTS: a dangling free end (a segment terminus
that is not a shared vertex, a via, or an own pad) that reaches the rest of the
net only by cap-overlapping another dangling free end.

A clean route ends every piece at a coincident vertex / via / pad. A free end
floating in copper means the real connecting segment was ripped and never
restored (butterstick DQ5), leaving the net held by a sliver of overlap -- a
fragile near-open. Parallel tracks and normal bends (shared vertices, or ends at
pads/vias) must NOT be flagged.

    python3 tests/test_soft_joint.py
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from check_drc import run_drc
from kicad_writer import generate_segment_sexpr, generate_via_sexpr


def _run(body):
    text = f'''(kicad_pcb
 (version 20221018)
 (layers (0 "F.Cu" signal) (2 "B.Cu" signal))
 (net 0 "")
 (net 1 "/A")
 {body}
)'''
    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False) as f:
        f.write(text)
        path = f.name
    try:
        v = run_drc(path, clearance=0.1, quiet=True)
        return [x for x in v if x['type'] == 'segment-endpoint-gap']
    finally:
        os.unlink(path)


def _seg(x1, y1, x2, y2, w=0.1, net=1):
    return generate_segment_sexpr((x1, y1), (x2, y2), w, 'F.Cu', net)


def run():
    fails = []

    def check(name, cond, detail=""):
        if not cond:
            fails.append(name)
        print(("  PASS " if cond else "  FAIL ") + name + (f"  {detail}" if detail else ""))

    # 1. DANGLING soft joint: two free ends 0.07mm apart, caps (0.1) overlap.
    body = _seg(0, 0, 1.0, 0.0) + _seg(1.05, 0.05, 2.0, 0.05)
    n = len(_run(body))
    check("dangling free ends bridged by overlap -> flagged", n == 1, f"got {n}")

    # 2. Clean COINCIDENT joint (a bend): shared vertex -> NOT flagged.
    body = _seg(0, 0, 1.0, 0.0) + _seg(1.0, 0.0, 1.5, 0.5)
    n = len(_run(body))
    check("coincident vertex (bend) -> not flagged", n == 0, f"got {n}")

    # 3. Free ends anchored at a VIA -> NOT flagged (legitimate terminus).
    via = generate_via_sexpr(1.02, 0.02, 0.3, 0.2, ['F.Cu', 'B.Cu'], 1)
    body = _seg(0, 0, 1.0, 0.0) + _seg(1.05, 0.05, 2.0, 0.05) + via
    n = len(_run(body))
    check("free ends on a via -> not flagged", n == 0, f"got {n}")

    # 4. Far apart (no overlap) -> NOT flagged (genuine disconnection, not a soft joint).
    body = _seg(0, 0, 1.0, 0.0) + _seg(1.5, 0.0, 2.5, 0.0)
    n = len(_run(body))
    check("free ends beyond cap overlap -> not flagged", n == 0, f"got {n}")

    print()
    if fails:
        print(f"FAIL: {len(fails)} check(s): {fails}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(run())

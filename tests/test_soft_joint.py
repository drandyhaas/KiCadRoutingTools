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
import math
import os
import re
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

from kicad_writer import generate_segment_sexpr, generate_via_sexpr


def _run(body):
    """Return the count of same-net soft-joint WARNINGS check_drc reports (they are
    surfaced as warnings, not counted violations)."""
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
        r = subprocess.run([sys.executable, 'check_drc.py', path, '-c', '0.1'],
                           capture_output=True, text=True, cwd=REPO)
        out = r.stdout + r.stderr
        # #320 step 3: soft joints are COUNTED violations (per-type header),
        # EXCEPT the sub-coincidence band (<= COINCIDENCE_TOL) -> warning line.
        m = re.search(r'SEGMENT-ENDPOINT-GAP violations \((\d+)\)', out)
        w = re.search(r'sub-coincidence endpoint gap: (\d+)', out)
        return (int(m.group(1)) if m else 0), (int(w.group(1)) if w else 0)
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
    n, _w = _run(body)
    check("dangling free ends bridged by overlap -> flagged", n == 1, f"got {n}")

    # 2. Clean COINCIDENT joint (a bend): shared vertex -> NOT flagged.
    body = _seg(0, 0, 1.0, 0.0) + _seg(1.0, 0.0, 1.5, 0.5)
    n, _w = _run(body)
    check("coincident vertex (bend) -> not flagged", n == 0, f"got {n}")

    # 3. Free ends anchored at a VIA -> NOT flagged (legitimate terminus).
    via = generate_via_sexpr(1.02, 0.02, 0.3, 0.2, ['F.Cu', 'B.Cu'], 1)
    body = _seg(0, 0, 1.0, 0.0) + _seg(1.05, 0.05, 2.0, 0.05) + via
    n, _w = _run(body)
    check("free ends on a via -> not flagged", n == 0, f"got {n}")

    # 4. Far apart (no overlap) -> NOT flagged (genuine disconnection, not a soft joint).
    body = _seg(0, 0, 1.0, 0.0) + _seg(1.5, 0.0, 2.5, 0.0)
    n, _w = _run(body)
    check("free ends beyond cap overlap -> not flagged", n == 0, f"got {n}")

    # 5. SUB-COINCIDENCE band: gap <= COINCIDENCE_TOL (0.02) is quantization-
    # level contact every gate/cleanup treats as connected -> reported as a
    # WARNING, never a counted violation (kuchen /USBH_DN).
    body = _seg(0, 0, 1.0, 0.0) + _seg(1.015, 0.0, 2.0, 0.0)
    n, w = _run(body)
    check("sub-coincidence gap (15um) -> not a counted violation", n == 0, f"got {n}")
    check("sub-coincidence gap (15um) -> warning line", w == 1, f"got {w}")

    # --- close_soft_joints repair pass ---
    from kicad_parser import Segment, Net, PCBData, BoardInfo
    from pcb_modification import close_soft_joints
    from routing_config import GridRouteConfig

    def _cfg():
        c = GridRouteConfig()
        c.clearance = 0.1
        c.layers = ['F.Cu', 'B.Cu']
        return c

    # two dangling free ends 0.07mm apart (caps 0.1 overlap) -> one tiny bridge.
    segs = [Segment(start_x=0.0, start_y=0.0, end_x=1.0, end_y=0.0, width=0.1, layer='F.Cu', net_id=1),
            Segment(start_x=1.05, start_y=0.05, end_x=2.0, end_y=0.05, width=0.1, layer='F.Cu', net_id=1)]
    pcb = PCBData(footprints={}, nets={1: Net(1, '/A')}, segments=segs, vias=[],
                  board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu']),
                  pads_by_net={})
    results = []
    n = close_soft_joints(results, pcb, None, _cfg())
    check("close_soft_joints bridges the soft joint", n == 1, f"added {n}")
    bridge = [s for s in pcb.segments if abs(s.start_x - 1.0) < 1e-6 and abs(s.end_x - 1.05) < 1e-6]
    check("bridge connects the two exact endpoints (coincident)", len(bridge) == 1)
    check("bridge is TINY (< a track width)",
          bridge and math.hypot(bridge[0].end_x - bridge[0].start_x,
                                bridge[0].end_y - bridge[0].start_y) < 0.1)
    # idempotent: re-running adds nothing (the joint is now coincident, count 2).
    n2 = close_soft_joints([], pcb, None, _cfg())
    check("close_soft_joints is idempotent", n2 == 0, f"added {n2}")

    print()
    if fails:
        print(f"FAIL: {len(fails)} check(s): {fails}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(run())

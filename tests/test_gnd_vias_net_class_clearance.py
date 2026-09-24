#!/usr/bin/env python3
"""route_planes --add-gnd-vias must place GND return vias at each foreign
net's CLASS clearance, not the single base clearance.

The GND-via post-pass built its obstacle map without the per-net clearance map
every other routing step auto-reads from the sibling .kicad_pro, so a return
via could land at the Default clearance (0.2) next to an HV net whose class
demands 0.8 -- a DRC violation acquired AFTER routing.

Tiny 2-layer board: a SIG via at (10,10), an HV SMD pad 1.8mm to its right
(gap 0.4mm from the natural first GND-via spot), a GND through-hole pad far
away to anchor the plane. HV is in a 0.8mm net class; Default is 0.2.

    python3 tests/test_gnd_vias_net_class_clearance.py
"""
import json
import math
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522

from kicad_parser import parse_kicad_pcb

HV_CLEARANCE = 0.8
BASE_CLEARANCE = 0.2
VIA_SIZE = 0.6
HV_PAD = (11.8, 10.0, 0.6)   # x, y, square size

BOARD = f"""(kicad_pcb (version 20240108) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (net 0 "")
  (net 1 "GND")
  (net 2 "SIG")
  (net 3 "HV")
  (footprint "t:TH" (layer "F.Cu") (at 18 18)
    (property "Reference" "J1" (at 0 0) (layer "F.SilkS"))
    (pad "1" thru_hole circle (at 0 0) (size 1.6 1.6) (drill 0.8)
      (layers "*.Cu") (net 1 "GND")))
  (footprint "t:SMD" (layer "F.Cu") (at {HV_PAD[0]} {HV_PAD[1]})
    (property "Reference" "U1" (at 0 0) (layer "F.SilkS"))
    (pad "1" smd rect (at 0 0) (size {HV_PAD[2]} {HV_PAD[2]})
      (layers "F.Cu") (net 3 "HV")))
  (via (at 10 10) (size {VIA_SIZE}) (drill 0.3) (layers "F.Cu" "B.Cu") (net 2))
  (gr_rect (start 0 0) (end 20 20) (stroke (width 0.1) (type default))
    (fill none) (layer "Edge.Cuts"))
)
"""

PROJECT = {
    "board": {"design_settings": {"rules": {}}},
    "net_settings": {
        "meta": {"version": 3},
        "classes": [
            {"name": "Default", "clearance": BASE_CLEARANCE, "track_width": 0.2,
             "via_diameter": VIA_SIZE, "via_drill": 0.3},
            {"name": "HV", "clearance": HV_CLEARANCE, "track_width": 0.2,
             "via_diameter": VIA_SIZE, "via_drill": 0.3},
        ],
        "netclass_assignments": {"HV": "HV"},
    },
    "meta": {"version": 1},
}


def _gap_to_hv_pad(via):
    """Copper gap (mm) between a via ring and the square HV pad."""
    px, py, s = HV_PAD
    dx = max(abs(via.x - px) - s / 2, 0.0)
    dy = max(abs(via.y - py) - s / 2, 0.0)
    return math.hypot(dx, dy) - via.size / 2


def _route(d, tag, extra):
    """Run route_planes --add-gnd-vias; return (gaps to HV pad, log)."""
    src = os.path.join(d, 'b.kicad_pcb')
    out = os.path.join(d, f'out_{tag}.kicad_pcb')   # fresh output per run
    r = subprocess.run(
        [sys.executable, os.path.join(ROOT, 'py_router', 'route_planes.py'),
         src, out, '--nets', 'GND', '--plane-layers', 'B.Cu',
         '--clearance', str(BASE_CLEARANCE), '--via-size', str(VIA_SIZE),
         '--via-drill', '0.3', '--track-width', '0.2', '--grid-step', '0.1',
         '--add-gnd-vias', '--gnd-via-distance', '3.0', *extra],
        capture_output=True, text=True, cwd=ROOT)
    if r.returncode != 0 or not os.path.exists(out):
        return None, r.stdout[-3000:] + r.stderr[-3000:]
    pcb = parse_kicad_pcb(out)
    gaps = [((v.x, v.y), _gap_to_hv_pad(v)) for v in pcb.vias if v.net_id == 1]
    return gaps, r.stdout[-3000:]


def run():
    fails = []
    with tempfile.TemporaryDirectory() as d:
        with open(os.path.join(d, 'b.kicad_pcb'), 'w') as f:
            f.write(BOARD)
        with open(os.path.join(d, 'b.kicad_pro'), 'w') as f:
            json.dump(PROJECT, f, indent=2)

        # 1. Classes honoured (no ceiling): every GND via clears HV at 0.8.
        # 2. --clearance-ceiling 0.2 caps every class (#439): 0.2 governs, so
        #    the via may take the natural spot -- but never below the ceiling.
        for tag, extra, need in (('honor', [], HV_CLEARANCE),
                                 ('ceiling', ['--clearance-ceiling',
                                              str(BASE_CLEARANCE)], BASE_CLEARANCE)):
            gaps, log = _route(d, tag, extra)
            if gaps is None:
                print(log)
                fails.append(f"[{tag}] route_planes did not produce an output board")
                continue
            if not gaps:
                print(log)
                fails.append(f"[{tag}] no GND return via placed")
            for (x, y), gap in gaps:
                print(f"  [{tag}] GND via at ({x:.2f},{y:.2f}): {gap:.3f}mm to HV pad "
                      f"(required {need})")
                if gap < need - 1e-6:
                    fails.append(f"[{tag}] GND via at ({x:.2f},{y:.2f}) is {gap:.3f}mm "
                                 f"from the HV pad < required {need}mm")

    print("=" * 60)
    if fails:
        for f in fails:
            print(f"FAIL: {f}")
        return 1
    print("PASS: GND return vias keep the HV net-class clearance (ceiling honoured)")
    return 0


if __name__ == "__main__":
    sys.exit(run())

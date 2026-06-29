#!/usr/bin/env python3
"""check_drc must flag pad-to-pad clearance/short violations between DIFFERENT
nets on a shared copper layer (issue #234). Previously it only had pad-segment
and pad-via passes, so two overlapping/too-close component pads were invisible
(KiCad reports these as shorting_items / clearance).

Pads of the SAME footprint are NOT flagged: a component's own adjacent pins are
fixed library geometry (fine-pitch parts legitimately sit below the routing
clearance) and are never pipeline-introduced -- flagging them would flood
grading with pre-existing noise. The cases #234 targets are between different
footprints (e.g. a cap nudged onto an IC pad).

    python3 tests/test_drc_pad_pad.py
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from check_drc import run_drc


def _board(pad2_x, pad2_net=2, pad2_layer="F.Cu", same_footprint=False):
    """Two single-pad footprints; pad1 at (10,10) net /A, pad2 at (pad2_x,10).
    Pads are 0.6mm square. When same_footprint, both pads live in one footprint.
    """
    if same_footprint:
        body = f'''(footprint "L:A" (layer "F.Cu") (at 10 10)
   (property "Reference" "U1")
   (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net 1 "/A"))
   (pad "2" smd rect (at {pad2_x - 10} 0) (size 0.6 0.6) (layers "{pad2_layer}") (net {pad2_net} "/B")))'''
    else:
        body = f'''(footprint "L:A" (layer "F.Cu") (at 10 10)
   (property "Reference" "U1")
   (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net 1 "/A")))
 (footprint "L:B" (layer "F.Cu") (at {pad2_x} 10)
   (property "Reference" "U2")
   (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "{pad2_layer}") (net {pad2_net} "/B")))'''
    return f'''(kicad_pcb
 (version 20221018)
 (net 0 "")
 (net 1 "/A")
 (net 2 "/B")
 {body}
)'''


def _run(**kw):
    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False) as f:
        f.write(_board(**kw))
        path = f.name
    try:
        v = run_drc(path, clearance=0.1, quiet=True)
        return [x for x in v if x['type'] == 'pad-pad']
    finally:
        os.unlink(path)


def run():
    fails = []

    def check(name, cond):
        print(f"  {'PASS' if cond else 'FAIL'}  {name}")
        if not cond:
            fails.append(name)

    # Overlapping different-net pads of different footprints -> a short.
    overlap = _run(pad2_x=10.3)
    check("overlapping different-net pads -> 1 pad-pad violation (short)",
          len(overlap) == 1 and abs(overlap[0]['overlap_mm'] - 0.1) < 1e-6)

    # Gap 0.05mm < 0.1mm clearance -> a clearance violation.
    subclr = _run(pad2_x=10.65)
    check("sub-clearance pads (gap 0.05) -> 1 violation, overlap 0.05",
          len(subclr) == 1 and abs(subclr[0]['overlap_mm'] - 0.05) < 1e-6)

    # Gap 0.2mm > 0.1mm clearance -> clean.
    check("well-spaced pads (gap 0.2) -> 0 violations", len(_run(pad2_x=10.8)) == 0)

    # Same net overlapping -> allowed (no violation).
    check("same-net overlapping pads -> 0 violations",
          len(_run(pad2_x=10.3, pad2_net=1)) == 0)

    # Different copper layers -> no shared layer, no conflict.
    check("overlapping pads on different layers -> 0 violations",
          len(_run(pad2_x=10.3, pad2_layer="B.Cu")) == 0)

    # Adjacent pins of the SAME footprint below clearance -> NOT flagged.
    check("same-footprint sub-clearance pins -> 0 violations (pre-existing noise)",
          len(_run(pad2_x=10.65, same_footprint=True)) == 0)

    print("=" * 60)
    if fails:
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("\n6/6 checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(run())

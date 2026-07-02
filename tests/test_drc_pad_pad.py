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


def _run_text(body, clearance=0.1, vtype='pad-pad'):
    text = f'''(kicad_pcb
 (version 20221018)
 (layers
  (0 "F.Cu" signal)
  (2 "B.Cu" signal)
 )
 (net 0 "")
 (net 1 "/A")
 (net 2 "/B")
 {body}
)'''
    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False) as f:
        f.write(text)
        path = f.name
    try:
        v = run_drc(path, clearance=clearance, quiet=True)
        return [x for x in v if x['type'] == vtype]
    finally:
        os.unlink(path)


def run():
    fails = []

    total = [0]

    def check(name, cond):
        total[0] += 1
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

    # Two circle pads offset DIAGONALLY with a legal true (edge-to-edge) gap.
    # The old perimeter sampler walked the bounding box, whose corner sits
    # sqrt(2)*r - r outside the copper, so this pair read as a SHORT even
    # though the circles clear each other (issue #260: nitrokey_pro J2 NPTH
    # vs plug pads, 0.186mm real gap flagged as 4 phantom shorts).
    diag = '''(footprint "L:A" (layer "F.Cu") (at 10 10)
   (property "Reference" "U1")
   (pad "1" smd circle (at 0 0) (size 1.1 1.1) (layers "F.Cu") (net 1 "/A")))
 (footprint "L:B" (layer "F.Cu") (at 11.27 10.95)
   (property "Reference" "U2")
   (pad "1" smd circle (at 0 0) (size 1.7 1.7) (layers "F.Cu") (net 2 "/B")))'''
    check("diagonal circle pads with legal true gap -> 0 violations (bbox phantom)",
          len(_run_text(diag)) == 0)

    # ... but genuinely overlapping circles on the same diagonal still flag.
    diag_touch = diag.replace("(at 11.27 10.95)", "(at 10.9 10.7)")
    check("diagonal circle pads truly overlapping -> 1 violation",
          len(_run_text(diag_touch)) == 1)

    # An NPTH mounting hole has NO copper even when its (layers ...) lists
    # *.Cu -- it must not produce pad-pad copper violations (issue #260).
    npth = '''(footprint "L:A" (layer "F.Cu") (at 10 10)
   (property "Reference" "J1")
   (pad "" np_thru_hole circle (at 0 0) (size 1.1 1.1) (drill 1.1) (layers "*.Cu" "*.Mask")))
 (footprint "L:B" (layer "F.Cu") (at 10.9 10)
   (property "Reference" "U2")
   (pad "1" smd circle (at 0 0) (size 1.7 1.7) (layers "F.Cu") (net 2 "/B")))'''
    check("NPTH hole overlapping a pad -> 0 pad-pad violations (no copper)",
          len(_run_text(npth)) == 0)

    # The same NPTH hole IS still guarded against tracks: copper-to-hole picks
    # it up even though its layer list claims *.Cu (previously missed).
    npth_track = '''(footprint "L:A" (layer "F.Cu") (at 10 10)
   (property "Reference" "J1")
   (pad "" np_thru_hole circle (at 0 0) (size 1.1 1.1) (drill 1.1) (layers "*.Cu" "*.Mask")))
 (segment (start 8 10.6) (end 12 10.6) (width 0.2) (layer "F.Cu") (net 2) (uuid "0"))'''
    check("track grazing the NPTH hole -> 1 track-hole violation",
          len(_run_text(npth_track, vtype='track-hole')) == 1)

    # A via overlapping a pad on a copper layer that carries NO segments must
    # still be flagged: routing_layers used to be derived from segments, so a
    # trackless layer's pad/via checks silently never ran (#253 -- a fanout
    # via ring over a B.Cu test pad graded clean until B.Cu gained tracks).
    layer_blind = '''(footprint "L:A" (layer "B.Cu") (at 10 10)
   (property "Reference" "TP1")
   (pad "1" smd circle (at 0 0) (size 0.5 0.5) (layers "B.Cu" "B.Mask") (net 1 "/A")))
 (segment (start 5 5) (end 8 5) (width 0.2) (layer "F.Cu") (net 2) (uuid "s1"))
 (via (at 10.25 10.25) (size 0.3) (drill 0.15) (layers "F.Cu" "B.Cu") (net 2) (uuid "v1"))'''
    v = _run_text(layer_blind, vtype='pad-via')
    check("via vs pad on a segment-less copper layer -> flagged (pad-via)",
          len(v) == 1)

    # A no-net pad overlap is still reported, but flagged no_net (not a SHORT:
    # unconnected copper cannot electrically short a net -- issue #260).
    nonet = '''(footprint "L:A" (layer "F.Cu") (at 10 10)
   (property "Reference" "U1")
   (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net 0 "")))
 (footprint "L:B" (layer "F.Cu") (at 10.3 10)
   (property "Reference" "U2")
   (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net 2 "/B")))'''
    nonet_v = _run_text(nonet)
    check("no-net pad overlap -> 1 violation carrying no_net=True",
          len(nonet_v) == 1 and nonet_v[0].get('no_net') is True)

    print("=" * 60)
    if fails:
        print(f"\n{len(fails)} failure(s)")
        return 1
    print(f"\n{total[0]}/{total[0]} checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(run())

#!/usr/bin/env python3
"""A gap via counts as clear of a movable cap only when its whole RING clears the pad.

  python3 tests/test_underpad_soft_via_ring.py

The under-pad engine keeps movable passives (unlocked 2-pad C/R/FB) as SOFT
keep-outs (#278): a dog-bone gap site under one is taken only when every
soft-free site is gone, because cap-opt is expected to move the part. The
soft stamps are a TRACK's keep (pad half + track/2 + clearance), and the
gap-site test read them at the via's CENTRE cell alone, so a via centred just
past that disk still put its ring on the pad and was taken as soft-free. On
butterstick U4 a lane-walked P_1V2_FPGA plane drop sat 0.502 mm from C39.2's
centre (the disk is 0.475), its 0.2 mm ring 0.06 mm into the GND pad;
place_fanout_clearance found no pose for C39 and the short shipped.

The fixture is that geometry on a 4x4, 0.8 mm BGA: a plane ball whose four
gap sites are A (ring on a B.Cu cap pad, centre outside the track-keep disk),
B and D (taken by foreign vias) and C (clean). Pinned:

1. the fixture discriminates: A's via ring overlaps the pad while its centre
   lies outside the soft stamp's radius (A faces the ball's nearer boundary,
   so it is tried before C, and the old centre-cell test took it);
2. the real drop pass (generate_plane_drops) lays the ball's via, and every
   via it lays clears every cap pad by the clearance -- at C, not A.
"""
import io
import math
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

PITCH, BALL = 0.8, 0.4
CX, CY = 50.0, 50.0
TW, CL, VIA, DRILL = 0.13, 0.09, 0.4, 0.2
LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
# the drop ball and its four gap sites (board coordinates)
BX, BY = CX - 0.4, CY - 0.4
SITE_A = (CX - 0.8, CY - 0.8)       # under the cap pad's edge
SITE_B = (CX - 0.8, CY)             # foreign via
SITE_C = (CX, CY - 0.8)             # clean
SITE_D = (CX, CY)                   # foreign via
# C1.1 sits 0.28 west and 0.45 north of A: its 0.64-tall pad edge is 0.13 from
# A, inside a 0.2 via ring, while A is 0.53 from its centre, outside the
# track-keep disk (0.32 + 0.065 + 0.09 = 0.475)
CAP_PAD1 = (SITE_A[0] - 0.28, SITE_A[1] - 0.45)
CAP_AT = (CAP_PAD1[0] - 0.485, CAP_PAD1[1])


def _board():
    nets = ['', 'P', 'X', 'G1', 'G2']
    coords = [-1.2, -0.4, 0.4, 1.2]
    balls = []
    for r, y in enumerate(coords):
        for c, x in enumerate(coords):
            name = 'ABCD'[r] + str(c + 1)
            if (x, y) in ((-0.4, -0.4), (1.2, 1.2)):
                net = 'P'
            else:
                net = 'S' + name
                nets.append(net)
            balls.append((name, x, y, net))
    nid = {n: i for i, n in enumerate(nets)}
    out = ['(kicad_pcb (version 20241229) (generator "synth") (generator_version "9.0")',
           '  (general (thickness 1.6) (legacy_teardrops no))', '  (paper "A4")',
           '  (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal) '
           '(31 "B.Cu" signal) (32 "B.Adhes" user) (34 "B.Paste" user) '
           '(35 "F.Paste" user) (37 "F.SilkS" user) (38 "B.Mask" user) '
           '(39 "F.Mask" user) (44 "Edge.Cuts" user))',
           '  (setup (pad_to_mask_clearance 0))']
    out += ['  (net %d "%s")' % (i, n) for i, n in enumerate(nets)]
    pads = ''.join(
        '    (pad "%s" smd circle (at %s %s) (size %s %s) (layers "F.Cu" "F.Paste" '
        '"F.Mask") (net %d "%s") (uuid "u1-%s"))\n'
        % (n, x, y, BALL, BALL, nid[net], net, n) for n, x, y, net in balls)
    out.append('  (footprint "synth:BGA16" (layer "F.Cu") (uuid "u1") (at %s %s)\n'
               '    (property "Reference" "U1" (at 0 -3 0) (layer "F.SilkS") (uuid "u1-ref")\n'
               '      (effects (font (size 0.5 0.5) (thickness 0.1))))\n%s  )'
               % (CX, CY, pads))
    out.append('  (footprint "synth:C_0402" (layer "B.Cu") (uuid "c1") (at %s %s)\n'
               '    (property "Reference" "C1" (at 0 1 0) (layer "B.SilkS") (uuid "c1-ref")\n'
               '      (effects (font (size 0.5 0.5) (thickness 0.1)) (justify mirror)))\n'
               '    (pad "1" smd roundrect (at 0.485 0) (size 0.59 0.64) '
               '(layers "B.Cu" "B.Paste" "B.Mask") (roundrect_rratio 0.25) '
               '(net %d "G1") (uuid "c1-1"))\n'
               '    (pad "2" smd roundrect (at -0.485 0) (size 0.59 0.64) '
               '(layers "B.Cu" "B.Paste" "B.Mask") (roundrect_rratio 0.25) '
               '(net %d "G2") (uuid "c1-2"))\n  )'
               % (CAP_AT[0], CAP_AT[1], nid['G1'], nid['G2']))
    for i, (x, y) in enumerate((SITE_B, SITE_D)):
        out.append('  (via (at %s %s) (size %s) (drill %s) (layers "F.Cu" "B.Cu") '
                   '(net %d) (uuid "vx%d"))' % (x, y, VIA, DRILL, nid['X'], i))
    out.append('  (gr_rect (start 0 0) (end 100 100) (layer "Edge.Cuts") '
               '(stroke (width 0.1) (type solid)) (fill none) (uuid "edge"))')
    return '\n'.join(out) + '\n)\n'


def _quiet(fn):
    buf, old = io.StringIO(), sys.stdout
    sys.stdout = buf
    try:
        return fn()
    finally:
        sys.stdout = old


def main():
    print('=' * 60)
    print("under-pad gap vias: a movable cap's pad is clear only of the whole ring")
    print('=' * 60)
    from kicad_parser import parse_kicad_pcb
    from routing_utils import point_to_pad_rect_dist
    from bga_fanout import generate_plane_drops
    from bga_fanout.geometry import via_clears_pad_rects
    fails = []
    d = tempfile.mkdtemp(prefix='softvia_')
    path = os.path.join(d, 'softvia.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(_board())
    pcb = _quiet(lambda: parse_kicad_pcb(path))
    cap = [p for p in pcb.footprints['C1'].pads]
    c11 = next(p for p in cap if p.pad_number == '1')

    # 1. the fixture discriminates
    if math.hypot(c11.global_x - CAP_PAD1[0], c11.global_y - CAP_PAD1[1]) > 1e-6:
        fails.append(f"fixture: C1.1 parsed at ({c11.global_x},{c11.global_y}), "
                     f"not {CAP_PAD1}")
    soft_r = max(c11.size_x, c11.size_y) / 2.0 + TW / 2 + CL
    centre = math.hypot(SITE_A[0] - c11.global_x, SITE_A[1] - c11.global_y)
    ring = point_to_pad_rect_dist(SITE_A[0], SITE_A[1], c11)
    if not centre > soft_r + 0.03:
        fails.append(f"fixture: site A is {centre:.3f} from C1.1's centre, not "
                     f"clearly outside the track-keep disk ({soft_r:.3f})")
    if not ring < VIA / 2:
        fails.append(f"fixture: a via at A is {ring:.3f} from C1.1's copper, so its "
                     f"{VIA / 2} ring does not reach the pad")
    if via_clears_pad_rects(*SITE_C, VIA / 2, CL, cap) is False:
        fails.append("fixture: site C is not clear of the cap")

    # 2. the real drop pass
    tracks, vias, rep = _quiet(lambda: generate_plane_drops(
        pcb.footprints['U1'], pcb, LAYERS, track_width=TW, clearance=CL,
        via_size=VIA, via_drill=DRILL, net_filter=['*', '!P'],
        plane_min_pads=1, verbose=False))
    pid = next(n for n, v in pcb.nets.items() if v.name == 'P')
    mine = [v for v in vias if v['net_id'] == pid
            and math.hypot(v['x'] - BX, v['y'] - BY) < PITCH]
    if not mine:
        fails.append(f"the drop pass laid no via for the ball at ({BX},{BY}) "
                     f"(report {rep}) -- the check below would pass on nothing")
    for v in vias:
        if not via_clears_pad_rects(v['x'], v['y'], v['size'] / 2.0, CL, cap):
            fails.append(f"a {v['size']} drop via at ({v['x']:.3f},{v['y']:.3f}) "
                         f"lands on C1 (pad edge "
                         f"{min(point_to_pad_rect_dist(v['x'], v['y'], p) for p in cap):.3f} "
                         f"from its centre) though site C was clean")
    for f in fails:
        print(f'  FAIL: {f}')
    if fails:
        return 1
    at = ', '.join(f"({v['x']:.2f},{v['y']:.2f})" for v in mine)
    print(f"PASS: site A's ring reaches C1.1 though its centre is outside the soft "
          f"stamp; the drop pass put the ball's via at {at}, clear of the cap")
    return 0


if __name__ == '__main__':
    sys.exit(main())

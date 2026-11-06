#!/usr/bin/env python3
"""#1732: the post-route cleanup treats a same-net X crossing as the joint it is.

Two same-layer tracks of one net that cross mid-span are joined there
(check_connected, KiCad, and since #1705 the length graph). The cleanup did
not know it:

  * the dangle trim (`trim_dangles_past_body_anchor`) cuts a dead-end tail
    back to a via or a tee on the segment's body, never to a crossing, so
    the 1 mm overshoot past an X shipped as an antenna (KiCad:
    track_dangling) and as length the pre-cleanup estimate leaves out;
  * the stub-gap snap (`snap_stub_gaps`, route.py's cleanup) read the short
    overshoot's end, 0.168 mm past the other track, as a stub that stopped
    short of it, and laid a connector straight back over the overshoot:
    stacked copper on a join that already existed.

Now a crossing is a body anchor for the trim, and the snap skips a target its
own segment already crosses.

Checks (P joined only by an X: a horizontal 1 mm past the crossing, a
vertical 0.168 mm past it, 0.14 mm tracks):
  1. route_diff's cleanup (snap off) cuts the 1 mm tail back to the
     crossing; the 0.168 mm one is under the trim's 3-width floor and stays,
     as any such nib does; P stays connected;
  2. route.py's cleanup (snap on) lays no connector over the short tail and
     ships the same copper;
  3. controls: a track crossing on ANOTHER layer is no anchor (the tail is
     cut back past it, to the tee behind it), and a T-junction tail is still
     trimmed as before.

    python3 tests/test_1732_crossing_trim_anchor.py
"""
import contextlib
import io
import math
import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb                  # noqa: E402
from routing_config import GridRouteConfig                # noqa: E402
from cleanup_pipeline import run_post_route_cleanup       # noqa: E402
from check_connected import check_net_connectivity        # noqa: E402
from run_utils import evidence                            # noqa: E402

failures = []
NETS = {'/CK_P': 1}


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def build(out, shape):
    def pad(ref, x, y, layers='"F.Cu" "B.Cu"'):
        return (f'  (footprint "T:{ref}" (layer "F.Cu") (at {x} {y})\n'
                f'    (property "Reference" "{ref}" (at 0 -1 0) (layer "F.SilkS") '
                f'(hide yes) (effects (font (size 1 1) (thickness 0.15))))\n'
                f'    (attr smd)\n    (pad "1" smd circle (at 0 0) (size 0.4 0.4) '
                f'(layers {layers}) (net 1 "/CK_P"))\n  )\n')

    def seg(x1, y1, x2, y2, lay='F.Cu'):
        return (f'  (segment (start {x1} {y1}) (end {x2} {y2}) (width 0.14) '
                f'(layer "{lay}") (net 1))\n')

    b = ('(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")\n'
         ' (general (thickness 1.6))\n (paper "A4")\n'
         ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))\n'
         ' (setup (pad_to_mask_clearance 0))\n (net 0 "")\n (net 1 "/CK_P")\n')
    b += pad('J1', 105, 105) + pad('J2', 115, 110)
    if shape == 'layers':
        b += pad('J3', 110, 110) + pad('J4', 115, 103)
    b += {
        'cross': seg(105, 105, 116, 105) + seg(115, 110, 115, 104.832),
        # the horizontal is teed at x=110 from J3 and runs on to x=116; a
        # B.Cu track J2-J4 crosses it at x=115
        'layers': (seg(105, 105, 116, 105) + seg(110, 110, 110, 105)
                   + seg(115, 110, 115, 103, 'B.Cu')),
        # the vertical ends ON the horizontal (a tee); the horizontal runs on
        'tee': seg(105, 105, 116, 105) + seg(115, 110, 115, 105),
    }[shape]
    for x1, y1, x2, y2 in ((100, 100, 125, 100), (125, 100, 125, 118),
                           (125, 118, 100, 118), (100, 118, 100, 100)):
        b += (f'  (gr_line (start {x1} {y1}) (end {x2} {y2}) (layer "Edge.Cuts") '
              f'(stroke (width 0.05) (type solid)))\n')
    with open(out, 'w') as fh:
        fh.write(b + ')\n')


def clean(path, snap):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(path)
    cfg = GridRouteConfig(track_width=0.14, clearance=0.1, layers=['F.Cu', 'B.Cu'])
    results = [{'new_segments': list(pcb.segments), 'new_vias': []}]
    with contextlib.redirect_stdout(io.StringIO()):
        if snap:   # route.py's switches
            run_post_route_cleanup(results, pcb, {1}, cfg)
        else:      # route_diff's
            run_post_route_cleanup(results, pcb, {1}, cfg, snap=False, phantom=False,
                                   neck=False)
    conn = check_net_connectivity(1, list(pcb.segments), [], pcb.pads_by_net[1])
    snaps = [s for r in results if r.get('cleanup') == 'stub_gap_snap'
             for s in r['new_segments']]
    copper = math.fsum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
                       for s in pcb.segments)
    return pcb, conn, snaps, copper


def horizontal_end(pcb, y=105.0, layer='F.Cu'):
    return max(max(s.start_x, s.end_x) for s in pcb.segments
               if s.layer == layer and abs(s.start_y - y) < 1e-6
               and abs(s.end_y - y) < 1e-6)


def main():
    with tempfile.TemporaryDirectory() as tmp:
        paths = {}
        for shape in ('cross', 'layers', 'tee'):
            paths[shape] = os.path.join(tmp, f'{shape}.kicad_pcb')
            build(paths[shape], shape)
            evidence(paths[shape], f'the {shape} board')

        for snap, who in ((False, "route_diff's cleanup (snap off)"),
                          (True, "route.py's cleanup (snap on)")):
            pcb, conn, snaps, copper = clean(paths['cross'], snap)
            x = horizontal_end(pcb)
            check(f'{who}: the 1 mm tail past the X is cut back to the crossing',
                  abs(x - 115.0) < 1e-6, f'horizontal ends at x={x:.4f}')
            check(f'{who}: P stays connected', conn.get('connected') is True,
                  str(conn.get('connected')))
            check(f'{who}: no connector is laid over the short tail, and the '
                  f'copper is 15.168 mm', not snaps and abs(copper - 15.168) < 1e-6,
                  f'{len(snaps)} snap connector(s); copper {copper:.4f}')

        pcb, _c, _s, copper = clean(paths['layers'], False)
        x = horizontal_end(pcb)
        check('control: a crossing on another layer is no anchor (the tail is cut '
              'back past it, to the tee at x=110)', abs(x - 110.0) < 1e-6,
              f'horizontal ends at x={x:.4f}')
        pcb, _c, _s, copper = clean(paths['tee'], False)
        check('control: a tee still anchors its tail (cut back to x=115)',
              abs(horizontal_end(pcb) - 115.0) < 1e-6, f'copper {copper:.4f}')

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

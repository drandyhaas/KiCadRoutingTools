#!/usr/bin/env python3
"""#1207: route_diff's companion GND vias keep the PAIR's class clearance.

`_create_gnd_vias` placed each GND via at `spacing + track/2 +
config.clearance + via/2` off the centerline and `via + config.clearance`
along it, and the router's reservation used the same offsets. That is the
run's base (the Default class); check_drc grades GND against a pair in a
wider class at max(Default, class). On CM5_MINIMA_3 a 90-ohm class at 0.2 over
a 0.13 Default put 12 GND vias inside it (19 VIA-SEGMENT violations).

Tiny 4-layer board: a pair /D_P,/D_N from an F.Cu part to a B.Cu part, so it
must change layers, in a 0.2 class over a 0.13 Default, with a GND pad.

Checks:
  1. `_gnd_via_offsets` prices the pair: 0.2, not the 0.13 base.
  2. route_diff routes the pair and places GND vias; every GND via clears
     every pair track AS DRAWN at 0.2 (the tracks follow the smoothed path,
     which can sit off the grid centerline the offsets are measured from).
  3. check_drc reports no GND <-> pair violation.

    python3 tests/test_1207_gnd_vias_pair_class.py
"""
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from geometry_utils import point_to_segment_distance_seg      # noqa: E402
from kicad_parser import parse_kicad_pcb                       # noqa: E402

BASE, CLASS, VIA, TRACK = 0.13, 0.2, 0.45, 0.2
BOARD = """(kicad_pcb (version 20240108) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal)
          (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (net 0 "")
  (net 1 "GND")
  (net 2 "/D_P")
  (net 3 "/D_N")
  (footprint "t:TH" (layer "F.Cu") (at 15 17)
    (property "Reference" "J1" (at 0 0) (layer "F.SilkS"))
    (pad "1" thru_hole circle (at 0 0) (size 1.6 1.6) (drill 0.8)
      (layers "*.Cu") (net 1 "GND")))
  (footprint "t:P2" (layer "F.Cu") (at 4 8)
    (property "Reference" "U1" (at 0 0) (layer "F.SilkS"))
    (pad "1" smd rect (at 0 -0.25) (size 0.8 0.3) (layers "F.Cu") (net 2 "/D_P"))
    (pad "2" smd rect (at 0 0.25) (size 0.8 0.3) (layers "F.Cu") (net 3 "/D_N")))
  (footprint "t:P2" (layer "B.Cu") (at 26 8)
    (property "Reference" "U2" (at 0 0) (layer "B.SilkS"))
    (pad "1" smd rect (at 0 -0.25) (size 0.8 0.3) (layers "B.Cu") (net 2 "/D_P"))
    (pad "2" smd rect (at 0 0.25) (size 0.8 0.3) (layers "B.Cu") (net 3 "/D_N")))
  (gr_rect (start 0 0) (end 30 20) (stroke (width 0.1) (type default))
    (fill none) (layer "Edge.Cuts"))
)
"""


def _cls(name, clr):
    return {"name": name, "clearance": clr, "track_width": TRACK,
            "via_diameter": VIA, "via_drill": 0.3, "diff_pair_gap": 0.2,
            "diff_pair_width": TRACK}


PROJECT = {"board": {"design_settings": {"rules": {}}},
           "net_settings": {"meta": {"version": 3},
                            "classes": [_cls("Default", BASE), _cls("90ohm", CLASS)],
                            "netclass_patterns": [{"netclass": "90ohm",
                                                   "pattern": "/D_*"}]},
           "meta": {"version": 1}}
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def main():
    # 1. The offsets price the pair.
    from routing_config import GridRouteConfig
    from diff_pair_routing import _gnd_via_offsets
    cfg = GridRouteConfig(clearance=BASE, track_width=TRACK, via_size=VIA,
                          via_drill=0.3, net_clearances={2: CLASS, 3: CLASS})
    perp, along, clr = _gnd_via_offsets(cfg, 0.2, 1, (2, 3))
    check('the GND via offsets use the pair class clearance',
          abs(clr - CLASS) < 1e-9 and abs(along - (VIA + CLASS)) < 1e-9
          and abs(perp - (0.2 + TRACK / 2 + CLASS + VIA / 2)) < 1e-9,
          f'perp {perp:.3f} along {along:.3f} clearance {clr:.3f}')

    with tempfile.TemporaryDirectory(prefix='t1207_') as d:
        src, out = os.path.join(d, 'b.kicad_pcb'), os.path.join(d, 'out.kicad_pcb')
        open(src, 'w').write(BOARD)
        json.dump(PROJECT, open(os.path.join(d, 'b.kicad_pro'), 'w'), indent=2)
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', 'py_router/route_diff.py', src,
             '--nets', '/D_*', '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu',
             '--track-width', str(TRACK), '--diff-pair-gap', '0.2',
             '--clearance', str(BASE), '--via-size', str(VIA), '--via-drill', '0.3',
             '--output', out],
            cwd=ROOT, capture_output=True, text=True, encoding='utf-8',
            errors='replace')
        log = r.stdout + r.stderr
        check('route_diff routes the pair', 'Diff pairs:    1/1 routed' in log,
              log[-400:] if 'Diff pairs:    1/1 routed' not in log else '')
        if not os.path.isfile(out):
            print('FAILED: no output board')
            return 1
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(out)
        gnd = [v for v in pcb.vias if v.net_id == 1]
        pair = [s for s in pcb.segments if s.net_id in (2, 3)]
        check('precondition: the pair changed layers and GND vias were placed',
              len(gnd) >= 2 and len({s.layer for s in pair}) >= 2,
              f'{len(gnd)} GND vias, layers {sorted({s.layer for s in pair})}')
        worst = min((point_to_segment_distance_seg(v.x, v.y, s)
                     - v.size / 2 - s.width / 2, (v.x, v.y), s.layer)
                    for v in gnd for s in pair) if gnd and pair else None
        check('every GND via clears the drawn pair tracks at the class clearance',
              worst is not None and worst[0] >= CLASS - 1e-4,
              f'tightest gap {worst[0]:.4f} at {worst[1]} on {worst[2]}' if worst else '')

        r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/check_drc.py',
                            out, '--clearance-margin', '0.1'],
                           cwd=ROOT, capture_output=True, text=True,
                           encoding='utf-8', errors='replace')
        bad = [l.strip() for l in r.stdout.splitlines() if 'Via:GND' in l]
        check('check_drc reports no GND <-> pair violation', not bad,
              '; '.join(bad[:4]))

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

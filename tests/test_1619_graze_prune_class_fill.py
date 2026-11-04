#!/usr/bin/env python3
"""#1619: the graze prune keeps a pad's only stub beside a high-clearance class.

The board: a 0.5 mm pitch pad row whose middle pad is GND and whose other
pads are in net class HS (0.5 mm); GND pours on F.Cu and In1.Cu at a 0.1 mm
zone clearance. The GND pad's fanout stub runs straight out of the row to a
via in the flood. KiCad's refill keeps the fill max(zone clearance, class)
from every HS pad, so it never reaches the GND pad: the stub is the pad's
only connection. A .kicad_dru lets HS pads come within 0.1 mm of a track, so
check_drc passes the stub.

route.py's cleanup holds the class map but does not publish it on the
board, so the prune's trusted fill model (#1581) was carved at the zone's
0.1 mm alone, poured onto the pad and graded the removal free; and its
graze test priced the HS pads at the class, not the rule, so the legal stub
was a candidate at all.

Checks (prune_grazing_segments called as cleanup_pipeline calls it: the
class map passed, never published):
  1. With the rule: the prune keeps the stub (base: removed it).
  2. Without the rule the stub IS under the class clearance, and the prune
     still keeps it, as its contract says a load-bearing graze is kept and
     left for DRC (base: removed it).
  3. With the rule and a second, redundant stub out of the other side of
     the pad: without `config` the prune removes one of the two (it prices
     the HS pads at their class), with `config` it removes neither -- a stub
     the rule allows is not a candidate.

    python3 tests/test_1619_graze_prune_class_fill.py
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import evidence                                  # noqa: E402

failures = []
NAMES = ['GND', '/HS1', '/HS2', '/HS3', '/HS4', '/X']


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def build(path, dru, second_stub=False):
    def fp(ref, x, y, pads):
        p = ''.join(f'    (pad "{n}" smd rect (at {px} 0) (size {sx} {sy}) (layers "F.Cu") '
                    f'(net {nid} "{nm}"))\n' for n, px, sx, sy, nid, nm in pads)
        return (f'  (footprint "T:{ref}" (layer "F.Cu") (at {x} {y})\n'
                f'    (property "Reference" "{ref}" (at 0 -2 0) (layer "F.SilkS") (hide yes) '
                f'(effects (font (size 1 1) (thickness 0.15))))\n    (attr smd)\n{p}  )\n')

    def zone(la):
        return (f'  (zone (net 1) (net_name "GND") (layer "{la}") (hatch edge 0.5) '
                f'(connect_pads yes (clearance 0.1))\n    (min_thickness 0.1) '
                f'(filled_areas_thickness no) (fill yes (thermal_gap 0.2) (thermal_bridge_width 0.2))\n'
                f'    (polygon (pts (xy 100.5 100.5) (xy 139.5 100.5) (xy 139.5 129.5) (xy 100.5 129.5))))\n')
    b = ('(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")\n'
         ' (general (thickness 1.6))\n (paper "A4")\n'
         ' (layers (0 "F.Cu" signal) (4 "In1.Cu" signal) (6 "In2.Cu" signal) (2 "B.Cu" signal) '
         '(25 "Edge.Cuts" user))\n (setup (pad_to_mask_clearance 0))\n (net 0 "")\n')
    b += ''.join(f' (net {i + 1} "{n}")\n' for i, n in enumerate(NAMES))
    b += fp('U1', 120, 115, [('1', -1, .25, 1, 2, '/HS1'), ('2', -.5, .25, 1, 3, '/HS2'),
                             ('3', 0, .25, 1, 1, 'GND'), ('4', .5, .25, 1, 4, '/HS3'),
                             ('5', 1, .25, 1, 5, '/HS4')])
    b += ('  (segment (start 120 115) (end 120 116.8) (width 0.15) (layer "F.Cu") (net 1))\n'
          '  (via (at 120 116.8) (size 0.45) (drill 0.2) (layers "F.Cu" "B.Cu") (net 1))\n')
    if second_stub:
        b += ('  (segment (start 120 115) (end 120 113.2) (width 0.15) (layer "F.Cu") (net 1))\n'
              '  (via (at 120 113.2) (size 0.45) (drill 0.2) (layers "F.Cu" "B.Cu") (net 1))\n')
    b += fp('G2', 108, 104, [('1', 0, 1, 1, 1, 'GND')])
    b += ('  (segment (start 108 104) (end 110 104) (width 0.15) (layer "F.Cu") (net 1))\n'
          '  (via (at 110 104) (size 0.45) (drill 0.2) (layers "F.Cu" "B.Cu") (net 1))\n')
    b += fp('X1', 104, 126, [('1', 0, .5, .5, 6, '/X')]) + fp('X2', 112, 126, [('1', 0, .5, .5, 6, '/X')])
    b += zone('F.Cu') + zone('In1.Cu')
    for x1, y1, x2, y2 in ((100, 100, 140, 100), (140, 100, 140, 130),
                           (140, 130, 100, 130), (100, 130, 100, 100)):
        b += (f'  (gr_line (start {x1} {y1}) (end {x2} {y2}) (layer "Edge.Cuts") '
              f'(stroke (width 0.05) (type solid)))\n')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(b + ')\n')

    def cl(n, c):
        return dict(name=n, clearance=c, track_width=0.15, via_diameter=0.45, via_drill=0.2,
                    wire_width=6, bus_width=12, line_style=0,
                    priority=(2147483647 if n == 'Default' else 0))
    with open(path[:-len('kicad_pcb')] + 'kicad_pro', 'w', encoding='utf-8') as fh:
        json.dump({'board': {'design_settings': {'rules': {'min_clearance': 0.1,
                                                           'min_hole_clearance': 0.1}}},
                   'meta': {'filename': os.path.basename(path)[:-len('kicad_pcb')] + 'kicad_pro',
                            'version': 3},
                   'net_settings': {'classes': [cl('Default', 0.1), cl('HS', 0.5)],
                                    'meta': {'version': 4},
                                    'netclass_patterns': [{'netclass': 'HS', 'pattern': '/HS*'}]}},
                  fh, indent=1)
    if dru:
        with open(path[:-len('kicad_pcb')] + 'kicad_dru', 'w', encoding='utf-8') as fh:
            fh.write('(version 1)\n(rule "hs track"\n'
                     '  (condition "A.NetClass == \'HS\' && B.Type == \'Track\'")\n'
                     '  (constraint clearance (min 0.1mm)))\n(rule "hs pad"\n'
                     '  (condition "A.NetClass == \'HS\' && B.Type == \'Pad\'")\n'
                     '  (constraint clearance (min 0.2mm)))\n')
    evidence(path, 'synthetic input board')


def prune(path, with_config):
    """(segments removed, GND segments left), as cleanup_pipeline calls it."""
    from kicad_parser import parse_kicad_pcb, canonicalize_pcb_data_order
    from list_nets import net_clearance_map_by_id
    from pcb_modification import prune_grazing_segments
    pcb = parse_kicad_pcb(path)
    canonicalize_pcb_data_order(pcb)
    gnd = next(i for i, n in pcb.nets.items() if n.name == 'GND')
    nc = net_clearance_map_by_id(path, {i: n.name for i, n in pcb.nets.items()})
    cfg = None
    if with_config:
        from routing_config import GridRouteConfig
        from kicad_dru import install_track_clearances
        cfg = GridRouteConfig()
        cfg.clearance = 0.1
        cfg.layers = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
        routed = [i for i, n in pcb.nets.items() if n.name in ('GND', '/X')]
        cfg.set_net_clearances(nc, routed)
        with contextlib.redirect_stdout(io.StringIO()):
            install_track_clearances(cfg, None, path, pcb, routed_net_ids=routed)
        if cfg.scoped_clearance is None:
            print("  [BROKEN TEST] the board's scoped rules were not installed")
            sys.exit(1)
    with contextlib.redirect_stdout(io.StringIO()):
        n = prune_grazing_segments([], pcb, {gnd}, clearance=0.1,
                                   check_foreign_segments=True,
                                   net_clearances=nc, config=cfg)[0]
    return n, sum(1 for s in pcb.segments if s.net_id == gnd)


def main():
    tmp = tempfile.mkdtemp(prefix='t1619_')
    try:
        print("1. with the rule: the stub is the pad's only connection")
        p = os.path.join(tmp, 'rule.kicad_pcb')
        build(p, dru=True)
        n, left = prune(p, with_config=False)
        check("the prune keeps it", n == 0 and left == 2, f"removed {n}, GND segments {left}")

        print("2. no rule: the stub is under the class clearance, still load-bearing")
        p = os.path.join(tmp, 'norule.kicad_pcb')
        build(p, dru=False)
        n, left = prune(p, with_config=False)
        check("the prune keeps it, for DRC", n == 0 and left == 2,
              f"removed {n}, GND segments {left}")

        print("3. with the rule and a second, redundant stub")
        p = os.path.join(tmp, 'two.kicad_pcb')
        build(p, dru=True, second_stub=True)
        n, left = prune(p, with_config=False)
        if n != 1:
            print(f"  [BROKEN TEST] without config the prune removed {n}, not one of the "
                  f"two stubs: this board does not reach the graze candidacy")
            return 1
        print("  [ok] without config one stub is removed (the class prices the HS pads)")
        n, left = prune(p, with_config=True)
        check("with config neither is a candidate", n == 0 and left == 3,
              f"removed {n}, GND segments {left}")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""#1969: a track joins a via where its COPPER reaches the via's copper.

`net_queries._net_track_graph` (under `net_pads_joined`, `pin_pair_path_length`,
`net_signal_length` and `net_match_length`) joined a track to a via only where
the track's CENTRELINE came within the via radius (+ the 0.02 mm tolerance).
check_connected and KiCad join at via radius + track half width, where the
copper touches. A net bridged by a via that both tracks only overlap read "not
joined" (every length None), and a lane that comes back past its own via
inside that band was measured at its drawn length.

Boards (0.14 mm tracks, a 0.46 mm via; the copper touches at 0.30 mm):
  bridge        two tracks 0.508 mm apart joined only by the via between them
                (each centreline 0.254 mm from the via centre)
  apart         control: the via 0.5 mm further off
  gap           control: the tracks 0.62 mm apart, the via between them (each
                centreline 0.31 mm off, the copper 0.01 mm short of the via's)
  loop          a lane that runs 2 mm past the via and comes back beside it
  loop_bridged  the same lane with the 0.508 mm short cut drawn as a track
  through       control: a 0.6 mm via, both legs' centrelines inside its radius

    python3 tests/test_1969_via_ring_join.py
"""
import contextlib
import io
import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb                       # noqa: E402
from net_queries import (net_match_length, net_pads_joined,    # noqa: E402
                         net_signal_length, pin_pair_path_length)
from check_connected import check_net_connectivity             # noqa: E402
from run_utils import evidence                                 # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def seg(x1, y1, x2, y2):
    return (f'  (segment (start {x1} {y1}) (end {x2} {y2}) (width 0.14) '
            f'(layer "F.Cu") (net 1))\n')


def pad(ref, x, y):
    return (f'  (footprint "T:{ref}" (layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 -1 0) (layer "F.SilkS") (hide yes) '
            f'(effects (font (size 1 1) (thickness 0.15))))\n    (attr smd)\n'
            f'    (pad "1" smd circle (at 0 0) (size 0.4 0.4) (layers "F.Cu") '
            f'(net 1 "/S"))\n  )\n')


def via(x, y, size=0.46):
    return (f'  (via (at {x} {y}) (size {size}) (drill 0.2) (layers "F.Cu" "B.Cu") '
            f'(net 1))\n')


def loop(size=0.46):
    """A lane out 2 mm past the via at (110, 105.254) and back beside it."""
    return (pad('J1', 105, 105) + pad('J2', 115, 107) + seg(105, 105, 112, 105)
            + seg(112, 105, 112, 105.508) + seg(112, 105.508, 108, 105.508)
            + seg(108, 105.508, 108, 107) + seg(108, 107, 115, 107)
            + via(110, 105.254, size))


def boards():
    two = (pad('J1', 105, 105) + pad('J2', 115, 105.508) + seg(105, 105, 112, 105)
           + seg(108, 105.508, 115, 105.508))
    wide = (pad('J1', 105, 105) + pad('J2', 115, 105.62) + seg(105, 105, 112, 105)
            + seg(108, 105.62, 115, 105.62))
    return {
        'bridge': two + via(110, 105.254),
        'apart': two + via(110, 105.754),
        'gap': wide + via(110, 105.31),
        'loop': loop(),
        'loop_bridged': loop() + seg(110, 105, 110, 105.508),
        # a 0.6 mm via: both legs' centrelines inside its radius
        'through': loop(0.6),
    }


def write(path, items):
    head = ('(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")\n'
            ' (general (thickness 1.6))\n (paper "A4")\n'
            ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))\n'
            ' (setup (pad_to_mask_clearance 0))\n (net 0 "")\n (net 1 "/S")\n')
    with open(path, 'w') as fh:
        fh.write(head + items + ')\n')


def load(path):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(path)
    a, b = pcb.pads_by_net[1]
    return pcb, a, b


def connected(pcb):
    r = check_net_connectivity(1, [s for s in pcb.segments if s.net_id == 1],
                               [v for v in pcb.vias if v.net_id == 1],
                               pcb.pads_by_net[1])
    return r.get('connected')


def near(v, want):
    return v is not None and abs(v - want) < 1e-6


def main():
    with tempfile.TemporaryDirectory() as tmp:
        pcbs = {}
        for name, items in boards().items():
            path = os.path.join(tmp, f'{name}.kicad_pcb')
            write(path, items)
            evidence(path, f'the {name} board')
            pcbs[name] = load(path)

        pcb, a, b = pcbs['bridge']
        check('check_connected joins the bridge (the reference)',
              connected(pcb) is True)
        check('net_pads_joined joins the bridge, as check_connected does',
              net_pads_joined(pcb, 1) is True)
        pp = pin_pair_path_length(pcb, 1, a, b)
        # 7 + 7 of track less the 4 mm the two tracks overlap past the via,
        # plus the two 0.254 mm hops to the via centre
        check('the bridge has a pin-pair length (10.508 mm)', near(pp, 10.508), str(pp))
        check('net_signal_length measures the bridge',
              near(net_signal_length(pcb, 1), 10.508), str(net_signal_length(pcb, 1)))
        check('net_match_length(total) measures the bridge (14.0 mm of copper)',
              near(net_match_length(pcb, 1, 'total'), 14.0),
              str(net_match_length(pcb, 1, 'total')))

        for name, why in (('apart', 'the via 0.5 mm further off'),
                          ('gap', 'copper 0.01 mm short of the via')):
            pcb, a, b = pcbs[name]
            check(f'control: {why}: check_connected does not join', connected(pcb) is False)
            check(f'control: {why}: net_pads_joined does not join either',
                  net_pads_joined(pcb, 1) is False)
            check(f'control: {why}: no pin-pair length',
                  pin_pair_path_length(pcb, 1, a, b) is None)

        pcb, a, b = pcbs['loop']
        lp = pin_pair_path_length(pcb, 1, a, b)
        check('the loop is walked through the via its return leg overlaps (16.0 mm)',
              near(lp, 16.0), str(lp))
        check('net_signal_length reads the loop at 16.0 mm',
              near(net_signal_length(pcb, 1), 16.0), str(net_signal_length(pcb, 1)))
        pcb, a, b = pcbs['loop_bridged']
        bp = pin_pair_path_length(pcb, 1, a, b)
        check('the same short cut drawn as a track reads the same 16.0 mm',
              near(bp, 16.0) and near(lp, bp), f'{bp} vs {lp}')

        pcb, a, b = pcbs['through']
        tp = pin_pair_path_length(pcb, 1, a, b)
        # each leg cut where its centreline crosses the via's edge and charged
        # straight to the centre from there (#1583); the same before #1969
        check('control: legs whose centrelines are inside the via radius read '
              'as before (15.7727 mm)', near(tp, 15.772726), str(tp))

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""A pour no copper of its net touches is tapped from the pads over it (#2020).

Since #1777 a terminal covers a fill region only on a layer it has copper on,
so a split under SMD parts -- In1.Cu GNDA under two F.Cu 0805 caps, the net's
one through-hole pin outside it -- covered no region at all, the pour-launch
ladder offered none of its fill, and the route joined the three pads with
40.9 mm of F.Cu track. KiCad then kept the untouched zone's fill and
reported it as isolated copper. The route step now taps one pad of each
copper group over such a region into it first (`_tap_bare_pours`, the plane
tap the plane finalize uses), and the ladder offers the region's fill as
that group's copper.

  1. `_bare_pour_regions`: GNDA's In1.Cu region, with C3.2 and C4.2 over it;
     nothing for GND (its zone holds J1.2's barrel), nor for GNDA once a via
     of the net stands in its zone;
  2. the issue's route (3 pads, multipoint): both caps tapped, J1.3 reaches
     the plane, the net is connected, its zone is touched, and its F.Cu track
     is a fraction of the 40.9 mm surface route;
  3. the same board with J1.3 on another net (2 pads): the two taps are the
     route -- the net is connected through the pour with no track.

The board is the #2020 attachment (`tests/fixtures/2020`), saved by pcbnew.

    python3 tests/test_2020_bare_pour_tap.py
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

from kicad_parser import parse_kicad_pcb, Via                  # noqa: E402
from check_connected import check_net_connectivity             # noqa: E402
from single_ended_routing import _bare_pour_regions            # noqa: E402
import run_utils                                               # noqa: E402

BOARD = os.path.join(TESTS_DIR, 'fixtures', '2020', 'split.kicad_pcb')
LAYERS = ['F.Cu', 'In1.Cu']
FAB_FLOOR = ("track_width  = 0.25\nclearance    = 0.2\nvia_diameter = 0.6\n"
             "via_drill    = 0.3\nhole_to_hole = 0.25\nboard_edge   = 0.3\n")
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def load(path):
    run_utils.evidence(path, 'a #2020 board')
    with contextlib.redirect_stdout(io.StringIO()):
        return parse_kicad_pcb(path)


def net(pcb, name):
    return next(nid for nid, n in pcb.nets.items() if n.name == name)


def connected(pcb, nid):
    return check_net_connectivity(
        nid, [s for s in pcb.segments if s.net_id == nid],
        [v for v in pcb.vias if v.net_id == nid],
        pcb.pads_by_net.get(nid, []),
        [z for z in pcb.zones if z.net_id == nid], pcb_data=pcb)['connected']


def regions():
    print("1. _bare_pour_regions")
    pcb = load(BOARD)
    gnda = net(pcb, 'GNDA')
    with contextlib.redirect_stdout(io.StringIO()):
        got = _bare_pour_regions(pcb, gnda, LAYERS)
    pads = [[f"{p.component_ref}.{p.pad_number}" for p in r[3]] for r in got]
    check("GNDA: one In1.Cu region, C3.2 and C4.2 over it",
          len(got) == 1 and got[0][0].layer == 'In1.Cu'
          and pads == [['C3.2', 'C4.2']], str(pads))
    with contextlib.redirect_stdout(io.StringIO()):
        gnd = _bare_pour_regions(pcb, net(pcb, 'GND'), LAYERS)
    check("GND: none (J1.2's barrel stands in its zone)", gnd == [], str(gnd))
    pcb.vias.append(Via(x=40.0, y=20.0, size=0.6, drill=0.3,
                        layers=['F.Cu', 'B.Cu'], net_id=gnda))
    with contextlib.redirect_stdout(io.StringIO()):
        touched = _bare_pour_regions(pcb, gnda, LAYERS)
    check("GNDA with a via of its net in the zone: none", touched == [],
          str(touched))


def route(src, out, tmp):
    fab = os.path.join(tmp, 'fab-floor.txt')
    with open(fab, 'w', encoding='utf-8') as f:
        f.write(FAB_FLOOR)
    _rc, log = run_utils.run_step(
        [run_utils.tool('route.py'), src, out, '--nets', 'GNDA',
         '--layers', *LAYERS, '--track-width', '0.5', '--clearance', '0.2',
         '--via-size', '0.6', '--via-drill', '0.3', '--fab-overrides', fab,
         '--escalation', 'off', '--power-nets', 'GNDA',
         '--power-nets-widths', '0.5'], outputs=[out])
    return log, load(out)


def graded(label, log, pcb, max_track_mm):
    gnda = net(pcb, 'GNDA')
    taps = [l for l in log.splitlines() if 'POUR-TAP (#2020)' in l]
    check(f"{label}: C3.2 and C4.2 tapped",
          len(taps) == 2 and 'C3.2' in taps[0] and 'C4.2' in taps[1],
          f"{len(taps)} tap line(s)")
    check(f"{label}: GNDA connected (fill-aware)", connected(pcb, gnda))
    with contextlib.redirect_stdout(io.StringIO()):
        left = _bare_pour_regions(pcb, gnda, LAYERS)
    check(f"{label}: its In1.Cu pour is touched", left == [], str(left))
    track = sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
                for s in pcb.segments if s.net_id == gnda)
    check(f"{label}: {track:.2f} mm of track <= {max_track_mm} mm",
          track <= max_track_mm)


def main():
    run_utils.evidence(BOARD, 'the #2020 board')
    regions()
    with tempfile.TemporaryDirectory() as tmp:
        print("2. the issue's route (3 pads)")
        log, pcb = route(BOARD, os.path.join(tmp, 'three.kicad_pcb'), tmp)
        # J1.3 is ~8 mm from the pour's edge; the surface route was 40.9 mm
        graded('3 pads', log, pcb, 15.0)
        print("3. J1.3 moved to VA (2 pads)")
        with open(BOARD, encoding='utf-8') as f:
            text = f.read()
        j1 = text.index('(pad "3" thru_hole', text.index('PinHeader_1x04'))
        k = text.index('(net "GNDA")', j1)
        two = os.path.join(tmp, 'two_in.kicad_pcb')
        with open(two, 'w', encoding='utf-8') as f:
            f.write(text[:k] + '(net "VA")' + text[k + len('(net "GNDA")'):])
        log, pcb = route(two, os.path.join(tmp, 'two.kicad_pcb'), tmp)
        graded('2 pads', log, pcb, 0.0)
    if failures:
        print(f"\n{len(failures)} check(s) FAILED")
        sys.exit(1)
    print("\nall checks passed")


if __name__ == '__main__':
    main()

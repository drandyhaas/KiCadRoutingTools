#!/usr/bin/env python3
"""#1620 / #1621: which nets the plane repair's blocker rip-up may take.

The plane repair (route.py's in-run plane finalize, repair_planes) rips a
net that blocks a plane tap. Its never-rip set held the nets THIS call
repairs plus protected and locked nets, so:

  #1620  another POURED net was a blocker like any signal: its fanout stub
         was ripped whole, and nothing re-laid it -- the reconnect saw its
         pads credited by the fill model ("Already fully connected").
  #1621  a net the route step's --nets negated ('!/RF') was ripped too, and
         came back on another layer with vias its rule forbids, or not at all.

The board (as #1261's): pad G1 on F.Cu boxed by a ring of a blocker net, a
VCC pour on B.Cu, and a same-net pad clearance that keeps the tap via off
the pad, so tapping G1 needs the blocker ripped. The engine is called
in-process, repairing VCC only.

Checks:
  1. Control: the blocker is a plain signal net -- it is ripped (the fixture
     reaches the blocker rip).
  2. #1620: the blocker pours (a GND zone on B.Cu that this call does not
     repair) -- excluded as a poured net, not ripped, its copper intact.
  3. #1621: the blocker is a plain net the caller hands off
     (hands_off_nets) -- excluded, not ripped.
  4. #1621: the same, the set only STASHED on the board by the route step
     (protected_nets.stash_hands_off), as the live finalize shares it.
  5. negated_net_names reads '!'-patterns as the step's selection does: an
     exclusion names its nets; an active-low literal that names a net does
     not negate it.

    python3 tests/test_1620_1621_finalize_rip_scope.py
"""
import contextlib
import io
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import evidence                                  # noqa: E402
from synth import RoutingBoard                                  # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def zone(b, net, layer, uid):
    return ('  (zone (net %d) (net_name "%s") (layer "%s") (uuid "%s") '
            '(hatch edge 0.5) (connect_pads (clearance 0.25)) (min_thickness 0.2) '
            '(filled_areas_thickness no) (fill yes (thermal_gap 0.3) '
            '(thermal_bridge_width 0.3)) (polygon (pts (xy 1 1) (xy 39 1) '
            '(xy 39 29) (xy 1 29))))' % (b.net(net), net, layer, uid))


def board(blocker, blocker_pours):
    b = RoutingBoard(layers=('F.Cu', 'In1.Cu', 'B.Cu'))
    b.pad('G1', 20, 15, 'VCC', sx=1.0, sy=1.0)
    b.pad('G2', 5, 5, 'VCC', sx=1.6, sy=1.6, tht=True)
    b.pad('S1', 30, 25, blocker, sx=1.0, sy=1.0)
    b.pad('S2', 32, 25, blocker, sx=1.0, sy=1.0)
    r = 0.95
    b.ring([(20 - r, 15 - r), (20 + r, 15 - r), (20 + r, 15 + r), (20 - r, 15 + r)],
           blocker, w=0.3)
    b.seg(20 + r, 15 + r, 30, 25, blocker)
    b.seg(30, 25, 32, 25, blocker)
    b.items.append(zone(b, 'VCC', 'B.Cu', 'synth-zone-vcc'))
    if blocker_pours:
        b.items.append(zone(b, blocker, 'In1.Cu', 'synth-zone-blk'))
    return b.text()


def run(tmp, tag, blocker, blocker_pours=False, hands_off=None, stash=None):
    from kicad_parser import parse_kicad_pcb
    from protected_nets import stash_hands_off
    from repair_planes import repair_planes
    inp = os.path.join(tmp, f'{tag}_in.kicad_pcb')
    out = os.path.join(tmp, f'{tag}_out.kicad_pcb')
    with open(inp, 'w', encoding='utf-8') as fh:
        fh.write(board(blocker, blocker_pours))
    evidence(inp, 'synthetic input board')
    pcb = parse_kicad_pcb(inp)
    if stash:
        stash_hands_off(pcb, stash)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        repair_planes(inp, out, net_names=['VCC'], plane_layers=['B.Cu'],
                      rip_blocker_nets=True, same_net_pad_clearance=0.2,
                      routing_layers=['F.Cu', 'B.Cu'], pcb_data=pcb,
                      hands_off_nets=hands_off)
    evidence(out, 'repaired output')
    out_pcb = parse_kicad_pcb(out)
    bid = next(i for i, n in out_pcb.nets.items() if n.name == blocker)
    return buf.getvalue(), sum(1 for s in out_pcb.segments if s.net_id == bid)


def main():
    tmp = tempfile.mkdtemp(prefix='t1620_')
    try:
        print("1. control: the blocker is a plain signal net")
        con, _ = run(tmp, 'control', 'NET_S')
        if 'blocked by NET_S - ripping' not in con:
            print("  [BROKEN TEST] the control never ripped NET_S: this board "
                  "does not reach the blocker rip")
            return 1
        print("  [ok] NET_S is ripped to tap G1")

        print("2. #1620: the blocker pours on a layer this call does not repair")
        con, segs = run(tmp, 'pour', 'GND', blocker_pours=True)
        check("GND is excluded as a poured net",
              'other poured net(s) excluded from blocker rip-up (GND)' in con)
        check("and not ripped", 'blocked by GND - ripping' not in con)
        check("its copper is intact", segs == 6, f"{segs} GND segment(s), 6 in")

        print("3. #1621: the blocker is a net the caller hands off")
        con, segs = run(tmp, 'off', 'NET_S', hands_off=['NET_S'])
        check("NET_S is excluded as handed off",
              "net(s) the step's --nets excludes kept out of blocker rip-up (NET_S)"
              in con)
        check("and not ripped", 'blocked by NET_S - ripping' not in con)
        check("its copper is intact", segs == 6, f"{segs} NET_S segment(s), 6 in")

        print("4. #1621: the set only stashed on the shared board")
        con, segs = run(tmp, 'stash', 'NET_S', stash=['NET_S'])
        check("not ripped", 'blocked by NET_S - ripping' not in con and segs == 6,
              f"{segs} NET_S segment(s)")

        print("5. negated_net_names")
        from protected_nets import negated_net_names
        names = ['/RF', '/RF2', 'GND', '!RESET', 'RESET', '/X']
        check("'!/RF*' negates both RF nets",
              negated_net_names(['*', '!/RF*'], names) == {'/RF', '/RF2'})
        check("an active-low literal that names a net negates nothing",
              negated_net_names(['!RESET'], names) == set())
        check("no pattern, no negation", negated_net_names(None, names) == set())
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

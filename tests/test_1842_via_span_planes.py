#!/usr/bin/env python3
"""#1842: the plane taps of a via-span step are vias of the span.

The plane repair engine (`repair_planes.py`, and the route step's in-run
finalize, which calls it) laid through vias only, and "this site reaches the
plane" was read off every layer of the stack. A route step on a partial span
therefore skipped the finalize. With `--via-span TOP BOTTOM` the taps and
region joins are vias of the span, a pour on a layer the span does not have
is not tapped, and for the call the board carries only the copper the span
has, as in the fan-out.

The fixture (tests/fixtures/1842/plane_under) is synthetic: the fan-out
fixture's 8-layer board with a second field of bottom-side pads between the
first, so no through via fits under the BGA; four inner balls and four
connector pads on a net `VDD`; and a `VDD` pour on In1.Cu.

Checks:
  1. through vias: `repair_planes.py` taps the connector pads and none of
     the balls (3 of 7 pads reach the plane by a tap; the balls are joined to
     the connector by surface tracks instead), with no via under the BGA;
  2. `--via-span F.Cu In1.Cu`: all 7 pads are tapped, with micro vias
     F.Cu-In1.Cu at the class's micro via size, at the balls; check_drc is
     clean, the net is connected, and the project's via and drilled-hole
     minimums are left alone. kicad-cli 10.0.8 grades that board at 0
     violations with the net connected;
  3. a plane on a layer the span does not have is refused by name;
  4. the route step under the same span runs its plane finalize (it used to
     skip it), lays no through via, and leaves the net connected.
"""
import json
import os
import re
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FIX = os.path.join(TESTS, 'fixtures', '1842', 'plane_under')
BGA = (6.0, 6.0, 14.0, 14.0)
KEYS = ('min_via_diameter', 'min_through_hole_diameter', 'min_microvia_diameter',
        'min_microvia_drill')
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def stage(td, tag):
    board = os.path.join(td, tag + '_in.kicad_pcb')
    for ext in ('.kicad_pcb', '.kicad_pro'):
        shutil.copy(evidence(FIX + ext, 'the fixture'), os.path.splitext(board)[0] + ext)
    return board, os.path.join(td, tag + '.kicad_pcb')


def repair(td, tag, *extra, **kw):
    board, out = stage(td, tag)
    r = check(PY + [os.path.join(REPO, 'py_router', 'repair_planes.py'), board, out,
                    '--nets', 'VDD', '--plane-layers', 'In1.Cu', '--clearance', '0.1',
                    '--track-width', '0.1', *extra], timeout=1800, **kw)
    log = (r.stdout or '') + (r.stderr or '')
    m = re.findall(r'Pad repair: (\d+)/(\d+) unconnected', log)
    return out, log, (int(m[-1][0]), int(m[-1][1])) if m else None


def under_bga(v):
    return BGA[0] < v.x < BGA[2] and BGA[1] < v.y < BGA[3]


def main():
    with open(FIX + '.kicad_pro', encoding='utf-8') as fh:
        own = json.load(fh)['board']['design_settings']['rules']
    with tempfile.TemporaryDirectory() as td:
        print("1. through vias")
        out, _log, got = repair(td, 'through', accept=True)
        pcb = parse_kicad_pcb(evidence(out, 'the repaired board'))
        ok(got == (3, 7), f"3 of 7 pads are tapped: {got}")
        ok(pcb.vias and all(v.via_type == 'through' for v in pcb.vias)
           and not any(under_bga(v) for v in pcb.vias),
           f"with through vias, none under the BGA: "
           f"{[(round(v.x, 2), round(v.y, 2)) for v in pcb.vias]}")

        print("2. a micro via span")
        out, log, got = repair(td, 'span', '--via-span', 'F.Cu', 'In1.Cu', accept=True)
        pcb = parse_kicad_pcb(evidence(out, 'the repaired board'))
        ok(got == (7, 7), f"all 7 pads are tapped: {got}")
        kinds = sorted({(v.via_type, tuple(v.layers), round(v.size, 4), round(v.drill, 4))
                        for v in pcb.vias})
        ok(kinds == [('micro', ('F.Cu', 'In1.Cu'), 0.25, 0.1)],
           f"with micro vias at the class's micro via size: {kinds} x{len(pcb.vias)}")
        ok(any(under_bga(v) for v in pcb.vias), "at the balls, under the BGA")
        ok(sorted({s.layer for s in pcb.segments}) in (['F.Cu'], ['F.Cu', 'In1.Cu']),
           f"copper on the span only: {sorted({s.layer for s in pcb.segments})}")
        d = check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), out,
                        '--clearance-margin-mm', '0.0005'], accept=True)
        ok('NO DRC VIOLATIONS FOUND' in (d.stdout or ''), "check_drc is clean")
        check(PY + [os.path.join(REPO, 'py_router', 'check_connected.py'), out,
                    '--nets', 'VDD'], accept=True)
        ok(True, "the net is connected")
        with open(os.path.splitext(out)[0] + '.kicad_pro', encoding='utf-8') as fh:
            rules = json.load(fh)['board']['design_settings']['rules']
        ok(all(rules.get(k) == own.get(k) for k in KEYS),
           f"the project's via and hole minimums are the board's own: "
           f"{ {k: rules.get(k) for k in KEYS} }")
        ok(re.search(r'Via span: \d+ plane via\(s\) written as micro F.Cu-In1.Cu', log)
           is not None, "and the run says what it wrote")

        print("3. a plane outside the span")
        repair(td, 'bad', '--via-span', 'In2.Cu', 'In6.Cu',
               refuse='are outside the span')
        ok(True, "is refused by name")

        print("4. the route step's plane finalize under the span")
        board, out = stage(td, 'route')
        r = check(PY + [os.path.join(REPO, 'py_router', 'route.py'), board, out,
                        '--nets', 'VDD', '--via-span', 'F.Cu', 'In1.Cu'],
                  accept=True, timeout=1800)
        log = (r.stdout or '') + (r.stderr or '')
        pcb = parse_kicad_pcb(evidence(out, 'the routed board'))
        ok('Plane finalize (#562)' in log and 'Plane finalize not run' not in log,
           "the finalize runs")
        ok(not any(v.via_type == 'through' for v in pcb.vias),
           f"no through via is laid: {sorted({v.via_type for v in pcb.vias})}")
        check(PY + [os.path.join(REPO, 'py_router', 'check_connected.py'), out,
                    '--nets', 'VDD'], accept=True)
        ok(True, "and the net is connected")
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())

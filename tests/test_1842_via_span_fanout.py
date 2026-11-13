#!/usr/bin/env python3
"""#1842: bga_fanout.py --via-span escapes a BGA on one sub-stack.

The fan-out laid through vias only, and its engines measure each via site
off the board with the barrel taken to span the stack: a pad on the far side
of the board under a ball blocked that ball's via. `--via-span TOP BOTTOM`
has route.py's meaning: the part escapes on that run of neighbouring copper
layers, its vias join exactly it, and for the call the board carries only
the copper the span has, so every engine's site test is the span's.

The fixture (tests/fixtures/1842/bga_under) is synthetic: the 8-layer board
of the route step's fixture with the bottom-side part moved so its pads sit
directly under the BGA's balls. 36 balls carry nets; the 12 inner ones need a
via.

Checks:
  1. with through vias the fan-out escapes 24 of 36 (no via site is legal
     over a bottom-side pad of another net);
  2. with `--via-span F.Cu In1.Cu` it escapes all 36, with 12 micro vias
     F.Cu-In1.Cu at the Default class's micro via size, copper on F.Cu and
     In1.Cu only, check_drc clean, and the project's via and drilled-hole
     minimums left alone. kicad-cli 10.0.8 grades that board at 0 violations;
  3. the board is as it was after the call: `only_span_copper` puts back
     every pad, track, via and pour it set aside, in place;
  4. a part that is on neither end of the span is refused by name.
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
FIX = os.path.join(TESTS, 'fixtures', '1842', 'bga_under')
KEYS = ('min_via_diameter', 'min_through_hole_diameter', 'min_microvia_diameter',
        'min_microvia_drill')
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def fan(td, tag, *extra):
    board = os.path.join(td, tag + '_in.kicad_pcb')
    for ext in ('.kicad_pcb', '.kicad_pro'):
        shutil.copy(evidence(FIX + ext, 'the fixture'), os.path.splitext(board)[0] + ext)
    out = os.path.join(td, tag + '.kicad_pcb')
    r = check(PY + [os.path.join(REPO, 'py_router', 'bga_fanout.py'), board, '-o', out,
                    '--component', 'U1', '--clearance', '0.1', '--track-width', '0.1',
                    *extra], accept=True, timeout=1800)
    log = (r.stdout or '') + (r.stderr or '')
    m = re.findall(r'"escaped": (\d+), "failed": (\d+)', log)
    return out, log, (int(m[-1][0]), int(m[-1][1])) if m else None


def restored():
    print("3. the board after the call")
    from via_span import only_span_copper
    pcb = parse_kicad_pcb(FIX + '.kicad_pcb')

    def state():
        return ([id(s) for s in pcb.segments], [id(v) for v in pcb.vias],
                {r: [id(p) for p in fp.pads] for r, fp in pcb.footprints.items()},
                {n: [id(p) for p in pads] for n, pads in pcb.pads_by_net.items()})
    before = state()
    lists = (pcb.segments, pcb.vias, pcb.footprints['U2'].pads)
    with only_span_copper(pcb, ['F.Cu', 'In1.Cu']):
        ok(len(pcb.footprints['U2'].pads) == 0 and len(pcb.footprints['U1'].pads) == 100,
           "inside, the bottom-side part has no pads and the BGA has all of its own")
    ok(state() == before, "every pad, track and via is back, in its place")
    ok(lists[0] is pcb.segments and lists[1] is pcb.vias
       and lists[2] is pcb.footprints['U2'].pads, "and the lists are the same objects")


def main():
    with open(FIX + '.kicad_pro', encoding='utf-8') as fh:
        own = json.load(fh)['board']['design_settings']['rules']
    with tempfile.TemporaryDirectory() as td:
        print("1. through vias")
        out, _log, got = fan(td, 'through', '--layers', 'F.Cu', 'In1.Cu')
        pcb = parse_kicad_pcb(evidence(out, 'the fanned board'))
        ok(got == (24, 12) and not pcb.vias,
           f"24 of 36 balls escape and no via is laid: {got}, {len(pcb.vias)} vias")

        print("2. a micro via span")
        out, log, got = fan(td, 'span', '--via-span', 'F.Cu', 'In1.Cu')
        pcb = parse_kicad_pcb(evidence(out, 'the fanned board'))
        ok(got == (36, 0), f"all 36 balls escape: {got}")
        kinds = sorted({(v.via_type, tuple(v.layers), round(v.size, 4), round(v.drill, 4))
                        for v in pcb.vias})
        ok(kinds == [('micro', ('F.Cu', 'In1.Cu'), 0.25, 0.1)] and len(pcb.vias) == 12,
           f"with 12 micro vias at the class's micro via size: {kinds} x{len(pcb.vias)}")
        ok(sorted({s.layer for s in pcb.segments}) == ['F.Cu', 'In1.Cu'],
           f"copper on the span only: {sorted({s.layer for s in pcb.segments})}")
        d = check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), out,
                        '--clearance-margin-mm', '0.0005'], accept=True)
        ok('NO DRC VIOLATIONS FOUND' in (d.stdout or ''), "check_drc is clean")
        with open(os.path.splitext(out)[0] + '.kicad_pro', encoding='utf-8') as fh:
            rules = json.load(fh)['board']['design_settings']['rules']
        ok(all(rules.get(k) == own.get(k) for k in KEYS),
           f"the project's via and hole minimums are the board's own: "
           f"{ {k: rules.get(k) for k in KEYS} }")
        ok('Via span: 12 fan-out via(s) written as micro F.Cu-In1.Cu' in log,
           "and the run says what it wrote")

        restored()

        print("4. a part on neither end of the span")
        board = os.path.join(td, 'bad_in.kicad_pcb')
        for ext in ('.kicad_pcb', '.kicad_pro'):
            shutil.copy(FIX + ext, os.path.splitext(board)[0] + ext)
        check(PY + [os.path.join(REPO, 'py_router', 'bga_fanout.py'), board,
                    '-o', os.path.join(td, 'bad.kicad_pcb'), '--component', 'U1',
                    '--via-span', 'In1.Cu', 'In6.Cu'],
              refuse='which is not an end of the span')
        ok(True, "is refused by name")
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())

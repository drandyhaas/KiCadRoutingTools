#!/usr/bin/env python3
"""#1842: `route.py --via-span TOP BOTTOM` lays vias that join only that run
of copper layers -- micro, blind or buried -- and routes on those layers.

The router laid through vias only, and every copper layer's obstacles blocked
a via site. On the fixture (tests/fixtures/1842, synthetic: 8 layers, a
10x10 BGA at 0.8 mm on F.Cu whose inner 6x6 balls carry `S01`..`S36` to a
header, and a 9x9 array of GND pads on B.Cu under it, on the inter-ball via
sites) a bare route completes 25 of the 36 nets: the bottom pads leave a
through via nowhere to land under the part.

Checks:
  1. `via_span.resolve_via_span` reads a span's layers and via type off the
     stack, and refuses a layer the board lacks, one layer twice and a type
     the span cannot be;
  2. `SpanView` hides the copper outside the span from the obstacle
     builders and keeps drilled pads;
  3. `route.py --via-span F.Cu In1.Cu` on the fixture: every net routed,
     every via a micro via F.Cu-In1.Cu at the Default class's micro via
     size, nothing on a layer outside the span, check_drc and
     check_connected clean;
  4. the output project keeps the board's via and drilled-hole minimums (a
     micro via says nothing about them);
  5. --via-span refuses a layer the board does not have, by name.
"""
import contextlib
import io
import json
import os
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from via_span import resolve_via_span, SpanView, file_copper_layers  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
BOARD = os.path.join(TESTS, 'fixtures', '1842', 'hdi_repro.kicad_pcb')
STACK = ['F.Cu', 'In1.Cu', 'In2.Cu', 'In3.Cu', 'In4.Cu', 'In5.Cu', 'In6.Cu', 'B.Cu']
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def refused(*args):
    try:
        resolve_via_span(*args)
    except ValueError as e:
        return str(e)
    return ''


def load(path):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(path)


def spans():
    print("1. a span's layers and via type")
    ok(file_copper_layers(BOARD) == STACK, f"the fixture's stack: {file_copper_layers(BOARD)}")
    want = {('F.Cu', 'In1.Cu'): (2, 'micro'), ('B.Cu', 'In6.Cu'): (2, 'micro'),
            ('F.Cu', 'In3.Cu'): (4, 'blind'), ('In1.Cu', 'In6.Cu'): (6, 'buried'),
            ('In2.Cu', 'In3.Cu'): (2, 'buried'), ('F.Cu', 'B.Cu'): (8, None)}
    got = {k: (len(resolve_via_span(STACK, *k)[0]), resolve_via_span(STACK, *k)[1])
           for k in want}
    ok(got == want, f"types by span: {got}")
    ok(resolve_via_span(STACK, 'In1.Cu', 'F.Cu')[0] == ['F.Cu', 'In1.Cu'],
       "the pair may be given bottom first")
    ok(resolve_via_span(STACK, 'F.Cu', 'In1.Cu', 'blind')[1] == 'blind',
       "a type the span can be overrides the one read off it")
    ok('not a copper layer' in refused(STACK, 'F.Cu', 'In9.Cu')
       and 'two different layers' in refused(STACK, 'In1.Cu', 'In1.Cu')
       and 'cannot span' in refused(STACK, 'F.Cu', 'In1.Cu', 'buried')
       and 'whole stack' in refused(STACK, 'F.Cu', 'B.Cu', 'micro'),
       "a layer the board lacks, one layer twice and an impossible type are refused")


def view():
    print("2. SpanView hides the copper outside the span")
    pcb = load(BOARD)
    v = SpanView(pcb, ['F.Cu', 'In1.Cu'])
    full = sum(len(fp.pads) for fp in pcb.footprints.values())
    seen = sum(len(fp.pads) for fp in v.footprints.values())
    ok((full, seen, len(v.footprints['U2'].pads)) == (217, 136, 0),
       f"{full} pads, {seen} in the view: the 81 B.Cu pads of U2 are hidden")
    ok(sum(len(p) for p in v.pads_by_net.values()) == seen and v.nets is pcb.nets,
       "pads_by_net agrees, and the rest of the board is the base's")


def routed(td):
    print("3. route.py --via-span F.Cu In1.Cu")
    board = os.path.join(td, 'in.kicad_pcb')
    out = os.path.join(td, 'out.kicad_pcb')
    summary = os.path.join(td, 'route.json')
    check(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'), evidence(BOARD), board],
          accept=True)
    r = check(PY + [os.path.join(REPO, 'py_router', 'route.py'), board, out,
                    '--nets', 'S*', '--via-span', 'F.Cu', 'In1.Cu', '--json-out', summary],
              accept=True, timeout=1800)
    with open(evidence(summary, 'the route summary'), encoding='utf-8') as fh:
        doc = json.load(fh)
    ok(len(doc.get('routed_single') or []) == 36 and not doc.get('failed_single')
       and not doc.get('open_single'),
       f"every net routed: {len(doc.get('routed_single') or [])} of 36, "
       f"failed {doc.get('failed_single')}")
    ok('micro vias F.Cu-In1.Cu' in r.stdout and 'micro via 0.25mm' in r.stdout
       and 'micro via 0.1mm' in r.stdout,
       "the run names the span and the class micro via it draws")
    pcb = load(evidence(out, 'the routed board'))
    kinds = {(v.via_type, tuple(v.layers), v.size, v.drill) for v in pcb.vias}
    ok(len(pcb.vias) > 0 and kinds == {('micro', ('F.Cu', 'In1.Cu'), 0.25, 0.1)},
       f"{len(pcb.vias)} vias, all micro F.Cu-In1.Cu 0.25/0.1: {sorted(kinds)}")
    layers = {s.layer for s in pcb.segments}
    ok(layers <= {'F.Cu', 'In1.Cu'}, f"tracks only on the span's layers: {sorted(layers)}")
    check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), out, '--quiet'],
          accept=True)
    check(PY + [os.path.join(REPO, 'py_router', 'check_connected.py'), out, '--nets', 'S*'],
          accept=True)
    ok(True, "check_drc and check_connected are clean on the routed nets")

    print("4. the board's via minimums are kept")
    with open(evidence(os.path.splitext(BOARD)[0] + '.kicad_pro'), encoding='utf-8') as fh:
        before = json.load(fh)['board']['design_settings']['rules']
    with open(evidence(os.path.splitext(out)[0] + '.kicad_pro', 'the output project'),
              encoding='utf-8') as fh:
        after = json.load(fh)['board']['design_settings']['rules']
    keys = ('min_via_diameter', 'min_through_hole_diameter', 'min_microvia_diameter',
            'min_microvia_drill')
    ok(all(after.get(k) == before.get(k) for k in keys),
       f"{ {k: (before.get(k), after.get(k)) for k in keys} }")
    return board


def refusal(td, board):
    print("5. a layer the board does not have")
    check(PY + [os.path.join(REPO, 'py_router', 'route.py'), board,
                os.path.join(td, 'x.kicad_pcb'), '--nets', 'S*',
                '--via-span', 'F.Cu', 'In9.Cu'],
          refuse='--via-span: In9.Cu is not a copper layer', code=2, allow=('error:',))
    ok(True, "--via-span F.Cu In9.Cu is refused by name (exit 2)")


def main():
    spans()
    view()
    with tempfile.TemporaryDirectory() as td:
        refusal(td, routed(td))
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())

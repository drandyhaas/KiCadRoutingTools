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
  4. the output project keeps the board's via, drilled-hole, annular and
     copper-to-hole minimums (a micro via says nothing about them), and the
     micro vias are held by `fix_kicad_drc_settings.MICRO_VIA_RULE` in the
     output's .kicad_dru;
  5. --via-span refuses a layer the board does not have, by name;
  6. `--via-span-escape only` lays a micro via beside every pad of the
     selected nets, joined by a short track, and routes nothing; a second
     step, `--via-span In1.Cu In6.Cu`, then routes the nets on the inner
     layers from those landings with buried vias In1.Cu-In6.Cu, and laid
     copper is clean in check_drc after each step. That step holds the
     copper it could not lay again -- the escapes -- in place: every micro
     via and stub is still there afterwards, on the nets it did not complete
     too, while its own routes are still ripped and retried;
  7. a board whose micro vias are IN the pads is routed from them the same
     way, and the inner step lays nothing on a layer outside its span (its
     in-pad rescue used to put a stub on F.Cu);
  8. on a board with a copper pour, a step on a partial span does not run
     the plane finalize (its taps are through vias at the step's via size),
     says so, and still lays only vias of the span.
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
            'min_microvia_drill', 'min_via_annular_width', 'min_hole_clearance')
    ok(all(after.get(k) == before.get(k) for k in keys),
       f"{ {k: (before.get(k), after.get(k)) for k in keys} }")
    # the micro vias' ring and drill are held by a rule of their own instead
    with open(evidence(os.path.splitext(out)[0] + '.kicad_dru', 'the output rules'),
              encoding='utf-8') as fh:
        dru = fh.read()
    ok("(rule \"kicad_routing_tools micro vias\"" in dru
       and "(condition \"A.Via_Type == 'Micro'\")" in dru
       and '(constraint annular_width (min 0.075mm))' in dru,
       f"and the micro vias have their rule:\n{dru}")
    return board


def refusal(td, board):
    print("5. a layer the board does not have")
    check(PY + [os.path.join(REPO, 'py_router', 'route.py'), board,
                os.path.join(td, 'x.kicad_pcb'), '--nets', 'S*',
                '--via-span', 'F.Cu', 'In9.Cu'],
          refuse='--via-span: In9.Cu is not a copper layer', code=2, allow=('error:',))
    ok(True, "--via-span F.Cu In9.Cu is refused by name (exit 2)")


def escapes(td, board):
    print("6. --via-span-escape only, then the inner sub-stack")
    route = os.path.join(REPO, 'py_router', 'route.py')
    step1 = os.path.join(td, 'escaped.kicad_pcb')
    r = check(PY + [route, board, step1, '--nets', 'S*', '--via-span', 'F.Cu', 'In1.Cu',
                    '--via-span-escape', 'only'], accept=True, timeout=1800)
    ok('72 micro via(s) F.Cu-In1.Cu placed at surface pads, 0 declined' in r.stdout
       and 'nothing is routed' in r.stdout,
       "the run reports 72 escapes placed, none declined, nothing routed")
    pcb = load(evidence(step1, 'the escaped board'))
    by_net = {}
    for v in pcb.vias:
        by_net.setdefault(v.net_id, []).append(v)
    ok(len(pcb.vias) == 72 and all(len(vs) == 2 for vs in by_net.values())
       and {(v.via_type, tuple(v.layers)) for v in pcb.vias} == {('micro', ('F.Cu', 'In1.Cu'))},
       f"{len(pcb.vias)} micro vias F.Cu-In1.Cu, two on each of {len(by_net)} nets")
    ok(len(pcb.segments) == 72 and {s.layer for s in pcb.segments} == {'F.Cu'},
       f"{len(pcb.segments)} short tracks on F.Cu join each via to its pad")
    beside = 0
    for nid, vs in by_net.items():
        for pad in pcb.pads_by_net[nid]:
            gap = min(((v.x - pad.global_x) ** 2 + (v.y - pad.global_y) ** 2) ** 0.5
                      for v in vs)
            beside += 0.1 < gap <= 1.5
    ok(beside == 72, f"every pad has its via beside it, not in it: {beside} of 72")
    check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), step1, '--quiet'],
          accept=True)

    step2 = os.path.join(td, 'inner.kicad_pcb')
    summary = os.path.join(td, 'inner.json')
    r = check(PY + [route, step1, step2, '--nets', 'S*', '--via-span', 'In1.Cu', 'In6.Cu',
                    '--json-out', summary], accept=True, timeout=1800)
    with open(evidence(summary, 'the inner step summary'), encoding='utf-8') as fh:
        doc = json.load(fh)
    n = len(doc.get('routed_single') or [])
    ok('buried vias In1.Cu-In6.Cu' in r.stdout and n >= 30,
       f"the inner step routes from the landings: {n} of 36 nets")
    pcb = load(evidence(step2, 'the inner-routed board'))
    buried = [v for v in pcb.vias if v.via_type == 'buried']
    micro = [v for v in pcb.vias if v.via_type == 'micro']
    stubs = [s for s in pcb.segments if s.layer == 'F.Cu']
    ok('existing track(s) and via(s) with copper outside the span are held in place'
       in r.stdout and len(micro) == 72 and len(stubs) == 72,
       f"the step keeps every escape: {len(micro)} micro vias and {len(stubs)} stubs "
       f"of 72, though {36 - n} net(s) did not complete")
    ok('Ripping' in r.stdout, "and it still rips and retries its own routes")
    ok(len(buried) > 0 and {tuple(v.layers) for v in buried} == {('In1.Cu', 'In6.Cu')}
       and {v.via_type for v in pcb.vias} == {'micro', 'buried'},
       f"{len(buried)} buried vias In1.Cu-In6.Cu beside the micro vias, no through via")
    inner = {s.layer for s in pcb.segments} - {'F.Cu'}
    ok(inner and inner <= {'In1.Cu', 'In2.Cu', 'In3.Cu', 'In4.Cu', 'In5.Cu', 'In6.Cu'}
       and len(inner) > 1,
       f"its tracks are on the inner layers: {sorted(inner)}")
    check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), step2, '--quiet'],
          accept=True)
    ok(True, "check_drc is clean after each step")


def in_pad(td, board):
    print("7. the inner sub-stack from micro vias in the pads")
    pcb = load(board)
    add = ''
    for nid, pads in sorted(pcb.pads_by_net.items()):
        name = pcb.nets[nid].name if nid in pcb.nets else ''
        if not name.startswith('S'):
            continue
        for p in pads:
            add += (f'\t(via micro\n\t\t(at {p.global_x} {p.global_y})\n\t\t(size 0.25)\n'
                    f'\t\t(drill 0.1)\n\t\t(layers "F.Cu" "In1.Cu")\n'
                    f'\t\t(net "{name}")\n\t)\n')
    with open(board, encoding='utf-8') as fh:
        t = fh.read()
    k = t.rindex('(embedded_fonts')
    k = t.rindex('\n', 0, k) + 1
    staged = os.path.join(td, 'inpad.kicad_pcb')
    with open(staged, 'w', encoding='utf-8') as fh:
        fh.write(t[:k] + add + t[k:])
    with open(os.path.splitext(board)[0] + '.kicad_pro', encoding='utf-8') as fh, \
            open(os.path.splitext(staged)[0] + '.kicad_pro', 'w', encoding='utf-8') as oh:
        oh.write(fh.read())
    out = os.path.join(td, 'inpad_routed.kicad_pcb')
    summary = os.path.join(td, 'inpad.json')
    check(PY + [os.path.join(REPO, 'py_router', 'route.py'), evidence(staged), out,
                '--nets', 'S*', '--via-span', 'In1.Cu', 'In6.Cu', '--json-out', summary],
          accept=True, timeout=1800)
    with open(evidence(summary, 'the summary'), encoding='utf-8') as fh:
        n = len(json.load(fh).get('routed_single') or [])
    pcb = load(evidence(out, 'the routed board'))
    micro = [v for v in pcb.vias if v.via_type == 'micro']
    ok(n >= 28, f"the step routes from the in-pad vias: {n} of 36 nets")
    ok(len(micro) == 72, f"every in-pad micro via is kept: {len(micro)} of 72")
    outside = sorted({s.layer for s in pcb.segments} - set(STACK[1:7]))
    ok(not outside, f"no track on a layer outside In1.Cu-In6.Cu: {outside}")
    check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), out, '--quiet'],
          accept=True)


ZONE = '''\t(zone
\t\t(net "S01")
\t\t(net_name "S01")
\t\t(layer "B.Cu")
\t\t(uuid "00000000-0000-4000-8000-000000001842")
\t\t(hatch edge 0.5)
\t\t(connect_pads
\t\t\t(clearance 0.2)
\t\t)
\t\t(min_thickness 0.2)
\t\t(filled_areas_thickness no)
\t\t(fill
\t\t\t(thermal_gap 0.3)
\t\t\t(thermal_bridge_width 0.3)
\t\t)
\t\t(polygon
\t\t\t(pts
\t\t\t\t(xy 0.5 0.5) (xy 2.5 0.5) (xy 2.5 2.5) (xy 0.5 2.5)
\t\t\t)
\t\t)
\t)
'''


def no_finalize(td, board):
    print("8. a partial span and the plane finalize")
    with open(board, encoding='utf-8') as fh:
        t = fh.read()
    k = t.rindex('\t(embedded_fonts')
    staged = os.path.join(td, 'pour.kicad_pcb')
    with open(staged, 'w', encoding='utf-8') as fh:
        fh.write(t[:k] + ZONE + t[k:])
    with open(os.path.splitext(board)[0] + '.kicad_pro', encoding='utf-8') as fh, \
            open(os.path.splitext(staged)[0] + '.kicad_pro', 'w', encoding='utf-8') as oh:
        oh.write(fh.read())
    ok(len(load(staged).zones) == 1, "the staged board carries a pour")
    out = os.path.join(td, 'pour_routed.kicad_pcb')
    r = check(PY + [os.path.join(REPO, 'py_router', 'route.py'), evidence(staged), out,
                    '--nets', 'S*', '--via-span', 'F.Cu', 'In1.Cu'],
              accept=True, timeout=1800)
    log = (r.stdout or '') + (r.stderr or '')
    ok('Plane finalize not run: this step lays micro vias on F.Cu-In1.Cu' in log,
       "the run says the plane finalize was not run")
    kinds = sorted({(v.via_type, tuple(v.layers)) for v in load(evidence(out)).vias})
    ok(kinds == [('micro', ('F.Cu', 'In1.Cu'))], f"every via is the span's: {kinds}")


def main():
    spans()
    view()
    with tempfile.TemporaryDirectory() as td:
        board = routed(td)
        refusal(td, board)
        escapes(td, board)
        in_pad(td, board)
        no_finalize(td, board)
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())

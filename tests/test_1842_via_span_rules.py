#!/usr/bin/env python3
"""#1842: the rules a via-span step keeps, on tests/fixtures/1842.

  A. a micro via is laser drilled from an outer layer to the layer next to
     it: `--via-span F.Cu In3.Cu --via-span-type micro` is refused by name
     (it used to be drawn at the 0.25/0.1 micro via, which check_drc grades
     as a mechanically drilled hole below the drill floor), and the CLI and
     the GUI read one predicate (`via_span.laser_drilled`), which agrees with
     `design_rules.laser_drilled_span` on every span and type of the stack;
  B. a board whose file format predates KiCad 10 has no `buried` via: the
     step writes `blind` (KiCad 9's blind/buried type) on the same layer pair,
     and a KiCad 10 board keeps `buried`;
  C. the plan converter reads `--via-span-type` / `--via-span-escape` as the
     words they are (they were switches, so a recorded type became True);
  D. the lift records a base build under a span binds land on the run's
     board, where `routing_context` reads them: a footprint's own copper
     notched around a routed pad is lifted for that pad's net;
  E. a rebuilt net cache under a span is the span's: a net's pad on a layer
     outside the span blocks no via site after a rip any more than before;
  F. an explicit via size on a micro span is not pinned to the fab floor;
  G. JSON_SUMMARY and the merged --json-out carry `via_span`: on a board
     with no pour the plane finalize taps nothing (`not run`), and the zone
     counts say how many pours lie outside the span; none without a span
     (test_1842_via_span_planes reads `run` where the finalize taps a pour);
  H. a route end beside an earlier step's escape stub (held by the span) is
     bridged like any other joint of its net: the span hold is no sense hold;
  I. held copper is not ripped with its net: `--force-reroute` on a step
     whose nets' only copper is held escape vias strips none of them (it
     used to strip all 72 and find no landing left), and routes the nets.
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'tests', 'stress'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
ROUTE = os.path.join(REPO, 'py_router', 'route.py')
DRC = os.path.join(REPO, 'py_router', 'check_drc.py')
BOARD = os.path.join(TESTS, 'fixtures', '1842', 'hdi_repro.kicad_pcb')
STACK = ['F.Cu', 'In1.Cu', 'In2.Cu', 'In3.Cu', 'In4.Cu', 'In5.Cu', 'In6.Cu', 'B.Cu']
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def load(path):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(path)


def stage(text, dst):
    """Write a board text beside a copy of the fixture's project."""
    with open(dst, 'w', encoding='utf-8') as fh:
        fh.write(text)
    shutil.copy(os.path.splitext(BOARD)[0] + '.kicad_pro', os.path.splitext(dst)[0] + '.kicad_pro')
    return dst


def fixture_text():
    with open(BOARD, encoding='utf-8') as fh:
        return fh.read()


def with_in_pad_vias(text, version=None):
    """The fixture with a micro via F.Cu-In1.Cu in every signal pad, so a step
    on In1.Cu-In6.Cu has a landing to route from (and lays buried vias)."""
    pcb = load(BOARD)
    add = ''
    for nid, pads in sorted(pcb.pads_by_net.items()):
        name = pcb.nets[nid].name if nid in pcb.nets else ''
        if not name.startswith('S'):
            continue
        for p in pads:
            add += (f'\t(via micro\n\t\t(at {p.global_x} {p.global_y})\n\t\t(size 0.25)\n'
                    f'\t\t(drill 0.1)\n\t\t(layers "F.Cu" "In1.Cu")\n\t\t(net "{name}")\n\t)\n')
    k = text.rindex('(embedded_fonts')
    k = text.rindex('\n', 0, k) + 1
    text = text[:k] + add + text[k:]
    if version:
        text = text.replace('(version 20260206)', f'(version {version})', 1)
    return text


def span_config(span=('F.Cu', 'In1.Cu'), kind='micro'):
    from routing_config import GridRouteConfig
    return GridRouteConfig(layers=list(span), via_span=tuple(span), via_span_type=kind,
                           track_width=0.1, clearance=0.1, via_size=0.25, via_drill=0.1,
                           grid_step=0.05)


def a_micro(td):
    print("A. a micro via joins an outer layer and the layer next to it")
    from via_span import resolve_via_span, laser_drilled
    from design_rules import laser_drilled_span
    try:
        resolve_via_span(STACK, 'F.Cu', 'In3.Cu', 'micro')
        why = ''
    except ValueError as e:
        why = str(e)
    ok('a micro via cannot span F.Cu-In3.Cu' in why, f"micro on F.Cu-In3.Cu is refused: {why}")
    agree, n = True, 0
    for i in range(len(STACK)):
        for j in range(i + 1, len(STACK)):
            for kind in (None, 'micro', 'blind', 'buried'):
                try:
                    span, got = resolve_via_span(STACK, STACK[i], STACK[j], kind)
                except ValueError:
                    continue
                n += 1
                agree &= (laser_drilled(STACK, [STACK[i], STACK[j]], kind)
                          == laser_drilled_span(got, span, STACK)
                          == (got == 'micro'))
    ok(agree and n > 20, f"laser_drilled, laser_drilled_span and the type agree on {n} spans")
    board = stage(fixture_text(), os.path.join(td, 'a.kicad_pcb'))
    check(PY + [ROUTE, board, os.path.join(td, 'a_out.kicad_pcb'), '--nets', 'S*',
                '--via-span', 'F.Cu', 'In3.Cu', '--via-span-type', 'micro'],
          refuse='--via-span: a micro via cannot span F.Cu-In3.Cu', code=2, allow=('error:',))
    ok(True, "route.py refuses --via-span F.Cu In3.Cu --via-span-type micro by name (exit 2)")


def b_file_version(td):
    print("B. a pre-KiCad-10 file has no `buried` via")
    from via_span import file_via_type
    ok((file_via_type('buried', 20241229), file_via_type('buried', 20260206),
        file_via_type('buried', 0), file_via_type('micro', 20241229))
       == ('blind', 'buried', 'buried', 'micro'),
       "buried -> blind before KiCad 10's format only; a live board's (0) is left")
    for ver, word in (('20241229', 'blind'), (None, 'buried')):
        board = stage(with_in_pad_vias(fixture_text(), ver),
                      os.path.join(td, f'b{ver or "10"}.kicad_pcb'))
        out = os.path.join(td, f'b{ver or "10"}_out.kicad_pcb')
        r = check(PY + [ROUTE, board, out, '--nets', 'S*', '--via-span', 'In1.Cu', 'In6.Cu'],
                  accept=True, timeout=1800)
        with open(evidence(out, 'the routed board'), encoding='utf-8') as fh:
            text = fh.read()
        laid = [v for v in load(out).vias if tuple(v.layers) == ('In1.Cu', 'In6.Cu')]
        other = 'buried' if word == 'blind' else 'blind'
        ok(laid and {v.via_type for v in laid} == {word}
           and f'(via {other}' not in text and f'(via {word}' in text,
           f"format {ver or 'KiCad 10'}: {len(laid)} vias In1.Cu-In6.Cu written "
           f"'{word}', none '{other}'")
        ok((ver is None) or 'predates KiCad 10' in r.stdout,
           "the run says why it writes blind" if ver else "(KiCad 10: no note)")
        check(PY + [DRC, out, '--quiet'], accept=True)


def c_converter():
    print("C. the plan converter reads the span flags' words")
    import manifest_to_plan as m2p
    step = m2p.parse_command(['python3', 'py_router/route.py', 'a.kicad_pcb', 'b.kicad_pcb',
                              '--nets', 'S*', '--via-span-escape', 'only',
                              '--via-span', 'F.Cu', 'In1.Cu', '--via-span-type', 'micro'])
    p = step['params']
    ok((p.get('via_span'), p.get('via_span_escape'), p.get('via_span_type'))
       == (['F.Cu', 'In1.Cu'], 'only', 'micro'), f"params: {p}")


TAB = '''\t\t(fp_poly
\t\t\t(pts
\t\t\t\t(xy -1.7 -9.3) (xy -0.25 -9.3) (xy -0.25 -8.8) (xy -1.05 -8.8)
\t\t\t\t(xy -1.05 -8.2) (xy -0.25 -8.2) (xy -0.25 -7.95) (xy -1.7 -7.95)
\t\t\t)
\t\t\t(stroke
\t\t\t\t(width 0)
\t\t\t\t(type solid)
\t\t\t)
\t\t\t(fill yes)
\t\t\t(layer "F.Cu")
\t\t\t(uuid "00000000-0000-4000-8000-0000000018d2")
\t\t)
'''


def d_lift_records(td):
    print("D. the lift records land on the run's board")
    from obstacle_map import build_base_obstacle_map
    from routing_context import ensure_own_pad_lift, release_own_pad_lift
    from via_span import SpanView
    t = fixture_text()
    k = t.index('\t\t(attr smd)', t.index('(property "Reference" "J1"'))
    board = stage(t[:k] + TAB + t[k:], os.path.join(td, 'tab.kicad_pcb'))
    pcb = load(board)
    nid = next(n for n, v in pcb.nets.items() if v.name == 'S01')
    ok(any(getattr(s, 'owner_ref', '') == 'J1' for s in pcb.segments if s.graphic),
       "J1 carries F.Cu graphic copper notched around its pad 1 (S01)")
    config = span_config()
    with contextlib.redirect_stdout(io.StringIO()):
        # the run's base build, as batch_route makes it: handed the view
        obstacles = build_base_obstacle_map(SpanView(pcb, config.via_span), config, [nid])
    rows = (pcb.__dict__.get('_graphic_own_pad_lift') or {}).get(nid)
    ok(rows is not None and len(rows) > 0,
       f"the board carries S01's own-pad rows: {0 if rows is None else len(rows)}")
    before = obstacles.get_stats()[0]
    token = ensure_own_pad_lift(obstacles, pcb, nid)
    after = obstacles.get_stats()[0]
    release_own_pad_lift(obstacles, nid, token)
    ok(token is not None and after < before and obstacles.get_stats()[0] == before,
       f"the lift read off the board frees cells for S01 ({before} -> {after}) "
       f"and the release puts them back")


def e_rebuilt_cache():
    print("E. a rebuilt cache under a span is the span's")
    from obstacle_cache import precompute_net_obstacles
    from via_span import SpanView
    import numpy as np
    pcb = load(BOARD)
    nid = next(n for n, v in pcb.nets.items() if v.name == 'S01')
    pad = next(p for p in pcb.footprints['U2'].pads if p.pad_number == '41')
    pad.net_id, pad.net_name = nid, 'S01'   # a signal pad on B.Cu, outside the span
    pcb.pads_by_net.setdefault(nid, []).append(pad)
    config = span_config()

    def vias(cd):
        return {tuple(r) for r in np.asarray(cd.blocked_vias).reshape(-1, 2)}
    with contextlib.redirect_stdout(io.StringIO()):
        full = precompute_net_obstacles(pcb, nid, config)
        view = precompute_net_obstacles(SpanView(pcb, config.via_span), nid, config)
    ok(vias(full) == vias(view) and len(vias(view)) > 0,
       f"handed the board, the cache equals the view's ({len(vias(full))} via cells, "
       f"view {len(vias(view))}): the B.Cu pad blocks no micro via site")


def f_explicit_micro(td):
    print("F. an explicit micro via size is not pinned to the fab floor")
    board = stage(fixture_text(), os.path.join(td, 'f.kicad_pcb'))
    out = os.path.join(td, 'f_out.kicad_pcb')
    check(PY + [ROUTE, board, out, '--nets', 'S*', '--via-span', 'F.Cu', 'In1.Cu',
                '--via-size', '0.25', '--via-drill', '0.1'], accept=True, timeout=1800)
    kinds = {(v.via_type, v.size, v.drill) for v in load(evidence(out)).vias}
    ok(kinds == {('micro', 0.25, 0.1)}, f"the vias are the asked micro via: {sorted(kinds)}")
    out2 = os.path.join(td, 'f_blind.kicad_pcb')
    check(PY + [ROUTE, board, out2, '--nets', 'S*', '--via-span', 'F.Cu', 'In1.Cu',
                '--via-span-type', 'blind', '--via-size', '0.25', '--via-drill', '0.1'],
          accept=True, timeout=1800)
    drills = {v.drill for v in load(evidence(out2)).vias}
    ok(drills and min(drills) >= 0.15 - 1e-9,
       f"a blind (mechanical) via of the same span keeps the drill floor: {sorted(drills)}")


def g_summary(td):
    print("G. the via span and what its plane finalize taps are in the summary")
    board = stage(fixture_text(), os.path.join(td, 'g.kicad_pcb'))
    for args, want in ((['--via-span', 'F.Cu', 'In1.Cu'],
                        {'layers': ['F.Cu', 'In1.Cu'], 'type': 'micro',
                         'plane_finalize': 'not run', 'zones': 0, 'zones_off_span': 0}),
                       ([], None)):
        out = os.path.join(td, f'g{len(args)}.kicad_pcb')
        js = os.path.join(td, f'g{len(args)}.json')
        r = check(PY + [ROUTE, board, out, '--nets', 'S0*', '--json-out', js] + args,
                  accept=True, timeout=1800)
        with open(evidence(js, 'the route summary'), encoding='utf-8') as fh:
            doc = json.load(fh)
        line = next((json.loads(l[len('JSON_SUMMARY: '):]) for l in r.stdout.splitlines()
                     if l.startswith('JSON_SUMMARY: ')), {})
        ok(doc.get('via_span') == want and line.get('via_span') == want,
           f"{'with' if args else 'without'} a span: --json-out {doc.get('via_span')}, "
           f"JSON_SUMMARY {line.get('via_span')}")


def h_bridge():
    print("H. the span hold is no sense hold")
    from pcb_modification import _sense_copper, close_soft_joints
    from kicad_parser import Segment
    from via_span import HELD_OUTSIDE_SPAN
    pcb = load(BOARD)
    nid = next(n for n, v in pcb.nets.items() if v.name == 'S01')
    held = Segment(start_x=17.0, start_y=20.0, end_x=17.5, end_y=20.0, width=0.1,
                   layer='F.Cu', net_id=nid)
    held.held_by = HELD_OUTSIDE_SPAN
    sense = Segment(start_x=17.0, start_y=21.0, end_x=17.5, end_y=21.0, width=0.1,
                    layer='F.Cu', net_id=nid)
    sense.held_by = 'sense:R1.1'
    ok(not _sense_copper(held) and _sense_copper(sense),
       "a span-held item is not sense copper; a sense-held one still is")
    new = Segment(start_x=17.55, start_y=20.0, end_x=18.0, end_y=20.0, width=0.1,
                  layer='F.Cu', net_id=nid)
    pcb.segments += [held, new]
    results = [{'new_segments': [new], 'new_vias': []}]
    with contextlib.redirect_stdout(io.StringIO()):
        n = close_soft_joints(results, pcb, {nid}, span_config())
    ok(n == 1, f"a route end 0.05 mm short of a held escape stub is bridged: {n} bridge(s)")


def i_held_not_ripped(td):
    print("I. held copper stays when its net is ripped")
    board = stage(with_in_pad_vias(fixture_text()), os.path.join(td, 'i.kicad_pcb'))
    out = os.path.join(td, 'i_out.kicad_pcb')
    js = os.path.join(td, 'i.json')
    r = check(PY + [ROUTE, board, out, '--nets', 'S*', '--via-span', 'In1.Cu', 'In6.Cu',
                    '--force-reroute', '--json-out', js], accept=True, timeout=1800)
    with open(evidence(js, 'the route summary'), encoding='utf-8') as fh:
        n = len(json.load(fh).get('routed_single') or [])
    micro = [v for v in load(evidence(out)).vias if v.via_type == 'micro']
    strip = next((l.strip() for l in r.stdout.splitlines()
                  if l.startswith('--force-reroute: stripped')), '')
    ok('stripped 0 segment(s) / 0 via(s)' in strip and len(micro) == 72 and n >= 34,
       f"--force-reroute strips no held via ({strip!r}), keeps {len(micro)} of 72, "
       f"routes {n} of 36")


def main():
    with tempfile.TemporaryDirectory() as td:
        a_micro(td)
        b_file_version(td)
        c_converter()
        d_lift_records(td)
        e_rebuilt_cache()
        f_explicit_micro(td)
        g_summary(td)
        h_bridge()
        i_held_not_ripped(td)
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())

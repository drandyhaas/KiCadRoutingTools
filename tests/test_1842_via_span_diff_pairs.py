#!/usr/bin/env python3
"""#1842: route_diff.py --via-span routes a pair on one sub-stack.

The diff-pair step laid through vias only, and copper on every layer blocked
a via site. `route_diff.py --via-span TOP BOTTOM` has route.py's meaning: the
pairs route on that run of neighbouring copper layers, every via the step
lays joins exactly it, and only copper on those layers (and drilled pads)
blocks a via site.

The fixture (tests/fixtures/1842/diff_pair) is synthetic: the 8-layer board
of the route step's fixture, one pair `/D0_P` + `/D0_N` with its pads either
side of a wall of F.Cu copper the full height of the board, and a field of
bottom-side pads under everything, 0.34 mm apart -- no through via fits
between them. The pair has to change layer to pass the wall.

Checks:
  1. with through vias (route_diff.py on F.Cu + In1.Cu) the pair does not
     route, coupled or member by member;
  2. with `--via-span F.Cu In1.Cu` it routes: four micro vias F.Cu-In1.Cu at
     the Default class's micro via size, tracks on F.Cu and In1.Cu only, the
     pair reported routed, check_drc clean, both members connected, and the
     project's via and drilled-hole minimums left alone. kicad-cli 10.0.8
     grades that board at 0 violations;
  3. GND return vias: with no GND pour on a layer of the span they are not
     laid, and the run says so;
  4. a span the stack does not have is refused by name;
  5. the pair's OWN pads are stamped from the span's view of the board
     (`add_diff_pair_own_pads_as_obstacles`): an own pad with copper only on
     a layer outside the span blocks no via site of the span, and the same
     pad blocks one without a span (the control).
"""
import json
import os
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
FIX = os.path.join(TESTS, 'fixtures', '1842', 'diff_pair')
KEYS = ('min_via_diameter', 'min_through_hole_diameter', 'min_microvia_diameter',
        'min_microvia_drill')
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def route(td, tag, *extra, accept=True, **kw):
    board = os.path.join(td, tag + '_in.kicad_pcb')
    for ext in ('.kicad_pcb', '.kicad_pro'):
        shutil.copy(evidence(FIX + ext, 'the fixture'),
                    os.path.splitext(board)[0] + ext)
    out = os.path.join(td, tag + '.kicad_pcb')
    summary = os.path.join(td, tag + '.json')
    r = check(PY + [os.path.join(REPO, 'py_router', 'route_diff.py'), board, out,
                    '--nets', '/D*', '--json-out', summary, *extra],
              accept=accept, timeout=1800, **kw)
    return out, summary, (r.stdout or '') + (r.stderr or '')


def own_pad_via_site():
    """An own pad of the pair moved to B.Cu (copper outside the span):
    does it block the via site at its centre? Not under the span; yes
    without one."""
    from dataclasses import replace
    from obstacle_map import add_diff_pair_own_pads_as_obstacles
    from routing_config import GridRouteConfig, GridCoord
    from grid_router import GridObstacleMap
    pcb = parse_kicad_pcb(FIX + '.kicad_pcb')
    ids = {n.name: nid for nid, n in pcb.nets.items()}
    p, n = ids['/D0_P'], ids['/D0_N']
    pad = replace(pcb.pads_by_net[p][0], layers=['B.Cu'], drill=0)
    pcb.pads_by_net[p] = [pad]
    pcb.pads_by_net[n] = []
    blocked = {}
    for tag, span in (('span', ['F.Cu', 'In1.Cu']), ('none', None)):
        cfg = GridRouteConfig(layers=['F.Cu', 'In1.Cu'], via_span=span,
                              via_span_type='micro' if span else None)
        obs = GridObstacleMap(len(cfg.layers))
        add_diff_pair_own_pads_as_obstacles(obs, pcb, p, n, cfg)
        gx, gy = GridCoord(cfg.grid_step).to_grid(pad.global_x, pad.global_y)
        blocked[tag] = bool(obs.is_via_blocked(gx, gy))
    ok(blocked == {'span': False, 'none': True},
       f"a B.Cu own pad blocks the via site under it without a span only: {blocked}")


def main():
    with open(FIX + '.kicad_pro', encoding='utf-8') as fh:
        own = json.load(fh)['board']['design_settings']['rules']
    with tempfile.TemporaryDirectory() as td:
        print("1. through vias")
        out, summary, _log = route(td, 'through', '--layers', 'F.Cu', 'In1.Cu',
                                   '--no-bga-zones')
        with open(evidence(summary, 'the summary'), encoding='utf-8') as fh:
            doc = json.load(fh)
        pcb = parse_kicad_pcb(evidence(out, 'the routed board'))
        ok(not doc.get('routed_diff_pairs') and not pcb.vias and not pcb.segments,
           f"the pair does not route: routed {doc.get('routed_diff_pairs')}, "
           f"{len(pcb.vias)} vias, {len(pcb.segments)} tracks")

        print("2. a micro via span")
        out, summary, log = route(td, 'span', '--via-span', 'F.Cu', 'In1.Cu')
        with open(evidence(summary, 'the summary'), encoding='utf-8') as fh:
            doc = json.load(fh)
        pcb = parse_kicad_pcb(evidence(out, 'the routed board'))
        ok(doc.get('routed_diff_pairs') == ['/D0'], f"the pair routes: {doc.get('routed_diff_pairs')}")
        kinds = sorted((v.via_type, tuple(v.layers), round(v.size, 4), round(v.drill, 4))
                       for v in pcb.vias)
        ok(kinds == [('micro', ('F.Cu', 'In1.Cu'), 0.25, 0.1)] * 4,
           f"with four micro vias at the class's micro via size: {sorted(set(kinds))} x{len(kinds)}")
        ok(sorted({s.layer for s in pcb.segments}) == ['F.Cu', 'In1.Cu'],
           f"tracks on the span only: {sorted({s.layer for s in pcb.segments})}")
        d = check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), out,
                        '--clearance-margin-mm', '0.0005'], accept=True)
        ok('NO DRC VIOLATIONS FOUND' in (d.stdout or ''), "check_drc is clean")
        check(PY + [os.path.join(REPO, 'py_router', 'check_connected.py'), out,
                    '--nets', '/D*'], accept=True)
        ok(True, "both members are connected")
        with open(os.path.splitext(out)[0] + '.kicad_pro', encoding='utf-8') as fh:
            rules = json.load(fh)['board']['design_settings']['rules']
        ok(all(rules.get(k) == own.get(k) for k in KEYS),
           f"the project's via and hole minimums are the board's own: "
           f"{ {k: rules.get(k) for k in KEYS} }")

        print("3. GND return vias under a span")
        ok("GND return vias NOT placed: no 'GND' pour is on a layer of the via span "
           "F.Cu-In1.Cu" in log, "the run says they are not laid")
        ok("no GND net was found" not in log, "and does not also say there is no GND net")
        vs = doc.get('via_span') or {}
        ok(vs.get('layers') == ['F.Cu', 'In1.Cu'] and vs.get('type') == 'micro'
           and str(vs.get('gnd_return_vias', '')).startswith("not laid: no 'GND' pour"),
           f"JSON_SUMMARY.via_span says so: {vs}")

        print("4. a span the stack does not have")
        _o, _s, log = route(td, 'bad', '--via-span', 'F.Cu', 'In9.Cu', accept=False,
                            refuse='--via-span', code=2)
        ok('In9.Cu' in log, "is refused by name")

    print("5. the pair's own pads under a span")
    own_pad_via_site()
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""A failed net leaves no rescue escape on a layer the run does not route.

The net rescue's #666 bare-ball escape gives a bare SMD pad a dogbone (a
via, plus a trace on the pad's layer) before the gap routes run, so they can
start from a via with every layer. A run restricted by --layers carries the
board's other copper layers at FORBIDDEN cost: nothing is routed on them.
On such a run the escape via could only reach forbidden copper, and when the
net still failed the rescue shipped it. Found on #1260's board: `route.py
--nets SIG --layers F.Cu` failed SIG and left a via-in-pad down to B.Cu on
each of SIG's two pads. The same happened with a keep-out across the route
instead of a pour.

`net_rescue.escape_has_routable_layer` now gates the escape: the pad's
layer must be routable, and so must another layer for the via to reach.

Rows (the boards are written here from s-expressions; the suite image has
no KiCad):
  * route.py on the 2-layer board with a ZONE_NET pour across the route,
    and on the same board with an F.Cu keep-out instead: SIG fails, and the
    written board has no via and no copper on B.Cu;
  * control: a keep-out across BOTH layers with `--layers F.Cu B.Cu`. SIG
    still fails, and the rescue still lays its escape vias, because both
    layers are routable (and, #1778, withdraws them, since they bought
    nothing);
  * the predicate itself, in process: forbidden other layers, a pad on a
    forbidden layer, two routable layers, and a 4-layer run whose through
    via spans forbidden inner layers.

    python3 tests/test_666_escape_routable_layers.py
"""
import json
import os
import re
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

import run_utils                                              # noqa: E402
from test_1260_pour_guard import write_board, _HEAD, _edge, _pad_fp  # noqa: E402

RUN_ALL_TIMEOUT = 900
ROUTE = run_utils.tool('route.py')
PY = [sys.executable, '-X', 'utf8']
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}"
          + (f" ({detail})" if detail else ''))
    if not ok:
        failures.append(label)


def _keepout(layers):
    lay = ' '.join(f'"{l}"' for l in layers)
    return ('\t(zone\n\t\t(layers %s)\n\t\t(name "KO")\n\t\t(hatch edge 0.5)\n'
            '\t\t(connect_pads\n\t\t\t(clearance 0)\n\t\t)\n'
            '\t\t(min_thickness 0.25)\n\t\t(keepout\n'
            '\t\t\t(tracks not_allowed)\n\t\t\t(vias not_allowed)\n'
            '\t\t\t(pads allowed)\n\t\t\t(copperpour not_allowed)\n'
            '\t\t\t(footprints allowed)\n\t\t)\n'
            '\t\t(fill\n\t\t\t(thermal_gap 0.5)\n'
            '\t\t\t(thermal_bridge_width 0.5)\n\t\t)\n'
            '\t\t(polygon\n\t\t\t(pts\n\t\t\t\t(xy 14 -1) (xy 26 -1) '
            '(xy 26 31) (xy 14 31)\n\t\t\t)\n\t\t)\n\t)\n' % lay)


def write_keepout_board(path, layers):
    parts = [_HEAD, _pad_fp('J1', 5, 15, 'SIG'), _pad_fp('J2', 35, 15, 'SIG')]
    corners = [(0, 0), (40, 0), (40, 30), (0, 30)]
    for i in range(4):
        parts.append(_edge(*corners[i], *corners[(i + 1) % 4]))
    parts.append(_keepout(layers))
    parts.append(')\n')
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(''.join(parts))
    return run_utils.evidence(path, 'keep-out board')


def route(src, out, layers):
    r = run_utils.check(PY + [ROUTE, src, out, '--nets', 'SIG', '--layers']
                        + layers + ['--escalation', 'off'], accept=True)
    log = r.stdout + r.stderr
    m = re.findall(r'JSON_SUMMARY_MIN: (\{.*\})', log)
    if not m:
        raise AssertionError(f'BROKEN TEST: no JSON_SUMMARY_MIN\n{log[-1500:]}')
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(run_utils.evidence(out, 'routed board'))
    vias = [(v.x, v.y) for v in pcb.vias]
    on_bcu = sum(1 for s in pcb.segments if s.layer == 'B.Cu')
    return json.loads(m[-1]), vias, on_bcu, log


def test_end_to_end(tmp):
    pour = write_board(os.path.join(tmp, 'pour.kicad_pcb'), 'full')
    ko_f = write_keepout_board(os.path.join(tmp, 'ko_f.kicad_pcb'), ['F.Cu'])
    for name, src in (('pour across the route', pour),
                      ('F.Cu keep-out across the route', ko_f)):
        mins, vias, on_bcu, log = route(
            src, os.path.join(tmp, f'{len(name)}_out.kicad_pcb'), ['F.Cu'])
        check(f"{name}, --layers F.Cu: SIG fails",
              mins.get('failed_single') == ['SIG'], str(mins.get('failed_single')))
        check(f"{name}: the failed net leaves no via behind", not vias,
              str(vias))
        check(f"{name}: and no track on B.Cu", on_bcu == 0, str(on_bcu))
        check(f"{name}: the rescue says why it laid no escape",
              'skipped (no routable layer for its via to reach' in log)

    ko_fb = write_keepout_board(os.path.join(tmp, 'ko_fb.kicad_pcb'),
                                ['F.Cu', 'B.Cu'])
    mins, vias, on_bcu, log = route(ko_fb, os.path.join(tmp, 'fb_out.kicad_pcb'),
                                    ['F.Cu', 'B.Cu'])
    check("control: with both layers routable and both blocked, SIG fails",
          mins.get('failed_single') == ['SIG'], str(mins))
    check("control: and the rescue still lays its escape vias",
          'dogbone' in log and 'skipped (no routable layer' not in log)
    # #1778: the escapes bought nothing (SIG is as open as before), so the
    # rescue takes them back out rather than ship vias that join nothing
    check("control: and withdraws them when they bought nothing",
          not vias and 'bare-ball escape via withdrawn' in log, str(vias))


def test_predicate():
    from net_rescue import escape_has_routable_layer
    from routing_config import GridRouteConfig

    class _Pad:
        def __init__(self, layers):
            self.layers = layers

    def cfg(layers, costs):
        return GridRouteConfig(layers=layers, layer_costs=costs)

    f = _Pad(['F.Cu', 'F.Mask', 'F.Paste'])
    b = _Pad(['B.Cu', 'B.Mask'])
    check("F.Cu pad, B.Cu forbidden: no escape",
          not escape_has_routable_layer(f, cfg(['F.Cu', 'B.Cu'], [1.0, -1])))
    check("B.Cu pad on a forbidden layer: no escape",
          not escape_has_routable_layer(b, cfg(['F.Cu', 'B.Cu'], [1.0, -1])))
    check("two routable layers: escape allowed",
          escape_has_routable_layer(f, cfg(['F.Cu', 'B.Cu'], [1.0, 3.0])))
    check("4 layers, F.Cu and In1.Cu routable: escape allowed (the through "
          "via spans forbidden In2/B.Cu, as a routed via may)",
          escape_has_routable_layer(f, cfg(
              ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'], [1.0, 1.0, -1, -1])))
    check("no layer costs at all: every layer routable",
          escape_has_routable_layer(f, cfg(['F.Cu', 'B.Cu'], [])))


def main():
    tmp = tempfile.mkdtemp(prefix='t666l_')
    try:
        print("route.py end to end")
        test_end_to_end(tmp)
        try:
            from net_rescue import escape_has_routable_layer  # noqa: F401
        except ImportError as e:
            check("net_rescue.escape_has_routable_layer exists", False, str(e))
        else:
            print("the predicate")
            test_predicate()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        print(f"\nFAILED {len(failures)}: " + '; '.join(failures))
        return 1
    print("\nall escape-layer checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())

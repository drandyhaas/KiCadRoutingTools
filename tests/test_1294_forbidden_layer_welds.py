#!/usr/bin/env python3
"""No weld track on a layer the route may not use, from a one-cell path (#1294).

#1292 fixed the pour-launch welds. Two more sources left 30-100 um tracks on
the inner planes of 4-layer boards routed `--layers F.Cu B.Cu` (2 on
kit-dev-coldfire-xilinx_5213, 10 on glasgow_revC). Every one was a one-cell
path: the source and the target shared a grid cell that a through via offers
on every layer, so the A* stood on the plane layer, and the converter welded
both terminals to that cell on it.

1. A stub terminal the router makes AT a via (#545 F4 takes a via within
   0.05 mm of a group's track end as its representative) resolved to its
   component only through a track end within 0.01 mm. coldfire's +3.3V via
   sits 30 um off the grid point its B.Cu and F.Cu tracks end on, so each
   layer's group became a terminal at the via centre in a singleton
   component, apart from the island that holds the via, and the router
   planned a zero-length edge between them: a weld from the via centre to
   its grid point and back, 29 times per run (glasgow's /SDA and /~{ALERT}
   too). get_terminal_component_info now resolves such a terminal through
   the via's own graph membership.
2. A pad-via rescue drops a via in the target pad and offers its cell on
   every layer, so a later tap can land on the plane layer at that cell and
   weld to the pad centre there (glasgow's U4.1, U30.D6, C57.2, D21.5,
   D21.10, D23.10). A bridge on a forbidden layer whose two ends lie in one
   through via of the net now draws nothing: that via already joins the
   copper at both ends on every layer.

Checks:
  A. labelling: the two stub terminals at the off-grid via share the
     island's component (3 groups, not 4), and no two terminals at one
     point sit in different components
  B. converter: a one-cell path on a forbidden layer inside a through via
     draws no bridge (both glasgow shapes, at their measured geometry); the
     same path on a routable layer, an end outside the via, and a blind via
     keep theirs
  C. routed, the coldfire junction with a track keepout over it, routed
     `--layers F.Cu B.Cu` twice: no zero-length MST edge planned, Phase 1
     routes a real edge, no stacked copper, no track off --layers, every
     net connected, identical copper. Before the fix: a zero-length edge,
     two Phase 1 edges failed at 5004 iterations each, and a stacked pair
     at (6.572, 10.610)-(6.600, 10.600).

    python3 tests/test_1294_forbidden_layer_welds.py
"""
import contextlib
import io
import json
import math
import os
import re
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from run_utils import evidence, tool                   # noqa: E402

FAILS = []
ROUTED = ('F.Cu', 'B.Cu')
# route.py appends the copper layers left out of --layers at the forbidden cost.
ALL_LAYERS = ['F.Cu', 'B.Cu', 'In1.Cu', 'In2.Cu']
FORBID = [1, 3, -1, -1]
ALLOW = [1, 1, 1, 1]
VIA = (6.5717, 10.6096)          # coldfire's +3.3V via, 30 um off its grid point
GRID_PT = (6.6, 10.6)


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def quiet(fn, *a, **kw):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **kw)


_N = [0]


def _uid():
    _N[0] += 1
    return '00000000-0000-4000-8000-%012d' % _N[0]


def _header(nets):
    return ('(kicad_pcb\n\t(version 20260206)\n\t(generator "test_1294")\n'
            '\t(generator_version "10.0")\n'
            '\t(general (thickness 1.6) (legacy_teardrops no))\n\t(paper "A4")\n'
            '\t(layers\n\t\t(0 "F.Cu" signal)\n\t\t(4 "In1.Cu" signal)\n'
            '\t\t(6 "In2.Cu" signal)\n\t\t(2 "B.Cu" signal)\n'
            '\t\t(1 "F.Mask" user)\n\t\t(3 "B.Mask" user)\n'
            '\t\t(25 "Edge.Cuts" user)\n\t)\n'
            '\t(setup (pad_to_mask_clearance 0))\n\t(net 0 "")\n'
            + ''.join(f'\t(net {i + 1} "{n}")\n' for i, n in enumerate(nets)))


def _smd(ref, x, y, layer, net, size=(0.8, 0.8)):
    mask = 'F.Mask' if layer == 'F.Cu' else 'B.Mask'
    return (f'\t(footprint "test:P" (layer "{layer}") (uuid "{_uid()}") (at {x} {y})\n'
            f'\t\t(property "Reference" "{ref}" (at 0 -1 0) (layer "F.Fab") '
            f'(uuid "{_uid()}") (effects (font (size 0.5 0.5) (thickness 0.1))))\n'
            f'\t\t(pad "1" smd rect (at 0 0) (size {size[0]} {size[1]}) '
            f'(layers "{layer}" "{mask}") (net "{net}") (uuid "{_uid()}"))\n\t)\n')


def _seg(x1, y1, x2, y2, layer, net, w=0.3):
    return (f'\t(segment (start {x1} {y1}) (end {x2} {y2}) (width {w}) '
            f'(layer "{layer}") (net "{net}") (uuid "{_uid()}"))\n')


def _edge():
    return (f'\t(gr_rect (start 0 0) (end 20 16) (stroke (width 0.05) (type default)) '
            f'(fill no) (layer "Edge.Cuts") (uuid "{_uid()}"))\n')


def junction_board(keepout):
    """kit-dev-coldfire's +3.3V junction: a B.Cu track from C1 and a dangling
    F.Cu track meet at a grid point 30 um off an off-grid through via; two
    more pads to route to. With `keepout`, a track keepout over the grid
    point on both routable layers (what made the old labelling's stub-only
    launch fail)."""
    p = [_header(['VCC']),
         _smd('C1', 6.0, 11.2, 'B.Cu', 'VCC', (1.0, 1.45)),
         _seg(6.0, 11.2, *GRID_PT, 'B.Cu', 'VCC'),
         f'\t(via (at {VIA[0]} {VIA[1]}) (size 0.5) (drill 0.3) (layers "F.Cu" "B.Cu") '
         f'(net "VCC") (uuid "{_uid()}"))\n',
         _seg(*GRID_PT, 6.5, 10.5, 'F.Cu', 'VCC'),
         _seg(6.5, 10.5, 6.4592, 9.8076, 'F.Cu', 'VCC'),
         _smd('R1', 14.0, 6.0, 'F.Cu', 'VCC'),
         _smd('R2', 3.0, 4.0, 'F.Cu', 'VCC')]
    if keepout:
        p.append(f'\t(zone (net 0) (net_name "") (layers "F.Cu" "B.Cu") (uuid "{_uid()}")\n'
                 '\t\t(hatch edge 0.5) (connect_pads (clearance 0)) (min_thickness 0.25)\n'
                 '\t\t(keepout (tracks not_allowed) (vias allowed) (pads allowed) '
                 '(copperpour allowed) (footprints allowed))\n'
                 '\t\t(fill (thermal_gap 0.5) (thermal_bridge_width 0.5))\n'
                 '\t\t(polygon (pts (xy 6.45 10.45) (xy 6.75 10.45) (xy 6.75 10.75) '
                 '(xy 6.45 10.75))))\n')
    p.append(_edge() + ')\n')
    return ''.join(p)


def vias_board():
    """glasgow_revC's two pad-tap vias at their measured geometry, plus a
    blind via of the same net, for the converter checks."""
    return (_header(['+3V3'])
            + '\t(via (at 95.1 84.95) (size 0.4) (drill 0.25) (layers "F.Cu" "B.Cu") '
              f'(net "+3V3") (uuid "{_uid()}"))\n'
            + '\t(via (at 85.45 95.5) (size 0.35) (drill 0.2) (layers "F.Cu" "B.Cu") '
              f'(net "+3V3") (uuid "{_uid()}"))\n'
            + '\t(via blind (at 60.0 60.0) (size 0.4) (drill 0.2) (layers "F.Cu" "In1.Cu") '
              f'(net "+3V3") (uuid "{_uid()}"))\n'
            + '\t(gr_rect (start 50 50) (end 120 120) (stroke (width 0.05) (type default)) '
              f'(fill no) (layer "Edge.Cuts") (uuid "{_uid()}"))\n)\n')


def _write(d, name, text):
    path = os.path.join(d, name)
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text)
    return evidence(path, name) or path


def t_labelling(board):
    print('A. stub terminals at an off-grid via join the island holding it')
    from kicad_parser import parse_kicad_pcb
    from routing_config import GridRouteConfig
    from connectivity import get_multipoint_net_pads, get_terminal_component_info
    pcb = quiet(parse_kicad_pcb, board)
    net = next(n for n, v in pcb.nets.items() if v.name == 'VCC')
    cfg = GridRouteConfig(layers=list(ALL_LAYERS), layer_costs=list(FORBID))
    pi = quiet(get_multipoint_net_pads, pcb, net, cfg)
    at_via = [i for i, p in enumerate(pi or [])
              if math.hypot(p[3] - VIA[0], p[4] - VIA[1]) < 1e-6]
    check('the router makes two stub terminals at the via (the case under test)',
          len(at_via) == 2, f'{len(at_via)} of {len(pi or [])} terminals')
    if len(at_via) != 2:
        return
    comps, _, _, vias_by = get_terminal_component_info(pcb, net, pi)
    via_comp = next((c for c, vs in vias_by.items()
                     if any(abs(v.x - VIA[0]) < 1e-6 for v in vs)), None)
    check('both stub terminals are labelled with the via\'s own component',
          all(comps[i] == via_comp for i in at_via),
          f'{[comps[i] for i in at_via]} vs via {via_comp}')
    groups = len(set(comps.values()))
    check('4 terminals in 3 groups (R1, R2, the island)', groups == 3, f'{groups} groups')
    zero = [(i, j) for i in range(len(pi)) for j in range(i + 1, len(pi))
            if comps[i] != comps[j]
            and math.hypot(pi[i][3] - pi[j][3], pi[i][4] - pi[j][4]) < 1e-6]
    check('no two terminals at one point in different components', not zero, f'{zero}')


def t_converter(board):
    print('B. the converter draws no bridge on a forbidden layer inside a through via')
    from kicad_parser import parse_kicad_pcb
    from routing_config import GridRouteConfig, GridCoord
    from single_ended_routing import _path_to_segments_vias
    pcb = quiet(parse_kicad_pcb, board)
    net = next(n for n, v in pcb.nets.items() if v.name == '+3V3')
    coord = GridCoord(0.1)
    in2 = ALL_LAYERS.index('In2.Cu')
    in1 = ALL_LAYERS.index('In1.Cu')

    def segs(costs, cell, start, end):
        cfg = GridRouteConfig(layers=list(ALL_LAYERS), layer_costs=list(costs))
        s, v = _path_to_segments_vias([cell], coord, ALL_LAYERS, net, cfg,
                                      start, end, set(), pcb)
        return [(x.layer, round(x.start_x, 4), round(x.start_y, 4),
                 round(x.end_x, 4), round(x.end_y, 4)) for x in s], v

    # glasgow U4.1: the rescue via at (95.1, 84.95), the tap's cell (951, 850)
    # on In2.Cu, welded to U4.1's centre (95.15, 85.05), 0.112 mm from the via.
    u4 = ((951, 850, in2), (95.1, 85.0, 'In2.Cu'), (95.15, 85.05, 'In2.Cu'))
    s, v = segs(FORBID, *u4)
    check('U4.1 shape, In2.Cu forbidden: no bridge, no via', not s and not v, f'{s} {v}')
    s, _ = segs(ALLOW, *u4)
    check('the same path on a routable In2.Cu keeps its bridge',
          s == [('In2.Cu', 95.1, 85.0, 95.15, 85.05)], f'{s}')
    # glasgow U30.D6: via (85.45, 95.5) r 0.175, the pad centre 0.15 off it.
    s, _ = segs(FORBID, (854, 955, in2), (85.4, 95.5, 'In2.Cu'), (85.3, 95.5, 'In2.Cu'))
    check('U30.D6 shape, In2.Cu forbidden: no bridge', not s, f'{s}')
    # The weld point 0.25 mm off the via, outside its 0.2 mm disc: kept.
    s, _ = segs(FORBID, (951, 850, in2), (95.1, 85.0, 'In2.Cu'), (95.35, 84.95, 'In2.Cu'))
    check('an end outside the via\'s copper keeps its bridge',
          s == [('In2.Cu', 95.1, 85.0, 95.35, 84.95)], f'{s}')
    # A blind F.Cu-In1.Cu via does not reach B.Cu: kept.
    s, _ = segs(FORBID, (600, 600, in1), (60.0, 60.0, 'In1.Cu'), (60.05, 60.05, 'In1.Cu'))
    check('a blind via keeps its bridge',
          s == [('In1.Cu', 60.0, 60.0, 60.05, 60.05)], f'{s}')


def t_routed(board, d):
    print('C. the coldfire junction, routed --layers F.Cu B.Cu')
    from kicad_parser import parse_kicad_pcb

    copper = []
    for run in (1, 2):
        out = os.path.join(d, f'run{run}.kicad_pcb')
        js = os.path.join(d, f'run{run}.json')
        r = subprocess.run([sys.executable, '-X', 'utf8', tool('route.py'), board,
                            '--output', out, '--json-out', js, '--nets', '*',
                            '--layers', *ROUTED],
                           cwd=ROOT, capture_output=True, text=True, timeout=900)
        check(f'run {run}: route.py exits 0', r.returncode == 0)
        if r.returncode != 0:
            print(r.stdout[-3000:], r.stderr[-3000:])
            return
        pcb = quiet(parse_kicad_pcb, evidence(out, 'routed board'))
        segs = sorted((round(s.start_x, 4), round(s.start_y, 4), round(s.end_x, 4),
                       round(s.end_y, 4), s.width, s.layer) for s in pcb.segments)
        vias = sorted((round(v.x, 4), round(v.y, 4), v.size) for v in pcb.vias)
        copper.append((segs, vias))
        if run == 1:
            m = re.search(r'Existing copper joins (\d+) terminals into (\d+) group', r.stdout)
            check('the junction is one group (4 terminals, 3 groups)',
                  bool(m) and m.groups() == ('4', '3'), m.group(0) if m else 'no line')
            check('no zero-length MST edge is planned', 'length=0.00mm' not in r.stdout)
            failed = re.findall(r'Phase 1 edge \(pads [0-9,]+\) failed after \d+', r.stdout)
            check('Phase 1 routes its first edge', not failed, f'{failed}')
            with open(evidence(js, 'route summary'), encoding='utf-8') as f:
                summ = json.load(f)
            stacked = summ.get('stacked_copper') or []
            check('no stacked copper', not stacked, f'{stacked[:2]}')
            off = [s for s in segs if s[5] not in ROUTED]
            check('no track on a layer outside --layers', not off, f'{off[:4]}')
            c = subprocess.run([sys.executable, '-X', 'utf8', tool('check_connected.py'), out],
                               cwd=ROOT, capture_output=True, text=True, timeout=300)
            check('every net connected',
                  c.returncode == 0 and 'ALL NETS FULLY CONNECTED' in c.stdout)
    check('a second run lays identical copper', copper[0] == copper[1])


if __name__ == '__main__':
    with tempfile.TemporaryDirectory(prefix='t1294_') as d:
        plain = _write(d, 'junction.kicad_pcb', junction_board(keepout=False))
        boxed = _write(d, 'junction_keepout.kicad_pcb', junction_board(keepout=True))
        vias = _write(d, 'vias.kicad_pcb', vias_board())
        t_labelling(plain)
        t_converter(vias)
        t_routed(boxed, d)
    print()
    if FAILS:
        print(f'FAILED: {len(FAILS)} check(s)')
        for f in FAILS:
            print('  - ' + f)
        sys.exit(1)
    print('ALL PASS')

#!/usr/bin/env python3
"""POUR-LAUNCH welds draw no bridge on a layer the route may not use (#1292).

A pour-launch cell is keyed on the routing grid, but its weld point was the
float of the fill-raster sample it came from, so the converter drew a
sub-cell bridge from the grid point to that sample on the ZONE's layer. When
that layer is one the config forbids track on (an inner GND plane left out of
--layers), the route reaches the cell only by a via at the grid point, and
the bridge was a 20-56 um GND track on that layer, inside the via's own
copper: 34 of them on In1.Cu on the reporter's 4-layer board routed
`--layers F.Cu B.Cu`. Both producers (_pour_launch_region_cells and
_pour_launch_pair_anchors) now weld a forbidden-layer cell at its grid point;
on a routable layer the weld stays the fill float, so the bridge still lands
ON fill copper. The forbidden set joins the region memo's keys, because
Phase 3's per-net power config can forbid a layer Phase 1 allowed.

The board (built here, after the reporter's): a 20x16 mm 4-layer outline, a
THT GND test point, three 0402s with a GND pad on F.Cu, a GND zone on In1.Cu.

Checks:
  A. _track_forbidden_layers is exactly the negative-cost layers
  B. region cells: forbidden-layer cells weld at their grid point, routable
     ones keep their fill float; the same grouping object asked under a
     different forbidden set does not get the other set's memoized welds
  C. pair anchors: the same rule
  D. route.py --layers F.Cu B.Cu, twice: pour-launch fired and a via reached
     the plane (the path under test), no track off --layers, every net
     connected, identical copper. Before the fix: one 20 um In1.Cu track at
     (11.5, 5.98)-(11.5, 6.0)

    python3 tests/test_1292_pour_launch_forbidden_welds.py
"""
import contextlib
import io
import os
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
IN1 = ALL_LAYERS.index('In1.Cu')
FORBID = [1, 1, -1, -1]
ALLOW = [1, 1, 1, 1]


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


def board_text():
    def smd(ref, x, y):
        pads = ''.join(
            f'\t\t(pad "{i + 1}" smd roundrect (at {dx} 0) (size 0.6 0.6) '
            f'(layers "F.Cu" "F.Mask") (roundrect_rratio 0.25) (net "{net}") '
            f'(uuid "{_uid()}"))\n'
            for i, (dx, net) in enumerate(((-0.5, 'GND'), (0.5, 'SIG'))))
        return (f'\t(footprint "test:R_0402" (layer "F.Cu") (uuid "{_uid()}") '
                f'(at {x} {y})\n'
                f'\t\t(property "Reference" "{ref}" (at 0 -1 0) (layer "F.Fab") '
                f'(uuid "{_uid()}") (effects (font (size 0.5 0.5) '
                f'(thickness 0.1))))\n{pads}\t)\n')

    parts = [
        '(kicad_pcb\n\t(version 20260206)\n\t(generator "test_1292")\n'
        '\t(generator_version "10.0")\n'
        '\t(general (thickness 1.6) (legacy_teardrops no))\n\t(paper "A4")\n'
        '\t(layers\n\t\t(0 "F.Cu" signal)\n\t\t(4 "In1.Cu" signal)\n'
        '\t\t(6 "In2.Cu" signal)\n\t\t(2 "B.Cu" signal)\n'
        '\t\t(1 "F.Mask" user)\n\t\t(3 "B.Mask" user)\n'
        '\t\t(25 "Edge.Cuts" user)\n\t)\n'
        '\t(setup (pad_to_mask_clearance 0))\n'
        '\t(net 0 "")\n\t(net 1 "GND")\n\t(net 2 "SIG")\n',
        f'\t(footprint "test:TP" (layer "F.Cu") (uuid "{_uid()}") (at 3 3)\n'
        f'\t\t(property "Reference" "J1" (at 0 -1.5 0) (layer "F.Fab") '
        f'(uuid "{_uid()}") (effects (font (size 0.5 0.5) (thickness 0.1))))\n'
        f'\t\t(pad "1" thru_hole circle (at 0 0) (size 1.7 1.7) (drill 1) '
        f'(layers "*.Cu" "*.Mask") (remove_unused_layers no) (net "GND") '
        f'(uuid "{_uid()}"))\n\t)\n',
    ]
    parts += [smd(f'R{k + 1}', x, y)
              for k, (x, y) in enumerate(((12, 6), (15, 11), (8, 12)))]
    parts.append(
        f'\t(gr_rect (start 0 0) (end 20 16) (stroke (width 0.05) (type default)) '
        f'(fill no) (layer "Edge.Cuts") (uuid "{_uid()}"))\n'
        f'\t(zone (net "GND") (layer "In1.Cu") (uuid "{_uid()}") (hatch edge 0.5)\n'
        '\t\t(connect_pads yes (clearance 0.2)) (min_thickness 0.16)\n'
        '\t\t(fill yes (thermal_gap 0.5) (thermal_bridge_width 0.5) '
        '(island_removal_mode 0))\n'
        '\t\t(polygon (pts (xy 0.3 0.3) (xy 19.7 0.3) (xy 19.7 15.7) (xy 0.3 15.7)))\n'
        '\t)\n)\n')
    return ''.join(parts)


def _gnd(pcb):
    return next(n for n, net in pcb.nets.items() if net.name == 'GND')


def _cfg(costs):
    from routing_config import GridRouteConfig
    return GridRouteConfig(layers=list(ALL_LAYERS), layer_costs=list(costs))


def _in1_welds(cells, coord):
    """(on_grid, off_grid) counts of the In1.Cu entries of a region-cell map."""
    rows = [(k, v) for cmap in cells.values() for k, v in cmap.items() if k[2] == IN1]
    on = sum(1 for k, v in rows if v == coord.to_float(k[0], k[1]))
    return on, len(rows) - on


def t_forbidden_set():
    print('A. the forbidden set')
    from single_ended_routing import _track_forbidden_layers
    check('forbidden layers are the cost < 0 ones',
          _track_forbidden_layers(_cfg([1, 1.5, -1, -1]), ALL_LAYERS) == {2, 3})
    check('no layer costs -> nothing forbidden',
          _track_forbidden_layers(_cfg([]), ALL_LAYERS) == frozenset())
    check('a layer the config does not list is not forbidden',
          _track_forbidden_layers(_cfg(FORBID), ['F.Cu', 'X.Cu']) == frozenset())


def t_region_cells(board):
    print('B. region cells (multipoint ladder)')
    from kicad_parser import parse_kicad_pcb
    from routing_config import GridCoord
    from connectivity import get_multipoint_net_pads, get_terminal_component_info
    from single_ended_routing import _pour_launch_region_cells

    pcb = quiet(parse_kicad_pcb, board)
    net_id = _gnd(pcb)
    coord = GridCoord(_cfg(ALLOW).grid_step)
    pad_info = quiet(get_multipoint_net_pads, pcb, net_id, _cfg(ALLOW))
    comps = get_terminal_component_info(pcb, net_id, pad_info)[0]

    def cells(costs):
        return quiet(_pour_launch_region_cells, pcb, net_id, pad_info, comps,
                     list(ALL_LAYERS), coord, {}, _cfg(costs))

    # Same pcb and SAME grouping object across the three calls: the identity
    # memo would hand back the previous call's welds if the forbidden set
    # were not part of its key.
    on, off = _in1_welds(cells(ALLOW), coord)
    check('routable In1.Cu: pour-launch offers fill cells', on + off > 0, f'{on + off}')
    check('routable In1.Cu: cells keep their fill float (bridge lands on fill)',
          off > 0, f'{off} of {on + off} off grid')
    on, off = _in1_welds(cells(FORBID), coord)
    check('forbidden In1.Cu: pour-launch still offers fill cells', on + off > 0)
    check('forbidden In1.Cu: every cell welds at its grid point', off == 0,
          f'{off} of {on + off} off grid')
    on, off = _in1_welds(cells(ALLOW), coord)
    check('routable again on the same grouping: the fill floats come back', off > 0)


def t_pair_anchors(board):
    print('C. pair anchors (2-pad endpoint path)')
    from kicad_parser import parse_kicad_pcb
    from routing_config import GridCoord
    from single_ended_routing import _pour_launch_pair_anchors

    pcb = quiet(parse_kicad_pcb, board)
    net_id = _gnd(pcb)

    def rows(costs):
        cfg = _cfg(costs)
        coord = GridCoord(cfg.grid_step)

        def row(x, y):
            g = coord.to_grid(x, y)
            return (g[0], g[1], 0, x, y)
        # R1.1 (near) -> J1.1 (the THT pad on the plane)
        _, extra_t = quiet(_pour_launch_pair_anchors, pcb, net_id, [row(11.5, 6.0)],
                           [row(3.0, 3.0)], cfg.layers, coord, cfg)
        r = [x for x in extra_t if x[2] == IN1]
        return r, sum(1 for x in r if (x[3], x[4]) != coord.to_float(x[0], x[1]))

    r, off = rows(FORBID)
    check('forbidden In1.Cu: anchors offered', len(r) > 0, f'{len(r)}')
    check('forbidden In1.Cu: every anchor welds at its grid point', off == 0,
          f'{off} of {len(r)} off grid')
    r, off = rows(ALLOW)
    check('routable In1.Cu: anchors keep their fill float', off > 0,
          f'{off} of {len(r)} off grid')


def t_routed(board, d):
    print('D. route.py --layers F.Cu B.Cu')
    from kicad_parser import parse_kicad_pcb

    copper = []
    for run in (1, 2):
        out = os.path.join(d, f'run{run}.kicad_pcb')
        r = subprocess.run([sys.executable, '-X', 'utf8', tool('route.py'), board,
                            '--output', out, '--nets', '*', '--layers', *ROUTED],
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
            check('pour-launch fired (the path under test)', 'POUR-LAUNCH v3' in r.stdout)
            gnd = _gnd(pcb)
            check('a GND via reaches the In1.Cu plane',
                  any(v.net_id == gnd for v in pcb.vias))
            off = [s for s in segs if s[5] not in ROUTED]
            check('no track on a layer outside --layers', not off, f'{off[:4]}')
            c = subprocess.run([sys.executable, '-X', 'utf8', tool('check_connected.py'), out],
                               cwd=ROOT, capture_output=True, text=True, timeout=300)
            check('every net connected',
                  c.returncode == 0 and 'ALL NETS FULLY CONNECTED' in c.stdout)
    check('a second run lays identical copper', copper[0] == copper[1])


if __name__ == '__main__':
    with tempfile.TemporaryDirectory(prefix='t1292_') as d:
        board = os.path.join(d, 'plane4.kicad_pcb')
        with open(board, 'w', encoding='utf-8', newline='\n') as f:
            f.write(board_text())
        evidence(board, 'staged board')
        t_forbidden_set()
        t_region_cells(board)
        t_pair_anchors(board)
        t_routed(board, d)
    print()
    if FAILS:
        print(f'FAILED: {len(FAILS)} check(s)')
        for f in FAILS:
            print('  - ' + f)
        sys.exit(1)
    print('ALL PASS')

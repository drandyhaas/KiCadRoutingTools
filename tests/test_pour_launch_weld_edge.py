#!/usr/bin/env python3
"""POUR-LAUNCH welds clear the board edge as a routing-grid cell would.

A pour-launch cell is keyed on the routing grid, but the converter ends the
route's track at the cell's WELD point -- the fill-raster sample's float --
with a sub-cell bridge from the grid point it searched. The fill holds the
zone's own edge clearance, so a weld near the fill's boundary there ends a
track half its width nearer the edge than the A* grid would ever let it go.
smartknob_base's route step laid GND (113.75, 109.15)-(113.80, 109.20) on
B.Cu 0.042 mm into its 0.5 mm board-edge band that way, on main as well as
on the fix branch, once its diff-pair step's copper changed. Both producers
(_pour_launch_region_cells, _pour_launch_pair_anchors) now skip a weld that
does not clear the board edge by edge clearance + track width / 2
(obstacle_map.track_edge_point_test, the keep-out add_board_edge_obstacles
stamps), and a rung counts only once a candidate clears; the width joins the
region memo's keys, since a per-net power config changes it.

The board (built here): a 20x16 mm 2-layer outline, a B.Cu GND pour that is
a 0.5 mm strip along the left edge (x 0.3-0.8, as on a ring board's rim), a
GND pad on it and two F.Cu GND pads 2 mm in from the edge beside it.

Checks:
  A. track_edge_point_test: the band is edge clearance + width/2, exact at a
     float point; an inner cut-out counts; edge clearance 0 falls back to
     the track clearance
  B. region cells: with the test removed, welds land in the band (the
     precondition); with it, none does and cells are still offered
  C. pair anchors: the same
  D. a wider track on the SAME grouping object does not get the narrower
     track's memoized welds

    python3 tests/test_pour_launch_weld_edge.py
"""
import contextlib
import io
import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from run_utils import evidence                          # noqa: E402

FAILS = []
LAYERS = ['F.Cu', 'B.Cu']
EDGE, WIDTH = 0.5, 0.2
STRIP = 0.8          # the pour's inner edge; its outer edge is 0.3 off the outline
SX = 2.0             # the beside-the-pour pads


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
    def pad_fp(ref, x, y, layer):
        return (f'\t(footprint "test:P" (layer "{layer}") (uuid "{_uid()}") (at {x} {y})\n'
                f'\t\t(property "Reference" "{ref}" (at 0 -1 0) (layer "F.Fab") '
                f'(uuid "{_uid()}") (effects (font (size 0.5 0.5) (thickness 0.1))))\n'
                f'\t\t(pad "1" smd rect (at 0 0) (size 0.4 0.4) '
                f'(layers "{layer}" "{layer[0]}.Mask") (net "GND") (uuid "{_uid()}"))\n\t)\n')

    return ('(kicad_pcb\n\t(version 20260206)\n\t(generator "test_weld_edge")\n'
            '\t(generator_version "10.0")\n'
            '\t(general (thickness 1.6) (legacy_teardrops no))\n\t(paper "A4")\n'
            '\t(layers\n\t\t(0 "F.Cu" signal)\n\t\t(2 "B.Cu" signal)\n'
            '\t\t(1 "F.Mask" user)\n\t\t(3 "B.Mask" user)\n\t\t(25 "Edge.Cuts" user)\n\t)\n'
            '\t(setup (pad_to_mask_clearance 0))\n\t(net 0 "")\n\t(net 1 "GND")\n'
            + pad_fp('J1', (0.3 + STRIP) / 2, 3, 'B.Cu')
            + pad_fp('TP2', SX, 8, 'F.Cu') + pad_fp('TP3', SX, 12, 'F.Cu')
            + f'\t(gr_rect (start 0 0) (end 20 16) (stroke (width 0.05) (type default)) '
            f'(fill no) (layer "Edge.Cuts") (uuid "{_uid()}"))\n'
            f'\t(zone (net "GND") (layer "B.Cu") (uuid "{_uid()}") (hatch edge 0.5)\n'
            '\t\t(connect_pads yes (clearance 0.2)) (min_thickness 0.16)\n'
            '\t\t(fill yes (thermal_gap 0.5) (thermal_bridge_width 0.5) '
            '(island_removal_mode 0))\n'
            f'\t\t(polygon (pts (xy 0.3 0.3) (xy {STRIP} 0.3) (xy {STRIP} 15.7) '
            '(xy 0.3 15.7)))\n\t)\n)\n')


def _cfg(width=WIDTH, edge=EDGE, clearance=0.1):
    from routing_config import GridRouteConfig
    return GridRouteConfig(layers=list(LAYERS), track_width=width,
                           board_edge_clearance=edge, clearance=clearance)


def _edge_dist(x, y):
    return min(x, y, 20 - x, 16 - y)


def _in_band(points, width=WIDTH):
    need = EDGE + width / 2
    return [p for p in points if _edge_dist(*p) < need - 1e-6]


@contextlib.contextmanager
def _edge_test_removed():
    """The producers import track_edge_point_test at call time: without it
    they weld where they did before the fix."""
    import obstacle_map
    real = obstacle_map.track_edge_point_test
    obstacle_map.track_edge_point_test = lambda *a, **k: None
    try:
        yield
    finally:
        obstacle_map.track_edge_point_test = real


def t_point_test(pcb):
    print('A. track_edge_point_test')
    from obstacle_map import track_edge_point_test
    ok = track_edge_point_test(pcb, _cfg())
    check('the band is edge clearance + width/2', abs(ok.need - 0.6) < 1e-9, f'{ok.need}')
    check('0.59 mm off the outline is inside it', not ok(0.59, 8.0))
    check('0.61 mm off the outline clears it', ok(0.61, 8.0))
    check('a wider track has a wider band', not track_edge_point_test(pcb, _cfg(width=0.4))(0.65, 8.0))
    fb = track_edge_point_test(pcb, _cfg(edge=0.0, clearance=0.15))
    check('edge clearance 0 falls back to the track clearance', abs(fb.need - 0.25) < 1e-9, f'{fb.need}')
    pcb.board_info.board_cutouts = [[(9, 7), (11, 7), (11, 9), (9, 9)]]
    try:
        cut = track_edge_point_test(pcb, _cfg())
        check('an inner cut-out counts', not cut(10.0, 9.5) and cut(10.0, 9.7))
    finally:
        pcb.board_info.board_cutouts = []


def _region_welds(pcb, net_id, pad_info, comps, width=WIDTH):
    from routing_config import GridCoord
    from single_ended_routing import _pour_launch_region_cells
    cfg = _cfg(width=width)
    cells = quiet(_pour_launch_region_cells, pcb, net_id, pad_info, comps, list(LAYERS),
                  GridCoord(cfg.grid_step), {}, cfg)
    return [v for cmap in cells.values() for v in cmap.values()]


def t_region_cells(pcb, net_id):
    print('B. region cells (multipoint ladder)')
    from connectivity import get_multipoint_net_pads, get_terminal_component_info
    pad_info = quiet(get_multipoint_net_pads, pcb, net_id, _cfg())
    comps = get_terminal_component_info(pcb, net_id, pad_info)[0]
    with _edge_test_removed():
        before = _region_welds(pcb, net_id, pad_info, comps)
    check('precondition: without the edge test, welds land in the band',
          len(_in_band(before)) > 0, f'{len(_in_band(before))} of {len(before)}')
    after = _region_welds(pcb, net_id, pad_info, comps)
    check('pour-launch still offers cells', len(after) > 0, f'{len(after)}')
    check('no weld in the band', not _in_band(after),
          f'{len(_in_band(after))} of {len(after)}; closest '
          f'{min((_edge_dist(*p) for p in after), default=None)}')
    return pad_info, comps


def t_pair_anchors(pcb, net_id):
    print('C. pair anchors (2-pad endpoint path)')
    from routing_config import GridCoord
    from single_ended_routing import _pour_launch_pair_anchors
    cfg = _cfg()
    coord = GridCoord(cfg.grid_step)

    def row(x, y, layer):
        g = coord.to_grid(x, y)
        return (g[0], g[1], layer, x, y)

    def anchors():
        _, extra_t = quiet(_pour_launch_pair_anchors, pcb, net_id, [row(SX, 8, 0)],
                           [row((0.3 + STRIP) / 2, 3, 1)], list(LAYERS), coord, cfg)
        return [(r[3], r[4]) for r in extra_t]

    with _edge_test_removed():
        before = anchors()
    check('precondition: without the edge test, anchors land in the band',
          len(_in_band(before)) > 0, f'{len(_in_band(before))} of {len(before)}')
    after = anchors()
    check('anchors still offered', len(after) > 0, f'{len(after)}')
    check('no anchor in the band', not _in_band(after), f'{len(_in_band(after))} of {len(after)}')


def t_width_memo(pcb, net_id, pad_info, comps):
    print('D. the memo keys on the width')
    narrow = _region_welds(pcb, net_id, pad_info, comps, width=WIDTH)
    wide = _region_welds(pcb, net_id, pad_info, comps, width=0.6)
    check('the narrow track welds on the strip', len(narrow) > 0, f'{len(narrow)}')
    check("a 0.6 mm track does not get the narrow track's welds back",
          sorted(wide) != sorted(narrow), f'{len(wide)} vs {len(narrow)}')
    check('a 0.6 mm track on the same grouping gets no weld in ITS band',
          not _in_band(wide, width=0.6), f'{len(_in_band(wide, width=0.6))} of {len(wide)}')


if __name__ == '__main__':
    from kicad_parser import parse_kicad_pcb
    with tempfile.TemporaryDirectory(prefix='tweld_') as d:
        board = os.path.join(d, 'strip.kicad_pcb')
        with open(board, 'w', encoding='utf-8', newline='\n') as f:
            f.write(board_text())
        pcb = quiet(parse_kicad_pcb, evidence(board, 'staged board'))
        net_id = next(n for n, net in pcb.nets.items() if net.name == 'GND')
        t_point_test(pcb)
        pad_info, comps = t_region_cells(pcb, net_id)
        t_pair_anchors(pcb, net_id)
        t_width_memo(pcb, net_id, pad_info, comps)
    print()
    if FAILS:
        print(f'FAILED: {len(FAILS)} check(s)')
        for f in FAILS:
            print('  - ' + f)
        sys.exit(1)
    print('ALL PASS')

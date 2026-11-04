#!/usr/bin/env python3
"""POUR-LAUNCH cells are offered only where their terminal copper clears
foreign copper.

A pour-launch cell is a source/target OVERRIDE for the A* (the route may
start or end in a cell another net's clearance blocks), and the converter
ends the track at the cell's weld point -- a fill-raster sample -- with a
sub-cell bridge from the grid point it searched. No keep-out applies to that
copper and the terminal neck can only narrow it, so a weld at the rim of the
fill, where the raster holds the zone clearance to a foreign pad, shipped
half a track width past it: a plane-net join from such a cell to the next
grid cell laid GND copper 0.017-0.023 mm into a 0201 cap's P2.5V pad
clearance, with both ends on launch cells and nothing in between for the
map to refuse. Both producers (_pour_launch_region_cells,
_pour_launch_pair_anchors) now test each candidate with _launch_weld_test:
the grid point and the bridge to the weld, at the fab-floor width the neck
can narrow to, must clear every foreign pad, track and via at the pair
clearance.

The board (built here, 6 copper layers so the fab floor is the router's):
the cap of the case, C51 (0201, 0.46 x 0.40 roundrect pads, P2.5V and GND)
on B.Cu, rotated 90 deg at its own position; a B.Cu GND pour around its
P2.5V pad with a GND test pad in it; a second GND pour under the cap's GND
pad; a third GND test pad on bare board. The pours are two components, so the first pour's region offers launch
cells toward the cap's GND pad -- and its nearest fill wraps the P2.5V pad.

Checks:
  A. the case's own weld (158.91, 98.87) -- grid point (158.90, 98.90) --
     is refused, and a weld clear of the pad is accepted
  B. region cells: with the test bypassed, a weld's terminal copper grazes
     the P2.5V pad (the precondition); with it, none does and cells are
     still offered
  C. pair anchors: the same

    python3 tests/test_pour_launch_weld_clearance.py
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
WIDTH, CLEARANCE = 0.0762, 0.0889
TP = (158.97, 99.75)          # the GND test pad in the first pour


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


def _zone(x0, y0, x1, y1):
    return (f'\t(zone (net "GND") (layer "B.Cu") (uuid "{_uid()}") (hatch edge 0.5)\n'
            '\t\t(connect_pads yes (clearance 0.09)) (min_thickness 0.1)\n'
            '\t\t(fill yes (thermal_gap 0.2) (thermal_bridge_width 0.15) '
            '(island_removal_mode 0))\n'
            f'\t\t(polygon (pts (xy {x0} {y0}) (xy {x1} {y0}) (xy {x1} {y1}) '
            f'(xy {x0} {y1})))\n\t)\n')


def board_text():
    cap = (f'\t(footprint "test:C_0201" (layer "B.Cu") (uuid "{_uid()}") '
           '(at 158.600000 98.750001 90)\n'
           f'\t\t(property "Reference" "C51" (at 0 1.05 90) (layer "B.SilkS") '
           f'(uuid "{_uid()}") (effects (font (size 0.5 0.5) (thickness 0.1)) (justify mirror)))\n'
           '\t\t(pad "1" smd roundrect (at -0.32 0 90) (size 0.46 0.4) '
           '(layers "B.Cu" "B.Mask") (roundrect_rratio 0.25) (net "P2.5V") '
           f'(uuid "{_uid()}"))\n'
           '\t\t(pad "2" smd roundrect (at 0.32 0 90) (size 0.46 0.4) '
           '(layers "B.Cu" "B.Mask") (roundrect_rratio 0.25) (net "GND") '
           f'(uuid "{_uid()}"))\n\t)\n')
    def tp_fp(ref, x, y):
        return (f'\t(footprint "test:TP" (layer "B.Cu") (uuid "{_uid()}") (at {x} {y})\n'
                f'\t\t(property "Reference" "{ref}" (at 0 -1 0) (layer "B.SilkS") '
                f'(uuid "{_uid()}") (effects (font (size 0.5 0.5) (thickness 0.1)) '
                '(justify mirror)))\n'
                '\t\t(pad "1" smd rect (at 0 0) (size 0.3 0.3) (layers "B.Cu" "B.Mask") '
                f'(net "GND") (uuid "{_uid()}"))\n\t)\n')
    # TP1 in the first pour; TP2 on bare board, so GND is a multipoint net
    tp = tp_fp('TP1', *TP) + tp_fp('TP2', 161.0, 101.0)
    layers = ''.join(f'\t\t({i} "{n}" signal)\n' for i, n in
                     ((0, 'F.Cu'), (4, 'In1.Cu'), (6, 'In2.Cu'), (8, 'In3.Cu'),
                      (10, 'In4.Cu'), (2, 'B.Cu')))
    return ('(kicad_pcb\n\t(version 20260206)\n\t(generator "test_weld_clearance")\n'
            '\t(generator_version "10.0")\n'
            '\t(general (thickness 1.6) (legacy_teardrops no))\n\t(paper "A4")\n'
            '\t(layers\n' + layers +
            '\t\t(1 "F.Mask" user)\n\t\t(3 "B.Mask" user)\n\t\t(25 "Edge.Cuts" user)\n\t)\n'
            '\t(setup (pad_to_mask_clearance 0))\n'
            '\t(net 0 "")\n\t(net 1 "GND")\n\t(net 2 "P2.5V")\n'
            + cap + tp
            + f'\t(gr_rect (start 155 95) (end 163 103) (stroke (width 0.05) (type default)) '
            f'(fill no) (layer "Edge.Cuts") (uuid "{_uid()}"))\n'
            # the first pour is a strip along the P2.5V pad's right side (the pad
            # spans x 158.40-158.80, y 98.84-99.30), so all its fill is rim
            + _zone(158.80, 98.80, 159.05, 99.95)
            # the second sits under the GND pad, apart from the first
            + _zone(158.20, 97.90, 159.00, 98.62)
            + ')\n')


def _cfg():
    from routing_config import GridRouteConfig
    return GridRouteConfig(layers=list(LAYERS), track_width=WIDTH, clearance=CLEARANCE)


def _floor(pcb, cfg, net_id):
    from single_ended_routing import _fab_track_floor
    return cfg.track_floor(net_id, None, _fab_track_floor(pcb))


def _shortfall(pcb, cfg, net_id, fx, fy):
    """How far the weld's terminal copper (grid point -> weld, at the floor
    width) comes inside the P2.5V pad's clearance, measured independently of
    the router: check_drc's exact pad distance at 1 um steps."""
    from check_drc import point_to_pad_distance
    from routing_config import GridCoord
    coord = GridCoord(cfg.grid_step)
    gx, gy = coord.to_float(*coord.to_grid(fx, fy))
    pad = next(p for p in pcb.footprints['C51'].pads if p.net_name == 'P2.5V')
    n = max(2, int(((fx - gx) ** 2 + (fy - gy) ** 2) ** 0.5 / 0.001) + 1)
    d = min(point_to_pad_distance(gx + (fx - gx) * i / n, gy + (fy - gy) * i / n, pad)
            for i in range(n + 1))
    return CLEARANCE - (d - _floor(pcb, cfg, net_id) / 2.0)


def _grazing(pcb, cfg, net_id, welds):
    return [w for w in welds if _shortfall(pcb, cfg, net_id, *w) > 1e-6]


@contextlib.contextmanager
def _test_bypassed():
    """Both producers build their test through this name at call time:
    without it they offer what they did before the fix."""
    import single_ended_routing as ser
    real = getattr(ser, '_launch_weld_test', None)
    if real is None:            # code without the test: already bypassed
        yield
        return
    ser._launch_weld_test = lambda *a, **k: (lambda fx, fy, li: True)
    try:
        yield
    finally:
        ser._launch_weld_test = real


def t_case_weld(pcb, net_id):
    print("A. the case's own weld")
    from routing_config import GridCoord
    import single_ended_routing as ser
    cfg = _cfg()
    if not hasattr(ser, '_launch_weld_test'):
        check('the producers have a weld clearance test', False)
        return
    ok = ser._launch_weld_test(pcb, net_id, cfg, GridCoord(cfg.grid_step), list(LAYERS),
                               frozenset())
    check('precondition: (158.91, 98.87) via (158.90, 98.90) grazes the pad',
          _shortfall(pcb, cfg, net_id, 158.91, 98.87) > 0.01,
          f'{_shortfall(pcb, cfg, net_id, 158.91, 98.87):.4f} mm short')
    check('it is refused', not ok(158.91, 98.87, 1))
    check('a weld 0.3 mm right of the pad is accepted',
          _shortfall(pcb, cfg, net_id, 159.11, 99.07) < 0 and ok(159.11, 99.07, 1))


def _region_welds(pcb, net_id):
    from connectivity import get_multipoint_net_pads, get_terminal_component_info
    from routing_config import GridCoord
    from single_ended_routing import _pour_launch_region_cells
    cfg = _cfg()
    pad_info = quiet(get_multipoint_net_pads, pcb, net_id, cfg)
    comps = get_terminal_component_info(pcb, net_id, pad_info)[0]
    pcb._pour_launch_memo = None
    cells = quiet(_pour_launch_region_cells, pcb, net_id, pad_info, comps, list(LAYERS),
                  GridCoord(cfg.grid_step), {}, cfg)
    return [v for cmap in cells.values() for v in cmap.values()]


def t_region_cells(pcb, net_id):
    print('B. region cells (multipoint ladder)')
    cfg = _cfg()
    with _test_bypassed():
        before = _region_welds(pcb, net_id)
    check('precondition: without the test, a weld grazes the P2.5V pad',
          len(_grazing(pcb, cfg, net_id, before)) > 0,
          f'{len(_grazing(pcb, cfg, net_id, before))} of {len(before)}')
    after = _region_welds(pcb, net_id)
    check('pour-launch still offers cells', len(after) > 0, f'{len(after)}')
    worst = max((_shortfall(pcb, cfg, net_id, *w) for w in after), default=None)
    check('no weld grazes the P2.5V pad', not _grazing(pcb, cfg, net_id, after),
          f'{len(_grazing(pcb, cfg, net_id, after))} of {len(after)}; worst {worst}')


def t_pair_anchors(pcb, net_id):
    print('C. pair anchors (2-pad endpoint path)')
    from routing_config import GridCoord
    from single_ended_routing import _pour_launch_pair_anchors
    cfg = _cfg()
    coord = GridCoord(cfg.grid_step)

    def row(x, y):
        g = coord.to_grid(x, y)
        return (g[0], g[1], 1, x, y)

    gnd = next(p for p in pcb.footprints['C51'].pads if p.net_name == 'GND')

    def anchors():
        # the target's strip offers anchors toward the cap's GND pad
        _, extra_t = quiet(_pour_launch_pair_anchors, pcb, net_id,
                           [row(gnd.global_x, gnd.global_y)], [row(*TP)],
                           list(LAYERS), coord, cfg)
        return [(r[3], r[4]) for r in extra_t]

    with _test_bypassed():
        before = anchors()
    check('precondition: without the test, an anchor grazes the P2.5V pad',
          len(_grazing(pcb, cfg, net_id, before)) > 0,
          f'{len(_grazing(pcb, cfg, net_id, before))} of {len(before)}')
    after = anchors()
    check('anchors still offered', len(after) > 0, f'{len(after)}')
    check('no anchor grazes the P2.5V pad', not _grazing(pcb, cfg, net_id, after),
          f'{len(_grazing(pcb, cfg, net_id, after))} of {len(after)}')


if __name__ == '__main__':
    from kicad_parser import parse_kicad_pcb
    with tempfile.TemporaryDirectory(prefix='tweldclr_') as d:
        board = os.path.join(d, 'cap.kicad_pcb')
        with open(board, 'w', encoding='utf-8', newline='\n') as f:
            f.write(board_text())
        pcb = quiet(parse_kicad_pcb, evidence(board, 'staged board'))
        net_id = next(n for n, net in pcb.nets.items() if net.name == 'GND')
        t_case_weld(pcb, net_id)
        t_region_cells(pcb, net_id)
        t_pair_anchors(pcb, net_id)
    print()
    if FAILS:
        print(f'FAILED: {len(FAILS)} check(s)')
        for f in FAILS:
            print('  - ' + f)
        sys.exit(1)
    print('ALL PASS')

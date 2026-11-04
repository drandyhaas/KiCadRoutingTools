"""A via's DRILL is held off a net tie's bridge, even for the nets it ties.

#908's own-pad lift frees a net tie's bridge for the tied nets, tracks AND
vias. The bridge carries no net, though, and KiCad's net-tie exemption covers
the tied nets' copper touching it, not a via's HOLE: it grades the hole
against the bridge at min_hole_clearance -- and one that cuts into it at any
floor. (A one-net part's own copper, a SOT89 tab, is the accepted #995 row
and is left as it was.)
Measured on routed boards: a tied net's via whose hole sat 0.10 mm from its
own tie's bridge against a 0.15 mm floor, and another whose hole cut the
bridge of a tie it did not even touch the pad of, both graded hole_clearance
by kicad-cli. A via ON the tie's own pad, drill overlapping the bridge, is
accepted by KiCad and stays open.

Rows:
  1. the via map of a tied net blocks a site whose drill would cut the
     bridge (the copper-free control leaves it open), and a site in the
     bridge's interior;
  2. the site on the tied net's own pad stays open, as on the control;
  3. a foreign net is still held off the bridge (unchanged);
  4. the copper-to-hole census a step's writeback reads measures a tied
     via's drill against the bridge, and skips one on the own pad.

    python3 tests/test_graphic_copper_via_hole.py
"""
import contextlib
import io
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'py_router'))

FAILS = []

_TIE_POLY = ('(fp_poly (pts (xy -0.4 -0.11775) (xy 0.4 -0.11775) (xy 0.4 0.11775) '
             '(xy -0.4 0.11775)) (stroke (width 0) (type solid)) (fill yes) '
             '(layer "{layer}") (uuid "aaaaaaaa-0000-4000-8000-000000000001"))')

BOARD = """(kicad_pcb (version 20241229) (generator "pcbnew")
  (general (thickness 1.6))
  (layers
    (0 "F.Cu" signal)
    (2 "B.Cu" signal)
    (25 "Edge.Cuts" user)
  )
  (net 0 "")
  (net 1 "TIED_A")
  (net 2 "TIED_B")
  (net 3 "FOREIGN")
  (footprint "test:net_tie" (layer "F.Cu")
    (at 10 10)
    (property "Reference" "NT1" (at 0 -2 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))
    (net_tie_pad_groups "1, 2")
    %s
    (pad "1" smd circle (at -0.4 0) (size 0.2355 0.2355) (layers "F.Cu") (net 1 "TIED_A"))
    (pad "2" smd circle (at 0.4 0) (size 0.2355 0.2355) (layers "F.Cu") (net 2 "TIED_B"))
  )
  (footprint "test:res" (layer "F.Cu")
    (at 16 10)
    (property "Reference" "R2" (at 0 -2 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))
    (pad "1" smd rect (at 0 0) (size 0.5 0.5) (layers "F.Cu") (net 1 "TIED_A"))
    (pad "2" smd rect (at 0 2) (size 0.5 0.5) (layers "F.Cu") (net 3 "FOREIGN"))
  )
  (gr_rect (start 0 0) (end 30 20) (layer "Edge.Cuts") (stroke (width 0.1) (type solid)))
)
"""

#: Via sites, mm. The bridge spans x 9.6..10.4, y 9.88225..10.11775; the
#: tied net's pad 1 is the 0.2355 circle at (9.6, 10).
CUTS = (10.0, 10.2)       # drill (r 0.1) reaches y 10.1 < 10.11775: cuts it
INTERIOR = (10.0, 10.0)   # in the bridge, away from both pads
ON_PAD = (9.6, 10.0)      # the tied net's own pad: KiCad accepts it


def check(name, cond, detail=''):
    print(('  PASS  ' if cond else '  FAIL  ') + name + (f'  [{detail}]' if detail else ''))
    if not cond:
        FAILS.append(name)


def _board(tie_layer):
    """F.Cu is the real board; F.SilkS the control with the bridge off copper."""
    from kicad_parser import parse_kicad_pcb
    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False) as f:
        f.write(BOARD % _TIE_POLY.format(layer=tie_layer))
        path = f.name
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            return parse_kicad_pcb(path)
    finally:
        os.unlink(path)


def _net(pcb, name):
    return next(n.net_id for n in pcb.nets.values() if n.name == name)


def _via_blocked(pcb, net_name, other_name, sites):
    """is_via_blocked at each site for `net_name`, after a real prepare."""
    from routing_config import GridRouteConfig, GridCoord
    from obstacle_map import build_base_obstacle_map, build_layer_map
    from routing_context import prepare_obstacles_inplace
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], grid_step=0.05,
                          clearance=0.1, via_size=0.3, via_drill=0.2)
    net, other = _net(pcb, net_name), _net(pcb, other_name)
    with contextlib.redirect_stdout(io.StringIO()):
        obs = build_base_obstacle_map(pcb, cfg, [net, other])
        prepare_obstacles_inplace(obs, pcb, cfg, net, [net, other], [], {},
                                  build_layer_map(cfg.layers), {})
    coord = GridCoord(cfg.grid_step)
    return [bool(obs.is_via_blocked(*coord.to_grid(x, y))) for x, y in sites]


def rows_tied_net_map():
    print('1-2. the tied net\'s via map')
    real, ctl = _board('F.Cu'), _board('F.SilkS')
    sites = [CUTS, INTERIOR, ON_PAD]
    r = _via_blocked(real, 'TIED_A', 'FOREIGN', sites)
    c = _via_blocked(ctl, 'TIED_A', 'FOREIGN', sites)
    check('control: every site is open without the bridge', c == [False] * 3, str(c))
    check('a site whose drill cuts the bridge is blocked', r[0], str(r))
    check('a site in the bridge interior is blocked', r[1], str(r))
    check('the site on the tied net\'s own pad stays open', not r[2], str(r))


def row_foreign():
    print('3. a foreign net')
    r = _via_blocked(_board('F.Cu'), 'FOREIGN', 'TIED_A', [CUTS, INTERIOR])
    check('a foreign net is held off the bridge', all(r), str(r))


def rows_census():
    print('4. the copper-to-hole census')
    from kicad_parser import Via
    from fix_kicad_drc_settings import step_copper_to_hole
    pcb = _board('F.Cu')
    a = _net(pcb, 'TIED_A')

    def census(x, y):
        v = Via(x=x, y=y, size=0.3, drill=0.2, layers=['F.Cu', 'B.Cu'], net_id=a)
        return step_copper_to_hole(pcb, [], [v], 0.25, clearance=0.1)
    near = census(10.0, 10.35)          # hole edge 0.13226 from the bridge
    check('a tied via\'s drill is measured against the bridge',
          near is not None and abs(near - 0.13225) < 1e-3, str(near))
    check('a drill cutting the bridge reads 0', census(*CUTS) == 0.0, str(census(*CUTS)))
    check('a via on the tied net\'s own pad is not counted',
          census(*ON_PAD) is None, str(census(*ON_PAD)))
    ctl = _board('F.SilkS')
    v = Via(x=10.0, y=10.35, size=0.3, drill=0.2, layers=['F.Cu', 'B.Cu'],
            net_id=_net(ctl, 'TIED_A'))
    check('control: nothing to measure without the bridge',
          step_copper_to_hole(ctl, [], [v], 0.25, clearance=0.1) is None)


def main():
    rows_tied_net_map()
    row_foreign()
    rows_census()
    print('-' * 60)
    if FAILS:
        print(f'{len(FAILS)} FAILED: ' + '; '.join(FAILS))
        sys.exit(1)
    print('ALL PASS')


if __name__ == '__main__':
    main()

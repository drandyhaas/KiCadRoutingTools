#!/usr/bin/env python3
"""#1687: a pair on a layer wider than the call's track width keeps the board's
copper-to-hole clearance from an NPTH.

A hole keep-out stamps the call's `track_width`. On a pair's centreline map a
member on a wider layer sits a half-pitch further out and is itself wider, so
its far edge is (layer width - track width) past what the stamp reserved, and
further at the outer corner of a bend. A wide segment is necked when it grazes
pads or tracks, but a neck does not look at holes: the copper shipped short of
the hole, and the step's writeback then lowered `min_hole_clearance` to what
that copper needed.

Rows:
  - stamps: on a baked map a wide layer's hole keep-out reaches the requirement,
    a layer no wider than the call's stays where it was, and the centreline
    map's reaches each layer's member far edge;
  - no baked width (the single-ended engine, whose margin carries the rest): the
    stamp stays at the call's width on every layer;
  - routed: --impedance 80 widening a 0.25 call to 0.41 on a pair whose straight
    run passes an NPTH at a distance only the nominal width clears. The members
    clear the hole by the 0.3 the board declares, the pair still couples and
    connects, and the writeback no longer lowers the floor.

    python3 tests/test_1687_wide_layer_hole_keepout.py
"""
import contextlib
import io
import json
import math
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import BoardInfo  # noqa: E402
from obstacle_map import build_base_obstacle_map  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402
from synth import make_net, make_pad, make_pcb  # noqa: E402

FAILS = []
GRID = 0.05
HOLE_X, HOLE_Y, DRILL = 10.0, 10.0, 1.0
TRACK, GAP, CLR, FLOOR = 0.1, 0.1, 0.1, 0.2  # FLOOR: the NPTH floor a stamp holds


def check(name, cond, detail=''):
    print(('  PASS  ' if cond else '  FAIL  ') + name + (f'  [{detail}]' if detail else ''))
    if not cond:
        FAILS.append(name)


def _npth_board():
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'}, copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=(0.0, 0.0, 20.0, 20.0))
    hole = make_pad(0, HOLE_X, HOLE_Y, ref='H1', num='', size_x=DRILL, size_y=DRILL,
                    shape='circle', layers=('*.Cu', '*.Mask'), drill=DRILL,
                    pad_type='np_thru_hole')
    return make_pcb(nets={0: make_net(0, '')}, segments=[], pads_by_net={0: [hole]},
                    board_info=bi)


def _radius(grid_map, layer_idx):
    """How far along +x from the hole centre this layer's keep-out reaches (mm)."""
    gx0, gy0 = int(round(HOLE_X / GRID)), int(round(HOLE_Y / GRID))
    for n in range(1, 200):
        if not grid_map.is_blocked(gx0 + n, gy0, layer_idx):
            return (n - 1) * GRID
    return None


def _map(reserve, extra):
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], clearance=CLR, track_width=TRACK,
                          diff_pair_gap=GAP, grid_step=GRID, via_size=0.3, via_drill=0.15,
                          layer_widths={'F.Cu': 0.3}, reserve_layer_widths=reserve)
    with contextlib.redirect_stdout(io.StringIO()):
        return build_base_obstacle_map(_npth_board(), cfg, [], extra)


def _reach(required):
    """The last cell a stamp of this radius blocks along the axis."""
    return math.ceil(required / GRID - 1e-6) * GRID - GRID


def test_stamps():
    half = DRILL / 2
    half_pitch = (TRACK + GAP) / 2
    wide = _map(True, 0.0)
    check('single-ended map on the baked config: the wide layer reserves its own width',
          abs(_radius(wide, 0) - _reach(half + FLOOR + 0.3 / 2)) < 1e-6, str(_radius(wide, 0)))
    check('... and the layer at the call\'s width stays there',
          abs(_radius(wide, 1) - _reach(half + FLOOR + TRACK / 2)) < 1e-6, str(_radius(wide, 1)))
    centre = _map(True, half_pitch)
    # the member's far edge: its own half width + the layer's half-pitch, which
    # at a 45-degree bend is half-pitch / cos(22.5 deg) at the outer corner
    mitre = 1.0 / math.cos(math.radians(22.5))
    far_edge = 0.3 / 2 + (0.3 + GAP) / 2 * mitre
    check('centreline map: the wide layer reaches the member\'s far edge',
          abs(_radius(centre, 0) - _reach(half + FLOOR + far_edge)) < 1e-6,
          str(_radius(centre, 0)))
    check('... and a layer at the call\'s width reaches that layer\'s',
          abs(_radius(centre, 1) - _reach(half + FLOOR + TRACK / 2 + half_pitch * mitre)) < 1e-6,
          str(_radius(centre, 1)))
    margin = _map(False, 0.0)
    for idx, name in enumerate(('F.Cu', 'B.Cu')):
        check(f'no baked width: {name} stays at the call\'s width',
              abs(_radius(margin, idx) - _reach(half + FLOOR + TRACK / 2)) < 1e-6,
              str(_radius(margin, idx)))


STACKUP = ('\t\t(stackup\n'
           '\t\t\t(layer "F.Cu" (type "copper") (thickness 0.035))\n'
           '\t\t\t(layer "dielectric 1" (type "core") (thickness 0.2) (material "FR4") '
           '(epsilon_r 4.5) (loss_tangent 0.02))\n'
           '\t\t\t(layer "B.Cu" (type "copper") (thickness 0.035))\n'
           '\t\t)\n')
# Above the pair's straight run (centreline y = 15, 16.5 >= x >= 12.6), on a
# board declaring 0.3 mm copper-to-hole: 1.2 mm from that centreline clears the
# nominal stamp's 1.175 and not the 1.333 a member at 0.41 needs.
MOUNTING_HOLE = ('\t(footprint "test:MH" (layer "F.Cu") (at 14.5 13.8)\n'
                 '\t\t(uuid "00000000-0000-0000-0000-0000000000a1")\n'
                 '\t\t(property "Reference" "H1" (at 0 0) (layer "F.SilkS")\n'
                 '\t\t\t(uuid "00000000-0000-0000-0000-0000000000a2") (effects (font (size 1 1))))\n'
                 '\t\t(pad "" np_thru_hole circle (at 0 0) (size 1 1) (drill 1) (layers "*.Cu" "*.Mask")\n'
                 '\t\t\t(uuid "00000000-0000-0000-0000-0000000000a3"))\n'
                 '\t)\n')
PRO = {"board": {"design_settings": {"rules": {"min_hole_clearance": 0.3}}},
       "net_settings": {"classes": [{
           "name": "Default", "clearance": 0.25, "track_width": 0.25, "diff_pair_gap": 0.25,
           "diff_pair_width": 0.25, "via_diameter": 0.6, "via_drill": 0.3}]}}


def test_routed_pair_clears_the_hole():
    import route_diff
    from check_connected import run_connectivity_check
    from kicad_parser import parse_kicad_pcb
    src = open(os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb'), encoding='utf-8').read()
    i = src.index('\n', src.index('(setup')) + 1
    txt = src[:i] + STACKUP + src[i:]
    k = txt.rstrip().rfind(')')
    txt = txt[:k] + MOUNTING_HOLE + txt[k:]
    with tempfile.TemporaryDirectory() as td:
        dst, out = os.path.join(td, 'cc.kicad_pcb'), os.path.join(td, 'o.kicad_pcb')
        with open(dst, 'w', encoding='utf-8', newline='') as fh:
            fh.write(txt)
        with open(os.path.join(td, 'cc.kicad_pro'), 'w', encoding='utf-8') as fh:
            json.dump(PRO, fh)
        log = io.StringIO()
        with contextlib.redirect_stdout(log):
            route_diff.batch_route_diff_pairs(dst, out, ['DPA_*'], impedance=80,
                                              track_width=0.25, diff_pair_gap=0.25,
                                              clearance=0.25, via_size=0.6, via_drill=0.3,
                                              layers=['F.Cu', 'B.Cu'])
            from fix_kicad_drc_settings import step_hole_clearance
            floor = step_hole_clearance(dst, out, 0.25, verbose=False, ledger=False)
        if not os.path.isfile(out):
            check('routed: an output board', False, log.getvalue()[-400:])
            return
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(out)
            conn = run_connectivity_check(out, net_patterns=['DPA_*'], quiet=True)
    names = {n.net_id: n.name for n in pcb.nets.values()}
    segs = [s for s in pcb.segments if names.get(s.net_id, '').startswith('DPA')]
    check('routed: the pair runs wider than the call',
          bool(segs) and max(s.width for s in segs) >= 0.4,
          str(sorted({round(s.width, 3) for s in segs})))
    check('routed: the pair is coupled, not deferred',
          '"routed_diff_pairs": ["DPA"]' in log.getvalue())
    check('routed: the pair is connected', list(conn) == [], str(conn)[:200])

    def wall_gap(s):
        ax, ay, bx, by = s.start_x, s.start_y, s.end_x, s.end_y
        dx, dy = bx - ax, by - ay
        t = 0.0 if dx == dy == 0 else max(0.0, min(1.0, ((14.5 - ax) * dx + (13.8 - ay) * dy)
                                                   / (dx * dx + dy * dy)))
        return math.hypot(ax + t * dx - 14.5, ay + t * dy - 13.8) - 0.5 - s.width / 2
    nearest = min((wall_gap(s) for s in segs), default=None)
    check('routed: every member clears the hole by the board\'s 0.3',
          nearest is not None and nearest >= 0.3 - 1e-3, f'{nearest}')
    check('routed: the writeback has nothing to lower', floor == 0.3, str(floor))


def main():
    test_stamps()
    test_routed_pair_clears_the_hole()
    if FAILS:
        print(f'\nFAIL: {len(FAILS)} check(s): ' + '; '.join(FAILS))
        return 1
    print('\nPASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())

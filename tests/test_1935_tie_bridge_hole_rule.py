"""#1935: a tied net's via keeps a `.kicad_dru` hole_clearance rule off its
net tie's bridge.

A net tie's bridge is lifted for the nets it ties, except for a via's drill:
the bridge has no net, so a hole beside it is held off at the board's
copper-to-hole floor (#908). A hole_clearance rule that asks more was not read
there, and KiCad applies it -- a rule between different nets binds a tied
net's via against net-less copper. Measured on kicad-cli 10.0.8, #908's board
with `hole_clearance (min 0.3mm)`: a `TIED_A` via whose hole is 0.20 mm from
the bridge is reported (`Polygon [<no net>] of NT1`, actual 0.2000 mm), one
0.32 mm away is clean, and a via on the tied pad itself is accepted.

The board is #908's net tie (tests/test_908_net_tie_own_copper.py): bridge on
F.Cu from y 9.9 to 10.1, vias 0.45 / 0.2, `min_hole_clearance` 0.1 mm. The
column x = 9.8 is clear of the other net's pad, so the first via site the map
leaves free there is the bridge's keep-out alone.

  1. no rule: the first free site leaves the hole 0.10 mm off the bridge (the
     board floor), as before;
  2. the rule: 0.30 mm;
  3. the rule: the tied pad itself stays a via site;
  4. the rule: a foreign net's via keep-out off the bridge is not narrower
     than the tied net's.
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'py_router'))

import run_utils  # noqa: E402
import test_908_net_tie_own_copper as t908  # noqa: E402

FAILS = []
BRIDGE_TOP = 10.1          # the bridge's edge on the +y side
DRILL = 0.2


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def stage(td, tag, rule):
    base = os.path.join(td, tag)
    with open(base + '.kicad_pcb', 'w', encoding='utf-8') as fh:
        fh.write(t908.BOARD % t908._TIE_POLY.format(layer='F.Cu'))
    with open(base + '.kicad_pro', 'w', encoding='utf-8') as fh:
        json.dump({'board': {'design_settings': {'rules': {
            'min_clearance': 0.0, 'min_hole_clearance': 0.1, 'min_hole_to_hole': 0.2}}},
            'net_settings': {'meta': {'version': 4}, 'classes': [
                {'name': 'Default', 'clearance': 0.1, 'track_width': 0.1,
                 'via_diameter': 0.45, 'via_drill': DRILL, 'priority': 2147483647}]},
            'meta': {'version': 3}}, fh)
    if rule:
        with open(base + '.kicad_dru', 'w', encoding='utf-8') as fh:
            fh.write('(version 1)\n(rule "hole to copper"\n'
                     '  (constraint hole_clearance (min 0.3mm))\n'
                     '  (condition "A.NetName != B.NetName"))\n')
    return run_utils.evidence(base + '.kicad_pcb')


def via_map(board, net_name):
    """(the step holds a hole rule, via-blocked test in mm) for `net_name`
    after the rules are installed and a real prepare."""
    from kicad_parser import parse_kicad_pcb
    from kicad_dru import install_layer_clearances
    from routing_config import GridRouteConfig, GridCoord
    from obstacle_map import build_base_obstacle_map, build_layer_map
    from routing_context import prepare_obstacles_inplace
    pcb = parse_kicad_pcb(board)
    nid = next(n.net_id for n in pcb.nets.values() if n.name == net_name)
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], clearance=0.1, track_width=0.1,
                          via_size=0.45, via_drill=DRILL, grid_step=0.05)
    with contextlib.redirect_stdout(io.StringIO()):
        install_layer_clearances(cfg, None, board, pcb, net_keepouts=True)
        obs = build_base_obstacle_map(pcb, cfg, [nid])
        prepare_obstacles_inplace(obs, pcb, cfg, nid, [nid], [], {},
                                  build_layer_map(cfg.layers), {})
    coord = GridCoord(cfg.grid_step)

    def blocked(x, y):
        return bool(obs.is_via_blocked(*coord.to_grid(x, y)))
    return getattr(cfg, 'hole_rules', None) is not None, blocked


def first_free(blocked, x=9.8):
    """The hole-edge distance (mm) from the bridge of the first free via site
    going up the column `x` from the bridge's centre line."""
    for k in range(0, 40):
        y = round(10.0 + 0.05 * k, 2)
        if not blocked(x, y):
            return round(y - BRIDGE_TOP - DRILL / 2.0, 3)
    return None


def main():
    td = tempfile.mkdtemp(prefix='krt1935_')
    try:
        print('1. no rule')
        ruled, blocked = via_map(stage(td, 'plain', False), 'TIED_A')
        ok(not ruled, 'the step holds no hole rule')
        got = first_free(blocked)
        ok(got == 0.1, f'the first free site is 0.10 mm off the bridge: {got}')

        print('2-3. the rule, a tied net')
        ruled, blocked = via_map(stage(td, 'ruled', True), 'TIED_A')
        ok(ruled, 'the step holds the hole rule')
        tied = first_free(blocked)
        ok(tied == 0.3, f'the first free site is 0.30 mm off the bridge: {tied}')
        ok(not blocked(9.6, 10.0), 'the tied pad itself stays a via site')

        print('4. the rule, a foreign net')
        _r, blocked = via_map(stage(td, 'foreign', True), 'FOREIGN')
        far = first_free(blocked)
        ok(far is not None and far >= tied,
           f"a foreign net's first free site is no nearer: {far} vs {tied}")
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if FAILS:
        print(f'FAIL: {len(FAILS)}: ' + '; '.join(FAILS))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())

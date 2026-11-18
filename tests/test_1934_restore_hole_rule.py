"""#1934: a restore keeps no copper inside a `.kicad_dru` hole_clearance rule.

When a ripped net's reroute fails, its saved copper is put back unless that
would collide with copper routed since the rip (`rip_up_reroute.
_saved_route_collides`, #134). The test priced copper to copper and drill to
drill. A board's hole_clearance rule measures from a HOLE to another net's
copper and, where the board declares one, is wider than both: a via restored
beside a via routed meanwhile passed the test and shipped inside the rule,
which every search and generator of the step holds (#1853).

The board: nets X and Y, a via of Y on the board (0.45 / 0.2), and a rule
`hole_clearance (min 0.3mm)` between different nets. Two 0.45 / 0.2 vias
clear each other's COPPER at 0.55 mm centre to centre (class 0.1), their
drills at 0.4 mm, and the rule at 0.625 mm (0.1 + 0.3 + 0.225).

  1. no rule: X's saved via 0.58 mm from Y's is restored (non-vacuity: the
     pair is copper-clear and drill-clear);
  2. the rule: the same via is refused, and the culprit named is Y's via;
  3. the rule: at 0.64 mm it is restored;
  4. the rule: a saved track 0.4 mm from Y's via centre, clear of its copper
     (0.375 mm), is refused (hole edge to track edge 0.25 mm), and restored at
     0.5 mm;
  5. two items the step was handed, both unmoved, are never refused (#1609),
     rule or not;
  6. a handed via beside a via laid since is refused.
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

FAILS = []
Y_VIA = (5.0, 5.0)


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def _via(net, x, y):
    return (f'  (via (at {x} {y}) (size 0.45) (drill 0.2) (layers "F.Cu" "B.Cu") '
            f'(net {net}) (uuid "eeeeeeee-0000-4000-8000-{int(x * 1000):06d}{int(y * 1000):06d}"))\n')


def stage(td, tag, rule, extra=''):
    """Y's via at Y_VIA on a two-layer board, with or without the hole rule."""
    base = os.path.join(td, tag)
    with open(base + '.kicad_pcb', 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20241229) (generator "pcbnew")\n'
                 '  (general (thickness 1.6))\n'
                 '  (layers\n    (0 "F.Cu" signal)\n    (2 "B.Cu" signal)\n'
                 '    (25 "Edge.Cuts" user)\n  )\n'
                 '  (net 0 "")\n  (net 1 "X")\n  (net 2 "Y")\n'
                 + _via(2, *Y_VIA) + extra +
                 '  (gr_rect (start 0 0) (end 12 12) (layer "Edge.Cuts") '
                 '(stroke (width 0.1) (type solid)))\n)\n')
    with open(base + '.kicad_pro', 'w', encoding='utf-8') as fh:
        json.dump({'board': {'design_settings': {'rules': {
            'min_clearance': 0.0, 'min_hole_clearance': 0.1, 'min_hole_to_hole': 0.2}}},
            'net_settings': {'meta': {'version': 4}, 'classes': [
                {'name': 'Default', 'clearance': 0.1, 'track_width': 0.1,
                 'via_diameter': 0.45, 'via_drill': 0.2, 'priority': 2147483647}]},
            'meta': {'version': 3}}, fh)
    if rule:
        with open(base + '.kicad_dru', 'w', encoding='utf-8') as fh:
            fh.write('(version 1)\n(rule "hole to copper"\n'
                     '  (constraint hole_clearance (min 0.3mm))\n'
                     '  (condition "A.NetName != B.NetName"))\n')
    return run_utils.evidence(base + '.kicad_pcb')


def load(board):
    """(pcb_data, the config a route step would hold for it)."""
    from kicad_parser import parse_kicad_pcb
    from kicad_dru import install_layer_clearances
    from routing_config import GridRouteConfig
    pcb = parse_kicad_pcb(board)
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], clearance=0.1, track_width=0.1,
                          via_size=0.45, via_drill=0.2, hole_to_hole_clearance=0.2)
    with contextlib.redirect_stdout(io.StringIO()):
        install_layer_clearances(cfg, None, board, pcb, net_keepouts=True)
    return pcb, cfg


def saved_via(dx):
    from kicad_parser import Via
    return Via(x=Y_VIA[0] + dx, y=Y_VIA[1], size=0.45, drill=0.2,
               layers=['F.Cu', 'B.Cu'], net_id=1)


def saved_track(dy):
    from kicad_parser import Segment
    return Segment(start_x=Y_VIA[0] - 1.0, start_y=Y_VIA[1] + dy,
                   end_x=Y_VIA[0] + 1.0, end_y=Y_VIA[1] + dy,
                   width=0.1, layer='F.Cu', net_id=1)


def collides(pcb, cfg, segs=(), vias=()):
    from rip_up_reroute import _saved_route_colliders
    return _saved_route_colliders({'new_segments': list(segs), 'new_vias': list(vias)},
                                  pcb, [1], cfg.clearance, config=cfg)


def restores(td):
    print('1. no rule: the pair is copper-clear and drill-clear')
    pcb, cfg = load(stage(td, 'plain', rule=False))
    ok(getattr(cfg, 'hole_rules', None) is None, 'the step holds no hole rule')
    ok(not collides(pcb, cfg, vias=[saved_via(0.58)]), 'a via 0.58 mm away is restored')
    ok(collides(pcb, cfg, vias=[saved_via(0.50)]),
       'and one 0.50 mm away is refused on copper clearance, as before')
    ok(not collides(pcb, cfg, segs=[saved_track(0.40)]),
       "a track 0.4 mm from Y's via centre is restored")

    print('2-4. the rule')
    pcb, cfg = load(stage(td, 'ruled', rule=True))
    ok(getattr(cfg, 'hole_rules', None) is not None, 'the step holds the hole rule')
    hits = collides(pcb, cfg, vias=[saved_via(0.58)])
    ok(bool(hits), 'a via 0.58 mm away is refused')
    ok(bool(hits) and all(getattr(o, 'net_id', None) == 2 for _k, o in hits),
       f"and the culprit named is Y's copper: {[(k, getattr(o, 'net_id', None)) for k, o in hits]}")
    ok(not collides(pcb, cfg, vias=[saved_via(0.64)]), 'a via 0.64 mm away is restored')
    ok(bool(collides(pcb, cfg, segs=[saved_track(0.40)])),
       "a track 0.25 mm from Y's hole is refused")
    ok(not collides(pcb, cfg, segs=[saved_track(0.50)]),
       "a track 0.35 mm from Y's hole is restored")


def handed(td):
    print('5-6. copper the step was handed')
    from rip_up_reroute import mark_input_copper
    board = stage(td, 'handed', rule=True, extra=_via(1, Y_VIA[0] + 0.58, Y_VIA[1]))
    pcb, cfg = load(board)
    mark_input_copper(pcb)
    mine = next(v for v in pcb.vias if v.net_id == 1)
    pcb.vias = [v for v in pcb.vias if v is not mine]           # ripped
    ok(not collides(pcb, cfg, vias=[mine]),
       'a handed via goes back beside the handed via it stood beside (#1609)')
    from kicad_parser import Via
    pcb.vias.append(Via(x=mine.x + 0.58, y=mine.y, size=0.45, drill=0.2,
                        layers=['F.Cu', 'B.Cu'], net_id=2))     # laid since
    ok(bool(collides(pcb, cfg, vias=[mine])),
       'and is refused beside a via laid since the rip')


def main():
    td = tempfile.mkdtemp(prefix='krt1934_')
    try:
        restores(td)
        handed(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if FAILS:
        print(f'FAIL: {len(FAILS)}: ' + '; '.join(FAILS))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())

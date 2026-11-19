#!/usr/bin/env python3
"""#1055: the diff-pair polarity crossover (py_router/diff_pair_crossover.py).

    python3 tests/test_1055_diff_pair_crossover.py

The polarity ladder's last rung crosses a pair's legs at a layer change. Its
routing end to end is pinned by test_1830's denied stage; this pins the two
pieces it is built from:

  1. the geometry: P's side swaps across the crossover, both barrels stand on
     one side of the pair, no leg turns more than 45 degrees, and the shape
     clears its own intra-pair checks;
  2. _board_clear, which grades the crossover's copper against the board's
     OTHER nets with check_drc's primitives. Measured in the first sets 1-28
     arm, two defects shipped through the checks it replaced:
       - a barrel on the PARTNER's pad (ethercat_ax58100 P0_TXOP on C36.2:
         the barrel check excluded both pair nets);
       - a leg 0.036 mm into a routed pair's track (berkeley_obsidian AD1_P /
         MGT_RX2_P: the single-track map leaves out the nets the step routes,
         and the connector graze gate passed it).
     Both are rebuilt here on the in-repo esp_prog board, refused, and shown
     to pass once moved clear (so a refusal is the geometry, not a crash).
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'py_router'))

FAILS = []


def check(name, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {name}" + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def side(pt, origin, u):
    """+1 left of u from origin, -1 right."""
    return 1 if (u[0] * (pt[1] - origin[1]) - u[1] * (pt[0] - origin[0])) > 0 else -1


def turn_deg(a, b, c):
    v1 = (b[0] - a[0], b[1] - a[1])
    v2 = (c[0] - b[0], c[1] - b[1])
    n1, n2 = math.hypot(*v1), math.hypot(*v2)
    if n1 < 1e-9 or n2 < 1e-9:
        return 0.0
    cosv = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))
    return math.degrees(math.acos(cosv))


def test_geometry():
    print('1. crossover geometry')
    from diff_pair_crossover import crossover_geometry, crossover_copper, _intra_clear
    half, via_half, via, drill, track, clr, h2h, grid = 0.1, 0.33, 0.45, 0.2, 0.1, 0.09, 0.2, 0.1
    for u in ((1.0, 0.0), (0.0, 1.0), (math.sqrt(0.5), math.sqrt(0.5))):
        for s_in in (1, -1):
            for first in ('P', 'N'):
                V = (100.0, 50.0)
                g = crossover_geometry(V, u, s_in, half, via_half, via, drill, track, clr, h2h, grid,
                                       first=first)
                tag = f'u={tuple(round(x, 2) for x in u)} s_in={s_in} {first} first'
                check(f'{tag}: P enters on its side and leaves on the other',
                      side(g['entry']['P'], V, u) == s_in and side(g['exit']['P'], V, u) == -s_in)
                bs = {side((x, y), V, u) for x, y, _k in g['vias']}
                check(f'{tag}: both barrels on one side', len(bs) == 1)
                worst = max((turn_deg(*pts[i:i + 3]) for runs in g['legs'].values()
                             for pts, _L in runs for i in range(len(pts) - 2)), default=0.0)
                check(f'{tag}: no leg turns more than 45 degrees', worst <= 45.0 + 1e-6, f'{worst:.1f}')
                segs, vias = crossover_copper(g, 'F.Cu', 'B.Cu', 1, 2, track, via, drill,
                                              ['F.Cu', 'B.Cu'])
                check(f'{tag}: intra-pair clear', _intra_clear(segs, vias, 1, 2, track, via, clr,
                                                                via + clr))


def test_board_clear():
    print('2. _board_clear on the in-repo esp_prog board')
    import contextlib
    import io
    from kicad_parser import parse_kicad_pcb, Segment, Via
    from routing_config import GridRouteConfig
    from diff_pair_crossover import _board_clear
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(os.path.join(REPO, 'kicad_files', 'esp_prog.kicad_pcb'))
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], grid_step=0.05, track_width=0.2,
                          clearance=0.15, via_size=0.45, via_drill=0.3)
    # a barrel on another net's pad -- the partner's, in the shipped defect
    pad = next(p for fp in pcb.footprints.values() for p in fp.pads
               if p.net_id and 'F.Cu' in p.layers and p.drill == 0)
    other = next(n for n in pcb.nets if n not in (0, pad.net_id))
    on_pad = Via(x=pad.global_x, y=pad.global_y, size=0.45, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=other)
    why = _board_clear([], [on_pad], pcb, cfg)
    check('a barrel on another net\'s pad is refused', bool(why) and 'pad' in why, why)
    # a leg 0.036 mm into another net's routed track
    t = Segment(start_x=150.0, start_y=150.0, end_x=152.0, end_y=150.0, width=0.2, layer='F.Cu',
                net_id=pad.net_id)
    pcb.segments.append(t)
    gap = 0.2 + cfg.pair_clearance(other, pad.net_id, 'F.Cu') - 0.036
    leg = Segment(start_x=150.5, start_y=150.0 + gap, end_x=151.5, end_y=150.0 + gap, width=0.2,
                  layer='F.Cu', net_id=other)
    why = _board_clear([leg], [], pcb, cfg)
    check('a leg 0.036 mm into another net\'s track is refused', bool(why) and 'track' in why, why)
    # the same pieces moved clear pass
    clear_leg = Segment(start_x=150.5, start_y=151.5, end_x=151.5, end_y=151.5, width=0.2,
                        layer='F.Cu', net_id=other)
    clear_via = Via(x=150.0, y=153.0, size=0.45, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=other)
    why = _board_clear([clear_leg], [clear_via], pcb, cfg)
    check('the same pieces moved clear pass', why == '', why)
    check('an empty set passes', _board_clear([], [], pcb, cfg) == '')


def main():
    test_geometry()
    test_board_clear()
    if FAILS:
        print(f'\nFAILED {len(FAILS)}: ' + '; '.join(FAILS))
        return 1
    print('\nALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())

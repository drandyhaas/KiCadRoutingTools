#!/usr/bin/env python3
"""#1583 A: an intra-pair undershoot adds bumps before it reports
'meanders-ran-out'.

`_lengthen_net_with_meanders` (intra-pair P/N matching and AC-coupled XNet
matching) sized its bump count as if every bump reached --meander-amplitude:
ceil(delta / (2*A - 2.34*chamfer)). A foreign rail beside the leg clamps each
bump a 0.7x step down (`get_safe_amplitude_at_point`), so that count falls
short, and the only retry kept the count and rescaled the amplitude to the
same value, then stopped. The reporter's pair between two rails 1.2 mm off
stopped 2.22 mm short at amplitude 1.0 while amplitude 0.7 matched it with 6
bumps. When the first shot and its rescale miss, the count now moves -- up
on an undershoot, down when the bumps already sit at their smallest -- each
count at the amplitude that shares the gap (never above --meander-amplitude),
and the closest count's amplitude is refined on the measured lengths.

The geometry is the issue's: a coupled P/N run between two foreign rails.
P's bumps can only go up (N is below it), toward the rail GAP mm off.

    python3 tests/test_1583_intra_pair_adds_bumps.py
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

from kicad_parser import BoardInfo, Net, PCBData, Segment       # noqa: E402
from routing_config import GridRouteConfig                     # noqa: E402
import length_matching as lm                                    # noqa: E402

P, N, RAIL = 1, 2, 3
Y = 115.0
GAP = 1.125           # copper to copper, P to the rail: the issue's rail 1.2 mm off the pair
DELTA = 5.99          # the issue's P/N gap
TOL = 0.1
P_Y = Y - 0.15        # P's centreline


def _seg(x1, y1, x2, y2, w, layer, net):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                   width=w, layer=layer, net_id=net)


def _board(gap=GAP, jogs=False):
    """P and N coupled at gap 0.15 / width 0.15 (pitch 0.3) on F.Cu, and a
    0.2 mm foreign rail `gap` mm (copper to copper) above P and below N, on
    both layers. `jogs` brings the upper rail in to 0.55 * gap over two
    stretches, so the bumps there are clamped harder than the rest."""
    p = [_seg(105.0, P_Y, 135.0, P_Y, 0.15, 'F.Cu', P)]
    n = [_seg(105.0, Y + 0.15, 135.0, Y + 0.15, 0.15, 'F.Cu', N)]
    top, bot = P_Y - 0.075 - gap - 0.1, Y + 0.15 + 0.075 + gap + 0.1
    pts = [(103.0, top), (137.0, top)]
    if jogs:
        near = P_Y - 0.075 - 0.55 * gap - 0.1
        pts = [(103, top), (113, top), (113, near), (119, near), (119, top),
               (124, top), (124, near), (127, near), (127, top), (137, top)]
    rails = [_seg(x1, y1, x2, y2, 0.2, la, RAIL) for la in ('F.Cu', 'B.Cu')
             for (x1, y1), (x2, y2) in zip(pts, pts[1:])]
    rails += [_seg(103.0, bot, 137.0, bot, 0.2, la, RAIL) for la in ('F.Cu', 'B.Cu')]
    bi = BoardInfo(layers={}, board_bounds=(100.0, 100.0, 140.0, 130.0),
                   copper_layers=['F.Cu', 'B.Cu'])
    pcb = PCBData(board_info=bi, footprints={}, vias=[], pads_by_net={},
                  nets={P: Net(P, '/CK_P'), N: Net(N, '/CK_N'), RAIL: Net(RAIL, '/RAIL')},
                  segments=p + n + rails)
    return pcb, p, n


def _cfg(amplitude, spacing):
    c = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.15,
                        clearance=0.15, diff_pair_gap=0.15)
    c.meander_amplitude = amplitude
    c.meander_spacing = spacing
    c.length_match_tolerance = TOL
    c.diff_pair_intra_match = True
    return c


def _length(segs):
    return math.fsum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) for s in segs)


def _first_shot(segs, cfg, pcb, delta=DELTA):
    """The count + amplitude the matcher tries first (its own formula), and
    what that shot adds."""
    chamfer = lm.resolve_meander_chamfer(cfg, P, 'F.Cu', 0.15)
    count = max(1, math.ceil(delta / (2 * cfg.meander_amplitude - 2.34 * chamfer)))
    amp = max(0.1, min((delta / count + 2.34 * chamfer) / 2, cfg.meander_amplitude))
    out, placed = lm.apply_meanders_to_route(
        segs, delta, cfg, pcb_data=pcb, net_id=P, paired_net_id=N,
        amplitude_override=amp, min_bumps=count)
    return count, out, placed, _length(out) - _length(segs)


def run():
    fails = []

    def check(name, cond, detail=''):
        print(('  PASS ' if cond else '  FAIL ') + name + (f'  ({detail})' if detail else ''))
        if not cond:
            fails.append(name)

    for amplitude, spacing in ((1.0, 2.0), (1.0, 4.0)):
        pcb, p, _n = _board()
        cfg = _cfg(amplitude, spacing)
        base = _length(p)
        count, _o, placed, added = _first_shot(p, cfg, pcb)
        tag = f'amplitude {amplitude}, spacing {spacing} W'
        # The precondition: the rails clamp the first shot's bumps, so it
        # undershoots -- without it this test exercises nothing.
        check(f'{tag}: the first shot undershoots ({placed} bumps)',
              placed == count and added < DELTA - TOL, f'adds {added:.3f} of {DELTA}')
        segs, total, bumps = lm._lengthen_net_with_meanders(
            p, P, N, [], 0.0, base, base + DELTA, DELTA, cfg, pcb)
        got = total - base
        check(f'{tag}: the leg is matched within {TOL} mm',
              abs(got - DELTA) <= TOL, f'adds {got:.3f}, {bumps} bumps')
        check(f'{tag}: it took more bumps than the first shot',
              bumps > count, f'{bumps} vs {count}')
        reach = max(max(abs(s.start_y - P_Y), abs(s.end_y - P_Y)) for s in segs)
        check(f'{tag}: no bump is taller than --meander-amplitude',
              reach <= amplitude + 1e-6, f'{reach:.3f}')
        check(f'{tag}: the bumps go away from the partner',
              all(min(s.start_y, s.end_y) <= P_Y + 1e-9 for s in segs))

    # DOWN: at spacing 4 W the chamfer is 0.3, so amplitude 0.7 IS a bump's
    # smallest (a 0.1 mm riser between its chamfers). The first shot's 6
    # bumps sit there and overshoot 3.5 mm by 0.68; fewer bumps share it.
    pcb, p, _n = _board()
    cfg = _cfg(0.7, 4.0)
    base = _length(p)
    count, _o, placed, added = _first_shot(p, cfg, pcb, delta=3.5)
    check('bumps at their smallest: the first shot overshoots',
          added > 3.5 + TOL, f'{placed} bumps add {added:.3f} of 3.5')
    _segs, total, bumps = lm._lengthen_net_with_meanders(
        p, P, N, [], 0.0, base, base + 3.5, 3.5, cfg, pcb)
    check('bumps at their smallest: fewer bumps match it',
          abs(total - base - 3.5) <= TOL and bumps < placed,
          f'adds {total - base:.3f}, {bumps} bumps')

    # REFINE: the rail jogs in over two stretches, so the bumps there clamp
    # harder and every count at its sharing amplitude comes up short; the
    # closest count's amplitude is raised on the measured lengths.
    pcb, p, _n = _board(gap=1.0, jogs=True)
    cfg = _cfg(1.0, 2.0)
    base = _length(p)
    _segs, total, bumps = lm._lengthen_net_with_meanders(
        p, P, N, [], 0.0, base, base + 14.0, 14.0, cfg, pcb)
    check('a rail that jogs in: matched within the tolerance',
          abs(total - base - 14.0) <= TOL, f'adds {total - base:.3f}, {bumps} bumps')

    # Through the intra-pair matcher: the record says matched, not
    # 'improved' / 'meanders-ran-out'. No pads on the board, so the matcher
    # reads the routing result's own lengths; N's stub makes it DELTA longer.
    pcb, p, n = _board()
    cfg = _cfg(1.0, 2.0)
    result = {'new_segments': list(p) + list(n), 'new_vias': [],
              'n_src_stub_length': DELTA}
    status = {}
    lm.apply_intra_pair_length_matching(result, cfg, pcb, p_net_id=P, n_net_id=N,
                                        status=status)
    check('apply_intra_pair_length_matching: matched',
          status.get('status') == 'matched' and status.get('reason') is None, str(status))
    check('the matched leg replaced P in the result',
          abs(_length([s for s in result['new_segments'] if s.net_id == P])
              - (_length(p) + DELTA)) <= TOL)

    # A leg the first shot matches is meandered exactly as before: amplitude
    # 0.7 fits under the rail, so its first shot is the answer.
    pcb, p, _n = _board()
    cfg = _cfg(0.7, 2.0)
    base = _length(p)
    count, first, placed, added = _first_shot(p, cfg, pcb)
    segs, total, bumps = lm._lengthen_net_with_meanders(
        p, P, N, [], 0.0, base, base + DELTA, DELTA, cfg, pcb)
    check('amplitude 0.7: the first shot matches', abs(added - DELTA) <= TOL,
          f'adds {added:.3f}')
    check('amplitude 0.7: the result is the first shot, unchanged',
          bumps == placed and [(s.start_x, s.start_y, s.end_x, s.end_y) for s in segs]
          == [(s.start_x, s.start_y, s.end_x, s.end_y) for s in first])

    print()
    if fails:
        print(f'FAIL: {len(fails)} check(s): {fails}')
        return 1
    print('PASS: an intra-pair undershoot adds bumps before it runs out (#1583)')
    return 0


if __name__ == '__main__':
    sys.exit(run())

#!/usr/bin/env python3
"""The BGA channel escape's raw-short checks see the part's OWN balls.

  python3 tests/test_bga_own_ball_short.py

The pad-aware checks (the collision resolver's candidates, the pad-aware
trigger and `repair_pad_crossings`) were handed `foreign_pads`, the pads of
every OTHER footprint. The channel geometry keeps a stub off every ball, but
the reroute helpers build theirs freely: `try_jogged_route` runs a 45-degree
stub from the ball to the route's channel, which an earlier reroute can have
moved 1.5 pitches away, straight across the diagonal neighbour. On daisho U1
that laid a V1P2 F.Cu stub through the centre of NC ball N8 -- a pad-segment
short that shipped to the routed board. Pinned here:

1. the geometry: daisho's jog, rebuilt at the origin. Given only foreign
   pads, `try_jogged_route` accepts it on F.Cu through the neighbour ball
   (so the fixture reproduces the short); given the ball too, it refuses
   F.Cu and takes the inner layer, where an SMD ball has no copper; and a
   ball of the route's OWN net does not refuse it;
2. the engine: on a real BGA fanout (interf_u U9) every raw-short check the
   core makes -- the pad-aware trigger, `resolve_collisions` and
   `repair_pad_crossings` -- is handed the part's own pads (spied, so a core
   that went back to `foreign_pads` fails here, not in a routed board);
3. `_seg_hits_pad`'s reach test answers exactly as the sampled test without
   it, on random segments against random (incl. rotated) pads: it only skips
   pads the samples cannot reach. The own balls lengthen the pad list every
   check walks, and the reach test is what keeps that free.
"""
import io
import math
import os
import random
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

from synth import make_pad  # noqa: E402
from bga_fanout.types import BGAGrid, Channel, FanoutRoute  # noqa: E402
from bga_fanout import reroute  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'interf_u_unrouted_placed.kicad_pcb')
LAYERS = ['F.Cu', 'In1.Cu']
LMAP = {L: i for i, L in enumerate(LAYERS)}


def _quiet(fn):
    buf, old = io.StringIO(), sys.stdout
    sys.stdout = buf
    try:
        return fn()
    finally:
        sys.stdout = old


def _jog(pads, own_net=1):
    """daisho U1's V1P2 jog at the origin: ball (0,0) escaping up, its route
    on the vertical channel x=-1.5 (1.5 pitches west, after a reroute), the
    farther channel x=-2.5. The 45-degree stub (0,0)->(-1.5,-1.5) crosses the
    diagonal neighbour at (-1,-1)."""
    grid = BGAGrid(pitch_x=1.0, pitch_y=1.0, rows=[float(i) for i in range(-3, 4)],
                   cols=[float(i) for i in range(-3, 4)], center_x=0.0, center_y=0.0,
                   min_x=-3.0, max_x=3.0, min_y=-3.0, max_y=3.0)
    ball = make_pad(own_net, 0.0, 0.0, ref='U1', num='P9', shape='circle',
                    size_x=0.5, size_y=0.5)
    ch = Channel('vertical', -1.5, 1)
    route = FanoutRoute(pad=ball, pad_pos=(0.0, 0.0), stub_end=(-1.5, -1.5),
                        exit_pos=(-1.5, -3.5), channel=ch, escape_dir='up',
                        layer='F.Cu')
    return reroute.try_jogged_route(
        route, Channel('vertical', -2.5, 0), grid, 0.5, [], [], LAYERS,
        0.1651, 0.09, None, False, None, None, LMAP, pads)


def check_geometry(fails):
    nc = make_pad(2, -1.0, -1.0, ref='U1', num='N8', shape='circle',
                  size_x=0.5, size_y=0.5)
    old = _jog([])
    if old is None or old[1] != 'F.Cu':
        fails.append(f"fixture: with no pads the jog should be taken on F.Cu "
                     f"(got {old and old[1]}) -- it no longer reproduces daisho's short")
        return
    stub = reroute.route_segments(old[0])[0]
    if not reroute._seg_hits_pad(*stub['start'], *stub['end'], nc):
        fails.append("fixture: the F.Cu stub does not cross the diagonal ball")
    new = _jog([nc])
    if new is None:
        fails.append("with the own ball listed, the jog was refused on EVERY layer "
                     "(In1.Cu has no ball copper)")
    elif new[1] != 'In1.Cu':
        fails.append(f"with the own ball listed, the jog still took {new[1]} "
                     f"through it")
    same = _jog([make_pad(1, -1.0, -1.0, ref='U1', num='N8', shape='circle',
                          size_x=0.5, size_y=0.5)])
    if same is None or same[1] != 'F.Cu':
        fails.append("a ball of the route's OWN net refused the jog")


def check_engine(fails):
    if not os.path.isfile(BOARD):
        fails.append(f"missing board {BOARD}")
        return
    from kicad_parser import parse_kicad_pcb
    import bga_fanout
    pcb = parse_kicad_pcb(BOARD)
    fp = pcb.footprints['U9']
    own = {id(p) for p in fp.pads}
    seen = {}

    def spy(name, orig, pos):
        def wrapped(*a, **k):
            pads = k.get('foreign_pads', a[pos] if len(a) > pos else None)
            seen.setdefault(name, []).append(
                bool(pads) and own <= {id(p) for p in pads})
            return orig(*a, **k)
        return wrapped
    saved = {n: getattr(bga_fanout, n) for n in
             ('route_clear_of_foreign_pads', 'resolve_collisions', 'repair_pad_crossings')}
    try:
        bga_fanout.route_clear_of_foreign_pads = spy(
            'route_clear_of_foreign_pads', saved['route_clear_of_foreign_pads'], 1)
        bga_fanout.resolve_collisions = spy(
            'resolve_collisions', saved['resolve_collisions'], 15)
        bga_fanout.repair_pad_crossings = spy(
            'repair_pad_crossings', saved['repair_pad_crossings'], 15)
        _quiet(lambda: bga_fanout.generate_bga_fanout(fp, pcb))
    finally:
        for n, f in saved.items():
            setattr(bga_fanout, n, f)
    if not seen.get('route_clear_of_foreign_pads'):
        fails.append("the pad-aware trigger never ran on U9 (no routes?) -- the "
                     "engine check measured nothing")
    for name, calls in sorted(seen.items()):
        if not all(calls):
            fails.append(f"{name}: {calls.count(False)} of {len(calls)} call(s) were "
                         f"NOT handed U9's own pads")


def _ref_hits(x1, y1, x2, y2, pad, samples=16, margin=0.0):
    """_seg_hits_pad's sampled rect test, without the reach test."""
    eps = 1e-9 if margin <= 0.0 else -1e-9
    hx = pad.size_x / 2.0 + margin + eps
    hy = pad.size_y / 2.0 + margin + eps
    rr = getattr(pad, 'rect_rotation', 0.0)
    c, s = math.cos(math.radians(rr)), math.sin(math.radians(rr))
    for t in range(samples + 1):
        f = t / samples
        dx = x1 + (x2 - x1) * f - pad.global_x
        dy = y1 + (y2 - y1) * f - pad.global_y
        if rr:
            dx, dy = dx * c + dy * s, -dx * s + dy * c
        if abs(dx) <= hx and abs(dy) <= hy:
            return True
    return False


def check_reach(fails):
    rng = random.Random(1328)
    hits = 0
    for _ in range(20000):
        pad = make_pad(1, rng.uniform(-2, 2), rng.uniform(-2, 2),
                       size_x=rng.uniform(0.1, 1.5), size_y=rng.uniform(0.1, 1.5),
                       rect_rotation=rng.choice((0.0, 0.0, 30.0, -45.0, 17.5)))
        x1, y1 = rng.uniform(-3, 3), rng.uniform(-3, 3)
        x2, y2 = x1 + rng.uniform(-2, 2), y1 + rng.uniform(-2, 2)
        m = rng.choice((0.0, 0.0, 0.09, 0.2))
        want = _ref_hits(x1, y1, x2, y2, pad, margin=m)
        hits += want
        if reroute._seg_hits_pad(x1, y1, x2, y2, pad, margin=m) != want:
            fails.append(f"_seg_hits_pad disagrees with the sampled test: segment "
                         f"({x1:.3f},{y1:.3f})-({x2:.3f},{y2:.3f}) margin {m} vs pad "
                         f"at ({pad.global_x:.3f},{pad.global_y:.3f})")
            return
    if not 1000 < hits < 19000:
        fails.append(f"the random cases do not discriminate ({hits} hits of 20000)")


def main():
    print('=' * 60)
    print("BGA channel escape: the raw-short checks see the part's own balls")
    print('=' * 60)
    fails = []
    check_geometry(fails)
    check_engine(fails)
    check_reach(fails)
    for f in fails:
        print(f'  FAIL: {f}')
    if fails:
        return 1
    print("PASS: daisho's through-ball jog is refused on F.Cu once the own ball is "
          "listed (and taken on the inner layer); every raw-short check of a real "
          "BGA fanout is handed the part's own pads; the reach test answers as the "
          "sampled test on 20000 random cases")
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""Issue #1515: close_soft_joints bridges a soft joint whose bridge lies
wholly inside existing copper, even where that copper is already short of
the gate's clearance.

A route sampled onto an off-grid, any-angle stub can land MID-SPAN, 0.0516 mm
short of the stub's free end. The two ends are a soft joint
(check_drc counts it; connectivity.strict_joint_roots keeps a mid-span
landing flagged on purpose), and close_soft_joints is the repair. Its bridge
runs from the stub's free end along the stub's own centreline to the landing,
so at the stub's width it adds no copper at all. The gate still refused it
whenever the stub itself was short of the gate's clearance (laid by an
earlier step at a looser one): refusing then drops the repair and keeps the
violation, #617's argument, which holds exactly when the bridge adds nothing.

Geometry: stub P0=(10.6225, 10.4183) -> E=(10.2, 9.65), width 0.15; the
route lands at the stub's 16/17 sample, 0.0516 mm from E; a foreign pad's
corner sits 0.2034 mm from E, off the stub's normal (needs 0.225 at
clearance 0.15, 0.175 at 0.1).

  1. at clearance 0.15 the inside-stub bridge is laid (it was refused);
  2. the bridge lies on the stub's centreline, from E to the landing;
  3. control: a bridge across a gap BEYOND the stub's free end adds copper,
     so the same pad still refuses it at 0.15 ...
  4. ... and admits it at 0.1, as before.

    python3 tests/test_1515_soft_joint_bridge_inside_stub.py
"""
import contextlib
import io
import math
import os
import sys
import tempfile

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _d in ('py_router', 'rust_router', os.path.join('tests', 'oracle')):
    sys.path.insert(0, os.path.join(_ROOT, _d))

from constraint_agreement import write_board  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from pcb_modification import close_soft_joints  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402
from single_ended_routing import _seg_foreign_pad_dist  # noqa: E402

FAILS = []
W = 0.15
P0 = (10.6225, 10.4183)
E = (10.2, 9.65)
L = math.hypot(E[0] - P0[0], E[1] - P0[1])
U = ((P0[0] - E[0]) / L, (P0[1] - E[1]) / L)       # from E into the stub
N = (U[1], -U[0])                                    # its normal, away from it
LAND = (P0[0] + 16 / 17 * (E[0] - P0[0]), P0[1] + 16 / 17 * (E[1] - P0[1]))
CORNER = (E[0] + 0.2034 * N[0], E[1] + 0.2034 * N[1])
PAD = (CORNER[0] + 0.1, CORNER[1] - 0.1)             # 0.2 square, corner at CORNER


def check(cond, msg):
    print(('  ok   ' if cond else '  FAIL ') + msg)
    if not cond:
        FAILS.append(msg)


def r6(p):
    return (round(p[0], 6), round(p[1], 6))


def board(td, name, route_end, route_from):
    path = os.path.join(td, name + '.kicad_pcb')
    segs = [(*r6(P0), *r6(E), W, 'F.Cu', 1),
            (*r6(route_from), *r6(route_end), W, 'F.Cu', 1),
            (9.0, 9.0, *r6(route_from), W, 'F.Cu', 1)]
    write_board(path, segments=segs, footprints=[
        {'ref': 'U1', 'x': P0[0], 'y': P0[1], 'net_id': 1, 'net_name': 'A',
         'size': 0.3},
        {'ref': 'U2', 'x': 9.0, 'y': 9.0, 'net_id': 1, 'net_name': 'A',
         'size': 0.3},
        {'ref': 'C1', 'x': round(PAD[0], 6), 'y': round(PAD[1], 6),
         'net_id': 2, 'net_name': 'B', 'size': 0.2}])
    return parse_kicad_pcb(path)


def bridge(pcb, clearance):
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], grid_step=0.05,
                          clearance=clearance, track_width=W)
    added = []
    with contextlib.redirect_stdout(io.StringIO()):
        close_soft_joints(added, pcb, {1}, cfg)
    return [s for r in added for s in (r.get('new_segments') or [])]


def on_stub(p):
    t = ((p[0] - E[0]) * U[0] + (p[1] - E[1]) * U[1])
    q = (E[0] + t * U[0], E[1] + t * U[1])
    return math.hypot(p[0] - q[0], p[1] - q[1]) < 1e-5 and -1e-6 <= t <= L


def main():
    with tempfile.TemporaryDirectory() as td:
        pcb = board(td, 'inside', LAND, (9.7952, 9.6952))
        d = _seg_foreign_pad_dist(pcb, 1, *E, *LAND, 'F.Cu', base_clearance=0.15)
        print(f'fixture: bridge centreline {d:.4f} mm from the foreign pad, '
              f'landing {math.hypot(LAND[0] - E[0], LAND[1] - E[1]):.4f} mm '
              f'short of the stub end')
        check(W / 2 + 0.1 <= d < W / 2 + 0.15,
              'fixture: the pad is inside the 0.15 gate and outside the 0.1 one')

        print('1-2. a bridge inside the stub at clearance 0.15')
        got = bridge(pcb, 0.15)
        check(len(got) == 1, f'one bridge laid (got {len(got)})')
        if got:
            b = got[0]
            ends = {r6((b.start_x, b.start_y)), r6((b.end_x, b.end_y))}
            check(ends == {r6(E), r6(LAND)},
                  'it joins the stub end to the landing')
            check(on_stub((b.start_x, b.start_y)) and on_stub((b.end_x, b.end_y))
                  and b.width <= W, 'it lies on the stub centreline, no wider')

        print('3-4. control: a bridge beyond the stub end adds copper')
        beyond = (E[0] - 0.05 * U[0], E[1] - 0.05 * U[1])
        frm = (beyond[0] - 0.4 * U[0], beyond[1] - 0.4 * U[1])
        pcb = board(td, 'beyond', beyond, frm)
        check(bridge(pcb, 0.15) == [], 'refused at 0.15 (the pad is too close)')
        pcb = board(td, 'beyond01', beyond, frm)
        check(len(bridge(pcb, 0.1)) == 1, 'laid at 0.1')

    if FAILS:
        print(f'\nFAILED ({len(FAILS)})')
        sys.exit(1)
    print('\nPASS')


if __name__ == '__main__':
    main()

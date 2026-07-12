#!/usr/bin/env python3
"""Regression test for issue #242 - vertical-escape edge-pair loop-back.

`build_converge_route` converges a differential edge pair to pair spacing with
45 stubs. For a pair escaping up/down it must converge in X (perpendicular to
the escape) and keep Y monotonic toward the edge. It used to always converge in
Y, so for a diagonal pair escaping up the top pad was sent *down* toward the
pair center before exiting up - a loop-back stub that doubles back toward the
BGA (e.g. butterstick /SYZYGY1.D4_P), and the pair stayed uncoupled (one pitch
apart) instead of converging.

This builds that geometry (diagonal edge pair escaping up) and asserts both
legs run monotonically toward the edge and converge to pair spacing.

Run:
    python3 tests/test_diff_converge_vertical_loopback.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Pad
from bga_fanout.types import BGAGrid
from bga_fanout import build_converge_route


def _pad(num, x, y, net_id, net_name):
    return Pad(component_ref='U4', pad_number=num, global_x=x, global_y=y,
               local_x=0.0, local_y=0.0, size_x=0.3, size_y=0.3, shape='circle',
               layers=['*.Cu'], net_id=net_id, net_name=net_name, drill=0.3)


def _poly_y(route):
    """Y coordinates of a converge route's polyline, pad -> stub -> exit."""
    return [route.pad_pos[1], route.stub_end[1], route.exit_pos[1]]


def main():
    pitch = 0.8
    rows = [95.7 + i * pitch for i in range(6)]
    cols = [126.7 + i * pitch for i in range(6)]
    grid = BGAGrid(
        pitch_x=pitch, pitch_y=pitch, rows=rows, cols=cols,
        center_x=(cols[0] + cols[-1]) / 2, center_y=(rows[0] + rows[-1]) / 2,
        min_x=cols[0] - pitch / 2, max_x=cols[-1] + pitch / 2,
        min_y=rows[0] - pitch / 2, max_y=rows[-1] + pitch / 2,
    )

    # Diagonal edge pair (like butterstick U4.A2/B1) escaping up.
    p_pad = _pad('A2', 128.30, 96.50, 44, '/SYZYGY1.D4_P')
    n_pad = _pad('B1', 127.50, 97.30, 54, '/SYZYGY1.D4_N')

    track_width, diff_pair_gap = 0.10, 0.10
    half_pair_spacing = (track_width + diff_pair_gap) / 2
    pair_spacing = track_width + diff_pair_gap

    kw = dict(pads_horizontal=False, escape_dir='up', is_edge=True, channel=None,
              grid=grid, channels=[], layers=['F.Cu'], exit_margin=0.5,
              half_pair_spacing=half_pair_spacing, use_adjacent_channels_h=False,
              pair_layer_assignments={}, pair_id='/SYZYGY1.D4')

    p_route = build_converge_route(p_pad, True, p_pad, n_pad, **kw)
    n_route = build_converge_route(n_pad, False, p_pad, n_pad, **kw)

    # 1) Both legs must run monotonically UP (Y non-increasing) - no loop-back.
    for nm, r in (('P', p_route), ('N', n_route)):
        ys = _poly_y(r)
        for a, b in zip(ys, ys[1:]):
            if b > a + 1e-9:
                print(f"FAIL: {nm} leg loops back toward the BGA "
                      f"(y goes {a:.3f} -> {b:.3f}; escape is 'up')")
                return 1

    # 2) The pair must converge to pair spacing in X at the exit.
    dx = abs(p_route.exit_pos[0] - n_route.exit_pos[0])
    if abs(dx - pair_spacing) > 1e-6:
        print(f"FAIL: pair not converged to pair spacing "
              f"(exit dx={dx:.4f}, expected {pair_spacing:.4f})")
        return 1

    print(f"PASS: #242 vertical-escape edge pair runs monotonically up and "
          f"converges in X (exit dx={dx:.4f} = pair spacing)")
    return 0


if __name__ == '__main__':
    sys.exit(main())

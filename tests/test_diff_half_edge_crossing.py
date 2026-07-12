#!/usr/bin/env python3
"""Regression test for issue #242 - half-edge diff-pair escape stubs crossing.

`bga_fanout`'s half-edge diff-pair builder (`build_half_edge_route`) routes the
inner pad in a "tent" that converges with the edge pad at pair spacing. The
convergence *side* (above/below the edge pad) used to be chosen purely by which
BGA edge was nearest the inner pad, ignoring where the inner pad actually sits
relative to the edge pad. When the inner pad sat on the opposite side from the
chosen channel, its escape crossed the edge pad's straight track - a
SEGMENT-CROSSING in the fanout output (e.g. schoko /USB1_DP x /USB1_DN).

This builds the schoko-like geometry directly (edge pad on the left edge, inner
pad one column in and *below* it, but high enough in a tall grid that the
nearest-edge heuristic would pick a channel *above*) and asserts the emitted P
and N polylines do not cross.

Run:
    python3 tests/test_diff_half_edge_crossing.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Pad
from bga_fanout.types import BGAGrid, Channel
from bga_fanout import build_half_edge_route, generate_tracks_from_routes


def _seg_cross(a1, a2, b1, b2):
    """True if open segments a1-a2 and b1-b2 properly cross (shared endpoints
    and collinear touching don't count)."""
    def o(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    EPS = 1e-9
    # Ignore segments that share an endpoint (legitimate joins within a net).
    for p in (a1, a2):
        for q in (b1, b2):
            if abs(p[0] - q[0]) < EPS and abs(p[1] - q[1]) < EPS:
                return False
    d1 = o(b1, b2, a1)
    d2 = o(b1, b2, a2)
    d3 = o(a1, a2, b1)
    d4 = o(a1, a2, b2)
    return ((d1 > EPS and d2 < -EPS) or (d1 < -EPS and d2 > EPS)) and \
           ((d3 > EPS and d4 < -EPS) or (d3 < -EPS and d4 > EPS))


def _polylines_cross(tracks_a, tracks_b):
    for ta in tracks_a:
        for tb in tracks_b:
            if _seg_cross(ta['start'], ta['end'], tb['start'], tb['end']):
                return (ta, tb)
    return None


def _pad(ref, num, x, y, net_id, net_name):
    return Pad(component_ref=ref, pad_number=num, global_x=x, global_y=y,
               local_x=0.0, local_y=0.0, size_x=0.3, size_y=0.3, shape='circle',
               layers=['*.Cu'], net_id=net_id, net_name=net_name, drill=0.3)


def main():
    pitch = 0.8
    rows = [10.0 + i * pitch for i in range(8)]   # tall grid: 10.0 .. 15.6
    cols = [10.0 + i * pitch for i in range(8)]
    grid = BGAGrid(
        pitch_x=pitch, pitch_y=pitch, rows=rows, cols=cols,
        center_x=(cols[0] + cols[-1]) / 2, center_y=(rows[0] + rows[-1]) / 2,
        min_x=cols[0] - pitch / 2, max_x=cols[-1] + pitch / 2,
        min_y=rows[0] - pitch / 2, max_y=rows[-1] + pitch / 2,
    )
    # Horizontal channels at the inter-row midpoints.
    channels = [Channel(orientation='horizontal', position=rows[i] + pitch / 2, index=i)
                for i in range(len(rows) - 1)]

    # Edge pad (N) on the LEFT edge, upper area; inner pad (P) one column in and
    # one row BELOW it - but still high in the grid so the old nearest-edge
    # heuristic would have picked a channel ABOVE the edge pad.
    n_pad = _pad('U1', 'E1', cols[0], rows[1], 54, '/USB1_DN')   # edge, y=10.8
    p_pad = _pad('U1', 'F2', cols[1], rows[2], 44, '/USB1_DP')   # inner, y=11.6 (below)

    track_width = 0.125
    diff_pair_gap = 0.15
    half_pair_spacing = (track_width + diff_pair_gap) / 2
    layers = ['F.Cu']
    escape_dir = 'left'
    pair_id = '/USB1_D'

    p_route = build_half_edge_route(
        p_pad, True, p_pad, n_pad, escape_dir, grid, channels, layers,
        0.5, half_pair_spacing, pair_id)
    n_route = build_half_edge_route(
        n_pad, False, p_pad, n_pad, escape_dir, grid, channels, layers,
        0.5, half_pair_spacing, pair_id)

    p_tracks, _, _ = generate_tracks_from_routes([p_route], track_width, 'F.Cu')
    n_tracks, _, _ = generate_tracks_from_routes([n_route], track_width, 'F.Cu')

    hit = _polylines_cross(p_tracks, n_tracks)
    if hit is not None:
        ta, tb = hit
        print("FAIL: #242 regression - P/N half-edge escape stubs cross")
        print(f"  P seg {ta['start']} -> {ta['end']}")
        print(f"  N seg {tb['start']} -> {tb['end']}")
        return 1

    # Sanity 1: the inner pad must converge on its own (lower) side of the edge
    # pad, i.e. the inner exit Y should be below the edge pad's Y.
    edge_y = n_pad.global_y
    inner_exit_y = p_route.exit_pos[1]
    if not inner_exit_y > edge_y:
        print(f"FAIL: inner pad converged to wrong side "
              f"(inner exit y={inner_exit_y:.3f}, edge y={edge_y:.3f}; "
              f"inner pad sits below the edge pad)")
        return 1

    # Sanity 2: no loop-back. The inner stub must reach its convergence point
    # at or inside (to the right of, for a left escape) the exit point, so the
    # final straight run goes out toward the edge - never back toward the BGA
    # (the #242 overshoot/loop-back).
    inner_stub_x = p_route.stub_end[0]
    inner_exit_x = p_route.exit_pos[0]
    if inner_stub_x < inner_exit_x:
        print(f"FAIL: inner stub overshoots the exit and loops back toward the "
              f"BGA (stub x={inner_stub_x:.3f} < exit x={inner_exit_x:.3f})")
        return 1

    print("PASS: #242 half-edge diff-pair escape stubs do not cross, no "
          f"loop-back (inner converges below edge: {inner_exit_y:.3f} > "
          f"{edge_y:.3f}; stub x={inner_stub_x:.3f} >= exit x={inner_exit_x:.3f})")
    return 0


if __name__ == '__main__':
    sys.exit(main())

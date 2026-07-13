#!/usr/bin/env python3
"""Phase-3 tap routing must not re-emit an endpoint connector the net already has.

A Phase-3 tap edge launches from points ON the net's existing copper
(get_all_segment_tap_points). When its A* start cell maps back through
tap_point_map to an off-grid original point that an earlier edge (the Phase-1
main route, or a previous tap) already bridged to that same grid cell,
_path_to_segments_vias re-emits the identical original->grid endpoint
connector: the output board then carries two coincident copies of a tiny
(~0.02-0.05 mm) segment at one stub tip. Observed on a synthetic fanout-stub
board: every stubbed net shipped one duplicated connector pair, e.g. two
identical 0.02 mm segments (40.32, 7.2)->(40.30, 7.2) on one net.

route_multipoint_taps now filters each edge's emitted segments against the
net's accumulated copper (_drop_segments_already_present) before committing
them, so the duplicate is never born. This tests that filter's semantics: an
exact twin (either orientation) is dropped; anything that differs in
geometry, layer, width, or net is copper the net does not have and is kept.

    python3 tests/test_duplicate_endpoint_connector.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Segment
from single_ended_routing import _drop_segments_already_present


def _seg(x1, y1, x2, y2, layer='F.Cu', w=0.2, net=5):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2, width=w,
                   layer=layer, net_id=net)


def run():
    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)

    # The observed duplicate: the Phase-1 main route already bridged the stub
    # tip (40.32, 7.2) to the grid point (40.30, 7.2); a Phase-3 tap edge
    # launching from the same tap point re-emits the identical connector.
    tip_connector = _seg(40.32, 7.2, 40.30, 7.2)
    existing = [tip_connector, _seg(40.30, 7.2, 40.30, 15.0)]

    # 1. An exact twin is dropped; the edge's genuinely new copper is kept.
    new_route = _seg(40.30, 7.2, 46.0, 7.2)
    kept = _drop_segments_already_present(
        [_seg(40.32, 7.2, 40.30, 7.2), new_route], existing)
    check("1 exact twin dropped", kept == [new_route])

    # 2. The reversed orientation is the same copper: dropped too.
    kept = _drop_segments_already_present(
        [_seg(40.30, 7.2, 40.32, 7.2)], existing)
    check("2 reversed twin dropped", kept == [])

    # 3. Same endpoints on another layer is different copper: kept.
    other_layer = _seg(40.32, 7.2, 40.30, 7.2, layer='B.Cu')
    kept = _drop_segments_already_present([other_layer], existing)
    check("3 other layer kept", kept == [other_layer])

    # 4. Same geometry but a different width is not a pure twin: kept
    #    (a wider segment changes the copper).
    wider = _seg(40.32, 7.2, 40.30, 7.2, w=0.4)
    kept = _drop_segments_already_present([wider], existing)
    check("4 different width kept", kept == [wider])

    # 5. Same geometry on a different net is kept (the filter must never eat
    #    another net's copper if given a mixed list).
    other_net = _seg(40.32, 7.2, 40.30, 7.2, net=9)
    kept = _drop_segments_already_present([other_net], existing)
    check("5 other net kept", kept == [other_net])

    # 6. Order and multiplicity of the non-duplicates are preserved.
    a = _seg(0, 0, 1, 0)
    b = _seg(1, 0, 1, 1)
    kept = _drop_segments_already_present(
        [a, _seg(40.32, 7.2, 40.30, 7.2), b], existing)
    check("6 order preserved", kept == [a, b])

    # 7. Sub-nanometer float noise still matches (round-to-6 key): the grid
    #    end computed as 40.300000000000004 equals the stored 40.30.
    noisy = _seg(40.32, 7.2, 40.300000000000004, 7.2)
    kept = _drop_segments_already_present([noisy], existing)
    check("7 float-noise twin dropped", kept == [])

    print("=" * 60)
    if fails:
        for f in fails:
            print(f"  FAIL  {f}")
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("  PASS  exact/reversed/float-noise twins dropped; layer, width,")
    print("        net differences and non-duplicates kept in order (7 cases)")
    return 0


if __name__ == '__main__':
    sys.exit(run())

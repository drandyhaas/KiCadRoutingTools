#!/usr/bin/env python3
"""The strict-fragment view of check_net_connectivity.

Run 6's VCC3V3 graded 25/27 pads connected while KiCad saw the copper in 7
islands -- the permissive credit rules (0.4mm pad proximity, grading-epsilon
endpoint caps, legacy zone blob) merged true track fragments. The strict view
(`strict_fragments=True`) is the PLANNER's model; the default stays the
grader's. Pinned here:

  a. two fat rail tips a hair apart: default connected, strict split;
  b. a 0.05mm-gap soft joint at 0.2 width (0.15mm lens): ONE component in
     BOTH views (the pinned test_component_multipoint semantics survive);
  c. pad-proximity phantom (endpoint near a pad but outside its copper):
     default joined, strict split;
  d. the EXACT pad rules still credit in strict mode (endpoint-in-pad);
  e. net_copper_fragments counts fragments, pad-less fragments, anchors.
"""
import os
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router'),      # #522 layout
          os.path.join(ROOT, 'py_tools'), os.path.join(ROOT, 'rust_router')):
    if p not in sys.path:
        sys.path.insert(0, p)

from synth import make_pad, make_seg
from check_connected import (check_net_connectivity, net_copper_fragments,
                             STRICT_JOINT_OVERLAP)


def _components(res):
    return res['num_components']


def test_fat_tip_gap_splits_only_in_strict():
    # Two 0.6mm-wide tracks whose tips sit 0.58mm apart (center-to-center):
    # cap radii 0.3+0.3=0.6 > 0.58 -> a 0.02mm lens. The grader shades that
    # connected; the planner must not (0.02 < STRICT_JOINT_OVERLAP).
    pads = [make_pad(1, 0.0, 0.0, num='1'), make_pad(1, 10.0, 0.0, num='2')]
    segs = [make_seg(0.0, 0.0, 4.71, 0.0, width=0.6),
            make_seg(5.29, 0.0, 10.0, 0.0, width=0.6)]
    perm = check_net_connectivity(1, segs, [], pads, [])
    strict = check_net_connectivity(1, segs, [], pads, [],
                                    strict_fragments=True)
    assert perm['connected'], perm
    assert not strict['connected'] and _components(strict) == 2, strict
    print("  PASS: fat-tip hair gap connected(grader) / split(planner)")


def test_soft_joint_stays_one_component():
    # 0.05mm endpoint gap at 0.2 width: lens depth 0.15 >= STRICT_JOINT_OVERLAP
    # -> one component in both views (the #317 soft-joint semantics).
    assert STRICT_JOINT_OVERLAP < 0.15
    pads = [make_pad(1, 0.0, 0.0, num='1'), make_pad(1, 10.0, 0.0, num='2')]
    segs = [make_seg(0.0, 0.0, 4.975, 0.0, width=0.2),
            make_seg(5.025, 0.0, 10.0, 0.0, width=0.2)]
    for strict in (False, True):
        res = check_net_connectivity(1, segs, [], pads, [],
                                     strict_fragments=strict)
        assert res['connected'], (strict, res)
    print("  PASS: 0.15mm soft-joint lens stays one component in both views")


def test_pad_proximity_phantom_splits_in_strict():
    # A thin track ending 0.09mm from a tiny pad's center: copper reach is
    # pad half 0.03 + cap 0.03 = 0.06 < 0.09 (NOT touching), but the default
    # pair rule's pad radius (0.4/4 = 0.1) joins it -- the phantom credit.
    # Strict drops the flat pad radius (0.06/4 = 0.015) and splits.
    pads = [make_pad(1, 0.0, 0.0, num='1', size_x=0.06, size_y=0.06),
            make_pad(1, 10.0, 0.0, num='2', size_x=0.06, size_y=0.06)]
    segs = [make_seg(0.09, 0.0, 9.91, 0.0, width=0.06)]
    perm = check_net_connectivity(1, segs, [], pads, [])
    strict = check_net_connectivity(1, segs, [], pads, [],
                                    strict_fragments=True)
    assert perm['connected'], perm
    assert not strict['connected'], strict
    print("  PASS: pad-proximity phantom joined(grader) / split(planner)")


def test_exact_pad_rules_survive_strict():
    # Endpoint INSIDE the pad copper: the #195 exact rule, live in both views.
    pads = [make_pad(1, 0.0, 0.0, num='1', size_x=0.5, size_y=0.5),
            make_pad(1, 10.0, 0.0, num='2', size_x=0.5, size_y=0.5)]
    segs = [make_seg(0.1, 0.0, 9.9, 0.0, width=0.2)]
    for strict in (False, True):
        res = check_net_connectivity(1, segs, [], pads, [],
                                     strict_fragments=strict)
        assert res['connected'], (strict, res)
    print("  PASS: exact endpoint-in-pad credit survives the strict view")


def test_net_copper_fragments_census():
    # Three true fragments: pad1--seg--+ tip gap +--seg--pad2, plus a stray
    # pad-less island far away.
    pads = [make_pad(1, 0.0, 0.0, num='1'), make_pad(1, 10.0, 0.0, num='2')]
    segs = [make_seg(0.0, 0.0, 4.71, 0.0, width=0.6),
            make_seg(5.29, 0.0, 10.0, 0.0, width=0.6),
            make_seg(20.0, 5.0, 22.0, 5.0, width=0.2)]
    frag = net_copper_fragments(1, segs, [], pads, [])
    assert frag['fragments'] == 3, frag
    assert frag['padless_fragments'] == 1, frag
    assert len(frag['fragment_anchors']) == 3, frag
    assert frag['zone_blob_fallback'] is False, frag
    print("  PASS: fragment census 3 fragments (1 pad-less), anchors present")


if __name__ == '__main__':
    test_fat_tip_gap_splits_only_in_strict()
    test_soft_joint_stays_one_component()
    test_pad_proximity_phantom_splits_in_strict()
    test_exact_pad_rules_survive_strict()
    test_net_copper_fragments_census()
    print("ALL PASS")

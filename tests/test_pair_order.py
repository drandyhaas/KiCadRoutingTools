#!/usr/bin/env python3
"""pair_order: pin-order inversions are a LOWER BOUND the score can't see.

Test-board run 5 (wk5 journal [3]): U3 at rot 0 met U1's QSPI pin stack in an
order needing many pairwise swaps -- the router shipped 4/7 across fifteen
ordering experiments -- and rotating U3 180 degrees made the same nets route
7/7 first try. The placement objective was indifferent; the pin-order count
was the signal. These tests pin the metric's arithmetic (identity 0, reversal
n(n-1)/2, LIS duality), the delta path used by pose ranking, and the REAL U3
geometry (pad locals hardcoded from the test-board), and assert pose_score
carries the number as a component.
"""
import math
import os
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for p in (ROOT, _TESTS):
    if p not in sys.path:
        sys.path.insert(0, p)

        sys.path.insert(0, os.path.join(p, 'py_placer'))  # placement split
        sys.path.insert(0, os.path.join(p, 'py_router'))  # placement split
        sys.path.insert(0, os.path.join(p, 'py_tools'))  # placement split
from kicad_parser import local_to_global  # noqa: E402
from placement.pair_order import (pair_metrics, pair_inversions,  # noqa: E402
                                  ref_inversions, inversions_delta)

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


class FakePart:
    """The slice of quench._Part the metric consumes, with the SAME pad
    transform (kicad_parser.local_to_global)."""

    def __init__(self, x, y, rot, pads_local):
        self.x, self.y, self.rot = x, y, rot
        self._pads = pads_local
        self.nets = sorted({n for _, _, n in pads_local})

    def pad_globals(self, x=None, y=None, rot=None):
        x = self.x if x is None else x
        y = self.y if y is None else y
        rot = self.rot if rot is None else rot
        return [(*local_to_global(x, y, rot, lx, ly), n)
                for lx, ly, n in self._pads]


class FakeState:
    def __init__(self, parts):
        self.parts = parts
        by = {}
        for ref, p in parts.items():
            for n in p.nets:
                by.setdefault(n, []).append(ref)
        self.net_refs = {n: sorted(r) for n, r in by.items()}


def _facing_pair(order_b):
    """Two vertical pin stacks 10 mm apart; A carries nets 1..n top-down,
    B carries `order_b` top-down."""
    n = len(order_b)
    a = FakePart(0.0, 0.0, 0.0, [(0.0, i * 1.0, i + 1) for i in range(n)])
    b = FakePart(10.0, 0.0, 0.0, [(0.0, i * 1.0, nid)
                                  for i, nid in enumerate(order_b)])
    return FakeState({'A': a, 'B': b})


def test_identity_is_planar():
    st = _facing_pair([1, 2, 3, 4, 5])
    m = pair_metrics(st, 'A', 'B')
    # #896/#891: `ties` is an additive key -- how many pads project to the
    # same point on the channel axis, where the sort falls back to the net
    # id and INVENTS an order. Asserted here rather than filtered out,
    # because a synthetic fixture with 0 ties is what makes the inversion
    # count above mean anything at all.
    assert m == {'inversions': 0, 'lis': 5, 'nets': 5, 'ties': 0}, m
    print("  PASS: matched order -> 0 inversions, all 5 nets planar, "
          "no ties")


def test_full_reversal_is_maximal():
    st = _facing_pair([5, 4, 3, 2, 1])
    m = pair_metrics(st, 'A', 'B')
    assert m == {'inversions': 10, 'lis': 1, 'nets': 5,
                 'ties': 0}, m               # n(n-1)/2, LIS 1, no ties
    print("  PASS: reversed order -> n(n-1)/2 inversions, LIS 1")


def test_rotating_the_partner_heals_the_order():
    """Rotating B 180 degrees flips its stack: the reversal becomes identity,
    and the delta path (used by pose ranking, no state mutation) agrees with
    recomputing from scratch."""
    st = _facing_pair([5, 4, 3, 2, 1])
    d = inversions_delta(st, 'B', 10.0, 4.0, 180.0)
    assert d['before'] == 10 and d['after'] == 0 and d['delta'] == -10, d
    assert d['pairs'][('A', 'B')]['inversions'] == 0
    # nothing was mutated: the live metric is still the reversal
    assert ref_inversions(st, 'B') == 10
    print("  PASS: rot-180 delta -10, state untouched")


def test_pair_inversions_enumerates_shared_pairs_only():
    st = _facing_pair([2, 1])
    st.parts['C'] = FakePart(20.0, 0.0, 0.0, [(0.0, 0.0, 99)])
    st.net_refs[99] = ['C']
    out = pair_inversions(st)
    assert set(out) == {('A', 'B')}, out  # C shares nothing; 1 net < 2 anyway
    assert out[('A', 'B')]['inversions'] == 1
    print("  PASS: only pairs sharing >=2 scoring nets are reported")


# --- the run-5 case, with the REAL pad geometry ------------------------------
# Pad locals read from test-board layout/layout.kicad_pcb (run 5's promoted
# board). Nets: SD3=1 SCLK=2 SD0=3 SD2=4 SD1=5. U1 (RP2350, rot 90) presents
# the stack [SD3 SCLK SD0 SD2 SD1] north->south; U3 is the SOIC-8 flash with
# the split pinout (SD1/SD2 west column, SD0/SCLK/SD3 east column).
U1_POSE = (125.5, 70.5, 90.0)
U1_PADS = [(-0.8, -3.4375, 1), (-1.2, -3.4375, 2), (-1.6, -3.4375, 3),
           (-2.0, -3.4375, 4), (-2.4, -3.4375, 5)]
U3_XY = (113.0, 76.5)
U3_PADS = [(-3.5875, -0.635, 5), (-3.5875, 0.635, 4), (3.5875, 1.905, 3),
           (3.5875, 0.635, 2), (3.5875, -0.635, 1)]


def _qspi_state(u3_rot):
    return FakeState({'U1': FakePart(*U1_POSE, U1_PADS),
                      'U3': FakePart(*U3_XY, u3_rot, U3_PADS)})


def test_u3_rotation_the_metric_sees_what_run5_measured():
    """rot 0 must score strictly worse than rot 180 (the pose run 5 shipped).
    The exact values are deterministic: 7 vs 3 on the direct U1<->U3 pair (the
    journal's hand count of 9 covered the whole escape FIELD, incl. the CS
    chain through R11 which is not a direct shared net)."""
    at = {rot: pair_metrics(_qspi_state(rot), 'U1', 'U3')['inversions']
          for rot in (0.0, 180.0)}
    assert at[0.0] == 7 and at[180.0] == 3, at
    print(f"  PASS: U3 rot0={at[0.0]} > rot180={at[180.0]} "
          f"(the flip run 5 found by hand)")


def test_pose_score_carries_the_component():
    if not os.path.isfile(BOARD):
        print("  SKIP: fixture missing")
        return
    from kicad_parser import parse_kicad_pcb
    import pose_score
    pcb = parse_kicad_pcb(BOARD)
    poses = pose_score.rank_poses(pcb, BOARD, 'C1', radius=0.5, step=0.5,
                                  limit=3)
    assert poses, "no legal poses for C1"
    for p in poses:
        assert 'inversions' in p['components'], p['components']
        assert 'inversions' in p['component_delta']
    print("  PASS: rank_poses carries inversions in components + delta")


if __name__ == '__main__':
    for k, v in sorted(globals().items()):
        if k.startswith('test_'):
            print(f"--- {k}")
            v()
    print("ALL PASS")

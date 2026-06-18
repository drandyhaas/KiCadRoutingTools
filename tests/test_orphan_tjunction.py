#!/usr/bin/env python3
"""check_orphan_stubs must not count connected T-junction tap landings (#84).

A degree-1 endpoint that lands on the interior copper of another same-net segment
is a T-junction -- the traces overlap, so it is electrically connected, not a
dead end. Only genuine dead ends (and near-misses that do not overlap) stay.

    python3 tests/test_orphan_tjunction.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from check_orphan_stubs import _endpoint_connected


def run():
    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)

    trunk = {'start': (0, 0), 'end': (10, 0), 'width': 0.2}
    tap = {'start': (5, 0), 'end': (5, 3), 'width': 0.13}

    # Tap end lands on trunk body -> connected (not an orphan).
    check("T-junction landing is connected", _endpoint_connected((5, 0), [trunk, tap]) is True)
    # The tap's far end lands on nothing -> genuine orphan.
    check("dead tip stays an orphan", _endpoint_connected((5, 3), [trunk, tap]) is False)

    # Landing near a trunk endpoint still connects (whole-segment proximity).
    tap2 = {'start': (0.3, 0), 'end': (0.3, 2), 'width': 0.13}
    check("near-endpoint landing connects",
          _endpoint_connected((0.3, 0), [trunk, tap2]) is True)

    # A stub passing 0.3 mm clear of the trunk does NOT connect (no over-clearing).
    miss = {'start': (5, 0.3), 'end': (5, 3), 'width': 0.13}
    check("0.3mm near-miss is not cleared", _endpoint_connected((5, 0.3), [trunk, miss]) is False)

    # A via's copper connects an endpoint within its radius even when >0.15mm
    # from the via centre (0.5mm via -> 0.25mm copper radius).
    chk = [{'start': (5, 0), 'end': (5, 3), 'width': 0.13}]
    check("endpoint inside via copper (0.2mm from 0.5mm via) connects",
          _endpoint_connected((5, 3.2), chk, vias=[(5, 3, 0.5)]) is True)
    check("endpoint outside via copper (0.4mm from 0.5mm via) is orphan",
          _endpoint_connected((5, 3.4), chk, vias=[(5, 3, 0.5)]) is False)

    # An endpoint inside a pad's copper extent connects.
    check("endpoint inside pad copper connects",
          _endpoint_connected((5, 3.3), chk, layer_pads=[(5, 3, 0.8)]) is True)

    print("=" * 60)
    if fails:
        for f in fails:
            print(f"  FAIL  {f}")
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("  PASS  T-junction, near-coincident, near-miss, via-radius, and")
    print("        pad-extent connection detection (7 cases)")
    print("\n7/7 checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(run())

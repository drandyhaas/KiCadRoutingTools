#!/usr/bin/env python3
"""
Regression test for issue #165 (tier 2, root-cause 1): differential-pair
terminal pairing.

`get_diff_pair_terminals` groups a pair's pads into (p_pad, n_pad) terminals.
The old greedy-by-distance matcher is only *locally* optimal: a closest-first
pick can strand the leftover pads into a far pairing. On a USB-C connector whose
redundant DP/DN pads interleave (J1 y-order B7,A6,A7,B6), greedy took
{A6,A7}(0.5mm) first and was then forced into {B6,B7}(1.5mm); that wide terminal
drives a long diagonal connector that grazes the partner pad (the tigard
/USB_DP+/USB_DN short). Minimum-TOTAL-cost matching yields {A6,B7}+{B6,A7}
(0.5+0.5), each terminal tight so its connector launches straight out.

Run:
    python3 tests/test_diff_terminal_pairing.py
"""

import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

import diff_pair_multipoint as m
from kicad_parser import Pad, PCBData

P_NET, N_NET = 10, 20


def _pad(ref, num, net, x, y):
    name = '/USB_DP' if net == P_NET else '/USB_DN'
    return Pad(ref, num, x, y, 0, 0, 0.3, 1.45, 'roundrect', ['F.Cu'],
               net, name, 0, None, 0.0, None, 0.25, 0.0, None)


def _sep(t):
    pp, nn = t
    return ((pp.global_x - nn.global_x) ** 2 + (pp.global_y - nn.global_y) ** 2) ** 0.5


def _pairset(terminals):
    return {frozenset((pp.pad_number, nn.pad_number)) for pp, nn in terminals}


def main():
    # USB-C J1: redundant DP (A6, B6) / DN (A7, B7) interleaved in y, plus the
    # chip's single DP/DN pair at U3.
    p_pads = [_pad('J1', 'A6', P_NET, 0.0, 60.15),
              _pad('J1', 'B6', P_NET, 0.0, 61.15),
              _pad('U3', '8',  P_NET, 0.0, 40.00)]
    n_pads = [_pad('J1', 'A7', N_NET, 0.0, 60.65),
              _pad('J1', 'B7', N_NET, 0.0, 59.65),
              _pad('U3', '7',  N_NET, 0.0, 40.50)]
    pcb = PCBData(footprints={}, nets={}, segments=[], vias=[],
                  board_info=None, pads_by_net={P_NET: p_pads, N_NET: n_pads})

    expected = {frozenset(('A6', 'B7')), frozenset(('B6', 'A7')),
                frozenset(('8', '7'))}
    bad = frozenset(('B6', 'B7'))  # the 1.5mm greedy terminal that grazes

    results = []

    def check(name, ok, detail=""):
        results.append((name, ok))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}{('  ' + detail) if detail else ''}")

    # Exercise both the scipy and the no-scipy brute-force paths.
    for path, use_scipy in (("scipy", True), ("brute-force", False)):
        if use_scipy and not m.HAS_SCIPY:
            print(f"  [skip] {path} path (scipy not installed)")
            continue
        saved = m.HAS_SCIPY
        m.HAS_SCIPY = use_scipy
        try:
            terms = m.get_diff_pair_terminals(pcb, P_NET, N_NET)
        finally:
            m.HAS_SCIPY = saved
        got = _pairset(terms)
        check(f"{path}: optimal nearest-N pairing", got == expected, str(sorted(map(sorted, got))))
        check(f"{path}: no wide {{B6,B7}} terminal", bad not in got)
        check(f"{path}: every terminal tight (<=0.6mm)", all(_sep(t) <= 0.6 for t in terms),
              f"max sep={max(_sep(t) for t in terms):.2f}mm")

    passed = sum(1 for _, ok in results if ok)
    total = len(results)
    print("\n" + "=" * 60)
    print(f"  {passed}/{total} checks passed")
    print("=" * 60)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())

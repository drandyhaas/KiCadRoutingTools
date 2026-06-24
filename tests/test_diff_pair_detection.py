#!/usr/bin/env python3
"""Unit coverage for extract_diff_pair_base() net-name parsing (net_queries).

Locks the supported diff-pair naming conventions AND the issue #192 fix: a
literal `_t_`/`_c_` *infix* (a section/channel letter like TARGET_C_SENSE) must
not be misread as DDR true/complement when the name carries a real +/- or _P/_N
polarity suffix.

    python3 tests/test_diff_pair_detection.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from net_queries import extract_diff_pair_base


# (p_name, n_name) pairs that MUST resolve to the same base + style and opposite
# polarity. Covers every convention the function advertises, plus the #192 cases.
PAIRS = [
    # DDR true/complement
    ("CK_t", "CK_c"),
    ("DQS0_t_A", "DQS0_c_A"),
    ("CK_T0", "CK_C0"),
    # LVDS _P/_N
    ("LVDS_TX_P", "LVDS_TX_N"),
    ("D_P", "D_N"),
    # SerDes / PCIe P/N (no underscore)
    ("TXP", "TXN"),
    ("SSRXP", "SSRXN"),
    # indexed P/N
    ("FE_CLK_P0", "FE_CLK_N0"),
    # USB
    ("USB_DP", "USB_DM"),
    ("USB_DP", "USB_DN"),
    ("DPLUS", "DMINUS"),
    # +/- suffix
    ("AUX_SENSE+", "AUX_SENSE-"),
    # issue #192: _C_/_T_ infix + polarity suffix must pair (not DDR-misread)
    ("TARGET_C_SENSE+", "TARGET_C_SENSE-"),
    ("FOO_T_BAR+", "FOO_T_BAR-"),
    ("ADC_C_OUT+", "ADC_C_OUT-"),
    ("TARGET_C_SENSE_P", "TARGET_C_SENSE_N"),
]

# Names that are NOT diff-pair halves -> None
NON_PAIRS = ["GND", "VCC", "CLK", "Net-(R1-Pad1)", "+3V3", ""]


def run():
    fails = []
    for p_name, n_name in PAIRS:
        p = extract_diff_pair_base(p_name)
        n = extract_diff_pair_base(n_name)
        if p is None or n is None:
            fails.append(f"{p_name}/{n_name}: not detected (p={p}, n={n})")
            continue
        (bp, posp, sp), (bn, posn, sn) = p, n
        if bp != bn:
            fails.append(f"{p_name}/{n_name}: base mismatch {bp!r} != {bn!r}")
        if sp != sn:
            fails.append(f"{p_name}/{n_name}: style mismatch {sp!r} != {sn!r}")
        if not (posp and not posn):
            fails.append(f"{p_name}/{n_name}: polarity wrong (p_pos={posp}, n_pos={posn})")

    for nm in NON_PAIRS:
        r = extract_diff_pair_base(nm)
        if r is not None:
            fails.append(f"{nm!r}: expected None, got {r}")

    if fails:
        print(f"FAIL ({len(fails)}):")
        for f in fails:
            print("  -", f)
        return False
    print(f"PASS: {len(PAIRS)} pairs + {len(NON_PAIRS)} non-pairs")
    return True


if __name__ == "__main__":
    sys.exit(0 if run() else 1)

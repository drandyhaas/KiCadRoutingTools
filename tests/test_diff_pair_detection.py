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
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # #522
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # #522

from net_queries import extract_diff_pair_base, near_miss_diff_pairs, near_miss_text


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
    # issue #1456: -P/-N and .P/.N
    ("/TOP/digital interface/ADC_MISO-P", "/TOP/digital interface/ADC_MISO-N"),
    ("CLK.P", "CLK.N"),
    ("Net-(U1-CLK-P)", "Net-(U1-CLK-N)"),
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
    # issue #290: +/- with a suffix after the sign (dilemma split-keyboard USB)
    ("/D+_L", "/D-_L"),
    ("USB_D+_R", "USB_D-_R"),
]

# Names that are NOT diff-pair halves -> None
NON_PAIRS = ["GND", "VCC", "CLK", "Net-(R1-Pad1)", "+3V3", "",
             # ordinary hyphenation is not polarity (issue #290 stays narrow:
             # only an underscore-led suffix after the sign qualifies)
             "3V3-MCU", "+3V3_AUX",
             # #1456: the ref-pin '-' of an auto-name is not a polarity separator
             "Net-(U1-P)", "Net-(U1-N)"]

# Names that read as one pair but that route_diff or KiCad's MatchDpSuffix
# misses: (p, n, route_diff pairs them, KiCad pairs them, the rename offered).
NEAR_MISSES = [
    ("/mcu/USB_DP_MCU", "/mcu/USB_DM_MCU", False, False,
     {"/mcu/USB_DP_MCU": "/mcu/USB_MCU_DP", "/mcu/USB_DM_MCU": "/mcu/USB_MCU_DN"}),
    ("USB_D+_MCU", "USB_D-_MCU", True, False,
     {"USB_D+_MCU": "USB_MCU_D+", "USB_D-_MCU": "USB_MCU_D-"}),
    ("LVDS_P_CLK", "LVDS_N_CLK", False, False,
     {"LVDS_P_CLK": "LVDS_CLK_P", "LVDS_N_CLK": "LVDS_CLK_N"}),
    ("TX_P_0", "TX_N_0", False, True, {"TX_P_0": "TX_0_P", "TX_N_0": "TX_0_N"}),
    ("clk_p", "clk_n", False, False, {"clk_p": "clk_P", "clk_n": "clk_N"}),
    ("USB_DP", "USB_DM", True, False, {"USB_DM": "USB_DN"}),
]


def run_near_misses():
    fails = []
    for p, n, route_diff, kicad, rename in NEAR_MISSES:
        got = near_miss_diff_pairs(["GND", p, n])
        want = [{'p': p, 'n': n, 'route_diff': route_diff, 'kicad': kicad, 'rename': rename}]
        if got != want:
            fails.append(f"{p}/{n}: {got} != {want}")
    # A rename that would take an existing name is not offered.
    got = near_miss_diff_pairs(["A_P_X", "A_N_X", "A_X_P"])
    if [m['rename'] for m in got] != [{}] or 'no rename found' not in near_miss_text(got[0]):
        fails.append(f"A_P_X/A_N_X beside A_X_P: {got}")
    # A pair both rules already pair, a lone half and an auto-name are no near miss.
    for p_name, n_name in PAIRS:
        for m in near_miss_diff_pairs([p_name, n_name]):
            if not m['route_diff']:
                fails.append(f"{p_name}/{n_name}: route_diff pairs it, read as a miss {m}")
    for names in (["/power/CHG_N"], ["Net-(U1-DP)", "Net-(U1-DM)"]):
        if near_miss_diff_pairs(names):
            fails.append(f"{names}: {near_miss_diff_pairs(names)}")
    if fails:
        print(f"FAIL near misses ({len(fails)}):")
        for f in fails:
            print("  -", f)
        return False
    print(f"PASS: {len(NEAR_MISSES)} near-miss pairs")
    return True


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
    sys.exit(0 if all([run(), run_near_misses()]) else 1)

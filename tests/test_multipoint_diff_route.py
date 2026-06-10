#!/usr/bin/env python3
"""
Pass/fail test for bare-pad and multi-point differential-pair routing.

Board: `kicad_files/lvds_converter_dualclk.kicad_pcb` - a simple 2-layer LVDS
converter where the diff pairs terminate directly on bare SMD/PTH pads (no
fanout stubs) at MULTIPLE pad-pair terminals:

- /DATA+ / /DATA-  : 3 terminals (J1 connector, IC4 pins 10/9, R4 termination)
- /CLK+  / /CLK-   : 4 terminals (J1, R3 termination, IC4 pins 2/1 AND 7/6)

This exercises, end to end:
- suffix-style-aware pair detection (/CLK+ pairs with /CLK-, not /CLK_N)
- synthesized escape directions for bare-pad endpoints
- multi-point chain routing (legs passing "through" shared terminals on
  opposite sides, alternative chain orderings on failure)
- per-leg polarity resolution: with fixing on, pad-swap and connector-flip
  candidates compete by routed length (swaps only at chain-fresh terminals);
  with --no-fix-polarity, flips only - pad swaps must never occur

Each scenario asserts the pair(s) routed, EVERY terminal is connected
(multi-point connectivity), polarity swaps match expectations, and the result
is DRC-clean scoped to the pair nets. Geometry matches the board's netclass
(0.2mm track / 0.25mm pair gap / 0.2mm clearance).

Also covers the issue #56 regression on
`kicad_files/lvds_converter_dualclk_gnd.kicad_pcb` - the same board with a
GND plane and stitching vias, one of which blocks the channel between R3 and
IC4. Wrap-around legs must never cross earlier legs (shorts): with polarity
fixing on the pair routes cleanly around the via; with --no-fix-polarity the
CLK pair must FAIL HONESTLY (all chain orderings rejected, no segments
committed) instead of shorting.

Run:
    python3 tests/test_multipoint_diff_route.py
    python3 tests/test_multipoint_diff_route.py -v   # keep routing output
"""

import argparse
import json
import os
import re
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

BOARD = os.path.join(ROOT_DIR, "kicad_files", "lvds_converter_dualclk.kicad_pcb")
GND_BOARD = os.path.join(ROOT_DIR, "kicad_files", "lvds_converter_dualclk_gnd.kicad_pcb")
# Match the board's netclass: 0.2mm track, 0.25mm pair gap, 0.2mm clearance
GEOM = ["--track-width", "0.2", "--diff-pair-gap", "0.25", "--clearance", "0.2",
        "--layers", "F.Cu", "B.Cu"]
CLEARANCE = "0.2"

# (name, sequential routing runs, expected legs markers across all runs,
#  extra CLI args, expected polarity-swapped pairs)
# Each run is (nets, expected routed pairs); later runs route on the previous
# run's output. CLK (4 terminals) routes before DATA: the more-constrained
# pair gets first pick of the space around the shared J1 connector.
# expected swaps: None = don't check; [] = swaps must NOT occur; [names] =
# exactly these pairs must have been polarity-swapped.
#
# With polarity fixing on (CLI default), legs compare pad-swap vs
# connector-flip candidates by length: DATA's flip wins, CLK's pad swap wins
# (it avoids a long wrap through IC4). With --no-fix-polarity, swaps must
# never occur - only the opposite-side connector resolution.
SCENARIOS = [
    ("DATA multi-point (fix-polarity on: flip beats swap)",
     [(["/DATA+", "/DATA-"], 1)], {"3 terminals, 2 legs": 1}, [], []),
    ("CLK multi-point (fix-polarity on: pad swap beats flips)",
     [(["/CLK+", "/CLK-"], 1)], {"4 terminals, 3 legs": 1}, [], ["/CLK"]),
    ("DATA multi-point (--no-fix-polarity)",
     [(["/DATA+", "/DATA-"], 1)], {"3 terminals, 2 legs": 1},
     ["--no-fix-polarity"], []),
    ("CLK multi-point (--no-fix-polarity)",
     [(["/CLK+", "/CLK-"], 1)], {"4 terminals, 3 legs": 1},
     ["--no-fix-polarity"], []),
    ("CLK then DATA (sequential runs)",
     [(["/CLK+", "/CLK-"], 1), (["/DATA+", "/DATA-"], 1)],
     {"4 terminals, 3 legs": 1, "3 terminals, 2 legs": 1}, [], None),
]


def route(board, nets, out, extra_args=()):
    """Route the given nets on *board*; return (summary_dict_or_None, output_text)."""
    cmd = [sys.executable, "route_diff.py", board, out, "--nets"] + nets + GEOM + list(extra_args)
    r = subprocess.run(cmd, cwd=ROOT_DIR, capture_output=True, text=True)
    txt = r.stdout + r.stderr
    m = re.search(r"JSON_SUMMARY: (\{.*\})", txt)
    return (json.loads(m.group(1)) if m else None), txt


def is_connected(board, nets):
    """True if check_connected reports ALL the pair nets fully connected
    (for multi-point pairs this requires every terminal to be reached)."""
    cmd = [sys.executable, "check_connected.py", board, "--nets"] + nets
    r = subprocess.run(cmd, cwd=ROOT_DIR, capture_output=True, text=True)
    return "ALL NETS FULLY CONNECTED" in (r.stdout + r.stderr)


def drc_clean(board, nets):
    """True if the routed nets have no DRC violations (scoped to themselves)."""
    cmd = [sys.executable, "check_drc.py", board, "--clearance", CLEARANCE,
           "--nets"] + nets
    r = subprocess.run(cmd, cwd=ROOT_DIR, capture_output=True, text=True)
    return "NO DRC VIOLATIONS" in (r.stdout + r.stderr)


def scenario(name, runs, expect_legs, extra_args, expect_swapped, verbose):
    log = []
    outs = []
    try:
        board = BOARD
        all_nets = []
        routed_ok = True
        swaps_ok = True
        all_swapped = []
        all_txt = ""
        for nets, expect_routed in runs:
            fd, out = tempfile.mkstemp(suffix=".kicad_pcb", prefix="mpdiff_")
            os.close(fd)
            outs.append(out)
            summary, txt = route(board, nets, out, extra_args)
            if verbose:
                print(txt)
            if summary is None or not os.path.exists(out):
                return name, False, ["route_diff produced no summary/output"]
            all_txt += txt
            all_nets += nets
            routed_ok = routed_ok and (summary.get("successful") == expect_routed
                                       and summary.get("failed") == 0)
            log.append(f"routed {','.join(nets)}: {summary.get('successful')}/{expect_routed} "
                       f"failed={summary.get('failed')}")
            all_swapped += summary.get("polarity_swapped_pairs", [])
            # Target swaps never apply to multi-point pairs
            swaps_ok = swaps_ok and summary.get("target_swaps") == []
            board = out  # next run routes on this output

        if expect_swapped is not None:
            swaps_ok = swaps_ok and (sorted(all_swapped) == sorted(expect_swapped))
        log.append(f"polarity_swapped={all_swapped} (expected "
                   f"{'any' if expect_swapped is None else expect_swapped}) ok={swaps_ok}")

        legs_ok = True
        for marker, count in expect_legs.items():
            found = all_txt.count(f"Multi-point pair: {marker}")
            if found != count:
                legs_ok = False
                log.append(f"expected {count}x 'Multi-point pair: {marker}', got {found}")
        if legs_ok:
            log.append("multi-point leg plan as expected")

        final = outs[-1]
        conn = is_connected(final, all_nets)
        clean = drc_clean(final, all_nets)
        log.append(f"connected(all terminals)={conn}  drc_clean(pair-scoped)={clean}")

        return name, (routed_ok and swaps_ok and legs_ok and conn and clean), log
    finally:
        for out in outs:
            if os.path.exists(out):
                os.remove(out)


def _pair_segments_and_crossings(board_path, net_names):
    """(segment count, proper same-layer crossing count) for the given nets."""
    from kicad_parser import parse_kicad_pcb

    pcb = parse_kicad_pcb(board_path)
    net_ids = {n.net_id for n in pcb.nets.values() if n.name in net_names}
    segs = [s for s in pcb.segments if s.net_id in net_ids]

    def crosses(a, b, eps=1e-6):
        for (x, y) in ((a.start_x, a.start_y), (a.end_x, a.end_y)):
            for (u, v) in ((b.start_x, b.start_y), (b.end_x, b.end_y)):
                if abs(x - u) < eps and abs(y - v) < eps:
                    return False
        def d(px, py, qx, qy, rx, ry):
            return (qx - px) * (ry - py) - (qy - py) * (rx - px)
        d1 = d(b.start_x, b.start_y, b.end_x, b.end_y, a.start_x, a.start_y)
        d2 = d(b.start_x, b.start_y, b.end_x, b.end_y, a.end_x, a.end_y)
        d3 = d(a.start_x, a.start_y, a.end_x, a.end_y, b.start_x, b.start_y)
        d4 = d(a.start_x, a.start_y, a.end_x, a.end_y, b.end_x, b.end_y)
        return ((d1 > eps) != (d2 > eps)) and ((d3 > eps) != (d4 > eps)) and \
            min(abs(d1), abs(d2), abs(d3), abs(d4)) > eps

    n_cross = sum(1 for i in range(len(segs)) for j in range(i + 1, len(segs))
                  if segs[i].layer == segs[j].layer and crosses(segs[i], segs[j]))
    return len(segs), n_cross


def gnd_obstacle_scenario_fix_on(verbose):
    """Issue #56 board, polarity fixing ON: both pairs route around the
    blocking stitching via with zero crossings."""
    name = "GND-via obstacle (fix-polarity on: routes clean, no crossings)"
    log = []
    fd, out = tempfile.mkstemp(suffix=".kicad_pcb", prefix="mpdiff56_")
    os.close(fd)
    try:
        nets = ["/CLK+", "/CLK-", "/DATA+", "/DATA-"]
        summary, txt = route(GND_BOARD, nets, out)
        if verbose:
            print(txt)
        if summary is None:
            return name, False, ["route_diff produced no summary"]
        routed_ok = summary.get("successful") == 2 and summary.get("failed") == 0
        log.append(f"routed: {summary.get('successful')}/2 failed={summary.get('failed')}")
        n_segs, n_cross = _pair_segments_and_crossings(out, nets)
        log.append(f"pair segments={n_segs} crossings={n_cross}")
        conn = is_connected(out, nets)
        clean = drc_clean(out, nets)
        log.append(f"connected={conn}  drc_clean(pair-scoped)={clean}")
        return name, (routed_ok and n_cross == 0 and conn and clean), log
    finally:
        if os.path.exists(out):
            os.remove(out)


def gnd_obstacle_scenario_no_fix(verbose):
    """Issue #56 board, --no-fix-polarity: every CLK chain ordering needs a
    crossing, so the pair must FAIL HONESTLY - crossing legs rejected, all
    legs ripped, nothing committed. DATA (unaffected by the via) routes."""
    name = "GND-via obstacle (--no-fix-polarity: CLK fails honestly, no shorts)"
    log = []
    fd, out = tempfile.mkstemp(suffix=".kicad_pcb", prefix="mpdiff56_")
    os.close(fd)
    try:
        nets = ["/CLK+", "/CLK-", "/DATA+", "/DATA-"]
        summary, txt = route(GND_BOARD, nets, out, ["--no-fix-polarity"])
        if verbose:
            print(txt)
        if summary is None:
            return name, False, ["route_diff produced no summary"]
        outcome_ok = (summary.get("failed_diff_pairs") == ["/CLK"]
                      and summary.get("routed_diff_pairs") == ["/DATA"])
        log.append(f"routed={summary.get('routed_diff_pairs')} "
                   f"failed={summary.get('failed_diff_pairs')}")
        rejected = ("crosses an earlier leg" in txt) or ("crosses itself" in txt)
        log.append(f"crossing legs rejected in output: {rejected}")
        # The honest failure must leave NOTHING behind: zero CLK segments
        # (every attempt's legs ripped), so no shorts can have been committed
        clk_segs, clk_cross = _pair_segments_and_crossings(out, ["/CLK+", "/CLK-"])
        log.append(f"CLK segments left on board={clk_segs} crossings={clk_cross}")
        data_conn = is_connected(out, ["/DATA+", "/DATA-"])
        data_clean = drc_clean(out, ["/DATA+", "/DATA-"])
        log.append(f"DATA connected={data_conn}  drc_clean={data_clean}")
        return name, (outcome_ok and rejected and clk_segs == 0
                      and data_conn and data_clean), log
    finally:
        if os.path.exists(out):
            os.remove(out)


def main():
    parser = argparse.ArgumentParser(
        description="Bare-pad / multi-point differential-pair routing test")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Show routing output")
    args = parser.parse_args()

    if not os.path.exists(BOARD):
        print(f"ERROR: board not found: {BOARD}")
        return 2

    print("=" * 70)
    print("Bare-pad / multi-point differential-pair routing test")
    print("=" * 70)

    results = []
    for name, runs, expect_legs, extra_args, expect_swapped in SCENARIOS:
        sname, passed, log = scenario(name, runs, expect_legs, extra_args, expect_swapped, args.verbose)
        print(f"\n[{'PASS' if passed else 'FAIL'}] {sname}")
        for line in log:
            print(f"        {line}")
        results.append((sname, passed))

    # Issue #56 regression (board with a GND stitching via blocking the
    # R3 -> IC4 channel)
    if os.path.exists(GND_BOARD):
        for fn in (gnd_obstacle_scenario_fix_on, gnd_obstacle_scenario_no_fix):
            sname, passed, log = fn(args.verbose)
            print(f"\n[{'PASS' if passed else 'FAIL'}] {sname}")
            for line in log:
                print(f"        {line}")
            results.append((sname, passed))
    else:
        print(f"\n[FAIL] issue #56 regression board missing: {GND_BOARD}")
        results.append(("issue #56 regression board missing", False))

    print("\n" + "=" * 70)
    n_pass = sum(1 for _, p in results if p)
    for sname, passed in results:
        print(f"  {'PASS' if passed else 'FAIL'}  {sname}")
    print(f"\n{n_pass}/{len(results)} scenarios passed")
    print("=" * 70)
    return 0 if n_pass == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())

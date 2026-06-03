#!/usr/bin/env python3
"""
Pass/fail tests for KiCad keep-out rule areas (PR #25, feat/keepout-obstacles).

This is the *native KiCad* keepout: a `(zone ... (keepout (tracks not_allowed)
(vias not_allowed) ...) (polygon (pts ...)))` drawn on copper layers. Unlike the
user-drawn User-layer keepout (`test_keepout.py`, issue #27), it has no enable
flag — it is honored automatically whenever the board contains one, and it blocks
only the disallowed item (tracks and/or vias) on the keepout's listed layers.

Self-contained: builds temporary boards from kicad_files/flat_hierarchy.kicad_pcb
by inserting a keepout zone, routes with route.py, and checks each scenario. Each
prints PASS/FAIL plus log detail; exits non-zero if any scenario fails. Run with
KiCad's python (needs the Rust router).

Scenarios
---------
R1  Avoid — a keepout straddling a net's straight path forces a detour; the net
    still connects, is DRC-clean, and no routed cell lies inside the polygon.
R2  Multi-net — two nets routed with the keepout present both connect and neither
    occupies a cell inside the polygon.
R3  No-op gating — a keepout that *allows* both tracks and vias is a no-op: the
    route cuts straight through, exactly as with no keepout (guards the
    tracks/vias-allowed gating, including the vias-not-allowed parse bug PR #25
    fixed — an allowed/allowed area must never block).
"""

import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, TESTS_DIR)

# Reuse the user-drawn keepout test harness (same board, helpers, geometry).
from test_keepout import (BASE_BOARD, box, run_route, is_connected,  # noqa: E402
                          drc_counts, intrusions, D8_BOX, D9_BOX)

_Z = 0


def _keepout_zone(points, layers=("F.Cu", "B.Cu"),
                  tracks_allowed=False, vias_allowed=False):
    """A KiCad keep-out rule-area zone over the given polygon (top-level, tab-indented)."""
    global _Z
    _Z += 1
    layer_str = " ".join(f'"{ln}"' for ln in layers)
    pts = " ".join(f"(xy {x} {y})" for x, y in points)
    tr = "allowed" if tracks_allowed else "not_allowed"
    vi = "allowed" if vias_allowed else "not_allowed"
    return (
        f"\t(zone\n"
        f'\t\t(net 0 "")\n'
        f"\t\t(layers {layer_str})\n"
        f'\t\t(uuid "0000{_Z:04d}-0000-0000-0000-000000000000")\n'
        f"\t\t(hatch edge 0.5)\n"
        f"\t\t(keepout (tracks {tr}) (vias {vi}) (pads allowed) "
        f"(copperpour not_allowed) (footprints allowed))\n"
        f"\t\t(polygon (pts {pts}))\n"
        f"\t)\n"
    )


def make_board_with_zones(zones_text):
    """Write a temp board = base board + the given keepout zone S-expressions."""
    text = open(BASE_BOARD).read()
    idx = text.rstrip().rfind(")")
    fd, path = tempfile.mkstemp(suffix=".kicad_pcb", prefix="ruleko_test_")
    os.close(fd)
    with open(path, "w") as f:
        f.write(text[:idx] + "\n" + zones_text + text[idx:])
    return path


# --------------------------------------------------------------------------
# Scenarios
# --------------------------------------------------------------------------

def scenario_avoid(verbose):
    """R1: keepout straddling the path forces a detour; zero cells inside, clean."""
    log = []
    polys = [D8_BOX]
    board = make_board_with_zones(_keepout_zone(D8_BOX))
    out = board.replace(".kicad_pcb", "_out.kicad_pcb")
    ok, _ = run_route(board, out, ["Net-(D8-A)"], keepout=False, verbose=verbose)
    if not ok or not os.path.exists(out):
        return "R1 avoid", False, ["route.py failed"]
    connected = is_connected(out, ["Net-(D8-A)"])
    inside = intrusions(out, "Net-(D8-A)", polys)
    real, selfx = drc_counts(out)
    log.append(f"connected={connected}  real_drc_violations={real}  same_net_crossings={selfx}")
    log.append(f"routed cells inside the rule area={inside} (must be 0)")
    passed = connected and inside == 0 and real == 0
    return "R1 avoid (route detours around the rule area, no clearance errors)", passed, log


def scenario_multi_net(verbose):
    """R2: two nets routed with a rule area present both connect and avoid it."""
    log = []
    polys = [D8_BOX, D9_BOX]
    zones = _keepout_zone(D8_BOX) + _keepout_zone(D9_BOX)
    board = make_board_with_zones(zones)
    out = board.replace(".kicad_pcb", "_out.kicad_pcb")
    nets = ["Net-(D8-A)", "Net-(D9-A)"]
    ok, _ = run_route(board, out, nets, keepout=False, verbose=verbose)
    if not ok or not os.path.exists(out):
        return "R2 multi-net", False, ["route.py failed"]
    connected = is_connected(out, nets)
    in8 = intrusions(out, "Net-(D8-A)", polys)
    in9 = intrusions(out, "Net-(D9-A)", polys)
    real, selfx = drc_counts(out)
    log.append(f"connected={connected}  real_drc_violations={real}  same_net_crossings={selfx}")
    log.append(f"cells inside rule areas: D8={in8}  D9={in9} (both must be 0)")
    passed = connected and in8 == 0 and in9 == 0 and real == 0
    return "R2 multi-net (both nets avoid the rule area)", passed, log


def scenario_noop_gating(verbose):
    """R3: a keepout that allows tracks+vias is a no-op (route cuts straight through)."""
    log = []
    polys = [D8_BOX]
    # tracks_allowed + vias_allowed -> nothing is blocked; route should ignore it.
    board = make_board_with_zones(
        _keepout_zone(D8_BOX, tracks_allowed=True, vias_allowed=True))
    out = board.replace(".kicad_pcb", "_out.kicad_pcb")
    ok, _ = run_route(board, out, ["Net-(D8-A)"], keepout=False, verbose=verbose)
    if not ok or not os.path.exists(out):
        return "R3 no-op gating", False, ["route.py failed"]
    connected = is_connected(out, ["Net-(D8-A)"])
    inside = intrusions(out, "Net-(D8-A)", polys)
    log.append(f"connected={connected}")
    log.append(f"routed cells inside the area={inside} "
               f"(tracks+vias allowed -> no-op -> route goes straight, must be > 0)")
    passed = connected and inside > 0
    return "R3 no-op gating (tracks+vias allowed area never blocks)", passed, log


SCENARIOS = [
    scenario_avoid,
    scenario_multi_net,
    scenario_noop_gating,
]


def main():
    import argparse
    parser = argparse.ArgumentParser(description="KiCad rule-area keepout tests (PR #25)")
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose routing output")
    args = parser.parse_args()

    if not os.path.exists(BASE_BOARD):
        print(f"ERROR: base board not found: {BASE_BOARD}")
        return 2

    print("=" * 70)
    print("KiCad keep-out rule-area routing tests")
    print("=" * 70)

    results = []
    for fn in SCENARIOS:
        name, passed, log = fn(args.verbose)
        status = "PASS" if passed else "FAIL"
        print(f"\n[{status}] {name}")
        for line in log:
            print(f"        {line}")
        results.append((name, passed))

    print("\n" + "=" * 70)
    n_pass = sum(1 for _, p in results if p)
    for name, passed in results:
        print(f"  {'PASS' if passed else 'FAIL'}  {name}")
    print(f"\n{n_pass}/{len(results)} scenarios passed")
    print("=" * 70)
    return 0 if n_pass == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())

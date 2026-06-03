#!/usr/bin/env python3
"""
Pass/fail tests for the keepout-zone routing feature (issue #27).

A user draws a closed polygon on a User layer (default User.2) and routed tracks
are kept OUT of it (a hard keepout). The block applies to every net routed in the
run and is fed into the shared obstacle map, so it works for single-ended,
multipoint, and differential-pair routing alike.

This script builds temporary boards from kicad_files/flat_hierarchy.kicad_pcb by
inserting a gr_poly on User.2, runs route.py with/without --keepout, and checks
each scenario. Each scenario prints a PASS/FAIL line plus log detail; the process
exits non-zero if any scenario fails.

Scenarios
---------
K0  Regression: a keepout polygon is present but the flag is OFF -> it is ignored
    and the net routes its normal straight path THROUGH the polygon region.
    Guards against the feature changing behavior when not requested.
K1  Hard avoid: a polygon straddling a net's straight path forces a detour; the
    net still connects, is DRC-clean, and NO routed cell lies inside the polygon.
K2  Multi-net: two nets routed with the same keepout enabled both connect and
    neither occupies a cell inside the polygon.
K3  Real board: kicad_files/lvds_converter_dualclk.kicad_pcb with a User.2 polygon
    across the /CLK net's path -> /CLK routes, connects, is DRC-clean, and avoids
    the polygon interior.

Run:
    python3 tests/test_keepout.py            # uses kicad_files/flat_hierarchy
    python3 tests/test_keepout.py -v          # verbose routing output
Use the same interpreter that has the Rust router available (e.g. KiCad's python).
"""

import argparse
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

import kicad_parser as kp  # noqa: E402
from routing_config import GridCoord  # noqa: E402
from obstacle_map import _polygon_grid_cells  # noqa: E402

BASE_BOARD = os.path.join(ROOT_DIR, "kicad_files", "flat_hierarchy.kicad_pcb")
LVDS_BOARD = os.path.join(ROOT_DIR, "kicad_files", "lvds_converter_dualclk.kicad_pcb")
GEOM = ["--track-width", "0.5", "--clearance", "0.4",
        "--via-size", "0.6", "--via-drill", "0.5", "--layers", "F.Cu", "B.Cu"]
CLEARANCE = "0.4"

_UUID = 0


def _gr_poly(points):
    """A closed gr_poly on User.2 from a list of (x, y) mm vertices."""
    global _UUID
    _UUID += 1
    pts = " ".join(f"(xy {x} {y})" for x, y in points)
    return (f'  (gr_poly (pts {pts}) (stroke (width 0.1) (type solid)) (fill none) '
            f'(layer "User.2") '
            f'(uuid 0000{_UUID:04d}-0000-0000-0000-000000000000))\n')


def box(x1, y1, x2, y2):
    """Rectangle polygon (4 vertices) from opposite corners."""
    return [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]


def make_board_with_keepout(polys):
    """Write a temp board = base board + the given User.2 keepout polygons.

    polys: list of vertex lists [(x, y), ...]. Returns the temp file path.
    """
    text = open(BASE_BOARD).read()
    blob = "\n" + "".join(_gr_poly(p) for p in polys) + "\n"
    idx = text.rstrip().rfind(")")
    fd, path = tempfile.mkstemp(suffix=".kicad_pcb", prefix="keepout_test_")
    os.close(fd)
    with open(path, "w") as f:
        f.write(text[:idx] + blob + text[idx:])
    return path


def run_route(board_in, board_out, nets, keepout=False, verbose=False, geom=None):
    """Run route.py; return (ok, combined_output)."""
    cmd = [sys.executable, "route.py", board_in, board_out] + (geom or GEOM) + ["--nets"] + nets
    if keepout:
        cmd.append("--keepout")
    if verbose:
        cmd.append("-v")
    r = subprocess.run(cmd, cwd=ROOT_DIR, capture_output=True, text=True)
    return (r.returncode == 0), (r.stdout + r.stderr)


def is_connected(board, nets):
    """True if check_connected reports all given nets fully connected."""
    cmd = [sys.executable, "check_connected.py", board, "--nets"] + nets
    r = subprocess.run(cmd, cwd=ROOT_DIR, capture_output=True, text=True)
    return "ALL NETS FULLY CONNECTED" in (r.stdout + r.stderr)


def drc_counts(board, clearance=CLEARANCE):
    """Return (real_violations, same_net_crossings).

    Same-net self-crossings are reported separately (KiCad permits same-net
    copper to overlap); real_violations counts everything else.
    """
    import re
    cmd = [sys.executable, "check_drc.py", board, "--clearance", str(clearance)]
    r = subprocess.run(cmd, cwd=ROOT_DIR, capture_output=True, text=True)
    out = r.stdout + r.stderr
    if "NO DRC VIOLATIONS" in out:
        return 0, 0
    total = re.search(r"FOUND (\d+) DRC", out)
    total = int(total.group(1)) if total else -1
    selfx = re.search(r"SEGMENT-CROSSING-SAME-NET violations \((\d+)\)", out)
    selfx = int(selfx.group(1)) if selfx else 0
    return (total - selfx if total >= 0 else -1), selfx


def net_id_for(pcb, name):
    for nid, net in pcb.nets.items():
        if net.name == name:
            return nid
    return None


def net_xy_cells(board, name, grid_step=0.1):
    """Set of (gx, gy) cells occupied by a net's segments (grid-sampled)."""
    from bresenham_utils import walk_line
    pcb = kp.parse_kicad_pcb(board)
    nid = net_id_for(pcb, name)
    coord = GridCoord(grid_step)
    cells = set()
    for s in pcb.segments:
        if s.net_id != nid:
            continue
        g1 = coord.to_grid(s.start_x, s.start_y)
        g2 = coord.to_grid(s.end_x, s.end_y)
        for gx, gy in walk_line(g1[0], g1[1], g2[0], g2[1]):
            cells.add((gx, gy))
    return cells


def keepout_cells(polys, grid_step=0.1):
    """All grid (gx, gy) cells inside any keepout polygon."""
    coord = GridCoord(grid_step)
    cells = set()
    for p in polys:
        cells |= _polygon_grid_cells(p, coord)
    return cells


def intrusions(board, name, polys, grid_step=0.1):
    """How many of a net's routed cells fall inside the keepout polygon(s)."""
    return len(net_xy_cells(board, name, grid_step) & keepout_cells(polys, grid_step))


# --------------------------------------------------------------------------
# Scenarios
# --------------------------------------------------------------------------

# Net-(D8-A): TH pads at (149.22, 77.47) and (153.67, 77.47) -> horizontal path.
# Net-(D9-A): TH pads at (149.22, 87.63) and (153.80, 87.63) -> horizontal path.
D8_BOX = box(150.6, 76.4, 152.3, 78.5)   # straddles the D8 path midpoint
D9_BOX = box(150.6, 86.6, 152.3, 88.7)   # straddles the D9 path midpoint


def scenario_regression_off(verbose):
    """K0: keepout present but flag OFF -> net cuts straight through the zone."""
    log = []
    polys = [D8_BOX]
    board = make_board_with_keepout(polys)
    out = board.replace(".kicad_pcb", "_out.kicad_pcb")
    ok, _ = run_route(board, out, ["Net-(D8-A)"], keepout=False, verbose=verbose)
    if not ok or not os.path.exists(out):
        return "K0 regression (flag off)", False, ["route.py failed"]
    connected = is_connected(out, ["Net-(D8-A)"])
    inside = intrusions(out, "Net-(D8-A)", polys)
    log.append(f"connected={connected}")
    log.append(f"routed cells inside the zone={inside} (flag off -> route ignores zone, must be > 0)")
    passed = connected and inside > 0
    return "K0 regression (keepout ignored when flag off)", passed, log


def scenario_hard_avoid(verbose):
    """K1: keepout straddling the path forces a detour; zero cells inside, clean."""
    log = []
    polys = [D8_BOX]
    board = make_board_with_keepout(polys)
    out = board.replace(".kicad_pcb", "_out.kicad_pcb")
    ok, _ = run_route(board, out, ["Net-(D8-A)"], keepout=True, verbose=verbose)
    if not ok or not os.path.exists(out):
        return "K1 hard avoid", False, ["route.py failed"]
    connected = is_connected(out, ["Net-(D8-A)"])
    inside = intrusions(out, "Net-(D8-A)", polys)
    real, selfx = drc_counts(out)
    log.append(f"connected={connected}  real_drc_violations={real}  same_net_crossings={selfx}")
    log.append(f"routed cells inside the zone={inside} (must be 0)")
    passed = connected and inside == 0 and real == 0
    return "K1 hard avoid (route detours around zone, no clearance errors)", passed, log


def scenario_multi_net(verbose):
    """K2: two nets routed with keepout; both connect and avoid the zone(s)."""
    log = []
    polys = [D8_BOX, D9_BOX]
    board = make_board_with_keepout(polys)
    out = board.replace(".kicad_pcb", "_out.kicad_pcb")
    nets = ["Net-(D8-A)", "Net-(D9-A)"]
    ok, _ = run_route(board, out, nets, keepout=True, verbose=verbose)
    if not ok or not os.path.exists(out):
        return "K2 multi-net", False, ["route.py failed"]
    connected = is_connected(out, nets)
    in8 = intrusions(out, "Net-(D8-A)", polys)
    in9 = intrusions(out, "Net-(D9-A)", polys)
    real, selfx = drc_counts(out)
    log.append(f"connected={connected}  real_drc_violations={real}  same_net_crossings={selfx}")
    log.append(f"cells inside zone: D8={in8}  D9={in9} (both must be 0)")
    passed = connected and in8 == 0 and in9 == 0 and real == 0
    return "K2 multi-net (both nets avoid the keepout)", passed, log


def scenario_real_board(verbose):
    """K3: real board /CLK avoids a User.2 keepout across its path."""
    log = []
    if not os.path.exists(LVDS_BOARD):
        return "K3 real board /CLK avoids keepout", True, ["SKIP: board not present"]
    # /CLK pads: J2 (125.75,58.73), IC3 (103.2,74.2), IC4 (97.69,65.82).
    # Box across the J2<->IC3 leg, clear of all three pads.
    polys = [box(112.5, 64.5, 116.0, 68.0)]
    geom = ["--track-width", "0.2", "--clearance", "0.2", "--layers", "F.Cu", "B.Cu"]
    text = open(LVDS_BOARD).read()
    blob = "\n" + "".join(_gr_poly(p) for p in polys) + "\n"
    idx = text.rstrip().rfind(")")
    fd, board = tempfile.mkstemp(suffix=".kicad_pcb", prefix="keepout_lvds_")
    os.close(fd)
    with open(board, "w") as f:
        f.write(text[:idx] + blob + text[idx:])
    out = os.path.join(tempfile.gettempdir(), "lvds_clk_keepout.kicad_pcb")
    ok, _ = run_route(board, out, ["/CLK"], keepout=True, verbose=verbose, geom=geom)
    if not ok or not os.path.exists(out):
        return "K3 real board /CLK avoids keepout", False, ["route.py failed"]
    connected = is_connected(out, ["/CLK"])
    inside = intrusions(out, "/CLK", polys)
    real, _selfx = drc_counts(out, clearance="0.2")
    log.append(f"connected={connected}  real_drc_violations={real}")
    log.append(f"routed cells inside the zone={inside} (must be 0)")
    passed = connected and inside == 0 and real == 0
    return "K3 real board: /CLK avoids its User.2 keepout", passed, log


SCENARIOS = [
    scenario_regression_off,
    scenario_hard_avoid,
    scenario_multi_net,
    scenario_real_board,
]


def main():
    parser = argparse.ArgumentParser(description="Keepout-zone routing tests (issue #27)")
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose routing output")
    args = parser.parse_args()

    if not os.path.exists(BASE_BOARD):
        print(f"ERROR: base board not found: {BASE_BOARD}")
        return 2

    print("=" * 70)
    print("Keepout-zone routing tests")
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

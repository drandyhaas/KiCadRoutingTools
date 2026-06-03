#!/usr/bin/env python3
"""
Pass/fail tests for the guide-corridor (waypoint) routing feature (issue #7).

A user draws a polyline on a User layer (default User.1) and the selected nets
are routed to FOLLOW it: source -> drawn vertices -> target, getting as close to
the line as obstacles allow. When a waypoint cell is blocked, the route aims for
the nearest free cell, and multiple nets asked to follow the same corridor pack
alongside one another without overlapping.

This script builds temporary boards from kicad_files/flat_hierarchy.kicad_pcb by
inserting graphic lines on User.1, runs route.py with/without --guide-corridor,
and checks each scenario. Each scenario prints a PASS/FAIL line plus log detail;
the process exits non-zero if any scenario fails.

Scenarios
---------
S0  Regression: with the flag OFF, a guide on the board is ignored and the net
    routes its normal (short, straight) path. Guards against the feature
    changing behavior when not requested.
S1  Follow: a single net follows an arch drawn over it (the routed track reaches
    near the drawn peak), connects, and has no clearance errors against other nets.
S2  Blocked waypoint: a guide vertex placed exactly on ANOTHER net's through-hole
    pad (blocked on all layers) is snapped to the nearest free cell; the net still
    connects with no clip of the blocking pad.
S3  Shared corridor: two nets told to follow the same arch both connect and do
    NOT overlap (no shared copper cell), i.e. they pack alongside each other.

Pass/fail is based on real clearance violations (clearance to OTHER nets), plus
connectivity / follow / non-overlap as appropriate. Same-net self-crossings are
reported as a quality note, not a failure: waypoint routing concatenates A* legs,
so a multi-point tap or a blocked-waypoint detour can cross an earlier leg of the
SAME net. KiCad permits same-net copper to overlap, so this is harmless
electrically (the net stays connected) though it is a route-quality nicety to
improve later.

Run:
    python3 tests/test_guide_corridor.py            # uses kicad_files/flat_hierarchy
    python3 tests/test_guide_corridor.py -v          # verbose routing output
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
from routing_config import GridCoord, GridRouteConfig  # noqa: E402
from single_ended_routing import build_corridor_waypoints  # noqa: E402

BASE_BOARD = os.path.join(ROOT_DIR, "kicad_files", "flat_hierarchy.kicad_pcb")
LVDS_BOARD = os.path.join(ROOT_DIR, "kicad_files", "lvds_converter_dualclk.kicad_pcb")
GEOM = ["--track-width", "0.5", "--clearance", "0.4",
        "--via-size", "0.6", "--via-drill", "0.5", "--layers", "F.Cu", "B.Cu"]
CLEARANCE = "0.4"

# A guide segment is one (x1,y1,x2,y2) line on User.1.
_UUID = 0


def _gr_line(x1, y1, x2, y2):
    global _UUID
    _UUID += 1
    return (f'  (gr_line (start {x1} {y1}) (end {x2} {y2}) '
            f'(stroke (width 0.1) (type solid)) (layer "User.1") '
            f'(uuid 0000{_UUID:04d}-0000-0000-0000-000000000000))\n')


def make_board_with_guide(segments):
    """Write a temp board = base board + the given User.1 guide segments.

    segments: list of (x1, y1, x2, y2) in mm. Returns the temp file path.
    """
    text = open(BASE_BOARD).read()
    guide = "\n" + "".join(_gr_line(*s) for s in segments) + "\n"
    idx = text.rstrip().rfind(")")
    fd, path = tempfile.mkstemp(suffix=".kicad_pcb", prefix="guide_test_")
    os.close(fd)
    with open(path, "w") as f:
        f.write(text[:idx] + guide + text[idx:])
    return path


def run_route(board_in, board_out, nets, corridor=False, verbose=False, geom=None):
    """Run route.py; return (ok, combined_output)."""
    cmd = [sys.executable, "route.py", board_in, board_out] + (geom or GEOM) + ["--nets"] + nets
    if corridor:
        cmd.append("--guide-corridor")
    if verbose:
        cmd.append("-v")
    r = subprocess.run(cmd, cwd=ROOT_DIR, capture_output=True, text=True)
    return (r.returncode == 0), (r.stdout + r.stderr)


DIFF_GEOM = ["--track-width", "0.2", "--clearance", "0.2", "--layers", "F.Cu", "B.Cu"]


def run_route_diff(board_in, board_out, nets, corridor=False, verbose=False, geom=None):
    """Run route_diff.py; return (routed_ok, combined_output)."""
    cmd = [sys.executable, "route_diff.py", board_in, board_out] + (geom or DIFF_GEOM) + ["--nets"] + nets
    if corridor:
        cmd.append("--guide-corridor")
    if verbose:
        cmd.append("-v")
    r = subprocess.run(cmd, cwd=ROOT_DIR, capture_output=True, text=True)
    out = r.stdout + r.stderr
    return ('"successful": 1' in out or "1/1 routed" in out), out


def _insert_user1_guide(base_path, segments):
    """Write a temp board = base_path + the given User.1 guide segments."""
    text = open(base_path).read()
    guide = "\n" + "".join(_gr_line(*s) for s in segments) + "\n"
    idx = text.rstrip().rfind(")")
    fd, path = tempfile.mkstemp(suffix=".kicad_pcb", prefix="diffguide_")
    os.close(fd)
    with open(path, "w") as f:
        f.write(text[:idx] + guide + text[idx:])
    return path


def _net_min_y(board, names):
    """Smallest y over the routed segments of the named nets (top of the route)."""
    pcb = kp.parse_kicad_pcb(board)
    nids = {nid for nid, net in pcb.nets.items() if net.name in names}
    ys = [v for s in pcb.segments if s.net_id in nids for v in (s.start_y, s.end_y)]
    return min(ys) if ys else None


def _pt_seg_dist(px, py, ax, ay, bx, by):
    """Distance (mm) from point to segment."""
    import math as _m
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return _m.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return _m.hypot(px - (ax + t * dx), py - (ay + t * dy))


def guide_vertices_followed(board, name, tol_mm=2.0):
    """Count how many User.1 guide vertices the routed net passes within tol_mm of.

    Returns (followed, total) - distinguishes a route that follows the drawn
    guide from one that ignores it and goes straight.
    """
    pcb = kp.parse_kicad_pcb(board)
    nid = net_id_for(pcb, name)
    segs = [s for s in pcb.segments if s.net_id == nid]
    verts = [pt for gp in pcb.guide_paths for pt in gp.points]
    followed = 0
    for (vx, vy) in verts:
        if segs and min(_pt_seg_dist(vx, vy, s.start_x, s.start_y, s.end_x, s.end_y) for s in segs) <= tol_mm:
            followed += 1
    return followed, len(verts)


def is_connected(board, nets):
    """True if check_connected reports all given nets fully connected."""
    cmd = [sys.executable, "check_connected.py", board, "--nets"] + nets
    r = subprocess.run(cmd, cwd=ROOT_DIR, capture_output=True, text=True)
    return "ALL NETS FULLY CONNECTED" in (r.stdout + r.stderr)


def drc_counts(board, clearance=CLEARANCE):
    """Return (real_violations, same_net_crossings).

    Same-net self-crossings are reported separately: KiCad permits same-net
    copper to overlap, so they are a route-quality note rather than a real
    clearance error. `real_violations` counts everything else (clearance to
    OTHER nets, board edge, etc.).
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


def net_y_extent(board, name):
    """(min_y, max_y) over the routed segments of a net."""
    pcb = kp.parse_kicad_pcb(board)
    nid = net_id_for(pcb, name)
    ys = [s.start_y for s in pcb.segments if s.net_id == nid]
    ys += [s.end_y for s in pcb.segments if s.net_id == nid]
    return (min(ys), max(ys)) if ys else (None, None)


def net_via_count(board, name):
    """Number of vias on a net."""
    pcb = kp.parse_kicad_pcb(board)
    nid = net_id_for(pcb, name)
    return sum(1 for v in pcb.vias if v.net_id == nid)


def net_occupied_cells(board, name, grid_step=0.1):
    """Set of (gx, gy, layer) cells occupied by a net's segments (grid-sampled)."""
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
            cells.add((gx, gy, s.layer))
    return cells


# --------------------------------------------------------------------------
# Scenarios
# --------------------------------------------------------------------------

def scenario_regression_off(verbose):
    """S0: guide present but flag OFF -> normal (straight) route, unchanged."""
    log = []
    # Arch over the multi-point net D10-A; without the flag it must be ignored.
    board = make_board_with_guide([(118.1, 57.1, 122.0, 67.0), (122.0, 67.0, 125.7, 62.2)])
    out = board.replace(".kicad_pcb", "_out.kicad_pcb")
    ok, _ = run_route(board, out, ["Net-(D10-A)"], corridor=False, verbose=verbose)
    if not ok or not os.path.exists(out):
        return "S0 regression (flag off)", False, ["route.py failed"]
    _, max_y = net_y_extent(out, "Net-(D10-A)")
    connected = is_connected(out, ["Net-(D10-A)"])
    # Pads are at y<=62.2; with the flag OFF the route must NOT bow up to the arch (y~67).
    not_following = max_y is not None and max_y < 63.0
    log.append(f"max_y={max_y:.2f} (pads y<=62.2, arch peak=67.0) -> following={not not_following}")
    log.append(f"connected={connected}")
    passed = connected and not_following
    return "S0 regression (guide ignored when flag off)", passed, log


def scenario_follow(verbose):
    """S1: single net follows an arch drawn over it; connected + DRC clean."""
    log = []
    board = make_board_with_guide([(118.1, 57.1, 122.0, 67.0), (122.0, 67.0, 125.7, 62.2)])
    out = board.replace(".kicad_pcb", "_out.kicad_pcb")
    ok, _ = run_route(board, out, ["Net-(D10-A)"], corridor=True, verbose=verbose)
    if not ok or not os.path.exists(out):
        return "S1 follow", False, ["route.py failed"]
    _, max_y = net_y_extent(out, "Net-(D10-A)")
    connected = is_connected(out, ["Net-(D10-A)"])
    real, selfx = drc_counts(out)
    follows = max_y is not None and max_y >= 65.0  # reaches near the arch peak (67.0)
    log.append(f"max_y={max_y:.2f} (arch peak=67.0) -> follows={follows}")
    log.append(f"connected={connected}  real_drc_violations={real}  same_net_crossings={selfx}")
    passed = connected and follows and real == 0
    return "S1 follow (route hugs drawn arch, no clearance errors)", passed, log


def scenario_blocked_waypoint(verbose):
    """S2: a guide vertex on another net's through-hole pad is snapped; clean."""
    log = []
    # Middle vertex (149.22, 83.82) is R19.1, a through-hole pad on Net-(D11-A).
    board = make_board_with_guide([(153.67, 77.47, 149.22, 83.82),
                                   (149.22, 83.82, 149.22, 77.47)])
    out = board.replace(".kicad_pcb", "_out.kicad_pcb")
    ok, _ = run_route(board, out, ["Net-(D8-A)"], corridor=True, verbose=verbose)
    if not ok or not os.path.exists(out):
        return "S2 blocked waypoint", False, ["route.py failed"]
    connected = is_connected(out, ["Net-(D8-A)"])
    real, selfx = drc_counts(out)
    log.append("waypoint placed on D11-A through-hole pad (blocked all layers)")
    log.append(f"connected={connected}  real_drc_violations={real} (no clip of the pad)  "
               f"same_net_crossings={selfx}")
    passed = connected and real == 0
    return "S2 blocked waypoint (snaps to free cell, no clearance errors)", passed, log


def scenario_shared_corridor(verbose):
    """S3: two nets follow the same arch; both connect and do not overlap."""
    log = []
    board = make_board_with_guide([(153.67, 77.47, 151.4, 72.0),
                                   (151.4, 72.0, 149.22, 77.47)])
    out = board.replace(".kicad_pcb", "_out.kicad_pcb")
    ok, _ = run_route(board, out, ["Net-(D8-A)", "Net-(D9-A)"], corridor=True, verbose=verbose)
    if not ok or not os.path.exists(out):
        return "S3 shared corridor", False, ["route.py failed"]
    connected = is_connected(out, ["Net-(D8-A)", "Net-(D9-A)"])
    cells_a = net_occupied_cells(out, "Net-(D8-A)")
    cells_b = net_occupied_cells(out, "Net-(D9-A)")
    overlap = cells_a & cells_b
    log.append(f"connected={connected}")
    log.append(f"D8 cells={len(cells_a)}  D9 cells={len(cells_b)}  "
               f"overlapping copper cells={len(overlap)} (must be 0)")
    passed = connected and len(overlap) == 0
    return "S3 shared corridor (two nets pack non-overlapping)", passed, log


def scenario_never_blocks(verbose):
    """S4: a corridor must never make a net fail that routes without it.

    Routes a multi-point net with a pathological guide (a vertex far off-board)
    that can't be followed, and asserts the net still routes and connects - the
    unfollowable waypoint is dropped and the route falls back per-segment.
    """
    log = []
    # D10-A pads are around (118-126, 57-62); send a vertex far off-board.
    board = make_board_with_guide([(118.1, 57.1, 10.0, 10.0), (10.0, 10.0, 125.7, 62.2)])
    out = board.replace(".kicad_pcb", "_out.kicad_pcb")
    ok, _ = run_route(board, out, ["Net-(D10-A)"], corridor=True, verbose=verbose)
    if not ok or not os.path.exists(out):
        return "S4 never blocks routing", False, ["route.py failed"]
    connected = is_connected(out, ["Net-(D10-A)"])
    real, selfx = drc_counts(out)
    log.append("guide has an unreachable off-board vertex (10,10)")
    log.append(f"connected={connected} (corridor must not prevent routing)  "
               f"real_drc_violations={real}  same_net_crossings={selfx}")
    passed = connected and real == 0
    return "S4 corridor never prevents a route that would otherwise succeed", passed, log


def scenario_real_board(verbose):
    """S5: real board regression - /CLK follows its User.1 guide across the MST.

    Uses kicad_files/lvds_converter_dualclk.kicad_pcb, which has a 6-vertex guide
    on User.1 spanning the 3-pad /CLK net. This is the board from the reported bug
    where the guide made /CLK fail to route. Asserts the net routes, connects, is
    DRC-clean, and actually follows the guide (the route passes near most vertices,
    distributed across both MST segments - not a straight ignore-the-guide route).
    """
    log = []
    if not os.path.exists(LVDS_BOARD):
        return "S5 real board /CLK follows guide", True, ["SKIP: board not present"]
    out = os.path.join(tempfile.gettempdir(), "lvds_clk_corridor.kicad_pcb")
    base = os.path.join(tempfile.gettempdir(), "lvds_clk_direct.kicad_pcb")
    geom = ["--track-width", "0.2", "--clearance", "0.2", "--layers", "F.Cu", "B.Cu"]
    ok, _ = run_route(LVDS_BOARD, out, ["/CLK"], corridor=True, verbose=verbose, geom=geom)
    ok_base, _ = run_route(LVDS_BOARD, base, ["/CLK"], corridor=False, verbose=verbose, geom=geom)
    if not ok or not os.path.exists(out):
        return "S5 real board /CLK follows guide", False, ["route.py failed"]
    connected = is_connected(out, ["/CLK"])
    # The corridor must not introduce vias the direct route didn't need.
    corr_vias = net_via_count(out, "/CLK")
    direct_vias = net_via_count(base, "/CLK") if ok_base and os.path.exists(base) else 0
    real, _selfx = drc_counts(out, clearance="0.2")  # this board's clearance
    followed, total = guide_vertices_followed(out, "/CLK", tol_mm=2.0)
    log.append(f"connected={connected}  real_drc_violations={real}")
    log.append(f"guide vertices followed: {followed}/{total} (within 2mm of the route)")
    log.append(f"vias: corridor={corr_vias}  direct={direct_vias} (corridor must not add vias)")
    passed = (connected and real == 0 and followed >= max(2, total - 1)
              and corr_vias <= direct_vias)
    return "S5 real board: /CLK follows its User.1 guide (issue #7 regression)", passed, log


def scenario_spacing_subdivides(verbose):
    """S6: guide_corridor_spacing > 0 subdivides long guide segments (unit test).

    Builds the waypoint list from the LVDS board's User.1 guide with spacing=0
    (vertices only) vs spacing=1.0mm, and checks the latter produces more, more
    closely-spaced waypoints (no consecutive gap larger than ~spacing).
    """
    log = []
    if not os.path.exists(LVDS_BOARD):
        return "S6 spacing>0 subdivides waypoints", True, ["SKIP: board not present"]
    pcb = kp.parse_kicad_pcb(LVDS_BOARD)
    grid_step, spacing_mm = 0.1, 1.0
    cfg0 = GridRouteConfig(guide_corridor_enabled=True, guide_corridor_spacing=0.0,
                           grid_step=grid_step, layers=["F.Cu", "B.Cu"])
    cfg1 = GridRouteConfig(guide_corridor_enabled=True, guide_corridor_spacing=spacing_mm,
                           grid_step=grid_step, layers=["F.Cu", "B.Cu"])
    wp0 = build_corridor_waypoints(pcb, cfg0)
    wp1 = build_corridor_waypoints(pcb, cfg1)
    spacing_grid = int(spacing_mm / grid_step)  # 10 cells
    gaps = [max(abs(wp1[i + 1][0] - wp1[i][0]), abs(wp1[i + 1][1] - wp1[i][1]))
            for i in range(len(wp1) - 1)]
    max_gap = max(gaps) if gaps else 0
    log.append(f"vertices-only waypoints={len(wp0)}  spacing={spacing_mm}mm waypoints={len(wp1)}")
    log.append(f"max consecutive gap @ spacing={spacing_mm}mm: {max_gap} cells "
               f"(must be <= {spacing_grid} + slack)")
    passed = len(wp1) > len(wp0) and max_gap <= spacing_grid + 2
    return "S6 spacing>0 subdivides long guide segments into denser waypoints", passed, log


def scenario_real_board_spacing(verbose):
    """S7: LVDS /CLK with spacing>0 still routes cleanly and follows the guide.

    Exercises the subdivision path end-to-end on the real board: denser waypoints
    must not break routing, add vias, or violate clearance.
    """
    log = []
    if not os.path.exists(LVDS_BOARD):
        return "S7 real board spacing>0 routes cleanly", True, ["SKIP: board not present"]
    base_geom = ["--track-width", "0.2", "--clearance", "0.2", "--layers", "F.Cu", "B.Cu"]
    out = os.path.join(tempfile.gettempdir(), "lvds_clk_spacing.kicad_pcb")
    base = os.path.join(tempfile.gettempdir(), "lvds_clk_direct2.kicad_pcb")
    ok, _ = run_route(LVDS_BOARD, out, ["/CLK"], corridor=True, verbose=verbose,
                      geom=base_geom + ["--guide-corridor-spacing", "1.0"])
    ok_base, _ = run_route(LVDS_BOARD, base, ["/CLK"], corridor=False, verbose=verbose, geom=base_geom)
    if not ok or not os.path.exists(out):
        return "S7 real board spacing>0 routes cleanly", False, ["route.py failed"]
    connected = is_connected(out, ["/CLK"])
    real, _selfx = drc_counts(out, clearance="0.2")
    corr_vias = net_via_count(out, "/CLK")
    direct_vias = net_via_count(base, "/CLK") if ok_base and os.path.exists(base) else 0
    followed, total = guide_vertices_followed(out, "/CLK", tol_mm=2.0)
    log.append(f"connected={connected}  real_drc_violations={real}")
    log.append(f"guide vertices followed: {followed}/{total}")
    log.append(f"vias: corridor={corr_vias}  direct={direct_vias}")
    passed = (connected and real == 0 and followed >= max(2, total - 1)
              and corr_vias <= direct_vias)
    return "S7 real board: /CLK with spacing>0 routes cleanly and follows", passed, log


def scenario_diff_pair_follow(verbose):
    """D1: a differential pair's centerline follows a guide arch drawn over it.

    Routes the /CLK+ /CLK- pair on the LVDS board with and without the guide and
    checks the guided centerline bows toward the drawn arch (issue #7 for diff
    pairs, via the pose-router wildcard waypoint legs).
    """
    log = []
    if not os.path.exists(LVDS_BOARD):
        return "D1 diff-pair follows guide", True, ["SKIP: board not present"]
    nets = ["/CLK+", "/CLK-"]
    # The pair routes in the y~63-67 band between J1 (x~84) and R3 (x~97);
    # arch peaks UP at y=57, so following it lowers the route's min_y.
    arch = [(86, 64, 91, 57), (91, 57, 96, 64)]
    board = _insert_user1_guide(LVDS_BOARD, arch)
    base_out = board.replace(".kicad_pcb", "_base.kicad_pcb")
    guided_out = board.replace(".kicad_pcb", "_guided.kicad_pcb")
    ok_b, _ = run_route_diff(board, base_out, nets, corridor=False, verbose=verbose)
    ok_g, _ = run_route_diff(board, guided_out, nets, corridor=True, verbose=verbose)
    if not (ok_b and ok_g):
        return "D1 diff-pair follows guide", False, [f"route_diff failed (base={ok_b}, guided={ok_g})"]
    mb = _net_min_y(base_out, set(nets))
    mg = _net_min_y(guided_out, set(nets))
    follows = mb is not None and mg is not None and mg <= mb - 1.0  # bows >=1mm toward arch
    log.append(f"baseline min_y={mb:.2f}  guided min_y={mg:.2f}  (arch peak=57.0)")
    log.append(f"both pairs routed; guided bows toward arch by {mb - mg:.2f}mm -> follows={follows}")
    return "D1 diff-pair centerline follows the drawn guide", (ok_g and follows), log


def scenario_diff_pair_never_blocks(verbose):
    """D2: a guide a diff pair can't follow must not make it fail (falls back)."""
    log = []
    if not os.path.exists(LVDS_BOARD):
        return "D2 diff-pair guide never blocks", True, ["SKIP: board not present"]
    nets = ["/CLK+", "/CLK-"]
    # An unreachable off-board vertex; the chained legs fail and the pair falls
    # back to the direct centerline.
    arch = [(86, 64, 10, 10), (10, 10, 96, 64)]
    board = _insert_user1_guide(LVDS_BOARD, arch)
    out = board.replace(".kicad_pcb", "_nb.kicad_pcb")
    ok, _ = run_route_diff(board, out, nets, corridor=True, verbose=verbose)
    log.append(f"guide has an unreachable off-board vertex (10,10); pair routed={ok}")
    return "D2 diff-pair guide never prevents routing (falls back)", ok, log


SCENARIOS = [
    scenario_regression_off,
    scenario_follow,
    scenario_blocked_waypoint,
    scenario_shared_corridor,
    scenario_never_blocks,
    scenario_real_board,
    scenario_spacing_subdivides,
    scenario_real_board_spacing,
    scenario_diff_pair_follow,
    scenario_diff_pair_never_blocks,
]


def main():
    parser = argparse.ArgumentParser(description="Guide-corridor routing tests (issue #7)")
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose routing output")
    args = parser.parse_args()

    if not os.path.exists(BASE_BOARD):
        print(f"ERROR: base board not found: {BASE_BOARD}")
        return 2

    print("=" * 70)
    print("Guide-corridor (waypoint) routing tests")
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

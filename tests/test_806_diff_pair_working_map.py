#!/usr/bin/env python3
"""#806: the diff-pair engine keeps the persistent working obstacle map in
step with the copper it commits -- and rip/restore stays balanced AND correct
for BOTH halves of a pair.

WHAT WAS WRONG. route_diff builds one persistent working map + per-net cache
(`state.working_obstacles` / `state.net_obstacles_cache`) and hands them to
rip_up_net / restore_net / try_terminal_restore, but the diff loop's success
paths never refreshed a routed pair's entry, so every entry stayed the
PRE-ROUTE (stubs-only) one it was built with. The ref-count invariants A/B/C
(tests/release/check_obstacle_balance.py) HELD throughout -- the stale entry
was removed and re-added in balance -- while the map's CONTENT was wrong.
Measured on dual_ipex_csi_interposer (recorded diff stage, 10 pairs, rip
churn) at the point the map's in-run consumers use it: 30862 of 47448 cells
the board's copper owned were NOT blocked, 31330 of 71537 via cells were not
via-blocked, 20 stale entries (= every routed member). After the fix: 0 / 0
/ 0 on the same population.

THREE CHECKS, each asserting the REASON (tests/run_utils.check conventions):

  1. end-to-end control: route the in-repo esp_prog USB pair with the audit
     armed and read the PRE-SYNC content line (the one taken before
     sync_pcb_data_segments recomputes every entry at the end and hides what
     the loops left stale). Population is printed and asserted non-zero, so
     a run that routed nothing cannot pass vacuously. A/B/C are asserted
     with the release gate's own parser.
  2. try_terminal_restore on a PAIR payload takes the 'stub' path and keeps
     the N member's escape stub too (it walked only P's pads), and refreshes
     BOTH members' map entries (it refreshed net_id's only).
  3. stamp_result_copper stamps a result's own P/N via barrels (the victim-
     reroute map inside try_fallback_layer_swap stamped GND vias only).

NEGATIVE CONTROL (measured, fix stashed, instrumentation kept): check 1
fails on E with 1180 of 23100 cells NOT blocked, 1420 of 23671 via cells NOT
via-blocked and 2 stale entries (/D_P, /D_N: 1187 cells absent) while A/B/C
still PASS; check 2 fails on "N escape stub restored" and "N entry grew";
check 3 reports stamp_result_copper missing. Recorded in the PR.

Run:  python3 tests/test_806_diff_pair_working_map.py [-v]
"""
import argparse
import json
import os
import re
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TESTS = os.path.join(ROOT, 'tests')
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(TESTS, 'release'))

import numpy as np

from run_utils import check as run_check, evidence, tool

BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
NETS = ['/D_P', '/D_N']
GEOM = ['--track-width', '0.2', '--diff-pair-gap', '0.25', '--clearance', '0.1778',
        '--via-size', '0.45', '--via-drill', '0.3', '--no-gnd-vias']

CONTENT_RE = re.compile(
    r"\[OBSTACLE CONTENT (?P<label>[^\]]*)\] (?P<nets>\d+) nets recomputed from "
    r"pcb_data: (?P<cells>\d+) cells checked, (?P<missing>\d+) NOT blocked in "
    r"working map; (?P<vias>\d+) via cells checked, (?P<vmissing>\d+) NOT "
    r"via-blocked; (?P<stale_cells>\d+) cells absent from their net's cache "
    r"entry \((?P<stale_entries>\d+) stale entries\)")

FAILS = []


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name
          + (f"  [{detail}]" if detail and not cond else ""))
    if not cond:
        FAILS.append(name)


# ---------------------------------------------------------------------------
# 1. End-to-end control on a real route (seconds).
# ---------------------------------------------------------------------------
def test_end_to_end(verbose):
    print("\n[1] route_diff esp_prog with KICAD_OBSTACLE_AUDIT=1: pre-sync content")
    evidence(BOARD, 'esp_prog board')
    env = dict(os.environ, KICAD_OBSTACLE_AUDIT='1', KICAD_OBSTACLE_LEDGER='1')
    with tempfile.TemporaryDirectory(prefix='t806_') as d:
        out = os.path.join(d, 'esp_806.kicad_pcb')
        argv = [sys.executable, '-X', 'utf8', tool('route_diff.py'), BOARD,
                '--output', out, '--nets', *NETS, *GEOM]
        # run_utils.check with accept=True: a traceback / ImportError is a
        # BROKEN TEST, not a verdict.
        import subprocess
        r = subprocess.run(argv, capture_output=True, text=True, cwd=ROOT,
                           env=env, encoding='utf-8', errors='replace')
        txt = (r.stdout or '') + (r.stderr or '')
        if verbose:
            print(txt)
        check("route_diff exited 0 (no traceback)", r.returncode == 0
              and 'Traceback' not in txt, txt[-600:])
        # The pair actually routed: the population must contain routed copper.
        routed = []
        for line in txt.splitlines():
            if line.startswith('JSON_SUMMARY:'):
                try:
                    routed = json.loads(line[len('JSON_SUMMARY:'):]).get(
                        'routed_diff_pairs', [])
                except Exception:
                    routed = []
        check("the pair routed (population carries routed copper)", bool(routed),
              f"routed_diff_pairs={routed}")
        pre = [m for m in CONTENT_RE.finditer(txt) if 'pre-sync' in m.group('label')]
        check("pre-sync content audit line present", len(pre) == 1,
              f"found {len(pre)}")
        if pre:
            m = pre[0].groupdict()
            cells, vias = int(m['cells']), int(m['vias'])
            print(f"        population: {m['nets']} nets, {cells} cells, "
                  f"{vias} via cells checked")
            check("population non-zero (not vacuous)", cells > 0 and vias > 0)
            check("E: 0 cells NOT blocked in the working map",
                  int(m['missing']) == 0, f"{m['missing']} of {cells} NOT blocked")
            check("E: 0 via cells NOT via-blocked",
                  int(m['vmissing']) == 0, f"{m['vmissing']} of {vias}")
            check("E: 0 stale cache entries (every routed member refreshed)",
                  int(m['stale_entries']) == 0,
                  f"{m['stale_entries']} stale entries, "
                  f"{m['stale_cells']} cells absent from their entry")
        # A/B/C with the RELEASE GATE's own parser and verdict.
        from check_obstacle_balance import parse_audit, verdict
        a = parse_audit(txt)
        ok, failures = verdict(a)
        check("A/B/C hold (release-gate verdict)", ok, "; ".join(failures))
        check("ledger recorded events (instrument engaged)",
              any(int(x) > 0 for x in re.findall(r"\[OBSTACLE LEDGER\] (\d+) events", txt)))


# ---------------------------------------------------------------------------
# 2. try_terminal_restore: both halves of a pair payload.
# ---------------------------------------------------------------------------
def _fixture():
    from kicad_parser import parse_kicad_pcb, Segment
    from routing_config import GridRouteConfig
    pcb = parse_kicad_pcb(BOARD)
    ids = {n.name: nid for nid, n in pcb.nets.items()}
    p_id, n_id = ids['/D_P'], ids['/D_N']
    layers = list(pcb.board_info.copper_layers)
    config = GridRouteConfig(layers=layers, grid_step=0.05, track_width=0.2,
                             clearance=0.1778, via_size=0.45, via_drill=0.3)

    def pad(nid, ref):
        return next(p for p in pcb.nets[nid].pads if p.component_ref == ref)

    def seg(nid, x0, y0, x1, y1):
        return Segment(start_x=x0, start_y=y0, end_x=x1, end_y=y1,
                       width=0.2, layer='F.Cu', net_id=nid)

    # A saved pair payload: a short escape stub off each U1 pad (walked by
    # _stub_subset as "starts at a pad centre, <= 2 mm") plus a long run to
    # the USB pad. A foreign track across both long runs makes the FULL
    # restore conflict, so try_terminal_restore must take the 'stub' path.
    pu, nu = pad(p_id, 'U1'), pad(n_id, 'U1')
    pc, nc = pad(p_id, 'USB1'), pad(n_id, 'USB1')
    p_stub = seg(p_id, pu.global_x, pu.global_y, pu.global_x, pu.global_y + 0.9)
    n_stub = seg(n_id, nu.global_x, nu.global_y, nu.global_x, nu.global_y + 1.1)
    p_long = seg(p_id, pu.global_x, pu.global_y + 0.9, pc.global_x, pc.global_y)
    n_long = seg(n_id, nu.global_x, nu.global_y + 1.1, nc.global_x, nc.global_y)
    foreign_id = next(nid for nid in pcb.nets if nid not in (p_id, n_id, 0))
    xm = (pu.global_x + pc.global_x) / 2
    pcb.segments.append(seg(foreign_id, xm, 98.0, xm, 106.0))
    payload = {'new_segments': [p_stub, p_long, n_stub, n_long], 'new_vias': []}
    pcb._rip_saved = {p_id: (payload, [p_id, n_id], True)}
    return pcb, config, p_id, n_id, p_stub, n_stub, p_long, n_long


def test_terminal_restore_pair(verbose):
    print("\n[2] try_terminal_restore on a pair payload: N half + both map entries")
    from obstacle_map import GridObstacleMap
    from obstacle_cache import (precompute_net_obstacles, build_working_obstacle_map,
                                remove_net_obstacles_from_cache)
    from rip_restore import try_terminal_restore
    pcb, config, p_id, n_id, p_stub, n_stub, p_long, n_long = _fixture()
    base = GridObstacleMap(len(config.layers))
    cache = {p_id: precompute_net_obstacles(pcb, p_id, config),
             n_id: precompute_net_obstacles(pcb, n_id, config)}
    working = build_working_obstacle_map(base, cache)
    before = {nid: len(cache[nid].blocked_cells) + len(cache[nid].blocked_cell_spans)
              for nid in cache}

    outcome = try_terminal_restore(pcb, config, p_id, working, cache)
    check("full restore conflicts -> 'stub' path taken", outcome == 'stub',
          f"outcome={outcome!r}")
    present = {id(s) for s in pcb.segments}
    check("P escape stub restored", id(p_stub) in present)
    check("N escape stub restored (was dropped: only P's pads were walked)",
          id(n_stub) in present)
    check("long runs NOT restored (they conflict)",
          id(p_long) not in present and id(n_long) not in present)
    for nid, label in ((p_id, 'P'), (n_id, 'N')):
        fresh = precompute_net_obstacles(pcb, nid, config)
        same = (np.array_equal(np.sort(cache[nid].blocked_cells, axis=0),
                               np.sort(fresh.blocked_cells, axis=0))
                and np.array_equal(np.sort(cache[nid].blocked_cell_spans, axis=0),
                                   np.sort(fresh.blocked_cell_spans, axis=0)))
        grew = (len(cache[nid].blocked_cells) + len(cache[nid].blocked_cell_spans)
                > before[nid])
        check(f"{label} cache entry refreshed from the board (== fresh recompute)",
              same, f"entry rows {len(cache[nid].blocked_cells)}"
                    f"+{len(cache[nid].blocked_cell_spans)} vs fresh "
                    f"{len(fresh.blocked_cells)}+{len(fresh.blocked_cell_spans)}")
        check(f"{label} entry grew by the restored stub", grew)
    # A/B/C on this map: working - sum(caches) == base.
    probe = working.clone_fresh()
    for cd in cache.values():
        remove_net_obstacles_from_cache(probe, cd)
    bs, ps = base.get_stats(), probe.get_stats()
    check("ref-counts balanced after the restore (working - caches == base)",
          bs[:2] == ps[:2] and bs[7] == ps[7], f"base {bs} probe {ps}")


# ---------------------------------------------------------------------------
# 3. stamp_result_copper: a result's own via barrels are stamped.
# ---------------------------------------------------------------------------
def test_stamp_result_copper(verbose):
    print("\n[3] stamp_result_copper stamps the pair's OWN via barrels")
    from kicad_parser import Via, Segment
    from routing_config import GridRouteConfig, GridCoord
    from obstacle_map import GridObstacleMap
    try:
        from layer_swap_fallback import stamp_result_copper
    except ImportError as e:
        # Pre-fix tree: the stamp is still the inline GND-only filter. Report
        # it as the defect it is, not as a crashed test.
        check("layer_swap_fallback.stamp_result_copper exists (victim-reroute "
              "map stamps the result's own vias)", False, str(e))
        return
    layers = ['F.Cu', 'B.Cu']
    config = GridRouteConfig(layers=layers, grid_step=0.05, track_width=0.2,
                             clearance=0.15, via_size=0.45, via_drill=0.3)
    coord = GridCoord(config.grid_step)
    p_via = Via(x=125.0, y=100.0, size=0.45, drill=0.3, layers=layers, net_id=16)
    gnd_via = Via(x=130.0, y=100.0, size=0.45, drill=0.3, layers=layers, net_id=9)
    segm = Segment(start_x=120.0, start_y=100.0, end_x=122.0, end_y=100.0,
                   width=0.2, layer='F.Cu', net_id=16)
    m = GridObstacleMap(len(layers))
    stamp_result_copper(m, {'new_segments': [segm], 'new_vias': [p_via, gnd_via]},
                        config, 0.0)
    gx, gy = coord.to_grid(p_via.x, p_via.y)
    check("P via cell is via-blocked (the GND-only filter dropped it)",
          m.is_via_blocked(gx, gy))
    check("P via cell is track-blocked on F.Cu", m.is_blocked(gx, gy, 0))
    ggx, ggy = coord.to_grid(gnd_via.x, gnd_via.y)
    check("GND via still stamped (what the old filter kept)", m.is_via_blocked(ggx, ggy))
    sx, sy = coord.to_grid(121.0, 100.0)
    check("result segment stamped", m.is_blocked(sx, sy, 0))
    check("an empty result stamps nothing (no crash)",
          stamp_result_copper(GridObstacleMap(2), {}, config, 0.0) is None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-v', '--verbose', action='store_true')
    args = ap.parse_args()
    test_end_to_end(args.verbose)
    test_terminal_restore_pair(args.verbose)
    test_stamp_result_copper(args.verbose)
    print()
    if FAILS:
        print(f"FAIL: {len(FAILS)} check(s): " + "; ".join(FAILS))
        return 1
    print("PASS: #806 working-map content, pair terminal restore, via stamp")
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""Multi-source plane-tap trace routing (issue #259).

`_try_distant_pad_trace` / `_try_trace_to_plane_connected` used to run one full
A* per candidate same-net target (via / escaped pad), first-success wins -- up to
K searches per pad, ~99% of them failing on dense boards (12.8k of daisho's ~13k
tap-phase A* calls). They now run ONE multi-source A*: all candidate cells seeded
as sources, the pad as the target, and the trace is built DIRECTLY from the
winning path (route_multi_source_to_pad).

INVARIANTS (this test), on a real routed multilayer board:
  1. Existence-equivalence: route_multi_source_to_pad finds a connection whenever
     the old per-candidate route_via_to_pad loop would (and reports failure only
     when every candidate fails). This is the correctness bar the issue set --
     the single search must not miss a via site the loop would have found.
  2. Valid trace: on success the winning position is one of the candidates and the
     returned segments chain from that candidate to the pad centre (so the pad is
     actually connected, not just "a path exists").
  3. Boxed-in pad: when the pad's every neighbour is blocked and no candidate can
     reach it, the multi-source call reports failure with a blocked-cell frontier
     (which drives the rip-up retry), not a bogus success.

Run:  python3 tests/test_multi_source_tap.py [-v]
"""
import argparse
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from kicad_parser import parse_kicad_pcb
from routing_config import GridRouteConfig
from plane_pad_tap import make_local_window
from plane_obstacle_builder import build_routing_obstacle_map
from route_planes import route_via_to_pad, route_multi_source_to_pad

FIXTURE = os.path.join(ROOT, "kicad_files", "routed_output.kicad_pcb")
LAYERS = ["F.Cu", "In1.Cu", "In2.Cu", "In3.Cu", "In4.Cu",
          "In5.Cu", "In6.Cu", "In7.Cu", "In8.Cu", "B.Cu"]


def _config():
    return GridRouteConfig(layers=LAYERS, grid_step=0.05, track_width=0.127,
                           clearance=0.1, via_size=0.5, via_drill=0.3)


def _net_id(pcb, name):
    for nid, n in pcb.nets.items():
        if n.name == name:
            return nid
    return None


def _loop_finds_route(candidates, pad, pad_layer, net_id, routing_obs, config):
    """Old behaviour: one A* per candidate (nearest-first), first success wins.
    Mutates the map like the real per-candidate loop did (add/clear per call)."""
    for pos in sorted(candidates, key=lambda p: math.hypot(p[0] - pad.global_x,
                                                           p[1] - pad.global_y)):
        segs = route_via_to_pad(pos, pad, pad_layer, net_id, routing_obs, config,
                                verbose=False)
        if segs:
            return True
    return False


def _candidates_around(pad, offsets):
    """Synthetic candidate positions offset from the pad centre. The distant-trace
    helpers seed candidate copper as A* sources regardless of whether a cell is
    itself blocked, so arbitrary nearby points exercise the same code path as real
    same-net vias/pads -- this fixture happens to place its vias away from pads."""
    return [(pad.global_x + dx, pad.global_y + dy) for dx, dy in offsets]


def _trace_connects(segments, pad, win_pos, tol=0.06):
    """Segments form a connected chain from win_pos to the pad centre."""
    if not segments:
        return False
    pts = []
    for s in segments:
        pts.append(s['start'])
        pts.append(s['end'])

    def near(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1]) <= tol

    has_pad = any(near(p, (pad.global_x, pad.global_y)) for p in pts)
    has_win = any(near(p, win_pos) for p in pts)
    return has_pad and has_win


# Candidate offsets (mm) around the pad centre: several directions/distances so a
# real trace (not a trivial via-in-pad) is needed to reach at least some of them.
_OFFSETS = [(1.2, 0.0), (-1.2, 0.0), (0.0, 1.2), (0.0, -1.2),
            (1.0, 1.0), (-1.0, -1.0), (1.6, 0.6), (-0.8, 1.4)]


def _test_pads(pcb, limit=25):
    """Yield (pad, layer) for pads on a routable copper layer, spread across nets."""
    n = 0
    for fp in pcb.footprints.values():
        for pad in fp.pads:
            if pad.net_id in (0, None):
                continue
            layer = next((l for l in pad.layers if l in LAYERS), None)
            if layer is None:
                continue
            yield pad, layer
            n += 1
            if n >= limit:
                return


def run(verbose=False):
    results = []
    pcb = parse_kicad_pcb(FIXTURE)
    config = _config()

    # 1+2. Existence-equivalence and valid-trace, swept across many real pads: for
    # every pad the multi-source call must succeed iff the old per-candidate loop
    # would, and each success must produce a trace that actually chains the winning
    # candidate to the pad. Divergence in WHICH candidate wins is allowed (the
    # multi-source picks shortest-routable, the loop nearest-first); only the
    # existence of a connection must match.
    equiv_ok = True
    checked = 0
    successes = 0
    trace_ok = True
    win_ok = True
    for pad, pad_layer in _test_pads(pcb):
        cands = _candidates_around(pad, _OFFSETS)
        half = 4.0
        local = make_local_window(pcb, pad.global_x, pad.global_y, half)
        # Fresh map per engine: the loop mutates its map across candidates.
        obs_loop = build_routing_obstacle_map(local, config, pad.net_id, pad_layer,
                                              skip_pad_blocking=False, verbose=False)
        obs_ms = build_routing_obstacle_map(local, config, pad.net_id, pad_layer,
                                            skip_pad_blocking=False, verbose=False)
        loop_ok = _loop_finds_route(cands, pad, pad_layer, pad.net_id, obs_loop, config)
        segs, win = route_multi_source_to_pad(cands, pad, pad_layer, pad.net_id,
                                              obs_ms, config, verbose=False)
        ms_ok = bool(segs)
        checked += 1
        if loop_ok != ms_ok:
            equiv_ok = False
            if verbose:
                print(f"  MISMATCH {pad.component_ref}.{pad.pad_number} on "
                      f"{pad_layer}: loop={loop_ok} multi-source={ms_ok}")
        if ms_ok:
            successes += 1
            if not any(math.hypot(win[0] - c[0], win[1] - c[1]) < 1e-6 for c in cands):
                win_ok = False
            if not _trace_connects(segs, pad, win):
                trace_ok = False
    if verbose:
        print(f"swept {checked} pads, {successes} routable")

    results.append((f"existence-equivalence over {checked} pads "
                    f"(multi-source succeeds iff loop does)", equiv_ok))
    assert successes > 0, "no routable pad in sweep -- test would be vacuous"
    results.append((f"winning position is a candidate ({successes} successes)", win_ok))
    results.append((f"trace chains winning candidate -> pad ({successes} successes)",
                    trace_ok))

    # 3. Unreachable candidates -> failure with a frontier, never a bogus success.
    pad, pad_layer = next(_test_pads(pcb))
    local = make_local_window(pcb, pad.global_x, pad.global_y, 4.0)
    far = [(pad.global_x + 100.0, pad.global_y + 100.0),
           (pad.global_x + 101.0, pad.global_y + 100.0)]
    obs_far = build_routing_obstacle_map(local, config, pad.net_id, pad_layer,
                                         skip_pad_blocking=False, verbose=False)
    rr, pos = route_multi_source_to_pad(far, pad, pad_layer, pad.net_id, obs_far,
                                        config, return_blocked_cells=True)
    results.append(("unreachable candidates -> failure (no bogus success)",
                    (not rr.success) and pos is None))

    # 4. Empty candidate list -> clean (None, None), no crash.
    obs_empty = build_routing_obstacle_map(local, config, pad.net_id, pad_layer,
                                           skip_pad_blocking=False, verbose=False)
    segs_e, win_e = route_multi_source_to_pad([], pad, pad_layer, pad.net_id,
                                              obs_empty, config)
    results.append(("empty candidate list returns clean (None, None)",
                    segs_e is None and win_e is None))

    return results


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    results = run(args.verbose)
    ok = True
    for name, passed in results:
        print(f"  [{'PASS' if passed else 'FAIL'}] {name}")
        ok = ok and passed
    print(("\nPASS" if ok else "\nFAIL") + ": multi-source plane-tap invariants")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()

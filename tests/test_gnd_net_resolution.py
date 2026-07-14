#!/usr/bin/env python3
"""GND return-via lookup must recognize the GND family, not a literal 'GND'.

Issue #379: the diff-pair GND-via placement found its ground net with an exact
``net.name.upper() == 'GND'`` match, so any board whose ground is spelled '/GND'
(KiCad's hierarchical sheet-path form), 'GNDA', 'DGND', 'GND_D', ... never
matched, and ``--gnd-vias`` was silently disabled with no warning. This pins the
shared resolver (net_queries.resolve_gnd_net_id) used by both route_diff's
batch_route_diff_pairs and diff_pair_routing.route_diff_pair_with_obstacles.

    python3 tests/test_gnd_net_resolution.py
"""
import os
import sys
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from net_queries import (resolve_gnd_net_id, is_ground_net_name,
                         gnd_candidate_names)


def _pcb(names):
    # net 0 is the unconnected net, mirroring real PCBData
    return NS(nets={i: NS(net_id=i, name=n) for i, n in enumerate(names)})


def run():
    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)

    # --- resolve_gnd_net_id: the reported bug and its family ---
    def resolved(names, **kw):
        return resolve_gnd_net_id(_pcb(names), **kw)[1]

    check("literal /GND resolves (the reported bug)",
          resolved(['', '/GND', '/SIG']) == '/GND')
    check("hierarchical /sheet/GND resolves",
          resolved(['', '/pwr/GND', 'X']) == '/pwr/GND')
    check("exact GND beats GNDA when both present",
          resolved(['', 'GNDA', 'GND']) == 'GND')
    check("family-only board picks a GND-family net",
          resolved(['', 'DGND', 'VCC']) == 'DGND')
    check("multiple family nets -> deterministic shortest/lowest-id",
          resolved(['', 'GNDD', 'GNDA']) == 'GNDD')
    check("case-insensitive", resolved(['', 'gnd']) == 'gnd')
    check("no ground net -> None (so the caller can warn)",
          resolve_gnd_net_id(_pcb(['', 'VCC', 'SIG'])) == (None, None))
    check("net 0 is never ground even if named oddly",
          resolve_gnd_net_id(_pcb(['GND'])) == (None, None))

    # --- preferred_name (e.g. --gnd-via-net), sheet-path-insensitive ---
    check("preferred name selects a specific family member",
          resolved(['', 'GND', 'GND_D'], preferred_name='/GND_D') == 'GND_D')
    check("preferred name matches ignoring leading slash",
          resolved(['', '/GND'], preferred_name='GND') == '/GND')

    # --- is_ground_net_name ---
    for n in ['GND', '/GND', 'GNDA', 'AGND', 'DGND', 'PGND', 'GND_D', 'GND1',
              '/pwr/GND', 'gnd']:
        check(f"is_ground_net_name({n!r})", is_ground_net_name(n))
    for n in ['VCC', '+3V3', 'SIG_GNDCTRL_X', 'DDR_VREF']:
        # note SIG_GNDCTRL_X: 'GND' is not a prefix/suffix, must NOT match
        check(f"not ground: {n!r}", not is_ground_net_name(n))

    # --- gnd_candidate_names diagnostic ---
    check("candidates list is sorted GND-ish names",
          gnd_candidate_names(_pcb(['', '/GND', 'GNDA', 'VCC', 'DGND']))
          == ['/GND', 'DGND', 'GNDA'])

    print("=" * 60)
    if fails:
        for f in fails:
            print(f"  FAIL  {f}")
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("  PASS  GND family resolves (/GND, GNDA, DGND, ...); exact GND wins;")
    print("        preferred name honored; no-ground -> None; diagnostic sorted")
    return 0


if __name__ == '__main__':
    sys.exit(run())

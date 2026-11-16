#!/usr/bin/env python3
"""Plane joins and oracle direct stitches must honor same-net pad via clearance (#581).
Run: python tests/test_same_net_plane_via_clearance.py
Tiny in-memory board on upstream's #581 fixture, real Rust A*: with an active
clearance a via may not land in or beside a same-net SMD pad; with none, the
legacy in-pad via is kept.
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
for sub in ('py_router', 'py_tools', 'rust_router', 'tests'):
    sys.path.insert(0, str(ROOT / sub))
from test_581_same_net_pad_via_clearance import _pad, _pcb
from routing_config import GridRouteConfig
from plane_region_connector import build_base_obstacles, route_plane_connection_wide
from obstacle_map import same_net_pad_via_keepout_cells


def main():
    failures = []
    def check(name, ok):
        print(f"{'PASS' if ok else 'FAIL'}: {name}")
        if not ok:
            failures.append(name)

    pad = _pad(6, 6)
    back = _pad(6, 6)
    back.layers = ['B.Cu']
    pcb = _pcb([pad, back])
    for gap in (-1.0, 0.0, 0.1):
        cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], grid_step=0.05,
                              track_width=0.15, clearance=0.15,
                              via_size=0.45, via_drill=0.2,
                              same_net_pad_clearance=gap)
        obs, _ = build_base_obstacles({1}, cfg.layers, pcb, cfg, 0.15, 0.15)
        check(f'{gap}: pad centre via block', obs.is_via_blocked(120, 120) == (gap > 0))
        check(f'{gap}: pad remains track reachable', not obs.is_blocked(120, 120, 0))
        result, _ = route_plane_connection_wide(
            [(6, 6, 'F.Cu')], [(6, 6, 'B.Cu')], 0, cfg.layers, obs, cfg, [],
            max_iterations=10000)
        check(f'{gap}: layer-changing route succeeds', bool(result and result[1]))
        if result:
            vias = result[1]
            check(f'{gap}: one new via', len(vias) == 1)
            if gap > 0:
                # Independent continuous rectangle distance, not the map helper.
                check('via copper edge >= 0.1mm from pad', all(
                    (max(abs(x-6)-0.5, 0)**2 + max(abs(y-6)-0.5, 0)**2)**0.5
                    - cfg.via_size/2 >= gap-1e-9 for x, y in vias))
            else:
                check(f'{gap}: legacy in-pad via preserved', vias == [(6.0, 6.0)])
        if gap > 0:
            keep = same_net_pad_via_keepout_cells(pcb, 1, cfg)
            check('whole pad margin stamped', all(obs.is_via_blocked(int(x), int(y)) for x,y in keep))
            reused, _ = route_plane_connection_wide(
                [(6, 6, 'F.Cu')], [(6, 6, 'B.Cu')], 0, cfg.layers, obs, cfg, [(6, 6)],
                max_iterations=10000)
            check('existing barrel reused without new via', reused is not None and not reused[1])
            # The oracle's direct stitch bypasses A* and asks the same map.
            from kicad_oracle import _same_net_pad_via_blocked
            for x, want in ((6, True), (8, False)):
                check(f'direct stitch blocked at {x}: {want}',
                      _same_net_pad_via_blocked(cfg, obs, x, 6) == want)
        else:
            from kicad_oracle import _same_net_pad_via_blocked
            check(f'{gap}: direct stitch keeps the in-pad via',
                  not _same_net_pad_via_blocked(cfg, obs, 6, 6))

    print(f'{len(failures)} failure(s)')
    return bool(failures)

if __name__ == '__main__':
    sys.exit(main())

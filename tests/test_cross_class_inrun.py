#!/usr/bin/env python3
"""Cross-class clearance for IN-RUN (same-call) mixed-class copper (PR392 Task 1a/1b).

The base-map fix (73c84e0) priced PRE-PLACED obstacles pairwise at
max(routing-side floor, obstacle net's class). This suite covers the boundary it
left open: the per-net obstacle CACHE (obstacle_cache.precompute_net_obstacles)
and the incremental in-run stampers (obstacle_map.add/remove_*_list) must price
foreign in-run copper at that same pairwise clearance, ADD and REMOVE must stamp
IDENTICAL cells (ref-count symmetry, #208/#309), and the CLI must auto-read the
map from the board's netclasses while the output .kicad_pro stops clamping them.

    python3 tests/test_cross_class_inrun.py
"""
import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from kicad_parser import PCBData, BoardInfo, Via, Segment
from routing_config import GridRouteConfig, GridCoord
from obstacle_cache import precompute_net_obstacles, add_net_obstacles_from_cache
from obstacle_map import (add_segments_list_as_obstacles, remove_segments_list_from_obstacles,
                          add_vias_list_as_obstacles, remove_vias_list_from_obstacles,
                          add_net_vias_as_obstacles)

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'rust_router'))
from grid_router import GridObstacleMap

GRID = 0.1
LAYERS = ['F.Cu', 'B.Cu']
N_PWR, N_DEF = 1, 2

passed = failed = 0


def check(name, got, want):
    global passed, failed
    ok = got == want
    passed += ok
    failed += not ok
    print(f"  {'PASS' if ok else 'FAIL'}  {name}: {got} (want {want})")


def cfg(clearance=0.15, net_clearances=None, routed=(N_PWR, N_DEF)):
    c = GridRouteConfig(grid_step=GRID, clearance=clearance, via_size=0.6,
                        track_width=0.2, via_drill=0.3,
                        hole_to_hole_clearance=0.0, layers=LAYERS)
    c.set_net_clearances(net_clearances or {}, list(routed))
    return c


# ---------------------------------------------------------------------------
# (i) A Default net routing past an IN-RUN POWER_HI via is blocked at 0.25
#     (via the mixed-class per-net CACHE, precompute_net_obstacles).
# ---------------------------------------------------------------------------
def test_inrun_cache_blocks_at_class():
    print("(i) in-run POWER_HI via via the per-net cache (mixed-class):")
    coord = GridCoord(GRID)
    # POWER_HI via (size 0.8) added mid-run; a Default route via (0.6) at q is
    # 0.22195 edge-to-edge: LEGAL at 0.15 (needs 0.85 c-c), ILLEGAL at 0.25 (needs 0.95).
    v_pwr = Via(x=55.6, y=71.7, size=0.8, drill=0.4, layers=LAYERS, net_id=N_PWR)
    q = coord.to_grid(54.9, 71.1)
    pcb = PCBData(board_info=BoardInfo(layers={}, copper_layers=LAYERS, board_bounds=None),
                  nets={}, footprints={}, vias=[v_pwr], segments=[], pads_by_net={})

    # Inert (empty map): priced at config.clearance 0.15 -> q free.
    c0 = cfg(net_clearances={})
    m0 = GridObstacleMap(len(LAYERS))
    add_net_obstacles_from_cache(m0, precompute_net_obstacles(pcb, N_PWR, c0))
    check("empty map: q free at 0.15", m0.is_via_blocked(*q), False)

    # POWER_HI class 0.25 (routed floor is Default 0.15) -> obstacle priced 0.25 -> q blocked.
    c1 = cfg(net_clearances={N_PWR: 0.25}, routed=(N_DEF,))
    check("floor stays 0.15 (foreign class does not inflate it)", c1.net_clearance_floor, 0.15)
    m1 = GridObstacleMap(len(LAYERS))
    add_net_obstacles_from_cache(m1, precompute_net_obstacles(pcb, N_PWR, c1))
    check("POWER_HI 0.25 class: q BLOCKED (cache priced at 0.25)", m1.is_via_blocked(*q), True)

    # Same via through the incremental stamper add_net_vias_as_obstacles.
    m2 = GridObstacleMap(len(LAYERS))
    add_net_vias_as_obstacles(m2, pcb, N_PWR, c1)
    check("POWER_HI 0.25 class: q BLOCKED (incremental stamper)", m2.is_via_blocked(*q), True)
    m3 = GridObstacleMap(len(LAYERS))
    add_net_vias_as_obstacles(m3, pcb, N_PWR, c0)
    check("empty map incremental: q free at 0.15", m3.is_via_blocked(*q), False)


# ---------------------------------------------------------------------------
# (ii) ADD then REMOVE of multi-class in-run copper returns the obstacle map to
#      its EXACT baseline (zero net delta) -- ref-count symmetry (#208/#309).
# ---------------------------------------------------------------------------
def test_addremove_refcount_symmetry():
    print("(ii) multi-class in-run add/remove ref-count symmetry:")
    c = cfg(net_clearances={N_PWR: 0.25, N_DEF: 0.15}, routed=(N_PWR, N_DEF))
    # Copper of TWO different classes, stamped together and ripped together.
    segs = [
        Segment(start_x=10.0, start_y=10.0, end_x=12.0, end_y=10.0, width=0.2, layer='F.Cu', net_id=N_PWR),
        Segment(start_x=13.1, start_y=13.07, end_x=14.4, end_y=14.38, width=0.1, layer='B.Cu', net_id=N_DEF),
    ]
    vias = [
        Via(x=20.0, y=20.0, size=0.8, drill=0.4, layers=LAYERS, net_id=N_PWR),
        Via(x=22.123, y=22.077, size=0.4, drill=0.2, layers=LAYERS, net_id=N_DEF),
    ]
    m = GridObstacleMap(len(LAYERS))
    base = m.get_stats()[0], m.get_stats()[1]

    add_segments_list_as_obstacles(m, segs, c)
    add_vias_list_as_obstacles(m, vias, c)
    after_add = m.get_stats()[0], m.get_stats()[1]
    check("multi-class add stamps copper (cells>0, vias>0)",
          after_add[0] > 0 and after_add[1] > 0, True)

    remove_vias_list_from_obstacles(m, vias, c)
    remove_segments_list_from_obstacles(m, segs, c)
    after_rm = m.get_stats()[0], m.get_stats()[1]
    check("add->remove returns to baseline (cells,vias)", after_rm, base)

    # And the class-aware clearance genuinely enlarged the stamp vs a plain
    # config.clearance map (proves the pricing took effect, not a no-op).
    m_plain = GridObstacleMap(len(LAYERS))
    add_vias_list_as_obstacles(m_plain, vias, cfg(net_clearances={}))
    plain_cells = m_plain.get_stats()[0]
    m_prm = GridObstacleMap(len(LAYERS))
    add_vias_list_as_obstacles(m_prm, vias, c)
    priced_cells = m_prm.get_stats()[0]
    check("class-aware stamp >= plain stamp (POWER_HI reserves more)",
          priced_cells >= plain_cells and priced_cells > 0, True)
    remove_vias_list_from_obstacles(m_prm, vias, c)
    check("priced via add->remove back to zero", m_prm.get_stats()[0] + m_prm.get_stats()[1], 0)


# ---------------------------------------------------------------------------
# (iii) Auto-read builds net_clearances {net_id: clearance} from a .kicad_pro.
# ---------------------------------------------------------------------------
_PCB_STUB = """(kicad_pcb (version 20221018) (generator test)
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
  (net 0 "")
  (net 1 "/HS_D0")
  (net 2 "/SIG")
)
"""


def test_autoread_net_clearance_map():
    print("(iii) auto-read net_clearance_map from .kicad_pro netclasses:")
    from list_nets import net_clearance_map_by_id
    d = tempfile.mkdtemp(prefix="xclr_")
    pcb = os.path.join(d, "b.kicad_pcb")
    pro = os.path.join(d, "b.kicad_pro")
    with open(pcb, "w") as f:
        f.write(_PCB_STUB)
    proj = {
        "net_settings": {
            "meta": {"version": 3},
            "classes": [
                {"name": "Default", "clearance": 0.15},
                {"name": "HighSpeed", "clearance": 0.25},
            ],
            "netclass_assignments": {"/HS_D0": "HighSpeed"},  # /SIG is Default (unassigned)
        }
    }
    with open(pro, "w") as f:
        json.dump(proj, f)
    nets = {1: "/HS_D0", 2: "/SIG"}
    m = net_clearance_map_by_id(pcb, nets)
    check("HighSpeed net -> 0.25", m.get(1), 0.25)
    check("Default/unassigned net omitted (inert)", 2 in m, False)

    # All-Default board -> empty map (byte-identical guarantee).
    proj2 = {"net_settings": {"meta": {"version": 3},
                              "classes": [{"name": "Default", "clearance": 0.2}],
                              "netclass_assignments": {}}}
    with open(pro, "w") as f:
        json.dump(proj2, f)
    check("all-Default board -> empty map", net_clearance_map_by_id(pcb, nets), {})


# ---------------------------------------------------------------------------
# (iv) Output .kicad_pro RETAINS original non-Default netclass clearances (no clamp
#      by default); the Default class + min_clearance still equalize to the routed.
# ---------------------------------------------------------------------------
def test_output_keeps_netclasses():
    print("(iv) output .kicad_pro keeps original non-Default netclass clearances:")
    from fix_kicad_drc_settings import apply_targets_to_project, compute_targets, severity_plan

    def proj():
        return {"board": {"design_settings": {"rules": {"min_clearance": 0.2}, "rule_severities": {}}},
                "net_settings": {"meta": {"version": 0}, "classes": [
                    {"name": "Default", "clearance": 0.2, "track_width": 0.2},
                    {"name": "HighSpeed", "clearance": 0.25, "track_width": 0.15},
                ]}}

    targets = compute_targets(clearance=0.15, track_width=0.1)  # routed tighter than the classes
    plan = severity_plan()

    # Default (no-clamp, PR392): HighSpeed class clearance UNCHANGED.
    p = proj()
    apply_targets_to_project(p, targets, plan)
    hs = next(c for c in p["net_settings"]["classes"] if c["name"] == "HighSpeed")
    default_c = next(c for c in p["net_settings"]["classes"] if c["name"] == "Default")
    check("HighSpeed clearance preserved (no clamp)", hs["clearance"], 0.25)
    check("HighSpeed track_width preserved (no clamp)", hs["track_width"], 0.15)
    check("Default clearance still equalized to routed 0.15", default_c["clearance"], 0.15)

    # Opt-in clamp still works when explicitly requested (narrow-fallback path).
    p2 = proj()
    apply_targets_to_project(p2, targets, plan, clamp_nondefault_netclasses=True)
    hs2 = next(c for c in p2["net_settings"]["classes"] if c["name"] == "HighSpeed")
    check("clamp=True clamps HighSpeed down to 0.15", hs2["clearance"], 0.15)


def run():
    test_inrun_cache_blocks_at_class()
    test_addremove_refcount_symmetry()
    test_autoread_net_clearance_map()
    test_output_keeps_netclasses()
    print(f"\n{passed}/{passed + failed} cross-class in-run tests passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(run())

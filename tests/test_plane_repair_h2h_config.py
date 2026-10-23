#!/usr/bin/env python3
"""The plane repair engine and the KiCad oracle price drills at the run's
hole-to-hole, not the config dataclass's 0.20.

repair_planes() -- the engine route.py's #562 plane finalize runs -- takes
the run's hole_to_hole_clearance and handed it to its region joins and
sub-runs directly, but built its GridRouteConfig WITHOUT it, so the config
kept GridRouteConfig's default 0.20. Everything that reads
config.hole_to_hole_clearance -- the tap via maps (plane_obstacle_builder,
SharedViaMaps), the tap-graze via nudge (cleanup_plane_taps_grazing ->
nudge_grazing_vias) and merge_close_same_net_vias -- held vias 0.20 apart on
a 0.25 board. oracle_reconnect likewise took the run's value as an argument
while its callers' configs carried 0.20, which its #649b stitching via and
rescue escalation read.

Found on butterstick (set2, P_1.35_DRAM): the tap-graze nudge moved two tap
vias to 0.402 mm from a U11 ball's via -- legal at 0.20 + 0.2 drill, 0.048
inside the board's 0.25 (traced: nudge_grazing_vias <-
cleanup_plane_taps_grazing <- repair_planes <- batch_route's finalize).

Asserted (repair_planes on a small two-pad plane board, dry run, h2h 0.25):
  * the engine's config carries 0.25 (it reaches merge_close_same_net_vias);
  * oracle_reconnect's private config carries the argument it was given.

    python3 tests/test_plane_repair_h2h_config.py
"""
import contextlib
import io
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', os.path.join('tests', 'oracle'), 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

fails = []


def check(name, cond, detail=''):
    print(("  ok  " if cond else " FAIL ") + name + (f"  {detail}" if detail else ''))
    if not cond:
        fails.append(name)


# 1. repair_planes: the engine config's hole-to-hole reaches the merge.
import pcb_modification  # noqa: E402
import repair_planes  # noqa: E402
from constraint_agreement import write_board  # noqa: E402

seen = []
real_merge = pcb_modification.merge_close_same_net_vias


def spy_merge(vias, segs, pcb, h2h, *a, **k):
    seen.append(h2h)
    return real_merge(vias, segs, pcb, h2h, *a, **k)


pcb_modification.merge_close_same_net_vias = spy_merge
try:
    with tempfile.TemporaryDirectory() as td:
        b = os.path.join(td, 'b.kicad_pcb')
        write_board(b, segments=[(5, 10, 15, 10, 0.2, 'F.Cu', 1),
                                 (5, 12, 15, 12, 0.2, 'F.Cu', 2)])
        with contextlib.redirect_stdout(io.StringIO()):
            repair_planes.repair_planes(
                b, os.path.join(td, 'o.kicad_pcb'), ['A'], ['B.Cu'],
                clearance=0.2, dry_run=True, repair_pads=False,
                hole_to_hole_clearance=0.25)
finally:
    pcb_modification.merge_close_same_net_vias = real_merge
check("repair_planes reached its same-net via merge (the path under test ran)",
      bool(seen), f"{len(seen)} call(s)")
check("...at the run's hole-to-hole 0.25, not the config default 0.20",
      bool(seen) and all(abs(v - 0.25) < 1e-12 for v in seen), f"got {seen}")

# 2. oracle_reconnect: its private config carries the hole_to_hole argument.
import kicad_oracle  # noqa: E402
import kicad_exact_fill  # noqa: E402
import plane_region_connector  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402

cfgs = []
saved = (kicad_exact_fill.exact_unconnected, kicad_oracle.kicad_unconnected,
         plane_region_connector.build_base_obstacles)
calls = {'n': 0}


def fake_exact(board_file, net_names=None, pcb_data=None, verbose=False,
               project_from=None):
    calls['n'] += 1
    return [] if calls['n'] > 1 else [
        ('A', (5.0, 10.0, 'F.Cu', 'track'), (5.0, 12.0, 'F.Cu', 'track'))]


def spy_bbo(*a, **k):
    cfgs.append(k.get('config'))
    raise RuntimeError('stop after the map config is seen')


kicad_exact_fill.exact_unconnected = fake_exact
kicad_oracle.kicad_unconnected = lambda *a, **k: None
plane_region_connector.build_base_obstacles = spy_bbo
try:
    with tempfile.TemporaryDirectory() as td:
        b = os.path.join(td, 'b.kicad_pcb')
        write_board(b, segments=[(5, 10, 15, 10, 0.2, 'F.Cu', 1),
                                 (5, 12, 15, 12, 0.2, 'F.Cu', 1)])
        caller = GridRouteConfig(clearance=0.2, track_width=0.2, via_size=0.6,
                                 via_drill=0.3, layers=['F.Cu', 'B.Cu'])
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                kicad_oracle.oracle_reconnect(
                    b, ['A'], caller, track_via_clearance=0.2,
                    hole_to_hole_clearance=0.25, max_rounds=1)
        except RuntimeError:
            pass
finally:
    (kicad_exact_fill.exact_unconnected, kicad_oracle.kicad_unconnected,
     plane_region_connector.build_base_obstacles) = saved
check("oracle built a link map (the path under test ran)", bool(cfgs))
check("...from a config carrying the run's hole-to-hole 0.25",
      bool(cfgs) and abs(cfgs[0].hole_to_hole_clearance - 0.25) < 1e-12,
      f"got {cfgs[0].hole_to_hole_clearance if cfgs else None}")
check("the caller's config is untouched", caller.hole_to_hole_clearance == 0.2)

print("-" * 60)
if fails:
    print(f"{len(fails)} FAILED: " + "; ".join(fails))
    sys.exit(1)
print("ALL PASS")

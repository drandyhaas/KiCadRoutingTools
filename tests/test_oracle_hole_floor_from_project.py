#!/usr/bin/env python3
"""The KiCad-oracle leg prices NPTH holes at the floor the REAL project
declares, not at the flat fab floor of the board it happens to parse.

Every NPTH check of oracle_reconnect -- the link maps' track and via keep-outs
(build_base_obstacles), the width upgrade, #1169's leg widening and the
emission gate (wide_route_clear), the seed filter (npth_floor_ok) -- asks
obstacle_map.resolve_hole_clearance(pcb_data, config), which reads the
project beside pcb_data.source_path. The oracle parses `board_file`: on the
CLI the route step's half-written OUTPUT, which has no project beside it yet
(route.py writes it at the end), on the GUI a staged pcbnew save. Both read as
declaring nothing, so the whole leg held tracks at the flat 0.20 fab floor and
laid no #1038 via keep-out, whatever the board declared.

Found on neo6502 (set1, route.py GND step, plane finalize): a 0.4 mm GND link
leg at (168.4, 70.2-70.7) shipped 0.21 mm off USB-PWR1's NPTH wall against
the board's declared 0.25 (KiCad: hole clearance). Every caller already hands
the real project in as `project_from`; the oracle now resolves the floor from
it once, onto its private config.

The link source, the refill and the routers are stubbed (as in
test_1137_oracle_class_map), so no KiCad is needed. Cases:
  * project_from declares 0.25 -> the link map's config prices 0.25, and the
    emission gate refuses a 0.2 leg 0.22 off the wall that the flat floor
    accepts; the caller's config is untouched;
  * project_from declares nothing -> 0 (raise-only: the leg is unchanged);
  * an explicit config.hole_clearance wins over the project.

    python3 tests/test_oracle_hole_floor_from_project.py
"""
import contextlib
import io
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

fails = []


def check(name, cond, detail=''):
    print(("  ok  " if cond else " FAIL ") + name + (f"  {detail}" if detail else ''))
    if not cond:
        fails.append(name)


HOLE = (20.0, 20.0)
DRILL = 1.0                                  # wall at r = 0.5
LEG_X = HOLE[0] + DRILL / 2 + 0.22 + 0.1     # a 0.2 leg 0.22 off the wall


def _board(path):
    """A KiCad-10 board: net A's two track pieces (the link), an NPTH hole."""
    with open(path, 'w', encoding='utf-8') as f:
        f.write(f'''(kicad_pcb (version 20250114) (generator "pcbnew") (generator_version "10.0")
  (general (thickness 1.6))
  (paper "A4")
  (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (gr_rect (start 0 0) (end 40 40) (stroke (width 0.1) (type default)) (fill none) (layer "Edge.Cuts"))
  (footprint "T:P" (layer "F.Cu") (at 5 10)
    (property "Reference" "R1" (at 0 -1.5 0) (layer "F.SilkS"))
    (attr smd)
    (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net "A")))
  (footprint "T:MH" (layer "F.Cu") (at {HOLE[0]} {HOLE[1]})
    (property "Reference" "MH1" (at 0 -1.5 0) (layer "F.SilkS"))
    (pad "" np_thru_hole circle (at 0 0) (size {DRILL} {DRILL}) (drill {DRILL})
      (layers "*.Cu" "*.Mask")))
  (segment (start 5 10) (end 10 10) (width 0.2) (layer "F.Cu") (net "A"))
  (segment (start 10.6 10) (end 15 10) (width 0.2) (layer "F.Cu") (net "A"))
)
''')


def _project(board_path, rules):
    with open(os.path.splitext(board_path)[0] + '.kicad_pro', 'w',
              encoding='utf-8') as f:
        json.dump({'board': {'design_settings': {'rules': rules}}}, f)


def _run(declared, explicit=0.0):
    """One stubbed oracle round. The parsed board sits in out/ with NO
    project (the CLI's mid-run output); `project_from` is src/b.kicad_pcb,
    whose project declares `declared` (None: declares nothing). Returns the
    (pcb_data, config) the link's obstacle map was built from, and the
    caller's config."""
    import kicad_oracle
    import kicad_exact_fill
    import plane_region_connector
    import net_rescue
    from routing_config import GridRouteConfig
    seen = []
    calls = {'n': 0}

    def fake_exact(board_file, net_names=None, pcb_data=None,
                   verbose=False, project_from=None):
        calls['n'] += 1
        if calls['n'] > 1:
            return []
        return [('A', (10.0, 10.0, 'F.Cu', 'track'),
                 (10.6, 10.0, 'F.Cu', 'track'))]

    real_bbo = plane_region_connector.build_base_obstacles

    def spy_bbo(*a, **k):
        seen.append((k.get('pcb_data'), k.get('config')))
        return real_bbo(*a, **k)

    saved = (kicad_exact_fill.exact_unconnected,
             kicad_oracle.kicad_unconnected,
             kicad_exact_fill.refill_islands,
             plane_region_connector.build_base_obstacles,
             plane_region_connector.route_plane_connection_wide,
             net_rescue._attempt_edge, kicad_oracle._direct_sliver_weld)
    kicad_exact_fill.exact_unconnected = fake_exact
    kicad_oracle.kicad_unconnected = lambda *a, **k: None
    kicad_exact_fill.refill_islands = lambda *a, **k: None
    plane_region_connector.build_base_obstacles = spy_bbo
    plane_region_connector.route_plane_connection_wide = \
        lambda *a, **k: (None, 0)
    net_rescue._attempt_edge = lambda *a, **k: (None, None)
    kicad_oracle._direct_sliver_weld = lambda *a, **k: None
    try:
        with tempfile.TemporaryDirectory() as td:
            os.makedirs(os.path.join(td, 'src'))
            os.makedirs(os.path.join(td, 'out'))
            src = os.path.join(td, 'src', 'b.kicad_pcb')
            out = os.path.join(td, 'out', 'o.kicad_pcb')
            _board(src)
            _board(out)
            _project(src, {} if declared is None
                     else {'min_hole_clearance': declared})
            caller = GridRouteConfig(clearance=0.1, track_width=0.2,
                                     via_size=0.6, via_drill=0.3,
                                     layers=['F.Cu', 'B.Cu'])
            caller.hole_clearance = explicit
            with contextlib.redirect_stdout(io.StringIO()):
                kicad_oracle.oracle_reconnect(
                    out, ['A'], caller, track_via_clearance=0.2,
                    hole_to_hole_clearance=0.25, max_rounds=1,
                    project_from=src)
    finally:
        (kicad_exact_fill.exact_unconnected, kicad_oracle.kicad_unconnected,
         kicad_exact_fill.refill_islands,
         plane_region_connector.build_base_obstacles,
         plane_region_connector.route_plane_connection_wide,
         net_rescue._attempt_edge, kicad_oracle._direct_sliver_weld) = saved
    return seen, caller


def _gate(pcb, cfg):
    """The emission gate's verdict on a 0.2 leg 0.22 off the NPTH wall."""
    from plane_region_connector import wide_route_clear
    nid = next(i for i, n in pcb.nets.items() if n.name == 'A')
    leg = [(LEG_X, HOLE[1] - 2, 'F.Cu'), (LEG_X, HOLE[1] + 2, 'F.Cu')]
    return wide_route_clear(leg, 0.2, pcb, nid, cfg)


from obstacle_map import resolve_hole_clearance  # noqa: E402

# 1. the declared floor reaches the leg
seen, caller = _run(0.25)
check("the oracle built a link map", bool(seen))
if seen:
    pcb, cfg = seen[0]
    with contextlib.redirect_stdout(io.StringIO()):
        got = resolve_hole_clearance(pcb, cfg)
    check("project_from declares 0.25 -> the link prices 0.25", abs(got - 0.25) < 1e-12,
          f"got {got}")
    check("...and the emission gate refuses a leg 0.22 off the NPTH wall",
          _gate(pcb, cfg) is False)
    check("...which the flat 0.20 floor (the caller's config) accepts",
          _gate(pcb, caller) is True)
    check("the caller's config is untouched",
          (getattr(caller, 'hole_clearance', 0.0) or 0.0) == 0.0)

# 2. raise-only: a project declaring nothing leaves the leg as it was
seen, _c = _run(None)
if seen:
    pcb, cfg = seen[0]
    with contextlib.redirect_stdout(io.StringIO()):
        got = resolve_hole_clearance(pcb, cfg)
    check("project declares nothing -> 0 (the flat floor, unchanged)", got == 0.0,
          f"got {got}")
    check("...and the 0.22 leg is still accepted", _gate(pcb, cfg) is True)

# 3. an explicit floor wins
seen, _c = _run(0.25, explicit=0.3)
if seen:
    check("an explicit config.hole_clearance 0.3 wins over the project",
          abs(seen[0][1].hole_clearance - 0.3) < 1e-12,
          f"got {seen[0][1].hole_clearance}")

print("-" * 60)
if fails:
    print(f"{len(fails)} FAILED: " + "; ".join(fails))
    sys.exit(1)
print("ALL PASS")

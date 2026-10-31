#!/usr/bin/env python3
"""#1554: a net an earlier step had complete, ripped by this step, must not
ship broken without a retry.

Two defects let a step trade an earlier step's connected net away:

  1. A phase-3 tap rip-up re-routes its victim, and decided whether the
     victim is multi-point from its SAVED result's mst_edges. A pre-existing
     victim's saved result is the registration of copper from an earlier
     step, which never carries mst_edges, so a 3-pad victim was re-routed as
     a 2-terminal net with no taps: one pad shipped open while the log said
     "Re-routed main path".
  2. A reconciliation lap rips pre-existing nets with full authority, and a
     victim it left open was reported "reconciliation retries it" -- but the
     lap IS the reconciliation, so nothing did. The lap now hands those
     victims back and the next lap routes them.

Checks:
  1. _reroute_phase3_ripped_nets routes a ripped 3-pad pre-existing victim
     through route_multipoint_main with all three terminals; a 2-pad victim
     still goes through route_net_with_obstacles.
  2. batch_route's lap loop (CLI leg, then GUI leg): a lap that leaves a rip
     victim open is followed by another lap whose targets include it; a lap
     that leaves none ends the loop. The laps are scripted stand-ins, so the
     test exercises the loop rule, not the router.

    python3 tests/test_1554_rip_victims_retried.py
"""
import contextlib
import io
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import PCBData, BoardInfo, Net, Pad                 # noqa: E402
from routing_config import GridRouteConfig                            # noqa: E402
from routing_state import RoutingState                                # noqa: E402
import phase3_routing as P3                                           # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


# ---------------------------------------------------------------- check 1
def victim_board(n_pads):
    pcb = PCBData(
        board_info=BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                             copper_layers=['F.Cu', 'B.Cu'],
                             board_bounds=(0.0, 0.0, 50.0, 50.0)),
        nets={7: Net(net_id=7, name='V')},
        footprints={}, vias=[], segments=[], pads_by_net={}, zones=[])
    pcb.pads_by_net = {7: [
        Pad(pad_number=str(i + 1), net_id=7, net_name='V', global_x=x,
            global_y=10.0 + 3.0 * i, local_x=0.0, local_y=0.0, size_x=0.5,
            size_y=0.5, shape='circle', layers=['F.Cu'], drill=0.0,
            pad_type='smd', component_ref=f'U{i + 1}')
        for i, x in enumerate((10.0, 20.0, 30.0)[:n_pads])]}
    pcb._rip_saved = {}
    return pcb


def reroute_victim(n_pads):
    """Rip-reroute a victim whose saved result is a PRE-EXISTING
    registration (what route.py registers for earlier-step copper): no
    mst_edges. Its copper is already off the board (ripped)."""
    pcb = victim_board(n_pads)
    saved = {'net_id': 7, 'new_segments': [], 'new_vias': [],
             'iterations': 0, 'is_existing_route': True}
    cfg = GridRouteConfig(clearance=0.2, track_width=0.2, via_size=0.6,
                          via_drill=0.3, grid_step=0.1, layers=['F.Cu', 'B.Cu'],
                          max_rip_up_count=0)
    state = RoutingState(pcb_data=pcb, config=cfg)
    state.working_obstacles = None
    calls = []
    real = (P3.build_single_ended_obstacles, P3.route_net_with_obstacles,
            P3.route_multipoint_main)
    P3.build_single_ended_obstacles = lambda *a, **k: (None, None)
    P3.route_net_with_obstacles = (
        lambda *a, **k: calls.append(('two-terminal', None)) or {'failed': True})
    P3.route_multipoint_main = (
        lambda pcb_data, nid, config, obstacles, pads, **k:
        calls.append(('multipoint', len(pads))) or {'failed': True})
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            P3._reroute_phase3_ripped_nets(
                [(7, saved, [7], True)], pcb, cfg, state, [], [], [7], {},
                {}, {}, [], {}, {'F.Cu': 0, 'B.Cu': 1}, None, None)
    finally:
        (P3.build_single_ended_obstacles, P3.route_net_with_obstacles,
         P3.route_multipoint_main) = real
    return calls


print("1. a ripped pre-existing victim re-routes by its pad count")
calls = reroute_victim(3)
check("3-pad victim: multi-point main route over all three terminals",
      calls == [('multipoint', 3)], str(calls))
calls = reroute_victim(2)
check("2-pad victim: two-terminal route, as before",
      calls == [('two-terminal', None)], str(calls))


# ---------------------------------------------------------------- check 2
def boxed_board():
    """Net T's pad 1 is ringed by LOCKED foreign copper (never rippable) on
    the only routing layer, so the main pass fails T and the reconciliation
    runs -- the loop under test."""
    ring = [((9.0, 9.0), (11.0, 9.0)), ((11.0, 9.0), (11.0, 11.0)),
            ((11.0, 11.0), (9.0, 11.0)), ((9.0, 11.0), (9.0, 9.0))]
    segs = ''.join(
        f' (segment (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) (width 0.3) '
        f'(locked yes) (layer "F.Cu") (net 2) '
        f'(uuid "00000000-0000-0000-0000-00000000010{i}"))\n'
        for i, (a, b) in enumerate(ring))
    return (
        '(kicad_pcb (version 20240108) (generator test)\n'
        ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
        ' (net 0 "")\n (net 1 "T")\n (net 2 "W")\n'
        ' (gr_rect (start 0 0) (end 40 30) (layer "Edge.Cuts") (width 0.1))\n'
        ' (footprint "test:a" (layer "F.Cu") (at 10 10)\n'
        '  (property "Reference" "U1" (at 0 0) (layer "F.SilkS"))\n'
        '  (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net 1 "T"))\n'
        ' )\n'
        ' (footprint "test:b" (layer "F.Cu") (at 30 10)\n'
        '  (property "Reference" "U2" (at 0 0) (layer "F.SilkS"))\n'
        '  (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net 1 "T"))\n'
        ' )\n'
        ' (footprint "test:c" (layer "F.Cu") (at 20 25)\n'
        '  (property "Reference" "U3" (at 0 0) (layer "F.SilkS"))\n'
        '  (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net 2 "W"))\n'
        '  (pad "2" smd rect (at 4 0) (size 0.6 0.6) (layers "F.Cu") (net 2 "W"))\n'
        ' )\n' + segs + ')\n')


def run_laps(gui):
    """The outer batch_route is the real one; every nested call (the
    reconciliation laps) is a scripted stand-in: lap 1 recovers its target
    and leaves victim V open, later laps leave nothing."""
    import route
    real = route.batch_route
    laps = []

    def lap(input_file, output_file, net_names, **kw):
        laps.append(list(net_names))
        lap._victims_left_open = ['V'] if len(laps) == 1 else []
        return (1, 0, 0.0, {}) if kw.get('return_results') else (1, 0, 0.0)

    route.batch_route = lap
    try:
        with tempfile.TemporaryDirectory() as td:
            src = os.path.join(td, 'in.kicad_pcb')
            dst = os.path.join(td, 'out.kicad_pcb')
            with open(src, 'w') as f:
                f.write(boxed_board())
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                real(src, '' if gui else dst, ['T'], layers=['F.Cu'],
                     track_width=0.15, clearance=0.15, grid_step=0.1,
                     return_results=gui)
    finally:
        route.batch_route = real
    return laps, buf.getvalue()


for leg, gui in (('CLI', False), ('GUI', True)):
    print(f"2. {leg} leg: a lap's open victim is the next lap's target")
    laps, out = run_laps(gui)
    # Vacuity guard: the reconciliation must have run, or the loop under
    # test never did.
    check("the main pass left T failed and the reconciliation ran",
          'Final reconciliation: retrying' in out and laps[:1] == [['T']],
          str(laps))
    check("a second lap ran with the victim enrolled",
          len(laps) == 2 and 'V' in laps[1], str(laps))
    check("the enrolment is announced", 'enrolling them for one more lap' in out)

if failures:
    print(f"\nFAILED: {failures}")
    sys.exit(1)
print("\nALL PASSED")

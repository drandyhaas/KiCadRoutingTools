#!/usr/bin/env python3
"""#1156: a restored net is not re-ripped, and a failed phase-3 victim keeps
at least its escape stub.

glasgow, 10 nets in --nets, no --rip-existing-nets: /D5's phase-3 victim
retry ripped /IO_Banks/U5 and restored it; /D3's #85 abandon then re-ripped
the whole #354 rip tree -- U5 included, although it was back on its input
copper -- and U5's fresh reroute failed. Nothing restored it: phase 3 never
called the #468 terminal restore, so U5 shipped with 0 segments and 0 vias,
its BGA via-in-pad escape gone. (The control arm lost it the same way through
the plane-repair reconnect's refused custody restore.)

Checks:
  1. _sink_record reports the sinks it wrote a net into FIRST; a restore
     un-records it from exactly those, and only when the restore took. A net
     ripped earlier in a frame's window (its copper postdates the frame) stays.
  2. _reroute_phase3_ripped_nets, its reroute failing: the victim's escape
     stub comes back (the 'stub' verdict), it is still reported stranded, and
     the outcome is recorded in state.terminal_restores.
  3. Same, with the saved route conflict-free: the victim is restored whole
     and is no longer stranded.
  4. The blocker hint prescribes --rip-existing-nets only for nets the run
     could not already rip (it used to name nets the same run had ripped).

    python3 tests/test_1156_rip_custody.py
"""
import contextlib
import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import PCBData, BoardInfo, Net, Pad, Segment, Via   # noqa: E402
from routing_config import GridRouteConfig                            # noqa: E402
from routing_state import RoutingState                                # noqa: E402
import phase3_routing as P3                                           # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


print("1. rip-tree sinks")
parent, child = {}, {}
first = P3._sink_record((parent, child), [5], {'failed_pads_info': []})
check("first rip writes both sinks", 5 in parent and 5 in child and len(first) == 2)
P3._sink_unrecord(first, routed_results={5: {}})
check("restored -> out of both", 5 not in parent and 5 not in child)
parent = {7: []}
first = P3._sink_record((parent, child), [7], None)
P3._sink_unrecord(first, routed_results={7: {}})
check("a net already in the parent's window stays there", 7 in parent and 7 not in child)
parent, child = {}, {}
first = P3._sink_record((parent, child), [9], None)
P3._sink_unrecord(first, routed_results={})
check("a refused restore (net not routed) stays recorded", 9 in parent and 9 in child)


def board(blocked):
    pcb = PCBData(
        board_info=BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                             copper_layers=['F.Cu', 'B.Cu'],
                             board_bounds=(0.0, 0.0, 50.0, 50.0)),
        nets={7: Net(net_id=7, name='U5'), 8: Net(net_id=8, name='D3')},
        footprints={}, vias=[], segments=[], pads_by_net={}, zones=[])
    pads = [Pad(pad_number=str(i + 1), net_id=7, net_name='U5', global_x=x,
                global_y=10.0, local_x=0.0, local_y=0.0, size_x=0.5, size_y=0.5,
                shape='circle', layers=['F.Cu'], drill=0.0, pad_type='smd',
                component_ref='U30') for i, x in enumerate((10.0, 20.0))]
    pcb.pads_by_net = {7: pads}
    if blocked:      # copper routed since the rip, across the long run
        pcb.segments.append(Segment(start_x=15.0, start_y=9.0, end_x=15.0,
                                    end_y=11.0, width=0.2, layer='F.Cu', net_id=8))
    stub = Segment(start_x=10.0, start_y=10.0, end_x=10.4, end_y=10.0,
                   width=0.2, layer='F.Cu', net_id=7)
    via = Via(x=10.4, y=10.0, size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=7)
    run = Segment(start_x=10.4, start_y=10.0, end_x=20.0, end_y=10.0,
                  width=0.2, layer='F.Cu', net_id=7)
    saved = {'new_segments': [stub, run], 'new_vias': [via],
             'failed_pads_info': [], 'path': [(0, 0, 0)]}
    pcb._rip_saved = {7: (saved, [7], True)}
    return pcb, saved, stub, via


def reroute(blocked):
    pcb, saved, stub, via = board(blocked)
    cfg = GridRouteConfig(clearance=0.2, track_width=0.2, via_size=0.6,
                          via_drill=0.3, grid_step=0.1, layers=['F.Cu', 'B.Cu'],
                          max_rip_up_count=0)
    state = RoutingState(pcb_data=pcb, config=cfg)
    state.working_obstacles = None
    real_build, real_route = P3.build_single_ended_obstacles, P3.route_net_with_obstacles
    P3.build_single_ended_obstacles = lambda *a, **k: (None, None)
    P3.route_net_with_obstacles = lambda *a, **k: {'failed': True}
    routed_results, results = {}, []
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            stranded = P3._reroute_phase3_ripped_nets(
                [(7, saved, [7], True)], pcb, cfg, state, [], [], [7], {},
                routed_results, {}, results, {}, {'F.Cu': 0, 'B.Cu': 1}, None, None)
    finally:
        P3.build_single_ended_obstacles, P3.route_net_with_obstacles = real_build, real_route
    return pcb, state, stranded, routed_results, stub, via


print("2. a failed victim keeps its escape stub")
pcb, state, stranded, rr, stub, via = reroute(blocked=True)
check("the stub segment and its via are back on the board",
      any(s is stub for s in pcb.segments) and any(v is via for v in pcb.vias))
check("the blocked long run is not", not any(s.net_id == 7 and s.end_x == 20.0
                                             for s in pcb.segments))
check("still reported stranded (the #85 arbitration's input)",
      [i[0][0] for i in stranded] == [7], str([i[0][0] for i in stranded]))
check("recorded as a 'stub' terminal restore", state.terminal_restores.get(7) == 'stub',
      str(state.terminal_restores))

print("3. a clear corridor restores the victim whole")
pcb, state, stranded, rr, stub, via = reroute(blocked=False)
check("restored and routed, not stranded", 7 in rr and not stranded,
      str([i[0][0] for i in stranded]))
check("recorded as 'full'", state.terminal_restores.get(7) == 'full',
      str(state.terminal_restores))

print("4. the blocker hint prescribes the flag only for nets it would add")
import plane_blocker_detection as PBD                                 # noqa: E402
from routing_diagnostics import preexisting_blocker_hint              # noqa: E402
hp = PCBData(board_info=BoardInfo(layers={0: 'F.Cu'}, copper_layers=['F.Cu'],
                                  board_bounds=(0.0, 0.0, 10.0, 10.0)),
             nets={1: Net(1, 'TARGET'), 2: Net(2, 'AUTO'), 3: Net(3, 'RIPPED'),
                   4: Net(4, 'BIG')},
             footprints={}, vias=[], segments=[], pads_by_net={}, zones=[])
hp._rip_authority_ids = {2}
hp._preexisting_rips = {3: 'RIPPED'}
_queue = [2, 3, 4]
_real_fb = PBD.find_route_blocker_from_frontier
PBD.find_route_blocker_from_frontier = lambda *a, **k: _queue.pop(0) if _queue else None
try:
    text, names = preexisting_blocker_hint([(1, 1, 0)], GridRouteConfig(), hp, 1,
                                           return_names=True)
finally:
    PBD.find_route_blocker_from_frontier = _real_fb
check("the returned names (the #103 authority input) are unchanged",
      names == ['AUTO', 'RIPPED', 'BIG'], str(names))
check("the retry command names only the net outside the run's authority",
      "--rip-existing-nets 'BIG' " in text and "--rip-existing-nets 'AUTO'" not in text,
      text[:160])
check("the others are named as already rippable, one ripped",
      "'AUTO' 'RIPPED'" in text and "ripped 1" in text and "adds no authority" in text)

if failures:
    print(f"\nFAILED: {failures}")
    sys.exit(1)
print("\nALL PASSED")

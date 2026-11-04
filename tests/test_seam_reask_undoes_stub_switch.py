#!/usr/bin/env python3
"""A losing #444 seam re-ask takes back the stub layer switch its reroute made.

The re-ask rips a net's whole tree, reroutes it, and restores the original
when the new tree is no better. The reroute can run a stub layer switch
(route_multipoint_main's main-edge retry), and that switch is COMMITTED the
moment the retry routes: the moved stub's segment modifications and its pad
via go into the run's write channel (state.all_segment_modifications /
all_swap_vias). Restoring the old tree put the old copper back but left the
switch in the channel, so the board shipped the switch's pad via (0.3 mm,
sized for the pad the rip had freed) on top of the restored tree's own
via-in-pad (0.25 mm): two same-net barrels at one point, a hole-to-hole
violation, which the end-of-run dedup did not remove because the sizes
differ.

Rows, on a synthetic net whose tree is twice its MST bound, with the
switch made by the real apply_stub_layer_switch inside a stand-in reroute:

  1. the re-ask LOSES: the switch leaves the write channel, its pad via
     leaves the board, the stub is back on its own layer, the restored tree
     is the one routed, and an entry the channel held before the re-ask
     (another net's switch) is untouched;
  2. control, the re-ask WINS: the switch is kept with the tree it rescued.

    python3 tests/test_seam_reask_undoes_stub_switch.py
"""
import contextlib
import io
import os
import sys
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import phase3_routing  # noqa: E402
from kicad_parser import BoardInfo, Net, Pad, PCBData, Segment, Via  # noqa: E402
from pcb_modification import add_route_to_pcb_data  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402
from stub_layer_switching import StubInfo, apply_stub_layer_switch  # noqa: E402

FAILS = []
NET = 1


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name
          + (f'  {detail}' if detail else ''))


def setup():
    pads = [Pad('U1', '1', 0.0, 0.0, 0, 0, 0.5, 0.5, 'rect', ['F.Cu'], NET,
                'SIG', pad_type='smd'),
            Pad('U2', '1', 10.0, 0.0, 0, 0, 0.5, 0.5, 'rect', ['F.Cu'], NET,
                'SIG', pad_type='smd')]
    # The escape stub off U1's pad: pre-existing copper, in no result.
    stub = Segment(0.0, 0.0, -1.0, 0.0, 0.15, 'F.Cu', NET)
    pcb = PCBData(footprints={}, nets={NET: Net(NET, 'SIG')},
                  segments=[stub], vias=[],
                  board_info=BoardInfo(layers={},
                                       copper_layers=['F.Cu', 'B.Cu']),
                  pads_by_net={NET: pads})
    # The composed tree, 2x its 10 mm MST bound, with its via-in-pad at U1.
    old = {'new_segments': [Segment(0.0, 0.0, 0.0, 5.0, 0.15, 'B.Cu', NET),
                            Segment(0.0, 5.0, 10.0, 5.0, 0.15, 'B.Cu', NET),
                            Segment(10.0, 5.0, 10.0, 0.0, 0.15, 'B.Cu', NET)],
           'new_vias': [Via(0.0, 0.0, 0.25, 0.15, ['F.Cu', 'B.Cu'], NET),
                        Via(10.0, 0.0, 0.25, 0.15, ['F.Cu', 'B.Cu'], NET)],
           'is_multipoint': True, 'tap_edges_routed': 2, 'path': [(0, 0)]}
    add_route_to_pcb_data(pcb, old)
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.15,
                          clearance=0.15, via_size=0.3, via_drill=0.15)
    other = {'start': (50.0, 50.0), 'end': (51.0, 50.0), 'net_id': 9,
             'net_name': 'OTHER', 'old_layer': 'F.Cu', 'new_layer': 'B.Cu'}
    state = SimpleNamespace(
        working_obstacles=None, net_obstacles_cache={},
        ripped_route_layer_costs={}, ripped_route_via_positions={},
        collision_refused_net_ids=set(), net_history={}, route_index=0,
        all_segment_modifications=[other], all_swap_vias=[])
    return pcb, cfg, state, stub, old, other


def reroute_with_switch(new_len):
    """A stand-in for the re-ask's reroute: the main edge needs a stub layer
    switch (committed to the write channel as route_multipoint_main does),
    then the net routes a tree of `new_len` mm."""
    def _reroute(items, pcb, cfg, state, routed_net_ids, remaining_net_ids,
                 all_unrouted, routed_net_paths, routed_results, dp, results,
                 *a, **k):
        nid = items[0][0]
        stub = StubInfo(nid, -1.0, 0.0, 'F.Cu',
                        [s for s in pcb.segments if s.net_id == nid
                         and s.layer == 'F.Cu'], 0.0, 0.0, False)
        vias, mods = apply_stub_layer_switch(pcb, stub, 'B.Cu', cfg)
        state.all_segment_modifications.extend(mods)
        state.all_swap_vias.extend(vias)
        y = new_len / 2.0 - 5.0
        new = {'new_segments': [Segment(0.0, 0.0, 0.0, y, 0.15, 'B.Cu', nid),
                                Segment(0.0, y, 10.0, y, 0.15, 'B.Cu', nid),
                                Segment(10.0, y, 10.0, 0.0, 0.15, 'B.Cu',
                                        nid)],
               'new_vias': [], 'is_multipoint': True, 'tap_edges_routed': 2,
               'path': [(0, 0)]}
        add_route_to_pcb_data(pcb, new)
        routed_results[nid] = new
        results.append(new)
        routed_net_ids.append(nid)
        if nid in remaining_net_ids:
            remaining_net_ids.remove(nid)
    return _reroute


def reask(pcb, cfg, state, old, new_len):
    routed_results = {NET: old}
    results = [old]
    saved = phase3_routing._reroute_phase3_ripped_nets
    phase3_routing._reroute_phase3_ripped_nets = reroute_with_switch(new_len)
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            won = phase3_routing.seam_reask_one_net(
                NET, pcb, cfg, state, None, None, set(), [NET], [], {},
                routed_results, {}, results, {}, None)
    finally:
        phase3_routing._reroute_phase3_ripped_nets = saved
    return won, routed_results


def vias_at_pad(pcb):
    return sorted(v.size for v in pcb.vias
                  if v.net_id == NET and abs(v.x) < 1e-6 and abs(v.y) < 1e-6)


def main():
    print('1. the re-ask loses: its stub switch goes with its tree')
    pcb, cfg, state, stub, old, other = setup()
    won, routed_results = reask(pcb, cfg, state, old, 40.0)
    check('the re-ask lost (the stand-in tree is longer)', won is False)
    check('the switch left the write channel; the earlier entry stayed',
          state.all_swap_vias == []
          and state.all_segment_modifications == [other],
          f'swap vias {[(v.x, v.y, v.size) for v in state.all_swap_vias]}, '
          f'{len(state.all_segment_modifications)} modification(s)')
    check('one via at the pad: the restored tree\'s 0.25, not a 0.3 on it',
          vias_at_pad(pcb) == [0.25], f'sizes at the pad {vias_at_pad(pcb)}')
    check('the escape stub is back on F.Cu', stub.layer == 'F.Cu',
          f'stub on {stub.layer}')
    check('the restored tree is the one routed',
          routed_results.get(NET) is old)

    print('2. control: the re-ask wins, and keeps the switch it needed')
    pcb, cfg, state, stub, old, other = setup()
    won, routed_results = reask(pcb, cfg, state, old, 10.4)
    check('the re-ask won (the stand-in tree is shorter)', won is True)
    check('the switch stays in the write channel and on the board',
          len(state.all_swap_vias) == 1 and stub.layer == 'B.Cu'
          and vias_at_pad(pcb) == [0.3],
          f'{len(state.all_swap_vias)} swap via(s), stub on {stub.layer}, '
          f'sizes at the pad {vias_at_pad(pcb)}')

    print(f"\n{'ALL PASS' if not FAILS else f'{len(FAILS)} FAILED: ' + ', '.join(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())

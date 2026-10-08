"""Two latent defects the Phase 3 ripped-route ghosts exposed on esp_prog.

Passing the run's ghost ledgers to Phase 3's fast builder (8013c817) is
right -- the slow builder always had them -- but on esp_prog it made a plain
route.py ship a dangling tail and a dangling via. Neither came from the
ghosts themselves:

  - the #444 seam re-ask rips a net's OWN tree to re-ask it
    (`history_conflict=False`), and rip_up_net still recorded a ripped-route
    ghost: the re-ask's reroute paid its own old corridor -- the tree it was
    trying to improve on -- and, when the re-ask was undone and the restore
    refused, the rejected tree's corridor stayed priced for every later net;
  - net_rescue's #666 bare-ball escape lays a dogbone via before the gap
    routes; when the route that closes the net reaches the pad on the pad's
    own layer, the via joins nothing on its other layers (KiCad
    via_dangling), and no cleanup pass removes a via with no segment of its
    own.

Rows:
  - a contention rip records the ghost; an own-tree rip records none and
    drops one an earlier rip left, and both rip_up_net sites go through it;
  - an escape via that joins nothing across layers once the net is whole is
    withdrawn -- the via only: an offset dogbone's trace stays in the result
    for the dead-end sweep -- while a via the route runs through is kept, and
    so is every escape while the net is still open.

    python3 tests/test_own_tree_rip_and_escape.py
"""
import inspect
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import rip_up_reroute as rr  # noqa: E402
import net_rescue  # noqa: E402
from kicad_parser import BoardInfo, Net, Pad, PCBData, Segment, Via  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402


def test_own_tree_rip_leaves_no_ghost():
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.2,
                          ripped_route_avoidance_cost=0.1)
    seg = Segment(0.0, 0.0, 5.0, 0.0, 0.2, 'F.Cu', 7)
    result = {'new_segments': [seg], 'new_vias': [], 'path': []}
    layer_map = {'F.Cu': 0, 'B.Cu': 1}
    lc, vp = {}, {}
    rr._record_ripped_ghost(result, [7], cfg, layer_map, lc, vp, True)
    assert 7 in lc and len(lc[7]) > 0, "a contention rip records its ghost"
    # The net re-routes, and is later ripped by its own seam re-ask: the
    # stale ghost would come back to life, so it goes.
    rr._record_ripped_ghost(result, [7], cfg, layer_map, lc, vp, False)
    assert 7 not in lc and 7 not in vp, (lc, vp)
    lc2 = {}
    rr._record_ripped_ghost(result, [7], cfg, layer_map, lc2, {}, False)
    assert lc2 == {}, "an own-tree rip records nothing"
    src = inspect.getsource(rr.rip_up_net)
    assert src.count('_record_ripped_ghost(') == 2, src.count('_record_ripped_ghost(')
    assert 'compute_ripped_route_costs' not in src, "a ghost recorded around it"


def _board(route_on_b, offset=False):
    """Net 1: pad P1 (F.Cu SMD) at (0, 0) with an escape via -- in the pad,
    or (offset) a dogbone at (0, 1.5) with its F.Cu trace -- and pad P2
    (F.Cu) at (6, 0). Closed either on F.Cu straight between the pads (the
    via joins nothing across layers), or from the via on B.Cu to a via at P2
    (the route needs it)."""
    pads = [Pad('U1', str(i), x, 0.0, 0.0, 0.0, 0.6, 0.6, 'rect', ['F.Cu'],
                1, 'N', pad_type='smd') for i, x in ((1, 0.0), (2, 6.0))]
    ey = 1.5 if offset else 0.0
    esc = Via(0.0, ey, 0.4, 0.2, ['F.Cu', 'B.Cu'], 1)
    trace = [Segment(0.0, 0.0, 0.0, ey, 0.2, 'F.Cu', 1)] if offset else []
    if route_on_b:
        segs = trace + [Segment(0.0, ey, 6.0, 0.0, 0.2, 'B.Cu', 1)]
        vias = [esc, Via(6.0, 0.0, 0.4, 0.2, ['F.Cu', 'B.Cu'], 1)]
    else:
        segs = trace + [Segment(0.0, 0.0, 6.0, 0.0, 0.2, 'F.Cu', 1)]
        vias = [esc]
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'}, copper_layers=['F.Cu', 'B.Cu'])
    pcb = PCBData(board_info=bi, nets={1: Net(1, 'N')}, footprints={},
                  vias=vias, segments=segs, pads_by_net={1: pads})
    return pcb, {'new_segments': list(trace), 'new_vias': [esc]}


def test_unused_escape_is_withdrawn():
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'])
    pcb, tap = _board(route_on_b=False)
    assert net_rescue._net_component_info(pcb, 1)[0] == 1
    kept = net_rescue._withdraw_unused_escapes(pcb, 1, [tap], cfg)
    assert kept == [] and tap['new_vias'][0] not in pcb.vias, "dangling via shipped"
    assert net_rescue._net_component_info(pcb, 1)[0] == 1, "withdrawal split the net"
    pcb, tap = _board(route_on_b=True)
    kept = net_rescue._withdraw_unused_escapes(pcb, 1, [tap], cfg)
    assert kept == [tap] and tap['new_vias'][0] in pcb.vias, "a used escape went"
    # An offset dogbone: the via goes, its trace stays for the sweep.
    pcb, tap = _board(route_on_b=False, offset=True)
    kept = net_rescue._withdraw_unused_escapes(pcb, 1, [tap], cfg)
    assert tap['new_vias'][0] not in pcb.vias, "dangling dogbone via shipped"
    assert [r['new_segments'] for r in kept] == [tap['new_segments']], kept
    assert kept[0]['new_vias'] == [] and tap['new_segments'][0] in pcb.segments
    pcb, tap = _board(route_on_b=True, offset=True)
    kept = net_rescue._withdraw_unused_escapes(pcb, 1, [tap], cfg)
    assert kept == [tap] and tap['new_vias'][0] in pcb.vias, "a used dogbone went"
    src = inspect.getsource(net_rescue.rescue_failed_nets)
    assert 'if tap_results and num <= 1:' in src, \
        "escapes are withdrawn only once the net is whole"


TESTS = [test_own_tree_rip_leaves_no_ghost, test_unused_escape_is_withdrawn]


if __name__ == '__main__':
    fails = 0
    for t in TESTS:
        try:
            t()
            print(f"  PASS {t.__name__}")
        except AssertionError as e:
            fails += 1
            print(f"  FAIL {t.__name__}: {e}")
    print('ALL PASS' if not fails else f'{fails} FAILED')
    sys.exit(1 if fails else 0)

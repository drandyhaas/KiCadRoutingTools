#!/usr/bin/env python3
"""A route laid from a terminal-restored escape stub ships with that stub,
and only a route that needs the stub takes it.

When a rip victim's reroute fails and its saved route would short copper
routed since the rip, the terminal restore (#468, `try_terminal_restore`'s
'stub' verdict) puts back only the escape stubs at its pads, so a later route
of the net can start from them. Those stubs are copper this run laid, and
after the restore no result owns them. The board keeps only copper a result
owns: the copper sync (`sync_pcb_data_segments`) dropped them, and the
writer, which writes results, never wrote them. A net re-routed from its stub
was graded connected by every in-run check and then shipped open at the
stub: a diff pair ripped for a neighbour, stub-restored, re-routed by the
casualty reconcile and reported REROUTED, left one member's pad open.

The stub goes to the result whose commit makes the net need it (a pad
reached only through the stub), never simply to the next result of the net.
A result's copper is read as one connected tree: Phase 3 seeds a multipoint
net's taps from it, so a stub at a pad the main route has not reached, in
that route's result, would make the pad read as reached and its tap would
never be laid.

Rows:
  1. a route laid from the stub's end: after the sync the net is connected
     and every piece of its copper is in a result, where the writer writes it;
  2. a partial route that does not reach the stub does not take it; the
     commit that later reaches the pad through it does;
  3. a route that reaches the stub's pad by itself does not take the stub.

    python3 tests/test_1736_restored_stub_ships.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

from kicad_parser import PCBData, BoardInfo, Net, Pad, Segment, Via  # noqa: E402
from routing_config import GridRouteConfig                           # noqa: E402
from rip_restore import try_terminal_restore                         # noqa: E402
from pcb_modification import add_route_to_pcb_data                  # noqa: E402
from routing_common import sync_pcb_data_segments                   # noqa: E402
from check_connected import check_net_connectivity                  # noqa: E402

NET = 7


def _board():
    """Two pads on one net. The saved route is a stub off pad 1 (a segment
    and a via) and a long run that foreign copper routed since the rip now
    crosses, so only the stub can come back."""
    pcb = PCBData(
        board_info=BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                             copper_layers=['F.Cu', 'B.Cu'],
                             board_bounds=(0.0, 0.0, 50.0, 50.0)),
        nets={NET: Net(net_id=NET, name='NET7'), 8: Net(net_id=8, name='NET8')},
        footprints={}, vias=[], segments=[], pads_by_net={}, zones=[])
    pads = [Pad(pad_number=str(i + 1), net_id=NET, net_name='NET7',
                global_x=x, global_y=10.0, local_x=0.0, local_y=0.0,
                size_x=0.5, size_y=0.5, shape='circle', layers=['F.Cu'],
                drill=0.0, pad_type='smd', component_ref='U1')
            for i, x in enumerate((10.0, 20.0))]
    pcb.pads_by_net = {NET: pads}
    pcb.segments.append(Segment(start_x=15.0, start_y=9.0, end_x=15.0,
                                end_y=11.0, width=0.2, layer='F.Cu', net_id=8))
    stub = Segment(start_x=10.0, start_y=10.0, end_x=10.4, end_y=10.0,
                   width=0.2, layer='F.Cu', net_id=NET)
    stub_via = Via(x=10.4, y=10.0, size=0.6, drill=0.3,
                   layers=['F.Cu', 'B.Cu'], net_id=NET)
    long_seg = Segment(start_x=10.4, start_y=10.0, end_x=20.0, end_y=10.0,
                       width=0.2, layer='F.Cu', net_id=NET)
    pcb._rip_saved = {NET: ({'new_segments': [stub, long_seg],
                             'new_vias': [stub_via]}, [NET], True)}
    return pcb, stub, stub_via


def _connected(pcb):
    r = check_net_connectivity(
        NET, [s for s in pcb.segments if s.net_id == NET],
        [v for v in pcb.vias if v.net_id == NET], pcb.pads_by_net[NET], [],
        tolerance=0.02)
    return bool(r.get('connected')), r.get('disconnected_pads')


def _seg(x0, x1, layer, y=10.0):
    return Segment(start_x=x0, start_y=y, end_x=x1, end_y=y, width=0.2,
                   layer=layer, net_id=NET)


def _via(x):
    return Via(x=x, y=10.0, size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'],
               net_id=NET)


def _restored(cfg):
    """The fixture after the terminal restore, or None when it no longer
    reaches the 'stub' verdict (and so tests nothing)."""
    pcb, stub, stub_via = _board()
    ids = ({id(s) for s in pcb.segments}, {id(v) for v in pcb.vias})
    verdict = try_terminal_restore(pcb, cfg, NET)
    if verdict != 'stub':
        print(f"BROKEN TEST: the fixture no longer reaches the 'stub' verdict "
              f"(got {verdict!r}), so it tests nothing")
        return None
    return pcb, stub, stub_via, ids


def _owns(result, obj):
    return any(o is obj for k in ('new_segments', 'new_vias')
               for o in result.get(k) or ())


def row_routed_from_stub(cfg, failures):
    """Row 1: the net's later route starts from the stub's far end (its via)
    and runs under the foreign copper on B.Cu to pad 2, as a reroute does:
    its result carries only the copper it laid."""
    got = _restored(cfg)
    if got is None:
        return False
    pcb, stub, stub_via, (original_ids, original_via_ids) = got
    reroute = {'new_segments': [_seg(10.4, 19.4, 'B.Cu'),
                                _seg(19.4, 20.0, 'F.Cu')],
               'new_vias': [_via(19.4)]}
    add_route_to_pcb_data(pcb, reroute)
    ok, _ = _connected(pcb)
    if not ok:
        print("BROKEN TEST: the reroute does not connect the net even before "
              "the sync, so the sync cannot be what opens it")
        return False
    sync_pcb_data_segments(pcb, {NET: reroute}, original_ids,
                           original_via_ids=original_via_ids)
    ok, open_pads = _connected(pcb)
    if not ok:
        failures.append(f"1. after the sync the net is OPEN at {open_pads}: "
                        f"the restored stub it was routed from was dropped")
    if not _owns(reroute, stub):
        failures.append("1. the restored stub segment is in no result, so the "
                        "writer never writes it")
    if not _owns(reroute, stub_via):
        failures.append("1. the restored stub via is in no result, so the "
                        "writer never writes it")
    if sum(1 for s in pcb.segments if s is stub) != 1:
        failures.append("1. the stub segment is not on the board exactly once")
    return True


def row_partial_route_first(cfg, failures):
    """Row 2: a first commit of the net reaches pad 2 and stops short of the
    stub (as a multipoint net's main route reaches some pads before its taps
    reach the rest); a second commit closes the gap to the stub's via."""
    got = _restored(cfg)
    if got is None:
        return False
    pcb, stub, stub_via, _ids = got
    first = {'new_segments': [_seg(20.0, 19.4, 'F.Cu'),
                              _seg(19.4, 15.0, 'B.Cu')],
             'new_vias': [_via(19.4)]}
    add_route_to_pcb_data(pcb, first)
    if _owns(first, stub) or _owns(first, stub_via):
        failures.append("2. a route that does not reach the stub took it: "
                        "its copper no longer reads as one tree, and the "
                        "stub's pad reads as reached")
    tap = {'new_segments': [_seg(15.0, 10.4, 'B.Cu')], 'new_vias': []}
    add_route_to_pcb_data(pcb, tap)
    ok, open_pads = _connected(pcb)
    if not ok:
        print(f"BROKEN TEST: row 2's two commits do not connect the net "
              f"(open at {open_pads})")
        return False
    if not (_owns(tap, stub) and _owns(tap, stub_via)):
        failures.append("2. the commit that reaches the pad through the stub "
                        "did not take it, so the stub ships in no result")
    return True


def row_route_reaches_pad(cfg, failures):
    """Row 3: the net's later route reaches the stub's pad by itself; the
    stub is redundant, and the sync takes it off as it always has."""
    got = _restored(cfg)
    if got is None:
        return False
    pcb, stub, stub_via, (original_ids, original_via_ids) = got
    route = {'new_segments': [
        Segment(start_x=10.0, start_y=10.0, end_x=10.0, end_y=12.0, width=0.2,
                layer='F.Cu', net_id=NET),
        _seg(10.0, 20.0, 'F.Cu', y=12.0),
        Segment(start_x=20.0, start_y=12.0, end_x=20.0, end_y=10.0, width=0.2,
                layer='F.Cu', net_id=NET)], 'new_vias': []}
    add_route_to_pcb_data(pcb, route)
    if _owns(route, stub) or _owns(route, stub_via):
        failures.append("3. a route that reaches the stub's pad by itself "
                        "took the redundant stub")
    sync_pcb_data_segments(pcb, {NET: route}, original_ids,
                           original_via_ids=original_via_ids)
    ok, open_pads = _connected(pcb)
    if not ok:
        failures.append(f"3. after the sync the net is OPEN at {open_pads}")
    return True


def main():
    cfg = GridRouteConfig(clearance=0.2, track_width=0.2, via_size=0.6,
                          via_drill=0.3, grid_step=0.1, layers=['F.Cu', 'B.Cu'])
    failures = []
    for row in (row_routed_from_stub, row_partial_route_first,
                row_route_reaches_pad):
        if not row(cfg, failures):
            return 1
    if failures:
        print("FAIL:")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("a route laid from a restored stub keeps it on the board and in its "
          "result, and only a route that needs the stub takes it. OK")
    return 0

if __name__ == '__main__':
    sys.exit(main())

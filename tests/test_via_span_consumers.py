#!/usr/bin/env python3
"""Every consumer that asks "which layers does this via join" reads it by the
via's TYPE (found reviewing #1268).

KiCad loads a via with no blind/buried/micro word as a THROUGH via whatever its
`(layers ...)` pair says, and a typed via joins every layer BETWEEN its ends,
not just the two ends. These consumers read the stored pair instead:
check_connected, plane_obstacle_builder's pad reach, plane_region_connector's
cross-layer points and route_planes' tap necking joined or blocked only the two
end layers of any via without both F.Cu and B.Cu; check_weird, net_queries and
diff_pair_routing's launch and leg-arrival layers read the pair's interval (the
leg's as an INDEX range, wrong for a full-stack layer order), so an untyped via
whose pair is F.Cu/In1.Cu was a blind via to them; merge_close_same_net_vias
kept a set of the two ends; fab_notes put a via under paste only when its pair
named that side; and the barrel-length measurements (net_queries, impedance,
connectivity) measured the pair. Each row below is a Via the way the tool
builds one in-run -- an untyped through via whose pair is not F.Cu/B.Cu is
exactly what the back-side BGA fanout produced -- plus a typed one, with a
plain through via and a micro via that really does stop short as controls.

    python3 tests/test_via_span_consumers.py
"""
import os
import sys
from types import SimpleNamespace

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_tools'))  # #522

from kicad_parser import Pad, Segment, Via  # noqa: E402

CU = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
FAILS = []


def check(label, ok, detail=''):
    print(('PASS ' if ok else 'FAIL ') + label + (f'  [{detail}]' if not ok else ''))
    if not ok:
        FAILS.append(label)


def via(layers, via_type='through', x=10.0, y=5.0, net=1):
    return Via(x=x, y=y, size=0.4, drill=0.2, layers=list(layers), net_id=net,
               via_type=via_type)


UNTYPED = (['F.Cu', 'In1.Cu'], 'through')     # through to KiCad
BLIND = (['F.Cu', 'In2.Cu'], 'blind')         # joins In1.Cu too
MICRO = (['F.Cu', 'In1.Cu'], 'micro')         # stops at In1.Cu (control)
THROUGH = (['F.Cu', 'B.Cu'], 'through')       # control


def pad(x, layer, net=1, ref='P'):
    return Pad(component_ref=ref, pad_number='1', global_x=x, global_y=5.0,
               local_x=0.0, local_y=0.0, size_x=0.6, size_y=0.6, shape='rect',
               layers=[layer], net_id=net, net_name='A')


def test_check_connected():
    from check_connected import check_net_connectivity
    for (layers, vt), far_layer, want, what in (
            (UNTYPED, 'B.Cu', True, "an untyped F.Cu/In1.Cu via joins B.Cu (KiCad: through)"),
            (BLIND, 'In1.Cu', True, "a blind F.Cu..In2.Cu via joins In1.Cu, between its ends"),
            (MICRO, 'B.Cu', False, "a micro F.Cu..In1.Cu via does NOT join B.Cu (control)"),
            (THROUGH, 'B.Cu', True, "a through via joins B.Cu (control)")):
        segs = [Segment(start_x=5, start_y=5, end_x=10, end_y=5, width=0.2,
                        layer='F.Cu', net_id=1),
                Segment(start_x=10, start_y=5, end_x=15, end_y=5, width=0.2,
                        layer=far_layer, net_id=1)]
        r = check_net_connectivity(1, segs, [via(layers, vt)],
                                   [pad(5, 'F.Cu'), pad(15, far_layer, ref='Q')])
        check(f"check_connected: {what}", r['connected'] is want, r['connected'])


def test_check_weird():
    from check_weird import _via_span
    for (layers, vt), want, what in (
            (UNTYPED, set(CU), "untyped pair -> every layer"),
            (BLIND, {'F.Cu', 'In1.Cu', 'In2.Cu'}, "blind -> its run"),
            (MICRO, {'F.Cu', 'In1.Cu'}, "micro -> its run (control)")):
        got = _via_span(via(layers, vt), CU)
        check(f"check_weird._via_span: {what}", got == want, sorted(got))


def _two_layer_net(layers, vt, far_layer='B.Cu'):
    """Pad on F.Cu at x=5, a track to a via at x=10, a track on `far_layer` to
    a pad there at x=15."""
    p1, p2 = pad(5, 'F.Cu'), pad(15, far_layer, ref='Q')
    return SimpleNamespace(
        board_info=SimpleNamespace(copper_layers=CU),
        segments=[Segment(start_x=5, start_y=5, end_x=10, end_y=5, width=0.2,
                          layer='F.Cu', net_id=1),
                  Segment(start_x=10, start_y=5, end_x=15, end_y=5, width=0.2,
                          layer=far_layer, net_id=1)],
        vias=[via(layers, vt)], pads_by_net={1: [p1, p2]}), p1, p2


def test_plane_pad_reach():
    from plane_obstacle_builder import _smd_pad_reaches_layer
    for (layers, vt), target, want, what in (
            (UNTYPED, 'B.Cu', True, "an untyped via carries the pad to B.Cu"),
            (BLIND, 'In1.Cu', True, "a blind F.Cu..In2.Cu via carries it to In1.Cu"),
            (MICRO, 'B.Cu', False, "a microvia does not carry it to B.Cu (control)")):
        pcb, p1, _ = _two_layer_net(layers, vt)
        got = _smd_pad_reaches_layer(p1, target, 1, pcb)
        check(f"plane_obstacle_builder._smd_pad_reaches_layer: {what}",
              got is want, got)


def test_pin_pair_path():
    from net_queries import pin_pair_path_length
    for (layers, vt), want, what in (
            (UNTYPED, True, "a path crosses an untyped via to B.Cu"),
            (MICRO, False, "no path crosses a microvia to B.Cu (control)")):
        pcb, p1, p2 = _two_layer_net(layers, vt)
        got = pin_pair_path_length(pcb, 1, p1, p2)
        check(f"net_queries.pin_pair_path_length: {what}",
              (got is not None) is want, got)


def test_launch_layers():
    from diff_pair_routing import _endpoint_launch_layer_indices
    shuffled = ['F.Cu', 'B.Cu', 'In1.Cu', 'In2.Cu']   # --layers F.Cu B.Cu
    cfg = SimpleNamespace(layers=shuffled, grid_step=0.05)
    for (layers, vt), want, what in (
            (UNTYPED, set(shuffled), "an untyped pair launches on every layer"),
            (MICRO, {'F.Cu', 'In1.Cu'}, "a microvia only on its span (control)")):
        pcb = SimpleNamespace(board_info=SimpleNamespace(copper_layers=CU),
                              vias=[via(layers, vt)], pads_by_net={})
        got = {shuffled[i] for i in _endpoint_launch_layer_indices(
            pcb, 1, 10.0, 5.0, cfg, tol=0.01)}
        check(f"diff_pair launch layers: {what}", got == want, sorted(got))


def test_leg_target_layers():
    """The coupled-middle leg may arrive on any routable layer the middle's
    via reaches. It used to take the INDEX range between the via's two stored
    layers, which a `--layers F.Cu B.Cu In1.Cu` run's [F.Cu, B.Cu, In1.Cu,
    In2.Cu] (In2.Cu forbidden) cut short at B.Cu."""
    try:
        from diff_pair_routing import _via_routable_layer_indices
    except ImportError as e:
        check("diff_pair_routing has _via_routable_layer_indices (the leg's "
              "arrival layers were an index range over the stored pair)",
              False, e)
        return
    names = ['F.Cu', 'B.Cu', 'In1.Cu', 'In2.Cu']
    cfg = SimpleNamespace(routable_layers=lambda: ['F.Cu', 'B.Cu', 'In1.Cu'])
    for (layers, vt), want, what in (
            (THROUGH, [0, 1, 2], "a through via: every ROUTABLE layer, In1.Cu "
                                 "included, In2.Cu (forbidden) not"),
            (UNTYPED, [0, 1, 2], "an untyped pair: the same, it is through"),
            (MICRO, [0, 2], "a microvia: F.Cu and In1.Cu only (control)")):
        got = _via_routable_layer_indices(via(layers, vt), names, cfg)
        check(f"diff_pair leg arrival layers: {what}", got == want, got)


def test_merge_close_same_net_vias():
    from pcb_modification import merge_close_same_net_vias
    for (layers, vt), want, what in (
            (UNTYPED, 1, "a new through via merges into an untyped survivor (it is through)"),
            (MICRO, 0, "a new through via does NOT merge into a microvia (control)")):
        survivor = via(layers, vt)
        pcb = SimpleNamespace(board_info=SimpleNamespace(copper_layers=CU),
                              vias=[survivor], segments=[], pads_by_net={})
        new = [{'x': 10.05, 'y': 5.0, 'size': 0.4, 'drill': 0.2,
                'layers': ['F.Cu', 'B.Cu'], 'net_id': 1}]
        merged = merge_close_same_net_vias(new, [], pcb, 0.25, verbose=False)
        check(f"merge_close_same_net_vias: {what}", merged == want, merged)


def test_barrels():
    seen = []

    def barrel(a, b):
        seen.append((a, b))
        return 1.0
    pcb = SimpleNamespace(get_via_barrel_length=barrel, pads_by_net={})
    from net_queries import calculate_via_barrel_length
    seen.clear()
    calculate_via_barrel_length([via(*UNTYPED)], pcb)
    check("net_queries.calculate_via_barrel_length: an untyped via's barrel is "
          "F.Cu..B.Cu", seen == [('F.Cu', 'B.Cu')], seen)
    seen.clear()
    calculate_via_barrel_length([via(*MICRO)], pcb)
    check("net_queries.calculate_via_barrel_length: a microvia's barrel is its "
          "own span (control)", seen == [('F.Cu', 'In1.Cu')], seen)
    from connectivity import calculate_stub_via_barrel_length
    seen.clear()
    calculate_stub_via_barrel_length([via(*UNTYPED)], 'F.Cu', pcb)
    check("connectivity.calculate_stub_via_barrel_length: an untyped via's "
          "other end is B.Cu", seen == [('F.Cu', 'B.Cu')], seen)
    import impedance
    seen.clear()
    real = impedance.get_via_barrel_epsilon_eff
    impedance.get_via_barrel_epsilon_eff = lambda p, a, b: 4.0
    try:
        impedance.calculate_route_propagation_time_ps([], [via(*UNTYPED)], pcb)
    finally:
        impedance.get_via_barrel_epsilon_eff = real
    check("impedance: an untyped via's barrel delay is F.Cu..B.Cu",
          seen == [('F.Cu', 'B.Cu')], seen)


def test_via_paste_sites():
    from fab_notes import via_paste_sites
    import paste_apertures as pa
    ap = SimpleNamespace(layer='F.Paste', bounds=(9.0, 4.0, 11.0, 6.0))
    real_for, real_pen = pa.apertures_for_net, pa.via_paste_penetration
    pa.apertures_for_net = lambda pcb, nid: [ap]
    pa.via_paste_penetration = lambda x, y, s, a: 0.2
    try:
        for (layers, vt), want, what in (
                ((['In1.Cu', 'In2.Cu'], 'through'), 1,
                 "an untyped In1/In2 pair is through, so it reaches F.Paste"),
                ((['In1.Cu', 'In2.Cu'], 'buried'), 0,
                 "a buried In1..In2 via does not (control)")):
            got = via_paste_sites([via(layers, vt)], SimpleNamespace())
            check(f"fab_notes.via_paste_sites: {what}", len(got) == want, len(got))
    finally:
        pa.apertures_for_net, pa.via_paste_penetration = real_for, real_pen


def test_plane_region_connector():
    """The plane repair's cross-layer points took a via's two ENDS, so a
    buried via did not join the layer between them."""
    from plane_region_connector import _collect_cross_layer_points
    for (layers, vt), want, what in (
            (BLIND, {'F.Cu', 'In1.Cu', 'In2.Cu'}, "a blind F.Cu..In2.Cu via joins In1.Cu"),
            (UNTYPED, set(CU), "an untyped pair joins every layer"),
            (MICRO, {'F.Cu', 'In1.Cu'}, "a microvia joins its two layers (control)")):
        pcb = SimpleNamespace(vias=[via(layers, vt)], pads_by_net={})
        got = _collect_cross_layer_points(1, pcb, CU)
        check(f"plane_region_connector cross-layer points: {what}",
              len(got) == 1 and got[0][2] == want, got and sorted(got[0][2]))


def test_neck_plane_segments():
    """A plane tap on In1.Cu beside a FOREIGN blind F.Cu..In2.Cu via must neck
    clear of it: the via has copper on In1.Cu. The neck pass took only the
    via's two end layers."""
    from route_planes import _neck_plane_segments
    for (layers, vt), want_neck, what in (
            (BLIND, True, "a blind F.Cu..In2.Cu via has copper on In1.Cu: neck"),
            (MICRO, False, "a microvia has none on In2.Cu: no neck (control)")):
        pcb = SimpleNamespace(vias=[via(layers, vt, net=2)], pads_by_net={})
        layer = 'In1.Cu' if want_neck else 'In2.Cu'
        seg = {'start': (5.0, 5.4), 'end': (15.0, 5.4), 'width': 0.5,
               'layer': layer, 'net_id': 1}
        _neck_plane_segments([seg], pcb, 0.2, CU)
        check(f"route_planes._neck_plane_segments: {what}",
              (seg['width'] < 0.5) is want_neck, seg['width'])


def main():
    test_check_connected()
    test_check_weird()
    test_plane_pad_reach()
    test_pin_pair_path()
    test_launch_layers()
    test_leg_target_layers()
    test_merge_close_same_net_vias()
    test_barrels()
    test_via_paste_sites()
    test_plane_region_connector()
    test_neck_plane_segments()
    if FAILS:
        print(f"\n{len(FAILS)} FAILED")
        return 1
    print("\nall passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())

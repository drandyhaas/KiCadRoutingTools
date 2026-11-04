"""ZoneFillModel must stamp pads at max(zone clearance, pad local override)
(audit H2, the live per-object term): KiCad's filler honors a resolved
per-pad clearance override (#326), so a fiducial keep-clear ring carves a
bigger fill hole than the flat zone clearance -- a model stamping flat zc
over-credits fill the refill actually removes.
"""
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.join(os.path.dirname(__file__), '..'), 'py_router'))  # #522
sys.path.insert(0, os.path.join(os.path.join(os.path.dirname(__file__), '..'), 'py_tools'))  # #522

from plane_fill_model import ZoneFillModel  # noqa: E402


def make_pcb(local_clearance):
    pad = SimpleNamespace(net_id=2, global_x=5.0, global_y=5.0,
                          size_x=1.0, size_y=1.0, shape='rect',
                          rect_rotation=0.0, layers=['F.Cu'], drill=0,
                          pad_type='smd', local_clearance=local_clearance)
    return SimpleNamespace(
        vias=[], segments=[], pads_by_net={2: [pad]},
        board_info=SimpleNamespace(keepouts=[]))


def make_zone():
    return SimpleNamespace(net_id=1, layer='F.Cu', clearance=0.2,
                           min_thickness=0.2,
                           polygon=[(0.0, 0.0), (10.0, 0.0),
                                    (10.0, 10.0), (0.0, 10.0)])


def main():
    # Probe 0.5mm from the pad edge (x = 6.0; pad edge at 5.5).
    # Plain pad (no override): 0.5 > zc 0.2 -> fill exists there.
    # query_component: truthy label = fill, 0 = carved, None = off-grid.
    m = ZoneFillModel(make_pcb(0.0), make_zone())
    assert m.ok
    assert m.query_component(6.0, 5.0), "fill missing outside plain zc guard"

    # 0.8mm override: KiCad carves to 0.8 -> no fill at 0.5mm from the edge,
    # while fill beyond the override ring still exists.
    m = ZoneFillModel(make_pcb(0.8), make_zone())
    assert m.ok
    assert not m.query_component(6.0, 5.0), \
        "pad local_clearance not honored by fill model"
    assert m.query_component(7.0, 5.0), \
        "fill missing outside the override ring"

    # Far corner unaffected either way (guard restored after the pad loop).
    assert m.query_component(1.0, 1.0)

    test_cutouts()
    print("PASS: fill model honors pad local clearance override (H2) "
          "+ board cutouts (#490)")


def test_cutouts():
    """Board cutouts are keep-outs (crkbd #490): KiCad's filler pulls fill
    out of Edge.Cuts openings; the model predicted fill straight through
    them, and the fill-aware via preference steered taps onto the phantom.
    The band is the board's edge clearance, not the zone's (#1643: probed on
    kicad-cli 10.0.6, a cutout is kept the edge rule away, and with no rule
    the fill runs to it)."""
    zone = make_zone()
    pcb = make_pcb(0.0)
    pcb.board_info.board_cutouts = [[(4.0, 4.0), (6.0, 4.0),
                                     (6.0, 6.0), (4.0, 6.0)]]
    pcb.live_rules_provider = lambda: {'min_copper_edge_clearance': 0.2}
    m = ZoneFillModel(pcb, zone)
    assert m.ok
    assert not m.query_component(5.0, 5.0), "fill inside cutout"
    assert not m.query_component(6.1, 5.0), "fill within the edge rule of the ring"
    assert m.query_component(7.5, 5.0), "fill missing outside band"
    pcb.live_rules_provider = lambda: {'min_copper_edge_clearance': 0.0}
    m = ZoneFillModel(pcb, zone)
    assert m.ok
    assert not m.query_component(5.0, 5.0), "fill inside cutout"
    assert m.query_component(6.1, 5.0), \
        "no edge rule: the fill must reach the ring, not stop at the zone clearance"


if __name__ == '__main__':
    main()

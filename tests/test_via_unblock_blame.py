"""#331 item 3: a via-in-pad unblock that declines must NAME the net whose
copper boxes the pad (pcb_data._via_unblock_blame), so the rip-up ladder can
rip the keystone instead of frontier-adjacent bystanders (#189, ottercast
SDC0_D3: an In1 trace directly under the ball that the A* frontier never
reaches).
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import BoardInfo
from routing_config import GridCoord, GridRouteConfig
from synth import make_net, make_pad, make_pcb, make_seg

from single_ended_routing import _place_shrunk_via_in_pad

VICTIM, KEYSTONE = 1, 2
LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']


def _board():
    bi = BoardInfo(layers={i: l for i, l in enumerate(LAYERS)},
                   copper_layers=list(LAYERS),
                   board_bounds=(-2.0, -2.0, 2.0, 2.0))
    pad = make_pad(VICTIM, 0.0, 0.0, ref='U1', num='A1', net_name='VICTIM',
                   size_x=0.4, size_y=0.4)
    # Keystone trace directly under the pad on In1: every fab-floor via rung
    # grazes it, so the via-in-pad unblock must decline - and name KEYSTONE.
    trace = make_seg(-2.0, 0.0, 2.0, 0.0, layer='In1.Cu', net_id=KEYSTONE,
                     width=0.2)
    return make_pcb(
        nets={VICTIM: make_net(VICTIM, 'VICTIM'),
              KEYSTONE: make_net(KEYSTONE, 'KEYSTONE')},
        segments=[trace], pads_by_net={VICTIM: [pad], KEYSTONE: []},
        board_info=bi), pad


def _cfg():
    c = GridRouteConfig()
    c.layers = list(LAYERS)
    c.grid_step = 0.05
    c.clearance = 0.1
    c.track_width = 0.127
    c.via_size = 0.5
    c.via_drill = 0.3
    return c


def test_declined_via_in_pad_names_the_keystone():
    pcb, pad = _board()
    cfg = _cfg()
    res = _place_shrunk_via_in_pad(pad, None, cfg, pcb, VICTIM,
                                   GridCoord(cfg.grid_step), LAYERS)
    assert res is None, "no via can legally fit over the KEYSTONE trace"
    blame = getattr(pcb, '_via_unblock_blame', None)
    assert blame and blame.get(VICTIM) == {KEYSTONE}, \
        f"decline must blame KEYSTONE, got {blame}"
    # And the (net, pad) failure is memoised as before.
    assert (VICTIM, 0.0, 0.0) in pcb._via_unblock_failed


def test_clear_pad_records_no_blame():
    pcb, pad = _board()
    pcb.segments = []  # nothing under the pad: the via places, no blame
    cfg = _cfg()
    res = _place_shrunk_via_in_pad(pad, None, cfg, pcb, VICTIM,
                                   GridCoord(cfg.grid_step), LAYERS)
    assert res is not None, "an unobstructed pad must accept a via"
    assert not getattr(pcb, '_via_unblock_blame', None)


def main():
    fns = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for fn in fns:
        print(f"  {fn.__name__} ...", end=" ")
        fn()
        print("OK")
    print(f"{len(fns)} test(s) passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())

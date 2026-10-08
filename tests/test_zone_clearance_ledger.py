"""Zone-clearance escalation must reach the clearance ledger (audit A1).

A pour whose clearance escalates below nominal (to thread a dense BGA via
lattice), or an explicit tight --zone-clearance, is written into the zone --
but a KiCad refill regrows the pour at max(zone clearance, netclass). Unless
the resolved pour clearance is ledger-recorded, the .kicad_pro writeback
keeps the nominal floor and the refill seals the corridors the escalation
opened: the router's verified fill and KiCad's refilled board diverge
(incomplete plane nets that only appear after a refill).
"""
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.join(os.path.dirname(__file__), '..'), 'py_router'))  # #522
sys.path.insert(0, os.path.join(os.path.join(os.path.dirname(__file__), '..'), 'py_tools'))  # #522

import clearance_ledger  # noqa: E402
from route_planes import _resolve_zone_clearance  # noqa: E402


def bga_board(via_size_in_field=0.3, pitch=0.65):
    """4-layer board with one BGA (grid pads at `pitch`) whose field holds
    plane-net vias of `via_size_in_field` -> lattice gap = pitch - via."""
    pads = [SimpleNamespace(global_x=10 + i * pitch, global_y=10 + j * pitch)
            for i in range(4) for j in range(4)]
    fp = SimpleNamespace(footprint_name='pkg:BGA-16', pads=pads,
                         reference='U1')
    vias = [SimpleNamespace(x=10 + pitch, y=10 + pitch,
                            size=via_size_in_field, net_id=2)]
    return SimpleNamespace(
        footprints={'U1': fp},
        nets={2: SimpleNamespace(name='GND')},
        vias=vias,
        board_info=SimpleNamespace(
            copper_layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']))


def main():
    # 1. Escalation: gap 0.65-0.25=0.40 less the 0.15 min width and the web's
    #    margin (plane_lattice.WEB_MARGIN: a neck exactly at the min width is
    #    the fill's own rounding to keep or drop), halved: 0.12 -- above the
    #    0.1 4-layer fab floor. Resolves to it AND drops the ledger floor with it.
    import plane_lattice
    want = plane_lattice.clearance_to_thread(0.65, 0.25, 0.15)
    assert abs(want - 0.12) < 1e-6, want
    clearance_ledger.reset()
    zc = _resolve_zone_clearance(None, 0.2, 0.15,
                                 bga_board(via_size_in_field=0.25), 0.4,
                                 ['GND'])
    assert abs(zc - want) < 1e-6, zc
    assert abs(clearance_ledger.effective(0.2) - want) < 1e-6, \
        clearance_ledger.get_min()

    # 2. Explicit tight --zone-clearance (no BGA pressure): honored, and the
    #    ledger floor still drops so refill preserves it.
    clearance_ledger.reset()
    board = bga_board()
    board.vias = []  # no plane-net via in the field -> no escalation path
    zc = _resolve_zone_clearance(0.12, 0.2, 0.15, board, 0.4, ['GND'])
    assert abs(zc - 0.12) < 1e-9, zc
    assert abs(clearance_ledger.effective(0.2) - 0.12) < 1e-9

    # 3. Needed below the fab floor: escalation refuses, clearance stays
    #    nominal, ledger floor unchanged (no phantom sub-nominal grading).
    clearance_ledger.reset()
    zc = _resolve_zone_clearance(None, 0.2, 0.15,
                                 bga_board(via_size_in_field=0.4), 0.4,
                                 ['GND'])
    assert abs(zc - 0.2) < 1e-9, zc
    assert abs(clearance_ledger.effective(0.2) - 0.2) < 1e-9

    print("PASS: zone-clearance escalation reaches the ledger (A1)")


if __name__ == '__main__':
    main()

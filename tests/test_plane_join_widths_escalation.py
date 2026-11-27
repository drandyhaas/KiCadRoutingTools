"""Plane region joins obey --escalation per net.

Before: repair_planes (and route.py's in-run plane finalize) joined plane
regions from the 0.2 mm REPAIR_MIN_TRACK_WIDTH up, and its last-resort bridge
(#217) at the run's --track-width, whatever the policy. Under
`--escalation off` a GND plane asked at 0.5 mm shipped 0.2-0.3 mm straps.
Now each plane net gets its own range: under `off` nothing
narrower than that net's width (two nets asking x and y each get theirs),
under `board` nothing below that net's rule floor, under `fab` unchanged.
"""
import os
import shutil
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'py_router'))

import fab_tiers  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from repair_planes import _plane_join_width_resolver  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402


def test_resolver_takes_the_strictest_layer():
    # A join can route on every layer it is given and draws one width, so a
    # net 0.3 mm on F.Cu and 0.6 mm on B.Cu joins at 0.6 when B.Cu is open.
    cfg = GridRouteConfig(track_width=0.3, layers=['F.Cu', 'B.Cu'])
    prev = fab_tiers.get_escalation_policy()
    try:
        fab_tiers.set_escalation_policy('off')
        resolve = _plane_join_width_resolver(
            cfg, None, '', [1], {}, {1: {'F.Cu': 0.3, 'B.Cu': 0.6}}, 0.2, 2.0)
        assert resolve(1, ['F.Cu']) == (0.3, 2.0, 0.3)
        assert resolve(1, ['F.Cu', 'B.Cu']) == (0.6, 2.0, 0.6)
    finally:
        fab_tiers.set_escalation_policy(*prev)


def test_resolver_reads_power_and_class_widths_per_net():
    cfg = GridRouteConfig(track_width=0.3, layers=['F.Cu', 'In1.Cu', 'B.Cu'])
    cfg.power_net_widths = {1: 0.5}
    prev = fab_tiers.get_escalation_policy()
    try:
        fab_tiers.set_escalation_policy('off')
        resolve = _plane_join_width_resolver(
            cfg, None, '', [1, 2, 3], {2: 0.4}, {}, 0.2, 2.0)
        assert resolve(1, ['In1.Cu']) == (0.5, 2.0, 0.5)    # power width
        assert resolve(2, ['In1.Cu']) == (0.4, 2.0, 0.4)    # class width
        assert resolve(3, ['In1.Cu']) == (0.3, 2.0, 0.3)    # --track-width
        # the caller's config is left alone
        assert cfg.net_track_widths == {}
        fab_tiers.set_escalation_policy('fab')
        resolve = _plane_join_width_resolver(
            cfg, None, '', [1, 2, 3], {2: 0.4}, {}, 0.2, 2.0)
        assert resolve(1, ['In1.Cu']) == (0.2, 2.0, 0.3)
    finally:
        fab_tiers.set_escalation_policy(*prev)


BOARD = (Path(__file__).resolve().parent.parent / 'kicad_files'
         / 'lvds_converter_dualclk_gnd.kicad_pcb')
DRU = """(version 1)
(rule "gnd"
  (constraint track_width (min 0.3mm) (opt 0.5mm))
  (condition "A.NetName == '/GND'"))
"""


def test_resolver_reads_the_board_rules():
    # net_track_widths None (the standalone CLI, --track-width omitted): under
    # off the joins read the .kicad_dru opt; under board, its min is the floor.
    td = tempfile.mkdtemp()
    prev = fab_tiers.get_escalation_policy()
    try:
        board = os.path.join(td, 'b.kicad_pcb')
        shutil.copy(BOARD, board)
        with open(os.path.join(td, 'b.kicad_dru'), 'w') as f:
            f.write(DRU)
        pcb = parse_kicad_pcb(board)
        gnd = next(n for n, net in pcb.nets.items() if net.name == '/GND')
        other = next(n for n in pcb.nets if n not in (0, gnd))
        layers = list(pcb.board_info.copper_layers)
        cfg = GridRouteConfig(track_width=0.25, layers=layers)
        fab_tiers.set_escalation_policy('off')
        resolve = _plane_join_width_resolver(
            cfg, pcb, board, [gnd, other], None, None, 0.2, 2.0)
        assert resolve(gnd, ['B.Cu']) == (0.5, 2.0, 0.5)
        fab_tiers.set_escalation_policy('board', {'track_width': 0.15})
        resolve = _plane_join_width_resolver(
            cfg, pcb, board, [gnd, other], None, None, 0.2, 2.0)
        assert resolve(gnd, ['B.Cu']) == (0.3, 2.0, 0.3)
        assert resolve(other, ['B.Cu']) == (0.2, 2.0, 0.25)
    finally:
        fab_tiers.set_escalation_policy(*prev)
        shutil.rmtree(td, ignore_errors=True)


DRU_MIN = """(version 1)
(rule "gnd_b"
  (layer B.Cu)
  (constraint track_width (min 0.35mm))
  (condition "A.NetName == '/GND'"))
"""


def _board_with(td, dru):
    board = os.path.join(td, 'b.kicad_pcb')
    shutil.copy(BOARD, board)
    with open(os.path.join(td, 'b.kicad_dru'), 'w') as f:
        f.write(dru)
    pcb = parse_kicad_pcb(board)
    gnd = next(n for n, net in pcb.nets.items() if net.name == '/GND')
    return board, pcb, gnd


def test_explicit_width_is_clamped_by_a_layer_rule():
    # An explicit --track-width ({}) under off: a custom rule's min on B.Cu
    # raises the GND joins there, as route.py clamps the same request (#2014).
    td = tempfile.mkdtemp()
    prev = fab_tiers.get_escalation_policy()
    try:
        board, pcb, gnd = _board_with(td, DRU_MIN)
        cfg = GridRouteConfig(track_width=0.2,
                              layers=list(pcb.board_info.copper_layers))
        fab_tiers.set_escalation_policy('off')
        resolve = _plane_join_width_resolver(
            cfg, pcb, board, [gnd], {}, {}, 0.2, 2.0)
        assert resolve(gnd, ['F.Cu']) == (0.2, 2.0, 0.2)
        assert resolve(gnd, ['F.Cu', 'B.Cu']) == (0.35, 2.0, 0.35)
    finally:
        fab_tiers.set_escalation_policy(*prev)
        shutil.rmtree(td, ignore_errors=True)


def test_board_floor_from_the_callers_rules():
    # route.py's in-run finalize repairs the output file, whose .kicad_dru
    # does not exist yet: the floor must come from the rules it forwards.
    td = tempfile.mkdtemp()
    prev = fab_tiers.get_escalation_policy()
    try:
        from design_rules import DesignRules
        board, pcb, gnd = _board_with(td, DRU)
        layers = list(pcb.board_info.copper_layers)
        rules = DesignRules.from_project(pcb, board, copper_layers=layers)
        out = os.path.join(td, 'out', 'o.kicad_pcb')
        cfg = GridRouteConfig(track_width=0.25, layers=layers)
        fab_tiers.set_escalation_policy('board', {})
        blind = _plane_join_width_resolver(
            cfg, pcb, out, [gnd], {}, {}, 0.2, 2.0)
        told = _plane_join_width_resolver(
            cfg, pcb, out, [gnd], {}, {}, 0.2, 2.0, design_rules=rules)
        assert blind(gnd, layers) == (0.2, 2.0, 0.25)
        assert told(gnd, layers) == (0.3, 2.0, 0.3)
    finally:
        fab_tiers.set_escalation_policy(*prev)
        shutil.rmtree(td, ignore_errors=True)


TESTS = [test_resolver_takes_the_strictest_layer,
         test_resolver_reads_power_and_class_widths_per_net,
         test_resolver_reads_the_board_rules,
         test_explicit_width_is_clamped_by_a_layer_rule,
         test_board_floor_from_the_callers_rules]

if __name__ == '__main__':
    for t in TESTS:
        t()
    print(f"{len(TESTS)} passed")

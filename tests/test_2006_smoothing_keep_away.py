#!/usr/bin/env python3
"""#2006: octolinear smoothing does not lengthen a keep-away band.

The router bends a track away from the other side's copper under --keep-away;
smoothing (#536) then replaced the bend with a shortcut through the band
(opentac30-lite sda-bus: 0.27 -> 1.28 mm in band, 0.441 -> 0.152 mm from an
analog pad, at any --keep-away-cost). A shortcut is now refused when it puts
more track inside a band than the copper it replaces, measured as
keep_away_report measures the shipped board.

Fixture: /SIG runs an L (10,10)-(10,16)-(16,16) between its two pads; the
only shortcut is the 45-degree diagonal. An /ANA pad beside the diagonal's
middle sits 0.27 mm from it edge to edge -- legal at the 0.1 mm clearance,
inside a 0.5 mm keep-away -- and 3 mm from the L.

    python3 tests/test_2006_smoothing_keep_away.py
"""
import contextlib
import io
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import keep_away as ka  # noqa: E402
from kicad_parser import BoardInfo, Net, Pad, PCBData, Segment  # noqa: E402
from pcb_modification import smooth_octolinear_chains  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402

CLR = 0.1
W = 0.15
L_PATH = ((10.0, 10.0), (10.0, 16.0), (16.0, 16.0))
NEAR_DIAGONAL = (13.35, 12.65)     # 0.27 mm from the diagonal, edge to edge
OFF_DIAGONAL = (13.0, 10.0)        # 2 mm or more from either path


def _config(rules=(), cost=0.5):
    return GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=W, clearance=CLR,
                           keep_away=ka.normalize_keep_away_specs(list(rules)),
                           keep_away_free=1.0, keep_away_cost=cost)


def _pad(ref, x, y, net_id, name, size):
    return Pad(ref, '1', x, y, 0.0, 0.0, size, size, 'circle', ['F.Cu'],
               net_id, name, pad_type='smd')


def _board(ana_at):
    segs = [Segment(a[0], a[1], b[0], b[1], W, 'F.Cu', 1)
            for a, b in zip(L_PATH, L_PATH[1:])]
    pads = {1: [_pad('U1', *L_PATH[0], 1, '/SIG', 0.5),
                _pad('U2', *L_PATH[-1], 1, '/SIG', 0.5)],
            2: [_pad('U3', *ana_at, 2, '/ANA', 0.3)]}
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'}, copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=(0.0, 0.0, 30.0, 30.0))
    return PCBData(board_info=bi, nets={1: Net(1, '/SIG'), 2: Net(2, '/ANA')},
                   footprints={}, vias=[], segments=segs, pads_by_net=pads)


def _smooth(pcb, cfg):
    with contextlib.redirect_stdout(io.StringIO()):
        _n, nets, _strip, _added, _st = smooth_octolinear_chains(
            [], pcb, clearance=CLR, config=cfg)
    return nets


def _diagonal(pcb):
    return any(abs(abs(s.end_x - s.start_x) - abs(s.end_y - s.start_y)) < 1e-6
               and math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) > 1.0
               for s in pcb.segments)


def _in_band(pcb, rules):
    with contextlib.redirect_stdout(io.StringIO()):
        rep = ka.keep_away_report(pcb, _config(rules))
    return rep['nets'].get('/SIG', {}).get('in_band_mm', 0.0)


def test_no_rule_takes_the_shortcut_through_the_band():
    """Without a rule the diagonal is taken, and it does run inside the band
    the rule would set: the fixture exercises the refusal below."""
    pcb = _board(NEAR_DIAGONAL)
    assert _smooth(pcb, _config()) == 1 and _diagonal(pcb), pcb.segments
    assert _in_band(pcb, ['/SIG:/ANA:0.5']) > 0.5


def test_rule_keeps_the_routed_copper():
    pcb = _board(NEAR_DIAGONAL)
    assert _in_band(pcb, ['/SIG:/ANA:0.5']) == 0.0
    before = [(s.start_x, s.start_y, s.end_x, s.end_y) for s in pcb.segments]
    assert _smooth(pcb, _config(['/SIG:/ANA:0.5'])) == 0
    assert [(s.start_x, s.start_y, s.end_x, s.end_y) for s in pcb.segments] == before
    assert _in_band(pcb, ['/SIG:/ANA:0.5']) == 0.0


def test_rule_takes_a_shortcut_outside_the_band():
    pcb = _board(OFF_DIAGONAL)
    assert _smooth(pcb, _config(['/SIG:/ANA:0.5'])) == 1 and _diagonal(pcb)
    assert _in_band(pcb, ['/SIG:/ANA:0.5']) == 0.0


def test_report_only_rule_does_not_steer_smoothing():
    """keep_away_cost 0 measures without steering the route, so it does not
    steer smoothing either."""
    pcb = _board(NEAR_DIAGONAL)
    assert _smooth(pcb, _config(['/SIG:/ANA:0.5'], cost=0)) == 1 and _diagonal(pcb)


TESTS = [test_no_rule_takes_the_shortcut_through_the_band,
         test_rule_keeps_the_routed_copper,
         test_rule_takes_a_shortcut_outside_the_band,
         test_report_only_rule_does_not_steer_smoothing]


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

#!/usr/bin/env python3
"""A length-match group measures its single-ended members as the board
carries them, and a member this step laid no copper for is that earlier
copper.

route.py records a net's `route_length` as the new copper PLUS every segment
the net already had (`stub_length`), while the meander loop measured the new
copper alone. So a member started in one measure and was lengthened and
reported in another: a net no bump fitted read shorter after "matching" by
exactly its stub length, and a net whose pads earlier copper already joined
(no new copper at all) read `new length = 0.00mm`, recorded 0 and wrote 0
back as its length. Neither measure is what ships, since a fanout stub the
new route leaves dead is stripped at the end of the step.

Rows:
  1. a 2-pad member routed this step with a dead stub on the board is
     measured on the board's pad-to-pad copper (`net_signal_length`), not new
     copper + stub, and its offset is the board measure less its own copper;
  2. that measure sets the group target: a stub-free 11 mm member is now the
     longest, where new copper + stub (13 mm) used to be;
  3. a member with no new copper is measured as its copper, not meandered,
     and recorded at that length -- no `new length = 0.00mm`.

    python3 tests/test_length_group_board_measure.py
"""
import contextlib
import io
import os
import sys

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import BoardInfo                                # noqa: E402
from routing_config import GridRouteConfig                        # noqa: E402
from synth import make_net, make_pad, make_pcb, make_seg          # noqa: E402
import length_matching as lm                                      # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def board():
    """Z: pads at (0,0) and (10,0), this step's track between them and an
    earlier 3 mm stub off pad 1 that the track leaves dead. W: an 11 mm
    straight route, no stub. Y: pads joined by 9 mm of earlier copper, short
    of W, so the group would lengthen it."""
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'}, copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=(-5.0, -5.0, 20.0, 15.0))
    pads = {1: [make_pad(1, 0, 0, ref='U1', num='1'), make_pad(1, 10, 0, ref='R1', num='1')],
            2: [make_pad(2, 0, 4, ref='U1', num='2'), make_pad(2, 11, 4, ref='R2', num='1')],
            3: [make_pad(3, 0, 8, ref='U1', num='3'), make_pad(3, 9, 8, ref='R3', num='1')]}
    z_route = make_seg(0, 0, 10, 0, net_id=1)
    z_stub = make_seg(0, 0, 0, -3, net_id=1)
    w_route = make_seg(0, 4, 11, 4, net_id=2)
    y_old = make_seg(0, 8, 9, 8, net_id=3)
    pcb = make_pcb(nets={1: make_net(1, 'Z'), 2: make_net(2, 'W'), 3: make_net(3, 'Y')},
                   segments=[z_route, z_stub, w_route, y_old], pads_by_net=pads,
                   board_info=bi)
    results = {
        'Z': {'new_segments': [z_route], 'new_vias': [], 'route_length': 13.0,
              'stub_length': 3.0},
        'W': {'new_segments': [w_route], 'new_vias': [], 'route_length': 11.0,
              'stub_length': 0.0},
        'Y': {'new_segments': [], 'new_vias': [], 'route_length': 9.0,
              'stub_length': 9.0},
    }
    return pcb, results


def main():
    print("1. a member with a dead stub, measured on the board")
    pcb, res = board()
    group = {'Z': res['Z']}
    die = {'Z': 0.0}
    offsets = lm._measure_single_ended_on_board(group, die, pcb)
    z = res['Z']['route_length']
    check("Z reads its pad-to-pad copper, not new copper + stub",
          9.5 < z < 10.5, f"{z:.3f} (was 13.0)")
    check("its offset is the board measure less its own 10 mm of copper",
          abs(offsets.get('Z', 99) - (z - 10.0)) < 1e-9, str(offsets))

    print("2. the board measure sets the group target")
    pcb, res = board()
    cfg = GridRouteConfig()
    cfg.length_match_tolerance = 0.2
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        lm.apply_length_matching_to_group({'Z': res['Z'], 'W': res['W']}, ['Z', 'W'],
                                          cfg, pcb)
    log = out.getvalue()
    check("the target is W's 11 mm, not Z's new copper + stub",
          'target=11.00mm' in log,
          next((l.strip() for l in log.splitlines() if 'target=' in l), ''))

    print("3. a member with no new copper")
    pcb, res = board()
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        lm.apply_length_matching_to_group({'Y': res['Y'], 'W': res['W']}, ['Y', 'W'],
                                          cfg, pcb)
    log = out.getvalue()
    check("it is named as earlier copper",
          'laid no copper this step' in log and 'Y' in log)
    check("no `new length = 0.00mm`", 'new length = 0.00mm' not in log)
    check("its length is still its copper, not 0",
          abs(res['Y']['route_length'] - 9.0) < 1e-9, str(res['Y']['route_length']))
    check("it is not meandered (no new segments laid for it)",
          res['Y']['new_segments'] == [], str(len(res['Y']['new_segments'])))
    check("it is reported as earlier copper the step cannot lengthen",
          'Y: 9.00mm (existing board copper, not modified)' in log)

    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())

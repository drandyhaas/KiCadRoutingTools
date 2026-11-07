#!/usr/bin/env python3
"""replay_negotiation.py BOARD FIELD OUT_DIR --sizes CLR,TRK,FTRK,VIA,DRILL SRC:DEST:MODE[:LAYERS] ... -- buses off one
shared array NEGOTIATED again from a recorded shared field (negotiated_buses.route_on_field): BOARD the board the
buses started from, FIELD the shared field a run planned (its BUS.bus/shared_field.kicad_pcb), each bus routed on it
round theirs with its own whole route, `--rounds` rounds (one by default), in process. The solo runs in turn and the
shared field are not run again.

What it is for: a change to the negotiated step judged on the field a run laid, in the time of the slowest bus's
rounds. Each bus's log OUT_DIR/neg.bus/neg/DEST/bus.log; the merged board OUT_DIR/neg.kicad_pcb; prints each bus's
grade line."""
KRT_TOOL = {'scope': [], 'kind': 'actor'}   # a research tool (awx), catalogued, shown at no door

import argparse
import contextlib
import io
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('board', help='the board the buses started from')
    ap.add_argument('field', help="a run's shared field (BUS.bus/shared_field.kicad_pcb)")
    ap.add_argument('out_dir')
    ap.add_argument('bus', nargs='+', metavar='SRC:DEST:MODE[:LAYERS]', help='each bus, as route_bus --bus names it')
    ap.add_argument('--sizes', required=True, metavar='CLR,TRK,FTRK,VIA,DRILL',
                    help="the run's --clearance-ceiling, --track-width, --fanout-track-width, --via-size, --via-drill")
    ap.add_argument('--rounds', type=int, default=1)
    a = ap.parse_args()
    board, field, outd = (os.path.abspath(x) for x in (a.board, a.field, a.out_dir))
    os.chdir(HERE)
    import negotiated_buses as nb
    from copy_board import copy_board
    os.makedirs(outd, exist_ok=True)
    f2, b2 = os.path.join(outd, 'shared_field.kicad_pcb'), os.path.join(outd, 'input.kicad_pcb')
    with contextlib.redirect_stdout(io.StringIO()):
        copy_board(field, f2)
        copy_board(board, b2)
    clr, trk, ftrk, vs, vd = a.sizes.split(',')
    buses = [tuple(b.split(':', 3)) for b in a.bus]
    buses = [b if len(b) == 4 else b + (None,) for b in buses]
    code, grades = nb.route_on_field(b2, f2, os.path.join(outd, 'neg.kicad_pcb'), buses, k=None, rounds=a.rounds,
                                     inproc=True, log=lambda s: print(s, flush=True), feedback_rounds=0,
                                     clearance_ceiling=clr, track_width=trk, fanout_track_width=ftrk, via_size=vs,
                                     via_drill=vd)
    for g in grades:
        print(g, flush=True)
    print(f'replay_negotiation: exit {code}')
    sys.exit(code)


if __name__ == '__main__':
    main()

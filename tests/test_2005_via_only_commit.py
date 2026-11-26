#!/usr/bin/env python3
"""#2005: a result that carries only vias is committed like any other.

`add_route_to_pcb_data` used to return at once when a result had no
segments, so a via-only result never reached `pcb_data.vias` -- while its
inverse, `remove_route_from_pcb_data`, removed one. A rip and a restore of
such a result lost its vias (e.g. a pair custody restore whose surviving
pieces are a via alone). Checks, on an in-repo board, with no routing:

  1. a via-only result commits its via, once
  2. removing it takes the via off; re-adding puts it back
  3. committing the same result twice does not duplicate the via
  4. a result with neither segments nor vias changes nothing
"""
import contextlib
import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
from kicad_parser import parse_kicad_pcb, Via                     # noqa: E402
from pcb_modification import (add_route_to_pcb_data,               # noqa: E402
                              remove_route_from_pcb_data)

BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
FAILS = []


def expect(cond, what):
    print(('  ok   ' if cond else '  FAIL ') + what)
    if not cond:
        FAILS.append(what)


def main():
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(BOARD)
    gnd = next(i for i, n in pcb.nets.items() if n.name.strip('/') == 'GND')
    pad = pcb.nets[gnd].pads[0]
    v = Via(x=pad.global_x + 3.0, y=pad.global_y + 3.0, size=0.45, drill=0.3,
            layers=['F.Cu', 'B.Cu'], net_id=gnd)
    n0, s0 = len(pcb.vias), len(pcb.segments)
    res = {'new_segments': [], 'new_vias': [v]}

    add_route_to_pcb_data(pcb, res)
    expect(sum(x is v for x in pcb.vias) == 1,
           "a via-only result puts its via on the board, once")
    expect(len(pcb.segments) == s0, "and no segment")

    remove_route_from_pcb_data(pcb, res)
    expect(not any(x is v for x in pcb.vias) and len(pcb.vias) == n0,
           "remove takes it off again")
    add_route_to_pcb_data(pcb, res, trace_event='restore')
    expect(sum(x is v for x in pcb.vias) == 1,
           "a restore puts it back (rip + restore keeps the via)")

    add_route_to_pcb_data(pcb, res)
    expect(sum(x is v for x in pcb.vias) == 1,
           "committing the same result again does not duplicate it")

    epoch = getattr(pcb, '_copper_epoch', 0)
    before = (len(pcb.vias), len(pcb.segments))
    add_route_to_pcb_data(pcb, {'new_segments': [], 'new_vias': []})
    add_route_to_pcb_data(pcb, {'new_segments': []})
    expect((len(pcb.vias), len(pcb.segments)) == before
           and getattr(pcb, '_copper_epoch', 0) == epoch,
           "an empty result (no segments, no vias) changes nothing")

    if FAILS:
        print(f"FAIL: {len(FAILS)} check(s)")
        return 1
    print("PASS: #2005 a via-only result is committed")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

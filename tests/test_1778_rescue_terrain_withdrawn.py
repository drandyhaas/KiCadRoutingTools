#!/usr/bin/env python3
"""A rescue escape that bought nothing is taken back out (#1778 follow-up).

net_rescue's #666 bare-ball escape lays a dogbone (a via, in or beside the
pad) before the rescue's gap routes, so they can start from a via with every
layer. When the gap routes then fail and the net is exactly as open as the
rescue found it, the escape was kept anyway as "terrain" for later passes --
and when nothing later used it, it shipped as a via joining nothing across
layers: KiCad's via_dangling, and a via-in-pad Type VII stamp for nothing
(the #1778 via-ko-nos1 board: U2's B.Cu pad got a via to an empty F.Cu).
Withdrawing it loses nothing: the next rescue of the net finds the pad bare
again and lays the escape anew.

`_withdraw_unused_escapes` now judges a withdrawal against the part count
BEFORE it (a connected net: 1, so the whole-net rule is unchanged), and the
rescue calls it, with the escape's own trace, when it ends with no gap
routed and no fewer parts than it started with.

  1. an open net's in-pad escape via that joins nothing is withdrawn (parts
     2 -> 2);
  2. an offset dogbone: the via AND its trace go (no sweep prunes a dead end
     on an open net);
  3. an escape via an open net's copper runs through (removing it would add
     a part) is kept, trace and all;
  4. the whole-net rule is what it was (a used via kept, an unused one gone);
  5. wiring: rescue_failed_nets withdraws a terrain-only outcome, with traces.

    python3 tests/test_1778_rescue_terrain_withdrawn.py
"""
import inspect
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import net_rescue                                                 # noqa: E402
from kicad_parser import BoardInfo, Net, Pad, PCBData, Segment, Via  # noqa: E402
from routing_config import GridRouteConfig                        # noqa: E402

CFG = GridRouteConfig(layers=['F.Cu', 'B.Cu'])
failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def board(offset=False, used=False, closed=False):
    """Net 1: P1 (F.Cu SMD) at (0, 0) with an escape via -- in the pad, or
    (offset) a dogbone at (0, 1.5) with its F.Cu trace -- P2 (F.Cu) at (6, 0)
    and P3 (F.Cu) at (0, 6). `used`: a B.Cu track from the escape via to a
    via at P3 (the escape joins P1 and P3). `closed`: an F.Cu track P1-P2
    (and, with `used`, P3 hangs off the via: the net is whole)."""
    pads = [Pad('U1', str(i), x, y, 0.0, 0.0, 0.6, 0.6, 'rect', ['F.Cu'], 1, 'N',
                pad_type='smd')
            for i, (x, y) in ((1, (0.0, 0.0)), (2, (6.0, 0.0)), (3, (0.0, 6.0)))]
    ey = 1.5 if offset else 0.0
    esc = Via(0.0, ey, 0.4, 0.2, ['F.Cu', 'B.Cu'], 1)
    trace = [Segment(0.0, 0.0, 0.0, ey, 0.2, 'F.Cu', 1)] if offset else []
    segs, vias = list(trace), [esc]
    if used:
        segs.append(Segment(0.0, ey, 0.0, 6.0, 0.2, 'B.Cu', 1))
        vias.append(Via(0.0, 6.0, 0.4, 0.2, ['F.Cu', 'B.Cu'], 1))
    if closed:
        segs.append(Segment(0.0, 0.0, 6.0, 0.0, 0.2, 'F.Cu', 1))
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'}, copper_layers=['F.Cu', 'B.Cu'])
    pcb = PCBData(board_info=bi, nets={1: Net(1, 'N')}, footprints={},
                  vias=vias, segments=segs, pads_by_net={1: pads})
    return pcb, {'new_segments': list(trace), 'new_vias': [esc]}


def parts(pcb):
    return net_rescue._net_component_info(pcb, 1)[0]


def main():
    print('1. open net, in-pad escape joining nothing')
    pcb, tap = board()
    n = parts(pcb)
    kept = net_rescue._withdraw_unused_escapes(pcb, 1, [tap], CFG, with_traces=True)
    ok('withdrawn', kept == [] and tap['new_vias'][0] not in pcb.vias, repr(kept))
    ok('part count unchanged', parts(pcb) == n == 3, f'{n} -> {parts(pcb)}')

    print('2. open net, offset dogbone')
    pcb, tap = board(offset=True)
    kept = net_rescue._withdraw_unused_escapes(pcb, 1, [tap], CFG, with_traces=True)
    ok('the via goes', tap['new_vias'][0] not in pcb.vias)
    ok('and its trace', tap['new_segments'][0] not in pcb.segments and kept == [],
       repr(kept))

    print('3. open net, an escape the copper runs through')
    for off in (False, True):
        pcb, tap = board(offset=off, used=True)
        n = parts(pcb)
        kept = net_rescue._withdraw_unused_escapes(pcb, 1, [tap], CFG, with_traces=True)
        ok(f'kept (offset={off})', kept == [tap] and tap['new_vias'][0] in pcb.vias
           and all(s in pcb.segments for s in tap['new_segments']), repr(kept))
        ok(f'parts unchanged (offset={off})', parts(pcb) == n == 2, f'{n} -> {parts(pcb)}')

    print('4. whole net: the rule is what it was')
    pcb, tap = board(closed=True, used=True)
    ok('the net is whole', parts(pcb) == 1)
    kept = net_rescue._withdraw_unused_escapes(pcb, 1, [tap], CFG)
    ok('a used via is kept', kept == [tap] and tap['new_vias'][0] in pcb.vias)
    pcb, tap = board(offset=True, closed=True)
    pcb.pads_by_net[1] = pcb.pads_by_net[1][:2]
    ok('the two-pad net is whole', parts(pcb) == 1)
    kept = net_rescue._withdraw_unused_escapes(pcb, 1, [tap], CFG)
    ok('an unused via goes, its trace stays for the sweep',
       tap['new_vias'][0] not in pcb.vias and tap['new_segments'][0] in pcb.segments
       and [r['new_segments'] for r in kept] == [tap['new_segments']], repr(kept))

    print('5. wiring')
    src = inspect.getsource(net_rescue.rescue_failed_nets)
    ok('the start count is taken before the escapes', 'num_start = num0' in src
       and src.find('num_start = num0') < src.find('bare-ball escape: '))
    ok('a terrain-only outcome is withdrawn with its traces',
       'elif tap_results and not edge_results and num >= num_start:' in src
       and 'with_traces=True' in src)

    if failures:
        print(f'FAILED: {len(failures)}: {failures}')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())

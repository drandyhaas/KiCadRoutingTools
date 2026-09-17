#!/usr/bin/env python3
"""A footprint segment bridging TWO nets' pads is not lifted for either.

    python3 tests/test_908_bridge_between_two_nets_blocks.py

#908's own-pad lift exists so a footprint's own copper cannot SEAL the pad it
was drawn around (#907). It names, per segment, the nets of that footprint's
pads the segment touches -- and it named ALL of them. A segment that bridges
pads of two DIFFERENT nets was therefore lifted for BOTH, each net routed into
it, and the two met inside net-less copper. KiCad grades that `shorting_items`.

MEASURED on a20_can (corpus set3), `3.3V/5.0V1` =
`OLIMEX_Jumpers-FP:SJ_2_SMALL_12_TIED`: one F.Cu segment from pad 1 `+5V` to
pad 2 `Net-(3.3V/5.0V1-Pad2)`, and `footprint.net_tie_groups == []` -- the NAME
says tied, the footprint DECLARES nothing, so #908's tie path never applied.

    closest track gap to that bridge      f0838d2d     HEAD before fix
      +5V                                 +3.481 mm      -0.171 mm
      Net-(3.3V/5.0V1-Pad2)               +1.352 mm      -0.230 mm

KiCad `shorting_items` 2 -> 4 (the other 2 are the jumper's own pads touching
its own bridge, present in the UNROUTED input). With the bridge blocking:
back to 2, `check_connected` EXIT=0, `drc_real` 0, completion 100%.

A DECLARED net tie keeps its lift -- that is the case KiCad itself exempts,
and the case #908 added the tie path for (cparti_fpga's NT1-NT4, whose 8 pads
shipped unconnected without it). Census over 489 corpus boards: of 1126
segments lifted for >= 2 nets, 1032 are declared ties and keep the lift; the
94 that are not are solder jumpers on 6 boards -- the a20_can family exactly.

The rows below pin the RULE on synthetic data (so they discriminate with no
corpus present) and the two REAL boards that stand on either side of it.
"""
import io
import os
import sys
import contextlib

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('', 'py_router', 'py_tools'):
    _q = os.path.join(ROOT, _p)
    if _q not in sys.path:
        sys.path.insert(0, _q)

from kicad_parser import (Footprint, Net, Pad, PCBData, Segment,  # noqa: E402
                          parse_kicad_pcb)
from check_drc import graphic_own_pad_nets, graphic_effective_nets  # noqa: E402

STRESS = os.path.expanduser('~/Documents/kicad_stress_test')
A20 = f'{STRESS}/boards_unrouted_set3/a20_can.kicad_pcb'
CPARTI = f'{STRESS}/boards_unrouted_set2/cparti_fpga.kicad_pcb'

fails = []


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}"
          + (f"   [{detail}]" if detail else ''))
    if not cond:
        fails.append(name)


def _bridge(tie_groups=None):
    """Two pads on DIFFERENT nets, one footprint segment spanning both."""
    p1 = Pad(pad_number='1', net_id=1, net_name='/A', global_x=0.0, global_y=0.0,
             local_x=0, local_y=0, size_x=1.0, size_y=1.0, shape='rect',
             layers=['F.Cu'], drill=0, pad_type='smd', component_ref='JP1')
    p2 = Pad(pad_number='2', net_id=2, net_name='/B', global_x=2.0, global_y=0.0,
             local_x=2, local_y=0, size_x=1.0, size_y=1.0, shape='rect',
             layers=['F.Cu'], drill=0, pad_type='smd', component_ref='JP1')
    bar = Segment(start_x=0.0, start_y=0.0, end_x=2.0, end_y=0.0, width=0.5,
                  layer='F.Cu', net_id=0, graphic=True, owner_ref='JP1')
    fp = Footprint(reference='JP1', footprint_name='Jumper:SJ', x=0.0, y=0.0,
                   rotation=0.0, layer='F.Cu', pads=[p1, p2])
    if tie_groups is not None:
        fp.net_tie_groups = tie_groups
    return PCBData(board_info=None, nets={1: Net(1, '/A'), 2: Net(2, '/B')},
                   footprints={'JP1': fp}, vias=[], segments=[bar],
                   pads_by_net={1: [p1], 2: [p2]}), bar


def _one_pad():
    """A stub touching ONE pad only -- the #907 case the lift exists for."""
    p1 = Pad(pad_number='1', net_id=1, net_name='/A', global_x=0.0, global_y=0.0,
             local_x=0, local_y=0, size_x=1.0, size_y=1.0, shape='rect',
             layers=['F.Cu'], drill=0, pad_type='smd', component_ref='U1')
    p2 = Pad(pad_number='2', net_id=2, net_name='/B', global_x=9.0, global_y=0.0,
             local_x=9, local_y=0, size_x=1.0, size_y=1.0, shape='rect',
             layers=['F.Cu'], drill=0, pad_type='smd', component_ref='U1')
    stub = Segment(start_x=0.4, start_y=0.0, end_x=3.0, end_y=0.0, width=0.3,
                   layer='F.Cu', net_id=0, graphic=True, owner_ref='U1')
    fp = Footprint(reference='U1', footprint_name='L:P', x=0.0, y=0.0,
                   rotation=0.0, layer='F.Cu', pads=[p1, p2])
    return PCBData(board_info=None, nets={1: Net(1, '/A'), 2: Net(2, '/B')},
                   footprints={'U1': fp}, vias=[], segments=[stub],
                   pads_by_net={1: [p1], 2: [p2]}), stub


def board(path):
    with contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(path)


def main():
    print("1. the rule, on synthetic data")
    pcb, bar = _bridge(tie_groups=None)
    own = graphic_own_pad_nets(pcb)
    check("an UNDECLARED bridge between two nets is lifted for NEITHER",
          id(bar) not in own, f"lifted for {sorted(own.get(id(bar), ()))}")

    pcb, bar = _bridge(tie_groups=[['1', '2']])
    own = graphic_own_pad_nets(pcb)
    check("a DECLARED net tie keeps its lift for both its nets",
          own.get(id(bar)) == frozenset({1, 2}), f"{own.get(id(bar))}")

    pcb, stub = _one_pad()
    own = graphic_own_pad_nets(pcb)
    check("a one-pad stub still lifts (the #907 case the lift exists for)",
          own.get(id(stub)) == frozenset({1}), f"{own.get(id(stub))}")

    print("2. the generator stays a SUBSET of the checker")
    # The documented invariant: own-pad subset-of effective(False).
    for label, mk in (("bridge", lambda: _bridge(None)),
                      ("declared tie", lambda: _bridge([['1', '2']])),
                      ("one-pad stub", _one_pad)):
        p, seg = mk()
        o = graphic_own_pad_nets(p).get(id(seg), frozenset())
        e = graphic_effective_nets(p, include_mutable=False).get(id(seg), frozenset())
        check(f"{label}: own-pad subset of effective(include_mutable=False)",
              o <= e, f"own={sorted(o)} eff={sorted(e)}")

    print("3. the real boards on either side of the rule")
    if os.path.isfile(A20):
        pcb = board(A20)
        fp = (pcb.footprints or {}).get('3.3V/5.0V1')
        gs = [s for s in pcb.segments if getattr(s, 'graphic', False)
              and getattr(s, 'owner_ref', None) == '3.3V/5.0V1']
        own = graphic_own_pad_nets(pcb)
        check("a20_can: the jumper declares NO tie group",
              fp is not None and not getattr(fp, 'net_tie_groups', None),
              f"net_tie_groups={getattr(fp, 'net_tie_groups', None) if fp else 'no fp'}")
        check("a20_can: its bridge is lifted for NO net (it blocks)",
              bool(gs) and all(id(s) not in own for s in gs),
              f"{len(gs)} seg(s), lifted={[sorted(own.get(id(s), ())) for s in gs]}")
    else:
        print(f"  SKIP: {A20} not present (corpus absent) -- rows 1/2 still ran")

    if os.path.isfile(CPARTI):
        pcb = board(CPARTI)
        own = graphic_own_pad_nets(pcb)
        segid = {id(s): s for s in pcb.segments}
        tie_lifts = [v for k, v in own.items()
                     if str(getattr(segid[k], 'owner_ref', '')).startswith('NT')]
        check("cparti_fpga: its DECLARED ties keep every multi-net lift",
              len(tie_lifts) == 16 and all(len(v) == 2 for v in tie_lifts),
              f"{len(tie_lifts)} lift(s), sizes={sorted({len(v) for v in tie_lifts})}")
    else:
        print(f"  SKIP: {CPARTI} not present (corpus absent)")

    print(f"\n{len(fails)} failed")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""A pad tap's trace meets the pad layer's .kicad_dru track_width rule.

A pad tap (plane tap, bare-ball escape, fanout rescue) draws its trace on the
pad's layer at the config's scalar track_width. That scalar is layer-blind: a
caller's 0.1016 signal width, or a fine-pitch rung floored at the net's
LAYER-LESS rule minimum. A net whose rule fixes it at 0.18 mm on the outer
layers and 0.102 mm inside got a 0.1016 F.Cu escape stub from the bare-ball
rescue -- a track_width violation per segment.

Invariants gated here, on a synthetic four-layer board and its .kicad_dru:
  1. an F.Cu tap of a rule-bound net draws every F.Cu segment at >= 0.18;
  2. a net no rule binds keeps the caller's width;
  3. no rules at all: the caller's width.
"""
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
from synth import RoutingBoard                                  # noqa: E402
from kicad_parser import parse_kicad_pcb                        # noqa: E402
from design_rules import DesignRules                            # noqa: E402
from routing_config import GridRouteConfig                      # noqa: E402
from fab_tiers import get_escalation_policy, set_escalation_policy  # noqa: E402
from plane_pad_tap import tap_pad_with_escalation               # noqa: E402

LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
DRU = """(version 1)
(rule "addr_outer" (layer outer) (condition "A.NetName == 'ADDR*'")
    (constraint track_width (min 0.18mm) (opt 0.18mm) (max 0.18mm)))
(rule "addr_inner" (layer inner) (condition "A.NetName == 'ADDR*'")
    (constraint track_width (min 0.102mm) (opt 0.102mm) (max 0.102mm)))
"""


def build(td, with_rules=True):
    b = RoutingBoard(w=20, h=10, layers=tuple(LAYERS))
    # small SMD pads: a via cannot sit in them, so the tap draws a trace
    for ref, x, net in (('A', 6, 'ADDR0'), ('B', 14, 'SIG0')):
        b.pad(ref + '1', x, 5, net, sx=0.25, sy=0.25)
        # foreign neighbours left, right and above: no via fits in or at the
        # pad, so the tap draws a trace out to a via below
        b.pad(ref + '2', x - 0.4, 5, 'GND', sx=0.25, sy=0.25)
        b.pad(ref + '3', x + 0.4, 5, 'GND', sx=0.25, sy=0.25)
        b.pad(ref + '4', x, 4.6, 'GND', sx=0.25, sy=0.25)
    path = os.path.join(td, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(b.text())
    if with_rules:
        with open(os.path.join(td, 'b.kicad_dru'), 'w', encoding='utf-8') as f:
            f.write(DRU)
    pcb = parse_kicad_pcb(path)
    rules = DesignRules.from_project(pcb, path) if with_rules else None
    return pcb, rules


def tap(pcb, rules, net_name):
    nid = {n.name: i for i, n in pcb.nets.items()}[net_name]
    pad = next(p for fp in pcb.footprints.values() for p in fp.pads
               if p.net_id == nid)
    cfg = GridRouteConfig(layers=list(LAYERS), track_width=0.1016,
                          clearance=0.1016, via_size=0.4, via_drill=0.2,
                          rules=rules)
    res = tap_pad_with_escalation(pad, 'F.Cu', nid, pcb, cfg,
                                  max_search_radius=1.5, via_size=0.4,
                                  via_drill=0.2, fine_for_all=True)
    segs = [s for s in (res.segments or [])
            if (s['layer'] if isinstance(s, dict) else s.layer) == 'F.Cu']
    widths = [s['width'] if isinstance(s, dict) else s.width for s in segs]
    return res.success, widths


def main():
    fails = []

    def check(name, cond):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}")
        if not cond:
            fails.append(name)

    prev = get_escalation_policy()
    try:
        # 'off': no via descent, so the tap must draw its trace
        set_escalation_policy('off')
        with tempfile.TemporaryDirectory() as td:
            pcb, rules = build(td)
            print("1: rule-bound net's F.Cu tap trace is >= 0.18")
            ok, w = tap(pcb, rules, 'ADDR0')
            print(f"     success={ok} F.Cu widths={w}")
            check("tap succeeds with a trace", ok and bool(w))
            check("every F.Cu segment >= 0.18",
                  bool(w) and all(x >= 0.18 - 1e-9 for x in w))
            print("1b: --escalation board (fine rungs, floor at the "
                  "layer-less rule minimum) draws nothing below 0.18 on F.Cu")
            set_escalation_policy('board')
            ok, w = tap(pcb, rules, 'ADDR0')
            print(f"     success={ok} F.Cu widths={w}")
            check("board: tap succeeds", ok)
            check("board: no F.Cu segment below 0.18",
                  all(x >= 0.18 - 1e-9 for x in w))
            set_escalation_policy('off')
            print("2: a net no rule binds keeps the caller's width")
            ok, w = tap(pcb, rules, 'SIG0')
            print(f"     success={ok} F.Cu widths={w}")
            check("SIG0 tap at 0.1016",
                  ok and bool(w) and all(abs(x - 0.1016) < 1e-9 for x in w))
        with tempfile.TemporaryDirectory() as td:
            pcb, rules = build(td, with_rules=False)
            print("3: no rules, caller's width")
            ok, w = tap(pcb, None, 'ADDR0')
            print(f"     success={ok} F.Cu widths={w}")
            check("ADDR0 tap at 0.1016 without rules",
                  ok and bool(w) and all(abs(x - 0.1016) < 1e-9 for x in w))
    finally:
        set_escalation_policy(*prev)

    print()
    if fails:
        print(f"FAILED: {len(fails)} check(s): {fails}")
        return 1
    print("All tap rule-width checks passed.")
    return 0


if __name__ == '__main__':
    sys.exit(main())

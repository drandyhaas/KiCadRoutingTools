#!/usr/bin/env python3
"""A power-channel width never leaves the net's per-layer rule window.

power_net_widths is a SCALAR per net and outranks the per-layer width map in
get_net_track_width. The width-preserving re-routes (final reconciliation,
plane-repair reconnect, rip victims) feed a net's dominant routed width
through that channel. A net whose .kicad_dru rule fixes it at 0.18 mm on the
outer layers and 0.102 mm on the inner ones routes mostly inside, so its
dominant width is 0.102; the reconciliation pinned that for every layer and
the net's B.Cu stub shipped at 0.102 (a track_width violation per segment).

Invariants gated here, on a synthetic four-layer board and its .kicad_dru:
  1. a preserved 0.102 power width reads 0.18 on F.Cu/B.Cu, 0.102 inside;
  2. a 0.3 power width is held to each layer's maximum (0.18 / 0.102);
  3. a net no rule binds keeps its power width exactly;
  4. under --escalation fab the rule MINIMUM no longer lifts the width (the
     maximum still binds), as rule_floors already behaves;
  5. no rules at all: the power width is returned unchanged.
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

LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
DRU = """(version 1)
(rule "addr_outer" (layer outer) (condition "A.NetName == 'ADDR*'")
    (constraint track_width (min 0.18mm) (opt 0.18mm) (max 0.18mm)))
(rule "addr_inner" (layer inner) (condition "A.NetName == 'ADDR*'")
    (constraint track_width (min 0.102mm) (opt 0.102mm) (max 0.102mm)))
"""


def main():
    fails = []

    def check(name, cond):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}")
        if not cond:
            fails.append(name)

    with tempfile.TemporaryDirectory() as td:
        b = RoutingBoard(w=20, h=10, layers=tuple(LAYERS))
        b.pad('A1', 2, 5, 'ADDR0')
        b.pad('A2', 18, 5, 'ADDR0')
        b.pad('B1', 2, 8, 'VCC')
        b.pad('B2', 18, 8, 'VCC')
        path = os.path.join(td, 'b.kicad_pcb')
        with open(path, 'w', encoding='utf-8') as f:
            f.write(b.text())
        with open(os.path.join(td, 'b.kicad_dru'), 'w', encoding='utf-8') as f:
            f.write(DRU)
        pcb = parse_kicad_pcb(path)
        rules = DesignRules.from_project(pcb, path)
    nid = {n.name: i for i, n in pcb.nets.items()}
    addr, vcc = nid['ADDR0'], nid['VCC']

    prev = get_escalation_policy()
    try:
        set_escalation_policy('board')

        def cfg(widths, with_rules=True):
            return GridRouteConfig(layers=list(LAYERS), track_width=0.1016,
                                   power_net_widths=dict(widths),
                                   rules=rules if with_rules else None)

        print("1: preserved 0.102 power width meets each layer's rule")
        c = cfg({addr: 0.102})
        check("B.Cu 0.18", abs(c.get_net_track_width(addr, 'B.Cu') - 0.18) < 1e-9)
        check("F.Cu 0.18", abs(c.get_net_track_width(addr, 'F.Cu') - 0.18) < 1e-9)
        check("In1.Cu 0.102", abs(c.get_net_track_width(addr, 'In1.Cu') - 0.102) < 1e-9)

        print("2: a wide power width is held to the layer maximum")
        c = cfg({addr: 0.3})
        check("B.Cu 0.18", abs(c.get_net_track_width(addr, 'B.Cu') - 0.18) < 1e-9)
        check("In2.Cu 0.102", abs(c.get_net_track_width(addr, 'In2.Cu') - 0.102) < 1e-9)

        print("3: a net no rule binds keeps its power width")
        c = cfg({vcc: 0.5})
        check("VCC 0.5 on B.Cu", abs(c.get_net_track_width(vcc, 'B.Cu') - 0.5) < 1e-9)
        check("VCC 0.5 on In1.Cu", abs(c.get_net_track_width(vcc, 'In1.Cu') - 0.5) < 1e-9)

        print("4: --escalation fab drops the minimum, keeps the maximum")
        set_escalation_policy('fab')
        c = cfg({addr: 0.102})
        check("B.Cu stays 0.102 under fab",
              abs(c.get_net_track_width(addr, 'B.Cu') - 0.102) < 1e-9)
        c = cfg({addr: 0.3})
        check("B.Cu max 0.18 under fab",
              abs(c.get_net_track_width(addr, 'B.Cu') - 0.18) < 1e-9)
        set_escalation_policy('board')

        print("5: no rules, power width unchanged")
        c = cfg({addr: 0.102}, with_rules=False)
        check("B.Cu 0.102 without rules",
              abs(c.get_net_track_width(addr, 'B.Cu') - 0.102) < 1e-9)
    finally:
        set_escalation_policy(*prev)

    print()
    if fails:
        print(f"FAILED: {len(fails)} check(s): {fails}")
        return 1
    print("All power-width rule-bound checks passed.")
    return 0


if __name__ == '__main__':
    sys.exit(main())

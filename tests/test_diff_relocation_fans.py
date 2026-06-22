#!/usr/bin/env python3
"""
Regression test for issue #165 (relocation fallback): the dense-pad guard
`_fans_fit` and the fan-copper attachment `_attach_fans`.

When a diff pair can't launch on its pads' congested layer, the multipoint
router relocates the terminals to an open inner layer by dropping a through-via
on each pad and growing an inner-layer stub (the "fan"). Two bugs this guards:

  1. `_attach_fans` must put the fan VIAS on a real leg result, because the
     output is written from the per-leg `results` and vias reach the file ONLY
     through a result's new_vias (segment sync doesn't carry vias). A fan via
     left only on the bookkeeping `merged` dict is dropped -> the relocated route
     floats on the inner layer with no via to its surface pad (the tigard
     /USB_D failure: DRC-clean but fully disconnected).

  2. `_fans_fit` rejects a relocation whose per-pad through-vias can't physically
     fit - on a fine-pitch connector (USB-C 0.5mm pad pitch, 0.5mm via) the vias
     collide, so relocating there just trades the blockage for via-via
     violations. The pair should fall through to single-ended instead.

Run:
    python3 tests/test_diff_relocation_fans.py
"""

import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

from kicad_parser import Pad, Via, PCBData
from routing_config import GridRouteConfig
from diff_pair_multipoint import _fans_fit, _attach_fans

NET = 10


def _via(x, y, size=0.5):
    return Via(x=x, y=y, size=size, drill=0.25, layers=['F.Cu', 'B.Cu'], net_id=NET)


def _pad(x, y):
    return Pad('U1', '1', x, y, 0, 0, 0.3, 0.3, 'rect', ['F.Cu'],
               99, '/OTHER', 0, None, 0.0, None, 0.25, 0.0, None)


def _cfg():
    c = GridRouteConfig()
    c.clearance = 0.1
    c.via_size = 0.5
    c.via_drill = 0.25
    c.layers = ['F.Cu', 'In1.Cu', 'B.Cu']
    return c


def _pcb(pads_by_net):
    return PCBData(footprints={}, nets={}, segments=[], vias=[],
                   board_info=None, pads_by_net=pads_by_net)


def main():
    cfg = _cfg()
    results = []

    def check(name, ok, detail=""):
        results.append((name, ok))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}{('  ' + detail) if detail else ''}")

    # --- _fans_fit ---
    # 1. USB-C-style 0.5mm pitch: two 0.5mm vias 0.5mm apart -> collide -> NOT fit.
    fans_tight = [(_via(0.0, 0.0), None), (_via(0.0, 0.5), None)]
    pcb = _pcb({})
    check("tight 0.5mm-pitch fan vias rejected",
          not _fans_fit(pcb, fans_tight, [], cfg))

    # 2. Roomy pitch: same vias 2mm apart -> fit.
    fans_roomy = [(_via(0.0, 0.0), None), (_via(0.0, 2.0), None)]
    check("roomy fan vias accepted", _fans_fit(pcb, fans_roomy, [], cfg))

    # 3. A fan via colliding with a FOREIGN pad is rejected...
    near_pad = _pad(0.15, 0.0)  # pad edge ~0 from a via at origin
    pcb_pad = _pcb({99: [near_pad]})
    check("fan via over a foreign pad rejected",
          not _fans_fit(pcb_pad, [(_via(0.0, 0.0), None)], [], cfg))

    # 4. ...but the via legitimately sits on its OWN relocated pad (excluded).
    own_pad = _pad(0.0, 0.0)
    pcb_own = _pcb({99: [own_pad]})
    check("fan via on its own relocated pad allowed",
          _fans_fit(pcb_own, [(_via(0.0, 0.0), None)], [own_pad], cfg))

    # --- _attach_fans ---
    # Fan copper must land on a real leg result (legs[0]) so the writer emits it.
    via, seg = _via(1.0, 1.0), ('seg-stub',)
    fans = [(via, seg)]
    legs = [{'new_segments': [], 'new_vias': []}]
    merged = {'new_segments': [], 'new_vias': []}
    _attach_fans(merged, fans, legs)
    check("fan via attached to leg result (reaches output)", via in legs[0]['new_vias'])
    check("fan stub attached to leg result", seg in legs[0]['new_segments'])
    check("merged kept consistent for obstacle sync", via in merged['new_vias'])

    passed = sum(1 for _, ok in results if ok)
    total = len(results)
    print("\n" + "=" * 60)
    print(f"  {passed}/{total} checks passed")
    print("=" * 60)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())

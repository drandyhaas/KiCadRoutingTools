#!/usr/bin/env python3
"""
Gate for #659's "classify the link, then pick the operation" rule.

KiCad reporting a net unconnected says the net is open. It does NOT say what
repair is called for, and the three answers are different operations:

  live <-> live  -> a GENUINE open. Weld it.
  pad-less end   -> rip/reroute DEBRIS. It conducts nothing, so welding adds
                    dead metal; the late orphan sweep deletes it instead.
  graphic end    -> net-tagged copper ART (#337/#513). Immutable: neither.

Measured over the recorded stress corpus, the KiCad-only-open links on
zone-less signal nets split 9 pad-less / 36 graphic / 6 live -- so treating
the class as "weld it" is wrong for 45 of 51 links, and measurably harmful:
routing three such nets on spartan6_4layer cost 448 s, 2.0 M A* iterations on
one link, pulled 28 neighbours in through the rip escalation, fixed none of
them, and manufactured three fresh debris nets.

Gated here:
  1. classify_unconnected_link returns 'live' for copper that reaches a pad.
  2. ...'padless' for a stranded cluster, including a BARE VIA.
  3. ...'graphic' for net-tagged copper art.
  4. KiCad's own item kinds ('pad'/'zone') are trusted without a search.
  5. The fragment gate does NOT enroll a net whose extra fragments are all
     pad-less (routing cannot help), but DOES still enroll one whose extra
     fragment carries a pad (the class the #578 fragment gate exists for).

Run:
    python3 tests/test_659_link_classification.py
"""

import os
import sys

_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS_DIR)
for _p in (_ROOT, os.path.join(_ROOT, 'py_router')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from check_connected import classify_unconnected_link, net_dead_copper
from routing_common import filter_already_routed
from routing_config import GridRouteConfig
from synth import make_pad, make_seg, make_via, make_pcb, make_net

FAILURES = []


def check(cond, label):
    print(f"  {'PASS' if cond else 'FAIL'}  {label}")
    if not cond:
        FAILURES.append(label)


def _pcb(segments, vias, pads, net_name='SIG', net_id=1):
    return make_pcb(segments=list(segments), vias=list(vias),
                    pads_by_net={net_id: pads},
                    nets={net_id: make_net(net_id, net_name)})


def test_live_and_padless():
    """A route joining two pads, plus a stranded 2-segment island. The link
    KiCad would report runs between them."""
    pads = [make_pad(net_id=1, x=0.0, y=0.0, ref='U1', num='1'),
            make_pad(net_id=1, x=10.0, y=0.0, ref='U2', num='1')]
    route = [make_seg(0.0, 0.0, 10.0, 0.0, net_id=1)]
    island = [make_seg(4.0, 8.0, 6.0, 8.0, net_id=1),
              make_seg(6.0, 8.0, 6.0, 9.0, net_id=1)]
    pcb = _pcb(route + island, [], pads)
    a, b = classify_unconnected_link(pcb, 1, (5.0, 0.0, 'F.Cu', 'track'),
                                     (5.0, 8.0, 'F.Cu', 'track'),
                                     'track', 'track')
    check(a == 'live', f"copper reaching a pad classifies 'live' (got {a!r})")
    check(b == 'padless',
          f"stranded island classifies 'padless' (got {b!r})")


def test_bare_via_is_padless():
    pads = [make_pad(net_id=1, x=0.0, y=0.0, ref='U1', num='1'),
            make_pad(net_id=1, x=10.0, y=0.0, ref='U2', num='1')]
    route = [make_seg(0.0, 0.0, 10.0, 0.0, net_id=1)]
    stray = make_via(5.0, 8.0, net_id=1)
    pcb = _pcb(route, [stray], pads)
    a, b = classify_unconnected_link(pcb, 1, (5.0, 0.0, 'F.Cu', 'track'),
                                     (5.0, 8.0, None, 'via'), 'track', 'via')
    check(a == 'live' and b == 'padless',
          f"a BARE VIA classifies 'padless' (got {a!r}/{b!r}) -- the "
          f"spartan6 class, which a segment-only search reports 'unknown'")


def test_graphic_is_its_own_class():
    """A graphics-anchored cluster must NOT be reported as debris: deleting
    input art is forbidden (#337), so it needs a distinct verdict."""
    pads = [make_pad(net_id=1, x=0.0, y=0.0, ref='U1', num='1'),
            make_pad(net_id=1, x=10.0, y=0.0, ref='U2', num='1')]
    route = [make_seg(0.0, 0.0, 10.0, 0.0, net_id=1)]
    art = make_seg(4.0, 8.0, 6.0, 8.0, net_id=1)
    art.graphic = True
    pcb = _pcb(route + [art], [], pads)
    a, b = classify_unconnected_link(pcb, 1, (5.0, 0.0, 'F.Cu', 'track'),
                                     (5.0, 8.0, 'F.Cu', 'track'),
                                     'track', 'track')
    check(a == 'live' and b == 'graphic',
          f"net-tagged copper art classifies 'graphic', not 'padless' "
          f"(got {a!r}/{b!r})")


def test_kicad_item_kinds_are_trusted():
    pads = [make_pad(net_id=1, x=0.0, y=0.0, ref='U1', num='1'),
            make_pad(net_id=1, x=10.0, y=0.0, ref='U2', num='1')]
    route = [make_seg(0.0, 0.0, 10.0, 0.0, net_id=1)]
    pcb = _pcb(route, [], pads)
    # A point nowhere near any copper, but KiCad calls it a pad item.
    a, _b = classify_unconnected_link(pcb, 1, (99.0, 99.0, 'F.Cu', 'pad'),
                                      (5.0, 0.0, 'F.Cu', 'track'),
                                      'pad', 'track')
    check(a == 'live', f"a 'pad' item is live by definition (got {a!r})")


# ------------------------------------------------------- fragment gate ------

def _gate(pcb, net_name='SIG', net_id=1):
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'])
    to_route, already = filter_already_routed(
        pcb, [(net_name, net_id)], cfg, fragment_gate=True)
    return [n for n, _i in to_route], already


def test_gate_skips_padless_only_fragments():
    """Pads connected + every extra fragment pad-less -> nothing to route."""
    pads = [make_pad(net_id=1, x=0.0, y=0.0, ref='U1', num='1'),
            make_pad(net_id=1, x=10.0, y=0.0, ref='U2', num='1')]
    route = [make_seg(0.0, 0.0, 10.0, 0.0, net_id=1)]
    island = [make_seg(4.0, 8.0, 6.0, 8.0, net_id=1)]
    pcb = _pcb(route + island, [], pads)
    to_route, _already = _gate(pcb)
    check(to_route == [],
          f"a pad-less-only fragment does NOT enroll the net (got {to_route})")


def test_gate_still_routes_pad_bearing_fragments():
    """NEGATIVE CONTROL: the #578 fragment-gate case must survive. A third pad on its
    own fragment is a real open, and routing IS the repair."""
    pads = [make_pad(net_id=1, x=0.0, y=0.0, ref='U1', num='1'),
            make_pad(net_id=1, x=10.0, y=0.0, ref='U2', num='1'),
            make_pad(net_id=1, x=5.0, y=8.0, ref='U3', num='1')]
    route = [make_seg(0.0, 0.0, 10.0, 0.0, net_id=1)]
    other = [make_seg(4.0, 8.0, 6.0, 8.0, net_id=1)]
    pcb = _pcb(route + other, [], pads)
    to_route, already = _gate(pcb)
    check(to_route == ['SIG'],
          f"a fragment CARRYING a pad still enrolls the net "
          f"(got to_route={to_route}, already={already})")


def test_phantom_split_is_not_dead_copper():
    """The trap that broke this gate's first version.

    A fragment that is STRICTLY pad-less can still be the net's real route --
    it just ends a hair short of the pad's copper, which is the phantom split
    #578 exists to route (and the very micro-gap class #659 is about). So
    "strictly pad-less" must NEVER be used as the dead-copper test: the
    authoritative graph, which joins that gap, has to be the one asked.

    Fixture is the #578 shape: a long run whose ends sit 0.09 mm from two
    0.06 mm pads, plus a stub that touches pad 1 exactly.
    """
    pads = [make_pad(net_id=1, x=0.0, y=0.0, ref='U1', num='1',
                     size_x=0.06, size_y=0.06),
            make_pad(net_id=1, x=10.0, y=0.0, ref='U2', num='2',
                     size_x=0.06, size_y=0.06)]
    segs = [make_seg(0.09, 0.0, 9.91, 0.0, net_id=1, width=0.06),
            make_seg(0.0, 0.0, 0.0, 1.5, net_id=1, width=0.06)]
    pcb = _pcb(segs, [], pads)
    dead_s, dead_v = net_dead_copper(pcb, 1, segs, [], pads, [])
    check(dead_s == [] and dead_v == [],
          f"the phantom-split run is NOT dead copper "
          f"(got {len(dead_s)} seg, {len(dead_v)} via) -- reporting it dead "
          f"would divert a net that genuinely needs routing")
    to_route, _already = _gate(pcb)
    check(to_route == ['SIG'],
          f"...so the fragment gate still routes it (got {to_route})")


def test_real_debris_is_dead_copper():
    """Companion direction: genuine stranded copper IS reported dead, or the
    test above could pass by never reporting anything."""
    pads = [make_pad(net_id=1, x=0.0, y=0.0, ref='U1', num='1'),
            make_pad(net_id=1, x=10.0, y=0.0, ref='U2', num='1')]
    route = [make_seg(0.0, 0.0, 10.0, 0.0, net_id=1)]
    stray = make_via(5.0, 8.0, net_id=1)
    pcb = _pcb(route, [stray], pads)
    dead_s, dead_v = net_dead_copper(pcb, 1, route + [], [stray], pads, [])
    check(dead_s == [] and [id(v) for v in dead_v] == [id(stray)],
          f"a stranded bare via IS dead copper (got {len(dead_s)} seg, "
          f"{len(dead_v)} via)")


def main():
    print(__doc__.strip().splitlines()[0])
    for fn in (test_live_and_padless, test_bare_via_is_padless,
               test_graphic_is_its_own_class, test_kicad_item_kinds_are_trusted,
               test_gate_skips_padless_only_fragments,
               test_gate_still_routes_pad_bearing_fragments,
               test_phantom_split_is_not_dead_copper,
               test_real_debris_is_dead_copper):
        print(f"\n{fn.__name__}:")
        fn()
    print()
    if FAILURES:
        print(f"FAILED ({len(FAILURES)}): " + "; ".join(FAILURES))
        return 1
    print("Link-classification and fragment-gate invariants hold.")
    return 0


if __name__ == '__main__':
    sys.exit(main())

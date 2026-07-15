#!/usr/bin/env python3
"""Issue #389: ripping a net must not destroy INPUT copper that twins its route.

`remove_route_from_pcb_data` used to match by geometry signature. A reroute
that retraces an existing same-net span on the grid produces a segment
byte-identical to the input original; ripping that net then removed BOTH twins
-- the routed copy AND the pre-existing input copper. The #134-refused restore
never returned the original, and the #220 stale strip mirrored the loss into
the output file (neo6502: GPIO5's input branch severed, stranding its U7
island behind a partial reroute whose tail grazed GPIO4).

The contract now: removal is by object identity (a result's new_segments hold
the very objects add_route_to_pcb_data appended). A to-remove object not found
by identity (a caller holding copies) falls back to ONE signature-matched
removal, newest board copper first -- never every twin.

    python3 tests/test_rip_identity_removal.py
"""
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pcb_modification import remove_route_from_pcb_data


def _seg(x1, y1, x2, y2, width=0.127, layer='B.Cu', net=48):
    return SimpleNamespace(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                           width=width, layer=layer, net_id=net)


def _via(x, y, size=0.7, drill=0.4, net=48, layers=('F.Cu', 'B.Cu')):
    return SimpleNamespace(x=x, y=y, size=size, drill=drill, net_id=net,
                           layers=list(layers))


def _board(segs, vias):
    return SimpleNamespace(segments=list(segs), vias=list(vias))


def run():
    fails = []

    def check(name, cond):
        print(('PASS' if cond else 'FAIL') + f': {name}')
        if not cond:
            fails.append(name)

    # 1. The #389 twin case: input original + routed twin with identical
    # geometry. Ripping the route must remove ONLY the routed object.
    orig = _seg(144.9, 101.35, 144.9, 102.5)
    twin = _seg(144.9, 101.35, 144.9, 102.5)   # routed copy, same span
    other = _seg(10.0, 10.0, 12.0, 10.0)       # unrelated route copper
    pcb = _board([orig, twin, other], [])
    remove_route_from_pcb_data(pcb, {'new_segments': [twin, other], 'new_vias': []})
    check('input original survives a twin rip',
          any(s is orig for s in pcb.segments))
    check('routed twin removed', not any(s is twin for s in pcb.segments))
    check('routed other removed', not any(s is other for s in pcb.segments))

    # 2. Via twin of the same class.
    vorig = _via(144.9, 101.35)
    vtwin = _via(144.9, 101.35)
    pcb = _board([], [vorig, vtwin])
    remove_route_from_pcb_data(pcb, {'new_segments': [], 'new_vias': [vtwin]})
    check('input via survives a twin rip', any(v is vorig for v in pcb.vias))
    check('routed via removed', not any(v is vtwin for v in pcb.vias))

    # 3. Copy fallback: a result holding a COPY (not the board object) still
    # removes exactly one signature match -- and prefers the NEWEST board
    # object (input originals parse first, routes append after).
    orig = _seg(144.9, 101.35, 144.9, 102.5)
    routed = _seg(144.9, 101.35, 144.9, 102.5)
    copy_of_routed = _seg(144.9, 101.35, 144.9, 102.5)
    pcb = _board([orig, routed], [])
    remove_route_from_pcb_data(pcb, {'new_segments': [copy_of_routed], 'new_vias': []})
    check('copy fallback removes exactly one twin', len(pcb.segments) == 1)
    check('copy fallback keeps the input original (oldest)',
          pcb.segments and pcb.segments[0] is orig)

    # 4. Copy fallback honors direction normalization (reversed endpoints).
    orig = _seg(1.0, 1.0, 2.0, 1.0)
    rev_copy = _seg(2.0, 1.0, 1.0, 1.0)
    pcb = _board([orig], [])
    remove_route_from_pcb_data(pcb, {'new_segments': [rev_copy], 'new_vias': []})
    check('reversed-endpoint copy still matches', len(pcb.segments) == 0)

    # 5. No-op on empty result.
    keep = _seg(5.0, 5.0, 6.0, 5.0)
    pcb = _board([keep], [])
    remove_route_from_pcb_data(pcb, {'new_segments': [], 'new_vias': []})
    check('empty result is a no-op', pcb.segments == [keep])

    print()
    if fails:
        print(f'{len(fails)} FAILURE(S): {fails}')
        return 1
    print('all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(run())

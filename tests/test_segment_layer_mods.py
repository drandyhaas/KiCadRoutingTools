#!/usr/bin/env python3
"""modify_segment_layers must apply each stub layer-swap to the RIGHT net's
segment (issue #264).

Two nets routed through the same fanout escape corridor can carry
identical-geometry stub segments on different layers (stacked escapes). KiCad 10
files reference nets by NAME, and the old writer parsed the numeric-net group
only, so every KiCad-10 segment read as net 0: the exact (coords, net) match
never fired and the coordinate-only fallback applied the LAST mod at those
coordinates to BOTH twins. On keks that flipped /HRAM_ADQ2's already-swapped tip
segment back to /HRAM_DQS0's layer, leaving one segment on the wrong layer
mid-run -- stacked on foreign-net copper (and invisible to KiCad DRC, which
net-unifies touching copper on load).

    python3 tests/test_segment_layer_mods.py
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_writer import modify_segment_layers


def _board(net_a='(net "/A")', net_b='(net "/B")'):
    return f'''(kicad_pcb
\t(segment
\t\t(start 147.100000 148.787500)
\t\t(end 147.200000 148.800000)
\t\t(width 0.127)
\t\t(layer "In1.Cu")
\t\t{net_a}
\t\t(uuid "a")
\t)
\t(segment
\t\t(start 147.100000 148.787500)
\t\t(end 147.200000 148.800000)
\t\t(width 0.127)
\t\t(layer "In2.Cu")
\t\t{net_b}
\t\t(uuid "b")
\t)
)'''


def _mods():
    return [
        {'start': (147.1, 148.7875), 'end': (147.2, 148.8), 'net_id': 2,
         'net_name': '/A', 'old_layer': 'In1.Cu', 'new_layer': 'In2.Cu'},
        {'start': (147.1, 148.7875), 'end': (147.2, 148.8), 'net_id': 1,
         'net_name': '/B', 'old_layer': 'In2.Cu', 'new_layer': 'In1.Cu'},
    ]


def _layers_by_net(content, name_form=True):
    if name_form:
        pairs = re.findall(r'\(layer "([^"]+)"\)\s*\n\s*\(net "([^"]+)"\)', content)
    else:
        pairs = re.findall(r'\(layer "([^"]+)"\)\s*\n\s*\(net (\d+)\)', content)
    return {net: layer for layer, net in pairs}


def run():
    fails = []

    def check(name, cond):
        print(f"  {'PASS' if cond else 'FAIL'}  {name}")
        if not cond:
            fails.append(name)

    # KiCad 10 (net "name") form: each twin gets ITS OWN net's swap.
    out, n = modify_segment_layers(_board(), _mods())
    got = _layers_by_net(out)
    check("KiCad10 twins: /A In1->In2 and /B In2->In1 (net-name matching)",
          n == 2 and got == {'/A': 'In2.Cu', '/B': 'In1.Cu'})

    # KiCad 9 numeric (net N) form: same via net-id matching.
    k9 = _board(net_a='(net 2)', net_b='(net 1)')
    out, n = modify_segment_layers(k9, _mods())
    got = _layers_by_net(out, name_form=False)
    check("KiCad9 twins: matched by numeric net id",
          n == 2 and got == {'2': 'In2.Cu', '1': 'In1.Cu'})

    # Net unmatched (reassigned by a target swap between recording and writing):
    # the coordinate fallback must pick by old_layer, not touch the twin.
    mods = [{'start': (147.1, 148.7875), 'end': (147.2, 148.8), 'net_id': 99,
             'net_name': '/RENAMED', 'old_layer': 'In1.Cu', 'new_layer': 'B.Cu'}]
    out, n = modify_segment_layers(_board(), mods)
    got = _layers_by_net(out)
    check("fallback: old_layer-filtered coord match leaves the twin alone",
          n == 1 and got == {'/A': 'B.Cu', '/B': 'In2.Cu'})

    # Chained swaps on one net: last mod wins (final layer).
    mods = [
        {'start': (147.1, 148.7875), 'end': (147.2, 148.8), 'net_id': 2,
         'net_name': '/A', 'old_layer': 'In1.Cu', 'new_layer': 'In2.Cu'},
        {'start': (147.1, 148.7875), 'end': (147.2, 148.8), 'net_id': 2,
         'net_name': '/A', 'old_layer': 'In2.Cu', 'new_layer': 'B.Cu'},
    ]
    out, n = modify_segment_layers(_board(), mods)
    got = _layers_by_net(out)
    check("chained swaps resolve to the final layer",
          got['/A'] == 'B.Cu')

    # Reversed endpoint order in the mod still matches.
    mods = [{'start': (147.2, 148.8), 'end': (147.1, 148.7875), 'net_id': 2,
             'net_name': '/A', 'old_layer': 'In1.Cu', 'new_layer': 'B.Cu'}]
    out, n = modify_segment_layers(_board(), mods)
    got = _layers_by_net(out)
    check("reversed start/end in the mod still matches", got['/A'] == 'B.Cu')

    print("=" * 60)
    if fails:
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("\n5/5 checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(run())

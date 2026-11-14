#!/usr/bin/env python3
"""check_reference_plane.py -- where signal tracks leave their reference plane (#595).

Reports, per signal net, the track length that runs over NO pour copper on
its adjacent plane layer(s), how many times a track crosses such a void, and
how many times the pour under it changes net -- as text and JSON. Read-only:
it measures a routed board (`reference_plane.reference_plane_report`) and
changes nothing.

A reference layer is a copper layer next to the track's in the stack that
carries a pour. The copper under a point is the zone's FILL (zone clearance
around foreign tracks, vias and pads included), so a slot a track or a row of
vias cuts in a plane counts like a split between two pours.

A signal net is one that is neither ground nor power
(`list_nets.power_net_evidence`). Track on a layer with no poured neighbour
is totalled apart as unreferenced.

Report-only: it exits 0 whatever it counts. A `--nets` pattern that names no
net of the board is a fault in the request: it is named and the exit is 2.

Usage:
    python3 py_tools/check_reference_plane.py board.kicad_pcb
        [--nets GLOB ...] [--step MM] [--json PATH]
"""

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing', 'combined'], 'kind': 'instrument'}

import _path  # noqa: F401  (#522: makes ../py_router importable)

import argparse
import json
import os
import sys

LIST_CAP = 20      # nets printed; the JSON carries every one


def main(argv=None) -> int:
    from reference_plane import DEFAULT_STEP
    ap = argparse.ArgumentParser(
        description='Where signal tracks leave their reference plane (report-only).')
    ap.add_argument('board')
    ap.add_argument('--nets', nargs='+', action='extend', metavar='GLOB',
                    help='only the tracks of these nets (route.py --nets glob '
                         'syntax); default every signal net')
    ap.add_argument('--step', type=float, default=DEFAULT_STEP, metavar='MM',
                    help=f'sample each track every MM (default {DEFAULT_STEP})')
    ap.add_argument('--json', metavar='PATH', help='write the report here')
    a = ap.parse_args(argv)
    if a.step <= 0:
        ap.error('--step must be positive')
    if not os.path.isfile(a.board):
        print(f"board not found: {a.board}", file=sys.stderr)
        return 3

    from kicad_parser import parse_kicad_pcb
    from net_queries import (account_net_patterns, board_net_names,
                             matches_net_filter, no_net_patterns_note)
    from reference_plane import reference_plane_report
    pcb = parse_kicad_pcb(a.board)
    net_ids, no_net = None, []
    if a.nets:
        want = {n for n in board_net_names(pcb) if matches_net_filter(n, a.nets)}
        net_ids = {nid for nid, n in pcb.nets.items() if n.name in want}
        no_net = account_net_patterns(a.nets, pcb)[0]
    rep = reference_plane_report(pcb, net_ids=net_ids, step=a.step)
    rep = {'board': os.path.abspath(a.board), 'nets': a.nets or None,
           'patterns_matching_no_net': no_net, **rep}

    if no_net:
        print(no_net_patterns_note(no_net, '--nets')
              + " -- the count below covers only the nets the other patterns name")
    t = rep['totals']
    if not rep['reference_layers']:
        print("Reference planes: no copper layer has a poured neighbour; nothing "
              "to measure")
    else:
        print(f"Reference planes: {t['crossings']} void crossing(s), "
              f"{t['reference_changes']} reference change(s), {t['void_mm']:.3f} mm "
              f"of {t['length_mm']:.3f} mm of referenced signal track over no pour "
              f"copper" + (f"; {t['unreferenced_mm']:.3f} mm on layers with no poured "
                           f"neighbour" if t['unreferenced_mm'] else ''))
        rows = sorted(((n, r) for n, r in rep['by_net'].items()
                       if r['crossings'] or r['reference_changes'] or r['void_mm']),
                      key=lambda kv: (-kv[1]['crossings'], -kv[1]['reference_changes'],
                                      -kv[1]['void_mm'], kv[0]))
        for name, r in rows[:LIST_CAP]:
            print(f"  {name}: {r['crossings']} crossing(s), {r['reference_changes']} "
                  f"change(s), {r['void_mm']:.3f} mm over no copper")
        if len(rows) > LIST_CAP:
            print(f"  ... {len(rows) - LIST_CAP} more net(s)"
                  + (" (every one in --json)" if a.json else " (--json lists every one)"))
        for z in rep['unmodelled_zones']:
            print(f"  NOT MEASURED: the fill of {z} could not be modelled; copper "
                  f"under tracks there reads as none")
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump(rep, f, indent=2)
    return 2 if no_net else 0


if __name__ == '__main__':
    sys.exit(main())

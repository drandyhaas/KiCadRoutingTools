#!/usr/bin/env python3
"""check_return_vias.py -- ground return-via coverage of signal vias (#1816).

Reports every signal via with no ground via or drilled ground pad of its OWN
ground domain within --distance mm, per net, as text and JSON. Read-only, and
the same test the step that adds return vias skips a covered via with
(`route_planes.py --add-gnd-vias`, the Planes tab): both call
`return_vias.return_via_coverage` / `ReturnViaModel`.

A signal via is a via on a net that is neither ground (every ground domain)
nor power (`list_nets.power_net_evidence`: a rail name, power pins on two or
more parts, or a pour the board draws for the net). The nets left out as
power are named, each with its via count and the reason it reads as power.

Report-only: it exits 0 whatever it counts. A `--nets` pattern that names no
net of the board is a fault in the request, not a count: it is named and the
exit is 2 (#1815), since `0 of 0` would read as full coverage. A raw count
over-reads in two known cases, which it states: vias inside a BGA field, and
a layer change between two layers that share one reference plane (no return
via is needed there; the stackup is not consulted).

Usage:
    python3 py_tools/check_return_vias.py board.kicad_pcb
        [--nets GLOB ...] [--distance MM] [--gnd-via-net NAME] [--json PATH]
"""

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing', 'combined'], 'kind': 'instrument'}

import _path  # noqa: F401  (#522: makes ../py_router importable)

import argparse
import json
import os
import sys

CAVEAT = ('a raw count over-reads vias inside a BGA field, and a layer change '
          'between two layers that share one reference plane (no return via '
          'is needed there; the stackup is not consulted)')
LIST_CAP = 20      # nets printed; the JSON carries every one


def main(argv=None) -> int:
    from routing_defaults import GND_VIA_DISTANCE
    ap = argparse.ArgumentParser(
        description='Ground return-via coverage of signal vias (report-only).')
    ap.add_argument('board')
    ap.add_argument('--nets', nargs='+', action='extend', metavar='GLOB',
                    help='only the vias of these nets (route.py --nets glob '
                         'syntax); default every signal via')
    ap.add_argument('--distance', type=float, default=GND_VIA_DISTANCE,
                    metavar='MM',
                    help=f'a return point within this distance covers a via '
                         f'(default {GND_VIA_DISTANCE}, the --add-gnd-vias '
                         f'step\'s --gnd-via-distance default)')
    ap.add_argument('--gnd-via-net', default='', metavar='NAME',
                    help='pin every return to this ground net, as '
                         '--gnd-via-net does for the step; default each via\'s '
                         'own ground domain')
    ap.add_argument('--json', metavar='PATH', help='write the report here')
    a = ap.parse_args(argv)
    if a.distance <= 0:
        ap.error('--distance must be positive')
    if not os.path.isfile(a.board):
        print(f"board not found: {a.board}", file=sys.stderr)
        return 3

    from kicad_parser import parse_kicad_pcb
    from net_queries import (account_net_patterns, board_net_names,
                             matches_net_filter, no_net_patterns_note)
    from return_vias import return_via_coverage
    pcb = parse_kicad_pcb(a.board)
    net_ids, no_net = None, []
    if a.nets:
        want = {n for n in board_net_names(pcb) if matches_net_filter(n, a.nets)}
        net_ids = {nid for nid, n in pcb.nets.items() if n.name in want}
        # #1815: each pattern is accounted on its own. One naming no net
        # selects nothing, and nothing selected reads `0 of 0`.
        no_net = account_net_patterns(a.nets, pcb)[0]
    rep = return_via_coverage(pcb, a.distance, net_ids=net_ids,
                              gnd_net_name=a.gnd_via_net)
    rep = {'board': os.path.abspath(a.board), 'nets': a.nets or None,
           'patterns_matching_no_net': no_net, **rep, 'caveat': CAVEAT}

    if no_net:
        print(no_net_patterns_note(no_net, '--nets')
              + " -- the count below covers only the nets the other "
                "patterns name")
    if rep['uncovered'] is None:
        print(f"Return vias NOT GRADED: {rep['reason']}")
    else:
        print(f"Return vias: {rep['uncovered']} of {rep['signal_vias']} signal "
              f"via(s) have no {rep['gnd_net']}-domain via or drilled pad "
              f"within {a.distance:g} mm"
              + (f" ({rep['power_vias']} power-rail via(s) left out)"
                 if rep['power_vias'] else ''))
        rows = sorted(((n, r) for n, r in rep['by_net'].items()
                       if r['uncovered']),
                      key=lambda kv: (-kv[1]['uncovered'], kv[0]))
        for name, row in rows[:LIST_CAP]:
            print(f"  {name}: {row['uncovered']} of {row['signal_vias']}")
        if len(rows) > LIST_CAP:
            print(f"  ... {len(rows) - LIST_CAP} more net(s)"
                  + (" (every one in --json)" if a.json else
                     " (--json lists every one)"))
        left = sorted(rep['power_nets'].items(),
                      key=lambda kv: (-kv[1]['vias'], kv[0]))
        if left:
            print(f"Left out as power, not examined: {len(left)} net(s)")
            for name, row in left[:LIST_CAP]:
                print(f"  {name}: {row['vias']} via(s) -- {row['why']}")
            if len(left) > LIST_CAP:
                print(f"  ... {len(left) - LIST_CAP} more net(s)"
                      + (" (every one in --json)" if a.json else
                         " (--json lists every one)"))
        if rep['uncovered']:
            print(f"Note: {CAVEAT}.")
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump(rep, f, indent=2)
    return 2 if no_net else 0


if __name__ == '__main__':
    sys.exit(main())

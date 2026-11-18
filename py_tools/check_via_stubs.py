#!/usr/bin/env python3
"""check_via_stubs.py -- the stub each via leaves past the layers it joins (#1925).

Reports, per via of the nets asked for, the length of barrel left on each
side of the layers its net uses (from the outermost layer the net's copper
joins the via on, to the via's end, through the stackup), the longest stub
per net, and which vias exceed --max-stub. Back-drills the board already
declares (KiCad 10's `backdrill` / `tertiary_drill`) are taken off the
barrel. Read-only: it changes no board and proposes no back-drill. The
measure is `via_stubs.via_stub_report`, which says what a stub is.

Report-only: it exits 0 whatever it counts. A `--nets` pattern that names no
net of the board is a fault in the request, not a count: it is named and the
exit is 2, since `0 of 0` would read as no stub. A board with no stackup has
no stub LENGTH: the layers each stub passes are still listed, and the report
says it is NOT GRADED in mm.

Usage:
    python3 py_tools/check_via_stubs.py board.kicad_pcb
        [--nets GLOB ...] [--max-stub MM] [--json PATH]
"""

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing', 'combined'], 'kind': 'instrument'}

import _path  # noqa: F401  (#522: makes ../py_router importable)

import argparse
import json
import os
import sys

NET_CAP = 20       # nets printed; the JSON carries every one
VIA_CAP = 10       # vias printed per net, longest stub first


def _mm(value) -> str:
    return '-' if value is None else f'{value:.3f}'


def _via_line(row) -> str:
    notes = []
    if row['backdrills']:
        notes.append('back-drilled ' + ', '.join(
            f"{bd['start_layer']}..{bd['end_layer']} at {bd['size']:g}"
            for bd in row['backdrills'].values()))
    if row['unjoined']:
        notes.append('joined on no layer')
    if row['joined_on_drilled']:
        notes.append('JOINED ON A BACK-DRILLED LAYER: '
                     + ', '.join(row['joined_on_drilled']))
    if row['over']:
        notes.append('OVER')
    joined = '/'.join(row['joined']) or 'none'
    return (f"    ({row['x']:.3f}, {row['y']:.3f}) {'-'.join(row['span'])} "
            f"joined {joined}: top {_mm(row['stub_top_mm'])} mm, "
            f"bottom {_mm(row['stub_bottom_mm'])} mm"
            + (f"  [{'; '.join(notes)}]" if notes else ''))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description='Via stub lengths through the stackup (report-only).')
    ap.add_argument('board')
    ap.add_argument('--nets', nargs='+', action='extend', metavar='GLOB',
                    help='only the vias of these nets (route.py --nets glob '
                         'syntax); default every via')
    ap.add_argument('--max-stub', type=float, default=None, metavar='MM',
                    help='name the vias whose longer stub exceeds this; no '
                         'default, since the length that matters is the '
                         'interface\'s')
    ap.add_argument('--json', metavar='PATH', help='write the report here')
    a = ap.parse_args(argv)
    if a.max_stub is not None and a.max_stub < 0:
        ap.error('--max-stub must not be negative')
    if not os.path.isfile(a.board):
        print(f"board not found: {a.board}", file=sys.stderr)
        return 3

    from kicad_parser import parse_kicad_pcb
    from net_queries import (account_net_patterns, board_net_names,
                             matches_net_filter, no_net_patterns_note)
    from via_stubs import via_stub_report
    pcb = parse_kicad_pcb(a.board)
    net_ids, no_net = None, []
    if a.nets:
        want = {n for n in board_net_names(pcb) if matches_net_filter(n, a.nets)}
        net_ids = {nid for nid, n in pcb.nets.items() if n.name in want}
        no_net = account_net_patterns(a.nets, pcb)[0]
    rep = via_stub_report(pcb, net_ids=net_ids, max_stub=a.max_stub)
    rep = {'board': os.path.abspath(a.board), 'nets': a.nets or None,
           'patterns_matching_no_net': no_net, **rep}

    if no_net:
        print(no_net_patterns_note(no_net, '--nets')
              + " -- the report below covers only the nets the other "
                "patterns name")
    total = len(rep['vias'])
    if rep['reason']:
        print(f"Via stubs NOT GRADED in mm: {rep['reason']} "
              f"({total} via(s); the layers each stub passes are in --json)")
    else:
        line = (f"Via stubs: {rep['measured']} of {total} via(s) measured"
                + (f", {rep['backdrilled_vias']} back-drilled"
                   if rep['backdrilled_vias'] else ''))
        if a.max_stub is not None:
            line += f", {rep['over']} over {a.max_stub:g} mm"
        print(line)
    by_net = {}
    for row in rep['vias']:
        by_net.setdefault(row['net'], []).append(row)
    order = sorted(by_net, key=lambda n: (-(rep['by_net'][n]['longest_mm'] or -1.0), n))
    more = " (every one in --json)" if a.json else " (--json lists every one)"
    for name in order[:NET_CAP]:
        net = rep['by_net'][name]
        print(f"  {name or '(no net)'}: {net['vias']} via(s), longest stub "
              f"{_mm(net['longest_mm'])} mm"
              + (f", {net['over']} over" if net['over'] else ''))
        rows = sorted(by_net[name], key=lambda r: -(r['longest_mm'] or -1.0))
        for row in rows[:VIA_CAP]:
            print(_via_line(row))
        if len(rows) > VIA_CAP:
            print(f"    ... {len(rows) - VIA_CAP} more via(s)" + more)
    if len(order) > NET_CAP:
        print(f"  ... {len(order) - NET_CAP} more net(s)" + more)
    if rep['pad_backdrills']:
        print(f"Drilled pads declaring a back-drill: {len(rep['pad_backdrills'])} "
              f"(a pad's stub is not measured)")
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump(rep, f, indent=2)
    return 2 if no_net else 0


if __name__ == '__main__':
    sys.exit(main())

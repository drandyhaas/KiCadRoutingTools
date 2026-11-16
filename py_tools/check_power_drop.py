#!/usr/bin/env python3
"""check_power_drop.py -- voltage drop per load and current per via of routed
power nets, from stated load currents (#1847).

For each net of the loads file, the net's tracks, vias and copper pours are
solved as one resistor network with every load drawing at once
(`power_drop.solve_net`):
the millivolts each load pin loses, and the current in each via and track
segment against its IPC rating. Read-only, and report-only: what it finds,
items over their rating included, never changes the exit code.

Exit codes: 0 the report is complete; 1 the report is INCOMPLETE -- a stated
load has no copper path to a source or is not a pad of its net, a net has no
source, or a net is not on the board (the other loads are solved as if the
left-out ones drew nothing, so their drops read low); 2 the loads file cannot
be read; 3 the board or the loads file is missing.

The loads file states what a board file cannot:

    {"nets": {"+3V3": {"sources": ["J1.1"],
                       "loads": {"U1.1": 0.5, "U2.1": 0.3}}}}

(pad references as REF.PAD, every pad of that part and number as one
terminal; currents in A). A pour is solved as a mesh of
its fill (--pour-mesh-mm). What the solve leaves out is in the report:
thermal-relief spokes (a pad's contact with a pour is taken as solid), a
pour with no fill model, and footprint graphic copper. A via's resistance
and rating rest on an ASSUMED barrel plating (--via-plating-um, default 25).

Usage:
    python3 py_tools/check_power_drop.py board.kicad_pcb --loads loads.json
        [--via-plating-um UM] [--pour-mesh-mm MM] [--temp-rise C] [--json PATH]
"""

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing', 'combined'], 'kind': 'instrument'}

import _path  # noqa: F401  (#522: makes ../py_router importable)

import argparse
import json
import os
import sys

LIST_CAP = 12      # segments printed per net; the JSON carries every one


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description='Voltage drop per load and current per via of routed power '
                    'nets, from stated load currents (report-only).')
    ap.add_argument('board')
    ap.add_argument('--loads', required=True, metavar='JSON',
                    help='the sources and load currents per net: {"nets": {"<net>": '
                         '{"sources": ["REF.PAD"], "loads": {"REF.PAD": amps}}}}')
    ap.add_argument('--via-plating-um', type=float, default=None, metavar='UM',
                    help='barrel plating thickness assumed for a via\'s resistance '
                         'and rating (default 25)')
    ap.add_argument('--pour-mesh-mm', type=float, default=None, metavar='MM',
                    help='side of the mesh cell a copper pour is solved on '
                         '(default 0.25)')
    ap.add_argument('--temp-rise', type=float, default=10.0, metavar='C',
                    help='temperature rise the IPC ratings are read at (default 10)')
    ap.add_argument('--json', metavar='PATH', help='write the report here')
    a = ap.parse_args(argv)
    if a.via_plating_um is not None and a.via_plating_um <= 0:
        ap.error('--via-plating-um must be positive')
    if a.temp_rise <= 0:
        ap.error('--temp-rise must be positive')
    if a.pour_mesh_mm is not None and a.pour_mesh_mm <= 0:
        ap.error('--pour-mesh-mm must be positive')
    for what, path in (('board', a.board), ('loads file', a.loads)):
        if not os.path.isfile(path):
            print(f"{what} not found: {path}", file=sys.stderr)
            return 3

    import power_drop
    try:
        nets = power_drop.read_loads(a.loads)
    except (ValueError, OSError) as e:
        print(f"loads file {a.loads}: {e}", file=sys.stderr)
        return 2
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(a.board)
    plating = (a.via_plating_um / 1000.0 if a.via_plating_um is not None
               else power_drop.DEFAULT_VIA_PLATING_MM)
    by_name = {n.name: nid for nid, n in pcb.nets.items() if n.name}
    rep = {'board': os.path.abspath(a.board), 'loads_file': os.path.abspath(a.loads),
           'via_plating_mm': plating, 'via_plating_assumed': a.via_plating_um is None,
           'temp_rise_c': a.temp_rise, 'nets': [], 'unknown_nets': []}
    for name, spec in nets.items():
        if name not in by_name:
            rep['unknown_nets'].append(name)
            continue
        rep['nets'].append(power_drop.solve_net(
            pcb, by_name[name], spec['sources'], spec['loads'],
            via_plating_mm=plating, temp_rise_c=a.temp_rise,
            pour_mesh_mm=(a.pour_mesh_mm if a.pour_mesh_mm is not None
                          else power_drop.DEFAULT_POUR_MESH_MM)))

    print(f"Power drop (#1847): {len(rep['nets'])} net(s), all loads drawing at once; "
          f"via barrels at {plating * 1000:g} um plating"
          f"{' (assumed)' if a.via_plating_um is None else ''}, ratings at "
          f"{a.temp_rise:g} C rise")
    rep['complete'] = (not rep['unknown_nets']
                       and all(r['complete'] for r in rep['nets']))
    for name in rep['unknown_nets']:
        print(f"  {name}: INCOMPLETE -- not a net of this board, not solved")
    for r in rep['nets']:
        worst = (f", worst drop {r['max_drop_mv']:.1f} mV" if r['max_drop_mv'] is not None
                 else '')
        if r['complete']:
            head = f"  {r['net']}: {r['total_current_a']:g} A solved{worst}"
        else:
            head = (f"  {r['net']}: INCOMPLETE -- {r['total_current_a']:g} of "
                    f"{r['stated_current_a']:g} A solved{worst}"
                    + (" over the solved loads alone, so it reads low"
                       if worst else ''))
        print(head + (f", {r['over']} item(s) over their rating" if r['over'] else ''))
        for why in r['incomplete']:
            print(f"    INCOMPLETE: {why}")
        for e in r['errors']:
            print(f"    ERROR: {e}")
        for row in r['loads']:
            if row.get('unreached'):
                print(f"    {row['pad']}: {row['current_a']:g} A -- NOT SOLVED, no "
                      f"copper path to a source")
            else:
                print(f"    {row['pad']}: {row['current_a']:g} A, drop {row['drop_mv']:.1f} mV")
        for v in sorted(r['vias'], key=lambda v: -v['current_a']):
            print(f"    via ({v['at'][0]:.2f},{v['at'][1]:.2f}) {v['type']} "
                  f"{v['size_mm']:g}/{v['drill_mm']:g}: {v['current_a']:.3f} A "
                  f"of {v['rating_a']:.2f} A" + ('  OVER' if v['over'] else ''))
        segs = sorted(r['segments'], key=lambda s: (not s['over'], -s['current_a']))
        for s in segs[:LIST_CAP]:
            print(f"    track {s['layer']} {s['width_mm']:g} mm ({s['start'][0]:.2f},"
                  f"{s['start'][1]:.2f})-({s['end'][0]:.2f},{s['end'][1]:.2f}): "
                  f"{s['current_a']:.3f} A of {s['rating_a']:.2f} A"
                  + ('  OVER' if s['over'] else ''))
        if len(segs) > LIST_CAP:
            print(f"    ... {len(segs) - LIST_CAP} more track segment(s)"
                  + (" (every one in --json)" if a.json else " (--json lists every one)"))
        for pour in r['pours']:
            print(f"    pour {pour['layer']}: solved on {pour['cells']} cell(s) of "
                  f"{pour['mesh_mm']:g} mm, {pour['sheet_mohm_per_sq']:g} mOhm/sq")
        for note in r['not_modelled']:
            print(f"    NOT IN THE SOLVE: {note}")
    if not rep['complete']:
        print("REPORT INCOMPLETE: a stated load or net was left out of the solve "
              "(exit 1)")
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump(rep, f, indent=2)
    return 0 if rep['complete'] else 1


if __name__ == '__main__':
    sys.exit(main())

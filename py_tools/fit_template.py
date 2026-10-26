#!/usr/bin/env python3
"""Fit a datasheet layout sketch to a board's footprints: a `templates[]` row.

    fit_template.py BOARD.kicad_pcb --pins U2
    fit_template.py BOARD.kicad_pcb --sketch SKETCH.json [-o ROW.json] [--render ROW.png]
                    [--intent INTENT.json --write-intent OUT.json]

`--pins REF` prints a part's pads as the sketch needs them: number, pin
function, position in the footprint's own frame and the side of the
footprint it is on, net, and the other parts on that net with their values
and pads -- what matching the datasheet's drawing to the board takes. It
takes any part, to walk a net from part to part. A net with more than 20
parts (a ground, a board-wide rail) prints as a count; with `--groups
GROUPS.json` it still lists the parts of REF's own group on it (a buck's
output caps on its 3V3 rail).

SKETCH names the chip (`anchor`) and its support parts in the datasheet
drawing's order, nearest the chip first, each with `pads`: its own pads, and
per partner (the chip, or a member listed earlier) the partner's pads it
belongs beside -- matched by net, so their order does not matter -- as an
intent `proximity[]` row's `pads`. Optionally a member gives the `side` of the
chip it sits on (`-x`, `+x`, `-y`, `+y` in the footprint's own frame: its
centre beyond that courtyard edge; `+x-y` for a corner, beyond both) or its
quarter turn (`rot`, relative to the chip):

    {"anchor": "U2", "source": "LMR36006 SNVSB48C Figure 57",
     "members": [
       {"ref": "C2", "pads": {"C2": ["1", "2"], "U2": ["2", "1"]}, "side": "+y"},
       {"ref": "L1", "pads": {"L1": ["1"], "U2": ["12"]}, "side": "-x"},
       {"ref": "C7", "pads": {"C7": ["2"], "L1": ["2"]}}]}

Each member is seated as close to its partner pads as the courtyards already
seated allow -- kept the board's clearance apart, the seat gate's courtyard
rule (`gap_mm` in the sketch for another) -- within `--reach` mm (4, or the
member's own `reach`) of them, at the quarter turn that brings its pads
closest (`placement.templates.fit`). A LOCKED chip's locked neighbours are
where the board has them: obstacles, and partners a member may name. Each
member prints with each leg's gap and its gap to the chip; a NOTE names a
member with one `side` that ended beside another edge.
A member that cannot be seated stops the fit with the reason (its side, or
the members in its way, named), and `--render` then draws the members seated
before it. A part with no courtyard is fitted on the estimated courtyard
place_simple seats it on -- its body and pads (#1300) -- and the run names
it: check that box against the part's real body. The row gives each
member's pose in the chip's frame, the chip at the origin unturned, and the
pad-edge gaps the fit reached (`fitted.legs_mm`, one per named pad of the
member, in the order of its pads; `legs_gap_mm` sums them). `--render` draws the
fitted template in the chip's frame -- courtyards, pads, the chip's pad
numbers, each member's legs -- to set beside the datasheet's drawing.
A sketch's `kind` (`switching stage` or `rf front end`, as the groups file
names the candidate) is copied to the row. For a switching stage the fit
prints each member cap's ground return -- the pad-edge gaps from its
ground pad, through other members' pads on that ground, to the chip's
ground pins or a member transistor's (`placement.templates.ground_returns`)
-- and a NOTE for one past `ground_return_mm` (3 mm, or the sketch's own),
which the grade then warns about (`template_ground_return`).
`--intent` also measures each member at its template pose against that
intent's decap limits (`decaps.max_distance_mm`, `max_pin_distance_mm`, the
ones it grades at error) and WARNS by name for one past a limit
(`fitted.decap_limits`): a row held as drawn would fail the grade.
`--write-intent`
writes INTENT with the row added to its `templates[]` (a row for the same
anchor is replaced, so a re-fit may write over its own input) and
`min_reader` raised to the version that reads it.

Exit 0 fitted, 2 an unreadable board, sketch or intent, or a sketch the
board cannot fit.
"""
#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['placement'], 'kind': 'instrument'}

import argparse
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'py_placer'):
    sys.path.insert(0, os.path.join(ROOT, _d))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('board')
    ap.add_argument('--pins', metavar='REF', help="print REF's pads for matching, and exit")
    ap.add_argument('--groups', metavar='JSON', help="with --pins: list REF's group's parts "
                    "on a rail too")
    ap.add_argument('--sketch', help='the sketch JSON')
    ap.add_argument('--render', metavar='PNG', help='draw the fitted template here')
    ap.add_argument('--reach', type=float, default=4.0, metavar='MM',
                    help='how far from its partner pads a member is sought (default 4)')
    ap.add_argument('-o', '--out', help='write the fitted row here')
    ap.add_argument('--intent', help='an intent to add the row to (with --write-intent)')
    ap.add_argument('--write-intent', metavar='OUT', help='write INTENT plus the row here')
    a = ap.parse_args(argv)
    if bool(a.intent) != bool(a.write_intent):
        ap.error('--intent and --write-intent go together')
    if not a.pins and not a.sketch:
        ap.error('give --pins REF or --sketch SKETCH.json')
    import pose_score
    from kicad_parser import parse_kicad_pcb
    from list_nets import board_floor_knobs
    from placement import floorplan
    from placement.parser import extract_courtyard_sides
    from placement.templates import (GROUND_RETURN_MM, FitError, TemplateError, fit,
                                     ground_returns_fitted, pin_table, render)
    if a.pins:
        try:
            pcb = parse_kicad_pcb(a.board)
            group = None
            if a.groups:
                from placement.grouping import load_groups
                group = next((set(g['refs']) for g in load_groups(a.groups)['groups']
                              if a.pins in g['refs']), set())
            rows = pin_table(pcb, a.pins, group)
        except (OSError, ValueError, KeyError) as ex:
            print(f"fit_template: {ex}", file=sys.stderr)
            return 2
        fp = pcb.footprints[a.pins]
        print(f"{a.pins} {fp.value} {fp.footprint_name} on {fp.layer}; positions in its own "
              f"frame (rotation 0, +x right, +y down):")
        for r in rows:
            on = ' '.join(f"{q['ref']}.{'/'.join(q['pads'])}[{q['value']}]"
                          + ('@' + q['layer'][:1] if q.get('layer') and q['layer'] != fp.layer else '')
                          for q in r.get('parts') or ())
            if 'rail' in r:
                on = f"rail, {r['rail']} parts" + (f"; of its group: {on}" if on else '')
            print(f"  pad {r['pad']:>4} {r['function'][:10]:10s} ({r['at'][0]:+7.3f}, "
                  f"{r['at'][1]:+7.3f}) {r['side']:8s} {r['net'][:28]:28s} {on}")
        print('JSON_SUMMARY: ' + json.dumps({'ref': a.pins, 'pads': rows}))
        return 0
    try:
        sketch = json.load(open(a.sketch, encoding='utf-8'))
        pcb = parse_kicad_pcb(a.board)
        clearance, edge, _ = board_floor_knobs(a.board)
        # #1300: a part that draws no courtyard is fitted on the ESTIMATED
        # courtyard place_simple seats it on, so the fit and the placement
        # space it alike
        state = pose_score.make_state(pcb, a.board, clearance=clearance,
                                      board_edge_clearance=edge,
                                      estimate_courtyards=True)
        names = [sketch.get('anchor')] + [m.get('ref') for m in sketch.get('members') or ()]
        crt = extract_courtyard_sides(a.board)
        boxed = [r for r in names if r in pcb.footprints and r not in crt]
        if boxed:
            print(f"  NOTE: no courtyard on {', '.join(boxed)}: each is fitted (and placed) on "
                  f"an estimated one, its body and pads (#1300) -- check it against the "
                  f"part's real body in the render",
                  flush=True)
        row = fit(pcb, state, sketch, reach=a.reach)
    except FitError as ex:
        print(f"fit_template: {ex}", file=sys.stderr)
        for m in ex.partial['members']:
            print(f"  seated {m['ref']:6s} at ({m['at'][0]:+7.3f}, {m['at'][1]:+7.3f}) "
                  f"rot {m['rot']:5.1f}", file=sys.stderr)
        if a.render:
            render(pcb, state, ex.partial, a.render, boxed=boxed)
            print(f"  drew the members seated so far: {a.render}", file=sys.stderr)
        return 2
    except (OSError, ValueError, TemplateError) as ex:
        print(f"fit_template: {ex}", file=sys.stderr)
        return 2
    for m in row['members']:
        legs = row['fitted']['legs_mm'][m['ref']]
        print(f"  {m['ref']:6s} at ({m['at'][0]:+7.3f}, {m['at'][1]:+7.3f}) rot {m['rot']:5.1f}"
              f"  legs {' + '.join(f'{g:.2f}' for g in legs)} mm"
              f"  ({row['fitted']['off_chip_mm'][m['ref']]:.2f} mm off the chip)")
    for n in row['fitted'].get('notes') or ():
        print(f"  NOTE: {n}")
    if row.get('kind') == 'switching stage':
        limit = float(row.get('ground_return_mm') or GROUND_RETURN_MM)
        caps = ground_returns_fitted(pcb, state, row)
        row['fitted']['ground_return_mm'] = {c['ref']: c['return_mm'] for c in caps}
        for c in caps:
            print(f"  ground return {c['ref']:6s} {c['return_mm']:5.2f} mm to {c['to']}"
                  + (f" via {', '.join(c['via'])}" if c['via'] else ''))
        for c in caps:
            if c['return_mm'] > limit:
                print(f"  NOTE: {c['ref']} returns its ground {c['return_mm']:.2f} mm from the "
                      f"chip's (over {limit:g}): give its ground pad a partner beside the "
                      f"chip's ground, as the drawing joins them (an output cap names an "
                      f"input cap's ground pad)")
    elif not row.get('kind'):
        print("  NOTE: the sketch has no `kind`: give it the groups file's (`switching stage` "
              "or `rf front end`); a switching stage's caps have their ground return checked")
    near = [o for o in row['fitted'].get('context') or () if o not in crt]
    if near:
        boxed = boxed + near
        print(f"  NOTE: no courtyard on the locked neighbour(s) {', '.join(near)}: each is an "
              f"obstacle on its estimated courtyard (#1300) -- check it against its real body")
    if row['fitted'].get('context'):
        print(f"  locked neighbours of the locked chip, as obstacles and partners: "
              f"{', '.join(row['fitted']['context'])}")
    # the row against the decap limits of the intent it is written into
    # (#1416): a member held past a limit the grade counts at ERROR fails every
    # placement that holds the row as drawn
    over = []
    if a.intent:
        from placement.templates import decap_limit_findings
        try:
            over = decap_limit_findings(pcb, a.board, state, row,
                                        floorplan.load_intent(a.intent))
        except (OSError, ValueError) as ex:
            print(f"  NOTE: the intent's decap limits were not checked against the row: {ex}")
        for f in over:
            print(f"  WARNING: {f['ref']} at its template pose reads {f['value']:.2f} mm against "
                  f"the intent's {f['rule']} limit of {f['limit']:.2f} mm, which it grades "
                  f"ERROR: a placement holding the row as drawn fails it. Seat the member "
                  f"nearer the chip in the sketch, or change the limit")
        row['fitted']['decap_limits'] = over
    if a.out:
        with open(a.out, 'w', encoding='utf-8') as fh:
            json.dump(row, fh, indent=1)
    if a.render:
        render(pcb, state, row, a.render, boxed=boxed)
    if a.write_intent:
        try:
            doc = json.load(open(a.intent, encoding='utf-8'))
        except (OSError, ValueError) as ex:
            print(f"fit_template: {ex}", file=sys.stderr)
            return 2
        rows = [t for t in doc.get('templates') or () if t.get('anchor') != row['anchor']]
        doc['templates'] = rows + [row]
        doc['min_reader'] = max(int(doc.get('min_reader') or 0), floorplan.TEMPLATES_READER,
                                *(floorplan.TEMPLATE_KIND_READER for t in doc['templates']
                                  if 'kind' in t or 'ground_return_mm' in t))
        with open(a.write_intent, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh, indent=1)
        try:
            floorplan.load_intent(a.write_intent)
        except ValueError as ex:
            print(f"fit_template: the written intent does not load: {ex}", file=sys.stderr)
            return 2
    print(f"JSON_SUMMARY: {json.dumps({'anchor': row['anchor'], 'members': len(row['members']), 'legs_gap_mm': row['fitted']['legs_gap_mm'], 'decap_over': sorted({f['ref'] for f in over})})}")
    return 0


if __name__ == '__main__':
    sys.exit(main())

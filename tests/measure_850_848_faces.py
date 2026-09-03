#!/usr/bin/env python3
"""Regenerate every number the #850 / #848 PR states, from the engine as it is.

Asserts nothing, and is deliberately not named `test_*`: `tests/run_all.py`
globs `test_*.py` and this is a report, not a gate. The gates are
`tests/test_850_demand_face_of.py` and `tests/test_848_side_obstruction.py`.

BOTH ARMS ARE MEASURED IN ONE PROCESS. This file carries its own copy of the
rule each change replaced -- the pad-CENTRE-against-extent face rule for #850,
the whole-part neighbour rect for #848 -- and prints before and after side by
side. So no table here depends on checking out a parent commit, and none of
them goes stale when the next change lands: the "before" column re-measures
too.

    python -X utf8 tests/measure_850_848_faces.py                 # all tables
    python -X utf8 tests/measure_850_848_faces.py --table demand
    python -X utf8 tests/measure_850_848_faces.py --out after.json
    python -X utf8 tests/measure_850_848_faces.py --diff before.json after.json
    python -X utf8 tests/measure_850_848_faces.py --boards ulx3s tigard

ONE BASIS, and it is stated rather than defaulted: clearance 0.2 / track 0.2 /
grid 0.05. `legality.part_copper_geometry`'s NPTH hole extents carry
`max(0, NPTH_TO_TRACK_CLEARANCE - clearance)`, so below 0.20mm the boxes grow
and a census taken at another clearance is a different census. `check_channels`
runs the corpus at 0.09, squarely inside that regime.

THE NEGATIVE CONTROL is the `int_R` / `int_E` pair in the demand table: the
interior-pad count each ledger reports for the same ref, printed adjacent, with
a WARN line whenever they differ. #841's failure was 244 pads going interior on
one board -- demand 0, deficit 0, and a sweep of green numbers on an instrument
that had stopped looking. Here that shows up as one of two adjacent columns
running away from the other, on the same page, rather than as a deficit that
merely improved.
"""
import argparse
import json
import os
import subprocess
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_HERE)
for _p in (_HERE, ROOT, os.path.join(ROOT, 'py_placer'),
           os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import run_utils                                             # noqa: E402
from kicad_parser import parse_kicad_pcb                     # noqa: E402
from placement import escape as E                            # noqa: E402
from placement import legality as L                          # noqa: E402
from placement import routability as R                       # noqa: E402

CLEARANCE, TRACK, GRID = 0.2, 0.2, 0.05
LANE = dict(clearance=CLEARANCE, track_width=TRACK, grid_step=GRID)
SKIP_EXIT = 77

#: Which keys `--diff` compares. Per-board scalars only: a per-pad census does
#: not have a stable identity across corpus changes, and a diff that reports
#: "this ref appeared" on every board addition is noise.
DIFF_KEYS = ('refs', 'demand_before', 'demand_after', 'deficit_before',
             'deficit_after', 'interior_routability', 'interior_escape',
             'faces_risen', 'starved_before', 'starved_after',
             'side_boxes_differ', 'side_charged', 'deficit_848_before',
             'deficit_848_after', 'escape_deficit')


def _old_face_rule(fp, geom, *, lane_mm, fallback_rect=None):
    """`face_lane_ledger`'s face rule as it stood at e239e067.

    `min` over |pad CENTRE - extent edge|: no tolerance, no interior bucket, so
    every pad lands on some face. `geom` is ignored on purpose -- the old rule
    never looked at the copper box, which is the whole point.
    """
    ext = fallback_rect
    out = []
    for pad in (fp.pads or []):
        d = {'north': abs(pad.global_y - ext[1]),
             'south': abs(pad.global_y - ext[3]),
             'west': abs(pad.global_x - ext[0]),
             'east': abs(pad.global_x - ext[2])}
        out.append((pad, min(d, key=d.get)))
    return E.FaceAssignment(faces=tuple(out), pitch_mm=0.0,
                            pitch_source='measure_old_rule')


def _whole_part_rect(geom, sides=None):
    """`rect_on_sides` as it stood before #848: the whole pad box, always."""
    return geom.rect


def _sweep(pcb, path, ctx, refs):
    return {r: R.face_lane_ledger(pcb, r, pcb_file=path, context=ctx, **LANE)
            for r in refs}


def _totals(rows_by_ref):
    dem = sum(r['demand_nets'] for rows in rows_by_ref.values() for r in rows)
    dfc = sum(r['deficit_finest_grid']
              for rows in rows_by_ref.values() for r in rows)
    return dem, dfc


def _starved(rows_by_ref, min_demand=7):
    return sum(1 for rows in rows_by_ref.values() for r in rows
               if r['supply_finest_grid'] == 0
               and r['demand_nets'] >= min_demand)


def _toggle(fn_owner, name, replacement, thunk):
    real = getattr(fn_owner, name)
    setattr(fn_owner, name, replacement)
    try:
        return thunk()
    finally:
        setattr(fn_owner, name, real)


def measure(path):
    """Every per-board figure, both arms, one parse."""
    pcb = parse_kicad_pcb(path)
    ctx = R.board_lane_context(pcb, CLEARANCE, pcb_file=path)
    refs = list(E.fine_pitch_parts(pcb))
    fps = pcb.footprints or {}

    now = _sweep(pcb, path, ctx, refs)
    before850 = _toggle(E, 'assign_faces', _old_face_rule,
                        lambda: _sweep(pcb, path, ctx, refs))
    # `rect_on_sides` lives in `legality` and `face_lane_ledger` imports it
    # lazily from there inside its own body, so THAT is the module to toggle.
    # Patching `routability` raises AttributeError -- which is how this line
    # was found, and see `main` for why that mattered more than the typo.
    before848 = _toggle(L, 'rect_on_sides', _whole_part_rect,
                        lambda: _sweep(pcb, path, ctx, refs))

    dem_b, dfc_b = _totals(before850)
    dem_a, dfc_a = _totals(now)
    _d848b, dfc848b = _totals(before848)
    risen = sum(1 for ref in refs
                for i, r in enumerate(before850.get(ref, []))
                if now[ref][i]['demand_nets'] > r['demand_nets'])
    int_r = sum(rows[0]['interior_pads'] for rows in now.values() if rows)

    # The escape ledger's own interior count for the same refs, at the same
    # clearance and with the same geometry -- the negative control.
    sides = E.board_side_map(pcb)
    cont = E.board_container_refs(pcb, path)
    int_e = 0
    for ref in refs:
        if not now.get(ref):
            continue
        int_e += E.part_escape(pcb, ref, ignore_net_ids=(),
                               obstruction_rects=ctx.geom, sides=sides,
                               containers=cont,
                               clearance=CLEARANCE).interior_pads
    esc = E.escape_ledger(pcb, pcb_file=path, track_width=TRACK,
                          clearance=CLEARANCE)
    esc_deficit = sum(f.deficit for p in esc for f in p.faces)

    # #848's geometry census, independent of any ledger.
    geom = L.part_copper_geometry(fps, CLEARANCE)
    boxes = sorted('%s/%s' % (ref, side)
                   for ref, g in geom.items()
                   for side, box in g.rect_sides.items() if box != g.rect)
    charged = sorted({ref for ref, g in geom.items()
                      for rows in now.values() for r in rows
                      if any(n == ref for n, _v in r['eaten_by'])
                      and any(b != g.rect for b in g.rect_sides.values())})

    # ...and the assignment box set by an UNNETTED pad (the third finding).
    unnetted = []
    for ref in refs:
        g = geom.get(ref)
        if g is None:
            continue
        pairs = [(p, L.pad_box(g, p)) for p in fps[ref].pads]
        if E._assignment_rect(pairs, g, None) != g.copper:
            n_old = sum(1 for p, b in pairs if p.net_id and E.face_of(
                p, g.copper, E.pad_pitch(fps[ref]), pad_box=b) is None)
            n_new = sum(1 for p, f in E.assign_faces(
                fps[ref], g, lane_mm=TRACK + CLEARANCE).faces
                if p.net_id and f is None)
            unnetted.append('%s %d->%d' % (ref, n_old, n_new))

    return {
        'refs': len(refs),
        'demand_before': dem_b, 'demand_after': dem_a,
        'deficit_before': dfc_b, 'deficit_after': dfc_a,
        'interior_routability': int_r, 'interior_escape': int_e,
        'faces_risen': risen,
        'starved_before': _starved(before850), 'starved_after': _starved(now),
        'side_boxes_differ': len(boxes), 'side_charged': len(charged),
        'deficit_848_before': dfc848b, 'deficit_848_after': dfc_a,
        'escape_deficit': esc_deficit,
        'side_box_refs': boxes, 'side_charged_refs': charged,
        'unnetted_box': unnetted,
    }


def table_demand(rows):
    print('\n#850 -- THE DEMAND MODEL (clearance %s / track %s / grid %s)'
          % (CLEARANCE, TRACK, GRID))
    print('%-30s %4s %13s %13s %6s %6s %5s %11s'
          % ('board', 'refs', 'demand b->a', 'deficit b->a', 'int_R', 'int_E',
             'rise', 'starved b->a'))
    warn = []
    for name, d in rows:
        if not (d['demand_before'] or d['interior_routability']):
            continue
        flag = ''
        if d['interior_routability'] != d['interior_escape']:
            flag = '  <-- WARN'
            warn.append(name)
        print('%-30s %4d %6d->%-6d %6d->%-6d %6d %6d %5d %5d->%-5d%s'
              % (name[:30], d['refs'], d['demand_before'], d['demand_after'],
                 d['deficit_before'], d['deficit_after'],
                 d['interior_routability'], d['interior_escape'],
                 d['faces_risen'], d['starved_before'], d['starved_after'],
                 flag))
    tot = lambda k: sum(d[k] for _n, d in rows)                # noqa: E731
    print('%-30s %4d %6d->%-6d %6d->%-6d %6d %6d %5d %5d->%-5d'
          % ('TOTAL', tot('refs'), tot('demand_before'), tot('demand_after'),
             tot('deficit_before'), tot('deficit_after'),
             tot('interior_routability'), tot('interior_escape'),
             tot('faces_risen'), tot('starved_before'), tot('starved_after')))
    print('\n  int_R / int_E is THE NEGATIVE CONTROL: the interior-pad count')
    print('  each ledger reports for the same refs. They must be equal --')
    print('  that is what says the two instruments were reconciled rather')
    print('  than that one stopped counting.')
    if warn:
        print('  WARN: they DISAGREE on %s. Do not read any other number on'
              % ', '.join(warn))
        print('  this page until that is explained.')
    else:
        print('  They agree on every board above.')
    print('\n  `rise` counts faces whose demand went UP. The tolerance and the')
    print('  `escape.FACES` tie order MOVE a net between faces; only the')
    print('  interior bucket takes one off. A report of the falls alone has')
    print('  measured half the change.')
    print('\n  ASSIGNMENT BOX SET BY AN UNNETTED PAD (netted pads interior,')
    print('  all-pad box -> netted-pad box):')
    any_un = False
    for name, d in rows:
        if d['unnetted_box']:
            any_un = True
            print('    %-28s %s' % (name[:28], ', '.join(d['unnetted_box'])))
    if not any_un:
        print('    none')


def table_side(rows):
    print('\n#848 -- THE PER-SIDE NEIGHBOUR RECT')
    print('%-30s %6s %8s %15s %8s'
          % ('board', 'boxes', 'charged', 'deficit b->a', 'escape'))
    for name, d in rows:
        if not d['side_boxes_differ'] and not d['deficit_848_before']:
            continue
        print('%-30s %6d %8d %6d->%-6d %8d'
              % (name[:30], d['side_boxes_differ'], d['side_charged'],
                 d['deficit_848_before'], d['deficit_848_after'],
                 d['escape_deficit']))
        if d['side_box_refs']:
            print('        %s' % ' '.join(d['side_box_refs']))
    tb = sum(d['side_boxes_differ'] for _n, d in rows)
    print('%-30s %6d' % ('TOTAL (ref, side) boxes differing', tb))
    print('\n  `deficit b->a` is whole-part-charged -> shared-side-charged.')
    print('  #848 predicted no movement, and on the reported DEFICIT it is')
    print('  right; supply and `eaten_by` do move. See --table demand for')
    print('  the deficit the OTHER change moves.')


def table_gate(_rows):
    print('\n#850 -- THE TRACKED --gate PAIR')
    fix = os.path.join(ROOT, 'tests', 'fixtures', 'run23')
    dmg = os.path.join(fix, 'tigard_damaged.kicad_pcb')
    ok = os.path.join(fix, 'tigard_placed.kicad_pcb')
    if not (os.path.isfile(dmg) and os.path.isfile(ok)):
        print('  the run23 pair is absent; nothing to measure')
        return
    tool = os.path.join(ROOT, 'py_tools', 'check_channels.py')
    for label, args in (('damaged vs placed', [dmg, '--baseline', ok]),
                        ('placed vs itself', [ok, '--baseline', ok])):
        r = subprocess.run([sys.executable, '-X', 'utf8', tool] + args
                           + ['--gate', '--track-width', '0.15',
                              '--clearance', '0.15'],
                           capture_output=True, text=True,
                           encoding='utf-8', errors='replace')
        new = [ln.strip() for ln in (r.stdout or '').splitlines()
               if ln.strip().startswith('NEW')]
        print('  %-20s exit %d, %d NEW' % (label, r.returncode, len(new)))
        for ln in new:
            print('      %s' % ln[:140])


def _diff(before, after):
    b = json.load(open(run_utils.evidence(before, 'the BEFORE json'),
                       encoding='utf-8'))
    a = json.load(open(run_utils.evidence(after, 'the AFTER json'),
                       encoding='utf-8'))
    bt, at = set(b.get('tables', ())), set(a.get('tables', ()))
    if bt != at:
        print('REFUSING to diff: the two runs did not produce the same tables')
        print('  before ran %s, after ran %s' % (sorted(bt), sorted(at)))
        print('  A table one side did not run reads as every value moving.')
        return 2
    moved = 0
    for board in sorted(set(b['boards']) | set(a['boards'])):
        if board not in b['boards']:
            print('  NEW BOARD  %s' % board)
            continue
        if board not in a['boards']:
            print('  GONE       %s' % board)
            continue
        for k in DIFF_KEYS:
            x, y = b['boards'][board].get(k), a['boards'][board].get(k)
            if x != y:
                print('  %-30s %-22s %r -> %r' % (board, k, x, y))
                moved += 1
    print('\n%d value(s) moved' % moved)
    if b.get('engine_sha') != a.get('engine_sha'):
        print('engine %s -> %s' % (b.get('engine_sha'), a.get('engine_sha')))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--table', default='demand,side,gate',
                    help='comma-separated: demand, side, gate')
    ap.add_argument('--boards', nargs='*', help='basenames to restrict to')
    ap.add_argument('--out', help='write the figures as json')
    ap.add_argument('--diff', nargs=2, metavar=('BEFORE', 'AFTER'))
    args = ap.parse_args()

    if args.diff:
        return _diff(*args.diff)

    tracked = run_utils.corpus_boards()
    if not tracked:
        print('SKIP: git could not name the tracked corpus')
        return SKIP_EXIT
    paths = {os.path.splitext(os.path.basename(p))[0]: os.path.join(ROOT, p)
             for p in tracked}
    if args.boards:
        paths = {k: v for k, v in paths.items() if k in set(args.boards)}
        if not paths:
            print('no tracked board matched %r' % (args.boards,))
            return 2

    wanted = [t.strip() for t in args.table.split(',') if t.strip()]
    rows, failed = [], []
    for name, path in sorted(paths.items()):
        try:
            rows.append((name, measure(path)))
        except Exception as exc:                             # noqa: BLE001
            failed.append(name)
            print('  %-30s FAILED: %r' % (name, exc))

    # REFUSE rather than print a clean page over nothing. The first run of
    # this file patched `rect_on_sides` on the wrong module, every board
    # raised, and it went on to print a table of zeroes under the words "they
    # agree on every board above" -- a report that said the negative control
    # was satisfied when it had measured no board at all. A measurement whose
    # input is missing measures nothing, and must say so.
    if failed or not rows:
        print('\nREFUSING to report: %d of %d board(s) did not measure (%s).'
              % (len(failed), len(failed) + len(rows),
                 ', '.join(failed[:6]) or 'none measured'))
        print('Every table below would read as "nothing moved" on no data.')
        return 2

    for t in wanted:
        {'demand': table_demand, 'side': table_side,
         'gate': table_gate}[t](rows)

    if args.out:
        sha = subprocess.run(['git', '-C', ROOT, 'rev-parse', 'HEAD'],
                             capture_output=True, text=True).stdout.strip()
        with open(args.out, 'w', encoding='utf-8') as f:
            json.dump({'engine_sha': sha, 'tables': sorted(wanted),
                       'basis': {'clearance': CLEARANCE, 'track': TRACK,
                                 'grid': GRID},
                       'boards': {n: d for n, d in rows}},
                      f, indent=1, sort_keys=True)
        print('\nwrote %s' % args.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

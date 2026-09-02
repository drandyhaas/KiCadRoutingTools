#!/usr/bin/env python3
"""#834/#835: what the two side-blind placement instruments cost, measured.

Regenerates every number the #834/#835 PR body and the updated docs state, from
the engine as it stands. Run it once BEFORE the fix and once after, into two
files, and diff them -- a run that lives only in a terminal is not evidence.

    python3 -X utf8 tests/measure_834_835_side_awareness.py
    python3 -X utf8 tests/measure_834_835_side_awareness.py --out before.json
    python3 -X utf8 tests/measure_834_835_side_awareness.py --diff before.json after.json
    python3 -X utf8 tests/measure_834_835_side_awareness.py --table A
    python3 -X utf8 tests/measure_834_835_side_awareness.py --boards ulx3s tigard

Three tables:

  A  the PAIR_TEST_CAP census (#834): which pairs take the over-cap branch, how
     many of them share no copper face, and how many involve a part with pads on
     more than one face -- the residual the per-side extent addresses.
  B  the escape ledger (#835): deficit lanes and faces per board, plus the two
     diagnostics that say WHY -- blocker charges whose payer is on the other
     face, and charges from a neighbour that CONTAINS the escaping part.
  C  the per-board deficit-parts/worst pair that `docs/placement-predictors.md`
     tabulates, so that table can be re-derived rather than hand-edited.
  D  the three-rectangle census (#841): both lane ledgers under each of the
     three boxes the tree contains for a neighbour -- the bbox of pad CENTRES
     (`escape._part_rect`, what escape charged), the pad COPPER box
     (`legality.part_copper_geometry`, what both charge now) and the COURTYARD
     (`GradedPart.rect`, what `face_lane_ledger` charged). This is what makes
     the #841 decision RE-DERIVABLE: before it, the argument for one rect over
     another lived in two code comments that disagreed with each other.
     Not in the default `--table ABC`: it runs each ledger three times.

NOT named `test_*.py`: `tests/run_all.py` does not collect it. It asserts
nothing, and table B runs the full escape ledger on every corpus board (minutes).

Boards come from `run_utils.corpus_boards()` -- the git-TRACKED set. A plain
glob of `kicad_files/` also returns generated boards, and the two sets give
different answers: 22 tracked boards carry 38 over-cap pairs, a 33-board glob
carries 66. Every number here is the tracked set.

Sides are keyed by REFERENCE, never by `id(footprint)`. A cache keyed on object
identity and shared across boards in one process reports the PREVIOUS board's
answer once its footprints have been freed and their ids reused -- measured,
that turned tigard's 41 (a negative control that must not move) into a 32 that
looked like a finding.
"""
import argparse
import json
import os
import subprocess
import sys
from itertools import combinations

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (ROOT, os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_placer'),
           os.path.join(ROOT, 'py_tools'), os.path.join(ROOT, 'tests')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import run_utils                                             # noqa: E402
from kicad_parser import parse_kicad_pcb                     # noqa: E402
from placement import escape as E                            # noqa: E402
from placement import legality as L                          # noqa: E402
from placement.options import deficit_totals                 # noqa: E402

#: The clearance table A prices pads at. Stated rather than defaulted: most
#: tracked boards carry no sibling `.kicad_pro` (#441), so an unstated basis
#: would silently be `routing_defaults.CLEARANCE`.
CLEARANCE = 0.2


def engine_sha():
    try:
        return subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT,
                              capture_output=True, text=True).stdout.strip()
    except Exception:                                        # noqa: BLE001
        return 'unknown'


def sides_by_ref(pcb):
    """{ref: frozenset} of the BOARD SIDES each footprint obstructs.

    The canonical readers, not a hand-rolled `fp.layer` comparison:
    `placement_state.py` records the tigard J2/JP1 bug that came from exactly
    that. Keyed by ref -- see the module docstring.
    """
    return {r: L.sides_occupied(L.footprint_side(f),
                                L.footprint_has_through_pads(f))
            for r, f in pcb.footprints.items()}


def pad_sides_by_ref(parts):
    """{ref: frozenset} of the sides a part's COPPER PADS occupy.

    Deliberately not `sides_occupied`, which answers a BODY question and is
    always a superset: glasgow's J5 has pads on F only but two NPTH holes, so
    its body occupies both faces. Pricing pad clearance at the body's answer
    would make the #834 fix inert on the one over-cap part that carries holes.
    """
    out = {}
    for ref, pp in parts.items():
        s = set()
        for t in pp.pads_local:
            if t[5] is None:
                s.update(('F', 'B'))
            else:
                s.add(t[5])
        out[ref] = frozenset(s)
    return out


def build_parts(pcb, path):
    model = L.PadClearanceModel.for_board(pcb, CLEARANCE, path)
    notes = list(model.notes)
    if not model.active:
        model = None
    return L.build_part_pads(
        pcb.footprints, CLEARANCE, model,
        npth_floor=L.resolve_npth_floor(pcb, path, notes))


def table_a(path):
    """The PAIR_TEST_CAP census for one board (#834)."""
    pcb = parse_kicad_pcb(path)
    parts = build_parts(pcb, path)
    ps = pad_sides_by_ref(parts)
    pairs, cross, mixed = [], [], []
    for a, b in combinations(sorted(parts), 2):
        if parts[a].n_pads * parts[b].n_pads <= L.PAIR_TEST_CAP:
            continue
        row = [a, b, parts[a].n_pads, parts[b].n_pads,
               ''.join(sorted(ps[a])), ''.join(sorted(ps[b])),
               len(parts[a].holes_extent) + len(parts[b].holes_extent)]
        pairs.append(row)
        if not (ps[a] & ps[b]):
            cross.append(row)
        if len(ps[a]) > 1 or len(ps[b]) > 1:
            mixed.append(row)
    return {'over_cap_pairs': len(pairs), 'cross_side': len(cross),
            'mixed_side_member': len(mixed), 'pairs': pairs,
            'cross_side_pairs': cross}


def contains(outer, inner):
    """Does `outer` enclose `inner`? Both are (x0, y0, x1, y1)."""
    return (outer[0] <= inner[0] and outer[1] <= inner[1]
            and outer[2] >= inner[2] and outer[3] >= inner[3])


def table_b(path):
    """The escape ledger and the two diagnostics that explain it (#835)."""
    pcb = parse_kicad_pcb(path)
    sides = sides_by_ref(pcb)
    led = E.escape_ledger(pcb, pcb_file=path)
    tot = deficit_totals(led)
    rects = {r: E._part_rect(f) for r, f in pcb.footprints.items() if f.pads}
    charges = cross = contained = 0
    witnesses = []
    for p in led:
        own = sides.get(p.ref, frozenset())
        own_rect = rects.get(p.ref)
        for f in p.faces:
            for b in f.blockers:
                charges += 1
                if not (sides.get(b, frozenset()) & own):
                    cross += 1
                    if len(witnesses) < 8:
                        witnesses.append(
                            [p.ref, ''.join(sorted(own)), f.face, f.deficit,
                             b, ''.join(sorted(sides.get(b, ())))])
                br = rects.get(b)
                if own_rect and br and contains(br, own_rect):
                    contained += 1
    faces = sum(1 for p in led for f in p.faces if f.deficit > 0)
    return {'deficit_lanes': tot['lanes'], 'deficit_parts': tot['parts'],
            'examined': tot['examined'], 'faces_in_deficit': faces,
            'blocker_charges': charges, 'cross_side_charges': cross,
            'container_charges': contained,
            'cross_side_witnesses': witnesses}


#: The three boxes the tree contains for "what does a neighbour contribute".
#: Named here rather than inline so the census and the prose cannot drift.
RECTS = ('centres', 'copper', 'courtyard')


def _rect_maps(pcb, path):
    """{kind: {ref: rect}} for the three candidate obstruction rectangles."""
    from placement.legality import (graded_parts_from_file,
                                    part_copper_geometry)
    geom = part_copper_geometry(pcb.footprints or {}, CLEARANCE)
    return {
        'centres': {r: E._part_rect(f)
                    for r, f in (pcb.footprints or {}).items() if f.pads},
        'copper': {r: g.rect for r, g in geom.items()},
        'courtyard': {g.ref: g.rect for g in graded_parts_from_file(pcb, path)},
    }


def table_d(path):
    """Both lane ledgers under each of the three neighbour rectangles (#841).

    The rect is swapped by substitution INSIDE the shared kernel, so the only
    thing that varies is which box a neighbour contributes -- the escaping
    part's own geometry is held. #841's verification section is explicit that
    swapping both at once reports the change in the opposite direction, and
    that is how it was first mis-measured.

    So the `centres` column is NOT "the ledger before #841": the subject rect
    stays at the pad-copper box in every arm, which is why tigard reads 37 here
    and 41 in a pre-#841 run. Read the columns against EACH OTHER, which is the
    comparison the decision rests on, not against a historical number.

    The DEMAND pair is printed for the same reason it is asserted in
    `tests/test_835_escape_side_aware.py`: demand and interior pads are
    properties of the escaping part's own pad lattice, so they must be
    IDENTICAL down each row. A column where they move is a run in which the
    subject geometry leaked into the neighbour arm, and every deficit above it
    is then measuring two things at once.

    A ref with no rect of a given kind is DROPPED for that arm and counted, not
    silently skipped: the courtyard arm carries every footprint, the copper and
    centre arms carry only those with pads, and the difference is exactly each
    board's `synthetic` set.
    """
    from placement import routability as R
    pcb = parse_kicad_pcb(path)
    maps = _rect_maps(pcb, path)
    refs = E.fine_pitch_parts(pcb)
    real = E.span_eaten
    out = {}
    for kind in RECTS:
        rects = maps[kind]
        def swap(lo, hi, band, horizontal, obstacles, _r=rects):
            return real(lo, hi, band, horizontal,
                        [(ref, _r[ref]) for ref, _x in obstacles if ref in _r])
        E.span_eaten = swap
        try:
            led = E.escape_ledger(pcb, pcb_file=path)
            tot = deficit_totals(led)
            fine = 0
            for ref in refs:
                for row in R.face_lane_ledger(pcb, ref, clearance=CLEARANCE,
                                              track_width=CLEARANCE,
                                              grid_step=0.05, pcb_file=path):
                    fine += row['deficit_finest_grid']
        finally:
            E.span_eaten = real
        out[kind] = {
            'escape_lanes': tot['lanes'],
            'escape_parts': tot['parts'],
            'lane_ledger_deficit_finest': fine,
            'rects': len(rects),
            'demand': sum(f.demand for p in led for f in p.faces),
            'interior_pads': sum(p.interior_pads for p in led),
        }
    return out


def table_c(path):
    """`docs/placement-predictors.md`'s deficit-parts / worst pair."""
    pcb = parse_kicad_pcb(path)
    led = E.escape_ledger(pcb, pcb_file=path)
    parts = sum(1 for p in led if p.worst)
    worst = max([p.worst.deficit for p in led if p.worst] or [0])
    return {'deficit_parts': parts, 'worst_deficit': worst}


def measure(paths, tables):
    out = {'engine_sha': engine_sha(), 'clearance': CLEARANCE,
           'pair_test_cap': L.PAIR_TEST_CAP, 'boards': {}}
    for path in paths:
        name = os.path.splitext(os.path.basename(path))[0]
        row = {}
        try:
            if 'A' in tables:
                row['A'] = table_a(path)
            if 'B' in tables:
                row['B'] = table_b(path)
            if 'C' in tables:
                row['C'] = table_c(path)
            if 'D' in tables:
                row['D'] = table_d(path)
        except Exception as exc:                             # noqa: BLE001
            row['error'] = '{}: {}'.format(type(exc).__name__, exc)
        out['boards'][name] = row
    return out


def report(doc, tables):
    print('engine {}  clearance {}  PAIR_TEST_CAP {}'.format(
        doc['engine_sha'][:12], doc['clearance'], doc['pair_test_cap']))
    if 'A' in tables:
        print()
        print('TABLE A -- the PAIR_TEST_CAP census (#834)')
        print('{:<34} {:>8} {:>11} {:>12}'.format(
            'board', 'over-cap', 'cross-side', 'mixed-side'))
        ta = tb = tc = 0
        for name, row in sorted(doc['boards'].items()):
            a = row.get('A')
            if not a or not a['over_cap_pairs']:
                continue
            print('{:<34} {:>8} {:>11} {:>12}'.format(
                name, a['over_cap_pairs'], a['cross_side'],
                a['mixed_side_member']))
            ta += a['over_cap_pairs']
            tb += a['cross_side']
            tc += a['mixed_side_member']
        print('{:<34} {:>8} {:>11} {:>12}'.format('TOTAL', ta, tb, tc))
        for name, row in sorted(doc['boards'].items()):
            for p in (row.get('A') or {}).get('cross_side_pairs', ()):
                print('   cross-side: {:<28} {}({}) x {}({})  {} x {} pads'
                      .format(name, p[0], p[4], p[1], p[5], p[2], p[3]))
    if 'B' in tables:
        print()
        print('TABLE B -- the escape ledger (#835)')
        print('{:<34} {:>6} {:>6} {:>6} {:>8} {:>7} {:>6}'.format(
            'board', 'lanes', 'parts', 'faces', 'charges', 'cross', 'contd'))
        for name, row in sorted(doc['boards'].items()):
            b = row.get('B')
            if not b:
                continue
            print('{:<34} {:>6} {:>6} {:>6} {:>8} {:>7} {:>6}'.format(
                name, b['deficit_lanes'], b['deficit_parts'],
                b['faces_in_deficit'], b['blocker_charges'],
                b['cross_side_charges'], b['container_charges']))
    if 'C' in tables:
        print()
        print('TABLE C -- docs/placement-predictors.md deficit parts / worst')
        for name, row in sorted(doc['boards'].items()):
            c = row.get('C')
            if c:
                print('   {:<34} {} / {}'.format(
                    name, c['deficit_parts'], c['worst_deficit']))
    if 'D' in tables:
        print()
        print('TABLE D -- the three neighbour rectangles (#841)')
        print('   escape deficit lanes | face_lane_ledger deficit at the '
              'finest grid')
        print('{:<34} {:>21} {:>21} {:>21}'.format(
            'board', 'pad CENTRES', 'pad COPPER (now)', 'COURTYARD'))
        for name, row in sorted(doc['boards'].items()):
            d = row.get('D')
            if not d:
                continue
            cells = []
            for kind in RECTS:
                k = d[kind]
                cells.append('{:>9} | {:<9}'.format(
                    k['escape_lanes'], k['lane_ledger_deficit_finest']))
            print('{:<34} {:>21} {:>21} {:>21}'.format(name, *cells))
        print()
        print('   DEMAND must not move with the rect -- it is a netlist fact.')
        print('{:<34} {:>21} {:>21} {:>21}'.format(
            'board', 'demand/interior', 'demand/interior', 'demand/interior'))
        for name, row in sorted(doc['boards'].items()):
            d = row.get('D')
            if not d:
                continue
            cells = ['{:>9}/{:<9}'.format(d[k]['demand'],
                                          d[k]['interior_pads'])
                     for k in RECTS]
            print('{:<34} {:>21} {:>21} {:>21}'.format(name, *cells))
    for name, row in sorted(doc['boards'].items()):
        if 'error' in row:
            print('   ERROR {:<30} {}'.format(name, row['error']))


DIFF_KEYS = (('A', 'over_cap_pairs'), ('A', 'cross_side'),
             ('A', 'mixed_side_member'),
             ('B', 'deficit_lanes'), ('B', 'deficit_parts'),
             ('B', 'faces_in_deficit'), ('B', 'blocker_charges'),
             ('B', 'cross_side_charges'), ('B', 'container_charges'),
             ('C', 'deficit_parts'), ('C', 'worst_deficit'))


def diff(before, after):
    with open(before, encoding='utf-8') as fh:
        a = json.load(fh)
    with open(after, encoding='utf-8') as fh:
        b = json.load(fh)
    print('before {}  ->  after {}'.format(
        a['engine_sha'][:12], b['engine_sha'][:12]))
    # A table one side did not RUN is not a change. Saying so is the whole
    # point: a `--table BC` run diffed against a full one otherwise reports
    # every board as moved, which is a instrument that cannot tell a real
    # transition from its own missing input.
    common = sorted({t for t, _ in DIFF_KEYS
                     if any(t in r for r in a['boards'].values())
                     and any(t in r for r in b['boards'].values())})
    skipped = sorted({t for t, _ in DIFF_KEYS} - set(common))
    if skipped:
        print('table(s) {} not measured on both sides -- not compared'
              .format(', '.join(skipped)))
    moved = 0
    for name in sorted(set(a['boards']) | set(b['boards'])):
        ra, rb = a['boards'].get(name, {}), b['boards'].get(name, {})
        lines = []
        for t, k in DIFF_KEYS:
            if t not in common:
                continue
            va = (ra.get(t) or {}).get(k)
            vb = (rb.get(t) or {}).get(k)
            if va is None and vb is None:
                continue
            if va != vb:
                lines.append('{}.{} {} -> {}'.format(t, k, va, vb))
        if lines:
            moved += 1
            print('  {:<34} {}'.format(name, ';  '.join(lines)))
        else:
            print('  {:<34} unchanged'.format(name))
    print('{} of {} board(s) moved, comparing table(s) {}'.format(
        moved, len(set(a['boards']) | set(b['boards'])), ''.join(common)))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--table', default='ABC',
                    help='which tables to run (subset of ABC)')
    ap.add_argument('--boards', nargs='*',
                    help='board basenames to restrict to')
    ap.add_argument('--out', help='write the measurement to this JSON file')
    ap.add_argument('--diff', nargs=2, metavar=('BEFORE', 'AFTER'),
                    help='print the transition between two --out files')
    args = ap.parse_args()

    if args.diff:
        for p in args.diff:
            run_utils.evidence(p, 'measurement file')
        diff(*args.diff)
        return 0

    paths = run_utils.corpus_boards()
    if not paths:
        print('SKIP: git could not name the tracked corpus, and a plain glob '
              'of kicad_files/ is a different set (generated boards). '
              'Nothing measured.')
        return 77
    if args.boards:
        want = set(args.boards)
        paths = [p for p in paths
                 if os.path.splitext(os.path.basename(p))[0] in want]
        if not paths:
            print('no tracked board matched {}'.format(sorted(want)))
            return 2
    tables = ''.join(t for t in 'ABCD' if t in args.table.upper())
    doc = measure(paths, tables)
    report(doc, tables)
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh, indent=1, sort_keys=True)
        print()
        print('wrote {}'.format(args.out))
    return 0


if __name__ == '__main__':
    sys.exit(main())

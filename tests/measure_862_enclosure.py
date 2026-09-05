#!/usr/bin/env python3
"""Regenerate every number the #862 PR states, from the engine as it is.

Asserts nothing, and is deliberately not named `test_*`: `tests/run_all.py`
globs `test_*.py` and this is a report. The gate is
`tests/test_862_enclosure.py`.

BOTH ARMS ARE MEASURED IN ONE PROCESS. The "before" column is the BOX RULE
ALONE, reached by calling `escape.assign_faces` without a corridor basis --
which is the documented pre-#862 answer, not a re-implementation of it. So no
table here depends on checking out a parent commit, and the before column
re-measures on every run instead of going stale.

    python3 -X utf8 tests/measure_862_enclosure.py                # all tables
    python3 -X utf8 tests/measure_862_enclosure.py --table interior
    python3 -X utf8 tests/measure_862_enclosure.py --out after.json
    python3 -X utf8 tests/measure_862_enclosure.py --diff a.json b.json

THE BASIS IS NOT ONE NUMBER, and that is the point of the `interior` table.
Before #862 `interior_pads` was a function of the part's geometry alone; since
#862 it is a function of the clearance and the track width too, because the
enclosure test asks whether a TRACK can leave. Three bases are in use --
`check_channels` runs the corpus at 0.09/0.10, `test_850`'s census is taken at
0.20/0.20, and the CLI's own defaults are 0.25/0.30 -- and two more are swept
that nothing routes at, because they are where two of the rule's parameters
are visible at all.

COUNTS ARE NOT SETS. The `interior` table prints the set differences in both
directions, not just the totals. `qfn_interior_pads` U1 reads 5 before and 5
after, and that is only a negative control if they are the SAME five pads: the
reverted `_assignment_rect` experiment printed "5 -> 1" in an "interior pads
recovered" column and nobody asked which four had moved.
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
import check_channels as CC                                  # noqa: E402

SKIP_EXIT = 77

#: The three bases anything actually resolves, then two that nothing does.
#: 0.05/0.10 and 0.40/0.40 are swept because they are the ONLY places on this
#: corpus where the threshold epsilon and (at 0.05) the band-overlap
#: strictness change an answer at all -- so a table without them cannot show
#: that those two decisions do anything.
BASES = ((0.09, 0.10), (0.20, 0.20), (0.25, 0.30), (0.05, 0.10), (0.40, 0.40))
IN_USE = BASES[:3]

DIFF_KEYS = ('refs', 'interior_box', 'interior_union', 'refs_moved',
             'ulx3s_U1', 'qfn_interior_pads_U1', 'starved_box',
             'starved_union', 'deficit_box', 'deficit_union')


def _interior(fp, geom, clr, trk):
    """(box set, union set) of the netted pads this part calls interior.

    Keyed on the pad's INDEX in `fp.pads`, paired with its number for
    display. Not on the number alone: a pad number is not unique within a
    footprint -- a thermal pad drawn as sub-rects and a row of NC pads both
    repeat one -- and keying a set on it silently merges them. Measured, that
    mistake undercounts this corpus by 60 interior pads (1994 against 2054)
    and drops two of the moved refs off the table entirely, which is the same
    "a count is not a set" error this script exists to catch in the rule.
    """
    asg = E.assign_faces(fp, geom, lane_mm=trk + clr,
                         clearance=clr, track_width=trk)
    box, union = set(), set()
    for i, (pad, face) in enumerate(asg.faces):
        if not getattr(pad, 'net_id', 0):
            continue
        key = (i, pad.pad_number)
        if asg.box_faces[i] is None:
            box.add(key)
        if face is None:
            union.add(key)
    return box, union


def _sweep(clr, trk):
    """{ref_key: (box set, union set)} over every tracked board."""
    out = {}
    for path in run_utils.corpus_boards():
        name = os.path.basename(path)[:-len('.kicad_pcb')]
        pcb = parse_kicad_pcb(path)
        refs = E.fine_pitch_parts(pcb)
        if not refs:
            continue
        geoms = L.part_copper_geometry(pcb.footprints, clr)
        for ref in refs:
            g = geoms.get(ref)
            if g is None:
                continue
            out['%s:%s' % (name, ref)] = _interior(
                pcb.footprints[ref], g, clr, trk)
    return out


def _gate_census(clr, trk):
    """(starved, deficit faces) under the box rule and under the union.

    The box arm is reached by stripping the basis at `assign_faces`, which is
    its own documented "the caller has no opinion" path -- so this is the
    shipped pre-#862 behaviour rather than a copy of it.
    """
    real = E.assign_faces

    def box_only(fp, geom, *, lane_mm, fallback_rect=None,
                 clearance=None, track_width=None):
        return real(fp, geom, lane_mm=lane_mm, fallback_rect=fallback_rect)

    res = {}
    for label, fn in (('box', box_only), ('union', real)):
        E.assign_faces = fn
        R.assign_faces = fn
        starved = deficit = 0
        for path in run_utils.corpus_boards():
            pcb = parse_kicad_pcb(path)
            refs = E.fine_pitch_parts(pcb)
            if not refs:
                continue
            ctx = R.board_lane_context(pcb, clr, pcb_file=path)
            led = {}
            for ref in refs:
                rows = R.face_lane_ledger(pcb, ref, clearance=clr,
                                          track_width=trk, grid_step=0.05,
                                          pcb_file=path, context=ctx)
                if rows:
                    led[ref] = rows
            starved += len(CC._starved_faces(led, CC.GATE_MIN_DEMAND))
            deficit += len(CC._deficit_faces(led))
        res[label] = (starved, deficit)
    E.assign_faces = real
    R.assign_faces = real
    return res


def measure(bases=BASES, with_gate=True):
    data = {}
    for clr, trk in bases:
        sw = _sweep(clr, trk)
        moved = {k: (len(b), len(u)) for k, (b, u) in sw.items()
                 if len(b) != len(u)}
        grew = {k: (len(b), len(u)) for k, (b, u) in sw.items()
                if len(u) > len(b)}
        not_subset = {k for k, (b, u) in sw.items() if not (u <= b)}
        row = {
            'refs': len(sw),
            'interior_box': sum(len(b) for b, _u in sw.values()),
            'interior_union': sum(len(u) for _b, u in sw.values()),
            'refs_moved': len(moved),
            'refs_grew': len(grew),
            'not_subset': sorted(not_subset),
            'moved': {k: v for k, v in sorted(moved.items())},
            'ulx3s_U1': list(map(len, sw.get('ulx3s:U1', (set(), set())))),
            'qfn_interior_pads_U1': list(
                map(len, sw.get('qfn_interior_pads:U1', (set(), set())))),
            'qfn_same_pads': (sw['qfn_interior_pads:U1'][0]
                              == sw['qfn_interior_pads:U1'][1]
                              if 'qfn_interior_pads:U1' in sw else None),
        }
        if with_gate and (clr, trk) in IN_USE:
            g = _gate_census(clr, trk)
            row['starved_box'], row['deficit_box'] = g['box']
            row['starved_union'], row['deficit_union'] = g['union']
        data['%.2f/%.2f' % (clr, trk)] = row
    return data


def table_interior(data):
    print('\n#862 -- THE ENCLOSURE RULE, box alone vs box OR corridor')
    print('%-14s %5s %8s %8s %6s %6s  %-11s %-11s'
          % ('clr/track', 'refs', 'box', 'union', 'moved', 'grew',
             'ulx3s:U1', 'qfn:U1'))
    for basis, d in data.items():
        used = '' if basis in ['%.2f/%.2f' % b for b in IN_USE] else '  (swept)'
        print('%-14s %5d %8d %8d %6d %6d  %-11s %-11s%s'
              % (basis, d['refs'], d['interior_box'], d['interior_union'],
                 d['refs_moved'], d['refs_grew'],
                 '%d -> %d' % tuple(d['ulx3s_U1']),
                 '%d -> %d' % tuple(d['qfn_interior_pads_U1']), used))
    print('\n  `grew` is the DIRECTION CONTROL and must be 0 everywhere: the')
    print('  rule is a UNION, so the escape set can only grow and the')
    print('  interior set can only shrink. A non-zero column means someone')
    print('  turned it into a replacement -- which is a real mutation, and')
    print('  measured, it gains interior pads on orangecrab U10 and rp2350 U4.')
    print('\n  `not_subset` is the same claim as a SET rather than a count:')
    for basis, d in data.items():
        if d['not_subset']:
            print('    %s  NOT A SUBSET: %s' % (basis, d['not_subset']))
    print('    (empty on every basis above)')
    print('\n  qfn_interior_pads U1 keeps the SAME pads, not merely the same')
    print('  count -- the reverted netted-box experiment printed 5 -> 1 as a')
    print('  win without asking which four pads had moved:')
    for basis, d in data.items():
        print('    %-14s same pads: %s' % (basis, d['qfn_same_pads']))


def table_moved(data):
    print('\n#862 -- WHICH REFS MOVE, at the bases in use')
    for basis, d in data.items():
        if basis not in ['%.2f/%.2f' % b for b in IN_USE]:
            continue
        print('  %s  (%d refs)' % (basis, d['refs_moved']))
        for k, (b, u) in d['moved'].items():
            print('      %-38s %4d -> %4d' % (k, b, u))


def table_gate(data):
    print('\n#862 -- THE RANKED RISK: does --gate newly fire?')
    print('  All three gate predicates are demand-gated (GATE_MIN_DEMAND = '
          '%d), so demand' % CC.GATE_MIN_DEMAND)
    print('  RISING is the direction that could make --gate exit 4 where it '
          'exits 0.')
    print('  %-14s %-22s %s' % ('clr/track', 'starved box -> union',
                                'deficit faces box -> union'))
    for basis, d in data.items():
        if 'starved_box' not in d:
            continue
        print('  %-14s %-22s %s'
              % (basis, '%d -> %d' % (d['starved_box'], d['starved_union']),
                 '%d -> %d' % (d['deficit_box'], d['deficit_union'])))


def _diff(before, after):
    b = json.load(open(run_utils.evidence(before, 'the BEFORE json'),
                       encoding='utf-8'))
    a = json.load(open(run_utils.evidence(after, 'the AFTER json'),
                       encoding='utf-8'))
    moved = 0
    for basis in sorted(set(b) | set(a)):
        if basis not in b:
            print('  %-14s ONLY IN AFTER -- not counted as movement' % basis)
            continue
        if basis not in a:
            print('  %-14s ONLY IN BEFORE -- not counted as movement' % basis)
            continue
        for k in DIFF_KEYS:
            if k in b[basis] and k in a[basis] and b[basis][k] != a[basis][k]:
                print('  %-14s %-24s %r -> %r'
                      % (basis, k, b[basis][k], a[basis][k]))
                moved += 1
    print('\n%d value(s) moved' % moved)
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--table', choices=('interior', 'moved', 'gate'))
    ap.add_argument('--out')
    ap.add_argument('--diff', nargs=2, metavar=('BEFORE', 'AFTER'))
    args = ap.parse_args()
    if args.diff:
        return _diff(*args.diff)
    if not run_utils.corpus_boards():
        print('SKIP: git could not name the tracked corpus')
        return SKIP_EXIT

    data = measure(with_gate=args.table in (None, 'gate'))
    if args.table in (None, 'interior'):
        table_interior(data)
    if args.table in (None, 'moved'):
        table_moved(data)
    if args.table in (None, 'gate'):
        table_gate(data)
    if args.out:
        try:
            sha = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT,
                                 capture_output=True, text=True,
                                 timeout=30).stdout.strip()
        except Exception:                                    # noqa: BLE001
            sha = 'unknown'
        payload = dict(data)
        payload['engine_sha'] = sha
        with open(args.out, 'w', encoding='utf-8') as fh:
            json.dump(payload, fh, indent=1, sort_keys=True)
        print('\nwrote %s' % args.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""Did this placement change move parts as a BLOCK, or just find something legal?

The wrong-basin problem, stated precisely. A reconstruction is asked to undo
damage that moved a group of parts by one vector. The honest answer restores
them: every member moves by the same vector (or by one of a small set -- an
exchange displaces its two groups by +v and -v). A search, given the same
board, can instead find a different arrangement that is perfectly legal, grades
DRC-0, satisfies the oracle, and sits millimetres from the truth. No legality
instrument can tell the two apart, because the wrong one is legal.

This one can, and it needs no ground truth -- only the two boards:

    a NEW contact pair between two parts that BOTH moved, by vectors differing
    by more than the tolerance, cannot occur in a rigid restore.

Why it is sound: parts displaced by the SAME vector keep their relative
geometry exactly, so they cannot newly touch each other. A restored part CAN
come to rest against a part that never moved -- that is the damage being
undone, and it is not flagged. So on a true rigid restore the count is 0 by
construction, while a search that pushed neighbours around to make room leaves
its signature in every pair it created.

Measured on a recorded wrong-basin run:

    damaged input -> the accepted placement   12 inconsistent pairs
    damaged input -> the TRUTH control         0

Contact means any body channel -- pad intersections, courtyard overlaps,
waived pairs included. A waiver says "this overlap is acceptable on this
board", not "this overlap is evidence of nothing".

Usage:
    python3 -X utf8 check_rigid_consistency.py BEFORE.kicad_pcb AFTER.kicad_pcb
        [--tol 1.0] [--clearance 0.1] [--json out.json]

Exit: 0 consistent, 2 usage/load error, 4 inconsistent pairs found.
"""

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['placement'], 'kind': 'instrument'}

import argparse
import json
import math
import os
import sys

_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _root)
# #522/py_placer layout: the engine modules (kicad_parser, cli_banner, ...)
# live in py_router/py_tools/py_placer, not at the repo root, so inserting
# only _root left `import cli_banner` failing when run as a script.
for _pkg in ('py_router', 'py_tools', 'py_placer'):
    _d = os.path.join(_root, _pkg)
    if os.path.isdir(_d):
        sys.path.insert(0, _d)

# A pair whose two members moved by vectors differing by less than this is
# treated as one block. It is a MILLIMETRE tolerance, not a grid step: real
# reconstructions land members on a grid and a legalize pass nudges them, so a
# sub-millimetre spread is still one block moving together.
DEFAULT_TOL_MM = 1.0
# Below this a part counts as not having moved at all.
STATIC_EPS_MM = 0.05


def _poses(pcb):
    return {ref: (fp.x, fp.y) for ref, fp in pcb.footprints.items()}


def _contact_keys(pcb, pcb_file, clearance):
    """Every body-channel pair on the board, keyed (a, b, kind).

    Waived pairs are INCLUDED: a waiver states an overlap is acceptable, which
    is a different claim from the overlap carrying no information.
    """
    from placement.legality import grade_body_overlap
    g = grade_body_overlap(pcb, clearance, pcb_file=pcb_file)
    return {(p.a, p.b, p.kind): p for p in g['pairs']}


def analyse(before_file, after_file, clearance=0.1, tol=DEFAULT_TOL_MM):
    from kicad_parser import parse_kicad_pcb

    b_pcb, a_pcb = parse_kicad_pcb(before_file), parse_kicad_pcb(after_file)
    b_pos, a_pos = _poses(b_pcb), _poses(a_pcb)

    disp = {}
    for ref in sorted(set(b_pos) & set(a_pos)):
        dx = a_pos[ref][0] - b_pos[ref][0]
        dy = a_pos[ref][1] - b_pos[ref][1]
        disp[ref] = (dx, dy)
    movers = {r: d for r, d in disp.items()
              if math.hypot(*d) > STATIC_EPS_MM}

    before_keys = _contact_keys(b_pcb, before_file, clearance)
    after_pairs = _contact_keys(a_pcb, after_file, clearance)
    new_pairs = [p for k, p in after_pairs.items() if k not in before_keys]

    flagged, mover_static, same_block = [], 0, 0
    for p in new_pairs:
        da, db = movers.get(p.a), movers.get(p.b)
        if da is None or db is None:
            mover_static += 1            # the damage being undone: not evidence
            continue
        spread = math.hypot(da[0] - db[0], da[1] - db[1])
        if spread <= tol:
            same_block += 1              # one block moving together
            continue
        flagged.append({
            'a': p.a, 'b': p.b, 'kind': p.kind, 'area_mm2': p.area_mm2,
            'waived': bool(p.waived),
            'disp_a': [round(da[0], 4), round(da[1], 4)],
            'disp_b': [round(db[0], 4), round(db[1], 4)],
            'spread_mm': round(spread, 4),
        })
    flagged.sort(key=lambda f: -f['spread_mm'])
    return {
        'before': before_file, 'after': after_file,
        'clearance': clearance, 'tol_mm': tol,
        'parts_compared': len(disp), 'parts_moved': len(movers),
        'new_pairs': len(new_pairs),
        'new_pairs_mover_vs_static': mover_static,
        'new_pairs_same_block': same_block,
        'inconsistent': flagged,
    }


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('before')
    ap.add_argument('after')
    ap.add_argument('--tol', type=float, default=DEFAULT_TOL_MM,
                    help='mm of displacement spread still counted as one block '
                         '(default: %(default)s)')
    ap.add_argument('--clearance', type=float, default=0.1)
    ap.add_argument('--json', default=None)
    args = ap.parse_args(argv)

    for path in (args.before, args.after):
        if not os.path.isfile(path):
            print(f'check_rigid_consistency: no such board: {path}',
                  file=sys.stderr)
            return 2
    try:
        rep = analyse(args.before, args.after, args.clearance, args.tol)
    except Exception as exc:                              # noqa: BLE001
        print(f'check_rigid_consistency: {type(exc).__name__}: {exc}',
              file=sys.stderr)
        return 2

    print(f'Rigid-consistency of {args.after}')
    print(f'  vs {args.before}')
    print(f'  {rep["parts_moved"]} of {rep["parts_compared"]} part(s) moved; '
          f'{rep["new_pairs"]} new contact pair(s) '
          f'({rep["new_pairs_mover_vs_static"]} mover-vs-static, '
          f'{rep["new_pairs_same_block"]} within one block, '
          f'{len(rep["inconsistent"])} inconsistent)')
    for f in rep['inconsistent']:
        print(f'    {f["a"]} <-> {f["b"]}  {f["kind"]} {f["area_mm2"]}mm2'
              + ('  [waived]' if f['waived'] else '')
              + f'\n        moved {f["disp_a"]} vs {f["disp_b"]} '
                f'-- {f["spread_mm"]}mm apart')
    if rep['inconsistent']:
        print(f'  VERDICT: NOT A RIGID RESTORE -- {len(rep["inconsistent"])} '
              f'new contact pair(s) between parts moved by DIFFERENT vectors. '
              f'Parts displaced together keep their relative geometry, so a '
              f'true restore cannot produce these. This placement found '
              f'something legal rather than undoing the damage; check it '
              f'against the intended arrangement before routing it.')
    else:
        print('  VERDICT: consistent with a rigid restore (no new contact '
              'between differently-displaced parts)')

    if args.json:
        with open(args.json, 'w', encoding='utf-8') as fh:
            json.dump(rep, fh, indent=1, sort_keys=True)
        print(f'  JSON -> {args.json}')
    return 4 if rep['inconsistent'] else 0


if __name__ == '__main__':
    import cli_banner
    cli_banner.install()
    sys.exit(main())

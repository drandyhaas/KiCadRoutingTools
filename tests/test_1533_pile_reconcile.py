"""#1533: reconciliation reads no edge or pose off a PILE's staging poses.

On a pile (`assess_placement(...).pile`) no unlocked part's pose is a claim
(#1103, #1353, #1508). `reconcile.reconcile` still read every declared
connector's edge off its current pose, so check_floorplan printed
`DRIFT J1:edge: intent 'west' [hypothesis]; board 'south' [inferred]` for
each one -- under `--plan-only`, in the grade and in `--emit-intent`, which
also wrote the rows into `context.reconciliation` in the same run that said
"staging poses are not claims".

Pinned here, on a tracked pile and the placed board it was piled from:
  1. on the pile no reconciliation row carries a `board` value for an
     unlocked part, and ONE report row says what was withheld and why;
  2. a KiCad-locked part keeps its board value on the pile (its pose IS a
     decision);
  3. on a placed board the board value is still read (a declared edge that
     disagrees with the board is still drift);
  4. end to end: `--plan-only` prints no DRIFT row for the pile, and
     `--emit-intent --brief` writes no `inferred` value into the emitted
     `context.reconciliation`.

    python3 tests/test_1533_pile_reconcile.py
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import reconcile                               # noqa: E402
from placement.placement_state import assess_placement        # noqa: E402
from run_utils import check as run_check, evidence, tool      # noqa: E402

PILE = os.path.join(ROOT, 'tests', 'fixtures', '959', 'run29_pile.kicad_pcb')
PLACED = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
FAILS = []


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not ok else ''))
    if not ok:
        FAILS.append(name)


def _quiet(fn, *a, **kw):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **kw)


def _intent(edges):
    return {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
            'edge_connectors': [{'ref': r, 'edge': e} for r, e in edges]}


def _board_values(rows):
    return {r['ref'] for r in rows
            if r.get('ref') and 'board' in (r.get('values') or {})}


def main():
    evidence(PILE, 'tracked pile')
    evidence(PLACED, 'placed board')
    pile = _quiet(parse_kicad_pcb, PILE)
    check('the fixture is a pile', _quiet(assess_placement, pile, PILE).pile)
    refs = [r for r in ('USB1', 'CON1') if r in pile.footprints]
    check('the pile carries the declared connectors', len(refs) == 2, refs)
    doc = _intent([(refs[0], 'west'), (refs[1], 'east')])

    rows = _quiet(reconcile.reconcile, pile, PILE, intent_doc=doc,
                  intent_source='intent.json')
    check('no row reads an unlocked part\'s board value on the pile',
          not _board_values(rows), [r['id'] for r in rows])
    notes = [r for r in rows if r.get('field') == 'pile']
    check('one report row says what the pile withheld',
          len(notes) == 1 and notes[0]['kind'] == 'report'
          and set(refs) <= set(notes[0].get('withheld') or ()), notes)
    text = '\n'.join(reconcile.format_rows(rows))
    check('...and the printed rows say so, with no DRIFT',
          'pile' in text and 'DRIFT' not in text, text)

    pile.footprints[refs[0]].locked = True
    rows = _quiet(reconcile.reconcile, pile, PILE, intent_doc=doc,
                  intent_source='intent.json')
    check('a KiCad-locked part keeps its board value on the pile',
          _board_values(rows) == {refs[0]}, [r['id'] for r in rows])

    placed = _quiet(parse_kicad_pcb, PLACED)
    rows = _quiet(reconcile.reconcile, placed, PLACED,
                  intent_doc=_intent([(refs[0], 'north')]),
                  intent_source='intent.json')
    check('on a placed board the board value is still read',
          _board_values(rows) == {refs[0]}
          and not any(r.get('field') == 'pile' for r in rows),
          [r['id'] for r in rows])

    tmp = tempfile.mkdtemp(prefix='t1533_')
    try:
        board = os.path.join(tmp, 'pile.kicad_pcb')
        run_check([sys.executable, tool('copy_board.py'), PILE, board],
                  accept=True)
        ipath = os.path.join(tmp, 'intent.json')
        with open(ipath, 'w') as fh:
            json.dump(doc, fh)
        r = run_check([sys.executable, tool('check_floorplan.py'), board,
                       '--intent', ipath, '--allow-unplaced', '--plan-only'],
                      accept=True)
        out = r.stdout + r.stderr
        check('--plan-only on the pile prints no DRIFT row',
              'DRIFT' not in out, out[-600:])
        brief = os.path.join(tmp, 'brief.json')
        with open(brief, 'w') as fh:
            json.dump({'schema': 1, 'kind': 'design-brief', 'units': 'mm',
                       'interfaces': [
                           {'ref': refs[0], 'edge': 'west', 'user_facing': True,
                            'why': 'a test interface'},
                           {'ref': refs[1], 'edge': 'east',
                            'user_facing': 'unknown',
                            'why': 'a test interface'}]}, fh)
        emitted = os.path.join(tmp, 'emitted.json')
        r = run_check([sys.executable, tool('check_floorplan.py'), board,
                       '--brief', brief, '--emit-intent', emitted,
                       '--allow-unplaced'], accept=True)
        with open(evidence(emitted, 'emitted intent')) as fh:
            ctx = json.load(fh).get('context') or {}
        recon = ctx.get('reconciliation') or []
        inferred = [r['id'] for r in recon
                    for v in (r.get('values') or {}).values()
                    if v.get('authority') == 'inferred']
        check('--emit-intent writes no inferred value into '
              'context.reconciliation', not inferred, inferred)
        check('...and records the pile notice there',
              any(r.get('field') == 'pile' for r in recon), recon)
        out = r.stdout + r.stderr
        check('...and prints no DRIFT row', 'DRIFT' not in out, out[-600:])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())

"""#1532: an `overhang_mm` band below zero is refused at load.

The band is how far a part reaches PAST its edge; a body inside the board
reads 0 (`connector_geometry`: `overhang = max(0, signed)`). A negative bound
-- a socket set back from the edge, written as `{min: -0.5, max: -0.1}` --
loaded clean, `--plan-only` said nothing, the grade then failed it on every
edge (the reading is never below 0) and the edge seat found no room. A
setback is `max_setback_mm`, which the refusal names.

Pinned here:
  1. the intent loader refuses a negative `min` or `max`, and a non-number,
     naming `max_setback_mm`; a band at or above zero still loads;
  2. the design brief refuses the same band on an interface;
  3. end to end: `check_floorplan --plan-only` refuses it (exit 2) with the
     reason, rather than accepting it silently.

    python3 tests/test_1532_negative_overhang_band.py
"""
import json
import os
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from placement import design_brief, floorplan                 # noqa: E402
from run_utils import check as run_check, evidence, tool      # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
FAILS = []


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not ok else ''))
    if not ok:
        FAILS.append(name)


def _doc(band):
    return {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
            'edge_connectors': [{'ref': 'CON1', 'edge': 'east',
                                 'overhang_mm': band}]}


def _refusal(fn, *a):
    try:
        fn(*a)
    except floorplan.IntentError as exc:
        return str(exc)
    return None


def main():
    for band in ({'min': -0.5, 'max': -0.1}, {'min': 0.0, 'max': -0.1},
                 {'min': -1.0}, {'max': -2}):
        why = _refusal(floorplan.intent_from_dict, _doc(band), '')
        check(f'the intent loader refuses {band}',
              why is not None and 'max_setback_mm' in why
              and 'overhang_mm' in why, why)
    why = _refusal(floorplan.intent_from_dict, _doc({'min': 'flush'}), '')
    check('...and a band that is not a number',
          why is not None and 'expected a number' in why, why)
    for band in ({'min': 0.0, 'max': 0.5}, {'min': 1.5}, {'max': 0},
                 {'min': 0, 'max': None}):
        why = _refusal(floorplan.intent_from_dict, _doc(band), '')
        check(f'a band at or above zero still loads: {band}', why is None,
              why)

    tmp = tempfile.mkdtemp(prefix='t1532_')
    try:
        brief = os.path.join(tmp, 'b.design-brief.json')
        with open(brief, 'w') as fh:
            json.dump({'schema': 1, 'kind': 'design-brief', 'units': 'mm',
                       'interfaces': [{'ref': 'CON1', 'edge': 'east',
                                       'overhang_mm': {'min': -2, 'max': -1},
                                       'why': 'a socket set back'}]}, fh)
        why = _refusal(design_brief.load_brief, brief)
        check('the design brief refuses the same band',
              why is not None and 'max_setback_mm' in why, why)

        evidence(BOARD, 'tracked board')
        board = os.path.join(tmp, 'board.kicad_pcb')
        run_check([sys.executable, tool('copy_board.py'), BOARD, board],
                  accept=True)
        ipath = os.path.join(tmp, 'intent.json')
        with open(ipath, 'w') as fh:
            json.dump(_doc({'min': -0.5, 'max': -0.1}), fh)
        run_check([sys.executable, tool('check_floorplan.py'), board,
                   '--intent', ipath, '--plan-only'],
                  refuse='max_setback_mm', code=2)
        check('check_floorplan --plan-only refuses the band, naming '
              'max_setback_mm', True)
    except AssertionError as exc:
        check('check_floorplan --plan-only refuses the band, naming '
              'max_setback_mm', False, str(exc)[-600:])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())

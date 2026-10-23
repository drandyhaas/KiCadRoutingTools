#!/usr/bin/env python3
"""#1357: an intent row's free text is `why`, and `note` / `reason` are
aliases accepted on every row type.

The spelling used to depend on the row: fixed_poses and assembly took
`why`; proximity, blocks, edge_connectors, keepouts and keep_away `note`;
arrays and replicate both; overlap_waivers `reason`. The #710 unknown-key
gate refused the others, so gp14's interf_u intent exited 2 on
`proximity[0]: unknown key(s) why`, and the mirror file on
`fixed_poses[0]: unknown key(s) note`.

Asserts:
  1. every row type loads with each of the three spellings;
  2. a truly unknown key is still refused, and a non-string free text too;
  3. readers take the text whatever its spelling: a block's `note` field,
     and fixed_poses' accept_courtyard_overlap, which needs a reason;
  4. free_text joins the spellings a row carries;
  5. check_floorplan, end to end, grades an intent with `why` on a
     proximity row and `note` on a fixed_poses row (it exited 2 before).

    python3 tests/test_1357_intent_free_text_aliases.py
"""
import copy
import json
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, TESTS_DIR)

import placement.floorplan as fp                                   # noqa: E402
from placement.floorplan import (IntentError, intent_from_dict,    # noqa: E402
                                 free_text, KIND, SCHEMA_VERSION,
                                 READER_VERSION)
from run_utils import check, evidence                              # noqa: E402

failures = []

#: One minimal valid row per row type (list rows) or object (assembly).
ROWS = {
    'blocks': {'name': 'b', 'refs': ['U1']},
    'keepouts': {'name': 'k', 'rect': [0, 0, 1, 1], 'sides': ['F']},
    'edge_connectors': {'ref': 'J1', 'edge': 'east'},
    'proximity': {'ref': 'Y1', 'near': 'U1', 'max_mm': 2.0},
    'keep_away': {'rule': 'SW:FB:0.5'},
    'replicate': {'from': 'CH1', 'to': ['CH2']},
    'arrays': {'name': 'bank', 'members': ['R1', 'R2']},
    'fixed_poses': {'ref': 'H1', 'x': 1.0, 'y': 2.0, 'basis': 'declared'},
    'overlap_waivers': {'pair': ['U1', 'U2']},
    'assembly': {'sides': 'F'},
}


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def intent_with(kind, row):
    raw = {'schema': SCHEMA_VERSION, 'kind': KIND, 'units': 'mm',
           'min_reader': READER_VERSION}
    raw[kind] = row if kind == 'assembly' else [row]
    return raw


def loads(raw):
    try:
        return intent_from_dict(raw), None
    except IntentError as e:
        return None, str(e)


def main():
    print("1: every row type loads with each spelling")
    ok("the alias set is why + note + reason",
       fp._FREE_TEXT_KEYS == {'why', 'note', 'reason'} and fp.FREE_TEXT_KEY == 'why')
    for kind, base in ROWS.items():
        for key in ('why', 'note', 'reason'):
            row = dict(base, **{key: 'the reason'})
            _i, err = loads(intent_with(kind, row))
            ok(f"{kind} with `{key}`", err is None, err or '')

    print("2: what is still refused")
    for kind, base in ROWS.items():
        _i, err = loads(intent_with(kind, dict(base, comment='x')))
        ok(f"{kind}: an unknown key (`comment`)",
           err is not None and 'unknown key(s) comment' in err, err or 'loaded')
    for kind in ('proximity', 'fixed_poses', 'arrays'):
        _i, err = loads(intent_with(kind, dict(ROWS[kind], note=3)))
        ok(f"{kind}: a non-string `note`",
           err is not None and 'expected a string' in err, err or 'loaded')

    print("3: readers take the text whatever its spelling")
    i, err = loads(intent_with('blocks', dict(ROWS['blocks'], why='power')))
    ok("a block's `why` reaches Block.note",
       i is not None and i.blocks[0].note == 'power', err or repr(i.blocks[0].note))
    waiver = dict(ROWS['fixed_poses'], accept_courtyard_overlap=['H2'])
    for key in ('why', 'note', 'reason'):
        _i, err = loads(intent_with('fixed_poses',
                                    dict(waiver, **{key: 'a stacked pair'})))
        ok(f"accept_courtyard_overlap with its reason in `{key}`",
           err is None, err or '')
    _i, err = loads(intent_with('fixed_poses', copy.deepcopy(waiver)))
    ok("accept_courtyard_overlap with no reason is still refused",
       err is not None and 'needs a `why`' in err, err or 'loaded')

    print("4: free_text")
    ok("one spelling", free_text({'note': 'n'}) == 'n')
    ok("canonical order, joined", free_text({'reason': 'r', 'why': 'w'}) == 'w; r')
    ok("none", free_text({'ref': 'U1'}) == '' and free_text(None) == '')

    print("5: check_floorplan end to end")
    board = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
    from kicad_parser import parse_kicad_pcb
    refs = sorted(parse_kicad_pcb(board).footprints)
    tmp = tempfile.mkdtemp(prefix='t1357_')
    try:
        fx = parse_kicad_pcb(board).footprints[refs[0]]
        raw = {'schema': SCHEMA_VERSION, 'kind': KIND, 'units': 'mm',
               'min_reader': READER_VERSION,
               'proximity': [{'ref': refs[0], 'near': refs[1],
                              'max_mm': 50.0, 'why': 'kept together'}],
               'fixed_poses': [{'ref': refs[0], 'x': fx.x, 'y': fx.y,
                                'basis': 'declared',
                                'note': 'where it already is'}]}
        path = os.path.join(tmp, 'intent.json')
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(raw, f)
        evidence(path, 'the intent')
        r = check([sys.executable, '-X', 'utf8',
                   os.path.join(ROOT, 'py_tools', 'check_floorplan.py'),
                   board, '--intent', path, '--exit-zero'], accept=True)
        log = r.stdout + r.stderr
        ok("graded, not refused", 'unknown key' not in log
           and 'JSON_SUMMARY:' in log, log[-200:])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if failures:
        print(f"\nFAIL ({len(failures)}): " + '; '.join(failures))
        return 1
    print("\nAll checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())

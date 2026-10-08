#!/usr/bin/env python3
"""#1127: the stack-gate A/B's pre-registration cannot change without a test edit.

`tests/1127_stack_ab_prereg.json` fixes, before any census or A/B number
existed, how the stack gate's two changes are measured and what would make
either a default:
- the arms: box, licence and exact;
- the boards and piles, with StickHub diagnostic only;
- the census, its controls and its trial-cell rule;
- family A (OFF -> licence, judged as a constraint) and family B (licence ->
  exact, judged as an objective term), each with its signals and guards;
- the SAT prover's ship rule, the ship table and the stop rule.

A pre-registration that the next edit can rewrite is not one. So this test:
- pins the file's sha256, with newlines normalised so a CRLF checkout
  agrees;
- checks that the boards are real corpus files;
- checks that every signal and guard is a key the A/B harness records;
- checks, once `legality.STACK_MODE` exists, that every arm value is one it
  accepts.

    python3 tests/test_1127_prereg.py
"""
import hashlib
import json
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, TESTS_DIR)

PREREG = os.path.join(TESTS_DIR, '1127_stack_ab_prereg.json')

#: sha256 of the pre-registration with CRLF folded to LF, committed with the
#: file before the census or any A/B number. Re-pinned once, for the
#: `amendments` entry (control 1's wording, found by running the controls
#: before any cell): the first pin was 2aa59db9615a.
PREREG_SHA256 = ('d3329fd8c97319b4170456bdc8abf7c16d8bc47abd5f53c17df24c27cacdf933')


def _bytes():
    with open(PREREG, 'rb') as fh:
        return fh.read().replace(b'\r\n', b'\n')


def _doc():
    return json.loads(_bytes().decode('utf-8'))


def test_the_preregistration_is_pinned():
    got = hashlib.sha256(_bytes()).hexdigest()
    assert got == PREREG_SHA256, (
        f"{os.path.basename(PREREG)} changed (sha256 {got}). A "
        f"pre-registration is fixed before its numbers exist; if this edit is "
        f"deliberate, say why in the commit and update PREREG_SHA256 with it")
    print(f"  PASS: pre-registration pinned ({got[:12]})")


def test_the_preregistration_names_real_things():
    import test_placement_ab as AB
    doc = _doc()
    for key in ('arms', 'boards', 'piles', 'diagnostic', 'engines', 'census',
                'family_A', 'family_B', 'sat_prover', 'ship', 'stop_rule'):
        assert key in doc, key
    for b in doc['boards']:
        assert os.path.isfile(os.path.join(ROOT, 'kicad_files', b)), b
    assert len(set(doc['boards'])) >= 3, doc['boards']
    recorded = set(AB.BASELINE_KEYS)
    for fam in ('family_A', 'family_B'):
        f = doc[fam]
        sigs = (f['signals'].values() if 'signals' in f else [f['signal']])
        for s in sigs:
            assert s in recorded, (fam, s)
        for eng, guards in f['guards'].items():
            assert eng in doc['engines'], (fam, eng)
            for g in guards:
                assert g in recorded, (fam, eng, g)
    arms = {k: v for k, v in doc['arms'].items()
            if isinstance(v, dict) and 'legality.STACK_MODE' in v}
    assert sorted(a['legality.STACK_MODE'] for a in arms.values()) == [
        'box', 'exact', 'licence'], arms
    from placement import legality
    modes = getattr(legality, 'STACK_MODES', None)
    if modes is not None:
        for name, a in arms.items():
            assert a['legality.STACK_MODE'] in modes, (name, modes)
        where = "and legality.STACK_MODES accepts each"
    else:
        where = "(legality.STACK_MODE not yet implemented)"
    print(f"  PASS: {len(doc['boards'])} boards, families A and B read "
          f"recorded keys, arms {sorted(arms)} {where}")


TESTS = [
    test_the_preregistration_is_pinned,
    test_the_preregistration_names_real_things,
]


if __name__ == '__main__':
    want = sys.argv[1:]
    for t in TESTS:
        if want and not any(w in t.__name__ for w in want):
            continue
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")

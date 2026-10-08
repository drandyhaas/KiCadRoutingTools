#!/usr/bin/env python3
"""#1105: the pile A/B's pre-registration cannot change without a test edit.

`tests/1105_pile_ab_prereg.json` fixes, before any pile number existed, how
stage 3.5's decap claim is measured on the issue's own basis and what would
make it a default: the boards, the pile stager, the intent's argv, the one
pre-registered ladder candidate, the arbiter's guards and the guards it never
reads (with their tolerances), the eligibility rule, and the stop rule.

A pre-registration that its own regenerator, or the next edit, can rewrite is
not one. So:

* the file's sha256 (newlines normalised, so a CRLF checkout agrees) is
  pinned here -- changing a threshold means changing this constant in the
  same commit, where a reviewer sees it;
* the candidate names real seeder knobs, and the boards are real corpus
  files;
* the pile rows in `tests/test_placement_ab.py` take their boards, candidate,
  guards and tolerances FROM this file, never from a copy of it.

    python3 tests/test_1105_pile_prereg.py
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

PREREG = os.path.join(TESTS_DIR, '1105_pile_ab_prereg.json')

#: sha256 of the pre-registration with CRLF folded to LF, committed with the
#: file before any pile measurement. The file's `base` names the commits the
#: branch's first build sat on (PR #1140's head, since closed); the branch was
#: rebuilt on upstream main without #1140's routing commits, and the record is
#: kept as written rather than edited after the fact.
PREREG_SHA256 = ('c885671e73c0145ac9c7a9ebd4a25be5a6df1fed06ea36542dbef6ceb06e3dce')


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
    from placement import seeder
    doc = _doc()
    for key in ('boards', 'input', 'intent', 'seed', 'signal',
                'arbiter_guards', 'unread_guards', 'eligibility', 'candidate',
                'ladder', 'pilot', 'decisive', 'stop_rule'):
        assert key in doc, key
    for b in doc['boards']:
        assert os.path.isfile(os.path.join(ROOT, 'kicad_files', b)), b
    assert len(set(doc['boards'])) >= 3, doc['boards']
    cand = doc['candidate']
    for k, v in cand.items():
        if k.isupper():
            assert hasattr(seeder, k), f"seeder has no knob {k}"
            assert isinstance(v, type(getattr(seeder, k))), (k, v)
    assert cand['DECAP_LATE_AT'] in ('after_last_owner', 'after_queue'), cand
    unread = {k: v for k, v in doc['unread_guards'].items() if k != 'why'}
    assert not set(unread) & set(doc['arbiter_guards']), (
        "a guard the arbiter reads cannot also be an unread guard")
    for k, tol in unread.items():
        assert set(tol) <= {'rel', 'abs'} and len(tol) == 1, (k, tol)
    print(f"  PASS: {len(doc['boards'])} boards, candidate "
          f"{cand['DECAP_LATE_AT']}+within_limit, {len(unread)} unread "
          f"guard(s)")


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

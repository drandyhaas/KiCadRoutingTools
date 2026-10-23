#!/usr/bin/env python3
"""`check_assembly --baseline` reads no overlap off a PILE (#1103).

The baseline's body grade exists to tag each advisory pair of the graded
board as NEW or inherited. A pile's overlaps are staging, not decisions --
the parts were dropped where they lie -- so none of them can excuse a pair,
and grading them cost 96% of the check on a 272-part pile (glasgow_revC:
8.9 s of 9.3). On a pile baseline every advisory pair is new
(`baseline_pile: true` in the JSON); on a placed baseline the pairs are
compared as before, and a board graded against itself has none new.

    python3 tests/test_check_assembly_pile_baseline.py
"""
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'tests'))
from run_utils import evidence  # noqa: E402

PILE = os.path.join(ROOT, 'tests', 'fixtures', '959', 'run29_pile.kicad_pcb')
PLACED = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
FAILS = []


def check(ok, what):
    print(f"  {'PASS' if ok else 'FAIL'}  {what}")
    if not ok:
        FAILS.append(what)


def grade(board, baseline, out):
    r = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_tools', 'check_assembly.py'),
                        board, '--baseline', baseline, '--json', out],
                       capture_output=True, text=True, cwd=ROOT)
    if not os.path.isfile(out):
        raise SystemExit(f"check_assembly wrote no JSON (exit {r.returncode}):\n"
                         f"{r.stdout[-800:]}{r.stderr[-800:]}")
    return json.load(open(out, encoding='utf-8')), r.stdout


def main():
    evidence(PILE)
    evidence(PLACED)
    with tempfile.TemporaryDirectory() as tmp:
        doc, out = grade(PILE, PILE, os.path.join(tmp, 'pile.json'))
        adv = doc.get('advisory_pairs') or []
        new = doc.get('new_advisory_pairs')
        check(doc.get('baseline_pile') is True, "a pile baseline is named a pile")
        check(new is not None and len(new) == len(adv),
              f"against a pile every advisory pair is new "
              f"({len(new or [])} new of {len(adv)})")
        check(len(adv) > 0, f"the pile has advisory pairs to tag ({len(adv)})")
        check('is a pile' in out, "the console says the baseline is a pile")
        doc, out = grade(PLACED, PLACED, os.path.join(tmp, 'placed.json'))
        check(doc.get('baseline_pile') is False, "a placed baseline is not a pile")
        check(doc.get('new_advisory_pairs') == [],
              "a placed board against itself has no new pair")
    if FAILS:
        print(f"FAIL: {len(FAILS)} check(s)")
        return 1
    print("PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""#1665: check_assembly without --intent says when an unread waiver could matter.

An intent's `overlap_waivers` are read only with --intent, so a pair a placement
run waived read NOT BUILDABLE on the two commands the skills gave, where
place_simple and the verifier read buildable, and the output did not say a
waiver might be unread. Both skills now pass the intent, and the tool notes it
beside the verdict, only when a gating pair is one a waiver reaches.

Pinned here, on a part locked over another (the skill's own "a part drawn over
another"):
  1. no --intent: NOT BUILDABLE, exit 4, and the note and `waiver_candidates`
     name that pair; the verdict line itself is unchanged;
  2. --intent with the pair waived: buildable, no note, `waiver_candidates` null;
  3. a NOT BUILDABLE no waiver reaches (pad copper off the outline): no note;
  4. a buildable board: no note;
  5. both causes at once: the note, saying the rest gate either way;
  6. the skills' commands pass the intent.

    python3 tests/test_1665_assembly_says_waivers_unread.py
"""
import json
import os
import re
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

from run_utils import check, evidence, tool                    # noqa: E402

PY = [sys.executable, '-B', '-X', 'utf8']
ASSEMBLY = tool('check_assembly.py')
POSE = tool('place_pose.py')
BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
NOTE = 'NOTE: no --intent, so no waiver was read'
EITHER_WAY = 'take no waiver, and gate either way'
OVER = ['set', 'R1', '127', '100', 'lock', 'R1', 'U1']
OFF = ['set', 'C1', '300', '300']


def posed(d, name, *moves):
    out = os.path.join(d, name)
    check(PY + [POSE, BOARD, out, *moves, '--force', '--allow-routed'],
          accept=True, allow=('WARNING',))
    return evidence(out, 'the posed board')


def grade(board, d, *flags, buildable):
    js = os.path.join(d, 'assembly.json')
    if os.path.exists(js):
        os.remove(js)
    r = (check(PY + [ASSEMBLY, board, '--json', js, *flags], accept=True, allow=('WARNING',))
         if buildable else
         check(PY + [ASSEMBLY, board, '--json', js, *flags], refuse='VERDICT: NOT BUILDABLE', code=4,
               allow=('WARNING',)))
    return r.stdout + r.stderr, json.load(open(evidence(js), encoding='utf-8'))


def test_commands():
    place = open(os.path.join(ROOT, '.claude/skills/plan-pcb-placement/SKILL.md'),
                 encoding='utf-8').read()
    route = open(os.path.join(ROOT, '.claude/skills/plan-pcb-routing/SKILL.md'),
                 encoding='utf-8').read()
    assert re.search(r'check_assembly\.py <placed>[^`]*--intent wk/<run>/intent\.json', place), \
        'plan-pcb-placement 3.6 must pass the run\'s intent'
    step0 = route[route.index('## Step 0'):route.index('## Step 1')]
    assert re.search(r'When placement left a floorplan intent, add\s+`--intent', step0), \
        'plan-pcb-routing Step 0 must add --intent when placement left one'
    print('  ok: both skills\' commands pass the intent')


def main():
    test_commands()
    d = tempfile.mkdtemp(prefix='t1665_')
    try:
        over = posed(d, 'over.kicad_pcb', *OVER)
        text, doc = grade(over, d, buildable=False)
        assert NOTE in text and 'R1<->U1' in text, text[-600:]
        assert EITHER_WAY not in text, text[-600:]
        assert {tuple(r['pair']) for r in doc['waiver_candidates']} == {('R1', 'U1')}, doc['waiver_candidates']
        print('  ok: no --intent: NOT BUILDABLE, and the note names the pair')

        intent = os.path.join(d, 'intent.json')
        check(PY + [tool('check_floorplan.py'), over, '--emit-intent', intent, '--quiet'],
              accept=True, allow=('WARNING',))
        idoc = json.load(open(evidence(intent), encoding='utf-8'))
        idoc['overlap_waivers'] = [{'pair': ['R1', 'U1'], 'why': 'a part drawn over another'}]
        json.dump(idoc, open(intent, 'w', encoding='utf-8'))
        text, doc = grade(over, d, '--intent', intent, buildable=True)
        assert NOTE not in text and doc['waiver_candidates'] is None, text[-600:]
        print('  ok: with the intent: buildable, no note')

        off = posed(d, 'off.kicad_pcb', *OFF)
        text, doc = grade(off, d, buildable=False)
        assert doc['oob_pad_copper_gating_refs'] == ['C1'], doc['oob_pad_copper_gating_refs']
        assert NOTE not in text and doc['waiver_candidates'] == [], text[-600:]
        print('  ok: NOT BUILDABLE on what no waiver reaches: no note')

        text, doc = grade(BOARD, d, buildable=True)
        assert NOTE not in text and doc['waiver_candidates'] == [], text[-600:]
        print('  ok: a buildable board: no note')

        both = posed(d, 'both.kicad_pcb', *OVER, *OFF)
        text, doc = grade(both, d, buildable=False)
        assert NOTE in text and EITHER_WAY in text, text[-600:]
        print('  ok: both causes: the note, and the rest gate either way')
    finally:
        shutil.rmtree(d, ignore_errors=True)
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())

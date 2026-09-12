#!/usr/bin/env python3
"""The routing skill names ONE final board, in every place that names one.

#941 row 4 moved Step 3's chain onto a closing `route.py`, which renamed the
board that ships from `board_step4` to `board_step4b`. Step 6 was updated and
the user-facing cleanup prompt, ~1050 lines away, was not -- so for one commit
the file mandated `board_step4b` "never board_step4" in one section and handed
the user `board_step4` as the final board in another, while listing
`board_step4b` nowhere at all. A plan following the cleanup prompt would have
deleted the only finalized board.

That is the #941 defect exactly: two sentences about one fact, and the reader
acts on whichever they reach first. A prose fix does not survive the next edit,
so this is the invariant instead.

The rule, stated once: every board name the file offers as FINAL must be a
board some command in the file actually WRITES, and the Step 6 sentence and the
cleanup prompt must offer the same set.

Run: python3 -X utf8 tests/test_941_final_board_is_one_name.py
"""
import io
import os
import re
import sys

RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 120

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKILL = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-routing', 'SKILL.md')

#: `board_<something>.kicad_pcb` only -- the worked example's own vocabulary.
BOARD = re.compile(r'\bboard_[a-z0-9_]+\.kicad_pcb\b')

#: The two sentences that tell a reader which board is the deliverable.
FINAL_SENTENCE = re.compile(r'^The final board is (.+)$', re.M)
CLEANUP_SENTENCE = re.compile(r'^> The final routed board is: (.+)$', re.M)

#: A board is WRITTEN if it is the second positional of a tool invocation, or
#: follows `--output`. Both spellings appear in the file.
WRITES = re.compile(
    r'py_(?:router|placer|tools)/[a-z_]+\.py\s+\S*?board_[a-z0-9_]+\.kicad_pcb'
    r'\s+(board_[a-z0-9_]+\.kicad_pcb)'
    r'|--output\s+(board_[a-z0-9_]+\.kicad_pcb)')

passed = failed = 0


def check(name, ok, detail=''):
    global passed, failed
    if ok:
        passed += 1
        print(f'  OK   {name}' + (f' -- {detail}' if detail else ''))
    else:
        failed += 1
        print(f'  FAIL {name}' + (f' -- {detail}' if detail else ''))


def _text():
    return io.open(SKILL, encoding='utf-8').read()


def _written(text):
    out = set()
    for a, b in WRITES.findall(text):
        out.add(a or b)
    return out


def test_both_final_sentences_name_the_same_boards():
    text = _text()
    step6 = FINAL_SENTENCE.findall(text)
    cleanup = CLEANUP_SENTENCE.findall(text)
    # Anti-vacuity: a regex that stopped matching would otherwise pass.
    check('both final-board sentences are still present',
          len(step6) == 1 and len(cleanup) == 1,
          f'Step 6 matches={len(step6)}, cleanup matches={len(cleanup)}')
    if len(step6) != 1 or len(cleanup) != 1:
        return
    a, b = set(BOARD.findall(step6[0])), set(BOARD.findall(cleanup[0]))
    check('anti-vacuity: each sentence names at least one board',
          bool(a) and bool(b), f'step6={sorted(a)} cleanup={sorted(b)}')
    check('Step 6 and the cleanup prompt name the SAME final boards',
          a == b,
          f'Step 6 says {sorted(a)}, the cleanup prompt says {sorted(b)}; '
          f'only in Step 6: {sorted(a - b)}; only in the prompt: '
          f'{sorted(b - a)}')


def test_every_board_offered_as_final_is_one_a_command_writes():
    text = _text()
    written = _written(text)
    check('anti-vacuity: the writer scan found commands',
          len(written) >= 3,
          f'{len(written)} written board(s): {sorted(written)}')
    offered = set()
    for m in list(FINAL_SENTENCE.findall(text)) + \
            list(CLEANUP_SENTENCE.findall(text)):
        offered |= set(BOARD.findall(m))
    check('every board offered as final is written by some command',
          offered <= written,
          f'offered but never written: {sorted(offered - written)}')


TESTS = (test_both_final_sentences_name_the_same_boards,
         test_every_board_offered_as_final_is_one_a_command_writes)


def _every_case_is_registered():
    defined = {n for n in globals() if n.startswith('test_')}
    listed = {f.__name__ for f in TESTS}
    assert defined == listed, f'not registered: {sorted(defined - listed)}'


if __name__ == '__main__':
    _every_case_is_registered()
    for fn in TESTS:
        print(f'--- {fn.__name__}')
        fn()
    print(f'\n{passed} passed, {failed} failed')
    sys.exit(1 if failed else 0)
